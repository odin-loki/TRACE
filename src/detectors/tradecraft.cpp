#include "trace/detectors/detectors.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <limits>

namespace trace {
namespace {

constexpr std::size_t kMaxVisits = 128;

/// How many scans an entity must be recorded in one cell before it counts as
/// having stopped there rather than passed through.
constexpr int kMinDwellVisits = 3;

/// Ordered key so a pair maps to one entry regardless of iteration order.
std::pair<std::string, std::string> pair_key(const std::string& a,
                                             const std::string& b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

}  // namespace

std::vector<DetectionEvent> TradecraftDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;

    for (const auto& t : tracks) {
        auto& v = visits_[t->id()];
        v.push_back(Visit{ctx.timestamp, t->position()});
        if (v.size() > kMaxVisits) v.pop_front();
    }

    // ---- Brush pass: two entities converge to within contact range ---------
    // Only the transition into contact is reported. Without the streak counter
    // two people walking together would raise an event every single scan.
    // A brush pass is by definition a close approach, so only near pairs can
    // produce one. Pairs that have separated are cleared lazily below.
    for (const auto& [i, j] : ctx.near_pairs(p.brush_pass_m * 2.0, tracks.size())) {
        {
            const Track& a = *tracks[i];
            const Track& b = *tracks[j];
            const Real sep = distance(a.position(), b.position());
            const auto key = pair_key(a.id(), b.id());

            if (sep < p.brush_pass_m) {
                const int streak = ++contact_streak_[key];
                if (streak == 1) {
                    auto e = make_event("BRUSH_PASS", name(), {a.id(), b.id()},
                                        Severity::HIGH, ctx.timestamp);
                    e.location = (a.position() + b.position()) * 0.5;
                    e.metrics.push_back({"separation_m", sep});
                    e.note = "entities converged to contact range";
                    events.push_back(std::move(e));
                }
            } else {
                contact_streak_[key] = 0;
            }
        }
    }

    // ---- Surveillance-detection route: a closed loop ----------------------
    // Winding number is the accumulated turn about the path's own centroid.
    // Someone circling a block to see who follows accumulates a full turn;
    // someone walking to work does not.
    for (const auto& t : tracks) {
        const auto& v = visits_[t->id()];
        if (v.size() < 8) continue;

        const std::size_t take =
            std::min(v.size(), static_cast<std::size_t>(std::max(p.sdr_window, 4)));
        Vec2 centroid{};
        for (std::size_t i = v.size() - take; i < v.size(); ++i) {
            centroid += v[i].position;
        }
        centroid = centroid / static_cast<Real>(take);

        Real total_turn = 0.0;
        Real prev_angle = 0.0;
        bool have_prev = false;
        for (std::size_t i = v.size() - take; i < v.size(); ++i) {
            const Vec2 d = v[i].position - centroid;
            if (d.norm() < 1e-6) continue;
            const Real angle = std::atan2(d.y, d.x);
            if (have_prev) {
                // Unwrap so a crossing of +/-pi does not read as a huge jump.
                Real delta = angle - prev_angle;
                while (delta > std::numbers::pi) delta -= 2.0 * std::numbers::pi;
                while (delta < -std::numbers::pi) delta += 2.0 * std::numbers::pi;
                total_turn += delta;
            }
            prev_angle = angle;
            have_prev = true;
        }

        const Real winding = std::abs(total_turn) / (2.0 * std::numbers::pi);
        if (winding >= p.sdr_turns) {
            auto e = make_event("SDR_PATTERN", name(), {t->id()}, Severity::HIGH,
                                ctx.timestamp);
            e.location = t->position();
            e.metrics.push_back({"winding_turns", winding});
            e.note = "closed-loop route consistent with surveillance detection";
            events.push_back(std::move(e));
        }
    }

    // ---- Dead drop: same place, different entities, never at the same time -
    if (tracks.size() >= 2 && ctx.scan_index % 3 == 0) {
        struct CellEntry {
            std::string tid;
            Real timestamp;
        };
        // Keyed by (grid offset, cell x, cell y). Two offsets per axis, half a
        // cell apart, so that any two visits within half a cell of each other
        // share at least one bin. A single grid puts a hard boundary through
        // arbitrary ground, and a drop that happens to sit on one is split
        // between cells and invisible - which is not a rare case, because the
        // places people use are exactly the sort of round coordinates a grid
        // lands its boundaries on.
        std::map<std::tuple<int, long, long>, std::vector<CellEntry>> cell_visits;

        // Small enough that crossing it takes less than a dwell. The dwell test
        // above only discriminates if walking through a cell yields fewer
        // records than stopping in one does, and at five times the chokepoint
        // radius it did not: a pedestrian crossed a 100 m cell in about three
        // scans, which is exactly what stopping looks like. One chokepoint
        // radius is the profile's own statement of "the same place".
        const Real cell_size = std::max(p.chokepoint_m, 1.0);
        // Look back over the window a dead drop is allowed to span, not over a
        // fixed number of visits. This took each track's last five, which at
        // any scan period longer than a few seconds is a shorter span than
        // `dead_drop_min_s` requires - so the detector could not fire at all
        // whenever the domain's idea of "hours apart" exceeded five scans,
        // which is every domain it was written for. The bound is now the
        // profile's own window, and `kMaxVisits` is what limits how far back
        // that can reach.
        for (const auto& t : tracks) {
            const auto& v = visits_[t->id()];
            for (auto it = v.rbegin(); it != v.rend(); ++it) {
                if (ctx.timestamp - it->timestamp > p.dead_drop_max_s) break;
                for (int ox = 0; ox < 2; ++ox) {
                    for (int oy = 0; oy < 2; ++oy) {
                        const Real sx = it->position.x + ox * cell_size * 0.5;
                        const Real sy = it->position.y + oy * cell_size * 0.5;
                        cell_visits[{ox * 2 + oy,
                                     static_cast<long>(std::floor(sx / cell_size)),
                                     static_cast<long>(std::floor(sy / cell_size))}]
                            .push_back(CellEntry{t->id(), it->timestamp});
                    }
                }
            }
        }
        std::vector<Vec2> already_reported;

        for (const auto& [cell, visitors] : cell_visits) {
            // A visitor is someone who *stopped*, not someone who walked past.
            // Without that distinction the finding degenerates to "two people
            // used this place at different times", which in any populated
            // scene is everybody: on the coordinated-evasion scenario it gave
            // 330 events of which 25 were at a real drop site. A dead drop is
            // defined by the pause - somebody has to put something down.
            std::map<std::string, int> dwell;
            for (const auto& v : visitors) ++dwell[v.tid];

            std::set<std::string> distinct;
            for (const auto& [tid, n] : dwell) {
                if (n >= kMinDwellVisits) distinct.insert(tid);
            }
            if (distinct.size() < 2) continue;

            // Timings over the entities that actually dwelt, not over every
            // passer-by whose track clipped the cell.
            Real tmin = std::numeric_limits<Real>::infinity();
            Real tmax = -std::numeric_limits<Real>::infinity();
            for (const auto& v : visitors) {
                if (distinct.count(v.tid) == 0) continue;
                tmin = std::min(tmin, v.timestamp);
                tmax = std::max(tmax, v.timestamp);
            }
            const Real spread = tmax - tmin;
            if (spread <= p.dead_drop_min_s || spread >= p.dead_drop_max_s) continue;

            // The defining feature is that they were never there together.
            // Overlapping visits are a meeting, which is a different finding.
            bool simultaneous = false;
            for (const auto& a : visitors) {
                if (distinct.count(a.tid) == 0) continue;
                for (const auto& b : visitors) {
                    if (distinct.count(b.tid) == 0) continue;
                    if (a.tid != b.tid &&
                        std::abs(a.timestamp - b.timestamp) < p.scan_dt_s * 0.5) {
                        simultaneous = true;
                    }
                }
            }
            if (simultaneous) continue;

            // The overlapping grids mean one place can match in up to four
            // bins. Report the place, not the bins.
            const Vec2 where{
                (static_cast<Real>(std::get<1>(cell)) + 0.5) * cell_size -
                    (std::get<0>(cell) / 2) * cell_size * 0.5,
                (static_cast<Real>(std::get<2>(cell)) + 0.5) * cell_size -
                    (std::get<0>(cell) % 2) * cell_size * 0.5};
            bool duplicate = false;
            for (const Vec2& seen : already_reported) {
                if (distance(seen, where) < cell_size) duplicate = true;
            }
            if (duplicate) continue;
            already_reported.push_back(where);

            std::vector<std::string> tids(distinct.begin(), distinct.end());
            if (tids.size() > 3) tids.resize(3);
            auto e = make_event("DEAD_DROP", name(), tids, Severity::CRITICAL,
                                ctx.timestamp);
            e.location = where;
            e.metrics.push_back({"time_spread_s", spread});
            e.metrics.push_back({"n_visitors", static_cast<Real>(distinct.size())});
            e.note = "same location used by multiple entities, never concurrently";
            events.push_back(std::move(e));
        }
    }

    return events;
}

}  // namespace trace
