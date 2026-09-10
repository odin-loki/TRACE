#include "trace/detectors/detectors.hpp"

#include <algorithm>
#include <cmath>

namespace trace {
namespace {

constexpr std::size_t kMaxVisits = 128;

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
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        for (std::size_t j = i + 1; j < tracks.size(); ++j) {
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
        std::map<std::pair<long, long>, std::vector<CellEntry>> cell_visits;

        const Real cell_size = std::max(p.chokepoint_m * 5.0, 1.0);
        for (const auto& t : tracks) {
            const auto& v = visits_[t->id()];
            const std::size_t take = std::min<std::size_t>(v.size(), 5);
            for (std::size_t i = v.size() - take; i < v.size(); ++i) {
                const auto cx = static_cast<long>(std::floor(v[i].position.x / cell_size));
                const auto cy = static_cast<long>(std::floor(v[i].position.y / cell_size));
                cell_visits[{cx, cy}].push_back(CellEntry{t->id(), v[i].timestamp});
            }
        }

        for (const auto& [cell, visitors] : cell_visits) {
            std::set<std::string> distinct;
            for (const auto& v : visitors) distinct.insert(v.tid);
            if (distinct.size() < 2) continue;

            Real tmin = visitors.front().timestamp;
            Real tmax = tmin;
            for (const auto& v : visitors) {
                tmin = std::min(tmin, v.timestamp);
                tmax = std::max(tmax, v.timestamp);
            }
            const Real spread = tmax - tmin;
            if (spread <= p.dead_drop_min_s || spread >= p.dead_drop_max_s) continue;

            // The defining feature is that they were never there together.
            // Overlapping visits are a meeting, which is a different finding.
            bool simultaneous = false;
            for (const auto& a : visitors) {
                for (const auto& b : visitors) {
                    if (a.tid != b.tid &&
                        std::abs(a.timestamp - b.timestamp) < p.scan_dt_s * 0.5) {
                        simultaneous = true;
                    }
                }
            }
            if (simultaneous) continue;

            std::vector<std::string> tids(distinct.begin(), distinct.end());
            if (tids.size() > 3) tids.resize(3);
            auto e = make_event("DEAD_DROP", name(), tids, Severity::CRITICAL,
                                ctx.timestamp);
            e.location = Vec2{(static_cast<Real>(cell.first) + 0.5) * cell_size,
                              (static_cast<Real>(cell.second) + 0.5) * cell_size};
            e.metrics.push_back({"time_spread_s", spread});
            e.metrics.push_back({"n_visitors", static_cast<Real>(distinct.size())});
            e.note = "same location used by multiple entities, never concurrently";
            events.push_back(std::move(e));
        }
    }

    return events;
}

}  // namespace trace
