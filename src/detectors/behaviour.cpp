// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/detectors/detectors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <optional>

namespace trace {
namespace {

/// How many distinct cells a track's chokepoint history remembers.
constexpr std::size_t kMaxCellVisits = 256;

std::pair<std::string, std::string> pair_key(const std::string& a,
                                             const std::string& b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

/// Is this track's regime one that implies it has stopped moving?
bool is_stationary_model(const std::string& m) {
    return m == "stationary" || m == "stopped" || m == "anchored" ||
           m == "standing" || m == "static" || m == "resting" ||
           m == "lying_up";
}

/// Is this regime a fast, carried mode rather than self-propelled?
bool is_transport_model(const std::string& m) {
    return m == "vehicle" || m == "highway" || m == "sprint" ||
           m == "fast_craft" || m == "forklift" || m == "conveyor" ||
           m == "transiting" || m == "fixed_wing" || m == "fast_jet";
}

/// Split a sample into a low and a high mode, Otsu-style: sweep every cut
/// point and keep the one maximising between-class variance. Returns the
/// smallest value in the high mode.
///
/// Returns nothing when the sample has no high mode worth the name - the best
/// available cut still leaves the two class means within `min_ratio` of each
/// other. That case matters: a quantile always yields a threshold, so against
/// a quantile the fastest member of a population of stationary objects is
/// "fast". Here it is not, which is the correct answer.
///
/// `sample` must be sorted ascending.
std::optional<Real> high_mode_threshold(const std::vector<Real>& sample,
                                        Real min_ratio) {
    const std::size_t n = sample.size();
    if (n < 4) return std::nullopt;
    const Real total = std::accumulate(sample.begin(), sample.end(), Real{0});

    Real best_var = 0.0;
    std::size_t best_k = 0;
    Real cum = 0.0;
    for (std::size_t k = 1; k < n; ++k) {
        cum += sample[k - 1];
        const Real w_lo = static_cast<Real>(k) / static_cast<Real>(n);
        const Real mean_lo = cum / static_cast<Real>(k);
        const Real mean_hi = (total - cum) / static_cast<Real>(n - k);
        const Real var = w_lo * (1.0 - w_lo) * (mean_hi - mean_lo) *
                         (mean_hi - mean_lo);
        if (var > best_var) {
            best_var = var;
            best_k = k;
        }
    }
    if (best_k == 0) return std::nullopt;

    const Real cut = std::accumulate(sample.begin(),
                                     sample.begin() + static_cast<long>(best_k),
                                     Real{0});
    const Real mean_lo = cut / static_cast<Real>(best_k);
    const Real mean_hi = (total - cut) / static_cast<Real>(n - best_k);
    if (mean_hi < mean_lo * min_ratio) return std::nullopt;
    return sample[best_k];
}

}  // namespace

// ---------------------------------------------------------------------------
// Parallel route — one entity shadowing another
// ---------------------------------------------------------------------------

std::vector<DetectionEvent> ParallelRouteDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;

    // A tail holds a fixed offset, so only pairs already within that offset
    // can be one.
    for (const auto& [i, j] : ctx.near_pairs(p.parallel_route_m * 1.5, tracks.size())) {
        {
            const Track& a = *tracks[i];
            const Track& b = *tracks[j];
            const auto key = pair_key(a.id(), b.id());

            const Vec2 va = a.smoothed_velocity();
            const Vec2 vb = b.smoothed_velocity();
            const Real na = va.norm();
            const Real nb = vb.norm();

            // Both must actually be moving; two stationary entities trivially
            // have "matching" headings and would false-positive forever.
            if (na < 1e-3 || nb < 1e-3) {
                streak_[key] = 0;
                continue;
            }

            const Real cos_sim = va.dot(vb) / (na * nb);
            const Real sep = distance(a.position(), b.position());

            const bool matching = cos_sim >= p.parallel_vel_cos &&
                                  sep <= p.parallel_route_m &&
                                  sep > p.brush_pass_m;  // together, not touching

            if (!matching) {
                streak_[key] = 0;
                continue;
            }

            const int streak = ++streak_[key];
            if (streak == p.parallel_scans) {
                auto e = make_event("PARALLEL_ROUTE", name(), {a.id(), b.id()},
                                    Severity::HIGH, ctx.timestamp);
                e.location = (a.position() + b.position()) * 0.5;
                e.metrics.push_back({"heading_cosine", cos_sim});
                e.metrics.push_back({"offset_m", sep});
                e.metrics.push_back({"scans_held", static_cast<Real>(streak)});
                e.note = "sustained matched heading at fixed offset";
                events.push_back(std::move(e));
            }
        }
    }
    return events;
}

// ---------------------------------------------------------------------------
// Mode transition — a handover between transport modes
// ---------------------------------------------------------------------------

std::vector<DetectionEvent> ModeTransitionDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;

    // Retire stops older than the detector's window.
    const Real cutoff = ctx.timestamp - p.mode_trans_scans * p.scan_dt_s;
    while (!recent_stops_.empty() && recent_stops_.front().timestamp < cutoff) {
        recent_stops_.pop_front();
    }

    for (const auto& t : tracks) {
        const std::string& model = t->dominant_model();

        // A newly-appeared track next to a recent stop is the signature: the
        // entity got out, and the tracker sees a fresh object rather than a
        // continuation.
        if (t->age() <= p.mode_trans_scans + 1) {
            for (const auto& stop : recent_stops_) {
                if (stop.track_id == t->id()) continue;
                const Real d = distance(t->position(), stop.position);
                if (d > p.mode_trans_m) continue;

                const std::string tag = stop.track_id + ">" + t->id();
                if (reported_.contains(tag)) continue;
                reported_.insert(tag);

                auto e = make_event("MODE_TRANSITION", name(),
                                    {stop.track_id, t->id()}, Severity::HIGH,
                                    ctx.timestamp);
                e.location = t->position();
                e.metrics.push_back({"handover_distance_m", d});
                e.metrics.push_back({"gap_s", ctx.timestamp - stop.timestamp});
                e.note = "entity appeared beside a halted " + stop.model +
                         " track: probable transport handover";
                events.push_back(std::move(e));
                break;
            }
        }

        if (is_stationary_model(model) || (is_transport_model(model) &&
                                           t->speed_mps(p.scan_dt_s) < 0.5)) {
            recent_stops_.push_back(Stop{t->id(), t->position(), ctx.timestamp,
                                         model});
        }
    }

    if (recent_stops_.size() > 200) {
        recent_stops_.erase(recent_stops_.begin(),
                            recent_stops_.begin() + 100);
    }
    return events;
}

// ---------------------------------------------------------------------------
// Loiter — dwelling beyond this entity's own norm
// ---------------------------------------------------------------------------

std::vector<DetectionEvent> LoiterDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;

    for (const auto& t : tracks) {
        // "Stopped" is relative to the domain: a drifting vessel covering 50 m
        // an hour is stationary, a pedestrian doing the same is not.
        const Real move_tol = std::max(p.brush_pass_m * 0.5, p.pos_noise_m * 2.0);
        auto& d = dwell_[t->id()];

        if (d.since == 0.0 || distance(t->position(), d.anchor) > move_tol) {
            d.anchor = t->position();
            d.since = ctx.timestamp;
            d.reported = false;
            continue;
        }

        const Real dwell_s = ctx.timestamp - d.since;
        if (dwell_s < p.loiter_min_s || d.reported) continue;

        // Threshold against the entity's own baseline where one exists. An
        // entity that habitually sits still for an hour should not alert at
        // the same dwell as one that never stops.
        Real threshold = p.loiter_min_s * p.loiter_mult;
        if (t->pol().fitted()) {
            const Real spread = t->pol().spatial_spread();
            const Real habitual = spread > move_tol ? 1.0 : p.loiter_mult;
            threshold = std::max(p.loiter_min_s, p.loiter_min_s * habitual);
        }
        if (dwell_s < threshold) continue;

        d.reported = true;
        auto e = make_event("LOITER", name(), {t->id()}, Severity::MEDIUM,
                            ctx.timestamp);
        e.location = t->position();
        e.metrics.push_back({"dwell_s", dwell_s});
        e.metrics.push_back({"threshold_s", threshold});
        if (t->pol().fitted()) {
            e.metrics.push_back(
                {"pol_anomaly", t->pol().anomaly_score(ctx.timestamp, t->position())});
        }
        e.note = "stationary well beyond this entity's normal dwell";
        events.push_back(std::move(e));
    }

    // Drop dwell state for tracks that are gone, so the map cannot grow without
    // bound over a long deployment.
    if (dwell_.size() > 512) {
        std::set<std::string> live;
        for (const auto& t : tracks) live.insert(t->id());
        std::erase_if(dwell_, [&](const auto& kv) { return !live.contains(kv.first); });
    }
    return events;
}

// ---------------------------------------------------------------------------
// Cover stop — a halt that the entity's routine does not explain
// ---------------------------------------------------------------------------

std::vector<DetectionEvent> CoverStopDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;
    if (ctx.rng == nullptr) return events;

    for (const auto& t : tracks) {
        if (!t->pol().fitted()) continue;
        if (t->speed_mps(p.scan_dt_s) > p.courier_speed_thresh * 0.2) continue;

        const Vec2 pos = t->position();
        const auto pred = t->pol().predict_location(ctx.timestamp, *ctx.rng);
        const Real off_pattern = distance(pos, pred.position);
        if (off_pattern < p.cover_stop_m) continue;  // stopped where it usually does

        // Extra weight if the unexplained stop is near something that matters.
        Real hvl_dist = std::numeric_limits<Real>::infinity();
        if (ctx.high_value_locations != nullptr) {
            for (const Vec2& h : *ctx.high_value_locations) {
                hvl_dist = std::min(hvl_dist, distance(pos, h));
            }
        }
        const bool near_hvl = hvl_dist < p.cover_stop_hvl_m;

        char tag_buf[128];
        std::snprintf(tag_buf, sizeof(tag_buf), "%s@%ld_%ld", t->id().c_str(),
                      static_cast<long>(pos.x / std::max(p.cover_stop_m, 1.0)),
                      static_cast<long>(pos.y / std::max(p.cover_stop_m, 1.0)));
        const std::string tag(tag_buf);
        if (reported_.contains(tag)) continue;
        reported_.insert(tag);

        auto e = make_event("COVER_STOP", name(), {t->id()},
                            near_hvl ? Severity::CRITICAL : Severity::HIGH,
                            ctx.timestamp);
        e.location = pos;
        e.metrics.push_back({"off_pattern_m", off_pattern});
        e.metrics.push_back({"pol_uncertainty_m", pred.uncertainty_m});
        if (std::isfinite(hvl_dist)) e.metrics.push_back({"hvl_distance_m", hvl_dist});
        e.note = near_hvl ? "unexplained halt close to a location of interest"
                          : "halt outside this entity's established pattern";
        events.push_back(std::move(e));
    }
    return events;
}

// ---------------------------------------------------------------------------
// Chokepoint — repeated passes through the same small cell
// ---------------------------------------------------------------------------

std::vector<DetectionEvent> ChokepointDetector::detect(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<DetectionEvent> events;
    const DomainProfile& p = *ctx.profile;
    const Real cell = std::max(p.chokepoint_m, 1.0);

    for (const auto& t : tracks) {
        const Vec2 pos = t->position();
        const auto cx = static_cast<long>(std::floor(pos.x / cell));
        const auto cy = static_cast<long>(std::floor(pos.y / cell));

        auto& visits = cells_[t->id()];
        auto it = std::find_if(visits.begin(), visits.end(),
                               [&](const CellVisit& v) {
                                   return v.cx == cx && v.cy == cy;
                               });

        // Every remembered cell the track is now genuinely away from - more
        // than one cell in either axis, so crossing a boundary and coming
        // straight back is not a departure - becomes eligible to be counted
        // again when the track returns to it.
        for (CellVisit& v : visits) {
            if (std::labs(v.cx - cx) > 1 || std::labs(v.cy - cy) > 1) v.away = true;
        }

        if (it == visits.end()) {
            visits.push_back(CellVisit{cx, cy, ctx.timestamp, 1, false, false});
            if (visits.size() > kMaxCellVisits) visits.erase(visits.begin());
            continue;
        }

        // Only count a genuine re-entry. The test used to be "more than two
        // scans since I was last here", which a track that never moves passes
        // as soon as estimate jitter carries it across a cell boundary and
        // back: out on one scan, in on the next, counted. A chokepoint is
        // repeated PASSAGE, so the track has to have gone somewhere.
        it->last_time = ctx.timestamp;
        if (!it->away) continue;
        it->away = false;
        ++it->count;

        if (it->count >= p.chokepoint_n && !it->reported) {
            it->reported = true;
            auto e = make_event("CHOKEPOINT", name(), {t->id()}, Severity::HIGH,
                                ctx.timestamp);
            e.location = Vec2{(static_cast<Real>(cx) + 0.5) * cell,
                              (static_cast<Real>(cy) + 0.5) * cell};
            e.metrics.push_back({"passes", static_cast<Real>(it->count)});
            e.metrics.push_back({"cell_m", cell});
            e.note = "repeated passage through one small area";
            events.push_back(std::move(e));
        }
    }
    return events;
}

// ---------------------------------------------------------------------------
// Network role inference
// ---------------------------------------------------------------------------

std::vector<NetworkRole> NetworkRoleDetector::roles(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<NetworkRole> out;
    const DomainProfile& p = *ctx.profile;

    // Roles are only meaningful relative to a population. Below three tracks
    // any classification is an artefact of the threshold, so we decline.
    if (tracks.size() < 3) return out;

    // Two different timescales, and conflating them costs accuracy in both
    // directions. `handler_stable_scans` says how long a track must be
    // observed before its behaviour counts as settled - it governs the speed
    // average and the age gate. How long a *relationship* stays current is a
    // separate question, and a much longer one: parties who deal with each
    // other every few weeks are still associates in between. The profile
    // already states that second timescale as `dormant_timeout` - how long an
    // unobserved thing goes on being believed in - which is exactly the
    // semantics wanted here.
    const int window = std::clamp(p.handler_stable_scans, 3, 60);
    const int contact_memory = std::max(p.dormant_timeout, window);
    ++scan_;

    for (const auto& t : tracks) last_seen_[t->id()] = scan_;
    for (const auto& [i, j] : ctx.near_pairs(p.coloc_dist_m, tracks.size())) {
        if (distance(tracks[i]->position(), tracks[j]->position()) < p.coloc_dist_m) {
            contacts_[tracks[i]->id()][tracks[j]->id()] = scan_;
            contacts_[tracks[j]->id()][tracks[i]->id()] = scan_;
        }
    }

    // Age out contacts, and drop state for tracks that have gone for good.
    for (auto it = contacts_.begin(); it != contacts_.end();) {
        auto& seen = it->second;
        for (auto e = seen.begin(); e != seen.end();) {
            e = (scan_ - e->second > contact_memory) ? seen.erase(e)
                                                      : std::next(e);
        }
        const auto ls = last_seen_.find(it->first);
        const bool gone =
            ls == last_seen_.end() || scan_ - ls->second > contact_memory;
        it = (seen.empty() && gone) ? contacts_.erase(it) : std::next(it);
    }
    for (auto it = last_seen_.begin(); it != last_seen_.end();) {
        if (scan_ - it->second > contact_memory) {
            role_history_.erase(it->first);
            speed_history_.erase(it->first);
            it = last_seen_.erase(it);
        } else {
            ++it;
        }
    }

    const auto n_contacts_of = [&](const std::string& id) {
        const auto it = contacts_.find(id);
        return it == contacts_.end() ? 0 : static_cast<int>(it->second.size());
    };

    // Classify against this population's own distribution rather than fixed
    // cut-offs: "moves more than most" travels between domains, "moves faster
    // than 3 m/s" does not.
    //
    // "Fast" used to mean `max(1.5 * median, courier_speed_thresh)`, and the
    // absolute floor in that max quietly vetoed the relative test. In the
    // mule-network scenario couriers ran at five times the population median
    // and were still not fast, because the profile's floor had been set from
    // the couriers' *true* speed while the classifier sees their *estimated*
    // speed, which is lower - a turning entity's smoothed velocity always is.
    // Any absolute floor has that failure mode; it is only ever calibrated
    // against a quantity nobody measures.
    //
    // So "fast" now means membership of a fast mode, if this population has
    // one. That keeps what the floor was for - in a population where nothing
    // moves, nobody is a courier - without needing a number per domain.
    // And it is *sustained* speed that carries the meaning. An entity shuttling
    // between two points reads as stationary every time it turns around, so
    // classifying on the instantaneous estimate had couriers oscillating
    // through every role in the list. Averaging over the window that already
    // defines when a property has settled - `handler_stable_scans` - is enough
    // to make the distinction stable, and it makes `avg_speed_mps` in the
    // report mean what its name says for the first time.
    std::vector<Real> speeds;
    speeds.reserve(tracks.size());
    for (const auto& t : tracks) {
        auto& hist = speed_history_[t->id()];
        hist.push_back(t->speed_mps(p.scan_dt_s));
        while (hist.size() > static_cast<std::size_t>(window)) hist.pop_front();
        speeds.push_back(std::accumulate(hist.begin(), hist.end(), Real{0}) /
                         static_cast<Real>(hist.size()));
    }

    // The population these statistics describe is the *settled* one. A scene
    // under churn carries short-lived fragments alongside its real tracks, and
    // a fragment has no velocity yet - it is born at rest and takes several
    // scans to acquire one. Including fragments drags every distribution
    // towards zero, which is how a scene full of couriers can end up with no
    // fast mode at all. Below a quorum there is nothing to be relative to, and
    // the whole population is used rather than reporting confident nonsense
    // about three tracks.
    std::vector<std::size_t> settled;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        if (tracks[i]->age() >= p.handler_stable_scans) settled.push_back(i);
    }
    if (settled.size() < 3) {
        settled.clear();
        for (std::size_t i = 0; i < tracks.size(); ++i) settled.push_back(i);
    }

    std::vector<Real> sorted_speeds;
    sorted_speeds.reserve(settled.size());
    for (std::size_t i : settled) sorted_speeds.push_back(speeds[i]);
    std::sort(sorted_speeds.begin(), sorted_speeds.end());
    const std::optional<Real> fast_mode = high_mode_threshold(sorted_speeds, 1.5);

    // Contact counts get the same treatment, for the same reason: what counts
    // as well-connected is a property of the network, not of the domain. The
    // median partitions the population cleanly and - unlike a quantile - does
    // not cap the share of the population that any one role may hold.
    std::vector<Real> contact_counts;
    contact_counts.reserve(settled.size());
    for (std::size_t i : settled) {
        contact_counts.push_back(
            static_cast<Real>(n_contacts_of(tracks[i]->id())));
    }
    std::sort(contact_counts.begin(), contact_counts.end());
    const Real median_contacts = contact_counts[contact_counts.size() / 2];

    // Betweenness is compared against this population too, not against a fixed
    // number. A normalised betweenness above 0.2 requires a near-perfect star,
    // which real contact graphs are not; against an absolute threshold the
    // hub test simply never fired, and every hub fell through to the catch-all
    // role. The speed test was already relative - this makes the two
    // consistent.
    // Over the tracks this decision can apply to, which is not everyone: a
    // courier cannot be a handler, but a courier's centrality was setting the
    // bar a handler had to clear. In any network where the couriers do the
    // moving they are also the bridges - a node that shuttles between two
    // otherwise separate neighbourhoods is the definition of one - so
    // including them put the threshold permanently out of reach of the
    // sedentary nodes it was meant to select. Comparing a quantity across
    // roles that are mutually exclusive by construction is the error; the
    // comparison group is the other candidates for the same role.
    std::vector<Real> bc_values;
    bc_values.reserve(settled.size());
    for (std::size_t i : settled) {
        const bool i_fast = fast_mode.has_value() && speeds[i] >= *fast_mode;
        if (i_fast) continue;
        bc_values.push_back(ctx.betweenness_of(tracks[i]->id()));
    }
    if (bc_values.size() < 3) {
        bc_values.clear();
        for (std::size_t i : settled) {
            bc_values.push_back(ctx.betweenness_of(tracks[i]->id()));
        }
    }
    std::sort(bc_values.begin(), bc_values.end());
    const Real bc_high = bc_values[(bc_values.size() * 3) / 4];   // upper quartile
    const Real bc_threshold = std::max(bc_high, 1e-9);

    for (std::size_t i = 0; i < tracks.size(); ++i) {
        const Track& t = *tracks[i];
        const int n_contacts = n_contacts_of(t.id());
        const Real speed = speeds[i];
        const Real bc = ctx.betweenness_of(t.id());

        std::string role = "UNKNOWN";
        Real confidence = 0.3;

        const bool fast = fast_mode.has_value() && speed >= *fast_mode;
        const bool well_connected = n_contacts > median_contacts;
        const bool few_contacts = !well_connected;
        const bool stable = t.age() >= p.handler_stable_scans;

        // The two sedentary roles test `!fast` rather than "slower than the
        // median". Half of any population is slower than its median by
        // construction, so that test excluded half the hubs for no reason -
        // and it excluded the wrong half. A near-stationary track's velocity
        // estimate is dominated by measurement noise, so the most static
        // entities in a scene routinely measure *above* the median speed. The
        // complement of the courier test says what was meant: not moving with
        // purpose.
        if (fast && well_connected && stable) {
            // Moves a lot, meets many: carries things between fixed points.
            // `stable` gates this branch as it gates the other two: a track
            // three scans old has not done anything yet, and calling it a
            // courier on the strength of one fast estimate is how a role
            // classifier ends up describing its own tracking noise.
            role = "COURIER";
            confidence = 0.55 + 0.1 * std::min(n_contacts, 4);
        } else if (bc >= bc_threshold && bc > 0.0 && stable && !fast) {
            // Sits still, but everything routes through them.
            role = "HANDLER";
            confidence = 0.5 + std::min(bc * 2.0, 0.4);
        } else if (few_contacts && stable && !fast) {
            role = "ASSET";
            confidence = 0.45;
        } else if (n_contacts > 0) {
            role = "ASSOCIATE";
            confidence = 0.35;
        }

        // A role that keeps changing is not a role. Agreement across recent
        // scans is the strongest evidence available here.
        auto& hist = role_history_[t.id()];
        hist.push_back(role);
        if (hist.size() > 10) hist.pop_front();
        const auto agree = static_cast<Real>(
            std::count(hist.begin(), hist.end(), role));
        confidence *= 0.5 + 0.5 * (agree / static_cast<Real>(hist.size()));

        NetworkRole r;
        r.track = t.id();
        r.role = role;
        r.n_contacts = n_contacts;
        r.avg_speed_mps = speed;
        r.betweenness = bc;
        r.confidence = std::clamp(confidence, 0.0, 0.99);
        r.severity = role == "HANDLER"   ? Severity::HIGH
                     : role == "COURIER" ? Severity::MEDIUM
                                         : Severity::INFO;
        out.push_back(std::move(r));
    }

    std::sort(out.begin(), out.end(), [](const NetworkRole& a, const NetworkRole& b) {
        return a.confidence > b.confidence;
    });
    return out;
}

std::vector<DetectionEvent> NetworkRoleDetector::detect(
    const std::vector<TrackPtr>& /*tracks*/, const DetectorContext& /*ctx*/) {
    return {};
}

// ---------------------------------------------------------------------------

std::vector<DetectorPtr> default_detectors() {
    std::vector<DetectorPtr> out;
    out.push_back(std::make_unique<TradecraftDetector>());
    out.push_back(std::make_unique<RendezvousWarner>());
    out.push_back(std::make_unique<ParallelRouteDetector>());
    out.push_back(std::make_unique<ModeTransitionDetector>());
    out.push_back(std::make_unique<LoiterDetector>());
    out.push_back(std::make_unique<CoverStopDetector>());
    out.push_back(std::make_unique<ChokepointDetector>());
    out.push_back(std::make_unique<NetworkRoleDetector>());
    return out;
}

}  // namespace trace
