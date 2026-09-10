#include "trace/sim/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <set>
#include <sstream>

namespace trace::sim {

Real Metrics::mean_latency_ms() const {
    if (latencies_ms.empty()) return 0.0;
    return std::accumulate(latencies_ms.begin(), latencies_ms.end(), Real{0.0}) /
           static_cast<Real>(latencies_ms.size());
}

Real Metrics::median_latency_ms() const {
    if (latencies_ms.empty()) return 0.0;
    std::vector<Real> v = latencies_ms;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

Real Metrics::p95_latency_ms() const {
    if (latencies_ms.empty()) return 0.0;
    std::vector<Real> v = latencies_ms;
    std::sort(v.begin(), v.end());
    const auto idx = std::min(v.size() - 1, static_cast<std::size_t>(v.size() * 0.95));
    return v[idx];
}

void score_scan(Metrics& m, const std::vector<Entity>& truth,
                const std::vector<TargetReport>& tracks, Real match_radius_m) {
    ++m.scans;
    m.total_tracks += static_cast<int>(tracks.size());

    std::set<std::string> claimed_tracks;

    for (const auto& e : truth) {
        if (!e.active) continue;
        ++m.total_truth;

        // Nearest confirmed track within the match radius, each track claimable
        // once - otherwise one good track could "cover" a whole crowd.
        const TargetReport* best = nullptr;
        Real best_d = match_radius_m;
        for (const auto& t : tracks) {
            if (claimed_tracks.contains(t.track_id)) continue;
            const Real d = distance(t.position, e.position);
            if (d < best_d) {
                best_d = d;
                best = &t;
            }
        }

        if (best == nullptr) continue;

        ++m.total_detected;
        claimed_tracks.insert(best->track_id);
        m.position_error_sum += best_d;
        ++m.position_error_n;
        m.max_position_error = std::max(m.max_position_error, best_d);

        // Identity continuity: the same entity should keep the same track id.
        // Switches are counted rather than averaged away because one switch can
        // invalidate an entire chain of downstream reasoning.
        const auto it = m.current_assignment.find(e.id);
        if (it == m.current_assignment.end()) {
            m.current_assignment[e.id] = best->track_id;
        } else if (it->second != best->track_id) {
            ++m.id_switches;
            ++m.assignment_changes[e.id];
            it->second = best->track_id;
        }
    }

    // Tracks nowhere near any truth entity are ghosts: clutter promoted to a
    // confirmed track, or a track that has drifted off its target.
    for (const auto& t : tracks) {
        bool matched = false;
        for (const auto& e : truth) {
            if (e.active && distance(t.position, e.position) < match_radius_m) {
                matched = true;
                break;
            }
        }
        if (!matched) ++m.ghost_tracks;
    }
}

std::string Metrics::summary() const {
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "  scans              %d\n"
        "  detection rate     %.1f%%   (%d of %d truth-scans had a track)\n"
        "  mean position err  %.2f m   (max %.2f m)\n"
        "  identity switches  %d       across %zu entities\n"
        "  ghost tracks       %d       (%.2f per scan)\n"
        "  latency            median %.3f ms  mean %.3f ms  p95 %.3f ms\n",
        scans, 100.0 * detection_rate(), total_detected, total_truth,
        mean_position_error(), max_position_error, id_switches,
        current_assignment.size(), ghost_tracks,
        scans > 0 ? static_cast<Real>(ghost_tracks) / scans : 0.0,
        median_latency_ms(), mean_latency_ms(), p95_latency_ms());
    return std::string(buf);
}

Metrics run(Scenario& scenario, Engine& engine) {
    Metrics m;
    const Real dt = scenario.engine_config.profile.scan_dt_s;

    for (int scan = 0; scan < scenario.n_scans; ++scan) {
        if (scenario.on_scan) scenario.on_scan(scenario, scan);

        scenario.world.step(dt);
        const WorldSnapshot truth = scenario.world.snapshot();
        const std::vector<Observation> obs =
            collect(scenario.sensors, truth, scenario.rng);

        const ScanReport report = engine.ingest(obs, truth.timestamp);

        score_scan(m, truth.entities, report.targets, scenario.match_radius_m);
        m.latencies_ms.push_back(report.latency_ms);

        if (scenario.on_report) scenario.on_report(scenario, scan, report, m);
    }
    return m;
}

}  // namespace trace::sim
