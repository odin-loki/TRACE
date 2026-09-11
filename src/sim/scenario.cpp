#include "trace/sim/scenario.hpp"

#include "trace/core/assignment.hpp"

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

    // Collect the live entities, keeping their indices so results map back.
    std::vector<Vec2> truth_pts;
    std::vector<const Entity*> live;
    for (const auto& e : truth) {
        if (!e.active) continue;
        live.push_back(&e);
        truth_pts.push_back(e.position);
    }
    m.total_truth += static_cast<int>(live.size());

    std::vector<Vec2> track_pts;
    track_pts.reserve(tracks.size());
    for (const auto& t : tracks) track_pts.push_back(t.position);

    // One globally optimal matching, rather than each entity independently
    // grabbing its nearest track. Independent nearest-neighbour lets a single
    // track cover a whole cluster and manufactures identity switches whenever
    // the arbitrary winner changes.
    const Assignment a = match_points(truth_pts, track_pts, match_radius_m);

    for (std::size_t i = 0; i < live.size(); ++i) {
        const int j = a.row_to_col[i];
        if (j < 0) continue;

        const auto& track = tracks[static_cast<std::size_t>(j)];
        const Real d = distance(track.position, live[i]->position);

        ++m.total_detected;
        m.position_error_sum += d;
        ++m.position_error_n;
        m.max_position_error = std::max(m.max_position_error, d);

        // Identity continuity: the same entity should keep the same track id.
        // Switches are counted rather than averaged away because one switch can
        // invalidate an entire chain of downstream reasoning.
        const auto it = m.current_assignment.find(live[i]->id);
        if (it == m.current_assignment.end()) {
            m.current_assignment[live[i]->id] = track.track_id;
        } else if (it->second != track.track_id) {
            ++m.id_switches;
            ++m.assignment_changes[live[i]->id];
            it->second = track.track_id;
        }
    }

    // A track matched to nothing is a ghost: clutter promoted to a confirmed
    // track, or a track that has drifted off its target.
    for (std::size_t j = 0; j < tracks.size(); ++j) {
        if (a.col_to_row[j] < 0) ++m.ghost_tracks;
    }
}

std::string Metrics::summary() const {
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "  scans              %d\n"
        "  sensor detections  %.1f%%   (%d of %d truth-scans produced a detection)\n"
        "  detection rate     %.1f%%   (%d of %d truth-scans had a track)\n"
        "  recovery           %.1f%%   of what the sensors made possible\n"
        "  mean position err  %.2f m   (max %.2f m)\n"
        "  identity switches  %d       across %zu entities\n"
        "  ghost tracks       %d       (%.2f per scan)\n"
        "  latency            median %.3f ms  mean %.3f ms  p95 %.3f ms\n",
        scans, 100.0 * sensor_coverage(), covered_truth, total_truth,
        100.0 * detection_rate(), total_detected, total_truth,
        100.0 * recovery_of_ceiling(),
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
        DetectionLedger ledger;
        const std::vector<Observation> obs =
            collect(scenario.sensors, truth, scenario.rng, &ledger);

        const ScanReport report = engine.ingest(obs, truth.timestamp);

        // What the sensors actually reported, before scoring what the engine
        // made of it.
        for (const auto& e : truth.entities) {
            if (e.active && ledger.contains(e.id)) ++m.covered_truth;
        }

        score_scan(m, truth.entities, report.targets, scenario.match_radius_m);
        m.latencies_ms.push_back(report.latency_ms);

        scenario.last_ledger = ledger;
        if (scenario.on_report) scenario.on_report(scenario, scan, report, m);
    }
    return m;
}

}  // namespace trace::sim
