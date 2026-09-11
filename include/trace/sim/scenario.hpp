// TRACE — scenario harness and scoring.
//
// Running a simulation is only half the job; the other half is deciding whether
// the engine got it right. These metrics compare engine output against the
// world's hidden truth, which the engine never sees.
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include "trace/core/engine.hpp"
#include "trace/sim/sensor.hpp"
#include "trace/sim/world.hpp"

namespace trace::sim {

/// Accumulated truth-vs-track comparison over a whole run.
struct Metrics {
    int scans{0};
    int total_truth{0};          ///< truth entities present, summed over scans
    int total_detected{0};       ///< of those, how many had a track nearby
    int total_tracks{0};         ///< confirmed tracks, summed over scans
    int id_switches{0};          ///< a truth entity changing assigned track id
    int ghost_tracks{0};         ///< confirmed tracks matching no truth entity
    Real position_error_sum{0.0};
    int position_error_n{0};
    Real max_position_error{0.0};

    /// Per-truth-entity assignment history, for identity continuity.
    std::map<std::string, std::string> current_assignment;
    std::map<std::string, int> assignment_changes;

    /// Truth-scans in which at least one sensor actually covered the entity.
    /// No tracker can report an entity nothing can see, so this is the honest
    /// denominator: detection_rate alone conflates tracker failure with sensor
    /// coverage, and in sparse-sensor domains the second dominates entirely.
    int covered_truth{0};

    std::vector<Real> latencies_ms;

    [[nodiscard]] Real sensor_coverage() const {
        return total_truth > 0 ? static_cast<Real>(covered_truth) / total_truth : 0.0;
    }
    /// Detection rate as a fraction of what the sensors made possible.
    [[nodiscard]] Real recovery_of_ceiling() const {
        return covered_truth > 0
                   ? static_cast<Real>(total_detected) / static_cast<Real>(covered_truth)
                   : 0.0;
    }

    [[nodiscard]] Real detection_rate() const {
        return total_truth > 0 ? static_cast<Real>(total_detected) / total_truth : 0.0;
    }
    [[nodiscard]] Real mean_position_error() const {
        return position_error_n > 0 ? position_error_sum / position_error_n : 0.0;
    }
    [[nodiscard]] Real mean_latency_ms() const;
    [[nodiscard]] Real p95_latency_ms() const;
    [[nodiscard]] Real median_latency_ms() const;

    [[nodiscard]] std::string summary() const;
};

/// Match tracks to truth for one scan and fold the result into `m`.
///
/// `match_radius_m` is how close a track must be to count as tracking that
/// entity. It should reflect the domain's own scale, not a universal constant.
void score_scan(Metrics& m, const std::vector<Entity>& truth,
                const std::vector<TargetReport>& tracks, Real match_radius_m);

/// A complete runnable scenario.
struct Scenario {
    std::string name;
    std::string description;
    EngineConfig engine_config;
    Real match_radius_m{25.0};
    int n_scans{100};

    World world;
    std::vector<SensorPtr> sensors;

    /// Called each scan before sensing - lets a scenario re-task entities,
    /// switch cameras off, or make a target go dark partway through.
    std::function<void(Scenario&, int)> on_scan;

    /// Called each scan after the engine has reported - for rendering.
    std::function<void(const Scenario&, int, const ScanReport&, const Metrics&)>
        on_report;

    /// Which entities the sensors actually detected on the scan being reported.
    ///
    /// The sensors' own ledger, which the engine never sees. Every claim in
    /// this repository about what the engine recovered is a ratio against this,
    /// and a scenario whose conditions change part-way through needs it per
    /// scan rather than summed over the run - an average across good conditions
    /// and bad hides exactly the thing such a scenario measures.
    DetectionLedger last_ledger;

    explicit Scenario(std::uint64_t seed = 20260910) : world(seed), rng(seed ^ 0xBEEF) {}

    Rng rng;
};

/// Run a scenario to completion and return its metrics.
Metrics run(Scenario& scenario, Engine& engine);

}  // namespace trace::sim
