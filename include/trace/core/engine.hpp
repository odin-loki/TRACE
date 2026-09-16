// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — the engine.
//
//   observations -> [T] tracking -> [R] re-identification -> [A] association
//                -> [C] convergence -> [E] events -> ScanReport
//
// One call to ingest() advances the whole pipeline by one scan and returns
// everything the engine concluded.
#pragma once

#include <algorithm>
#include <set>
#include <unordered_map>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "trace/core/network.hpp"
#include "trace/core/observation.hpp"
#include "trace/core/coverage.hpp"
#include "trace/core/pmbm.hpp"
#include "trace/core/profile.hpp"
#include "trace/core/report.hpp"
#include "trace/core/threat.hpp"
#include "trace/detectors/base.hpp"

namespace trace {

struct EngineConfig {
    DomainProfile profile{UrbanHUMINT()};
    Area area{-5000.0, 5000.0, -5000.0, 5000.0};
    std::vector<Vec2> high_value_locations;
    std::uint64_t seed{0x5EED};
    int forecast_horizon{6};
    /// Optional: confine entities to a road, rail or corridor network. Free
    /// space is the default and is right for most domains.
    MotionConstraintPtr motion_constraint;

    /// Optional: what each sensor can see. Supplying it lets the engine stop
    /// guessing at two things it currently infers - whether a miss happened
    /// inside anybody's coverage, and whether a silent scan means an empty
    /// scene or a dead estate. See coverage.hpp.
    SensorCoveragePtr coverage;
};

class Engine {
public:
    explicit Engine(EngineConfig config = {});
    ~Engine();

    Engine(Engine&&) noexcept;
    Engine& operator=(Engine&&) noexcept;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    /// Advance one scan.
    ScanReport ingest(const std::vector<Observation>& observations, Real timestamp);

    // -- Detector registry --------------------------------------------------
    void register_detector(DetectorPtr detector);
    bool unregister_detector(std::string_view name);
    [[nodiscard]] std::vector<std::string> detector_names() const;

    // -- Introspection ------------------------------------------------------
    [[nodiscard]] const DomainProfile& profile() const { return config_.profile; }
    [[nodiscard]] const EngineConfig& config() const { return config_; }
    /// The most recent scans, oldest first, at most `kHistoryScans` of them.
    ///
    /// Bounded deliberately. A ScanReport carries the scan's targets, clusters,
    /// events and warnings, so it is kilobytes rather than bytes, and keeping
    /// one per scan for the life of the engine grew resident memory by about
    /// 10 MB per 1000 scans with the live track count held flat - linear, with
    /// no plateau, for a deployment that is meant to run for weeks. Everything
    /// `performance_report()` needs is accumulated as it goes instead, so no
    /// reported number changed when the bound was introduced.
    [[nodiscard]] const std::deque<ScanReport>& history() const { return history_; }

    /// How many scans `history()` retains. Two hundred and fifty-six is enough
    /// for any caller that wants "the recent past" and costs a few megabytes.
    static constexpr std::size_t kHistoryScans = 256;
    [[nodiscard]] int scan_count() const { return scan_count_; }

    /// Current trust score for a sensor, 0..1. Starts at 0.8 for an unknown
    /// source and moves with how well its reports fit the tracks they are
    /// assigned to.
    /// What the engine has learned about each sensor's detection probability.
    /// Empty unless `adaptive_p_detection` is on and a source has produced
    /// enough samples.
    [[nodiscard]] std::vector<std::pair<std::string, Real>> detection_rates() const {
        return pmbm_.detection_rates();
    }

    /// What the engine has learned about each sensor's real measurement noise,
    /// as a multiple of what its profile asserts. Empty unless
    /// `adaptive_meas_noise` is on and a source has produced enough samples.
    [[nodiscard]] std::vector<std::pair<std::string, Real>> noise_scales() const {
        return pmbm_.noise_scales();
    }

    [[nodiscard]] Real source_credibility(const std::string& source_id) const {
        return pmbm_.credibility().get(source_id);
    }

    /// Human-readable single-scan summary for the console.
    [[nodiscard]] std::string summary(const ScanReport& report) const;

    /// Session-level totals.
    [[nodiscard]] std::string performance_report() const;

private:
    EngineConfig config_;
    PmbmManager pmbm_;
    NetworkAnalyser network_;
    AnomalyEscalator escalator_;
    std::vector<DetectorPtr> detectors_;
    std::unordered_map<std::string, std::vector<Observation>> obs_cache_;
    std::deque<ScanReport> history_;

    /// Session statistics, accumulated per scan so the full history does not
    /// have to be kept to compute them.
    struct Running {
        int peak_tracks{0};
        long events{0};
        long rendezvous{0};
        long roles{0};
        int last_n_dormant{0};
        /// One double per scan: 8 KB per thousand scans, against the kilobytes
        /// per scan a retained ScanReport costs.
        std::vector<Real> latencies;
        /// Every track id ever reported. Unbounded in principle, but this IS
        /// the statistic - "how many distinct identities has this session
        /// seen" - and an id is a short string, so a thousand of them is tens
        /// of kilobytes rather than tens of megabytes.
        std::set<std::string> unique_ids;
    };
    Running stats_;

    Rng rng_;
    int scan_count_{0};
    Real total_latency_ms_{0.0};
};

}  // namespace trace
