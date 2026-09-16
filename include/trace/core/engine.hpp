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
#include <cstdint>

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

    /// Movable. A moved-from Engine may be destroyed or assigned to; nothing
    /// else. That is the usual contract, and it is stated because the usual
    /// *implementation* of it was wrong here until the release audit - see
    /// `config_` below for what a defaulted move used to do to a live engine.
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
    [[nodiscard]] const DomainProfile& profile() const { return config_->profile; }
    [[nodiscard]] const EngineConfig& config() const { return *config_; }
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
    /// Held indirectly so its address never changes.
    ///
    /// The profile inside it is not copied into the subsystems that read it:
    /// PmbmManager, every Track, and through each Track its particle filter
    /// and its pattern-of-life model, all keep a `const DomainProfile*` into
    /// this one object. While the config was a value member, moving an Engine
    /// moved that object to a new address and left every one of those pointers
    /// aimed at the source - so a moved-to Engine read a hollowed-out profile,
    /// and once the source was destroyed it read freed memory. ASan called it
    /// on the third scan, in Track::predict.
    ///
    /// Pinning the config fixes all of them at once, which is the point: the
    /// alternative was a rebind pass walking pmbm -> tracks -> filters on every
    /// move, and a rebind pass is only correct until someone adds the next
    /// thing that holds the pointer.
    ///
    /// Null only in a moved-from Engine.
    std::unique_ptr<EngineConfig> config_;
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
        /// Latency as a fixed histogram rather than one sample per scan.
        ///
        /// A vector of one double per scan is 8 KB per thousand scans and
        /// never stops, which is the same shape of growth as the retained
        /// ScanReports it replaced - slower by three orders of magnitude, and
        /// still unbounded, so an engine meant to run for weeks still did not
        /// plateau. 10,000 buckets of 0.01 ms up to 100 ms, plus one overflow
        /// bucket, is 40 KB whatever the session length.
        ///
        /// 0.01 ms is exactly the resolution `performance_report` prints, so
        /// the quantiles below lose nothing a reader could see. The mean and
        /// the maximum are kept exactly, as scalars, rather than read off the
        /// histogram.
        std::vector<std::uint32_t> latency_hist;
        long latency_samples{0};
        Real max_latency_ms{0.0};
        /// Every track id ever reported. Unbounded in principle, but this IS
        /// the statistic - "how many distinct identities has this session
        /// seen" - and an id is a short string, so a thousand of them is tens
        /// of kilobytes rather than tens of megabytes.
        std::set<std::string> unique_ids;
    };
    Running stats_;

    static constexpr std::size_t kLatencyBuckets = 10001;   // [0,100) ms + overflow
    static constexpr Real kLatencyBucketMs = 0.01;
    /// The count at or below which `frac` of the samples lie, in ms.
    [[nodiscard]] Real latency_quantile(Real frac) const;

    Rng rng_;
    int scan_count_{0};
    Real total_latency_ms_{0.0};
};

}  // namespace trace
