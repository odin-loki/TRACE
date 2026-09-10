// TRACE — Poisson Multi-Bernoulli Mixture track manager.
//
// Owns the full track population: which sighting belongs to which entity, when
// to start a new track, when to let one go dormant, and when a fresh sighting
// is actually a dormant entity resurfacing.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include "trace/core/observation.hpp"
#include "trace/core/track.hpp"
#include "trace/core/types.hpp"

namespace trace {

/// Beta-Poisson estimate of the clutter rate, learned from unassigned
/// detections. A fixed clutter prior over- or under-trusts the sensor as soon
/// as conditions change; this adapts within a few scans.
class ClutterEstimator {
public:
    void update(int n_unassigned);
    [[nodiscard]] Real rate() const { return alpha_ / beta_; }
    [[nodiscard]] Real density(Real volume) const {
        return volume > 0.0 ? rate() / volume : 1e-6;
    }

private:
    Real alpha_{3.0};
    Real beta_{1.0};
    std::deque<int> window_;
};

/// Running trust score per sensor. A source whose reports repeatedly fail to
/// match any track loses influence — the cheapest available defence against a
/// spoofed or misaligned feed.
class SourceCredibility {
public:
    void update(const std::string& source_id, Real obs_loglik, Real threshold);
    [[nodiscard]] Real get(const std::string& source_id) const;

private:
    std::unordered_map<std::string, Real> scores_;
};

/// Gibbs-sampled measurement-to-track assignment.
///
/// Greedy nearest-neighbour breaks down exactly when it matters — two entities
/// passing close together. Sampling the assignment instead explores competing
/// hypotheses and lets the conflict penalty push them apart.
class GibbsAssigner {
public:
    explicit GibbsAssigner(int sweeps = 14) : sweeps_(sweeps) {}

    /// Returns track index -> observation index for assigned tracks only.
    [[nodiscard]] std::unordered_map<int, int> assign(
        const std::vector<TrackPtr>& tracks,
        const std::vector<const Observation*>& observations,
        const DomainProfile& profile, Rng& rng) const;

private:
    int sweeps_{14};
};

class PmbmManager {
public:
    PmbmManager(const DomainProfile& profile, Area area, std::uint64_t seed);

    void predict();
    void update(const std::vector<Observation>& observations, Real timestamp);

    /// Tracks whose existence probability clears the reporting threshold,
    /// strongest first.
    [[nodiscard]] std::vector<TrackPtr> confirmed() const;

    [[nodiscard]] const std::vector<TrackPtr>& all_tracks() const { return tracks_; }
    [[nodiscard]] std::size_t dormant_count() const { return dormant_.size(); }
    [[nodiscard]] Real clutter_rate() const { return clutter_.rate(); }
    [[nodiscard]] int scan() const { return scan_; }
    [[nodiscard]] const SourceCredibility& credibility() const { return cred_; }

    /// Total tracks ever created — the id counter, useful in reports.
    [[nodiscard]] int total_created() const { return track_counter_; }

private:
    struct DormantEntry {
        TrackPtr track;
        int dormant_since_scan{0};
    };

    [[nodiscard]] TrackPtr try_reacquire(const Observation& obs, Real timestamp);
    void try_group_spawn(Track& fresh);
    void prune(Real timestamp);
    void merge_duplicates();
    [[nodiscard]] std::string next_id();

    const DomainProfile* profile_{nullptr};
    Area area_{};
    MouConstants mou_{};

    std::vector<TrackPtr> tracks_;
    std::vector<DormantEntry> dormant_;

    ClutterEstimator clutter_;
    SourceCredibility cred_;
    GibbsAssigner gibbs_;

    /// Unassigned detections from the previous scan, for two-point initiation.
    std::vector<Vec2> prev_unassigned_;

    int scan_{0};
    int track_counter_{0};
    Rng rng_;
    std::uint64_t seed_{0};
};

}  // namespace trace
