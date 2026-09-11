// TRACE — Poisson Multi-Bernoulli Mixture track manager.
//
// Owns the full track population: which sighting belongs to which entity, when
// to start a new track, when to let one go dormant, and when a fresh sighting
// is actually a dormant entity resurfacing.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <set>
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
    /// Judge a source against how well its report fits the track it was
    /// assigned to. Weak evidence: a sensor whose reports have been steering a
    /// track all along will fit it perfectly, however wrong it is.
    void update(const std::string& source_id, Real obs_loglik, Real threshold);

    /// Judge a source against what *other* sources said about the same entity
    /// in the same scan.
    ///
    /// This is the only test that can catch a systematically biased sensor. A
    /// sensor that consistently lies is consistent with the track its own lies
    /// produced - the fit-to-track test is circular, and a camera whose mount
    /// had slipped sixteen times its own noise ended up scoring *higher* than
    /// its sound neighbours. Peer disagreement is not circular, because the
    /// peers are independent of the source being judged.
    ///
    /// `disagreement_m` is how far this source's report sits from the mean of
    /// the others'; `expected_m` is how far apart independent reports of the
    /// same entity should be on noise alone.
    /// Requires at least two peers, i.e. three sources reporting the entity.
    /// With only one peer the disagreement is symmetric by construction and
    /// carries no information about which of the two is wrong - see
    /// `record_pairwise_conflict`.
    void update_against_peers(const std::string& source_id, Real disagreement_m,
                              Real expected_m);

    /// Two sources disagreeing, with no third to break the tie. The conflict is
    /// real and worth surfacing, but blaming either would be a coin flip, so
    /// neither credibility is touched.
    void record_pairwise_conflict(const std::string& a, const std::string& b,
                                  Real disagreement_m, Real expected_m);

    /// Sensor pairs whose reports of the same entity persistently disagree by
    /// more than noise explains. Actionable even when attribution is not:
    /// somebody should go and look at these two.
    struct Conflict {
        std::string source_a;
        std::string source_b;
        int observations{0};
        int disagreements{0};
        Real mean_disagreement_m{0.0};
        [[nodiscard]] Real rate() const {
            return observations > 0
                       ? static_cast<Real>(disagreements) / observations
                       : 0.0;
        }
    };
    [[nodiscard]] std::vector<Conflict> conflicts(Real min_rate = 0.5) const;

    /// Accumulate the signed offset between a source's report and the track it
    /// was assigned to.
    ///
    /// This is the signal that survives where the others do not. A sound
    /// sensor's residuals are zero-mean - it is wrong in every direction
    /// equally - while a miscalibrated one is wrong in the *same* direction
    /// every time. Consistency of direction is the evidence, not magnitude, so
    /// it works with a single sensor on a track and needs no peer overlap.
    void note_residual(const std::string& source_id, Vec2 residual,
                       Real pos_noise_m);

    /// A sensor whose residuals point consistently one way, with the offset
    /// estimated. Operationally this is better than a trust score: it says
    /// which sensor to re-survey and by how much it is out.
    struct Bias {
        std::string source_id;
        Vec2 offset_m{};
        Real magnitude_m{0.0};
        Real significance{0.0};   ///< offset relative to what noise explains
        int samples{0};
        /// True when this source's residual points against the consensus.
        ///
        /// A track is a weighted mean of the sources feeding it, so their
        /// residuals must very nearly cancel. A single biased sensor drags the
        /// track towards itself, which leaves *its* residual pointing one way
        /// and every sound sensor's pointing the other. The minority direction
        /// is the culprit - and unlike magnitude, that is recoverable.
        bool minority_direction{false};
    };
    [[nodiscard]] std::vector<Bias> biases(Real min_significance = 3.0) const;

    /// Record whether a source's report was assigned to any track.
    ///
    /// The residual test above can only see a bias smaller than the association
    /// gate: beyond that the sensor's reports stop matching anything and the
    /// evidence is censored exactly when it becomes most damning. A chronically
    /// unassigned source is the complementary signal - it is either badly
    /// miscalibrated, or reporting things nobody else can see.
    void note_assignment(const std::string& source_id, bool assigned);

    struct Orphaned {
        std::string source_id;
        Real unassigned_rate{0.0};
        int reports{0};
    };
    [[nodiscard]] std::vector<Orphaned> orphaned_sources(Real min_rate = 0.5) const;

    /// Trust in a source, as a multiplier in [0,1].
    ///
    /// Returns the neutral default whenever only one source has ever reported.
    /// Credibility is a *relative* judgement and the class says so twice over:
    /// the fit-to-track test is circular, and peer disagreement - the one test
    /// that is not - needs peers. With a single sensor the only input is the
    /// circular one, and in a dense scene it falls steadily for a reason that
    /// has nothing to do with the sensor: ambiguous association is not the
    /// sensor's fault. Since the score multiplies into the birth gate, that
    /// decay shut track birth off completely part-way through MOT20's longer
    /// sequences - the track count fell from 45 to 10 while the detector went
    /// on supplying 80 detections a frame. Discounting the only source there
    /// is cannot be right in any case: there is nothing to compare it against,
    /// and nothing left if you disbelieve it.
    [[nodiscard]] Real get(const std::string& source_id) const;

    /// Note that a source reported this scan, whether or not it was assigned.
    /// Establishes how many sources exist, which is what makes the scores
    /// above comparable to anything.
    void note_source(const std::string& source_id);

private:
    struct ResidualState {
        Vec2 mean{};
        int n{0};
        Real noise{1.0};
    };
    std::unordered_map<std::string, Real> scores_;
    std::set<std::string> seen_sources_;
    struct AssignState {
        int total{0};
        int unassigned{0};
    };
    std::unordered_map<std::string, ResidualState> residuals_;
    std::unordered_map<std::string, AssignState> assignment_;
    std::map<std::pair<std::string, std::string>, Conflict> conflicts_;
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
    PmbmManager(const DomainProfile& profile, Area area, std::uint64_t seed,
                MotionConstraintPtr constraint = nullptr);

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
    MotionConstraintPtr constraint_;
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
