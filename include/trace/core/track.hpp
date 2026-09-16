// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — a single hypothesised entity.
//
// Each track is a Bernoulli component: a probability `r` that the entity exists
// at all, paired with a particle distribution over where it is if it does. That
// separation is what lets a track survive an occlusion — existence decays
// gracefully instead of the track being deleted on the first missed scan.
//
// Alongside the Bayesian `r`, a possibilistic existence `pi_r` is propagated
// under different assumptions. When the two diverge sharply the track is
// flagged: it means the evidence is internally inconsistent, which is the
// signature of a spoofed or failing sensor rather than a moving entity.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "trace/core/descriptor.hpp"
#include "trace/core/observation.hpp"
#include "trace/core/particle_filter.hpp"
#include "trace/core/pattern_of_life.hpp"
#include "trace/core/profile.hpp"
#include "trace/core/types.hpp"

namespace trace {

/// One remembered step of a track's history.
struct TrackSample {
    Real timestamp{0.0};
    Vec2 position{};
    Vec2 velocity{};
};

class Track {
public:
    Track(std::string id, Real r, const DomainProfile& profile,
          const MouConstants& mou, Real t0, std::uint64_t seed,
          MotionConstraintPtr constraint = nullptr);

    // -- Lifecycle ----------------------------------------------------------
    void predict();
    /// Fold a detection in.
    ///
    /// `source_trust` is what SourceCredibility thinks of the sensor that
    /// produced `obs`, in [0,1]. It is passed alongside the observation rather
    /// than multiplied into `obs.confidence` beforehand, because the two say
    /// different things and only one of them belongs in every consumer:
    ///
    ///   - the *filter* should believe a distrusted sensor less precisely, so
    ///     trust widens the assumed measurement noise. That is what it is for.
    ///   - the *possibility* measure asks how good this evidence is on its own
    ///     terms - which modality, at what confidence the sensor asserted.
    ///     Folding trust in there made `possibility_mismatch` fire on
    ///     everything the moment a scene had more than one source, because
    ///     credibility is a relative score that sits near 0.45 for a perfectly
    ///     sound camera. Whether a sensor is trustworthy is a separate
    ///     question, and `credibility()`, `biases()`, `conflicts()` and
    ///     `orphaned_sources()` are how it gets answered.
    void update_hit(const Observation& obs, Real scan_dt, Real source_trust = 1.0);
    /// `p_detect` negative means "use the profile's assertion", which is the
    /// default and what every caller did before the estimate existed.
    void update_miss(Real p_detect = -1.0);

    /// Record that `source` gave this track a detection in `scan`.
    ///
    /// The pair matters, not the scan alone. Two tracks fed in the same scan by
    /// the *same* sensor must be two entities, because one sensor reports an
    /// entity once per scan. Two tracks fed in the same scan by *different*
    /// sensors are exactly what one entity under overlapping coverage looks
    /// like - and treating that as proof of distinctness was blocking the merge
    /// that should have collapsed them.
    void note_hit_scan(int scan, const std::string& source);
    [[nodiscard]] bool shares_hit_scan_with(const Track& other) const;

    /// Do the two tracks draw on any of the same sensors? Disjoint sensor sets
    /// are what one entity under overlapping coverage looks like.
    [[nodiscard]] bool shares_source_with(const Track& other) const;
    [[nodiscard]] std::size_t hit_record_count() const { return hit_scans_.size(); }

    /// Absorb another track that has been judged to be the same entity.
    void absorb(const Track& other);

    /// This track's running appearance model, blended from the descriptors of
    /// every detection assigned to it.
    [[nodiscard]] const Descriptor& appearance() const { return appearance_; }

    /// Fold this scan's threat score into the running estimate. Persistence is
    /// tracked separately: a single high score is noise, a sustained one is a
    /// finding.
    void update_threat(Real score);

    // -- Identity -----------------------------------------------------------
    [[nodiscard]] const std::string& id() const { return id_; }
    [[nodiscard]] const std::optional<std::string>& parent_id() const {
        return parent_id_;
    }
    void set_parent(std::string parent) { parent_id_ = std::move(parent); }

    // -- Existence ----------------------------------------------------------
    [[nodiscard]] Real existence() const { return r_; }
    [[nodiscard]] Real possibility() const { return pi_r_; }
    [[nodiscard]] Real possibility_mismatch() const { return poss_mismatch_; }
    void set_existence(Real r) { r_ = std::clamp(r, 0.0, 1.0); }

    /// True once this track has been confident enough to report. A track that
    /// reaches this is an identity worth remembering even after its existence
    /// collapses; one that never does is discarded outright.
    [[nodiscard]] bool ever_confirmed() const { return ever_confirmed_; }
    void mark_confirmed() { ever_confirmed_ = true; }

    // -- State --------------------------------------------------------------
    [[nodiscard]] Vec2 position() const { return pf_.position(); }
    [[nodiscard]] Vec2 velocity() const { return pf_.velocity(); }  ///< m/s
    [[nodiscard]] Vec2 velocity_mps(Real scan_dt) const {
        return pf_.velocity_mps(scan_dt);
    }
    [[nodiscard]] Real speed_mps(Real scan_dt) const {
        return velocity_mps(scan_dt).norm();
    }
    [[nodiscard]] Real velocity_uncertainty() const {
        return pf_.velocity_uncertainty();
    }
    [[nodiscard]] Real position_velocity_covariance() const {
        return pf_.position_velocity_covariance();
    }
    [[nodiscard]] Real position_uncertainty() const {
        return pf_.position_uncertainty();
    }
    [[nodiscard]] const std::string& dominant_model() const {
        return pf_.dominant_model_name();
    }

    ParticleFilter& filter() { return pf_; }
    [[nodiscard]] const ParticleFilter& filter() const { return pf_; }
    PatternOfLife& pol() { return pol_; }
    [[nodiscard]] const PatternOfLife& pol() const { return pol_; }

    // -- Counters -----------------------------------------------------------
    [[nodiscard]] Real born_at() const { return born_at_; }
    [[nodiscard]] Real last_seen() const { return last_seen_; }
    [[nodiscard]] int age() const { return age_; }
    /// Detections accepted, counting each sensor separately.
    [[nodiscard]] int hits() const { return n_hit_; }

    /// Scans in which this track was detected at all, counting a scan once
    /// however many sensors reported it. This is the numerator of a rate;
    /// `hits()` is not.
    [[nodiscard]] int hit_scans() const { return n_hit_scans_; }
    [[nodiscard]] int misses() const { return n_miss_; }
    /// Fraction of the scans this track has existed for in which it was
    /// detected. In [0,1] by construction.
    ///
    /// Counted per SCAN, not per detection. `update_hit` runs once for every
    /// observation in a scan's group, so a track under four overlapping
    /// sensors used to score four hits against one scan of age and report a
    /// "rate" of 4.0. Both thresholds tested against it - the group-spawn test
    /// at 0.85 and the merge test at 0.55 - were then true for any track with
    /// more than one sensor on it, whatever its actual detection history.
    ///
    /// Computed on demand rather than cached at the last hit. A cached value
    /// is stale by exactly the length of the current coast, so a track that
    /// has not been seen for twenty scans went on reporting the rate it had
    /// when it was last detected - high, and wrong in the same direction as
    /// the per-detection count was.
    ///
    /// The denominator is `age_ + 1`, not `age_`. A track is born during a
    /// scan and detected in that same scan, so it has existed for one scan
    /// when its age is zero; `predict()` increments age at the start of each
    /// scan after. Dividing by `age_` counted the scan of birth in the
    /// numerator and not the denominator, which on its own put a
    /// perfectly-detected track at 2.0 on its second scan and left it above 1
    /// for the rest of its life.
    [[nodiscard]] Real measurement_rate() const {
        return static_cast<Real>(n_hit_scans_) / static_cast<Real>(age_ + 1);
    }
    /// As `measurement_rate`, kept separate because callers read it as a
    /// density rather than a rate. The `min` used to be load-bearing, clamping
    /// a per-detection count that could exceed 1; it is now a guard only.
    [[nodiscard]] Real detection_density() const {
        return std::min(1.0, static_cast<Real>(n_hit_scans_) /
                                 static_cast<Real>(age_ + 1));
    }
    [[nodiscard]] Real mean_observation_quality() const;

    // -- History ------------------------------------------------------------
    [[nodiscard]] const std::deque<TrackSample>& history() const { return history_; }
    [[nodiscard]] Real threat_ema() const { return threat_ema_; }
    [[nodiscard]] int threat_persistence() const { return threat_persistence_; }
    [[nodiscard]] const std::deque<Real>& threat_history() const {
        return threat_history_;
    }

    /// Mean velocity over the last `n` history samples, in **metres per
    /// second**. More stable than the instantaneous filter velocity for
    /// heading tests.
    ///
    /// The samples it averages are `pf_.velocity()`, which `velocity()` above
    /// documents as m/s, so this is m/s too. It said "metres per scan" until
    /// the units were checked. Nothing consumed it wrongly - its only caller
    /// takes a ratio of two of them - but this engine has shipped three
    /// scans-for-seconds defects already, and a comment that misstates a unit
    /// is where the fourth would come from.
    ///
    /// `n == 0` returns the instantaneous velocity rather than dividing by
    /// zero.
    [[nodiscard]] Vec2 smoothed_velocity(std::size_t n = 4) const;

    /// Total path length walked over the last `n` samples.
    [[nodiscard]] Real path_length(std::size_t n) const;

    /// Net displacement over the last `n` samples. The ratio of this to
    /// path_length separates a straight transit from a loop or a loiter.
    [[nodiscard]] Real net_displacement(std::size_t n) const;

    /// Accumulated heading change in radians — the winding number that flags a
    /// surveillance-detection route.
    [[nodiscard]] Real winding(std::size_t n) const;

private:
    std::string id_;
    std::optional<std::string> parent_id_;

    Real r_{0.65};
    Real pi_r_{0.65};
    Real poss_mismatch_{0.0};
    bool ever_confirmed_{false};

    ParticleFilter pf_;
    PatternOfLife pol_;
    const DomainProfile* profile_{nullptr};

    Real born_at_{0.0};
    Real last_seen_{0.0};
    int age_{0};
    int n_hit_{0};
    /// Scans in which at least one sensor reported this track, and the last
    /// such scan, so a multi-sensor scan is counted once.
    int n_hit_scans_{0};
    int last_hit_scan_{-1};
    int n_miss_{0};

    std::deque<TrackSample> history_;
    std::deque<Real> obs_weights_;
    std::deque<Real> threat_history_;
    Real threat_ema_{0.5};
    int threat_persistence_{0};

    std::optional<Vec2> last_obs_pos_;
    Real last_obs_ts_{-1e18};
    struct HitRecord {
        int scan{0};
        std::size_t source{0};   ///< hashed source id
        friend bool operator==(const HitRecord& a, const HitRecord& b) {
            return a.scan == b.scan && a.source == b.source;
        }
    };
    std::deque<HitRecord> hit_scans_;
    Descriptor appearance_{};
};

using TrackPtr = std::shared_ptr<Track>;

}  // namespace trace
