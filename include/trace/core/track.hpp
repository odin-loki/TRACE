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
    void update_hit(const Observation& obs, Real scan_dt);
    void update_miss();

    /// Record that this track was assigned a detection in `scan`. Two tracks
    /// that are never hit in the same scan are competing for one entity's
    /// detections; two that often are belong to different entities.
    void note_hit_scan(int scan);
    [[nodiscard]] bool shares_hit_scan_with(const Track& other) const;
    [[nodiscard]] const std::deque<int>& hit_scans() const { return hit_scans_; }

    /// Absorb another track that has been judged to be the same entity.
    void absorb(const Track& other);

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
    [[nodiscard]] int hits() const { return n_hit_; }
    [[nodiscard]] int misses() const { return n_miss_; }
    [[nodiscard]] Real measurement_rate() const { return mrate_; }
    [[nodiscard]] Real detection_density() const {
        return std::min(1.0, static_cast<Real>(n_hit_) / std::max(age_, 1));
    }
    [[nodiscard]] Real mean_observation_quality() const;

    // -- History ------------------------------------------------------------
    [[nodiscard]] const std::deque<TrackSample>& history() const { return history_; }
    [[nodiscard]] Real threat_ema() const { return threat_ema_; }
    [[nodiscard]] int threat_persistence() const { return threat_persistence_; }
    [[nodiscard]] const std::deque<Real>& threat_history() const {
        return threat_history_;
    }

    /// Mean velocity over the last `n` history samples, in metres per scan.
    /// More stable than the instantaneous filter velocity for heading tests.
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
    int n_miss_{0};
    Real mrate_{0.0};

    std::deque<TrackSample> history_;
    std::deque<Real> obs_weights_;
    std::deque<Real> threat_history_;
    Real threat_ema_{0.5};
    int threat_persistence_{0};

    std::optional<Vec2> last_obs_pos_;
    Real last_obs_ts_{-1e18};
    std::deque<int> hit_scans_;
};

using TrackPtr = std::shared_ptr<Track>;

}  // namespace trace
