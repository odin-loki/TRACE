// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — Mixed Ornstein-Uhlenbeck particle filter.
//
// State is [x, y, vx, vy] held structure-of-arrays so the propagation and
// likelihood kernels vectorise cleanly. Velocity follows an OU process whose
// regime (foot / vehicle / stationary / fast, or the domain's equivalent) is
// itself a Markov chain — the "mixed" in MOU. Regime probabilities are carried
// as an IMM-style mixture and resampled per particle each step.
//
// Time base: SI throughout. theta is per second, sigma is the OU velocity
// diffusion, velocities are metres per second, and one predict() advances the
// profile's scan period. This is what lets a single set of motion models mean
// the same thing at a 40 ms sports frame rate and a 4-hour satellite revisit.
#pragma once

#include <algorithm>
#include <cmath>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "trace/core/motion_constraint.hpp"
#include "trace/core/profile.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/types.hpp"

namespace trace {

/// Per-regime constants derived once from a profile's motion models.
struct MouConstants {
    std::array<Real, kNumModels> alpha{};    ///< velocity retention per scan
    std::array<Real, kNumModels> sigma_v{};  ///< velocity diffusion per scan
    std::array<Real, kNumModels> ss_vvar{};  ///< steady-state velocity variance

    /// The exact discretisation of the POSITION half of the same process.
    ///
    /// Integrating an OU velocity over one scan gives a Gaussian displacement
    /// whose mean is `x_mean * v` and whose noise is correlated with the
    /// velocity draw that produced it. Writing it out:
    ///
    ///   v' = alpha v + sigma_v e1
    ///   dx = x_mean v + x_sig1 e1 + x_sig2 e2       e1, e2 ~ N(0,1) iid
    ///
    /// The mean coefficient is `(1 - alpha)/theta`, NOT `dt (1 + alpha)/2`.
    /// The trapezoid this replaced agrees with it to second order in
    /// `theta dt` and diverges after that, always upwards: +8% at `theta dt`
    /// of 1, +31% at 2, and a factor of five at 10. Several shipped profiles
    /// live out there - `OrganisedCrimeNetwork` samples every 300 s with a
    /// stationary regime that holds its heading for 30, so its coasting
    /// prediction ran five times too far.
    std::array<Real, kNumModels> x_mean{};   ///< metres of drift per m/s held
    std::array<Real, kNumModels> x_sig1{};   ///< noise shared with the velocity draw
    std::array<Real, kNumModels> x_sig2{};   ///< the independent remainder

    /// `dt` defaults to the profile's own scan period.
    static MouConstants from(const DomainProfile& p, Real dt = -1.0);
};

class ParticleFilter {
public:
    ParticleFilter() = default;
    ParticleFilter(const DomainProfile& profile, const MouConstants& mou,
                   std::uint64_t seed, MotionConstraintPtr constraint = nullptr);

    /// Scatter particles about a first detection.
    void init(Vec2 pos, Real pos_sigma = 35.0);

    /// One MOU step: mix regimes, propagate velocity, integrate position.
    void predict();

    /// Reweight against a position measurement. `r_scale` widens the assumed
    /// measurement noise for a low-confidence source.
    void update(Vec2 obs, Real r_scale = 1.0);

    /// Multiplier on the profile's assumed measurement variance for this track.
    ///
    /// Affects both the update weighting and the innovation covariance, and it
    /// has to be both: scaling only the weighting would leave the association
    /// gate as tight as ever, and the gate is what rejects true detections when
    /// a sensor degrades. Scaling both also closes the loop - the normalised
    /// innovation the estimate is driven by is itself measured against the
    /// scaled covariance - so the estimate settles instead of running away.
    void set_noise_scale(Real s) {
        noise_scale_ = std::max(s, 1e-6);
        cache_valid_ = false;
    }
    [[nodiscard]] Real noise_scale() const { return noise_scale_; }

    /// Reweight against an implied velocity from two consecutive detections.
    void update_trajectory(Vec2 obs_curr, Vec2 obs_prev, Real dt = -1.0);

    // -- Estimates (cached; invalidated by any state change) ----------------
    [[nodiscard]] Vec2 position() const;
    [[nodiscard]] Vec2 velocity() const;             ///< metres per second
    [[nodiscard]] Vec2 velocity_mps(Real scan_dt) const;
    [[nodiscard]] Mat2 position_covariance() const;
    [[nodiscard]] Mat2 innovation_covariance() const;
    [[nodiscard]] Real mahalanobis_sq(Vec2 obs) const;
    [[nodiscard]] Real position_uncertainty() const; ///< sqrt(trace(P_xy))
    [[nodiscard]] Real velocity_uncertainty() const; ///< sqrt(trace(P_vv)), m/s
    /// cov(x,vx) + cov(y,vy). Positive while the cloud's position error and
    /// its velocity error point the same way, which is most of the time and is
    /// what makes a forecast's uncertainty grow faster than either term alone.
    [[nodiscard]] Real position_velocity_covariance() const;

    [[nodiscard]] int dominant_model() const;
    [[nodiscard]] const std::string& dominant_model_name() const;
    [[nodiscard]] const std::array<Real, kNumModels>& model_probabilities() const {
        return mu_;
    }

    /// Effective sample size, a direct read on particle degeneracy.
    [[nodiscard]] Real effective_sample_size() const;

    [[nodiscard]] std::size_t size() const { return n_; }
    [[nodiscard]] bool initialised() const { return initialised_; }

    // Raw access for the smoother and the CUDA path.
    [[nodiscard]] const std::vector<Real>& xs() const { return x_; }
    [[nodiscard]] const std::vector<Real>& ys() const { return y_; }
    [[nodiscard]] const std::vector<Real>& vxs() const { return vx_; }
    [[nodiscard]] const std::vector<Real>& vys() const { return vy_; }
    [[nodiscard]] const std::vector<Real>& weights() const { return w_; }

    /// Draw a particle index according to the current weights — used by the
    /// Monte-Carlo forecasters so they inherit the filter's own uncertainty.
    [[nodiscard]] std::size_t sample_index(Rng& rng) const;

private:
    void resample();
    /// Fold the measurement's verdict back into the regime mixture.
    void update_model_posterior();
    void invalidate() { cache_valid_ = false; }
    void ensure_cache() const;

    const DomainProfile* profile_{nullptr};
    MotionConstraintPtr constraint_;
    MouConstants mou_{};
    std::size_t n_{320};

    std::vector<Real> x_, y_, vx_, vy_, w_;
    // Per-particle motion constants, gathered once per predict from the
    // regime each particle drew. Five of them, because the position step is
    // the exact integral of the velocity step rather than a trapezoid over it.
    std::vector<Real> scratch_a_, scratch_b_;   // alpha, sigma_v
    std::vector<Real> scratch_m_, scratch_c_, scratch_d_;  // x_mean, x_sig1, x_sig2
    std::vector<int>  model_idx_;

    std::array<Real, kNumModels> mu_{};
    Real noise_scale_{1.0};
    bool initialised_{false};

    mutable bool cache_valid_{false};
    mutable Vec2 pos_c_{};
    mutable Vec2 vel_c_{};
    mutable Mat2 P_c_{};
    mutable Mat2 Pv_c_{};
    mutable Real Pxv_c_{0.0};
    mutable Mat2 S_c_{};
    mutable Mat2 S_inv_c_{};

    Rng rng_;
    SimdRng srng_;
};

}  // namespace trace
