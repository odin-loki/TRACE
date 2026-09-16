// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/core/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "trace/backend/simd.hpp"

namespace trace {
namespace {

/// Velocity-measurement noise for the trajectory update, as a multiple of the
/// domain's own speed scale rather than a fixed metres-per-scan constant.
constexpr Real kTrajVelNoiseScale = 3.0;

constexpr Real kResampleFracPos  = 0.40;  // ESS below 40% of n -> resample
constexpr Real kResampleFracTraj = 0.35;

}  // namespace

MouConstants MouConstants::from(const DomainProfile& p, Real dt) {
    // theta is per second and sigma is the OU velocity diffusion in SI, so the
    // step must be the real scan period. Building these against a nominal
    // dt = 1 "scan" made every profile's motion models mean whatever the scan
    // rate happened to be: a vessel profile written for 9.5 m/s was being
    // asked to explain 9.5 m per hour.
    MouConstants c;
    if (dt <= 0.0) dt = std::max(p.scan_dt_s, 1e-6);
    for (int k = 0; k < kNumModels; ++k) {
        const Real theta = std::max(p.mou_models[k].theta, 1e-6);
        const Real sigma = p.mou_models[k].sigma;
        const Real a = std::exp(-theta * dt);
        c.alpha[k] = a;
        c.sigma_v[k] =
            sigma * std::sqrt((1.0 - std::exp(-2.0 * theta * dt)) / (2.0 * theta));
        c.ss_vvar[k] = sigma * sigma / (2.0 * theta);

        // The position half, exactly. With v(t) an OU process,
        //
        //   X(dt) = v0 (1-a)/theta + (sigma/theta) INT_0^dt (1 - e^{-theta u}) dW
        //
        // so the displacement is Gaussian with
        //
        //   mean  = v0 (1-a)/theta
        //   var   = (sigma/theta)^2 [ dt - 2(1-a)/theta + (1-a^2)/(2 theta) ]
        //   cov   = (sigma^2/(2 theta^2)) (1-a)^2      against v(dt) - a v0
        //
        // Splitting that covariance out lets one standard normal serve both
        // the velocity step and the part of the displacement that must move
        // with it; the remainder is independent. Two draws per axis, which is
        // what the trapezoid cost as well.
        const Real var_x = (sigma * sigma / (theta * theta)) *
                           (dt - 2.0 * (1.0 - a) / theta +
                            (1.0 - a * a) / (2.0 * theta));
        const Real cov_xv = (sigma * sigma / (2.0 * theta * theta)) *
                            (1.0 - a) * (1.0 - a);
        c.x_mean[k] = (1.0 - a) / theta;
        c.x_sig1[k] = c.sigma_v[k] > 1e-12 ? cov_xv / c.sigma_v[k] : 0.0;
        // Rounding can take this fractionally below zero when theta*dt is tiny
        // and the two terms nearly cancel; the true value cannot be negative.
        c.x_sig2[k] =
            std::sqrt(std::max(var_x - c.x_sig1[k] * c.x_sig1[k], Real{0.0}));
    }
    return c;
}

ParticleFilter::ParticleFilter(const DomainProfile& profile,
                               const MouConstants& mou, std::uint64_t seed,
                               MotionConstraintPtr constraint)
    : profile_(&profile),
      constraint_(std::move(constraint)),
      mou_(mou),
      n_(static_cast<std::size_t>(profile.n_particles)),
      rng_(seed),
      srng_(seed ^ 0xA5A5A5A5A5A5A5A5ULL) {
    x_.resize(n_);
    y_.resize(n_);
    vx_.resize(n_);
    vy_.resize(n_);
    w_.resize(n_);
    scratch_a_.resize(n_);
    scratch_b_.resize(n_);
    scratch_m_.resize(n_);
    scratch_c_.resize(n_);
    scratch_d_.resize(n_);
    model_idx_.resize(n_);
    mu_.fill(1.0 / kNumModels);
}

void ParticleFilter::init(Vec2 pos, Real pos_sigma) {
    // Initial velocity spread is the mixture's steady-state standard deviation,
    // so a track born in traffic is not forced to start slow.
    Real mix_var = 0.0;
    for (int k = 0; k < kNumModels; ++k) mix_var += mu_[k] * mou_.ss_vvar[k];
    const Real v_sigma = std::sqrt(std::max(mix_var, 1e-9));

    for (std::size_t i = 0; i < n_; ++i) {
        x_[i] = pos.x + rng_.normal(0.0, pos_sigma);
        y_[i] = pos.y + rng_.normal(0.0, pos_sigma);
        vx_[i] = rng_.normal(0.0, v_sigma);
        vy_[i] = rng_.normal(0.0, v_sigma);
        w_[i] = 1.0 / static_cast<Real>(n_);
    }
    mu_.fill(1.0 / kNumModels);
    initialised_ = true;
    invalidate();
}

void ParticleFilter::predict() {
    if (!initialised_) return;

    // Advance the regime mixture through the transition matrix.
    std::array<Real, kNumModels> new_mu{};
    for (int j = 0; j < kNumModels; ++j) {
        Real acc = 0.0;
        for (int i = 0; i < kNumModels; ++i) acc += profile_->model_trans[i][j] * mu_[i];
        new_mu[j] = acc;
    }
    Real mu_sum = std::accumulate(new_mu.begin(), new_mu.end(), Real{0.0});
    if (mu_sum <= 0.0) {
        new_mu.fill(1.0 / kNumModels);
        mu_sum = 1.0;
    }
    for (auto& m : new_mu) m /= mu_sum;

    // One MOU step, exactly:
    //
    //   v' = alpha v + sigma_v e1
    //   x' = x + x_mean v + x_sig1 e1 + x_sig2' e2
    //
    // e1 and e2 are independent standard normals, and the displacement's
    // dependence on the velocity draw is carried by x_sig1 rather than by
    // reusing v'. See MouConstants for the derivation and for what the
    // trapezoid this replaced cost at large `theta dt`.
    //
    // Jitter keeps the cloud from collapsing between measurements; scaling it
    // to the sensor's own noise keeps it meaningful across domains. It is
    // folded into the independent coefficient rather than added as a third
    // draw, so the step still costs two normals per axis.
    const Real jitter_m = std::max(profile_->pos_noise_m * 0.3, 1e-3);
    std::array<Real, kNumModels> sig2_eff{};
    for (int k = 0; k < kNumModels; ++k) {
        sig2_eff[k] =
            std::sqrt(mou_.x_sig2[k] * mou_.x_sig2[k] + jitter_m * jitter_m);
    }

    // Draw each particle's regime, then expand to per-particle OU constants.
    // The gather is scalar and cheap; the arithmetic below is the hot part.
    for (std::size_t i = 0; i < n_; ++i) {
        const std::size_t k = rng_.categorical(new_mu.data(), kNumModels);
        model_idx_[i] = static_cast<int>(k);
        scratch_a_[i] = mou_.alpha[k];
        scratch_b_[i] = mou_.sigma_v[k];
        scratch_m_[i] = mou_.x_mean[k];
        scratch_c_[i] = mou_.x_sig1[k];
        scratch_d_[i] = sig2_eff[k];
    }

    const std::size_t lanes = simd::kLanes;
    const std::size_t vec_end = (n_ / lanes) * lanes;

    for (std::size_t i = 0; i < vec_end; i += lanes) {
        simd::Batch ex, ey, jx, jy;
        srng_.normal_pair(ex, ey);
        srng_.normal_pair(jx, jy);

        const simd::Batch a  = simd::load_u(&scratch_a_[i]);
        const simd::Batch s  = simd::load_u(&scratch_b_[i]);
        const simd::Batch m  = simd::load_u(&scratch_m_[i]);
        const simd::Batch c1 = simd::load_u(&scratch_c_[i]);
        const simd::Batch c2 = simd::load_u(&scratch_d_[i]);
        const simd::Batch vx = simd::load_u(&vx_[i]);
        const simd::Batch vy = simd::load_u(&vy_[i]);

        const simd::Batch nvx = simd::fma_(a, vx, s * ex);
        const simd::Batch nvy = simd::fma_(a, vy, s * ey);

        const simd::Batch nx = simd::fma_(
            m, vx, simd::load_u(&x_[i]) + simd::fma_(c1, ex, c2 * jx));
        const simd::Batch ny = simd::fma_(
            m, vy, simd::load_u(&y_[i]) + simd::fma_(c1, ey, c2 * jy));

        simd::store_u(&x_[i], nx);
        simd::store_u(&y_[i], ny);
        simd::store_u(&vx_[i], nvx);
        simd::store_u(&vy_[i], nvy);
    }

    for (std::size_t i = vec_end; i < n_; ++i) {
        const Real ex = rng_.normal();
        const Real ey = rng_.normal();
        const Real nvx = scratch_a_[i] * vx_[i] + scratch_b_[i] * ex;
        const Real nvy = scratch_a_[i] * vy_[i] + scratch_b_[i] * ey;
        x_[i] += scratch_m_[i] * vx_[i] + scratch_c_[i] * ex +
                 scratch_d_[i] * rng_.normal();
        y_[i] += scratch_m_[i] * vy_[i] + scratch_c_[i] * ey +
                 scratch_d_[i] * rng_.normal();
        vx_[i] = nvx;
        vy_[i] = nvy;
    }

    // Confine the cloud to whatever the entity is allowed to move along.
    // Applied after the free-space step rather than instead of it, so the
    // motion model still sets the scale and only the geometry is restricted.
    //
    // The on-network decision is made once, for the whole cloud, from its mean.
    // Deciding per particle instead meant that any particle which wandered past
    // the tolerance was thereafter exempt from the constraint and free to keep
    // going - so the cloud leaked sideways one particle at a time, which is
    // precisely what the constraint exists to prevent. Whether an entity is on
    // the road is a fact about the entity, not about each Monte-Carlo sample.
    if (constraint_) {
        Real mx = 0.0, my = 0.0;
        for (std::size_t i = 0; i < n_; ++i) {
            mx += w_[i] * x_[i];
            my += w_[i] * y_[i];
        }
        if (constraint_->on_network(Vec2{mx, my})) {
            // The mean is on-network, so the entity is; project every particle
            // unconditionally.
            for (std::size_t i = 0; i < n_; ++i) {
                const Vec2 projected = constraint_->project_unconditional(Vec2{x_[i], y_[i]});
                x_[i] = projected.x;
                y_[i] = projected.y;
                const Vec2 aligned =
                    constraint_->align_unconditional(projected, Vec2{vx_[i], vy_[i]});
                vx_[i] = aligned.x;
                vy_[i] = aligned.y;
            }
        }
    }

    mu_ = new_mu;
    invalidate();
}

void ParticleFilter::update(Vec2 obs, Real r_scale) {
    if (!initialised_) return;

    const Real var =
        std::max(profile_->meas_noise_var * noise_scale_ * r_scale, 1e-9);
    const Real inv_2var = 1.0 / (2.0 * var);

    // Work in log space, then shift by the max before exponentiating: a track
    // several sigma from its measurement otherwise underflows every weight to
    // zero and the resample divides by nothing.
    Real max_log = -std::numeric_limits<Real>::infinity();
    for (std::size_t i = 0; i < n_; ++i) {
        const Real dx = obs.x - x_[i];
        const Real dy = obs.y - y_[i];
        const Real lw = -(dx * dx + dy * dy) * inv_2var +
                        std::log(w_[i] + 1e-300);
        scratch_a_[i] = lw;
        max_log = std::max(max_log, lw);
    }

    Real total = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        w_[i] = std::exp(scratch_a_[i] - max_log);
        total += w_[i];
    }
    if (total <= 0.0) {
        std::fill(w_.begin(), w_.end(), 1.0 / static_cast<Real>(n_));
    } else {
        for (auto& w : w_) w /= total;
    }

    update_model_posterior();

    if (effective_sample_size() < static_cast<Real>(n_) * kResampleFracPos) {
        resample();
    }
    invalidate();
}

// Which motion regime is this entity in?
//
// The obvious approach - sum particle weight by the regime each particle was
// drawn from - does not work, and it is worth saying why. Over one scan a
// regime only controls how fast velocity DECAYS: alpha = exp(-dt/hold). At a
// 1 s scan with hold times of 4-15 s every alpha is 0.78-0.94, so a particle
// labelled "standing" that inherited 9 m/s still travels about 7 m. Every
// regime explains the step almost equally well, the weights carry no
// information about regime, and the posterior just reproduces the transition
// matrix's fixed point.
//
// What actually separates the regimes is sustained speed, which is exactly
// what their steady-state distributions describe. Under an OU velocity process
// each axis is zero-mean Gaussian with std sqrt(ss_vvar), so speed is Rayleigh
// with that scale. Scoring the track's current speed against each regime's
// Rayleigh gives a likelihood that genuinely discriminates.
void ParticleFilter::update_model_posterior() {
    Real mvx = 0.0, mvy = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        mvx += w_[i] * vx_[i];
        mvy += w_[i] * vy_[i];
    }
    const Real speed = std::sqrt(mvx * mvx + mvy * mvy);

    std::array<Real, kNumModels> logl{};
    Real max_ll = -std::numeric_limits<Real>::infinity();
    for (int k = 0; k < kNumModels; ++k) {
        const Real var = std::max(mou_.ss_vvar[k], 1e-12);
        // log Rayleigh(speed; sigma) = log(s) - log(var) - s^2 / (2 var)
        logl[k] = std::log(speed + 1e-9) - std::log(var) - speed * speed / (2.0 * var);
        max_ll = std::max(max_ll, logl[k]);
    }

    std::array<Real, kNumModels> post{};
    Real total = 0.0;
    for (int k = 0; k < kNumModels; ++k) {
        post[k] = mu_[k] * std::exp(logl[k] - max_ll);
        total += post[k];
    }
    if (total <= 0.0) return;

    // Blend rather than replace, and keep a floor under every regime: a hard
    // switch would let one noisy scan strand the filter in a regime it can
    // never leave, because particles are only drawn from regimes with weight.
    constexpr Real kBlend = 0.6;
    constexpr Real kFloor = 0.02;
    Real renorm = 0.0;
    for (int k = 0; k < kNumModels; ++k) {
        mu_[k] = (1.0 - kBlend) * mu_[k] + kBlend * (post[k] / total);
        mu_[k] = std::max(mu_[k], kFloor);
        renorm += mu_[k];
    }
    for (auto& m : mu_) m /= renorm;
}

void ParticleFilter::update_trajectory(Vec2 obs_curr, Vec2 obs_prev, Real dt) {
    if (!initialised_) return;
    if (dt <= 0.0) dt = std::max(profile_->scan_dt_s, 1e-6);

    // Two consecutive detections imply a velocity. That is independent evidence
    // from the position update and sharpens the heading, which matters most for
    // the convergence predictors downstream.
    const Vec2 v_obs = (obs_curr - obs_prev) / dt;  // metres per second
    const Real noise =
        std::max(profile_->courier_speed_thresh * kTrajVelNoiseScale, 1e-3);
    const Real inv_2var = 1.0 / (2.0 * noise * noise);

    Real max_log = -std::numeric_limits<Real>::infinity();
    for (std::size_t i = 0; i < n_; ++i) {
        const Real dvx = v_obs.x - vx_[i];
        const Real dvy = v_obs.y - vy_[i];
        const Real lw = -(dvx * dvx + dvy * dvy) * inv_2var +
                        std::log(w_[i] + 1e-300);
        scratch_a_[i] = lw;
        max_log = std::max(max_log, lw);
    }

    Real total = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        w_[i] = std::exp(scratch_a_[i] - max_log);
        total += w_[i];
    }
    if (total <= 0.0) {
        std::fill(w_.begin(), w_.end(), 1.0 / static_cast<Real>(n_));
    } else {
        for (auto& w : w_) w /= total;
    }

    if (effective_sample_size() < static_cast<Real>(n_) * kResampleFracTraj) {
        resample();
    }
    invalidate();
}

void ParticleFilter::resample() {
    // Systematic resampling: one uniform draw, n evenly spaced strata. Lower
    // variance than multinomial and O(n) in a single pass.
    scratch_b_[0] = w_[0];
    for (std::size_t i = 1; i < n_; ++i) scratch_b_[i] = scratch_b_[i - 1] + w_[i];
    const Real total = scratch_b_[n_ - 1];
    if (total <= 0.0) return;

    const Real u0 = rng_.uniform() / static_cast<Real>(n_);
    std::vector<Real> nx(n_), ny(n_), nvx(n_), nvy(n_);
    std::vector<int> nmi(n_);

    std::size_t j = 0;
    for (std::size_t i = 0; i < n_; ++i) {
        const Real u = (u0 + static_cast<Real>(i) / static_cast<Real>(n_)) * total;
        while (j + 1 < n_ && scratch_b_[j] < u) ++j;
        nx[i] = x_[j];
        ny[i] = y_[j];
        nvx[i] = vx_[j];
        nvy[i] = vy_[j];
        nmi[i] = model_idx_[j];
    }

    x_.swap(nx);
    y_.swap(ny);
    vx_.swap(nvx);
    vy_.swap(nvy);
    model_idx_.swap(nmi);
    std::fill(w_.begin(), w_.end(), 1.0 / static_cast<Real>(n_));
}

Real ParticleFilter::effective_sample_size() const {
    Real sum_sq = 0.0;
    for (const Real w : w_) sum_sq += w * w;
    return 1.0 / (sum_sq + 1e-300);
}

void ParticleFilter::ensure_cache() const {
    if (cache_valid_ || !initialised_) return;

    Real mx = 0.0, my = 0.0, mvx = 0.0, mvy = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        mx += w_[i] * x_[i];
        my += w_[i] * y_[i];
        mvx += w_[i] * vx_[i];
        mvy += w_[i] * vy_[i];
    }
    pos_c_ = Vec2{mx, my};
    vel_c_ = Vec2{mvx, mvy};

    Real pxx = 0.0, pxy = 0.0, pyy = 0.0;
    Real qxx = 0.0, qxy = 0.0, qyy = 0.0, pxv = 0.0;
    for (std::size_t i = 0; i < n_; ++i) {
        const Real dx = x_[i] - mx;
        const Real dy = y_[i] - my;
        pxx += w_[i] * dx * dx;
        pxy += w_[i] * dx * dy;
        pyy += w_[i] * dy * dy;
        const Real dvx = vx_[i] - mvx;
        const Real dvy = vy_[i] - mvy;
        qxx += w_[i] * dvx * dvx;
        qxy += w_[i] * dvx * dvy;
        qyy += w_[i] * dvy * dvy;
        pxv += w_[i] * (dx * dvx + dy * dvy);
    }
    P_c_ = Mat2{pxx, pxy, pyy};
    // The velocity spread comes out of the same pass. It is what a forecast
    // needs and nothing was computing it: how far an entity will have got is
    // uncertain because the model diffuses AND because the velocity it starts
    // from is itself an estimate.
    Pv_c_ = Mat2{qxx, qxy, qyy};
    Pxv_c_ = pxv;

    const Real r = (profile_ ? profile_->meas_noise_var : 25.0) * noise_scale_;
    S_c_ = Mat2{pxx + r, pxy, pyy + r};
    S_inv_c_ = S_c_.inverse();
    cache_valid_ = true;
}

Vec2 ParticleFilter::position() const { ensure_cache(); return pos_c_; }
Vec2 ParticleFilter::velocity() const { ensure_cache(); return vel_c_; }

Vec2 ParticleFilter::velocity_mps(Real /*scan_dt*/) const {
    ensure_cache();
    return vel_c_;  // the filter's state velocity is already in m/s
}

Mat2 ParticleFilter::position_covariance() const { ensure_cache(); return P_c_; }
Mat2 ParticleFilter::innovation_covariance() const { ensure_cache(); return S_c_; }

Real ParticleFilter::mahalanobis_sq(Vec2 obs) const {
    ensure_cache();
    return S_inv_c_.quad(obs - pos_c_);
}

Real ParticleFilter::position_uncertainty() const {
    ensure_cache();
    return std::sqrt(std::max(P_c_.trace_(), 0.0));
}

Real ParticleFilter::velocity_uncertainty() const {
    ensure_cache();
    return std::sqrt(std::max(Pv_c_.trace_(), 0.0));
}

Real ParticleFilter::position_velocity_covariance() const {
    ensure_cache();
    return Pxv_c_;
}

int ParticleFilter::dominant_model() const {
    return static_cast<int>(std::distance(
        mu_.begin(), std::max_element(mu_.begin(), mu_.end())));
}

const std::string& ParticleFilter::dominant_model_name() const {
    static const std::string kUnknown = "unknown";
    if (!profile_) return kUnknown;
    return profile_->mou_models[dominant_model()].name;
}

std::size_t ParticleFilter::sample_index(Rng& rng) const {
    Real u = rng.uniform();
    for (std::size_t i = 0; i < n_; ++i) {
        u -= w_[i];
        if (u <= 0.0) return i;
    }
    return n_ - 1;
}

}  // namespace trace
