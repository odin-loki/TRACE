// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// The filter is the load-bearing component: if its units, prediction or
// resampling are wrong, everything above it degrades in ways that look like
// detector bugs.
#include "trace/core/particle_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "test_harness.hpp"

using namespace trace;

namespace {

void test_mou_constants_are_si() {
    // The single most consequential property: a profile author writes a speed
    // in m/s and the filter must actually model that speed at the profile's
    // own scan rate, whether that is 40 ms or 4 hours.
    for (const auto& make : {&UrbanHUMINT, &Maritime, &Airspace, &SportsPitch}) {
        const DomainProfile p = make();
        const MouConstants c = MouConstants::from(p);
        for (int k = 0; k < kNumModels; ++k) {
            const Real steady_speed = std::sqrt(c.ss_vvar[k]);
            const Real intended = p.mou_models[k].sigma /
                                  std::sqrt(2.0 * p.mou_models[k].theta);
            CHECK_NEAR(steady_speed, intended, intended * 1e-6);
            CHECK(std::isfinite(c.alpha[k]));
            CHECK(c.alpha[k] >= 0.0 && c.alpha[k] <= 1.0);
        }
    }
}

/// The first two moments of an OU velocity and its integral, obtained by
/// integrating their own ODEs rather than by any closed form.
///
/// With dv = -theta v dt + sigma dW and x the integral of v,
///
///   d/dt E[v]   = -theta E[v]
///   d/dt E[v^2] = -2 theta E[v^2] + sigma^2
///   d/dt E[xv]  = E[v^2] - theta E[xv]
///   d/dt E[x^2] = 2 E[xv]
///   d/dt E[x]   = E[v]
///
/// RK4 at a few thousand steps settles these to far better than the tolerances
/// below, and nothing in it comes from the code under test. Deterministic, so
/// there is no Monte-Carlo error to budget for either.
struct OuMoments { Real mean_x; Real var_x; Real cov_xv; Real var_v; };

OuMoments ou_moments(Real theta, Real sigma, Real dt, Real v0, int steps = 4000) {
    // y = [E v, E v^2, E xv, E x^2, E x]
    std::array<Real, 5> y{v0, v0 * v0, 0.0, 0.0, 0.0};
    const auto deriv = [&](const std::array<Real, 5>& z) {
        return std::array<Real, 5>{-theta * z[0],
                                   -2.0 * theta * z[1] + sigma * sigma,
                                   z[1] - theta * z[2],
                                   2.0 * z[2],
                                   z[0]};
    };
    const auto axpy = [](const std::array<Real, 5>& a, Real c,
                         const std::array<Real, 5>& b) {
        std::array<Real, 5> r{};
        for (int i = 0; i < 5; ++i) r[i] = a[i] + c * b[i];
        return r;
    };
    const Real h = dt / steps;
    for (int n = 0; n < steps; ++n) {
        const auto k1 = deriv(y);
        const auto k2 = deriv(axpy(y, h / 2.0, k1));
        const auto k3 = deriv(axpy(y, h / 2.0, k2));
        const auto k4 = deriv(axpy(y, h, k3));
        for (int i = 0; i < 5; ++i) {
            y[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
    return OuMoments{y[4], y[3] - y[4] * y[4], y[2] - y[4] * y[0],
                     y[1] - y[0] * y[0]};
}

void test_position_step_is_the_exact_ou_integral() {
    // The position half of the MOU step used to be a trapezoid over the
    // velocity step: x += (v + v')/2 * dt. That is the right answer only while
    // `theta * dt` is small. The exact mean is v (1 - alpha)/theta, and the
    // trapezoid's v dt (1 + alpha)/2 exceeds it by 8% at theta*dt = 1, 31% at
    // 2, and a factor of five at 10 - always upwards, so a coasting track ran
    // too far and a reacquisition gate derived from it opened too wide.
    //
    // Four shipped profiles sit past theta*dt = 1 in at least one regime, and
    // OrganisedCrimeNetwork - a 300 s scan against a 30 s stationary regime -
    // sits at 10.
    //
    // Checked against moments integrated from the SDE itself, so this test
    // shares no algebra with the code it checks.
    struct Case { const char* what; Real theta; Real sigma; Real dt; };
    const Case cases[] = {
        {"CityCameraSurveillance walking", 0.125, 1.4 * std::sqrt(2.0 * 0.125), 1.0},
        {"UrbanHUMINT foot", 1.0 / 90.0, 1.4 * std::sqrt(2.0 / 90.0), 60.0},
        {"UrbanHUMINT stationary", 1.0 / 30.0, 0.2 * std::sqrt(2.0 / 30.0), 60.0},
        {"OrganisedCrimeNetwork stationary", 1.0 / 30.0, 0.2 * std::sqrt(2.0 / 30.0), 300.0},
        {"WildlifeTelemetry fleeing", 1.0 / 7200.0, 6.0 * std::sqrt(2.0 / 7200.0), 14400.0},
    };

    for (const auto& c : cases) {
        DomainProfile p = UrbanHUMINT();
        p.scan_dt_s = c.dt;
        for (int k = 0; k < kNumModels; ++k) {
            p.mou_models[k].theta = c.theta;
            p.mou_models[k].sigma = c.sigma;
        }
        const MouConstants mou = MouConstants::from(p);
        const Real v0 = 1.0;
        const OuMoments m = ou_moments(c.theta, c.sigma, c.dt, v0);

        const Real var_x = mou.x_sig1[0] * mou.x_sig1[0] +
                           mou.x_sig2[0] * mou.x_sig2[0];
        const Real cov = mou.x_sig1[0] * mou.sigma_v[0];
        const Real u = c.theta * c.dt;
        const Real trapezoid = c.dt * (1.0 + mou.alpha[0]) / 2.0;

        std::printf("  %-34s theta*dt %6.3f  mean %9.3f (ode %9.3f, "
                    "trapezoid %9.3f)  sd_x %8.3f (ode %8.3f)\n",
                    c.what, u, mou.x_mean[0], m.mean_x, trapezoid,
                    std::sqrt(var_x), std::sqrt(m.var_x));

        CHECK_NEAR(mou.x_mean[0], m.mean_x, std::abs(m.mean_x) * 1e-6);
        CHECK_NEAR(var_x, m.var_x, std::abs(m.var_x) * 1e-5);
        CHECK_NEAR(cov, m.cov_xv, std::abs(m.cov_xv) * 1e-5 + 1e-12);
        // sigma_v is the other half of the same discretisation; check it here
        // too, since the split of var_x into correlated and independent parts
        // is only meaningful if it is right.
        CHECK_NEAR(mou.sigma_v[0] * mou.sigma_v[0], m.var_v,
                   std::abs(m.var_v) * 1e-5);
        // The independent remainder must be a real number, not a rescued NaN.
        CHECK(mou.x_sig2[0] >= 0.0);
        CHECK(std::isfinite(mou.x_sig2[0]));

        // And the old expression must be visibly wrong wherever theta*dt is
        // not small - otherwise this test is not testing anything.
        if (u > 1.0) {
            CHECK(trapezoid > m.mean_x * 1.05);
        }
    }
}

void test_predict_moves_at_modelled_speed() {
    // A cloud propagated with no measurements must spread the way the motion
    // model says it should.
    //
    // This used to bracket the spread between 0.05 and 3.0 times "the mixture's
    // mean steady-state speed times elapsed time" - a sixtyfold band around a
    // quantity that is not what an integrated OU process does. Anything that
    // moved at all passed it. The prediction below is the real one: for each
    // regime, the standard deviation of the integral of a *stationary* OU
    // velocity over the elapsed time, which is
    //
    //     Var[X] = Var[X | v0] + ss_vvar * ((1 - alpha)/theta)^2
    //
    // and the cloud, which redraws its regime every step, should sit near the
    // root-mean-square of those. Measured across six profiles spanning scan
    // periods from 0.04 s to an hour it lands at 0.72 to 0.92 of it - below
    // rather than at, because switching regimes between steps averages the
    // extremes out - and 1.44 for SportsPitch, whose 0.2 s horizon is short
    // enough that the one-metre initial scatter dominates.
    for (const auto& make : {&UrbanHUMINT, &Maritime, &Airspace,
                             &CityCameraSurveillance}) {
        DomainProfile p = make();
        const MouConstants c = MouConstants::from(p);
        ParticleFilter pf(p, c, 7);
        pf.init(Vec2{0, 0}, 1.0);

        const int steps = 5;
        for (int i = 0; i < steps; ++i) pf.predict();

        const Real spread = pf.position_uncertainty();
        const Real elapsed = steps * p.scan_dt_s;

        Real mean_var = 0.0;
        for (int k = 0; k < kNumModels; ++k) {
            const Real theta = p.mou_models[k].theta;
            const Real sigma = p.mou_models[k].sigma;
            const OuMoments m = ou_moments(theta, sigma, elapsed, 1.0);
            const Real ss = sigma * sigma / (2.0 * theta);
            mean_var += (m.var_x + ss * m.mean_x * m.mean_x) / kNumModels;
        }
        // position_uncertainty is sqrt(trace(P_xy)), i.e. sqrt(2) per-axis sd.
        const Real predicted = std::sqrt(2.0 * mean_var);

        std::printf("  %-24s spread after %dx%.2fs = %.2f m, integrated-OU "
                    "prediction %.2f m (ratio %.2f)\n",
                    p.name.c_str(), steps, p.scan_dt_s, spread, predicted,
                    spread / predicted);
        CHECK(spread > 0.5 * predicted);
        CHECK(spread < 2.0 * predicted);
    }
}

void test_update_pulls_to_measurement() {
    // How close the cloud's mean gets to a fixed measurement in three updates
    // is a draw from a wide distribution, not a number.
    //
    // This used to run one seed and require the answer to be under 20 m. It
    // was under 20 m on the machine it was written on, and on most builds -
    // but the value depends on the SIMD lane count, because the lane count
    // decides the order the RNG is consumed in. The same correct code gives
    // 1.08 m at one and two lanes, 2.52 m at eight, and **27.86 m at four**,
    // which is what a hosted x86-64-v3 runner has. CI's release job had been
    // red on it while three other configurations of the same commit were
    // green.
    //
    // So the property is measured over a sample instead. Over twenty-four
    // seeds the median lands at 2.9-6.4 m depending on the architecture, and
    // the worst single seed at 73-106 m - so a per-seed threshold anywhere
    // near the median was always going to be a coin toss.
    DomainProfile p = UrbanHUMINT();
    const Vec2 obs{120.0, -80.0};
    const int seeds = 24;

    std::vector<Real> errors;
    Real worst_ratio = 0.0;
    for (int seed = 1; seed <= seeds; ++seed) {
        ParticleFilter pf(p, MouConstants::from(p),
                          static_cast<std::uint64_t>(seed));
        pf.init(Vec2{0, 0}, 50.0);
        const Real before = distance(pf.position(), obs);
        for (int i = 0; i < 3; ++i) {
            pf.predict();
            pf.update(obs);
        }
        const Real after = distance(pf.position(), obs);
        errors.push_back(after);
        worst_ratio = std::max(worst_ratio, after / std::max(before, 1e-9));
        // Every seed has to move towards the measurement. That is the
        // qualitative claim, and it holds on every one: the worst observed is
        // 73% of where it started.
        CHECK(after < 0.90 * before);
    }

    std::sort(errors.begin(), errors.end());
    const Real median = errors[errors.size() / 2];
    std::printf("  three updates at a fixed point, %d seeds: median error "
                "%.2f m, worst %.2f m, worst as a fraction of the starting "
                "offset %.2f\n",
                seeds, median, errors.back(), worst_ratio);

    // The median is the stable quantity: 15 m is a two-to-fivefold margin on
    // what every architecture from x86-64 to AVX-512 produces.
    CHECK(median < 15.0);
    CHECK(worst_ratio < 0.90);
}

void test_resampling_keeps_weights_normalised() {
    DomainProfile p = CityCameraSurveillance();
    ParticleFilter pf(p, MouConstants::from(p), 3);
    pf.init(Vec2{5, 5}, 2.0);
    for (int i = 0; i < 40; ++i) {
        pf.predict();
        // Deliberately awkward: a measurement far from the cloud each time.
        pf.update(Vec2{5.0 + i * 1.3, 5.0});
    }
    Real total = 0.0;
    for (const Real w : pf.weights()) {
        CHECK(w >= 0.0);
        total += w;
    }
    CHECK_NEAR(total, 1.0, 1e-9);
    CHECK(pf.effective_sample_size() > 1.0);
    CHECK(std::isfinite(pf.position().x));
}

void test_model_posterior_tracks_reality() {
    // A stationary entity must end up dominated by a stationary regime. Before
    // the measurement was allowed to update the regime mixture, this test
    // failed: mu simply relaxed to the transition matrix's fixed point.
    DomainProfile p = CityCameraSurveillance();
    ParticleFilter pf(p, MouConstants::from(p), 5);
    pf.init(Vec2{0, 0}, 1.0);
    for (int i = 0; i < 60; ++i) {
        pf.predict();
        pf.update(Vec2{0.0, 0.0});
    }
    const std::string still = pf.dominant_model_name();
    const auto& mu = pf.model_probabilities();
    std::printf("  stationary entity -> regime '%s' (probs %.2f %.2f %.2f %.2f)\n",
                still.c_str(), mu[0], mu[1], mu[2], mu[3]);
    CHECK(still == "standing" || still == "walking");

    // A fast, straight mover must not be classified as standing.
    ParticleFilter pf2(p, MouConstants::from(p), 6);
    pf2.init(Vec2{0, 0}, 1.0);
    for (int i = 0; i < 60; ++i) {
        pf2.predict();
        pf2.update(Vec2{static_cast<Real>(i) * 9.0, 0.0});  // 9 m/s
    }
    const auto& mu2 = pf2.model_probabilities();
    std::printf("  9 m/s mover -> regime '%s' (probs %.2f %.2f %.2f %.2f)\n",
                pf2.dominant_model_name().c_str(), mu2[0], mu2[1], mu2[2], mu2[3]);
    std::printf("  regime order: %s %s %s %s | filter speed %.2f m/s\n",
                p.mou_models[0].name.c_str(), p.mou_models[1].name.c_str(),
                p.mou_models[2].name.c_str(), p.mou_models[3].name.c_str(),
                pf2.velocity().norm());
    CHECK(pf2.dominant_model_name() != "standing");
}

void test_determinism() {
    DomainProfile p = UrbanHUMINT();
    ParticleFilter a(p, MouConstants::from(p), 99);
    ParticleFilter b(p, MouConstants::from(p), 99);
    a.init(Vec2{1, 2});
    b.init(Vec2{1, 2});
    for (int i = 0; i < 20; ++i) {
        a.predict(); a.update(Vec2{i * 3.0, i * 1.0});
        b.predict(); b.update(Vec2{i * 3.0, i * 1.0});
    }
    CHECK_NEAR(a.position().x, b.position().x, 1e-12);
    CHECK_NEAR(a.position().y, b.position().y, 1e-12);
}

}  // namespace

int main() {
    test_mou_constants_are_si();
    test_position_step_is_the_exact_ou_integral();
    test_predict_moves_at_modelled_speed();
    test_update_pulls_to_measurement();
    test_resampling_keeps_weights_normalised();
    test_model_posterior_tracks_reality();
    test_determinism();
    return trace::test::summary("test_particle_filter");
}
