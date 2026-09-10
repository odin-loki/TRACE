// The filter is the load-bearing component: if its units, prediction or
// resampling are wrong, everything above it degrades in ways that look like
// detector bugs.
#include "trace/core/particle_filter.hpp"

#include <cstdio>

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

void test_predict_moves_at_modelled_speed() {
    // A cloud initialised at rest, propagated with no measurements, should
    // spread at roughly the mixture's steady-state speed times elapsed time.
    DomainProfile p = UrbanHUMINT();
    const MouConstants c = MouConstants::from(p);
    ParticleFilter pf(p, c, 7);
    pf.init(Vec2{0, 0}, 1.0);

    for (int i = 0; i < 5; ++i) pf.predict();

    const Real spread = pf.position_uncertainty();
    const Real elapsed = 5.0 * p.scan_dt_s;
    Real mix_speed = 0.0;
    for (int k = 0; k < kNumModels; ++k) mix_speed += 0.25 * std::sqrt(c.ss_vvar[k]);

    std::printf("  spread after 5x%.0fs = %.1f m, mixture speed %.2f m/s -> %.0f m\n",
                p.scan_dt_s, spread, mix_speed, mix_speed * elapsed);
    CHECK(spread > 0.05 * mix_speed * elapsed);
    CHECK(spread < 3.0 * mix_speed * elapsed);
}

void test_update_pulls_to_measurement() {
    DomainProfile p = UrbanHUMINT();
    ParticleFilter pf(p, MouConstants::from(p), 11);
    pf.init(Vec2{0, 0}, 50.0);
    const Vec2 obs{120.0, -80.0};
    for (int i = 0; i < 3; ++i) {
        pf.predict();
        pf.update(obs);
    }
    const Real err = distance(pf.position(), obs);
    std::printf("  after 3 updates at a fixed point, error = %.2f m\n", err);
    CHECK(err < 20.0);
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
    test_predict_moves_at_modelled_speed();
    test_update_pulls_to_measurement();
    test_resampling_keeps_weights_normalised();
    test_model_posterior_tracks_reality();
    test_determinism();
    return trace::test::summary("test_particle_filter");
}
