// Per-source trust. Every test here exists because the original mechanism -
// "does this source's report fit the track it was assigned to?" - is circular:
// a sensor that has been steering a track fits it perfectly however wrong it
// is, and a camera biased by sixteen times its own noise scored *higher* than
// its sound neighbours.
#include "trace/core/engine.hpp"

#include <cstdio>

#include "test_harness.hpp"

using namespace trace;

namespace {

struct DriftResult {
    Real credibility_biased{0.0};
    Real credibility_sound{0.0};
    std::vector<SensorBias> biases;
    std::vector<SensorConflict> conflicts;
};

/// `n_sensors` overlapping cameras watching the same corridor; sensor 1 has a
/// mount that slips at `drift_mps`.
DriftResult run_drift(int n_sensors, Vec2 drift_mps, int scans = 200) {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 1.5;
    cfg.profile.meas_noise_var = 9.0;
    cfg.area = Area{0, 600, 0, 300};
    cfg.seed = 31;
    Engine eng(cfg);

    Rng rng(77);
    ScanReport last;
    for (int i = 0; i < scans; ++i) {
        const Real t = i * 1.0;
        std::vector<Observation> obs;
        for (int e = 0; e < 4; ++e) {
            const Vec2 truth{40.0 + i * 2.4, 60.0 + e * 55.0};
            for (int sensor = 0; sensor < n_sensors; ++sensor) {
                if (!rng.bernoulli(0.88)) continue;
                const Vec2 bias = (sensor == 1) ? drift_mps * t : Vec2{0.0, 0.0};
                obs.emplace_back(
                    "o" + std::to_string(i) + "_" + std::to_string(e) + "_" +
                        std::to_string(sensor),
                    t,
                    Vec2{truth.x + bias.x + rng.normal(0, 1.5),
                         truth.y + bias.y + rng.normal(0, 1.5)},
                    Modality::GEOINT, 0.9,
                    sensor == 1 ? "BIASED" : "SOUND_" + std::to_string(sensor));
            }
        }
        last = eng.ingest(obs, t);
    }

    return DriftResult{eng.source_credibility("BIASED"),
                       eng.source_credibility("SOUND_0"),
                       last.operational.sensor_biases,
                       last.operational.sensor_conflicts};
}

void test_biased_sensor_is_identified() {
    const DriftResult r = run_drift(4, Vec2{0.10, 0.06});

    std::printf("  credibility: biased %.3f, sound %.3f\n", r.credibility_biased,
                r.credibility_sound);
    CHECK(r.credibility_biased < r.credibility_sound);

    // The residual direction must single it out. A track is a weighted mean of
    // the sources feeding it, so their residuals nearly cancel: the biased
    // sensor drags the track towards itself, leaving its residual pointing one
    // way and every sound sensor's pointing the other.
    std::vector<std::string> suspects;
    for (const auto& b : r.biases) {
        if (b.minority_direction) suspects.push_back(b.source_id);
    }
    std::printf("  flagged against consensus: %zu source(s)%s\n", suspects.size(),
                suspects.empty() ? "" : (" -> " + suspects.front()).c_str());
    CHECK(suspects.size() == 1);
    if (!suspects.empty()) CHECK(suspects.front() == "BIASED");
}

void test_sound_estate_flags_nobody() {
    // The test that matters most in practice: no false accusations when every
    // sensor is behaving.
    const DriftResult r = run_drift(4, Vec2{0.0, 0.0});
    int suspects = 0;
    for (const auto& b : r.biases) {
        if (b.minority_direction) ++suspects;
    }
    std::printf("  sound estate: %zu biases reported, %d flagged as suspect\n",
                r.biases.size(), suspects);
    CHECK(suspects == 0);
    CHECK_NEAR(r.credibility_biased, r.credibility_sound, 0.35);
}

void test_two_sensors_record_a_conflict_but_blame_nobody() {
    // With only two sources the disagreement is identical from both sides.
    // Blaming either is a coin flip - and the first implementation did exactly
    // that, punishing the sound camera harder than the drifting one. The
    // conflict is still worth surfacing: somebody should go and look.
    const DriftResult r = run_drift(2, Vec2{0.10, 0.06});

    bool conflict_names_both = false;
    for (const auto& c : r.conflicts) {
        if ((c.source_a == "BIASED" || c.source_b == "BIASED") &&
            c.mean_disagreement_m > 3.0) {
            conflict_names_both = true;
            std::printf("  two sensors: conflict %s vs %s, %.1f m on %.0f%% of "
                        "shared sightings\n",
                        c.source_a.c_str(), c.source_b.c_str(),
                        c.mean_disagreement_m, 100.0 * c.rate);
        }
    }
    CHECK(conflict_names_both);
}

void test_credibility_stays_in_range() {
    const DriftResult r = run_drift(3, Vec2{0.05, 0.05});
    CHECK(r.credibility_biased >= 0.0 && r.credibility_biased <= 1.0);
    CHECK(r.credibility_sound >= 0.0 && r.credibility_sound <= 1.0);
    for (const auto& b : r.biases) {
        CHECK(std::isfinite(b.magnitude_m));
        CHECK(std::isfinite(b.significance));
        CHECK(b.samples > 0);
    }
}

}  // namespace

int main() {
    test_biased_sensor_is_identified();
    test_sound_estate_flags_nobody();
    test_two_sensors_record_a_conflict_but_blame_nobody();
    test_credibility_stays_in_range();
    return trace::test::summary("test_credibility");
}
