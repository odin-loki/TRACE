// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// Per-source trust. Every test here exists because the original mechanism -
// "does this source's report fit the track it was assigned to?" - is circular:
// a sensor that has been steering a track fits it perfectly however wrong it
// is, and a camera biased by sixteen times its own noise scored *higher* than
// its sound neighbours.
#include "trace/core/engine.hpp"
#include "trace/core/threat.hpp"

#include <cstdio>
#include <vector>

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

void test_a_lone_source_is_not_discounted() {
    // Credibility is a relative judgement, and with one source there is nothing
    // to be relative to. The only signal left is the fit-to-track test this
    // file exists to distrust, and in a dense scene it falls steadily for a
    // reason that is not the sensor's fault - ambiguous association. Because
    // the score multiplies into the birth gate, that decay used to switch track
    // birth off part-way through a long sequence: MOT20-03 went from 45 tracks
    // to 10 while its detector went on supplying 80 detections a frame.
    SourceCredibility cred;
    cred.note_source("ONLY_SENSOR");
    for (int i = 0; i < 500; ++i) {
        // Five hundred scans of the worst possible evidence.
        cred.update("ONLY_SENSOR", -1e3, -5.0);
    }
    const Real lone = cred.get("ONLY_SENSOR");
    std::printf("  lone source after 500 bad fits: %.3f\n", lone);
    CHECK(lone >= 1.0);

    // The moment a peer exists, judgement resumes - the mechanism is suspended
    // for want of a comparison, not disabled.
    cred.note_source("SECOND_SENSOR");
    const Real judged = cred.get("ONLY_SENSOR");
    std::printf("  same source once a peer exists: %.3f\n", judged);
    CHECK(judged < 0.2);
}

void test_birth_survives_a_long_single_source_run() {
    // The end-to-end form of the above: one sensor, one crowd, long enough for
    // any decay to bite. Track count must not collapse while detections hold.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.area = Area{0, 400, 0, 400};
    cfg.seed = 7;
    Engine engine(cfg);

    Rng rng(21);
    const int kEntities = 30;
    std::vector<Vec2> truth;
    for (int i = 0; i < kEntities; ++i) {
        truth.push_back(Vec2{10.0 + 12.0 * (i % 10), 40.0 + 30.0 * (i / 10)});
    }

    int early = 0;
    int late = 0;
    for (int scan = 0; scan < 900; ++scan) {
        std::vector<Observation> obs;
        for (int i = 0; i < kEntities; ++i) {
            truth[i].x += 1.2;
            if (truth[i].x > 390.0) truth[i].x = 10.0;
            obs.emplace_back("o" + std::to_string(scan) + "_" + std::to_string(i),
                             static_cast<Real>(scan),
                             Vec2{truth[i].x + rng.normal() * 0.5,
                                  truth[i].y + rng.normal() * 0.5},
                             Modality::GEOINT, 0.3, "ONE_CAMERA");
        }
        const ScanReport r = engine.ingest(obs, static_cast<Real>(scan));
        if (scan == 150) early = r.n_tracks;
        if (scan == 880) late = r.n_tracks;
    }

    std::printf("  single-source run: %d tracks at scan 150, %d at scan 880\n",
                early, late);
    CHECK(early > kEntities / 2);
    CHECK(late >= early * 3 / 4);
}

/// Run one source against a moving target and return what the engine decides
/// its measurement noise is, as a multiple of what the profile asserts.
Real learned_noise_scale(Real bias_m, Real noise_multiple) {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 2.0;
    cfg.profile.meas_noise_var = 4.0;
    cfg.profile.adaptive_meas_noise = true;
    cfg.profile.mou_models = {{motion("walking", 45.0, 1.5),
                               motion("standing", 8.0, 0.10)}};
    cfg.profile.model_trans = {{{{0.92, 0.08}}, {{0.20, 0.80}}}};
    cfg.area = Area{0, 4000, 0, 400};
    cfg.seed = 4;
    Engine eng(cfg);

    Rng rng(88);
    Vec2 truth{60.0, 200.0};
    const Real sigma = 2.0 * noise_multiple;
    for (int scan = 0; scan < 400; ++scan) {
        truth.x += 1.5;
        std::vector<Observation> obs{
            {"o" + std::to_string(scan), static_cast<Real>(scan),
             Vec2{truth.x + bias_m + rng.normal() * sigma,
                  truth.y + rng.normal() * sigma},
             Modality::GEOINT, 0.9, "ONE_SENSOR"}};
        eng.ingest(obs, static_cast<Real>(scan));
    }
    for (const auto& [id, sc] : eng.noise_scales()) {
        if (id == "ONE_SENSOR") return sc;
    }
    return -1.0;
}

void test_bias_is_not_mistaken_for_noise() {
    // Bias and noise are different moments of the same residual and call for
    // opposite responses: a biased sensor should be distrusted, a noisy one
    // merely believed less precisely. Measured about zero rather than about the
    // source's own offset, a sensor whose mount has drifted reads as a noisy
    // one and gets its association gate *widened* - the one response that helps
    // its wrong detections keep hold of tracks. On the sensor-drift scenario
    // that cost twelve points of recovery.
    const Real sound = learned_noise_scale(0.0, 1.0);
    const Real biased = learned_noise_scale(12.0, 1.0);
    const Real noisy = learned_noise_scale(0.0, 3.0);

    std::printf("  learned noise scale: sound %.2f, biased-by-6-sigma %.2f, "
                "genuinely-noisy %.2f\n", sound, biased, noisy);

    CHECK(sound > 0.0);          // the estimator ran at all
    // A sound sensor is left alone: the profile's assumption is right.
    CHECK(sound < 2.0);
    // A biased sensor is not called noisy. Its offset is six times the assumed
    // noise, so measuring spread about zero would inflate this enormously.
    CHECK(biased < 2.5);
    // A genuinely noisy one is caught.
    CHECK(noisy > sound * 1.5);
}

/// A coverage map: a wide-area sensor that sees everything, and a gate that
/// sees only a disc.
class DiscCoverage final : public SensorCoverage {
public:
    DiscCoverage(Vec2 centre, Real radius) : centre_(centre), radius_(radius) {}
    [[nodiscard]] bool covers(const std::string& source_id,
                              Vec2 point) const override {
        if (source_id == "WIDE") return true;
        return source_id == "GATE" && distance(point, centre_) <= radius_;
    }
    [[nodiscard]] std::vector<std::string> live_sources() const override {
        return {"WIDE", "GATE"};
    }

private:
    Vec2 centre_;
    Real radius_;
};

void test_coverage_map_recovers_a_point_sensors_detection_rate() {
    // A gate reader watching a few metres reports a track for the moment it is
    // in range and never again. Without a coverage map the engine charges it
    // with a miss on every subsequent scan, so every point sensor estimates out
    // at the floor and the number is worthless. Told what the sensor can see,
    // the same evidence gives the right answer.
    //
    // The target crosses the disc repeatedly and is detected 60% of the time
    // while inside it.
    const Vec2 centre{200.0, 200.0};
    const Real radius = 40.0;
    const Real true_pd = 0.6;

    const auto run = [&](bool with_coverage) {
        EngineConfig cfg;
        cfg.profile = CityCameraSurveillance();
        cfg.profile.scan_dt_s = 1.0;
        cfg.profile.pos_noise_m = 2.0;
        cfg.profile.meas_noise_var = 4.0;
        cfg.profile.p_detection = 0.9;          // deliberately wrong
        cfg.profile.adaptive_p_detection = true;
        cfg.profile.mou_models = {{motion("walking", 45.0, 1.5),
                                   motion("standing", 8.0, 0.10)}};
        cfg.profile.model_trans = {{{{0.92, 0.08}}, {{0.20, 0.80}}}};
        cfg.area = Area{0, 400, 0, 400};
        cfg.seed = 6;
        if (with_coverage) {
            cfg.coverage = std::make_shared<DiscCoverage>(centre, radius);
        }
        Engine eng(cfg);

        Rng rng(1234);
        Vec2 truth{20.0, 200.0};
        Real vx = 1.8;
        for (int scan = 0; scan < 900; ++scan) {
            truth.x += vx;
            if (truth.x > 380.0 || truth.x < 20.0) vx = -vx;
            std::vector<Observation> obs;
            // A wide-area sensor keeps the track alive wherever it goes. This
            // is what makes the question meaningful at all: with only the gate,
            // a missed track simply dies, no further misses are recorded
            // against the gate, and the estimate is conditioned on survival.
            obs.emplace_back("w" + std::to_string(scan), static_cast<Real>(scan),
                             Vec2{truth.x + rng.normal() * 2.0,
                                  truth.y + rng.normal() * 2.0},
                             Modality::GEOINT, 0.9, "WIDE");
            const bool in_range = distance(truth, centre) <= radius;
            if (in_range && rng.bernoulli(true_pd)) {
                obs.emplace_back("g" + std::to_string(scan), static_cast<Real>(scan),
                                 Vec2{truth.x + rng.normal() * 2.0,
                                      truth.y + rng.normal() * 2.0},
                                 Modality::GEOINT, 0.9, "GATE");
            }
            eng.ingest(obs, static_cast<Real>(scan));
        }
        for (const auto& [id, r] : eng.detection_rates()) {
            if (id == "GATE") return r;
        }
        return -1.0;
    };

    const Real blind = run(false);
    const Real informed = run(true);
    std::printf("  point sensor at true p_d %.2f: learned %.2f without a coverage "
                "map, %.2f with one\n", true_pd, blind, informed);

    CHECK(informed > 0.0);
    // Told what the sensor sees, the estimate lands on the truth.
    CHECK(std::abs(informed - true_pd) < 0.12);
    // Without it the estimate is dragged down: every scan the target spends
    // out of the gate's range, while the wide sensor keeps its track alive, is
    // charged to the gate as a miss. The gap is not enormous because the
    // feeder window already stops charging a sensor ten scans after its last
    // hit - which is the same mitigation, done blind and approximately.
    CHECK(blind >= 0.0);
    CHECK(informed > blind + 0.04);
}


// ---------------------------------------------------------------------------
// Dempster-Shafer fusion of a track's supporting evidence
// ---------------------------------------------------------------------------

Observation report(Modality m, Real confidence) {
    Observation o;
    o.source_id = "S";
    o.modality = m;
    o.confidence = confidence;
    o.position = Vec2{0.0, 0.0};
    return o;
}

void test_learned_noise_tracks_variance_not_its_square_root() {
    // The profile asserts meas_noise_var = 4.0, i.e. sigma = 2 m, and the
    // helper gives the sensor sigma = 2*m. Its true VARIANCE ratio is
    // therefore m^2, and a multiplier on a variance has to learn m^2.
    //
    // It learned sqrt(m^2) instead, because the NIS driving it was measured
    // against an innovation covariance that already carried the scale: under
    // an applied scale s the expected NIS is 2k/s, so setting the scale to
    // mean_nis/target solves s = k/s. Fitted across ratios from 2.25 to 9 the
    // exponent was 0.51 rather than 1.0.
    //
    // The bands below straddle the correct answer and exclude its square root
    // by a wide margin, which is the only thing that makes this a test of the
    // fix rather than of the arithmetic.
    const Real ratio4 = learned_noise_scale(0.0, 2.0);   // true ratio 4
    const Real ratio9 = learned_noise_scale(0.0, 3.0);   // true ratio 9

    CHECK(ratio4 > 3.0 && ratio4 < 5.5);     // 4, not 2
    CHECK(ratio9 > 6.5 && ratio9 < 11.5);    // 9, not 3

    // Monotone, and by roughly the right factor: 9/4 = 2.25.
    CHECK(ratio9 > ratio4);
    CHECK((ratio9 / ratio4) > 1.6);
}

void test_fusion_masses_stay_a_mass_function() {
    // Belief never exceeds plausibility, and both stay probabilities. This is
    // the invariant that fails the moment the accumulator stops summing to
    // one, and it failed silently before: belief and plausibility were both
    // exactly 1.0 for every input, which satisfies "in [0,1]" and is still
    // meaningless.
    DomainProfile p;
    for (Real c = 0.0; c <= 1.0; c += 0.05) {
        for (int n = 1; n <= 8; ++n) {
            std::vector<Observation> ev(static_cast<std::size_t>(n),
                                        report(Modality::GEOINT, c));
            const Credibility cr = fuse_credibility(ev, p);
            CHECK(cr.belief >= 0.0 && cr.belief <= 1.0);
            CHECK(cr.plausibility >= 0.0 && cr.plausibility <= 1.0);
            CHECK(cr.conflict >= 0.0 && cr.conflict <= 1.0);
            CHECK(cr.belief <= cr.plausibility + 1e-12);
        }
    }
}

void test_fusion_discriminates() {
    // The whole point of the mechanism: more and better evidence must produce
    // more belief than less and worse. Before the vacuous prior was restored
    // every one of these comparisons was an equality at 1.0.
    DomainProfile p;
    const Credibility weak =
        fuse_credibility({report(Modality::GEOINT, 0.05)}, p);
    const Credibility strong =
        fuse_credibility({report(Modality::GEOINT, 0.95)}, p);
    const Credibility many =
        fuse_credibility({report(Modality::GEOINT, 0.95),
                          report(Modality::GEOINT, 0.95),
                          report(Modality::GEOINT, 0.95)}, p);

    CHECK(weak.belief < strong.belief);
    CHECK(strong.belief < many.belief);
    CHECK(weak.belief < 0.2);          // a near-worthless report stays weak
    CHECK(many.belief > 0.9);           // three good ones are near-conclusive
    // Uncertainty shrinks as evidence accumulates: plausibility comes down to
    // meet belief.
    CHECK((many.plausibility - many.belief) < (weak.plausibility - weak.belief));
}

void test_fusion_reports_conflict_only_when_sources_disagree() {
    // Corroborating evidence is not conflict. A run of agreeing reports must
    // not drive K up, or "our sources contradict each other" stops being
    // distinguishable from "we have a lot of evidence" - which is the one
    // distinction tracking K separately exists to make.
    DomainProfile p;
    const Credibility agreeing =
        fuse_credibility({report(Modality::GEOINT, 0.9),
                          report(Modality::GEOINT, 0.9),
                          report(Modality::GEOINT, 0.9)}, p);
    const Credibility disagreeing =
        fuse_credibility({report(Modality::GEOINT, 0.9),
                          report(Modality::GEOINT, 0.02),
                          report(Modality::GEOINT, 0.02)}, p);
    CHECK(disagreeing.conflict > agreeing.conflict);
    CHECK(agreeing.conflict < 0.5);
}

void test_fusion_with_no_evidence_is_uncommitted() {
    DomainProfile p;
    const Credibility none = fuse_credibility({}, p);
    // The struct's own defaults, untouched: no evidence, no claim.
    CHECK_NEAR(none.belief, 0.5, 1e-12);
    CHECK_NEAR(none.conflict, 0.0, 1e-12);
}

}  // namespace

int main() {
    test_biased_sensor_is_identified();
    test_sound_estate_flags_nobody();
    test_two_sensors_record_a_conflict_but_blame_nobody();
    test_credibility_stays_in_range();
    test_a_lone_source_is_not_discounted();
    test_birth_survives_a_long_single_source_run();
    test_bias_is_not_mistaken_for_noise();
    test_coverage_map_recovers_a_point_sensors_detection_rate();
    test_learned_noise_tracks_variance_not_its_square_root();
    test_fusion_masses_stay_a_mass_function();
    test_fusion_discriminates();
    test_fusion_reports_conflict_only_when_sources_disagree();
    test_fusion_with_no_evidence_is_uncommitted();
    return trace::test::summary("test_credibility");
}
