// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// Engine-level contracts: what a caller is entitled to rely on in a ScanReport.
#include "trace/core/engine.hpp"
#include "trace/core/pmbm.hpp"
#include "trace/detectors/detectors.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#define TRACE_TEST_HAVE_PROC 1
#include <unistd.h>
#endif

// A sanitizer makes resident memory meaningless as a leak signal: ASan pads
// every allocation with redzones and holds freed blocks in a quarantine, so
// the same workload that plateaus at 8 MB plateaus at 444 MB and climbs 260 MB
// over the stretch this test bounds at 6. The workload still RUNS under the
// sanitizers - which is where they do their own, better, leak and
// undefined-behaviour checking - and only the RSS assertion stands down.
#if defined(__SANITIZE_ADDRESS__)
#define TRACE_TEST_SANITIZED 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(memory_sanitizer)
#define TRACE_TEST_SANITIZED 1
#endif
#endif

#include "test_harness.hpp"

using namespace trace;

namespace {

std::vector<Observation> one(const std::string& id, Real t, Vec2 p,
                             Real conf = 0.92) {
    return {Observation{id, t, p, Modality::GEOINT, conf, "CAM_1"}};
}

void test_empty_scan_is_safe() {
    // A scan with nothing in it happens constantly in the field; it must not be
    // a special case the caller has to guard.
    Engine eng;
    const ScanReport r = eng.ingest({}, 0.0);
    CHECK(r.n_observations == 0);
    CHECK(r.n_tracks == 0);
    CHECK(r.targets.empty());
    CHECK(r.scan == 1);
    CHECK(!eng.summary(r).empty());
}

void test_track_forms_and_reports() {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.area = Area{0, 200, 0, 200};
    Engine eng(cfg);

    for (int i = 0; i < 25; ++i) {
        const Real t = i * 1.0;
        eng.ingest(one("o" + std::to_string(i), t, Vec2{20.0 + i * 1.2, 40.0}), t);
    }
    const ScanReport r = eng.ingest(one("final", 25.0, Vec2{50.0, 40.0}), 25.0);

    CHECK(r.n_tracks >= 1);
    CHECK(!r.targets.empty());
    if (!r.targets.empty()) {
        const auto& t = r.targets.front();
        CHECK(t.existence > 0.5);
        CHECK(t.hits > 10);
        CHECK(distance(t.position, Vec2{50.0, 40.0}) < 15.0);
        CHECK(t.threat.mean >= 0.0 && t.threat.mean <= 1.0);
        CHECK(t.threat.stddev >= 0.0);
        // The spread is the point: a score without one cannot be acted on.
        CHECK(t.threat.p90 >= t.threat.mean - 1e-9);
        CHECK(!t.dominant_model.empty());
        std::printf("  track %s: threat %.3f +/- %.3f, %d hits, model %s\n",
                    t.track_id.c_str(), t.threat.mean, t.threat.stddev, t.hits,
                    t.dominant_model.c_str());
    }
}

void test_detector_registry() {
    Engine eng;
    const auto names = eng.detector_names();
    CHECK(names.size() == 8);
    CHECK(eng.unregister_detector("Loiter"));
    CHECK(eng.detector_names().size() == 7);
    CHECK(!eng.unregister_detector("NoSuchDetector"));

    // A detector that throws must not take the engine down: a deployment can
    // register anything, and losing the tracking core to a plugin is not an
    // acceptable failure mode.
    struct Exploding final : Detector {
        [[nodiscard]] std::string name() const override { return "Exploding"; }
        std::vector<DetectionEvent> detect(const std::vector<TrackPtr>&,
                                           const DetectorContext&) override {
            throw std::runtime_error("deliberate");
        }
    };
    eng.register_detector(std::make_unique<Exploding>());
    const ScanReport r = eng.ingest(one("x", 0.0, Vec2{1, 1}), 0.0);
    const auto errors = r.events_of_type("DETECTOR_ERROR");
    CHECK(errors.size() == 1);
    std::printf("  faulty detector contained: %zu DETECTOR_ERROR event(s)\n",
                errors.size());
}

void test_determinism() {
    // Two engines, same seed, same input must agree exactly - otherwise no
    // simulation result in this repository is reproducible.
    EngineConfig cfg;
    cfg.seed = 4242;
    cfg.profile.scan_dt_s = 10.0;
    Engine a(cfg);
    Engine b(cfg);

    for (int i = 0; i < 30; ++i) {
        const Real t = i * 10.0;
        const auto obs = one("o" + std::to_string(i), t, Vec2{i * 12.0, i * 4.0});
        const ScanReport ra = a.ingest(obs, t);
        const ScanReport rb = b.ingest(obs, t);
        CHECK(ra.n_tracks == rb.n_tracks);
        if (!ra.targets.empty() && !rb.targets.empty()) {
            CHECK_NEAR(ra.targets[0].position.x, rb.targets[0].position.x, 1e-12);
            CHECK_NEAR(ra.targets[0].threat.mean, rb.targets[0].threat.mean, 1e-12);
        }
    }
}

void test_every_profile_runs() {
    // A profile that crashes or produces nonsense is worse than no profile;
    // this is the cheapest possible guard on all thirteen.
    for (const auto& name : profile_names()) {
        EngineConfig cfg;
        cfg.profile = profile_by_name(name);
        const Real span = std::max(cfg.profile.coloc_dist_m * 40.0, 1000.0);
        cfg.area = Area{-span, span, -span, span};
        Engine eng(cfg);

        const Real dt = cfg.profile.scan_dt_s;
        const Real speed = cfg.profile.courier_speed_thresh;
        for (int i = 0; i < 30; ++i) {
            const Real t = i * dt;
            eng.ingest(one("o" + std::to_string(i), t,
                           Vec2{i * speed * dt, i * speed * dt * 0.3}),
                       t);
        }
        const ScanReport r = eng.ingest({}, 30 * dt);
        CHECK(r.domain == name);
        CHECK(r.clutter_rate >= 0.0);
        for (const auto& t : r.targets) {
            CHECK(std::isfinite(t.position.x) && std::isfinite(t.position.y));
            CHECK(std::isfinite(t.threat.mean));
            CHECK(t.existence >= 0.0 && t.existence <= 1.0);
        }
    }
    std::printf("  all %zu profiles ran 30 scans without producing a NaN\n",
                profile_names().size());
}

void test_convergence_is_predicted_before_contact() {
    // The engine's distinguishing claim: warn before the meeting, not during.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.rv_threshold_m = 10.0;
    cfg.area = Area{0, 400, 0, 200};
    Engine eng(cfg);

    Real first_warning_sep = -1.0;
    for (int i = 0; i < 60; ++i) {
        const Real t = i * 1.0;
        // Two entities closing head-on at 2 m/s each.
        const Vec2 a{20.0 + i * 2.0, 100.0};
        const Vec2 b{380.0 - i * 2.0, 100.0};
        std::vector<Observation> obs{
            {"a" + std::to_string(i), t, a, Modality::GEOINT, 0.93, "CAM_A"},
            {"b" + std::to_string(i), t, b, Modality::GEOINT, 0.93, "CAM_B"}};
        const ScanReport r = eng.ingest(obs, t);
        if (first_warning_sep < 0.0 && !r.rendezvous.empty()) {
            first_warning_sep = r.rendezvous.front().current_sep_m;
            std::printf("  first convergence warning at %.0f m separation, "
                        "ETA %.0f s, method %s\n",
                        first_warning_sep, r.rendezvous.front().eta_s,
                        r.rendezvous.front().method.c_str());
        }
    }
    CHECK(first_warning_sep > 0.0);
    // Warned well before they were anywhere near meeting range.
    CHECK(first_warning_sep > cfg.profile.rv_threshold_m * 3.0);
}

void test_slow_convergence_is_still_predicted() {
    // The convergence detector skips a pair that cannot meet inside the
    // warning horizon at the sum of its own two speeds. That bound is what
    // stops it walking every pair in a dense scene, and the failure mode it
    // risks is the opposite of the one above: a pair closing slowly, where the
    // separation is large relative to what their speeds cover. If the bound is
    // ever tightened past what the horizon actually permits, this is the test
    // that notices.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.rv_threshold_m = 10.0;
    cfg.area = Area{0, 400, 0, 200};
    Engine eng(cfg);

    Real first_warning_sep = -1.0;
    for (int i = 0; i < 90; ++i) {
        const Real t = i * 1.0;
        // Closing at 0.35 m/s each - a quarter of walking pace.
        const Vec2 a{140.0 + i * 0.35, 100.0};
        const Vec2 b{260.0 - i * 0.35, 100.0};
        std::vector<Observation> obs{
            {"a" + std::to_string(i), t, a, Modality::GEOINT, 0.93, "CAM_A"},
            {"b" + std::to_string(i), t, b, Modality::GEOINT, 0.93, "CAM_B"}};
        const ScanReport r = eng.ingest(obs, t);
        if (first_warning_sep < 0.0 && !r.rendezvous.empty()) {
            first_warning_sep = r.rendezvous.front().current_sep_m;
            std::printf("  slow closers: first warning at %.0f m, ETA %.0f s, "
                        "method %s\n",
                        first_warning_sep, r.rendezvous.front().eta_s,
                        r.rendezvous.front().method.c_str());
        }
    }
    CHECK(first_warning_sep > 0.0);
    CHECK(first_warning_sep > cfg.profile.rv_threshold_m * 3.0);
}

void test_possibility_mismatch_discriminates() {
    // The dual-existence diagnostic must separate a track built on strong
    // evidence from one built on weak evidence repeated often. Before the
    // possibility update was fixed it could only fall, so every long-lived
    // track saturated at a mismatch of 1.0 and the diagnostic was noise.
    const auto mismatch_after = [](Modality m, Real conf) {
        EngineConfig cfg;
        cfg.profile = CityCameraSurveillance();
        cfg.profile.scan_dt_s = 1.0;
        cfg.area = Area{0, 400, 0, 400};
        cfg.seed = 4;
        Engine eng(cfg);
        ScanReport last;
        for (int i = 0; i < 40; ++i) {
            const Real t = i * 1.0;
            std::vector<Observation> obs{
                {"o" + std::to_string(i), t, Vec2{50.0 + i * 1.5, 200.0}, m, conf, "S"}};
            last = eng.ingest(obs, t);
        }
        return last.targets.empty() ? -1.0 : last.targets[0].possibility_mismatch;
    };

    const Real strong = mismatch_after(Modality::GEOINT, 0.95);
    const Real marginal = mismatch_after(Modality::OSINT, 0.60);
    std::printf("  possibility mismatch: strong evidence %.2f, marginal %.2f\n",
                strong, marginal);

    CHECK(strong >= 0.0);
    CHECK(marginal >= 0.0);
    CHECK(strong < 0.40);      // strong evidence is not flagged
    CHECK(marginal > 0.50);    // weak evidence promoted to certainty is
    CHECK(marginal > strong * 1.5);
}

}  // namespace

/// Resident set size in kilobytes, or -1 where it cannot be read.
long rss_kb() {
#if defined(TRACE_TEST_HAVE_PROC)
    std::ifstream in("/proc/self/statm");
    if (!in) return -1;
    long size = 0, resident = 0;
    in >> size >> resident;
    if (!in) return -1;
    return resident * (sysconf(_SC_PAGESIZE) / 1024);
#else
    return -1;
#endif
}

void test_memory_plateaus_under_track_turnover() {
    // A deployment runs for weeks. Entities arrive, are tracked, and leave, so
    // the number of live tracks stays flat while the number of track ids ever
    // created climbs without limit - and every per-track map in the engine, the
    // detectors, the contact graph and the escalator is keyed by that id.
    //
    // Before the engine swept retired ids, resident memory rose about 10 MB per
    // thousand scans on exactly this load and never levelled off: 4.3 MB at the
    // first scan, 44.7 MB by the four thousandth. Two thirds of that was
    // ScanReports retained one per scan for the life of the engine; the rest
    // was per-track state for identities long gone.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.area = Area{-2000, 2000, -2000, 2000};
    cfg.seed = 7;
    Engine engine(cfg);
    Rng rng(99);

    const Real dt = cfg.profile.scan_dt_s;
    constexpr int kLive = 6;         // entities alive at any moment
    constexpr int kLifeScans = 25;   // then they leave and are replaced
    constexpr int kWarmup = 600;     // let allocation settle before measuring
    constexpr int kScans = 2600;

    long rss_warm = -1;
    for (int scan = 0; scan < kScans; ++scan) {
        std::vector<Observation> obs;
        for (int e = 0; e < kLive; ++e) {
            const int cohort = scan / kLifeScans;
            const Real phase = static_cast<Real>((cohort * 13 + e * 7) % 97);
            obs.push_back(Observation{
                "o" + std::to_string(scan) + "_" + std::to_string(e),
                static_cast<Real>(scan) * dt,
                Vec2{phase * 20.0 - 1000.0 + rng.normal(0.0, 1.0),
                     static_cast<Real>(e) * 100.0 - 300.0 + rng.normal(0.0, 1.0)},
                Modality::GEOINT, 0.9, "CAM"});
        }
        engine.ingest(obs, static_cast<Real>(scan) * dt);
        if (scan == kWarmup) rss_warm = rss_kb();
    }
    const long rss_end = rss_kb();

    // The history is bounded outright, which is checkable without /proc.
    CHECK(engine.history().size() <= Engine::kHistoryScans);

#if defined(TRACE_TEST_SANITIZED)
    std::printf("  memory: RSS is not a leak signal under a sanitizer "
                "(%.1f MB here); the workload ran, the history bound is "
                "checked, and ASan/LSan/UBSan cover the rest\n",
                rss_end / 1024.0);
    return;
#endif
    if (rss_warm <= 0 || rss_end <= 0) {
        std::printf("  memory: resident size unavailable on this platform, "
                    "checked history bound only\n");
        return;
    }
    const long growth = rss_end - rss_warm;
    std::printf("  memory: %.1f MB after %d scans, +%.1f MB over the last %d "
                "(was ~+20 MB before retired ids were swept)\n",
                rss_end / 1024.0, kScans, growth / 1024.0, kScans - kWarmup);

    // Generous: the unswept engine grew by roughly 20 MB over this stretch, so
    // a 6 MB bound catches the regression without being sensitive to how the
    // allocator happens to behave.
    CHECK(growth < 6 * 1024);
}

void test_forecast_never_claims_more_precision_than_the_sensor() {
    // A forecast's uncertainty was `position_uncertainty() * sqrt(k+1)`, and
    // `position_uncertainty()` is sqrt(trace(P)) over the particle cloud - a
    // quantity the filter is free to drive below the noise of the sensor that
    // fed it, because the cloud contracts on agreement between successive
    // detections rather than on the accuracy of any one of them.
    //
    // It does. Fed a perfectly still entity at 25 fps, SportsPitch's cloud
    // tightens to 4.6 cm against a `pos_noise_m` of 25 cm - so six steps out
    // the engine published 12 cm of uncertainty for a position it could not
    // know to better than a quarter of a metre. (Exact zero, which would make
    // every waypoint a claim of perfect certainty, was NOT reachable in any
    // shipped profile; the floor is there for the reachable case.)
    EngineConfig cfg;
    cfg.profile = SportsPitch();
    cfg.area = Area{-500.0, 500.0, -500.0, 500.0};
    cfg.seed = 5;
    // A forecast is only produced for a track the threat scorer rates HIGH or
    // IMMEDIATE - "forecast only what warrants the compute" - so the entity has
    // to be worth forecasting before any of this is reachable. Sitting it on a
    // high-value location is the cheapest way to get there.
    cfg.high_value_locations = {Vec2{0.0, 0.0}};
    Engine engine(cfg);

    const Real dt = cfg.profile.scan_dt_s;
    Real tightest_track = std::numeric_limits<Real>::infinity();
    Real tightest_forecast = std::numeric_limits<Real>::infinity();
    int steps_seen = 0;
    for (int s = 0; s < 400; ++s) {
        const ScanReport r = engine.ingest(one("s" + std::to_string(s),
                                               static_cast<Real>(s) * dt,
                                               Vec2{0.0, 0.0}, 0.99),
                                           static_cast<Real>(s) * dt);
        for (const auto& tr : r.targets) {
            tightest_track = std::min(tightest_track, tr.position_uncertainty_m);
            for (const ForecastStep& f : tr.forecast) {
                tightest_forecast = std::min(tightest_forecast, f.uncertainty_m);
                ++steps_seen;
            }
        }
    }
    std::printf("  forecast floor: cloud reached %.4f m, forecast never below "
                "%.4f m, pos_noise %.3f m (%d steps)\n",
                tightest_track, tightest_forecast, cfg.profile.pos_noise_m,
                steps_seen);

    CHECK(steps_seen > 0);
    // The setup has to actually bite, or this test proves nothing.
    CHECK(tightest_track < cfg.profile.pos_noise_m);
    // And the forecast must not inherit it.
    CHECK(tightest_forecast >= cfg.profile.pos_noise_m - 1e-9);
}

void test_forecast_advances_a_scan_period_per_step() {
    // A forecast step stamped one scan period into the future must place the
    // entity one scan period of travel away. `Track::velocity()` is metres per
    // second, so the step is v * scan_dt_s - not v, which is one second of
    // travel and correct only where the scan period is one second.
    //
    // Nine of the ten shipped profiles have a scan period that is not one
    // second, from 0.04 s at 25 fps to 14,400 s for a satellite collar duty
    // cycle, so the forecast was out by that factor in all of them: a vessel
    // at 9.5 m/s predicted an hour ahead was placed 9.5 m from where it
    // started rather than 34 km. Checked here on a 60 s profile, where the
    // error was sixty-fold.
    EngineConfig cfg;
    cfg.profile = UrbanHUMINT();            // scan_dt_s = 60
    cfg.area = Area{-500000, 500000, -500000, 500000};
    cfg.seed = 3;
    Engine engine(cfg);

    // Walk a vessel in a straight line long enough for the filter to settle on
    // a velocity, then look at what it predicts.
    const Real dt = cfg.profile.scan_dt_s;
    const Real speed = 1.4;                 // m/s, walking pace
    Vec2 truth{0.0, 0.0};
    const TargetReport* target = nullptr;
    ScanReport last;
    for (int i = 0; i < 24; ++i) {
        last = engine.ingest(one("v" + std::to_string(i), static_cast<Real>(i) * dt,
                                 truth, 0.95),
                             static_cast<Real>(i) * dt);
        truth.x += speed * dt;
    }
    for (const auto& t : last.targets) {
        if (!t.forecast.empty()) { target = &t; break; }
    }
    // Not a skip. Twenty-four clean detections of one walker at p=0.95 is the
    // easiest thing this engine is ever asked to do, so no forecast here means
    // forecasting has stopped working - and a test that returns quietly in
    // that case is a test that passes with nothing asserted, which is the one
    // outcome it must not have.
    CHECK(target != nullptr);
    if (target == nullptr) return;

    // Each step must move by one scan period of the reported velocity, and the
    // timestamps must agree with the distance.
    const Vec2 v = target->velocity_mps;
    Vec2 expected = target->position;
    for (std::size_t k = 0; k < target->forecast.size(); ++k) {
        expected += v * dt;
        const ForecastStep& f = target->forecast[k];
        const Real err = distance(f.position, expected);
        CHECK(err <= 1e-6 * (1.0 + std::abs(expected.x) + std::abs(expected.y)));
    }
    // No forecast step may claim perfect certainty. `position_uncertainty()`
    // is sqrt(trace(P)) over the particle cloud, and a cloud that has
    // collapsed - which a long run of tight detections produces - returns
    // zero; zero times sqrt(k+1) is still zero, so every waypoint went out
    // saying the entity would be exactly there. No estimate is better than the
    // measurement that fed it, so the floor is the profile's own position
    // noise.
    Real worst_unc = std::numeric_limits<Real>::infinity();
    for (const ForecastStep& f : target->forecast) {
        worst_unc = std::min(worst_unc, f.uncertainty_m);
        CHECK(f.uncertainty_m >= cfg.profile.pos_noise_m - 1e-9);
        CHECK(std::isfinite(f.uncertainty_m));
    }
    std::printf("  forecast: tightest uncertainty %.3f m against pos_noise %.3f m\n",
                worst_unc, cfg.profile.pos_noise_m);
    // And it must widen with the horizon, not stay flat.
    CHECK(target->forecast.back().uncertainty_m >
          target->forecast.front().uncertainty_m);

    const Real step_m = distance(target->forecast[0].position, target->position);
    const Real step_s = target->forecast[0].timestamp - last.timestamp;
    std::printf("  forecast: first step %.0f m over %.0f s (%.2f m/s reported)\n",
                step_m, step_s, v.norm());
    // The distance travelled must match the time elapsed at the reported
    // speed. With `p += v` it was out by the scan period - 3600x here.
    CHECK(std::abs(step_m - v.norm() * step_s) <= 1e-3 * (step_m + 1.0));
}

void test_fitted_velocity_is_metres_per_scan() {
    // `fitted_velocity` fits a line to a track's recent positions and returns
    // the slope in metres per SCAN. `Track::history_` is appended once per
    // accepted DETECTION, not once per scan, so the sample index is not a
    // clock: two sensors reporting the same scan add two samples at the same
    // instant, and a track that goes unseen adds none at all.
    //
    // Regressed against the index, that reads a slope of metres per SAMPLE and
    // calls it metres per scan. On a 2 m/s target with a 60 s scan - truth
    // 120 m per scan - one sensor gave 123, two gave 45.8 and four gave 31.0.
    // Regressed against the timestamp the samples already carry, the bias goes.
    const DomainProfile profile = UrbanHUMINT();
    const Real truth_per_scan = 2.0 * profile.scan_dt_s;

    for (int n_sensors : {1, 2, 4}) {
        PmbmManager pmbm(profile, Area{-50000, 50000, -50000, 50000}, 5);
        Vec2 truth{0.0, 0.0};
        for (int i = 0; i < 14; ++i) {
            std::vector<Observation> obs;
            for (int s = 0; s < n_sensors; ++s) {
                obs.push_back({"o" + std::to_string(i) + "_" + std::to_string(s),
                               static_cast<Real>(i) * profile.scan_dt_s, truth,
                               Modality::GEOINT, 0.95, "CAM" + std::to_string(s)});
            }
            pmbm.predict();
            pmbm.update(obs, static_cast<Real>(i) * profile.scan_dt_s);
            truth.x += 2.0 * profile.scan_dt_s;
        }
        CHECK(!pmbm.all_tracks().empty());
        if (pmbm.all_tracks().empty()) continue;
        const Vec2 v = fitted_velocity(*pmbm.all_tracks().front(), profile.scan_dt_s);
        const Real ratio = v.x / truth_per_scan;
        std::printf("  fitted_velocity, %d sensor(s): %.1f m/scan of %.1f (ratio %.2f)\n",
                    n_sensors, v.x, truth_per_scan, ratio);
        // Within 15% however many sensors are reporting, and on any build.
        // Against the index regression the four-sensor case read 0.26; with a
        // sample-count window rather than a time window it read 1.13 on the
        // vectorised build and 1.75 on the scalar one, which is what made the
        // window a span of time.
        CHECK(ratio > 0.85);
        CHECK(ratio < 1.15);
    }
}

int main() {
    test_empty_scan_is_safe();
    test_possibility_mismatch_discriminates();
    test_forecast_never_claims_more_precision_than_the_sensor();
    test_forecast_advances_a_scan_period_per_step();
    test_fitted_velocity_is_metres_per_scan();
    test_memory_plateaus_under_track_turnover();
    test_track_forms_and_reports();
    test_detector_registry();
    test_determinism();
    test_every_profile_runs();
    test_convergence_is_predicted_before_contact();
    test_slow_convergence_is_still_predicted();
    return trace::test::summary("test_engine");
}
