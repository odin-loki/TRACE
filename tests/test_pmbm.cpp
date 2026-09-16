// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// End-to-end check on the tracking core: can it hold identity on moving
// targets, in clutter, through missed detections?
#include "trace/core/pmbm.hpp"

#include "trace/core/engine.hpp"
#include "trace/core/track.hpp"
#include "trace/core/particle_filter.hpp"
#include <map>
#include <set>

#include <cstdio>
#include <algorithm>
#include <numbers>
#include <string>
#include <vector>

#include "test_harness.hpp"

using namespace trace;

namespace {

struct TruthTarget {
    Vec2 pos;
    Vec2 vel;  // metres per second
};

/// Straight-line targets with process noise, detected with probability p_d,
/// plus Poisson clutter — the standard multi-target tracking test case.
struct Scenario {
    std::vector<std::vector<Observation>> scans;
    std::vector<std::vector<Vec2>> truth;
};

Scenario make_scenario(int n_scans, int n_targets, Real area, Real p_detect,
                       Real clutter_rate, std::uint64_t seed,
                       Real scan_dt = 60.0) {
    Rng rng(seed);
    std::vector<TruthTarget> targets;
    for (int i = 0; i < n_targets; ++i) {
        targets.push_back(TruthTarget{
            Vec2{rng.uniform(-area * 0.5, area * 0.5),
                 rng.uniform(-area * 0.5, area * 0.5)},
            // ~1.4 m/s: the walking speed UrbanHUMINT's 'foot' regime models.
            Vec2{rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4)}});
    }

    Scenario s;
    int obs_counter = 0;
    for (int scan = 0; scan < n_scans; ++scan) {
        const Real t = static_cast<Real>(scan) * scan_dt;
        std::vector<Observation> scan_obs;
        std::vector<Vec2> scan_truth;

        for (auto& tgt : targets) {
            tgt.vel.x += rng.normal(0.0, 0.10);
            tgt.vel.y += rng.normal(0.0, 0.10);
            tgt.pos += tgt.vel * scan_dt;
            // Reflect at the boundary so targets stay in the area of regard.
            if (std::abs(tgt.pos.x) > area) { tgt.pos.x = std::copysign(area, tgt.pos.x); tgt.vel.x *= -0.7; }
            if (std::abs(tgt.pos.y) > area) { tgt.pos.y = std::copysign(area, tgt.pos.y); tgt.vel.y *= -0.7; }
            scan_truth.push_back(tgt.pos);

            if (rng.bernoulli(p_detect)) {
                scan_obs.emplace_back(
                    "o" + std::to_string(obs_counter++), t,
                    Vec2{tgt.pos.x + rng.normal(0.0, 5.0),
                         tgt.pos.y + rng.normal(0.0, 5.0)},
                    Modality::GEOINT, std::clamp(rng.normal(0.88, 0.08), 0.2, 1.0),
                    // A camera covers a region. Assigning a random source per
                    // detection would make source identity meaningless, which
                    // is not how any real sensor estate behaves.
                    "CAM_" + std::to_string((tgt.pos.x > 0 ? 2 : 0) +
                                            (tgt.pos.y > 0 ? 1 : 0)));
            }
        }

        const int n_clutter = rng.poisson(clutter_rate);
        for (int c = 0; c < n_clutter; ++c) {
            scan_obs.emplace_back(
                "c" + std::to_string(obs_counter++), t,
                Vec2{rng.uniform(-area, area), rng.uniform(-area, area)},
                Modality::OSINT, rng.uniform(0.05, 0.30), "NOISE");
        }

        s.scans.push_back(std::move(scan_obs));
        s.truth.push_back(std::move(scan_truth));
    }
    return s;
}

/// Mean distance from each truth position to the nearest confirmed track.
Real mean_nearest_error(const std::vector<TrackPtr>& tracks,
                        const std::vector<Vec2>& truth) {
    if (tracks.empty() || truth.empty()) return 1e9;
    Real acc = 0.0;
    for (const Vec2& tp : truth) {
        Real best = 1e9;
        for (const auto& tr : tracks) best = std::min(best, distance(tr->position(), tp));
        acc += best;
    }
    return acc / static_cast<Real>(truth.size());
}

void test_tracks_clean_targets() {
    // Tracking outcome varies substantially with the random stream: across
    // seeds the mean position error in this scenario spans roughly 15-200 m,
    // because a 1.4 m/s pedestrian sampled every 60 s is genuinely hard to
    // localise between detections. Asserting on a single seed tests the seed.
    const DomainProfile profile = UrbanHUMINT();
    std::vector<Real> errors;
    std::vector<std::size_t> peaks;

    for (int seed = 0; seed < 9; ++seed) {
        PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000},
                         static_cast<std::uint64_t>(seed));
        const auto sc = make_scenario(40, 5, 3000.0, 0.90, 2.0,
                                      static_cast<std::uint64_t>(seed) * 31 + 7);
        Real err_sum = 0.0;
        int err_n = 0;
        std::size_t peak = 0;
        for (std::size_t i = 0; i < sc.scans.size(); ++i) {
            pmbm.predict();
            pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
            const auto conf = pmbm.confirmed();
            peak = std::max(peak, conf.size());
            if (i >= 10) {
                err_sum += mean_nearest_error(conf, sc.truth[i]);
                ++err_n;
            }
        }
        errors.push_back(err_sum / std::max(err_n, 1));
        peaks.push_back(peak);
    }

    std::sort(errors.begin(), errors.end());
    std::sort(peaks.begin(), peaks.end());
    const Real median_err = errors[errors.size() / 2];
    const std::size_t median_peak = peaks[peaks.size() / 2];
    std::printf("  clean: median peak tracks=%zu (truth 5), median err=%.1f m "
                "(range %.0f-%.0f)\n",
                median_peak, median_err, errors.front(), errors.back());

    CHECK(median_peak >= 5);          // found essentially all of them
    CHECK(median_peak <= 8);          // without spawning a swarm of ghosts
    CHECK(median_err < 120.0);        // and localised them
}

void test_survives_detection_gap() {
    // Deliberately blind the sensor for a stretch: existence should decay but
    // the track should still be there to pick the target back up.
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 11);
    auto sc = make_scenario(40, 3, 2000.0, 0.95, 0.5, 21);

    for (int i = 12; i < 20; ++i) sc.scans[static_cast<std::size_t>(i)].clear();

    std::size_t before = 0, during = 0, after = 0;
    std::set<std::string> ids_before, ids_during, ids_after;
    Real r_before = 0.0, r_during = 0.0;
    for (std::size_t i = 0; i < sc.scans.size(); ++i) {
        pmbm.predict();
        pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
        const auto conf = pmbm.confirmed();
        // Identity and existence are read over EVERY live track, not just the
        // confirmed ones. A track that has not been seen for eight scans is
        // supposed to fall below `r_confirm` - that is the threshold doing its
        // job - so measuring either over `confirmed()` alone asks whether the
        // track is still trusted, when the question is whether it still exists.
        const auto live = pmbm.all_tracks();
        auto ids = [&live] {
            std::set<std::string> s;
            for (const auto& tr : live) s.insert(tr->id());
            return s;
        };
        auto mean_r = [&live] {
            if (live.empty()) return 0.0;
            Real acc = 0.0;
            for (const auto& tr : live) acc += tr->existence();
            return acc / static_cast<Real>(live.size());
        };
        if (i == 11) { before = conf.size(); ids_before = ids(); r_before = mean_r(); }
        if (i == 19) { during = conf.size(); ids_during = ids(); r_during = mean_r(); }
        if (i == 30) { after = conf.size(); ids_after = ids(); }
    }
    std::size_t held = 0, kept = 0;
    for (const auto& id : ids_during) held += ids_before.count(id);
    for (const auto& id : ids_after) kept += ids_before.count(id);
    std::printf("  gap: before=%zu during-blackout=%zu after=%zu; "
                "held %zu ids through, %zu of the originals still live after; "
                "mean existence %.3f -> %.3f\n",
                before, during, after, held, kept, r_before, r_during);

    CHECK(before >= 2);
    CHECK(after >= 2);

    // `during` used to be computed, printed, and asserted on by nothing, so a
    // tracker that dropped every track the moment the sensor went quiet and
    // built fresh ones afterwards passed a test called "survives a detection
    // gap". Surviving means the SAME identities are still there, which is the
    // only reading under which the name is true.
    //
    // It does NOT mean they are still confirmed. Eight blind scans take
    // existence below `r_confirm`, which is the threshold working: a track
    // nobody has seen for eight scans should not be trusted, only kept. The
    // first version of this assertion demanded `confirmed()` hold up through
    // the blackout and failed against correct behaviour.
    CHECK(held >= 2);
    CHECK(kept >= 2);

    // The docstring says existence should decay across the blackout, so check
    // that rather than asserting it in prose. Soft, because with a coverage map
    // the engine is entitled to treat a silent scan as nobody looking: this
    // requires only that it does not RISE on no evidence.
    CHECK(r_during <= r_before + 1e-9);
}

void test_clutter_estimate_adapts() {
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 3);
    const auto sc = make_scenario(30, 2, 3000.0, 0.9, 12.0, 5);
    for (std::size_t i = 0; i < sc.scans.size(); ++i) {
        pmbm.predict();
        pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
    }
    std::printf("  clutter rate estimate=%.2f (true 12.0)\n", pmbm.clutter_rate());
    CHECK(pmbm.clutter_rate() > 4.0);   // it noticed the noise
    CHECK(pmbm.clutter_rate() < 25.0);  // without running away
}

void test_ids_are_stable() {
    // Identity persistence is the product; a track that keeps changing id is
    // useless downstream regardless of how good its position is.
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 77);
    const auto sc = make_scenario(35, 3, 2500.0, 0.92, 1.0, 99);

    std::unordered_map<std::string, int> lifetimes;
    for (std::size_t i = 0; i < sc.scans.size(); ++i) {
        pmbm.predict();
        pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
        for (const auto& t : pmbm.confirmed()) lifetimes[t->id()]++;
    }
    int long_lived = 0;
    for (const auto& [id, n] : lifetimes) {
        if (n >= 20) ++long_lived;
    }
    std::printf("  ids: %zu distinct, %d lived >=20 scans (truth 3)\n",
                lifetimes.size(), long_lived);
    CHECK(long_lived >= 2);
}

void test_identity_survives_a_blackout() {
    // The capability the whole dormancy machinery exists for: an entity that
    // disappears entirely and comes back should keep its identity, not be
    // reported as somebody new.
    const DomainProfile profile = Maritime();
    PmbmManager pmbm(profile, Area{0, 200000, 0, 200000}, 7);
    const Real dt = profile.scan_dt_s;

    std::string before, after;
    bool went_dormant = false;

    for (int i = 0; i < 90; ++i) {
        const Real t = i * dt;
        // A patrol box, so the entity is near where it was when it returns.
        const Real ph = 2.0 * std::numbers::pi * (i % 20) / 20.0;
        const Vec2 p{100000.0 + 20000.0 * std::cos(ph),
                     100000.0 + 20000.0 * std::sin(ph)};

        std::vector<Observation> obs;
        const bool dark = (i >= 40 && i < 52);
        if (!dark) {
            obs.emplace_back("o" + std::to_string(i), t, p, Modality::SIGINT, 0.85,
                             "AIS");
        }
        pmbm.predict();
        pmbm.update(obs, t);

        if (pmbm.dormant_count() > 0) went_dormant = true;
        const auto conf = pmbm.confirmed();
        if (i == 39 && !conf.empty()) before = conf.front()->id();
        if (i == 60 && !conf.empty()) after = conf.front()->id();
    }

    std::printf("  blackout: identity %s -> %s (dormant engaged: %s)\n",
                before.c_str(), after.c_str(), went_dormant ? "yes" : "no");
    CHECK(!before.empty());
    CHECK(!after.empty());
    CHECK(went_dormant);
    CHECK(before == after);
}

void test_overlapping_sensors_do_not_spawn_duplicates() {
    // Two sensors covering the same ground both report the same entity in the
    // same scan. That second report is corroboration, not a second entity.
    //
    // With exclusivity enforced globally rather than per sensor, it was left
    // unassigned and founded a duplicate track on every scan - which is what
    // multi-source fusion does for a living, so it is the case that has to
    // work.
    const DomainProfile profile = UrbanHUMINT();
    int clean_runs = 0;
    std::size_t worst = 0;

    for (int seed = 0; seed < 9; ++seed) {
        PmbmManager pmbm(profile, Area{-2000, 2000, -2000, 2000},
                         static_cast<std::uint64_t>(seed));
        Rng rng(static_cast<std::uint64_t>(seed) * 17 + 3);
        std::size_t peak = 0;
        for (int i = 0; i < 40; ++i) {
            const Real t = i * profile.scan_dt_s;
            const Vec2 truth{-500.0 + i * 40.0, 100.0};
            std::vector<Observation> obs;
            obs.emplace_back("a" + std::to_string(i), t,
                             Vec2{truth.x + rng.normal(0, 5), truth.y + rng.normal(0, 5)},
                             Modality::GEOINT, 0.9, "SENSOR_A");
            obs.emplace_back("b" + std::to_string(i), t,
                             Vec2{truth.x + rng.normal(0, 5), truth.y + rng.normal(0, 5)},
                             Modality::GEOINT, 0.9, "SENSOR_B");
            pmbm.predict();
            pmbm.update(obs, t);
            if (i > 5) peak = std::max(peak, pmbm.confirmed().size());
        }
        if (peak == 1) ++clean_runs;
        worst = std::max(worst, peak);
    }
    std::printf("  one entity, two overlapping sensors: exactly one track in "
                "%d of 9 seeds (worst %zu)\n", clean_runs, worst);

    // Before the per-source fix this was one duplicate per scan, every seed.
    CHECK(clean_runs >= 7);
    CHECK(worst <= 2);
}

void test_two_entities_one_sensor_stay_separate() {
    // The converse guard: per-sensor exclusivity must not let one sensor's two
    // detections collapse onto a single track.
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-2000, 2000, -2000, 2000}, 9);

    std::size_t peak = 0;
    Rng rng(4);
    for (int i = 0; i < 40; ++i) {
        const Real t = i * profile.scan_dt_s;
        std::vector<Observation> obs;
        for (int k = 0; k < 2; ++k) {
            const Vec2 p{-500.0 + i * 40.0, 100.0 + k * 600.0};
            obs.emplace_back("o" + std::to_string(i) + "_" + std::to_string(k), t,
                             Vec2{p.x + rng.normal(0, 5), p.y + rng.normal(0, 5)},
                             Modality::GEOINT, 0.9, "SENSOR_A");
        }
        pmbm.predict();
        pmbm.update(obs, t);
        if (i > 5) peak = std::max(peak, pmbm.confirmed().size());
    }
    std::printf("  two entities, one sensor: peak confirmed tracks %zu "
                "(should be 2)\n", peak);
    CHECK(peak == 2);
}

}  // namespace

void test_reacquisition_is_one_to_one() {
    // Reacquisition is a one-to-one problem - each reappearing detection is at
    // most one vanished track - and it was being solved one detection at a
    // time, each taking whichever dormant track scored best for it. That is
    // the same defect the main association had, and it fails the same way: it
    // hands one entity's identity to its neighbour.
    //
    // Four entities in well-separated lanes, all vanishing together and all
    // reappearing together. Anything other than four identities preserved
    // means the engine crossed them over.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 2.0;
    cfg.profile.meas_noise_var = 4.0;
    cfg.profile.reacquire_kinematic_s = 30.0;
    cfg.profile.mou_models = {{motion("walking", 45.0, 1.5),
                               motion("standing", 8.0, 0.10)}};
    cfg.profile.model_trans = {{{{0.92, 0.08}}, {{0.20, 0.80}}}};
    cfg.area = Area{0, 600, 0, 400};
    cfg.seed = 3;
    Engine eng(cfg);

    Rng rng(17);
    constexpr int kN = 4;
    std::vector<Vec2> truth;
    for (int i = 0; i < kN; ++i) truth.push_back(Vec2{40.0, 60.0 + 80.0 * i});

    std::map<int, std::string> before;
    std::map<int, std::string> after;
    for (int scan = 0; scan < 90; ++scan) {
        for (auto& t : truth) t.x += 1.5;
        const bool dark = scan >= 50 && scan < 62;          // 12-scan outage
        std::vector<Observation> obs;
        if (!dark) {
            for (int i = 0; i < kN; ++i) {
                obs.emplace_back("o" + std::to_string(scan) + "_" + std::to_string(i),
                                 static_cast<Real>(scan),
                                 Vec2{truth[i].x + rng.normal() * 2.0,
                                      truth[i].y + rng.normal() * 2.0},
                                 Modality::GEOINT, 0.9, "CAM");
            }
        }
        const ScanReport r = eng.ingest(obs, static_cast<Real>(scan));

        std::map<int, std::string> now;
        for (int i = 0; i < kN; ++i) {
            const TargetReport* best = nullptr;
            Real bd = 20.0;
            for (const auto& t : r.targets) {
                const Real d = distance(t.position, truth[i]);
                if (d < bd) { bd = d; best = &t; }
            }
            if (best != nullptr) now[i] = best->track_id;
        }
        if (scan == 49) before = now;
        if (scan >= 62 && scan <= 75) {
            for (const auto& [i, id] : now) {
                if (after.count(i) == 0) after[i] = id;
            }
        }
    }

    int kept = 0;
    for (const auto& [i, id] : before) {
        const auto it = after.find(i);
        if (it != after.end() && it->second == id) ++kept;
    }
    std::printf("  reacquisition across a 12-scan outage: %d of %zu identities kept\n",
                kept, before.size());
    CHECK(before.size() == kN);
    // Four lanes 80 m apart, a 12-second gap, and a velocity estimate that is
    // sound: there is no excuse for losing any of them.
    CHECK(kept == kN);
}

void test_vague_tracks_do_not_win_reacquisition() {
    // The reacquisition score was -distance/uncertainty, which tends to zero -
    // the best score available - as uncertainty grows, so the vaguest dormant
    // track won every detection it was gated for. Being uncertain is not
    // evidence. Two dormant tracks, one seen recently and one long gone: a
    // detection on top of the recent one must go to the recent one.
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 2.0;
    cfg.profile.meas_noise_var = 4.0;
    cfg.profile.reacquire_kinematic_s = 60.0;
    cfg.area = Area{0, 600, 0, 400};
    cfg.seed = 9;
    Engine eng(cfg);

    Rng rng(23);
    // A: observed for 40 scans, then stops. B: observed for 40 scans further
    // away, and goes quiet 20 scans earlier, so its prediction is far vaguer.
    std::string id_a;
    for (int scan = 0; scan < 80; ++scan) {
        std::vector<Observation> obs;
        if (scan < 60) {
            obs.emplace_back("a" + std::to_string(scan), static_cast<Real>(scan),
                             Vec2{100.0 + rng.normal() * 2.0, 100.0 + rng.normal() * 2.0},
                             Modality::GEOINT, 0.9, "CAM");
        }
        if (scan < 40) {
            obs.emplace_back("b" + std::to_string(scan), static_cast<Real>(scan),
                             Vec2{160.0 + rng.normal() * 2.0, 100.0 + rng.normal() * 2.0},
                             Modality::GEOINT, 0.9, "CAM");
        }
        const ScanReport r = eng.ingest(obs, static_cast<Real>(scan));
        if (scan == 59) {
            for (const auto& t : r.targets) {
                if (distance(t.position, Vec2{100.0, 100.0}) < 15.0) id_a = t.track_id;
            }
        }
    }

    // Now detections exactly where A was. They are A's, not the vaguer B's.
    // Scored over several scans rather than one: a revived track re-enters the
    // report at birth confidence and needs a scan or two to be confirmed, so
    // looking only at the first scan measures the reporting delay.
    std::string got;
    for (int scan = 80; scan < 90 && got.empty(); ++scan) {
        std::vector<Observation> obs{
            {"rev" + std::to_string(scan), static_cast<Real>(scan),
             Vec2{100.0, 100.0}, Modality::GEOINT, 0.9, "CAM"}};
        const ScanReport r = eng.ingest(obs, static_cast<Real>(scan));
        for (const auto& t : r.targets) {
            if (distance(t.position, Vec2{100.0, 100.0}) < 15.0) got = t.track_id;
        }
    }
    std::printf("  reacquired '%s' where '%s' vanished\n",
                got.empty() ? "(nothing)" : got.c_str(),
                id_a.empty() ? "(none)" : id_a.c_str());
    CHECK(!id_a.empty());
    CHECK(got == id_a);
}

/// Build a track, then offer it one detection displaced from the track's own
/// predicted position by `offset_m`, and report the existence that results.
///
/// The displacement has to be measured from the PREDICTION, not from a fixed
/// point in the world. A coasting track's mean drifts, so an observation held
/// at the origin is not held at a constant innovation - an earlier version of
/// this helper offset from the origin and produced existence that RISES with
/// distance, purely because the drift happened to run the same way.
Real existence_after_one_displaced_detection(Real offset_m) {
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 4242);
    auto feed = [&](Vec2 p, int i) {
        Observation o;
        o.obs_id = "o" + std::to_string(i);
        o.source_id = "CAM";
        o.timestamp = static_cast<Real>(i) * profile.scan_dt_s;
        o.position = p;
        o.modality = Modality::GEOINT;
        o.confidence = 0.9;
        pmbm.predict();
        pmbm.update({o}, o.timestamp);
    };
    for (int i = 0; i < 4; ++i) feed(Vec2{0.0, 0.0}, i);

    // One more prediction, then place the detection relative to where the
    // filter now thinks the entity is.
    pmbm.predict();
    if (pmbm.all_tracks().empty()) return -1.0;
    const Vec2 predicted = pmbm.all_tracks().front()->position();

    Observation o;
    o.obs_id = "probe";
    o.source_id = "CAM";
    o.timestamp = 4.0 * profile.scan_dt_s;
    o.position = Vec2{predicted.x + offset_m, predicted.y};
    o.modality = Modality::GEOINT;
    o.confidence = 0.9;
    pmbm.update({o}, o.timestamp);

    Real r = 0.0;
    for (const auto& t : pmbm.all_tracks()) r = std::max(r, t->existence());
    return r;
}

void test_existence_responds_to_fit() {
    // The Bernoulli/JIPDA existence update for a track that was detected is
    //
    //     r' = r p_D g(z) / ( r p_D g(z) + (1-r) lambda_c )
    //
    // and g(z) - the likelihood density of the detection under the track's own
    // innovation covariance - is the only term carrying the innovation. Without
    // it the numerator is a bare probability while the denominator is a density
    // per square metre, so the ratio is not a quantity at all; and, more
    // visibly, the posterior becomes a function of (r, p_D, clutter) alone, so
    // a detection on top of the prediction and one far out give byte-identical
    // existence.
    //
    // verification/v08 finds the counterexample against the old form and v09
    // proves what it cost: every track that got any detection cleared
    // r_confirm = 0.55 on it, and no track that did not. The threshold was
    // `n_hits >= 1` wearing a probability's clothing.
    const Real tight = existence_after_one_displaced_detection(0.0);
    const Real loose = existence_after_one_displaced_detection(600.0);

    std::printf("  existence vs fit: on-prediction=%.6f  600 m off=%.6f\n",
                tight, loose);

    CHECK(tight > 0.0);
    CHECK(loose > 0.0);
    // The point of the fix: fit reaches existence at all.
    CHECK(tight > loose);
    // And a well-fitting detection is still strong evidence, because where
    // clutter is sparse it genuinely is. The fix makes r informative; it does
    // not make it timid.
    CHECK(tight > 0.9);
}

void test_absorb_keeps_measurement_rate_a_rate() {
    // `absorb()` takes the LARGER hit-scan count of the two tracks, on the
    // reasoning that the merged track existed for the union of the two spans.
    // It did not take the larger age, so a young survivor absorbing an older
    // track inherited the older track's numerator over its own denominator.
    // `merge_duplicates` absorbs j into i in index order, which has nothing to
    // do with age, so the survivor is younger about half the time.
    //
    // That put `measurement_rate()` back above 1 - the exact defect the rate
    // was rewritten to remove - and with it the merge guard at 0.55 and the
    // group-spawn guard at 0.85, both of which read it as a fraction.
    const DomainProfile profile = UrbanHUMINT();
    const MouConstants mou = MouConstants::from(profile);

    Track young("young", profile.r_birth, profile, mou, 0.0, 11);
    Track old_t("old", profile.r_birth, profile, mou, 0.0, 12);

    // The survivor is four scans old and was seen in two of them.
    for (int s = 0; s < 4; ++s) {
        if (s > 0) young.predict();
        if (s % 2 == 0) young.note_hit_scan(s, "CAM0");
    }
    // The absorbed track is twenty-one scans old and was seen in every one.
    for (int s = 0; s < 21; ++s) {
        if (s > 0) old_t.predict();
        old_t.note_hit_scan(s, "CAM1");
    }
    CHECK(young.measurement_rate() <= 1.0 + 1e-9);
    CHECK(old_t.measurement_rate() <= 1.0 + 1e-9);
    CHECK(old_t.hit_scans() > young.age() + 1);   // the setup actually bites

    // Three more things a merge must carry, none of which it used to.
    old_t.mark_confirmed();
    const std::size_t records_before = young.hit_record_count();
    CHECK(!young.ever_confirmed());
    CHECK(old_t.hit_record_count() > 0);

    young.absorb(old_t);
    std::printf("  after absorb: age %d, hit-scans %d, rate %.3f, records %zu\n",
                young.age(), young.hit_scans(), young.measurement_rate(),
                young.hit_record_count());
    CHECK(young.measurement_rate() <= 1.0 + 1e-9);
    CHECK(young.detection_density() <= 1.0 + 1e-9);
    // The merged age is the union of the two spans, matching born_at_, which
    // absorb() already takes as the earlier of the two.
    CHECK(young.age() >= old_t.age());

    // A confirmation is a fact about the ENTITY. Leaving it behind let a merge
    // silently un-confirm an identity that prune() and the report both decide
    // by `ever_confirmed()`.
    CHECK(young.ever_confirmed());

    // And the absorbed track's hit records come too. They are what
    // shares_hit_scan_with and shares_source_with test, so discarding them
    // threw away the evidence that decides whether the NEXT merge is
    // legitimate: the survivor came out looking as though the absorbed
    // track's sensors had never fed it.
    CHECK(young.hit_record_count() > records_before);
    CHECK(young.shares_source_with(old_t));
}

/// Defects 7 and 9: a track that stops being seen must be retired on the
/// clock, and a track that was once reportable must survive that retirement as
/// an identity rather than being discarded.
///
/// Both were invisible for the same reason. Retirement depended solely on
/// existence decay, and where `p_detection` is low a miss is almost
/// uninformative - at p_d = 0.15 existence falls from 0.9990 to 0.9988 per
/// missed scan - so a track nobody had seen for an hour sat in the live set
/// with its existence intact, which is correct Bayesian behaviour and a
/// useless operational outcome. Dormancy, meanwhile, required existence to land
/// inside a band between `r_dormant` and `r_prune`, typically 0.01 wide, which
/// a decaying existence falls straight through - and two shipped profiles had
/// the two values inverted, making the band empty.
///
/// So the test has to check both halves at once: the track leaves the live set
/// on time, its existence is still high when it does (which is what makes the
/// old rule unable to retire it), and it turns up in the dormant pool rather
/// than being dropped.
void test_unseen_tracks_time_out_and_stay_recognisable() {
    DomainProfile profile = CityCameraSurveillance();
    profile.scan_dt_s = 1.0;
    profile.p_detection = 0.15;    // a miss is nearly uninformative
    profile.max_coast_s = 20.0;
    PmbmManager pmbm(profile, Area{0, 500, 0, 500}, 11);

    // Twelve scans of a walker, enough to confirm it.
    Vec2 truth{50.0, 250.0};
    Real t = 0.0;
    for (int s = 0; s < 12; ++s, t += profile.scan_dt_s) {
        pmbm.predict();
        pmbm.update({Observation{"o" + std::to_string(s), t, truth,
                                 Modality::GEOINT, 0.95, "CAM"}}, t);
        truth.x += 1.2;
    }
    CHECK(pmbm.all_tracks().size() == 1);
    if (pmbm.all_tracks().empty()) return;
    const std::string id = pmbm.all_tracks().front()->id();
    CHECK(pmbm.all_tracks().front()->ever_confirmed());

    // Now nobody reports it again. Step until it leaves the live set, watching
    // what its existence is doing while that happens.
    Real existence_at_retirement = -1.0;
    Real coasted_at_retirement = -1.0;
    const Real last_seen = t - profile.scan_dt_s;
    for (int s = 0; s < 60 && !pmbm.all_tracks().empty(); ++s, t += profile.scan_dt_s) {
        pmbm.predict();
        // Read it before the update, because the update is what prunes: after
        // the scan that retires the track there is nothing left to ask.
        const Real before = pmbm.all_tracks().front()->existence();
        pmbm.update({}, t);
        if (pmbm.all_tracks().empty()) {
            existence_at_retirement = before;
            coasted_at_retirement = t - last_seen;
        }
    }

    std::printf("  unseen track left the live set after %.0f s of coasting "
                "(limit %.0f), existence still %.4f against r_prune %.2f; "
                "dormant pool holds %zu\n",
                coasted_at_retirement, profile.max_coast_s,
                existence_at_retirement, profile.r_prune, pmbm.dormant_count());

    // Retired, and on the clock rather than on the evidence.
    CHECK(pmbm.all_tracks().empty());
    CHECK(coasted_at_retirement > 0.0);
    CHECK(coasted_at_retirement <= profile.max_coast_s + 2.0 * profile.scan_dt_s);
    // The half that makes the timeout necessary: existence had barely moved,
    // so an existence-only rule would still be holding this track.
    CHECK(existence_at_retirement > profile.r_prune);
    CHECK(existence_at_retirement > 0.5);
    // And it is remembered rather than discarded.
    CHECK(pmbm.dormant_count() == 1);
    CHECK(pmbm.known_ids().count(id) == 1);
}

/// Defect 9, the other half: dormancy must be reachable in every shipped
/// profile. It was unreachable in two of them, and the consequence is quiet -
/// reacquisition never fires, every reappearance becomes a new track, and the
/// cost shows up as identity switches a long way from the cause.
void test_every_profile_can_go_dormant() {
    for (const auto& name : profile_names()) {
        DomainProfile profile = profile_by_name(name);
        const Real dt = profile.scan_dt_s;
        PmbmManager pmbm(profile, Area{-50000, 50000, -50000, 50000}, 3);

        Vec2 truth{0.0, 0.0};
        const Real step = profile.courier_speed_thresh * dt;
        Real t = 0.0;
        for (int s = 0; s < 14; ++s, t += dt) {
            pmbm.predict();
            pmbm.update({Observation{"o" + std::to_string(s), t, truth,
                                     Modality::GEOINT, 0.95, "CAM"}}, t);
            // update() prunes
            truth.x += step;
        }
        const bool confirmed = !pmbm.all_tracks().empty() &&
                               pmbm.all_tracks().front()->ever_confirmed();
        CHECK(confirmed);

        // Then silence, for long enough that no profile can still be coasting.
        const Real limit = profile.max_coast_s > 0.0
                               ? profile.max_coast_s
                               : profile.dormant_timeout * dt;
        const int quiet_scans =
            static_cast<int>(limit / std::max(dt, 1e-9)) + 40;
        for (int s = 0; s < quiet_scans && !pmbm.all_tracks().empty(); ++s, t += dt) {
            pmbm.predict();
            pmbm.update({}, t);
            // update() prunes
        }

        CHECK(pmbm.all_tracks().empty());
        CHECK(pmbm.dormant_count() >= 1);
        if (pmbm.dormant_count() == 0) {
            std::printf("  %s: confirmed track vanished instead of going dormant\n",
                        name.c_str());
        }
    }
    std::printf("  all %zu profiles retire a confirmed track into the dormant "
                "pool rather than discarding it\n", profile_names().size());
}

void test_source_trust_does_not_move_evidence_quality() {
    // Two axes that look alike and are not: how good a detection is, and how
    // much the sensor that produced it has earned. `possibility()` is the
    // first; SourceCredibility is the second.
    //
    // They were conflated by multiplying credibility into an observation's
    // confidence before handing it to update_hit, which meant the possibility
    // measure - documented as "the normalised quality of a track's evidence" -
    // silently became "quality times trust". Credibility is a relative score
    // whose mid-range is normal for a healthy sensor: two sound cameras in the
    // `spoofing` scenario sit at about 0.45. So every real track's pi_r sat
    // near 0.45 against an r of 0.9999, and possibility_mismatch, whose whole
    // purpose is to separate weak evidence from strong, fired on 476 of 496
    // real-entity scans - scoring real entities (0.66) as more suspicious than
    // a deliberately high-confidence phantom (0.57).
    //
    // The engine-level test of that diagnostic used a single source, and
    // SourceCredibility returns 1.0 when only one source has ever reported, so
    // it never saw the discount. This one states the contract directly.
    const DomainProfile profile = CityCameraSurveillance();
    const MouConstants mou = MouConstants::from(profile);

    struct Outcome { Real possibility; Real uncertainty; Real quality; };
    const auto run = [&](Real trust) {
        Track t("t", profile.r_birth, profile, mou, 0.0, 7);
        t.filter().init(Vec2{0.0, 0.0});   // as PmbmManager does on birth
        // Identical observations for both runs, noise included: the only thing
        // that differs between them is the trust argument.
        Rng rng(5);
        for (int s = 0; s < 20; ++s) {
            if (s > 0) t.predict();
            const Vec2 truth{static_cast<Real>(s) * 2.0, 0.0};
            const Observation o{"o" + std::to_string(s),
                                static_cast<Real>(s) * profile.scan_dt_s,
                                Vec2{truth.x + rng.normal(0.0, profile.pos_noise_m),
                                     truth.y + rng.normal(0.0, profile.pos_noise_m)},
                                Modality::GEOINT, 0.95, "CAM"};
            t.update_hit(o, profile.scan_dt_s, trust);
            t.note_hit_scan(s, "CAM");
        }
        return Outcome{t.possibility(), t.position_uncertainty(),
                       t.mean_observation_quality()};
    };

    const Outcome trusted = run(1.0);
    const Outcome doubted = run(0.30);
    std::printf("  trust 1.00 -> possibility %.3f, sigma %.6f m; "
                "trust 0.30 -> possibility %.3f, sigma %.6f m\n",
                trusted.possibility, trusted.uncertainty,
                doubted.possibility, doubted.uncertainty);

    // Evidence quality is what the sensor asserted. Distrusting the sensor does
    // not make its imagery lower-resolution.
    CHECK_NEAR(doubted.possibility, trusted.possibility, 1e-9);
    CHECK_NEAR(doubted.quality, trusted.quality, 1e-9);
    // GEOINT at 0.95 normalised against the best modality weight: high, and
    // nowhere near the 0.4 mismatch threshold once r saturates.
    CHECK(trusted.possibility > 0.85);

    // And the filter still weighs a doubted source less, which is the entire
    // point of having a trust score. Measured as how far one detection drags
    // the estimate: trust scales the assumed measurement variance, so a
    // doubted report flattens the likelihood and moves the cloud less.
    // (Spread is the wrong probe for this - a flatter likelihood also
    // resamples less often, and resampling is itself a source of jitter, so
    // the two effects partly cancel. Displacement does not have that problem.)
    const auto pull = [&](Real trust) {
        Track t("t", profile.r_birth, profile, mou, 0.0, 7);
        t.filter().init(Vec2{0.0, 0.0});
        const Vec2 before = t.position();
        const Observation o{"o", profile.scan_dt_s, Vec2{40.0, 0.0},
                            Modality::GEOINT, 0.95, "CAM"};
        t.update_hit(o, profile.scan_dt_s, trust);
        return t.position().x - before.x;
    };
    const Real pull_trusted = pull(1.0);
    const Real pull_doubted = pull(0.30);
    std::printf("  one 40 m detection pulls the estimate %.2f m at trust 1.00, "
                "%.2f m at trust 0.30\n", pull_trusted, pull_doubted);
    CHECK(pull_trusted > pull_doubted);
}

void test_measurement_rate_is_a_rate() {
    // `measurement_rate()` is the fraction of scans a track has existed for in
    // which it was detected, so it cannot exceed 1. Two things used to let it.
    //
    // `update_hit` runs once per observation in a scan's group, so a track
    // under four overlapping sensors scored four hits against one scan of age.
    // And the denominator was `age_`, which is zero on the scan a track is
    // born in even though the track was detected in that scan - counting the
    // birth scan in the numerator and not the denominator.
    //
    // Together they put a perfectly-detected four-sensor track at 8.0. Both
    // thresholds tested against this - the group-spawn test at 0.85 and the
    // merge test at 0.55 - were then true for any track with more than one
    // sensor on it, whatever its detection history.
    const DomainProfile profile = UrbanHUMINT();
    for (int n_sensors : {1, 2, 4}) {
        PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 5);
        Real worst = 0.0;
        Real last = 0.0;
        for (int i = 0; i < 20; ++i) {
            std::vector<Observation> obs;
            for (int s = 0; s < n_sensors; ++s) {
                obs.push_back(Observation{
                    "o" + std::to_string(i) + "_" + std::to_string(s),
                    static_cast<Real>(i) * profile.scan_dt_s, Vec2{0.0, 0.0},
                    Modality::GEOINT, 0.9, "CAM" + std::to_string(s)});
            }
            pmbm.predict();
            pmbm.update(obs, static_cast<Real>(i) * profile.scan_dt_s);
            for (const auto& t : pmbm.all_tracks()) {
                worst = std::max(worst, t->measurement_rate());
                last = t->measurement_rate();
                CHECK(t->detection_density() <= 1.0);
            }
        }
        std::printf("  measurement_rate with %d sensor(s): max %.3f, final %.3f\n",
                    n_sensors, worst, last);
        CHECK(worst <= 1.0 + 1e-9);
        // Detected in every scan it existed for, so the rate is exactly 1
        // however many sensors were doing the detecting.
        CHECK(std::abs(last - 1.0) < 1e-9);
    }
}

int main() {
    test_overlapping_sensors_do_not_spawn_duplicates();
    test_two_entities_one_sensor_stay_separate();
    test_tracks_clean_targets();
    test_identity_survives_a_blackout();
    test_survives_detection_gap();
    test_clutter_estimate_adapts();
    test_ids_are_stable();
    test_reacquisition_is_one_to_one();
    test_vague_tracks_do_not_win_reacquisition();
    test_existence_responds_to_fit();
    test_unseen_tracks_time_out_and_stay_recognisable();
    test_every_profile_can_go_dormant();
    test_source_trust_does_not_move_evidence_quality();
    test_measurement_rate_is_a_rate();
    test_absorb_keeps_measurement_rate_a_rate();
    return trace::test::summary("test_pmbm");
}
