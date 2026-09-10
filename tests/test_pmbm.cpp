// End-to-end check on the tracking core: can it hold identity on moving
// targets, in clutter, through missed detections?
#include "trace/core/pmbm.hpp"

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
    for (std::size_t i = 0; i < sc.scans.size(); ++i) {
        pmbm.predict();
        pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
        const auto conf = pmbm.confirmed();
        if (i == 11) before = conf.size();
        if (i == 19) during = conf.size();
        if (i == 30) after = conf.size();
    }
    std::printf("  gap: before=%zu during-blackout=%zu after=%zu\n",
                before, during, after);
    CHECK(before >= 2);
    CHECK(after >= 2);  // reacquired after the blackout
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

int main() {
    test_overlapping_sensors_do_not_spawn_duplicates();
    test_two_entities_one_sensor_stay_separate();
    test_tracks_clean_targets();
    test_identity_survives_a_blackout();
    test_survives_detection_gap();
    test_clutter_estimate_adapts();
    test_ids_are_stable();
    return trace::test::summary("test_pmbm");
}
