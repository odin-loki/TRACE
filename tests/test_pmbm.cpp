// End-to-end check on the tracking core: can it hold identity on moving
// targets, in clutter, through missed detections?
#include "trace/core/pmbm.hpp"

#include <cstdio>
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
                    "CAM_" + std::to_string(rng.uniform_int(0, 3)));
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
    const DomainProfile profile = UrbanHUMINT();
    PmbmManager pmbm(profile, Area{-5000, 5000, -5000, 5000}, 42);
    const auto sc = make_scenario(40, 5, 3000.0, 0.90, 2.0, 7);

    Real err_sum = 0.0;
    int err_n = 0;
    std::size_t peak = 0;
    for (std::size_t i = 0; i < sc.scans.size(); ++i) {
        pmbm.predict();
        pmbm.update(sc.scans[i], static_cast<Real>(i) * 60.0);
        const auto conf = pmbm.confirmed();
        peak = std::max(peak, conf.size());
        if (i >= 10) {  // allow the filter to settle
            err_sum += mean_nearest_error(conf, sc.truth[i]);
            ++err_n;
        }
    }
    const Real mean_err = err_sum / std::max(err_n, 1);
    std::printf("  clean: peak tracks=%zu (truth 5), mean err=%.1f m\n", peak, mean_err);

    CHECK(peak >= 4);          // found essentially all of them
    CHECK(peak <= 12);         // without spawning a swarm of ghosts
    CHECK(mean_err < 120.0);   // and localised them
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

}  // namespace

int main() {
    test_tracks_clean_targets();
    test_survives_detection_gap();
    test_clutter_estimate_adapts();
    test_ids_are_stable();
    return trace::test::summary("test_pmbm");
}
