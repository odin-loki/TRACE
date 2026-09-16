// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// Guards the engine's cost profile.
//
// This exists because the profile was wrong in a way nobody would notice from
// correctness tests: the convergence detector recomputed each track's
// pattern-of-life forecast inside its pair loop, so every track's forecast was
// rebuilt once per other track. At 270 tracks that was 95% of the engine's
// runtime and made overall cost grow as n^1.8. Everything still worked; it was
// just unusable at scale.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "trace/core/engine.hpp"
#include "test_harness.hpp"

using namespace trace;

namespace {

struct Point {
    int tracks{0};
    Real median_ms{0.0};
};

Point measure(int n_entities, int scans) {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 1.5;
    cfg.profile.meas_noise_var = 9.0;
    cfg.profile.p_detection = 0.88;
    // Density held constant, so this measures scaling in track count rather
    // than the separate effect of packing entities closer together.
    const Real span = 200.0 * std::sqrt(static_cast<Real>(n_entities) / 10.0);
    cfg.area = Area{0, span, 0, span};
    cfg.seed = 4242;
    Engine engine(cfg);

    Rng rng(99);
    std::vector<Vec2> pos(static_cast<std::size_t>(n_entities));
    std::vector<Vec2> vel(static_cast<std::size_t>(n_entities));
    for (int i = 0; i < n_entities; ++i) {
        pos[static_cast<std::size_t>(i)] = Vec2{rng.uniform(0, span), rng.uniform(0, span)};
        vel[static_cast<std::size_t>(i)] = Vec2{rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4)};
    }

    std::vector<Real> latencies;
    int peak = 0;
    for (int s = 0; s < scans; ++s) {
        const Real t = s * 1.0;
        std::vector<Observation> obs;
        for (int i = 0; i < n_entities; ++i) {
            auto& p = pos[static_cast<std::size_t>(i)];
            auto& v = vel[static_cast<std::size_t>(i)];
            p += v;
            if (p.x < 0 || p.x > span) v.x = -v.x;
            if (p.y < 0 || p.y > span) v.y = -v.y;
            if (!rng.bernoulli(0.88)) continue;
            obs.emplace_back("o" + std::to_string(s) + "_" + std::to_string(i), t,
                             Vec2{p.x + rng.normal(0, 1.5), p.y + rng.normal(0, 1.5)},
                             Modality::GEOINT, 0.9, "CAM");
        }
        const ScanReport r = engine.ingest(obs, t);
        peak = std::max(peak, r.n_tracks);
        if (s >= 4) latencies.push_back(r.latency_ms);
    }
    std::sort(latencies.begin(), latencies.end());
    return Point{peak, latencies.empty() ? 0.0 : latencies[latencies.size() / 2]};
}

void test_cost_is_not_quadratic() {
    const Point small = measure(20, 14);
    const Point large = measure(160, 14);

    CHECK(small.tracks > 0);
    CHECK(large.tracks > small.tracks);
    CHECK(small.median_ms > 0.0);

    // cost ~ n^k  =>  k = log(ratio of cost) / log(ratio of tracks)
    const Real k = std::log(large.median_ms / small.median_ms) /
                   std::log(static_cast<Real>(large.tracks) / small.tracks);
    std::printf("  %d tracks %.2f ms -> %d tracks %.2f ms : cost ~ n^%.2f\n",
                small.tracks, small.median_ms, large.tracks, large.median_ms, k);

    // Generous, because timing on a shared machine is noisy - but far below
    // the n^1.8 the engine exhibited before the forecast was hoisted out of
    // the pair loop, and nowhere near the n^2 it would reach if a future
    // change reintroduced per-pair recomputation.
    CHECK(k < 1.55);
}

/// Peak track count when `n` entities walk about for `scans` scans, with the
/// cap set to `cap`.
///
/// The entities have to be real. An earlier version of the test below scattered
/// 120 independent uniform points per scan and asserted the peak was at or
/// below a cap of 25. Points with no continuity between scans are clutter, the
/// engine confirmed almost none of them, and the peak was **3** - so the
/// assertion held for a reason that had nothing to do with the cap, and would
/// have held just as well with the cap removed.
int peak_tracks_under_cap(int n, int cap, int scans) {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 1.5;
    cfg.profile.meas_noise_var = 9.0;
    cfg.profile.max_tracks = cap;
    cfg.area = Area{0, 2000, 0, 2000};
    cfg.seed = 7;
    Engine engine(cfg);

    Rng rng(5);
    std::vector<Vec2> pos(static_cast<std::size_t>(n));
    std::vector<Vec2> vel(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        pos[static_cast<std::size_t>(i)] = Vec2{rng.uniform(0, 2000), rng.uniform(0, 2000)};
        vel[static_cast<std::size_t>(i)] = Vec2{rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4)};
    }

    int peak = 0;
    for (int s = 0; s < scans; ++s) {
        const Real t = s * 1.0;
        std::vector<Observation> obs;
        for (int i = 0; i < n; ++i) {
            auto& p = pos[static_cast<std::size_t>(i)];
            auto& v = vel[static_cast<std::size_t>(i)];
            p += v;
            if (p.x < 0 || p.x > 2000) v.x = -v.x;
            if (p.y < 0 || p.y > 2000) v.y = -v.y;
            obs.emplace_back("o" + std::to_string(s) + "_" + std::to_string(i), t,
                             Vec2{p.x + rng.normal(0, 1.5), p.y + rng.normal(0, 1.5)},
                             Modality::GEOINT, 0.9, "CAM");
        }
        peak = std::max(peak, engine.ingest(obs, t).n_tracks);
    }
    return peak;
}

void test_track_cap_is_honoured_and_configurable() {
    // The cap was a hardcoded 80 with nothing to say so, which silently
    // discarded tracks in any genuinely crowded scene.
    const int uncapped = peak_tracks_under_cap(120, 1000, 25);
    const int capped = peak_tracks_under_cap(120, 25, 25);
    std::printf("  120 entities: %d tracks uncapped, %d with the cap at 25\n",
                uncapped, capped);

    // The setup has to bite, or the assertion below means nothing.
    CHECK(uncapped > 25);
    CHECK(capped <= 25);
    // And the cap has to be the reason, not a coincidence of the scene.
    CHECK(capped < uncapped);

    // The default must not be so low that a crowd is silently truncated.
    CHECK(CityCameraSurveillance().max_tracks >= 200);
}

}  // namespace

int main() {
    test_cost_is_not_quadratic();
    test_track_cap_is_honoured_and_configurable();
    return trace::test::summary("test_scaling");
}
