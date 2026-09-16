// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — scalability benchmark.
//
// The engine's cost is dominated by three things that grow differently with the
// number of simultaneous entities:
//
//   per-track   particle propagation and the pattern-of-life fit, linear in
//               tracks and independent of how many others there are;
//   pairwise    convergence prediction and co-location clustering, quadratic
//               in principle, sparse in practice because both are gated;
//   assignment  Gibbs sweeps over the track-by-detection matrix, gated the
//               same way.
//
// Which of those dominates at a realistic track count is a measurement, not an
// argument, and it decides whether the engine is usable for stadium egress or
// city-scale camera estates. This measures it.
//
//   ./trace_bench                 # sweep 10 -> 400 entities
//   ./trace_bench --max 1000      # push until it hurts
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "trace/core/engine.hpp"

using namespace trace;

namespace {

struct Result {
    std::vector<std::pair<std::string, Real>> stages;
    int entities{0};
    int peak_tracks{0};
    Real median_ms{0.0};
    Real p95_ms{0.0};
    Real per_track_us{0.0};
};

// The profile's own track cap, raised so a sweep can go past it. Without this
// every point above CityCameraSurveillance's 400 measures the same 400 tracks
// and the curve flattens for a reason that has nothing to do with cost.
int g_track_cap = 0;

Result measure(int n_entities, int scans, bool detectors_on) {
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    if (g_track_cap > 0) cfg.profile.max_tracks = g_track_cap;
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 1.5;
    cfg.profile.meas_noise_var = 9.0;
    cfg.profile.p_detection = 0.88;

    // Area grows with the crowd so density stays constant: this measures
    // scaling in track count, not the separate effect of packing entities
    // closer together until association becomes ambiguous.
    const Real span = 200.0 * std::sqrt(static_cast<Real>(n_entities) / 10.0);
    cfg.area = Area{0, span, 0, span};
    cfg.seed = 4242;

    Engine engine(cfg);
    if (!detectors_on) {
        for (const auto& name : engine.detector_names()) {
            engine.unregister_detector(name);
        }
    }

    Rng rng(99);
    std::vector<Vec2> pos(static_cast<std::size_t>(n_entities));
    std::vector<Vec2> vel(static_cast<std::size_t>(n_entities));
    for (int i = 0; i < n_entities; ++i) {
        pos[static_cast<std::size_t>(i)] = Vec2{rng.uniform(0, span), rng.uniform(0, span)};
        vel[static_cast<std::size_t>(i)] = Vec2{rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4)};
    }

    std::vector<Real> latencies;
    latencies.reserve(static_cast<std::size_t>(scans));
    int peak = 0;
    std::vector<std::pair<std::string, Real>> stage_totals;

    for (int s = 0; s < scans; ++s) {
        const Real t = s * 1.0;
        std::vector<Observation> obs;
        obs.reserve(static_cast<std::size_t>(n_entities));
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
        if (s >= 5) {
            latencies.push_back(r.latency_ms);   // let it warm up
            if (stage_totals.empty()) stage_totals = r.stage_ms;
            else {
                for (std::size_t k = 0; k < stage_totals.size() && k < r.stage_ms.size(); ++k) {
                    stage_totals[k].second += r.stage_ms[k].second;
                }
            }
        }
    }

    std::sort(latencies.begin(), latencies.end());
    Result out;
    out.entities = n_entities;
    out.peak_tracks = peak;
    out.median_ms = latencies.empty() ? 0.0 : latencies[latencies.size() / 2];
    out.p95_ms = latencies.empty()
                     ? 0.0
                     : latencies[std::min(latencies.size() - 1,
                                          static_cast<std::size_t>(latencies.size() * 0.95))];
    out.per_track_us = peak > 0 ? out.median_ms * 1000.0 / peak : 0.0;
    const auto n = static_cast<Real>(std::max<std::size_t>(latencies.size(), 1));
    for (auto& [name, total] : stage_totals) out.stages.emplace_back(name, total / n);
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    int max_entities = 400;
    int scans = 40;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--max") == 0 && i + 1 < argc) {
            max_entities = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--scans") == 0 && i + 1 < argc) {
            scans = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-tracks") == 0 && i + 1 < argc) {
            g_track_cap = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::puts("usage: trace_bench [--max N] [--scans N] [--max-tracks N]");
            return 0;
        }
    }

    if (g_track_cap <= 0) g_track_cap = std::max(max_entities, 400);
    std::printf("TRACE scalability  (%s, %d scans per point, density held "
                "constant, track cap %d)\n\n",
                simd::backend_name(), scans, g_track_cap);
    std::printf("  %8s %8s %11s %10s %14s   %s\n", "entities", "tracks", "median ms",
                "p95 ms", "us per track", "detectors");
    std::printf("  %s\n", std::string(74, '-').c_str());

    // The sweep points, ending exactly on --max.
    //
    // The geometric walk alone overshoots and then stops early: from 270 the
    // next point is 405, so `--max 400` - the default - measured 270 and
    // called it done, while the README published a 400-track row nobody could
    // reproduce. `--max N` now means the sweep ends at N.
    std::vector<int> sizes;
    for (int n = 10; n <= max_entities; n = (n < 50 ? n * 2 : n * 3 / 2)) {
        sizes.push_back(n);
    }
    if (sizes.empty() || sizes.back() != max_entities) sizes.push_back(max_entities);

    std::vector<Result> full;
    for (int n : sizes) {
        const Result on = measure(n, scans, true);
        const Result off = measure(n, scans, false);
        full.push_back(on);
        std::printf("  %8d %8d %11.2f %10.2f %14.1f   on\n", on.entities,
                    on.peak_tracks, on.median_ms, on.p95_ms, on.per_track_us);
        std::printf("  %8s %8d %11.2f %10.2f %14.1f   off (tracking only)\n", "",
                    off.peak_tracks, off.median_ms, off.p95_ms, off.per_track_us);
    }

    // Where does the time actually go at the largest size measured?
    if (!full.empty() && !full.back().stages.empty()) {
        std::printf("\n  stage breakdown at %d tracks (mean ms per scan):\n",
                    full.back().peak_tracks);
        auto stages = full.back().stages;
        std::sort(stages.begin(), stages.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        for (const auto& [name, ms] : stages) {
            if (ms < 0.05) continue;
            std::printf("    %-24s %8.2f ms  %5.1f%%\n", name.c_str(), ms,
                        100.0 * ms / full.back().median_ms);
        }
    }

    // Fit the exponent: if cost ~ n^k, then log(cost) is linear in log(n) with
    // slope k. Anything near 1 is linear; near 2 means the pairwise stages
    // dominate and city scale is out of reach without spatial partitioning.
    if (full.size() >= 2) {
        const auto& a = full.front();
        const auto& b = full.back();
        if (a.median_ms > 0.0 && a.peak_tracks > 0 && b.peak_tracks > 0) {
            const Real k = std::log(b.median_ms / a.median_ms) /
                           std::log(static_cast<Real>(b.peak_tracks) / a.peak_tracks);
            std::printf("\n  cost scales as about n^%.2f over %d..%d tracks%s\n", k,
                        a.peak_tracks, b.peak_tracks,
                        k < 1.35 ? "  (effectively linear)"
                                 : (k > 1.7 ? "  (approaching quadratic)" : ""));
        }
    }
    return 0;
}
