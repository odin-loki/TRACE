// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// The detector layer had no direct tests at all. Everything under
// src/detectors/ was covered only through whole-scenario runs, which say
// whether the engine as a whole still produces the same summary and nothing
// about whether a given detector can fire, or fires twice when it should.
#include "trace/detectors/detectors.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "trace/core/particle_filter.hpp"
#include "trace/core/profile.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/spatial_index.hpp"
#include "trace/core/track.hpp"

#include "test_harness.hpp"

using namespace trace;

namespace {

TrackPtr make_track(const std::string& id, const DomainProfile& p,
                    const MouConstants& mou, Vec2 at, std::uint64_t seed) {
    auto t = std::make_shared<Track>(id, p.r_birth, p, mou, 0.0, seed);
    t->filter().init(at);
    return t;
}

int count(const std::vector<DetectionEvent>& events, const char* type) {
    int n = 0;
    for (const auto& e : events) {
        if (e.type == type) ++n;
    }
    return n;
}

void test_brush_pass_reports_every_meeting() {
    // BRUSH_PASS reports the transition INTO contact, counted by a streak so
    // that two people walking together do not raise an event every scan. The
    // streak was cleared in the `else` of the near-pair loop, which only looks
    // at pairs within twice the contact radius - so a pair that separated
    // FURTHER than that in one scan was never visited, kept its streak at 1,
    // and on coming back together scored 2 and was refused.
    //
    // Whether a pair can clear twice the contact radius in a scan is a
    // property of the domain: CityCameraSurveillance's contact radius is 10 m,
    // and this test moves the pair 400 m apart, which a vehicle does in
    // seconds.
    const DomainProfile p = CityCameraSurveillance();
    const MouConstants mou = MouConstants::from(p);
    Rng rng(9);

    TradecraftDetector det;
    DetectorContext ctx;
    ctx.profile = &p;
    ctx.rng = &rng;

    // The spatial index is what makes near_pairs actually near. Without one
    // DetectorContext falls back to every pair regardless of radius, and the
    // separated scan below then reaches the detector after all - which is why
    // the first version of this test passed against the defect it was written
    // for. A test of a radius gate has to supply the thing that does the
    // gating.
    auto run = [&](Vec2 pa, Vec2 pb, Real ts, int scan) {
        auto a = make_track("A", p, mou, pa, 1);
        auto b = make_track("B", p, mou, pb, 2);
        const std::vector<Vec2> pts{a->position(), b->position()};
        const SpatialIndex index(pts, p.brush_pass_m * 2.0);
        ctx.index = &index;
        ctx.timestamp = ts;
        ctx.scan_index = scan;
        return count(det.detect({a, b}, ctx), "BRUSH_PASS");
    };

    int meetings = 0;
    for (int meeting = 0; meeting < 3; ++meeting) {
        // Together: well inside the contact radius.
        meetings += run(Vec2{0.0, 0.0}, Vec2{1.0, 0.0}, meeting * 100.0, meeting * 2);
        // Apart: 400 m, far beyond twice the 10 m contact radius, in one step,
        // so the pair is not a near pair and the detector never looks at it.
        (void)run(Vec2{0.0, 0.0}, Vec2{400.0, 0.0}, meeting * 100.0 + 50.0,
                  meeting * 2 + 1);
    }
    std::printf("  brush pass: %d event(s) over 3 separated meetings\n", meetings);
    CHECK(meetings == 3);
}

void test_brush_pass_does_not_repeat_while_together() {
    // The other half of the same contract: entities that stay in contact must
    // raise exactly one event, not one per scan.
    const DomainProfile p = CityCameraSurveillance();
    const MouConstants mou = MouConstants::from(p);
    Rng rng(9);

    TradecraftDetector det;
    DetectorContext ctx;
    ctx.profile = &p;
    ctx.rng = &rng;

    int events = 0;
    for (int scan = 0; scan < 8; ++scan) {
        auto a = make_track("A", p, mou, Vec2{0.0, 0.0}, 1);
        auto b = make_track("B", p, mou, Vec2{1.0, 0.0}, 2);
        const std::vector<Vec2> pts{a->position(), b->position()};
        const SpatialIndex index(pts, p.brush_pass_m * 2.0);
        ctx.index = &index;
        ctx.timestamp = scan * 10.0;
        ctx.scan_index = scan;
        events += count(det.detect({a, b}, ctx), "BRUSH_PASS");
    }
    std::printf("  brush pass: %d event(s) over 8 scans in continuous contact\n", events);
    CHECK(events == 1);
}

void test_parallel_route_window_is_satisfiable() {
    // ParallelRouteDetector needs brush_pass_m < separation <= parallel_route_m.
    // VehicleConvoy had these the wrong way round - 20 and 15 - so no
    // separation could satisfy both and the detector was dead under that
    // profile. WarehouseAssets set the first to 1.5 m and inherited 80 m for
    // the second from the urban preset, in a facility whose coloc_dist_m is 4.
    //
    // Checked for every shipped profile rather than the two that were wrong.
    const DomainProfile profiles[] = {
        UrbanHUMINT(),      Maritime(),       Airspace(),
        VehicleConvoy(),    CityCameraSurveillance(), CounterTerrorism(),
        OrganisedCrimeNetwork(), FugitiveTracking(),  BorderPatrol(),
        IndoorVenue(),      WarehouseAssets(), WildlifeTelemetry(),
        SportsPitch()};
    for (const DomainProfile& p : profiles) {
        if (p.parallel_route_m <= p.brush_pass_m) {
            std::printf("  %-24s EMPTY WINDOW: brush_pass %.1f >= parallel_route %.1f\n",
                        p.name.c_str(), p.brush_pass_m, p.parallel_route_m);
        }
        CHECK(p.parallel_route_m > p.brush_pass_m);
        // And wide enough to survive the profile's own position noise: a
        // separation carries noise of pos_noise_m * sqrt(2) from two tracks.
        const Real sep_noise = p.pos_noise_m * 1.41421356;
        if (p.parallel_route_m - p.brush_pass_m < sep_noise) {
            std::printf("  %-24s narrow: window %.1f m against %.1f m of separation noise\n",
                        p.name.c_str(), p.parallel_route_m - p.brush_pass_m, sep_noise);
        }
    }
}

}  // namespace

int main() {
    test_brush_pass_reports_every_meeting();
    test_brush_pass_does_not_repeat_while_together();
    test_parallel_route_window_is_satisfiable();
    return trace::test::summary("test_detectors");
}
