// A constrained motion model is only useful if it confines the estimate without
// distorting it, and declines to confine things that are genuinely off-network.
#include "trace/core/motion_constraint.hpp"

#include <cstdio>

#include "trace/core/particle_filter.hpp"
#include "test_harness.hpp"

using namespace trace;

namespace {

void test_projection_onto_a_line() {
    const RoadNetwork road = RoadNetwork::from_polyline({{0, 0}, {100, 0}}, 30.0);

    // A point beside the road snaps onto it.
    const Vec2 p = road.project(Vec2{50, 10});
    CHECK_NEAR(p.x, 50.0, 1e-9);
    CHECK_NEAR(p.y, 0.0, 1e-9);

    // A point beyond the tolerance is left alone: asserting it is on the road
    // would be claiming something we do not know.
    const Vec2 far = road.project(Vec2{50, 200});
    CHECK_NEAR(far.y, 200.0, 1e-9);

    // Past the end of a segment the nearest point is the endpoint - it is a
    // segment, not an infinite line - provided the point is within tolerance.
    const Vec2 beyond = road.project(Vec2{120, 5});
    CHECK_NEAR(beyond.x, 100.0, 1e-9);
    CHECK_NEAR(beyond.y, 0.0, 1e-9);

    // Further past the end than the tolerance allows, it is left alone.
    const Vec2 way_beyond = road.project(Vec2{140, 5});
    CHECK_NEAR(way_beyond.x, 140.0, 1e-9);

    CHECK_NEAR(road.distance_to(Vec2{50, 10}), 10.0, 1e-9);
}

void test_velocity_alignment_preserves_direction() {
    const RoadNetwork road = RoadNetwork::from_polyline({{0, 0}, {100, 0}}, 30.0);

    // Across-road motion is discarded, along-road motion kept.
    const Vec2 v = road.align(Vec2{50, 0}, Vec2{10.0, 7.0});
    CHECK_NEAR(v.x, 10.0, 1e-9);
    CHECK_NEAR(v.y, 0.0, 1e-9);

    // Travelling the other way must stay travelling the other way.
    const Vec2 back = road.align(Vec2{50, 0}, Vec2{-10.0, 7.0});
    CHECK_NEAR(back.x, -10.0, 1e-9);
}

void test_grid_construction() {
    const RoadNetwork g = RoadNetwork::grid(Area{0, 300, 0, 200}, 3, 2, 10.0);
    CHECK(g.segments().size() == 4 + 3);   // 4 verticals, 3 horizontals
    CHECK_NEAR(g.distance_to(Vec2{100, 100}), 0.0, 1e-9);  // on a junction line
    CHECK(g.distance_to(Vec2{50, 50}) > 0.0);              // inside a block
}

void test_constrained_filter_keeps_particles_on_the_road() {
    // The point of the whole exercise: a filter coasting between sparse
    // observations should stay on the carriageway instead of spreading
    // sideways into the verge. Tested at the filter directly, because that is
    // where the constraint acts.
    auto road = std::make_shared<RoadNetwork>(
        RoadNetwork::from_polyline({{0, 500}, {4000, 500}}, 60.0));

    DomainProfile p = VehicleConvoy();
    p.scan_dt_s = 2.0;
    const MouConstants mou = MouConstants::from(p);

    const auto worst_lateral = [&](MotionConstraintPtr c) {
        ParticleFilter pf(p, mou, 11, std::move(c));
        pf.init(Vec2{100.0, 500.0}, 5.0);
        Real worst = 0.0;
        for (int i = 0; i < 30; ++i) {
            pf.predict();  // coasting, no measurements at all
            worst = std::max(worst, std::abs(pf.position().y - 500.0));
            for (std::size_t k = 0; k < pf.ys().size(); ++k) {
                worst = std::max(worst, std::abs(pf.ys()[k] - 500.0));
            }
        }
        return worst;
    };

    const Real free_drift = worst_lateral(nullptr);
    const Real road_drift = worst_lateral(road);
    std::printf("  worst lateral spread over 30 coasting scans: "
                "free space %.0f m, road-constrained %.0f m\n",
                free_drift, road_drift);

    // Unconstrained, the cloud spreads across the map; constrained, it cannot
    // leave the road's tolerance.
    CHECK(free_drift > 200.0);
    CHECK(road_drift <= 60.0 + 1e-6);
    CHECK(road_drift < free_drift * 0.5);
}

}  // namespace

int main() {
    test_projection_onto_a_line();
    test_velocity_alignment_preserves_direction();
    test_grid_construction();
    test_constrained_filter_keeps_particles_on_the_road();
    return trace::test::summary("test_motion_constraint");
}
