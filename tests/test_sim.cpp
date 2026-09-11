// The simulation layer is what every performance claim in this repository rests
// on, so its own correctness matters: if the sensors leak truth, or the scoring
// mis-assigns tracks, every number downstream is meaningless.
#include "trace/sim/maze.hpp"
#include "trace/sim/scenario.hpp"

#include <algorithm>
#include <cstdio>
#include <set>

#include "test_harness.hpp"

using namespace trace;
using namespace trace::sim;

namespace {

void test_maze_is_connected() {
    // A maze with unreachable cells would silently strand travellers and make
    // detection rates look bad for the wrong reason.
    Rng rng(1234);
    Maze maze(21, 13, 8.0);
    maze.generate(rng);

    const Cell origin{0, 0};
    int reachable = 0;
    for (int r = 0; r < maze.height(); ++r) {
        for (int c = 0; c < maze.width(); ++c) {
            if (!maze.path(origin, Cell{c, r}).empty()) ++reachable;
        }
    }
    std::printf("  maze: %d of %d cells reachable\n", reachable,
                maze.width() * maze.height());
    CHECK(reachable == maze.width() * maze.height());
}

void test_maze_paths_respect_walls() {
    Rng rng(99);
    Maze maze(15, 15, 5.0);
    maze.generate(rng);
    maze.add_loops(rng, 0.1);

    const auto path = maze.path(Cell{0, 0}, Cell{14, 14});
    CHECK(path.size() >= 2);
    for (std::size_t i = 1; i < path.size(); ++i) {
        // Every step must be to an adjacent cell with no wall between.
        const int dc = std::abs(path[i].col - path[i - 1].col);
        const int dr = std::abs(path[i].row - path[i - 1].row);
        CHECK(dc + dr == 1);
        CHECK(!maze.wall_between(path[i - 1], path[i]));
    }
}

void test_maze_coordinate_roundtrip() {
    Maze maze(10, 8, 12.0);
    for (int r = 0; r < maze.height(); ++r) {
        for (int c = 0; c < maze.width(); ++c) {
            const Cell back = maze.cell_at(maze.centre_of(Cell{c, r}));
            CHECK(back.col == c);
            CHECK(back.row == r);
        }
    }
}

void test_camera_only_sees_its_footprint() {
    // The central guarantee of the whole simulation layer: a sensor reports
    // only what it can actually see, and never leaks ground truth.
    CameraPanel::Config cfg;
    cfg.id = "CAM";
    cfg.footprint = Area{0, 100, 0, 100};
    cfg.p_detect = 1.0;
    cfg.pos_noise_m = 0.0;
    cfg.false_alarm_rate = 0.0;
    CameraPanel cam(cfg);

    WorldSnapshot truth;
    truth.timestamp = 0.0;
    Entity inside;
    inside.id = "in";
    inside.position = Vec2{50, 50};
    Entity outside;
    outside.id = "out";
    outside.position = Vec2{500, 500};
    truth.entities = {inside, outside};

    Rng rng(7);
    const auto obs = cam.observe(truth, rng);
    CHECK(obs.size() == 1);
    if (!obs.empty()) {
        CHECK(distance(*obs[0].position, Vec2{50, 50}) < 1e-9);
        // Nothing in an Observation may identify which entity produced it.
        CHECK(obs[0].obs_id.find("in") == std::string::npos ||
              obs[0].obs_id.rfind("CAM", 0) == 0);
        CHECK(obs[0].source_id == "CAM");
    }
    CHECK(!cam.covers(Vec2{500, 500}));
    CHECK(cam.covers(Vec2{50, 50}));
}

void test_disabled_sensor_is_silent() {
    CameraPanel::Config cfg;
    cfg.footprint = Area{0, 100, 0, 100};
    cfg.p_detect = 1.0;
    cfg.false_alarm_rate = 5.0;
    cfg.enabled = false;
    CameraPanel cam(cfg);

    WorldSnapshot truth;
    Entity e;
    e.position = Vec2{50, 50};
    truth.entities = {e};
    Rng rng(3);
    CHECK(cam.observe(truth, rng).empty());
    CHECK(!cam.covers(Vec2{50, 50}));
}

void test_wide_area_silencing() {
    // The dark-vessel mechanism: one entity stops being reported while others
    // continue normally.
    WideAreaReporter::Config cfg;
    cfg.footprint = Area{0, 1000, 0, 1000};
    cfg.p_detect = 1.0;
    cfg.pos_noise_m = 0.0;
    cfg.false_alarm_rate = 0.0;
    WideAreaReporter rep(cfg);

    WorldSnapshot truth;
    Entity a; a.id = "a"; a.position = Vec2{100, 100};
    Entity b; b.id = "b"; b.position = Vec2{200, 200};
    truth.entities = {a, b};

    Rng rng(5);
    CHECK(rep.observe(truth, rng).size() == 2);
    rep.silenced.insert("a");
    CHECK(rep.observe(truth, rng).size() == 1);
    rep.silenced.clear();
    CHECK(rep.observe(truth, rng).size() == 2);
}

void test_world_follows_waypoints_without_overshoot() {
    World world(1);
    Entity e;
    e.id = "walker";
    e.position = Vec2{0, 0};
    e.velocity = Vec2{2.0, 0.0};   // 2 m/s
    e.waypoints = {Vec2{0, 0}, Vec2{10, 0}, Vec2{10, 10}};
    e.waypoint_index = 1;
    world.add(std::move(e));

    for (int i = 0; i < 20; ++i) world.step(1.0);
    const auto& w = world.entities()[0];
    // It must turn the corner rather than cutting through: the corner is the
    // whole point of walking a maze.
    CHECK(w.position.x <= 10.0 + 1e-6);
    CHECK(w.position.y >= 0.0);
    std::printf("  waypoint walker ended at (%.1f, %.1f) after 20 s\n",
                w.position.x, w.position.y);
}

void test_world_survives_repeated_waypoints() {
    // Concatenating two path segments repeats the shared endpoint, which is how
    // every route in the scenario suite is built. A repeated waypoint used to
    // zero the entity's velocity - Vec2::unit() of a zero delta is {0,0} - after
    // which the step function fell through to a hardcoded 1.4 m/s walking pace.
    // In a domain whose units are not metres that default is arbitrary, and at
    // an hourly scan period it came to thousands of units per step: the entity
    // tore through its whole route and then stood still for the rest of the run.
    World world(1);
    Entity e;
    e.id = "shuttle";
    e.position = Vec2{0, 0};
    e.velocity = Vec2{2.0, 0.0};                  // 2 m/s
    e.waypoints = {Vec2{0, 0},   Vec2{20, 0},     // segment one
                   Vec2{20, 0},  Vec2{0, 0},      // segment two, repeated join
                   Vec2{0, 0},   Vec2{20, 0}};
    world.add(std::move(e));

    // The route is 60 m at 2 m/s, so 25 steps leaves it still under way.
    Vec2 previous = world.entities()[0].position;
    Real max_step = 0.0;
    for (int i = 0; i < 25; ++i) {
        world.step(1.0);
        const Vec2 now = world.entities()[0].position;
        max_step = std::max(max_step, distance(previous, now));
        previous = now;
    }
    // Never further in one second than 2 m/s allows, whatever the route does.
    std::printf("  repeated-waypoint shuttle: largest step %.3f m (budget 2.0)\n",
                max_step);
    CHECK(max_step <= 2.0 + 1e-6);
    // Still moving, rather than having torn through the route and stopped.
    CHECK(world.entities()[0].velocity.norm() > 1e-6);
    // And it is past the repeated join rather than stuck on it: 50 m of a
    // 60 m route in 25 seconds puts it on the third leg.
    std::printf("  repeated-waypoint shuttle: at waypoint %zu after 25 s\n",
                world.entities()[0].waypoint_index);
    CHECK(world.entities()[0].waypoint_index >= 4);
}

void test_world_travels_at_its_configured_speed() {
    // Reaching a waypoint mid-scan used to cost the remainder of that scan, so
    // a route with many short legs moved an entity far slower than its own
    // velocity said. Over a long route the two must agree closely.
    World world(1);
    Entity e;
    e.id = "runner";
    e.position = Vec2{0, 0};
    e.velocity = Vec2{3.0, 0.0};                  // 3 m/s
    for (int i = 1; i <= 40; ++i) {               // 40 legs of 5 m
        e.waypoints.push_back(Vec2{static_cast<Real>(5 * i), 0.0});
    }
    world.add(std::move(e));

    for (int i = 0; i < 50; ++i) world.step(1.0);
    const Real travelled = world.entities()[0].position.x;
    std::printf("  runner covered %.1f m in 50 s at 3 m/s (expected 150)\n",
                travelled);
    CHECK(travelled > 149.0);
    CHECK(travelled <= 150.0 + 1e-6);
}

void test_scoring_counts_switches_and_ghosts() {
    Metrics m;
    std::vector<Entity> truth(1);
    truth[0].id = "e0";
    truth[0].position = Vec2{0, 0};

    TargetReport t1;
    t1.track_id = "T0001";
    t1.position = Vec2{1, 0};
    score_scan(m, truth, {t1}, 10.0);
    CHECK(m.total_detected == 1);
    CHECK(m.id_switches == 0);

    // Same entity, different track id -> one identity switch.
    TargetReport t2;
    t2.track_id = "T0002";
    t2.position = Vec2{1, 0};
    score_scan(m, truth, {t2}, 10.0);
    CHECK(m.id_switches == 1);

    // A track nowhere near any truth entity is a ghost.
    TargetReport far;
    far.track_id = "T0003";
    far.position = Vec2{900, 900};
    const int before = m.ghost_tracks;
    score_scan(m, truth, {t2, far}, 10.0);
    CHECK(m.ghost_tracks == before + 1);

    // One track cannot cover two entities.
    Metrics m2;
    std::vector<Entity> two(2);
    two[0].id = "a"; two[0].position = Vec2{0, 0};
    two[1].id = "b"; two[1].position = Vec2{2, 0};
    TargetReport only;
    only.track_id = "T1";
    only.position = Vec2{1, 0};
    score_scan(m2, two, {only}, 10.0);
    CHECK(m2.total_truth == 2);
    CHECK(m2.total_detected == 1);
}

void test_end_to_end_maze_run() {
    // The claim on the front page, asserted rather than described.
    Rng rng(20260910);
    Maze maze(15, 9, 8.0);
    maze.generate(rng);
    maze.add_loops(rng, 0.10);

    auto panels = CameraGrid::partition(maze, 3, 3, 0.82, 1.2);
    for (auto& p : panels) p.swap_probability = 0.10;
    panels[4].enabled = false;

    Scenario s(20260910);
    s.n_scans = 60;
    s.match_radius_m = 12.0;

    DomainProfile p = CityCameraSurveillance();
    p.scan_dt_s = 1.0;
    p.pos_noise_m = 1.2;
    p.meas_noise_var = 1.2 * 1.2 * 4.0;
    p.p_detection = 0.82;
    p.rv_threshold_m = 9.6;
    p.coloc_dist_m = 16.0;
    p.brush_pass_m = 6.4;
    s.engine_config.profile = p;
    s.engine_config.area = maze.bounds();
    s.engine_config.seed = 20260910;

    for (const auto& c : panels) s.sensors.push_back(std::make_unique<CameraPanel>(c));
    for (int i = 0; i < 3; ++i) {
        Entity e;
        e.id = "traveller_" + std::to_string(i);
        const Cell start = maze.random_cell(rng);
        const Cell goal = maze.random_cell(rng);
        e.position = maze.centre_of(start);
        e.waypoints = maze.waypoints(maze.path(start, goal));
        e.velocity = Vec2{1.3 + 0.25 * i, 0.0};
        s.world.add(std::move(e));
    }
    Maze* mp = &maze;
    Rng* rp = &rng;
    s.on_scan = [mp, rp](Scenario& sc, int) {
        for (auto& e : sc.world.entities()) {
            if (e.waypoint_index < e.waypoints.size() || e.dwell_remaining_s > 0.0) {
                continue;
            }
            const auto cells = mp->path(mp->cell_at(e.position), mp->random_cell(*rp));
            if (cells.size() < 2) continue;
            e.waypoints = mp->waypoints(cells);
            e.waypoint_index = 1;
            e.dwell_remaining_s = rp->uniform(0.0, 6.0);
        }
    };

    Engine eng(s.engine_config);
    const Metrics m = run(s, eng);
    std::printf("  maze end-to-end: detection %.0f%%, err %.2f m, switches %d, "
                "ghosts %.2f/scan, median %.2f ms\n",
                100.0 * m.detection_rate(), m.mean_position_error(), m.id_switches,
                static_cast<Real>(m.ghost_tracks) / m.scans, m.median_latency_ms());

    CHECK(m.detection_rate() > 0.70);
    CHECK(m.mean_position_error() < 4.0);

    // Identity stability, expressed per entity per 100 scans so the bar means
    // the same thing whatever the scenario length. Before association was made
    // one-to-one this figure was ~29; anything above 5 means duplicate tracks
    // are being maintained and swapped between.
    const Real switches_per_entity_per_100 =
        static_cast<Real>(m.id_switches) * 100.0 /
        (static_cast<Real>(m.current_assignment.size()) * m.scans);
    std::printf("  identity: %.1f switches per entity per 100 scans\n",
                switches_per_entity_per_100);
    CHECK(switches_per_entity_per_100 < 5.0);

    CHECK(static_cast<Real>(m.ghost_tracks) / m.scans < 0.5);

#ifdef NDEBUG
    // Only meaningful in an optimised build - a Debug or sanitiser build is an
    // order of magnitude slower and asserting a wall-clock bound there would
    // fail for reasons that say nothing about the engine.
    CHECK(m.median_latency_ms() < 10.0);
#endif
}

}  // namespace

int main() {
    test_maze_is_connected();
    test_maze_paths_respect_walls();
    test_maze_coordinate_roundtrip();
    test_camera_only_sees_its_footprint();
    test_disabled_sensor_is_silent();
    test_wide_area_silencing();
    test_world_follows_waypoints_without_overshoot();
    test_world_survives_repeated_waypoints();
    test_world_travels_at_its_configured_speed();
    test_scoring_counts_switches_and_ghosts();
    test_end_to_end_maze_run();
    return trace::test::summary("test_sim");
}
