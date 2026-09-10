// TRACE — maze / camera-grid simulation driver.
//
// Builds a maze, lays a grid of cameras over it, walks travellers through, and
// shows what the engine makes of it — side by side with the truth it never sees.
//
//   ./trace_maze --width 21 --height 11 --panels 4x3 --travellers 3 --blind 2
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "trace/core/engine.hpp"
#include "trace/sim/maze.hpp"
#include "trace/sim/scenario.hpp"

using namespace trace;
using namespace trace::sim;

namespace {

struct Options {
    int width{21};
    int height{11};
    int panel_cols{4};
    int panel_rows{3};
    int travellers{3};
    int blind_panels{2};
    int scans{120};
    Real cell_size{8.0};
    Real p_detect{0.82};
    Real swap_probability{0.10};
    Real loop_fraction{0.10};
    std::uint64_t seed{20260910};
    int delay_ms{110};
    bool animate{true};
    bool colour{true};
    bool quiet{false};
};

void print_usage() {
    std::puts(
        "TRACE maze/camera simulation\n"
        "\n"
        "  --width N          maze width in cells        (default 21)\n"
        "  --height N         maze height in cells       (default 11)\n"
        "  --panels CxR       camera grid, e.g. 4x3      (default 4x3)\n"
        "  --travellers N     entities to track          (default 3)\n"
        "  --blind N          panels switched off        (default 2)\n"
        "  --scans N          scans to run               (default 120)\n"
        "  --cell M           cell size in metres        (default 8)\n"
        "  --pd P             camera detection prob      (default 0.82)\n"
        "  --swap P           per-panel id-confusion prob(default 0.10)\n"
        "  --loops F          extra-wall fraction        (default 0.10)\n"
        "  --seed N           rng seed                   (default 20260910)\n"
        "  --delay MS         frame delay                (default 110)\n"
        "  --no-animate       print only the final frame\n"
        "  --no-color         plain ASCII output\n"
        "  --quiet            metrics only, no frames\n");
}

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&](Real& dst) {
            if (i + 1 < argc) dst = std::atof(argv[++i]);
        };
        const auto next_i = [&](int& dst) {
            if (i + 1 < argc) dst = std::atoi(argv[++i]);
        };

        if (a == "--help" || a == "-h") { print_usage(); return false; }
        else if (a == "--width") next_i(o.width);
        else if (a == "--height") next_i(o.height);
        else if (a == "--travellers") next_i(o.travellers);
        else if (a == "--blind") next_i(o.blind_panels);
        else if (a == "--scans") next_i(o.scans);
        else if (a == "--delay") next_i(o.delay_ms);
        else if (a == "--cell") next(o.cell_size);
        else if (a == "--pd") next(o.p_detect);
        else if (a == "--swap") next(o.swap_probability);
        else if (a == "--loops") next(o.loop_fraction);
        else if (a == "--seed") { if (i + 1 < argc) o.seed = std::strtoull(argv[++i], nullptr, 10); }
        else if (a == "--no-animate") o.animate = false;
        else if (a == "--no-color" || a == "--no-colour") o.colour = false;
        else if (a == "--quiet") { o.quiet = true; o.animate = false; }
        else if (a == "--panels") {
            if (i + 1 < argc) {
                const std::string spec = argv[++i];
                const auto x = spec.find('x');
                if (x != std::string::npos) {
                    o.panel_cols = std::atoi(spec.substr(0, x).c_str());
                    o.panel_rows = std::atoi(spec.substr(x + 1).c_str());
                }
            }
        } else {
            std::printf("unknown option: %s\n\n", a.c_str());
            print_usage();
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return 0;

    Rng rng(opt.seed);

    // ---- Build the maze ---------------------------------------------------
    Maze maze(opt.width, opt.height, opt.cell_size);
    maze.generate(rng);
    maze.add_loops(rng, opt.loop_fraction);

    // ---- Lay the camera estate over it ------------------------------------
    auto panel_configs = CameraGrid::partition(maze, opt.panel_cols, opt.panel_rows,
                                               opt.p_detect, opt.cell_size * 0.15);
    for (auto& p : panel_configs) p.swap_probability = opt.swap_probability;

    // Switch some panels off: these become blind corridors where the engine
    // has to hold identity on prediction alone and reacquire on the far side.
    for (int i = 0; i < opt.blind_panels && !panel_configs.empty(); ++i) {
        const auto idx = static_cast<std::size_t>(
            rng.uniform_int(0, static_cast<int>(panel_configs.size()) - 1));
        panel_configs[idx].enabled = false;
    }

    // ---- Scenario ---------------------------------------------------------
    Scenario scenario(opt.seed);
    scenario.name = "maze-camera-grid";
    scenario.n_scans = opt.scans;
    scenario.match_radius_m = opt.cell_size * 1.5;

    DomainProfile profile = CityCameraSurveillance();
    profile.scan_dt_s = 1.0;   // one scan per simulated second
    profile.pos_noise_m = opt.cell_size * 0.15;
    profile.meas_noise_var = profile.pos_noise_m * profile.pos_noise_m * 4.0;
    profile.p_detection = opt.p_detect;
    profile.rv_threshold_m = opt.cell_size * 1.2;
    profile.coloc_dist_m = opt.cell_size * 2.0;
    profile.chokepoint_m = opt.cell_size;
    profile.brush_pass_m = opt.cell_size * 0.8;
    profile.parallel_route_m = opt.cell_size * 1.5;

    scenario.engine_config.profile = profile;
    scenario.engine_config.area = maze.bounds();
    scenario.engine_config.seed = opt.seed;

    for (auto& cfg : panel_configs) {
        scenario.sensors.push_back(std::make_unique<CameraPanel>(cfg));
    }

    // ---- Travellers -------------------------------------------------------
    // Each walks a shortest path between two random cells, then picks a new
    // destination. Continuous motion through a constrained topology is what
    // makes this a real test rather than a straight-line demo.
    for (int i = 0; i < opt.travellers; ++i) {
        Entity e;
        e.id = "traveller_" + std::to_string(i);
        const Cell start = maze.random_cell(rng);
        const Cell goal = maze.random_cell(rng);
        e.position = maze.centre_of(start);
        e.goal = maze.centre_of(goal);
        e.waypoints = maze.waypoints(maze.path(start, goal));
        e.velocity = Vec2{1.3 + 0.25 * i, 0.0};  // distinct walking speeds
        e.mode = "walking";
        e.role = (i == 0 ? "subject" : "civilian");
        scenario.world.add(std::move(e));
    }

    // Re-task any traveller that has arrived, so the sim never runs dry.
    scenario.on_scan = [&maze, &rng](Scenario& s, int) {
        for (auto& e : s.world.entities()) {
            if (e.waypoint_index < e.waypoints.size() || e.dwell_remaining_s > 0.0) {
                continue;
            }
            const Cell from = maze.cell_at(e.position);
            const Cell to = maze.random_cell(rng);
            const auto cells = maze.path(from, to);
            if (cells.size() < 2) continue;
            e.waypoints = maze.waypoints(cells);
            e.waypoint_index = 1;
            // A short pause at the destination gives the loiter and
            // pattern-of-life machinery something to chew on.
            e.dwell_remaining_s = rng.uniform(0.0, 6.0);
        }
    };

    // ---- Render -----------------------------------------------------------
    MazeRenderer renderer;
    renderer.colour = opt.colour;

    Engine engine(scenario.engine_config);

    scenario.on_report = [&](const Scenario& s, int scan, const ScanReport& r,
                             const Metrics& m) {
        if (opt.quiet) return;
        if (!opt.animate && scan != s.n_scans - 1) return;

        if (opt.animate) std::printf("\033[H\033[J");  // home + clear

        std::printf("TRACE  maze %dx%d  cameras %dx%d (%d off)  travellers %d\n",
                    maze.width(), maze.height(), opt.panel_cols, opt.panel_rows,
                    opt.blind_panels, opt.travellers);
        std::printf("scan %3d/%d   t=%.0fs   tracks=%d   dormant=%d   %.2f ms\n\n",
                    scan + 1, s.n_scans, r.timestamp, r.n_tracks, r.n_dormant,
                    r.latency_ms);

        std::fputs(renderer.render(maze, panel_configs, s.world.entities(),
                                   r.targets).c_str(), stdout);
        std::fputs(renderer.legend(panel_configs, s.world.entities()).c_str(),
                   stdout);

        std::printf("\n  detection %.0f%%   pos err %.2f m   id switches %d   ghosts %d\n",
                    100.0 * m.detection_rate(), m.mean_position_error(),
                    m.id_switches, m.ghost_tracks);

        if (!r.rendezvous.empty()) {
            const auto& w = r.rendezvous.front();
            std::printf("  next convergence: %s<->%s in %.1f s (%s, conf %.2f)\n",
                        w.track_a.c_str(), w.track_b.c_str(), w.eta_s,
                        w.method.c_str(), w.confidence);
        }
        if (!r.events.empty()) {
            std::printf("  events:");
            for (std::size_t i = 0; i < std::min<std::size_t>(r.events.size(), 4); ++i) {
                std::printf(" %s", r.events[i].type.c_str());
            }
            std::printf("\n");
        }

        if (opt.animate && opt.delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(opt.delay_ms));
        }
    };

    const Metrics m = run(scenario, engine);

    std::printf("\n== TRACE maze simulation complete ==\n");
    std::printf("  maze          %dx%d cells @ %.1f m\n", maze.width(), maze.height(),
                maze.cell_size());
    std::printf("  cameras       %zu panels, %d disabled, p_detect=%.2f, swap=%.2f\n",
                panel_configs.size(), opt.blind_panels, opt.p_detect,
                opt.swap_probability);
    std::fputs(m.summary().c_str(), stdout);
    std::fputs(engine.performance_report().c_str(), stdout);
    return 0;
}
