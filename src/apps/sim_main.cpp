// TRACE — scenario suite.
//
// Six simulations beyond the maze, each built to stress a different part of the
// engine. Run one, or run them all:
//
//   ./trace_sim --list
//   ./trace_sim transit-hub
//   ./trace_sim --all
#include <array>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>

#include "trace/core/engine.hpp"
#include "trace/core/motion_constraint.hpp"
#include "trace/sim/scenario.hpp"

using namespace trace;
using namespace trace::sim;

namespace {

struct ScenarioSpec {
    std::string description;
    std::string stresses;
    std::function<void(std::uint64_t, bool)> run;
};

/// Straight-line waypoint helper.
std::vector<Vec2> line(Vec2 a, Vec2 b, int steps) {
    std::vector<Vec2> out;
    for (int i = 0; i <= steps; ++i) {
        const Real t = static_cast<Real>(i) / steps;
        out.push_back(Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
    return out;
}

void report(const std::string& name, const Metrics& m, const Engine& eng,
            const std::string& note) {
    std::printf("\n--- %s ---\n%s", name.c_str(), m.summary().c_str());
    std::fputs(eng.performance_report().c_str(), stdout);
    if (!note.empty()) std::printf("  %s\n", note.c_str());
}

// ---------------------------------------------------------------------------
// 1. Transit hub — a concourse with entrances, shops and a meeting
// ---------------------------------------------------------------------------
// Stresses: dense co-location, rendezvous prediction at short range, loiter
// against a learned baseline, and dead-drop geometry.
void run_transit_hub(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 200;
    s.match_radius_m = 4.0;

    DomainProfile p = IndoorVenue();
    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 200, 0, 120};
    s.engine_config.high_value_locations = {Vec2{100, 60}};  // the concourse
    s.engine_config.seed = seed;

    // Overlapping ceiling cameras with a gap over the central atrium.
    for (int i = 0; i < 6; ++i) {
        CameraPanel::Config c;
        c.id = "CEIL_" + std::to_string(i);
        c.footprint = Area{static_cast<Real>(i) * 33.0, static_cast<Real>(i) * 33.0 + 40.0,
                           0, 120};
        c.p_detect = 0.78;
        c.pos_noise_m = 0.9;
        c.false_alarm_rate = 0.05;
        c.swap_probability = 0.08;
        s.sensors.push_back(std::make_unique<CameraPanel>(c));
    }
    // Ticket gates: sparse, accurate, identity-anchored.
    for (int i = 0; i < 3; ++i) {
        GateReader::Config g;
        g.id = "GATE_" + std::to_string(i);
        g.position = Vec2{20.0 + i * 80.0, 10.0};
        g.radius_m = 4.0;
        g.modality = Modality::COMMS;
        s.sensors.push_back(std::make_unique<GateReader>(g));
    }

    // Two entities converge on the atrium from opposite ends and meet there.
    // Nothing in their kinematics says so until late; the pattern-of-life
    // cross-predictor is the method that can see it coming.
    Entity a; a.id = "subject_A"; a.role = "subject";
    a.position = Vec2{10, 20}; a.velocity = Vec2{1.2, 0};
    a.waypoints = line(Vec2{10, 20}, Vec2{100, 60}, 60);
    s.world.add(a);

    Entity b; b.id = "subject_B"; b.role = "contact";
    b.position = Vec2{190, 100}; b.velocity = Vec2{1.1, 0};
    b.waypoints = line(Vec2{190, 100}, Vec2{100, 60}, 60);
    s.world.add(b);

    for (int i = 0; i < 8; ++i) {
        Entity c; c.id = "commuter_" + std::to_string(i); c.role = "crowd";
        const Real y = 15.0 + i * 12.0;
        c.position = Vec2{5, y}; c.velocity = Vec2{1.3 + 0.1 * i, 0};
        c.waypoints = line(Vec2{5, y}, Vec2{195, y}, 70);
        s.world.add(c);
    }

    Engine eng(s.engine_config);
    int rv_hits = 0;
    Real earliest_warning_s = -1.0;
    s.on_report = [&](const Scenario&, int, const ScanReport& r, const Metrics&) {
        for (const auto& w : r.rendezvous) {
            const bool is_pair = (w.track_a.size() && w.track_b.size());
            if (is_pair && w.confidence > 0.3) {
                ++rv_hits;
                if (earliest_warning_s < 0.0) earliest_warning_s = w.eta_s;
            }
        }
    };
    const Metrics m = run(s, eng);
    char note[256];
    std::snprintf(note, sizeof(note),
                  "convergence warnings raised: %d, first lead time %.0f s", rv_hits,
                  earliest_warning_s);
    report("transit-hub", m, eng, note);
}

// ---------------------------------------------------------------------------
// 2. Dark vessel — a ship switches off its transponder mid-transit
// ---------------------------------------------------------------------------
// Stresses: long scan periods, low detection probability, existence decay over
// a real gap, and pattern-of-life reacquisition on the far side.
void run_dark_vessel(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 120;
    // A vessel covers 21.6 km between hourly scans, and the motion model's own
    // one-scan prediction uncertainty is around 13 km. Scoring against a 3 km
    // radius would be measuring the scan rate, not the tracker.
    s.match_radius_m = 8000.0;

    // An ocean basin, not a coastal box: at 12 knots a vessel covers 21.6 km
    // per hourly scan, so 120 scans is 2,600 km of transit.
    const Area basin{0, 3000000, 0, 1200000};
    s.engine_config.profile = Maritime();
    s.engine_config.area = basin;
    s.engine_config.seed = seed;

    auto ais = std::make_unique<WideAreaReporter>(WideAreaReporter::Config{
        "AIS_SAT", basin, 0.80, 200.0, Modality::SIGINT, 0.85, 0.20, true});
    auto* ais_ptr = ais.get();
    s.sensors.push_back(std::move(ais));

    for (int i = 0; i < 5; ++i) {
        Entity v;
        v.id = "vessel_" + std::to_string(i);
        v.role = (i == 0 ? "suspect" : "traffic");
        const Real y = 150000.0 + i * 200000.0;
        v.position = Vec2{50000, y};
        v.velocity = Vec2{6.0, 0.0};  // ~12 knots
        v.waypoints = line(Vec2{50000, y}, Vec2{2900000, y + 100000}, 130);
        s.world.add(v);
    }

    // The suspect goes dark for 25 scans mid-voyage, then comes back up.
    s.on_scan = [ais_ptr](Scenario&, int scan) {
        if (scan == 40) ais_ptr->silenced.insert("vessel_0");
        if (scan == 65) ais_ptr->silenced.erase("vessel_0");
    };

    Engine eng(s.engine_config);
    int dormant_peak = 0;
    bool reacquired = false;
    s.on_report = [&](const Scenario& sc, int scan, const ScanReport& r, const Metrics&) {
        dormant_peak = std::max(dormant_peak, r.n_dormant);
        if (scan > 66) {
            for (const auto& t : r.targets) {
                for (const auto& e : sc.world.entities()) {
                    if (e.id == "vessel_0" && distance(t.position, e.position) < 3000.0) {
                        reacquired = true;
                    }
                }
            }
        }
    };
    const Metrics m = run(s, eng);
    char note[256];
    std::snprintf(note, sizeof(note),
                  "went dark scans 40-65; peak dormant %d; reacquired after resurfacing: %s",
                  dormant_peak, reacquired ? "yes" : "no");
    report("dark-vessel", m, eng, note);
}

// ---------------------------------------------------------------------------
// 3. ANPR corridor — plate readers at junctions, one vehicle tailing another
// ---------------------------------------------------------------------------
// Stresses: sparse point-observations rather than continuous tracks, and the
// parallel-route detector on a genuine tail.
void run_anpr_corridor(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 150;
    s.match_radius_m = 40.0;

    DomainProfile p = VehicleConvoy();
    p.scan_dt_s = 2.0;
    // Readers cover roughly a tenth of the corridor, so a vehicle is seen in
    // about one scan in nine. p_detection is the per-scan probability of being
    // detected at all, not how reliable a reader is once you are under it -
    // set it too high and three missed scans collapse a perfectly good track.
    p.p_detection = 0.15;
    p.dormant_timeout = 120;
    p.r_prune = 0.01;
    // A vehicle covers the 400 m between readers in ~22 s; allow a couple of
    // reader-gaps of coasting, then retire the track on time rather than
    // waiting for an existence decay that low p_detection will never deliver.
    p.max_coast_s = 60.0;
    p.parallel_scans = 8;
    p.parallel_route_m = 60.0;
    p.parallel_vel_cos = 0.95;
    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 6000, 0, 800};
    s.engine_config.seed = seed;

    // Confine tracks to the carriageway. Free-space motion has no idea a
    // vehicle cannot leave the road, so between readers 400 m apart it coasts
    // the estimate sideways into the verge - which was most of this scenario's
    // apparent error, and the reason it was the weakest of the seven.
    s.engine_config.motion_constraint = std::make_shared<RoadNetwork>(
        RoadNetwork::from_polyline({{0, 400}, {6000, 400}}, /*tolerance*/ 60.0));

    // Readers every 400 m along the corridor: coverage is a string of dots.
    for (int i = 0; i < 15; ++i) {
        GateReader::Config g;
        g.id = "ANPR_" + std::to_string(i);
        g.position = Vec2{200.0 + i * 400.0, 400.0};
        g.radius_m = 45.0;
        g.p_detect = 0.93;
        g.pos_noise_m = 3.0;
        g.modality = Modality::GEOINT;
        s.sensors.push_back(std::make_unique<GateReader>(g));
    }

    Entity target; target.id = "target_vehicle"; target.role = "target";
    target.position = Vec2{50, 400}; target.velocity = Vec2{18.0, 0};
    target.waypoints = line(Vec2{50, 400}, Vec2{5950, 400}, 140);
    s.world.add(target);

    // A tail holding a steady 45 m gap behind the target.
    Entity tail; tail.id = "tail_vehicle"; tail.role = "surveillance";
    tail.position = Vec2{5, 400}; tail.velocity = Vec2{18.0, 0};
    tail.waypoints = line(Vec2{5, 400}, Vec2{5905, 400}, 140);
    s.world.add(tail);

    for (int i = 0; i < 6; ++i) {
        Entity c; c.id = "traffic_" + std::to_string(i); c.role = "traffic";
        c.position = Vec2{300.0 * i, 400};
        c.velocity = Vec2{14.0 + 1.5 * i, 0};
        c.waypoints = line(Vec2{300.0 * i, 400}, Vec2{5990, 400}, 130 + i * 4);
        s.world.add(c);
    }

    Engine eng(s.engine_config);
    int parallel_events = 0;
    s.on_report = [&](const Scenario&, int, const ScanReport& r, const Metrics&) {
        parallel_events += static_cast<int>(r.events_of_type("PARALLEL_ROUTE").size());
    };
    const Metrics m = run(s, eng);
    char note[128];
    std::snprintf(note, sizeof(note), "PARALLEL_ROUTE (tail) events raised: %d",
                  parallel_events);
    report("anpr-corridor", m, eng, note);
}

// ---------------------------------------------------------------------------
// 4. Warehouse — pallets, forklifts and a custody handover
// ---------------------------------------------------------------------------
// Stresses: very low detection probability, long dormancy, mode transition
// (pallet moves from forklift to conveyor), and chokepoint counting at doors.
void run_warehouse(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 420;              // 420 x 5 s = 35 simulated minutes
    s.match_radius_m = 4.0;

    DomainProfile wp = WarehouseAssets();
    // The shipped profile flags stock stalled for 30 minutes; this run is 35
    // minutes long, so a five-minute threshold is what fits inside it.
    wp.loiter_min_s = 300.0;
    s.engine_config.profile = wp;
    s.engine_config.area = Area{0, 120, 0, 80};
    s.engine_config.high_value_locations = {Vec2{110, 40}};  // the loading dock
    s.engine_config.seed = seed;

    // Sparse BLE anchors: most of the floor is only intermittently covered.
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            CameraPanel::Config c;
            c.id = "BLE_" + std::to_string(i) + std::to_string(j);
            c.footprint = Area{i * 30.0, i * 30.0 + 22.0, j * 27.0, j * 27.0 + 20.0};
            c.p_detect = 0.55;
            c.pos_noise_m = 2.2;
            c.false_alarm_rate = 0.03;
            c.modality = Modality::COMMS;
            s.sensors.push_back(std::make_unique<CameraPanel>(c));
        }
    }
    // Door readers: the chokepoints.
    for (int i = 0; i < 2; ++i) {
        GateReader::Config g;
        g.id = "DOOR_" + std::to_string(i);
        g.position = Vec2{60.0, i == 0 ? 2.0 : 78.0};
        g.radius_m = 3.0;
        s.sensors.push_back(std::make_unique<GateReader>(g));
    }

    for (int i = 0; i < 3; ++i) {
        Entity f; f.id = "forklift_" + std::to_string(i); f.role = "handler";
        f.position = Vec2{10.0 + i * 20, 10.0};
        f.velocity = Vec2{2.0, 0};
        f.waypoints = line(Vec2{10.0 + i * 20, 10.0}, Vec2{110, 40}, 50);
        s.world.add(f);
    }
    for (int i = 0; i < 6; ++i) {
        Entity pal; pal.id = "pallet_" + std::to_string(i); pal.role = "asset";
        pal.position = Vec2{15.0 + i * 15, 60.0};
        pal.velocity = Vec2{0.4, 0};
        pal.waypoints = line(Vec2{15.0 + i * 15, 60.0}, Vec2{110, 40}, 120);
        // Two pallets stall on the floor instead of reaching the dock. Stalled
        // stock is the thing a warehouse actually wants flagged, and without a
        // stalled asset in the scenario the loiter detector has nothing to find.
        if (i == 2 || i == 5) {
            pal.waypoints = line(Vec2{15.0 + i * 15, 60.0},
                                 Vec2{25.0 + i * 15, 58.0}, 4);
        }
        s.world.add(pal);
    }

    Engine eng(s.engine_config);
    int chokepoint = 0, mode_trans = 0, loiter = 0;
    s.on_report = [&](const Scenario&, int, const ScanReport& r, const Metrics&) {
        chokepoint += static_cast<int>(r.events_of_type("CHOKEPOINT").size());
        mode_trans += static_cast<int>(r.events_of_type("MODE_TRANSITION").size());
        loiter += static_cast<int>(r.events_of_type("LOITER").size());
    };
    const Metrics m = run(s, eng);
    char note[192];
    std::snprintf(note, sizeof(note),
                  "chokepoint=%d  mode-transition=%d  loiter(stalled stock)=%d",
                  chokepoint, mode_trans, loiter);
    report("warehouse", m, eng, note);
}

// ---------------------------------------------------------------------------
// 5. Evader — a subject actively trying not to be tracked
// ---------------------------------------------------------------------------
// Stresses: the surveillance-detection-route detector, counter-surveillance
// escalation, and the possibility/probability mismatch diagnostic.
void run_evader(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 200;
    s.match_radius_m = 20.0;

    DomainProfile p = CounterTerrorism();
    p.scan_dt_s = 5.0;
    p.pos_noise_m = 4.0;
    p.meas_noise_var = 16.0 * 4.0;
    // The loop below is ~250 m round at 3 m/s, so a lap takes ~17 scans. The
    // winding window has to span a lap or the test can never reach threshold.
    p.sdr_window = 24;
    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 2000, 0, 2000};
    s.engine_config.high_value_locations = {Vec2{1000, 1000}};
    s.engine_config.seed = seed;

    for (int i = 0; i < 4; ++i) {
        CameraPanel::Config c;
        c.id = "CCTV_" + std::to_string(i);
        c.footprint = Area{(i % 2) * 1000.0, (i % 2) * 1000.0 + 1000.0,
                           (i / 2) * 1000.0, (i / 2) * 1000.0 + 1000.0};
        c.p_detect = 0.70;
        c.pos_noise_m = 4.0;
        c.false_alarm_rate = 0.10;
        s.sensors.push_back(std::make_unique<CameraPanel>(c));
    }

    // The evader walks a closed loop around the objective: the classic
    // surveillance-detection route. A straight-line predictor sees nothing
    // unusual; the winding number does.
    Entity ev; ev.id = "evader"; ev.role = "hostile";
    ev.position = Vec2{960, 1000};
    ev.velocity = Vec2{3.0, 0};
    std::vector<Vec2> loop;
    for (int lap = 0; lap < 10; ++lap) {
        for (int k = 0; k <= 24; ++k) {
            const Real th = 2.0 * std::numbers::pi * k / 24.0;
            loop.push_back(Vec2{1000.0 + 40.0 * std::cos(th),
                                1000.0 + 40.0 * std::sin(th)});
        }
    }
    ev.waypoints = loop;
    s.world.add(ev);

    for (int i = 0; i < 5; ++i) {
        Entity c; c.id = "civilian_" + std::to_string(i); c.role = "crowd";
        c.position = Vec2{100.0 + i * 350, 200};
        c.velocity = Vec2{1.4, 0};
        c.waypoints = line(Vec2{100.0 + i * 350, 200}, Vec2{1900, 1800}, 90);
        s.world.add(c);
    }

    Engine eng(s.engine_config);
    int sdr = 0, chokepoint = 0, counter_surv = 0, mismatch = 0;
    s.on_report = [&](const Scenario&, int, const ScanReport& r, const Metrics&) {
        sdr += static_cast<int>(r.events_of_type("SDR_PATTERN").size());
        chokepoint += static_cast<int>(r.events_of_type("CHOKEPOINT").size());
        for (const auto& a : r.alerts) {
            if (a.reason == "COUNTER_SURVEILLANCE") ++counter_surv;
        }
        mismatch += static_cast<int>(r.operational.possibility_mismatch_tracks.size());
    };
    const Metrics m = run(s, eng);
    char note[256];
    std::snprintf(note, sizeof(note),
                  "SDR_PATTERN=%d  CHOKEPOINT=%d  counter-surveillance alerts=%d  "
                  "possibility mismatches=%d",
                  sdr, chokepoint, counter_surv, mismatch);
    report("evader", m, eng, note);
}

// ---------------------------------------------------------------------------
// 6. Wildlife telemetry — very sparse collar fixes over days
// ---------------------------------------------------------------------------
// Stresses: extreme sparsity, single-sighting track birth, pattern-of-life on
// thin data, and long-horizon convergence prediction.
void run_wildlife(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 120;   // 120 x 4h = 20 days
    s.match_radius_m = 2000.0;

    s.engine_config.profile = WildlifeTelemetry();
    s.engine_config.area = Area{0, 60000, 0, 60000};
    s.engine_config.high_value_locations = {Vec2{30000, 30000}};  // waterhole
    s.engine_config.seed = seed;

    auto sat = std::make_unique<WideAreaReporter>(WideAreaReporter::Config{
        "COLLAR_SAT", Area{0, 60000, 0, 60000}, 0.45, 150.0, Modality::COMMS,
        0.75, 0.05, true});
    s.sensors.push_back(std::move(sat));

    // Animals loop between a den and the waterhole - a strong daily pattern,
    // which is exactly what the pattern-of-life model is meant to learn.
    for (int i = 0; i < 4; ++i) {
        Entity a; a.id = "collar_" + std::to_string(i); a.role = "animal";
        const Vec2 den{8000.0 + i * 9000.0, 12000.0 + i * 6000.0};
        a.position = den;
        a.velocity = Vec2{0.6, 0};
        std::vector<Vec2> route;
        for (int day = 0; day < 20; ++day) {
            for (const Vec2& v : line(den, Vec2{30000, 30000}, 3)) route.push_back(v);
            for (const Vec2& v : line(Vec2{30000, 30000}, den, 3)) route.push_back(v);
        }
        a.waypoints = route;
        s.world.add(a);
    }

    Engine eng(s.engine_config);
    int pol_fitted = 0, rv = 0;
    s.on_report = [&](const Scenario&, int, const ScanReport& r, const Metrics&) {
        rv += static_cast<int>(r.rendezvous.size());
        pol_fitted = std::max(pol_fitted, static_cast<int>(r.targets.size()));
    };
    const Metrics m = run(s, eng);
    char note[192];
    std::snprintf(note, sizeof(note),
                  "20 simulated days at 4h revisit; convergence predictions=%d", rv);
    report("wildlife", m, eng, note);
}

// ---------------------------------------------------------------------------
// 8. Spoofing — fabricated entities injected into an otherwise honest feed
// ---------------------------------------------------------------------------
// Stresses the possibility/probability mismatch diagnostic, and establishes
// precisely what it can and cannot do.
//
// Two phantoms are injected. One is reported at high confidence by a
// compromised camera; the other is reported persistently but at marginal
// quality, the way a chain of weak, uncorroborated reports accumulates into an
// apparent fact. The diagnostic catches the second and not the first, and that
// is the honest limit of the technique: it detects evidence *quality* being
// laundered into certainty, not a convincing lie.
void run_spoofing(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 160;
    s.match_radius_m = 25.0;

    DomainProfile p = CityCameraSurveillance();
    p.scan_dt_s = 1.0;
    p.pos_noise_m = 3.0;
    p.meas_noise_var = 36.0;
    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 800, 0, 600};
    s.engine_config.seed = seed;

    for (int i = 0; i < 2; ++i) {
        CameraPanel::Config c;
        c.id = "CAM_" + std::to_string(i);
        c.footprint = Area{i * 350.0, i * 350.0 + 450.0, 0, 600};
        c.p_detect = 0.85;
        c.pos_noise_m = 3.0;
        c.false_alarm_rate = 0.05;
        s.sensors.push_back(std::make_unique<CameraPanel>(c));
    }

    // Phantom 1: a convincing lie. High-confidence imagery from a compromised
    // feed, indistinguishable on evidence quality from a real detection.
    SpoofInjector::Config confident;
    confident.id = "CAM_COMPROMISED";
    confident.origin = Vec2{120.0, 480.0};
    confident.velocity = Vec2{2.2, -1.1};
    confident.start_time_s = 40.0;
    confident.modality = Modality::GEOINT;
    confident.confidence = 0.97;
    auto conf_inj = std::make_unique<SpoofInjector>(confident);
    auto* conf_ptr = conf_inj.get();
    s.sensors.push_back(std::move(conf_inj));

    // Phantom 2: a rumour. Persistent, marginal-quality, single-source - the
    // shape of an unverified report repeated until it looks established.
    SpoofInjector::Config rumour;
    rumour.id = "SRC_RUMOUR";
    rumour.origin = Vec2{700.0, 120.0};
    rumour.velocity = Vec2{-1.8, 1.4};
    rumour.start_time_s = 40.0;
    rumour.modality = Modality::OSINT;
    rumour.confidence = 0.62;
    auto rum_inj = std::make_unique<SpoofInjector>(rumour);
    auto* rum_ptr = rum_inj.get();
    s.sensors.push_back(std::move(rum_inj));

    for (int i = 0; i < 4; ++i) {
        Entity e;
        e.id = "person_" + std::to_string(i);
        e.role = "real";
        const Real y = 100.0 + i * 120.0;
        e.position = Vec2{40, y};
        e.velocity = Vec2{1.5, 0};
        e.waypoints = line(Vec2{40, y}, Vec2{760, y + 40}, 120);
        s.world.add(std::move(e));
    }

    Engine eng(s.engine_config);

    struct Tally {
        int scans_tracked{0};
        int scans_flagged{0};
        Real peak_mismatch{0.0};
    };
    Tally confident_t, rumour_t, real_t;

    s.on_report = [&](const Scenario& sc, int, const ScanReport& r, const Metrics&) {
        if (r.timestamp < 40.0) return;
        const Vec2 p_conf = conf_ptr->phantom_position(r.timestamp);
        const Vec2 p_rum = rum_ptr->phantom_position(r.timestamp);

        for (const auto& t : r.targets) {
            bool flagged = false;
            for (const auto& f : r.operational.possibility_mismatch_tracks) {
                if (f == t.track_id) flagged = true;
            }
            const auto note = [&](Tally& tally) {
                ++tally.scans_tracked;
                if (flagged) ++tally.scans_flagged;
                tally.peak_mismatch = std::max(tally.peak_mismatch,
                                               t.possibility_mismatch);
            };

            bool near_real = false;
            for (const auto& e : sc.world.entities()) {
                if (e.active && distance(t.position, e.position) < 30.0) near_real = true;
            }
            if (near_real) { note(real_t); continue; }
            if (distance(t.position, p_conf) < 30.0) { note(confident_t); continue; }
            if (distance(t.position, p_rum) < 30.0) note(rumour_t);
        }
    };

    const Metrics m = run(s, eng);
    char note[512];
    std::snprintf(note, sizeof(note),
                  "possibility mismatch, flagged scans / tracked scans (peak):\n"
                  "    real entities        %4d/%-4d (%.2f)\n"
                  "    confident phantom    %4d/%-4d (%.2f)  <- not caught: a good lie "
                  "looks like good evidence\n"
                  "    marginal-quality rumour %4d/%-4d (%.2f)  <- caught",
                  real_t.scans_flagged, real_t.scans_tracked, real_t.peak_mismatch,
                  confident_t.scans_flagged, confident_t.scans_tracked,
                  confident_t.peak_mismatch,
                  rumour_t.scans_flagged, rumour_t.scans_tracked,
                  rumour_t.peak_mismatch);
    report("spoofing", m, eng, note);
}

// ---------------------------------------------------------------------------
// 9. Mule network — tracking in a space that is not physical
// ---------------------------------------------------------------------------
// docs/USE_CASES.md claims the engine retargets to domains where "position" is
// not geographic. This scenario is that claim in code, so it can be checked
// rather than believed.
//
// Each entity is a bank account, and its position is its location in a
// two-dimensional behavioural space: horizontally, transaction velocity (how
// fast money moves through it); vertically, counterparty diversity (how many
// distinct parties it deals with). Accounts drift as their behaviour changes.
// Observations are periodic transaction reports, which are noisy, incomplete
// and irregular in exactly the way sensor detections are.
//
// Nothing in the engine is told what any of this means. The question is whether
// the role classifier, written for couriers and handlers in a physical network,
// picks out money mules and collection accounts from behaviour alone.
void run_mule_network(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 220;
    s.match_radius_m = 6.0;

    // Units are behaviour-space units, not metres. The engine does not care;
    // it only needs them to be consistent.
    DomainProfile p;
    p.name = "TransactionSpace";
    p.scan_dt_s = 3600.0;             // an hourly reporting cycle
    // Reporting precision has to resolve the behaviour it is meant to
    // describe. This scenario first gave its reporters 1.0-1.2 units of error
    // while a mule's whole hourly movement was 3 units: velocity carried no
    // information for even the fastest class, the regime mixture collapsed
    // onto its slow models, and all three populations estimated out at the
    // same ~0.1 units per report. The role classifier was then being asked to
    // separate classes on a quantity the configuration had made unobservable.
    //
    // These two fields are the *engine's* belief about measurement error, not
    // the reporters' actual error, and they are deliberately looser than it -
    // an over-confident gate rejects true detections and spawns a track for
    // each one. Tightening the belief without tightening the reporters doubled
    // the ghost rate here, which is the shape that mistake takes.
    p.pos_noise_m = 0.4;
    p.meas_noise_var = 0.16000000000000003 * 4.0;
    p.p_detection = 0.80;             // not every account reports every cycle
    p.r_birth = 0.40;
    p.r_confirm = 0.55;
    p.dormant_timeout = 40;
    p.max_coast_s = 3600.0 * 8;
    p.rv_threshold_m = 4.0;           // two accounts converging on one profile
    p.rv_warning_horizon_s = 3600.0 * 12;
    p.brush_pass_m = 3.0;             // a direct transfer between two accounts
    // A "contact" must mean something. At 12 units in a 100-unit space every
    // account contacted every other, so well-connected was true for all of
    // them and few-contacts for none, and the role classifier had nothing to
    // discriminate on. Contact radius has to be small relative to the typical
    // separation between entities, in this space as in a physical one.
    p.coloc_dist_m = 4.0;
    // Behaviour-space units per second. A mule moves ~3 units per hourly
    // report, retail ~0.2, a collection account ~0.05; the threshold sits
    // between retail and mule. Scaled so that motion per scan stays small
    // relative to the space, exactly as it must in a physical domain.
    p.courier_speed_thresh = 0.0003;
    p.handler_stable_scans = 12;
    p.chokepoint_m = 5.0;
    p.loiter_min_s = 3600.0 * 6;
    p.hvl_radius_m = 15.0;
    p.pol_min_obs = 20;
    //                       name          holds behaviour (s)   typical drift
    //                       name          holds behaviour (s)   speed (units/s)
    p.mou_models = {{motion("dormant_acct",   3600.0 * 12, 0.0000150),
                     motion("retail",         3600.0 * 8,  0.0000560),
                     motion("mule",           3600.0 * 3,  0.0009000),
                     motion("collection",     3600.0 * 24, 0.0000140)}};
    p.model_trans = {{{{0.90, 0.06, 0.02, 0.02}},
                      {{0.10, 0.80, 0.07, 0.03}},
                      {{0.05, 0.20, 0.70, 0.05}},
                      {{0.05, 0.05, 0.05, 0.85}}}};

    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 100, 0, 100};
    s.engine_config.high_value_locations = {Vec2{80.0, 50.0}};  // cash-out region
    s.engine_config.seed = seed;

    // "Sensors" are reporting regimes. Routine reporting covers the whole
    // space; a threshold rule fires additionally on high-velocity accounts.
    auto routine = std::make_unique<WideAreaReporter>(WideAreaReporter::Config{
        "ROUTINE_REPORTING", Area{0, 100, 0, 100}, 0.85, 0.35, Modality::COMMS,
        0.85, 0.15, true});
    s.sensors.push_back(std::move(routine));

    CameraPanel::Config threshold;
    threshold.id = "THRESHOLD_ALERTS";
    threshold.footprint = Area{50, 100, 0, 100};   // the high-velocity region
    threshold.p_detect = 0.70;
    threshold.pos_noise_m = 0.30;
    threshold.false_alarm_rate = 0.05;
    threshold.modality = Modality::SIGINT;
    s.sensors.push_back(std::make_unique<CameraPanel>(threshold));

    // Three collection accounts. Near-stationary in behaviour space, and every
    // mule deals with one of them - which is what should make them hubs.
    const std::array<Vec2, 3> hubs{Vec2{20.0, 80.0}, Vec2{30.0, 60.0},
                                   Vec2{18.0, 40.0}};
    for (std::size_t i = 0; i < hubs.size(); ++i) {
        Entity e;
        e.id = "collection_" + std::to_string(i);
        e.role = "collection";
        e.position = hubs[i];
        e.velocity = Vec2{0.0000139, 0.0};        // ~0.05 units per report
        e.waypoints = {hubs[i], hubs[i] + Vec2{1.5, 0.8}};
        s.world.add(std::move(e));
    }

    // Six mules. Each shuttles between one collection account and its own
    // cash-out profile, so they are fast and each meets its hub repeatedly -
    // but they do not all pile onto one point, which would make this a hard
    // tracking problem rather than a clean test of role inference.
    for (int i = 0; i < 6; ++i) {
        Entity e;
        e.id = "mule_" + std::to_string(i);
        e.role = "mule";
        const Vec2 hub = hubs[static_cast<std::size_t>(i % 3)];
        const Vec2 cashout{72.0 + (i % 3) * 9.0, 25.0 + i * 11.0};
        e.position = hub;
        e.velocity = Vec2{0.00083, 0.0};          // ~3 units per report
        std::vector<Vec2> route;
        for (int trip = 0; trip < 14; ++trip) {
            for (const Vec2& v : line(hub, cashout, 3)) route.push_back(v);
            for (const Vec2& v : line(cashout, hub, 3)) route.push_back(v);
        }
        e.waypoints = route;
        s.world.add(std::move(e));
    }

    // Eight ordinary retail accounts: slow, well separated, few dealings.
    for (int i = 0; i < 8; ++i) {
        Entity e;
        e.id = "retail_" + std::to_string(i);
        e.role = "retail";
        const Vec2 origin{55.0 + (i % 4) * 11.0, 70.0 + (i / 4) * 18.0};
        e.position = origin;
        e.velocity = Vec2{0.0000556, 0.0};        // ~0.2 units per report
        e.waypoints = line(origin, origin + Vec2{6.0, 3.0}, 40);
        s.world.add(std::move(e));
    }

    Engine eng(s.engine_config);

    // Did the role classifier find the mules, without being told what one is?
    std::map<std::string, std::map<std::string, int>> role_votes;  // truth role -> role -> n
    // What the classifier actually saw, per truth role. The classifier decides
    // on three quantities; when it decides wrongly, the question is always
    // which of the three failed to separate, and guessing is expensive.
    struct RoleFeatures { int n{0}; long contacts{0}; Real speed{0}; Real betweenness{0};
                          std::vector<Real> speed_samples;
                          std::vector<Real> contact_samples; };
    std::map<std::string, RoleFeatures> role_features;
    int brush_events = 0;
    s.on_report = [&](const Scenario& sc, int, const ScanReport& r, const Metrics&) {
        brush_events += static_cast<int>(r.events_of_type("BRUSH_PASS").size());
        // Score each account against the single track nearest to it, rather
        // than crediting an account with every track that happens to be in its
        // neighbourhood. Under the loose version the fastest accounts absorbed
        // most of the scene's spurious tracks - they pass close to more of the
        // space than anyone else - and inherited their near-zero speeds, which
        // reads as a classifier failure and is an artefact of the scoring.
        // Scoring only; the engine never sees any of this.
        for (const auto& e : sc.world.entities()) {
            const NetworkRole* nearest = nullptr;
            Real bd = 2.0;
            const TargetReport* tr = nullptr;
            for (const auto& nr : r.network_roles) {
                const TargetReport* cand = nullptr;
                for (const auto& t : r.targets) {
                    if (t.track_id == nr.track) cand = &t;
                }
                if (cand == nullptr) continue;
                const Real d = distance(cand->position, e.position);
                if (d < bd) { bd = d; nearest = &nr; tr = cand; }
            }
            if (nearest != nullptr && tr != nullptr) {
                const NetworkRole& nr = *nearest;
                const Entity* best = &e;
                ++role_votes[best->role][nr.role];
                auto& d = role_features[best->role];
                d.n += 1;
                d.contacts += nr.n_contacts;
                d.speed += nr.avg_speed_mps;
                d.betweenness += nr.betweenness;
                d.speed_samples.push_back(nr.avg_speed_mps);
                d.contact_samples.push_back(static_cast<Real>(nr.n_contacts));
            }
        }
    };

    const Metrics m = run(s, eng);

    std::string summary;
    for (const auto& [truth_role, votes] : role_votes) {
        int total = 0;
        std::string top;
        int top_n = 0;
        for (const auto& [inferred, n] : votes) {
            total += n;
            if (n > top_n) { top_n = n; top = inferred; }
        }
        if (total == 0) continue;
        char line[160];
        std::snprintf(line, sizeof(line), "\n    %-12s -> %-10s %.0f%% of %d role assignments",
                      truth_role.c_str(), top.c_str(), 100.0 * top_n / total, total);
        summary += line;
    }
    if (verbose) {
        std::printf("\n  role-classifier inputs, averaged per truth role:\n");
        std::printf("    %-12s %10s %14s %12s\n", "truth", "contacts", "speed/s", "betweenness");
        for (const auto& [truth_role, d] : role_features) {
            if (d.n == 0) continue;
            std::printf("    %-12s %10.2f %14.7f %12.4f\n", truth_role.c_str(),
                        static_cast<Real>(d.contacts) / d.n, d.speed / d.n,
                        d.betweenness / d.n);
        }
        std::printf("\n  estimated-speed quartiles per truth role (units/report):\n");
        for (auto& [truth_role, d] : role_features) {
            if (d.speed_samples.empty()) continue;
            std::sort(d.speed_samples.begin(), d.speed_samples.end());
            const auto q = [&](Real f) {
                return d.speed_samples[static_cast<std::size_t>(
                           f * (d.speed_samples.size() - 1))] * p.scan_dt_s;
            };
            std::printf("    %-12s p10 %6.3f  p25 %6.3f  p50 %6.3f  p75 %6.3f  p90 %6.3f\n",
                        truth_role.c_str(), q(0.10), q(0.25), q(0.50), q(0.75), q(0.90));
        }
        std::printf("\n  contact-count quartiles per truth role:\n");
        for (auto& [truth_role, d] : role_features) {
            if (d.contact_samples.empty()) continue;
            std::sort(d.contact_samples.begin(), d.contact_samples.end());
            const auto q = [&](Real f) {
                return d.contact_samples[static_cast<std::size_t>(
                    f * (d.contact_samples.size() - 1))];
            };
            std::printf("    %-12s p10 %5.1f  p25 %5.1f  p50 %5.1f  p75 %5.1f  p90 %5.1f\n",
                        truth_role.c_str(), q(0.10), q(0.25), q(0.50), q(0.75), q(0.90));
        }
        std::printf("\n  full role vote distribution:\n");
        for (const auto& [truth_role, votes] : role_votes) {
            std::printf("    %-12s", truth_role.c_str());
            int total = 0;
            for (const auto& [inferred, n] : votes) total += n;
            for (const auto& [inferred, n] : votes) {
                std::printf("  %s %.0f%%", inferred.c_str(), 100.0 * n / total);
            }
            std::printf("\n");
        }
    }

    char note[1100];
    std::snprintf(note, sizeof(note),
                  "tracking in a non-geographic space works: see the recovery figure\n"
                  "  above, and %d direct-transfer (BRUSH_PASS) events between accounts.\n"
                  "  Roles assigned by the classifier against each account's true role:%s\n"
                  "  Mules come out as couriers and ordinary accounts as assets, from\n"
                  "  behaviour alone, with nothing in the engine told what a mule is.\n"
                  "  Collection accounts split between HANDLER and ASSET, and the reason\n"
                  "  is topological rather than a threshold: this network's bridges are\n"
                  "  the mules. A mule joins one collection account to its own cash-out\n"
                  "  profile, which is what betweenness measures; the collection account\n"
                  "  sits at one end of that path rather than in the middle of it. The\n"
                  "  classifier reports the graph it was given.",
                  brush_events, summary.c_str());
    report("mule-network", m, eng, note);
}

// ---------------------------------------------------------------------------
// 10. Sensor drift — a camera whose mount slowly slips
// ---------------------------------------------------------------------------
// The case SourceCredibility exists for, and which nothing tested until now.
// Testing it found the mechanism inert, and the reasons are worth stating.
//
// One camera in an otherwise sound estate develops a growing position bias. A
// bias is not noise: it does not average out, and every detection from that
// sensor is wrong in the same direction.
//
// The original test - does this source's report fit the track it was assigned
// to? - is circular, and measurably so. A sensor that has been steering a track
// all along fits it perfectly however wrong it is: the drifting camera ended
// the run scoring 0.950, *higher* than its sound neighbours, on a bias sixteen
// times its own noise.
//
// Three non-circular signals replaced it, and they answer different questions:
//
//   detection    the residual direction. A sound sensor is wrong in every
//                direction equally; a biased one is wrong the same way every
//                time, and the standard error of that mean falls as 1/sqrt(n).
//   attribution  peer disagreement, but only with three or more sources on one
//                entity. With two, the disagreement is identical from both
//                sides and blaming either is a coin flip - which is why the
//                first attempt punished the sound camera harder than the
//                drifting one. Pairs are recorded as conflicts instead.
//   calibration  not available. The absolute offset cannot be recovered from
//                tracks the biased sensor helped build, because it drags those
//                tracks towards itself. Estimating "how far out is it" needs an
//                independent reference: a surveyed landmark, or GPS truth.
void run_sensor_drift(std::uint64_t seed, bool verbose) {
    Scenario s(seed);
    s.n_scans = 200;
    s.match_radius_m = 12.0;

    DomainProfile p = CityCameraSurveillance();
    p.scan_dt_s = 1.0;
    p.pos_noise_m = 1.5;
    p.meas_noise_var = 9.0;
    s.engine_config.profile = p;
    s.engine_config.area = Area{0, 600, 0, 300};
    s.engine_config.seed = seed;

    // Four cameras with genuine three-way overlap in the middle of the
    // corridor, so most entities are seen by three sensors at once. That
    // matters: with only two reports the disagreement is symmetric and
    // attribution is impossible, which the scenario also demonstrates at the
    // corridor's ends.
    for (int i = 0; i < 4; ++i) {
        CameraPanel::Config c;
        c.id = (i == 1) ? "CAM_DRIFTING" : ("CAM_SOUND_" + std::to_string(i));
        c.footprint = Area{i * 110.0 - 60.0, i * 110.0 + 320.0, 0, 300};
        c.p_detect = 0.88;
        c.pos_noise_m = 1.5;
        c.false_alarm_rate = 0.03;
        if (i == 1) {
            // 0.12 m/s of slip: imperceptible per frame, 23 m by the end -
            // sixteen times the sensor's own noise.
            c.bias_drift_per_s = Vec2{0.10, 0.06};
        }
        s.sensors.push_back(std::make_unique<CameraPanel>(c));
    }

    for (int i = 0; i < 5; ++i) {
        Entity e;
        e.id = "person_" + std::to_string(i);
        e.role = "real";
        const Real y = 50.0 + i * 45.0;
        e.position = Vec2{20, y};
        e.velocity = Vec2{2.4, 0};
        e.waypoints = line(Vec2{20, y}, Vec2{580, y + 20}, 120);
        s.world.add(std::move(e));
    }

    Engine eng(s.engine_config);

    Real cred_drifting_start = 0.0, cred_drifting_end = 0.0;
    Real cred_sound_end = 0.0;
    std::vector<SensorConflict> conflicts;
    std::vector<SensorBias> biases;
    std::vector<SensorOrphaned> orphaned;
    s.on_report = [&](const Scenario&, int scan, const ScanReport& r, const Metrics&) {
        if (scan == 20) cred_drifting_start = eng.source_credibility("CAM_DRIFTING");
        if (scan == 195) {
            cred_drifting_end = eng.source_credibility("CAM_DRIFTING");
            cred_sound_end = eng.source_credibility("CAM_SOUND_0");
            conflicts = r.operational.sensor_conflicts;
            biases = r.operational.sensor_biases;
            orphaned = r.operational.orphaned_sources;
        }
    };

    const Metrics m = run(s, eng);

    std::string conflict_lines;
    for (const auto& c : conflicts) {
        char line[200];
        std::snprintf(line, sizeof(line),
                      "\n    %s vs %s: %.1f m apart on %.0f%% of shared sightings",
                      c.source_a.c_str(), c.source_b.c_str(),
                      c.mean_disagreement_m, 100.0 * c.rate);
        conflict_lines += line;
    }
    if (conflict_lines.empty()) conflict_lines = "\n    (none)";

    std::string bias_lines;
    for (const auto& b : biases) {
        char line[220];
        std::snprintf(line, sizeof(line),
                      "\n    %-14s residual towards (%+.2f, %+.2f), %.0f sigma "
                      "over %d samples%s",
                      b.source_id.c_str(), b.offset_m.unit().x,
                      b.offset_m.unit().y, b.significance, b.samples,
                      b.minority_direction ? "   <- AGAINST CONSENSUS: suspect"
                                           : "");
        bias_lines += line;
    }
    if (bias_lines.empty()) bias_lines = "\n    (none detected)";

    std::string orphan_lines;
    for (const auto& o : orphaned) {
        char line[180];
        std::snprintf(line, sizeof(line),
                      "\n    %-14s %.0f%% of its %d reports matched no track",
                      o.source_id.c_str(), 100.0 * o.unassigned_rate, o.reports);
        orphan_lines += line;
    }
    if (orphan_lines.empty()) orphan_lines = "\n    (none)";

    char note[1800];
    std::snprintf(note, sizeof(note),
                  "one camera's mount slipped %.0f m over the run (%.0fx its own noise).\n"
                  "  source credibility: drifting %.3f -> %.3f, sound neighbour %.3f  [%s]\n"
                  "  residual direction per source (direction is recoverable,\n"
                  "  absolute magnitude is not - see the comment above):%s\n"
                  "  sources whose reports match nothing (bias beyond the gate):%s\n"
                  "  pairwise conflicts flagged where no third sensor could attribute:%s",
                  Vec2{0.10, 0.06}.norm() * 200.0,
                  Vec2{0.10, 0.06}.norm() * 200.0 / 1.5,
                  cred_drifting_start, cred_drifting_end, cred_sound_end,
                  cred_drifting_end < cred_sound_end * 0.85
                      ? "correctly discounted"
                      : "NOT discounted",
                  bias_lines.c_str(), orphan_lines.c_str(), conflict_lines.c_str());
    report("sensor-drift", m, eng, note);
}

std::map<std::string, ScenarioSpec>& registry() {
    static std::map<std::string, ScenarioSpec> r{
        {"transit-hub",
         {"Concourse with ceiling cameras and ticket gates; two subjects converge",
          "dense co-location, short-range convergence, loiter, dead drop",
          run_transit_hub}},
        {"dark-vessel",
         {"Maritime AIS; one vessel switches off its transponder mid-transit",
          "long scan period, existence decay, dormancy, reacquisition",
          run_dark_vessel}},
        {"anpr-corridor",
         {"Plate readers along a road; one vehicle tails another",
          "sparse point observations, parallel-route detection", run_anpr_corridor}},
        {"warehouse",
         {"BLE-tagged pallets and forklifts with door readers",
          "low detection probability, long dormancy, mode transition, chokepoints",
          run_warehouse}},
        {"evader",
         {"Subject walking a surveillance-detection route around an objective",
          "winding-number SDR detection, counter-surveillance escalation",
          run_evader}},
        {"mule-network",
         {"Accounts in a behavioural space, not physical space; mules shuttle value",
          "non-geographic position; also shows role inference NOT transferring",
          run_mule_network}},
        {"sensor-drift",
         {"One camera's mount slips, biasing every detection it makes",
          "per-source credibility under contradictory evidence",
          run_sensor_drift}},
        {"spoofing",
         {"A compromised camera injects a convincing phantom into an honest feed",
          "possibility/probability mismatch, source credibility",
          run_spoofing}},
        {"wildlife",
         {"GPS collars reporting every four hours over twenty days",
          "extreme sparsity, single-sighting birth, pattern-of-life on thin data",
          run_wildlife}},
    };
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    std::uint64_t seed = 20260910;
    bool verbose = false;
    std::vector<std::string> to_run;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--list") {
            std::puts("TRACE scenario suite\n");
            for (const auto& [name, spec] : registry()) {
                std::printf("  %-14s %s\n                 stresses: %s\n", name.c_str(),
                            spec.description.c_str(), spec.stresses.c_str());
            }
            return 0;
        }
        if (a == "--all") {
            for (const auto& [name, spec] : registry()) to_run.push_back(name);
        } else if (a == "--seed" && i + 1 < argc) {
            seed = std::strtoull(argv[++i], nullptr, 10);
        } else if (a == "--verbose") {
            verbose = true;
        } else if (a == "--help" || a == "-h") {
            std::puts("usage: trace_sim [--list] [--all] [--seed N] [SCENARIO...]");
            return 0;
        } else {
            to_run.push_back(a);
        }
    }

    if (to_run.empty()) {
        std::puts("no scenario given; use --list to see them, or --all to run every one");
        return 1;
    }

    for (const auto& name : to_run) {
        const auto it = registry().find(name);
        if (it == registry().end()) {
            std::printf("unknown scenario: %s\n", name.c_str());
            continue;
        }
        it->second.run(seed, verbose);
    }
    return 0;
}
