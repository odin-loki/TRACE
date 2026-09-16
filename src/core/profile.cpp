// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/core/profile.hpp"

#include <algorithm>

namespace trace {

DomainProfile UrbanHUMINT() {
    return DomainProfile{};  // the defaults are the urban preset
}

DomainProfile Maritime() {
    DomainProfile p;
    p.name = "Maritime";
    p.scan_dt_s = 3600.0;          // hourly AIS / satellite refresh
    p.pos_noise_m = 200.0;
    p.meas_noise_var = 200.0 * 200.0;
    p.p_detection = 0.75;          // vessels go dark
    p.rv_threshold_m = 2000.0;
    p.rv_warning_horizon_s = 7200.0;
    p.brush_pass_m = 500.0;
    p.parallel_route_m = 800.0;
    p.parallel_vel_cos = 0.99;
    p.mode_trans_m = 500.0;
    p.coloc_dist_m = 3000.0;
    p.hvl_radius_m = 5000.0;
    p.courier_speed_thresh = 2.0;
    p.chokepoint_m = 1500.0;
    p.mou_models = {{motion("drifting",     7200.0,  0.5),
                     motion("transiting",  14400.0,  6.0),   // ~12 knots
                     motion("anchored",     3600.0,  0.05),
                     motion("fast_craft",   7200.0, 15.0)}}; // ~30 knots
    p.model_trans = {{{{0.80, 0.15, 0.04, 0.01}},
                      {{0.05, 0.88, 0.05, 0.02}},
                      {{0.20, 0.05, 0.74, 0.01}},
                      {{0.02, 0.30, 0.01, 0.67}}}};
    return p;
}

DomainProfile Airspace() {
    DomainProfile p;
    p.name = "Airspace";
    p.scan_dt_s = 5.0;             // radar sweep
    p.pos_noise_m = 50.0;
    p.meas_noise_var = 50.0 * 50.0;
    p.p_detection = 0.98;
    p.rv_threshold_m = 1000.0;
    p.rv_warning_horizon_s = 600.0;
    p.brush_pass_m = 300.0;
    p.coloc_dist_m = 2000.0;
    p.hvl_radius_m = 20000.0;
    p.parallel_route_m = 500.0;
    p.parallel_vel_cos = 0.995;
    p.chokepoint_m = 2000.0;
    p.mou_models = {{motion("hovering",      10.0,   2.0),
                     motion("fixed_wing",    60.0, 120.0),
                     motion("gliding",       30.0,  30.0),
                     motion("fast_jet",      90.0, 300.0)}};
    p.model_trans = {{{{0.90, 0.05, 0.04, 0.01}},
                      {{0.02, 0.92, 0.03, 0.03}},
                      {{0.05, 0.10, 0.83, 0.02}},
                      {{0.01, 0.15, 0.01, 0.83}}}};
    return p;
}

DomainProfile VehicleConvoy() {
    DomainProfile p;
    p.name = "VehicleConvoy";
    p.scan_dt_s = 10.0;
    p.pos_noise_m = 3.0;
    p.meas_noise_var = 9.0;
    p.p_detection = 0.92;
    p.rv_threshold_m = 30.0;
    p.rv_warning_horizon_s = 300.0;
    p.brush_pass_m = 20.0;
    p.parallel_route_m = 15.0;
    p.parallel_vel_cos = 0.99;
    p.coloc_dist_m = 100.0;
    p.hvl_radius_m = 500.0;
    p.chokepoint_m = 25.0;
    p.mou_models = {{motion("stopped",       10.0,  0.1),
                     motion("slow_roll",     20.0,  3.0),
                     motion("highway",       60.0, 25.0),
                     motion("sprint",        40.0, 40.0)}};
    p.model_trans = {{{{0.75, 0.20, 0.04, 0.01}},
                      {{0.15, 0.72, 0.08, 0.05}},
                      {{0.03, 0.10, 0.82, 0.05}},
                      {{0.02, 0.08, 0.10, 0.80}}}};
    return p;
}

DomainProfile CityCameraSurveillance() {
    DomainProfile p;
    p.name = "CityCameraSurveillance";
    p.scan_dt_s = 1.0;             // camera frame-rate aggregation
    p.pos_noise_m = 8.0;           // re-ID projected to ground plane
    p.meas_noise_var = 64.0;
    p.p_detection = 0.70;          // coverage gaps dominate
    p.max_tracks = 400;            // a city camera estate is genuinely crowded
    p.r_birth = 0.40;              // two detections before a track is reported
    p.dormant_timeout = 120;       // long gaps between camera zones
    p.rv_threshold_m = 25.0;
    p.rv_warning_horizon_s = 300.0;
    p.brush_pass_m = 10.0;
    p.parallel_route_m = 12.0;
    p.parallel_scans = 10;
    p.mode_trans_m = 30.0;
    p.loiter_min_s = 120.0;
    p.coloc_dist_m = 30.0;
    p.chokepoint_m = 15.0;
    p.chokepoint_n = 3;
    p.courier_speed_thresh = 1.2;
    p.hvl_radius_m = 100.0;
    p.mou_models = {{motion("walking",        8.0,  1.4),
                     motion("running",        6.0,  4.0),
                     motion("standing",       4.0,  0.15),
                     motion("vehicle",       15.0, 10.0)}};
    p.model_trans = {{{{0.86, 0.06, 0.07, 0.01}},
                      {{0.25, 0.65, 0.05, 0.05}},
                      {{0.20, 0.02, 0.76, 0.02}},
                      {{0.05, 0.02, 0.03, 0.90}}}};
    return p;
}

DomainProfile CounterTerrorism() {
    DomainProfile p = UrbanHUMINT();
    p.name = "CounterTerrorism";
    p.p_detection = 0.80;
    p.r_birth = 0.40;
    p.r_confirm = 0.45;              // accept weaker tracks; misses cost more
    p.dormant_timeout = 90;
    p.rv_warning_horizon_s = 3600.0; // one hour of warning
    p.loiter_mult = 2.0;             // trip-wire earlier on dwell
    p.loiter_min_s = 180.0;
    p.cover_stop_hvl_m = 1200.0;
    p.sdr_window = 48;               // a counter-surveillance loop is slow
    p.chokepoint_n = 2;              // two passes is already interesting
    p.hvl_radius_m = 1000.0;
    p.threat_weights = {0.26, 0.20, 0.14, 0.12, 0.08, 0.06, 0.08, 0.06};
    return p;
}

DomainProfile OrganisedCrimeNetwork() {
    DomainProfile p = UrbanHUMINT();
    p.name = "OrganisedCrimeNetwork";
    p.scan_dt_s = 300.0;             // slower, longer-baseline collection
    p.dormant_timeout = 200;
    p.rv_warning_horizon_s = 5400.0;
    p.coloc_dist_m = 500.0;
    p.handler_stable_scans = 20;
    p.pol_min_obs = 25;              // richer baselines before judging normal
    p.threat_weights = {0.16, 0.14, 0.12, 0.12, 0.10, 0.10, 0.16, 0.10};
    return p;
}

DomainProfile FugitiveTracking() {
    DomainProfile p = UrbanHUMINT();
    p.name = "FugitiveTracking";
    p.p_detection = 0.35;            // sightings are rare and unreliable
    // Deliberately inverted: when a sighting may be the only one for days,
    // a single credible report should raise a reportable track immediately,
    // with no demand for corroboration in the very next scan.
    p.r_birth = 0.45;
    p.r_confirm = 0.35;
    p.two_point_initiation = false;
    p.r_prune = 0.02;
    p.r_dormant = 0.01;
    p.dormant_timeout = 500;         // weeks of dormancy, then reacquire
    p.pol_min_obs = 8;               // learn from whatever little there is
    p.pol_refit_interval = 3;
    p.rv_warning_horizon_s = 7200.0;
    p.coloc_dist_m = 800.0;
    p.threat_weights = {0.30, 0.10, 0.10, 0.15, 0.10, 0.05, 0.12, 0.08};
    return p;
}

DomainProfile BorderPatrol() {
    DomainProfile p;
    p.name = "BorderPatrol";
    p.scan_dt_s = 30.0;
    p.pos_noise_m = 25.0;
    p.meas_noise_var = 625.0;
    p.p_detection = 0.65;
    p.dormant_timeout = 60;
    p.rv_threshold_m = 300.0;
    p.rv_warning_horizon_s = 2400.0;
    p.brush_pass_m = 150.0;
    p.parallel_route_m = 200.0;
    p.mode_trans_m = 200.0;          // vehicle drop-off then foot crossing
    p.loiter_min_s = 600.0;          // lying up before a crossing
    p.cover_stop_m = 500.0;
    p.chokepoint_m = 100.0;
    p.chokepoint_n = 2;
    p.coloc_dist_m = 400.0;
    p.hvl_radius_m = 1500.0;
    p.mou_models = {{motion("foot",          60.0,  1.2),
                     motion("vehicle",       90.0, 15.0),
                     motion("lying_up",      30.0,  0.05),
                     motion("fast_run",      45.0,  4.0)}};
    p.model_trans = {{{{0.82, 0.06, 0.10, 0.02}},
                      {{0.10, 0.83, 0.04, 0.03}},
                      {{0.22, 0.04, 0.72, 0.02}},
                      {{0.20, 0.05, 0.05, 0.70}}}};
    return p;
}

DomainProfile IndoorVenue() {
    DomainProfile p;
    p.name = "IndoorVenue";
    p.scan_dt_s = 2.0;
    p.pos_noise_m = 1.5;
    p.meas_noise_var = 2.25;
    p.p_detection = 0.75;
    p.dormant_timeout = 60;
    p.rv_threshold_m = 3.0;
    p.rv_warning_horizon_s = 180.0;
    p.brush_pass_m = 2.0;
    p.parallel_route_m = 3.0;
    p.parallel_scans = 12;
    p.mode_trans_m = 5.0;
    p.loiter_min_s = 90.0;
    p.loiter_mult = 2.5;
    p.cover_stop_m = 20.0;
    p.cover_stop_hvl_m = 30.0;
    p.chokepoint_m = 4.0;
    p.coloc_dist_m = 5.0;
    p.courier_speed_thresh = 0.8;
    p.hvl_radius_m = 15.0;
    p.mou_models = {{motion("walking",       10.0,  1.3),
                     motion("hurrying",       8.0,  2.5),
                     motion("standing",       5.0,  0.10),
                     motion("browsing",       6.0,  0.40)}};
    p.model_trans = {{{{0.80, 0.06, 0.09, 0.05}},
                      {{0.30, 0.62, 0.05, 0.03}},
                      {{0.18, 0.02, 0.72, 0.08}},
                      {{0.15, 0.02, 0.13, 0.70}}}};
    return p;
}

DomainProfile WarehouseAssets() {
    DomainProfile p;
    p.name = "WarehouseAssets";
    p.scan_dt_s = 5.0;
    p.pos_noise_m = 2.5;             // BLE/RFID trilateration is coarse
    p.meas_noise_var = 6.25;
    p.p_detection = 0.60;            // reader coverage is patchy by design
    p.r_birth = 0.38;
    p.r_confirm = 0.50;
    p.dormant_timeout = 300;         // a pallet can sit for hours unread
    p.rv_threshold_m = 2.0;          // asset meets handler
    p.rv_warning_horizon_s = 120.0;
    p.brush_pass_m = 1.5;
    p.mode_trans_m = 3.0;            // pallet moves forklift -> conveyor
    p.mode_trans_scans = 3;
    p.loiter_min_s = 1800.0;         // dwell here means stalled inventory
    p.loiter_mult = 4.0;
    p.chokepoint_m = 3.0;
    p.chokepoint_n = 2;
    p.coloc_dist_m = 4.0;
    p.courier_speed_thresh = 0.5;
    p.hvl_radius_m = 10.0;
    p.mou_models = {{motion("static",        20.0,  0.02),
                     motion("handled",       10.0,  0.50),
                     motion("forklift",      20.0,  2.50),
                     motion("conveyor",      30.0,  1.00)}};
    p.model_trans = {{{{0.94, 0.04, 0.01, 0.01}},
                      {{0.30, 0.55, 0.10, 0.05}},
                      {{0.10, 0.15, 0.72, 0.03}},
                      {{0.10, 0.05, 0.05, 0.80}}}};
    return p;
}

DomainProfile WildlifeTelemetry() {
    DomainProfile p;
    p.name = "WildlifeTelemetry";
    p.scan_dt_s = 14400.0;           // 4-hour satellite duty cycle
    p.pos_noise_m = 150.0;
    p.meas_noise_var = 150.0 * 150.0;
    p.p_detection = 0.45;            // canopy, battery, orbit geometry
    // Also inverted: collar fixes are scarce enough that one is worth acting on.
    p.r_birth = 0.60;
    p.r_confirm = 0.45;
    p.two_point_initiation = false;
    p.dormant_timeout = 60;
    p.rv_threshold_m = 500.0;
    p.rv_warning_horizon_s = 86400.0;  // a day of warning
    p.brush_pass_m = 300.0;
    p.parallel_route_m = 1000.0;
    p.loiter_min_s = 43200.0;        // a den or kill site
    p.loiter_mult = 2.0;
    p.cover_stop_m = 2000.0;
    p.chokepoint_m = 800.0;
    p.chokepoint_n = 3;
    p.coloc_dist_m = 1500.0;
    p.courier_speed_thresh = 0.3;
    p.hvl_radius_m = 3000.0;         // waterhole, boundary, poaching hotspot
    p.pol_min_obs = 20;
    // Heading-hold times are long because the sampling interval is four hours:
    // anything shorter has fully decorrelated by the next fix, and the filter
    // would have no basis on which to predict.
    p.mou_models = {{motion("resting",     14400.0,  0.05),
                     motion("foraging",   21600.0,  0.40),
                     motion("transiting", 43200.0,  1.50),
                     motion("fleeing",     7200.0,  6.00)}};
    p.model_trans = {{{{0.78, 0.18, 0.03, 0.01}},
                      {{0.22, 0.68, 0.08, 0.02}},
                      {{0.08, 0.22, 0.68, 0.02}},
                      {{0.05, 0.15, 0.30, 0.50}}}};
    return p;
}

DomainProfile SportsPitch() {
    DomainProfile p;
    p.name = "SportsPitch";
    p.scan_dt_s = 0.04;              // 25 fps
    p.pos_noise_m = 0.25;
    p.meas_noise_var = 0.0625;
    p.p_detection = 0.88;
    p.max_tracks = 60;             // a squad, not a crowd
    p.r_birth = 0.38;
    p.r_confirm = 0.50;
    p.dormant_timeout = 50;          // occlusion in a ruck or scrum
    p.rv_threshold_m = 1.5;
    p.rv_warning_horizon_s = 4.0;
    p.brush_pass_m = 1.0;
    p.parallel_route_m = 2.0;
    p.parallel_scans = 25;
    p.parallel_vel_cos = 0.95;
    p.mode_trans_m = 2.0;
    p.loiter_min_s = 8.0;
    p.chokepoint_m = 2.0;
    p.coloc_dist_m = 3.0;
    p.courier_speed_thresh = 4.0;
    p.hvl_radius_m = 8.0;            // the goal, the try line
    p.pol_min_obs = 40;
    p.mou_models = {{motion("jogging",        2.0,  3.5),
                     motion("sprinting",     1.5,  8.0),
                     motion("standing",      1.0,  0.30),
                     motion("cutting",       0.6,  5.0)}};
    p.model_trans = {{{{0.72, 0.14, 0.06, 0.08}},
                      {{0.28, 0.60, 0.02, 0.10}},
                      {{0.35, 0.05, 0.55, 0.05}},
                      {{0.30, 0.20, 0.02, 0.48}}}};
    return p;
}

namespace {

struct NamedProfile {
    std::string_view name;
    DomainProfile (*make)();
};

constexpr std::array<NamedProfile, 13> kRegistry{{
    {"UrbanHUMINT", &UrbanHUMINT},
    {"Maritime", &Maritime},
    {"Airspace", &Airspace},
    {"VehicleConvoy", &VehicleConvoy},
    {"CityCameraSurveillance", &CityCameraSurveillance},
    {"CounterTerrorism", &CounterTerrorism},
    {"OrganisedCrimeNetwork", &OrganisedCrimeNetwork},
    {"FugitiveTracking", &FugitiveTracking},
    {"BorderPatrol", &BorderPatrol},
    {"IndoorVenue", &IndoorVenue},
    {"WarehouseAssets", &WarehouseAssets},
    {"WildlifeTelemetry", &WildlifeTelemetry},
    {"SportsPitch", &SportsPitch},
}};

}  // namespace

DomainProfile profile_by_name(std::string_view name) {
    const auto it = std::find_if(kRegistry.begin(), kRegistry.end(),
                                 [&](const NamedProfile& p) { return p.name == name; });
    return it != kRegistry.end() ? it->make() : UrbanHUMINT();
}

std::vector<std::string> profile_names() {
    std::vector<std::string> out;
    out.reserve(kRegistry.size());
    for (const auto& p : kRegistry) out.emplace_back(p.name);
    return out;
}

}  // namespace trace
