// TRACE — DomainProfile: the single object that retargets the engine.
//
// Every subsystem reads its constants from the active profile rather than from
// file-scope globals. Swapping the profile reinterprets the same code for a
// different domain — this is the mechanism behind every use case in
// docs/USE_CASES.md.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "trace/core/types.hpp"

namespace trace {

/// One Mixed Ornstein-Uhlenbeck motion regime.
///
/// theta is the mean-reversion rate on velocity, in units of 1/second - its
/// reciprocal is how long the entity holds a heading. sigma is the velocity
/// diffusion, in m/s^1.5. The steady-state speed is sigma/sqrt(2*theta).
///
/// Prefer building these with motion() below: raw theta/sigma pairs are almost
/// impossible to sanity-check, and getting them wrong is silent - the filter
/// simply stops being able to predict, and the failure shows up much later as
/// unexplained identity switches.
struct MotionModel {
    std::string name;
    Real theta{0.30};
    Real sigma{2.0};
};

/// Build a motion regime from quantities a person can actually verify:
/// how many seconds an entity holds its heading, and how fast it typically
/// moves. Both in SI.
inline MotionModel motion(std::string name, Real heading_hold_s,
                          Real typical_speed_mps) {
    const Real theta = 1.0 / std::max(heading_hold_s, 1e-6);
    const Real sigma = typical_speed_mps * std::sqrt(2.0 * theta);
    return MotionModel{std::move(name), theta, sigma};
}

inline constexpr int kNumModels = 4;

using ModelTransition = std::array<std::array<Real, kNumModels>, kNumModels>;

/// Fully parameterised domain bundle.
struct DomainProfile {
    std::string name{"UrbanHUMINT"};

    // -- Scan / sensor ------------------------------------------------------
    Real scan_dt_s{60.0};      ///< nominal seconds between scans
    Real pos_noise_m{5.0};     ///< 1-sigma position measurement noise
    Real meas_noise_var{25.0}; ///< R diagonal used by the filter's likelihood

    /// Learn how wrong `meas_noise_var` is, per sensor, from the normalised
    /// innovations the engine already computes, and scale it accordingly.
    ///
    /// Off by default: the estimate is sound but it changes the association
    /// gate, and a profile that has been tuned against a fixed assumption
    /// should keep it until someone has measured the difference.
    bool adaptive_meas_noise{false};

    std::array<Real, static_cast<std::size_t>(Modality::Count)> modality_weights{
        0.95, 0.82, 0.75, 0.65, 0.55};

    // -- PMBM filter --------------------------------------------------------
    Real p_detection{0.85};
    Real p_survival{0.995};
    // Birth sits BELOW confirm on purpose: a newborn track is not reportable
    // until a second detection corroborates it. With birth above confirm, a
    // single false alarm becomes a confirmed track for a scan, and the ghost
    // rate tracks the sensor's false-alarm rate exactly. Profiles that would
    // rather chase every sighting (see FugitiveTracking) invert this knowingly.
    Real r_birth{0.45};
    Real r_confirm{0.55};
    Real r_prune{0.05};
    Real r_dormant{0.04};
    int  dormant_timeout{40};
    /// Hard limit on how long a track may coast unseen before it is retired,
    /// regardless of existence probability. Where p_detection is low a missed
    /// scan is almost uninformative - r decays by ~0.0002 per miss - so
    /// evidence alone never retires anything and stale tracks pile up.
    /// Negative derives it from dormant_timeout x scan_dt_s.
    Real max_coast_s{-1.0};
    /// How long a dormant track can still be reacquired by extrapolating its
    /// last known motion. Pattern-of-life reacquisition answers "days later, at
    /// his usual place"; this answers "moments later, where he was heading",
    /// which is the only cue available to a track too short-lived to have a
    /// baseline. Negative derives it from the scan period.
    Real reacquire_kinematic_s{-1.0};
    Real gate_chi2{13.816};    ///< chi2(0.999, df=2)
    /// Two tracks closer than this, and statistically indistinguishable, are
    /// treated as one entity. Negative means "derive from pos_noise_m".
    Real merge_distance_m{-1.0};
    Real merge_chi2{4.605};    ///< chi2(0.90, df=2)
    /// Require a corroborating unassigned detection in the previous scan
    /// before a new track is born. Kills isolated false alarms, which cannot
    /// repeat plausibly, while real entities produce detections every scan.
    /// Profiles where a lone sighting is precious turn this off.
    bool two_point_initiation{true};
    /// How far a real entity could have moved between scans. Negative derives
    /// it from the fastest motion model's steady-state speed.
    Real birth_gate_m{-1.0};
    int  gibbs_sweeps{14};
    /// Hard ceiling on simultaneously maintained tracks. Beyond this the
    /// weakest are dropped, so it is a real limit on how crowded a scene can
    /// be - a stadium concourse needs hundreds, a convoy needs a dozen. It was
    /// a hardcoded 80 with nothing to say so.
    int  max_tracks{80};
    /// How much weight association gives to appearance relative to position.
    /// Zero ignores descriptors entirely, which is the right default: most
    /// domains have no appearance evidence, and inventing a term for absent
    /// evidence would only add noise.
    Real appearance_weight{0.0};
    /// Appearance distance, as (1 - cosine similarity), treated as Gaussian
    /// with this standard deviation. Smaller makes appearance more decisive.
    Real appearance_sigma{0.35};
    /// Momentum of a track's running appearance model. Near 1 remembers a long
    /// way back, which is what carries identity across an occlusion.
    Real appearance_momentum{0.9};
    /// How strongly appearance similarity counts when deciding whether a new
    /// detection is a dormant identity resurfacing.
    Real reacquire_appearance_gain{4.0};
    /// Below this cosine similarity a reacquisition is refused outright.
    /// Negative disables the check, which is correct where descriptors are
    /// weak or absent - refusing on weak evidence loses identities needlessly.
    Real reacquire_min_similarity{-1.0};
    int  n_particles{320};

    // -- Motion models ------------------------------------------------------
    //                      name          holds heading   typical speed
    std::array<MotionModel, kNumModels> mou_models{{
        motion("foot",              90.0,   1.4),
        motion("vehicle",          120.0,  12.0),
        motion("stationary",        30.0,   0.15),
        motion("fast",             180.0,  25.0),
    }};

    ModelTransition model_trans{{
        {{0.85, 0.10, 0.04, 0.01}},
        {{0.05, 0.88, 0.02, 0.05}},
        {{0.15, 0.05, 0.78, 0.02}},
        {{0.02, 0.20, 0.01, 0.77}},
    }};

    // -- Rendezvous (stacked warner) ---------------------------------------
    Real rv_threshold_m{150.0};        ///< distance that counts as a meeting
    int  rv_horizon_scans{4};          ///< short-horizon Monte-Carlo depth
    Real rv_warning_horizon_s{1800.0}; ///< 30-minute closest-approach window
    int  rv_sep_rate_window{8};        ///< scans used for the closure-rate fit
    Real rv_pol_window_s{3600.0};      ///< pattern-of-life lookahead

    // -- Behaviour detection ------------------------------------------------
    Real brush_pass_m{60.0};
    Real parallel_route_m{80.0};
    Real parallel_vel_cos{0.97};
    int  parallel_scans{6};
    Real mode_trans_m{50.0};
    int  mode_trans_scans{2};
    Real loiter_mult{3.0};
    Real loiter_min_s{300.0};
    Real cover_stop_m{300.0};
    Real cover_stop_hvl_m{800.0};
    /// Surveillance-detection-route window: how many recent positions the
    /// winding-number test looks back over, and how many full turns count.
    /// The window must span a whole loop at the domain's own speed - too short
    /// and the test cannot reach threshold no matter what the entity does.
    int  sdr_window{16};
    Real sdr_turns{0.65};
    Real chokepoint_m{40.0};
    int  chokepoint_n{3};
    Real dead_drop_min_s{60.0};
    Real dead_drop_max_s{1800.0};

    // -- Network analysis ---------------------------------------------------
    Real coloc_dist_m{350.0};
    Real courier_speed_thresh{3.0};
    int  handler_stable_scans{10};

    // -- Pattern of life ----------------------------------------------------
    int  pol_components{5};
    int  pol_min_obs{15};
    int  pol_refit_interval{5};

    // -- Threat scoring -----------------------------------------------------
    std::array<Real, 8> threat_weights{0.23, 0.18, 0.13, 0.13,
                                       0.08, 0.08, 0.10, 0.07};
    Real hvl_radius_m{600.0};
    int  threat_mc_samples{250};

    [[nodiscard]] Real modality_weight(Modality m) const {
        return modality_weights[static_cast<std::size_t>(m)];
    }
};

// -- Built-in profiles --------------------------------------------------------
// The first four are ports of the reference implementation's presets. The rest
// were specified in the law-enforcement brief but never shipped as code; they
// are real profiles here.

DomainProfile UrbanHUMINT();
DomainProfile Maritime();
DomainProfile Airspace();
DomainProfile VehicleConvoy();

DomainProfile CityCameraSurveillance();
DomainProfile CounterTerrorism();
DomainProfile OrganisedCrimeNetwork();
DomainProfile FugitiveTracking();
DomainProfile BorderPatrol();

// Civil / commercial profiles — same engine, non-security domains.
DomainProfile IndoorVenue();      ///< retail, transit hall, hospital, campus
DomainProfile WarehouseAssets();  ///< RFID/BLE pallet and forklift tracking
DomainProfile WildlifeTelemetry();///< sparse satellite collar fixes
DomainProfile SportsPitch();      ///< dense, fast, heavy occlusion

/// Look a profile up by name; returns UrbanHUMINT() for an unknown name.
DomainProfile profile_by_name(std::string_view name);

/// Names of every registered built-in profile.
std::vector<std::string> profile_names();

}  // namespace trace
