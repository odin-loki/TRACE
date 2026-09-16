// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — structured output of one scan.
//
// The reference implementation returned nested dictionaries. Here the report is
// typed: a GUI, a CSV writer and a test all consume the same struct, and adding
// a field is a compile error at every consumer rather than a silent KeyError.
#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "trace/core/types.hpp"

namespace trace {

/// A named numeric detail attached to an event (separation, angle, dwell...).
struct Metric {
    std::string name;
    Real value{0.0};
};

/// Anything a detector wants to raise: a brush pass, a loiter, a mode change.
struct DetectionEvent {
    std::string type;            ///< e.g. "BRUSH_PASS"
    std::string detector;        ///< which detector raised it
    std::vector<std::string> tracks;
    Severity severity{Severity::MEDIUM};
    Real timestamp{0.0};
    std::optional<Vec2> location;
    std::vector<Metric> metrics;
    std::string note;

    [[nodiscard]] Real metric(std::string_view name, Real fallback = 0.0) const {
        for (const auto& m : metrics) {
            if (m.name == name) return m.value;
        }
        return fallback;
    }
};

/// A predicted meeting between two entities.
struct RendezvousWarning {
    std::string track_a;
    std::string track_b;
    Real eta_s{0.0};
    Real current_sep_m{0.0};
    std::string method;          ///< which of the stacked predictors fired
    Real confidence{0.0};
    std::optional<Vec2> location;
    Priority priority{Priority::MEDIUM};

    [[nodiscard]] Real eta_min() const { return eta_s / 60.0; }
};

/// Inferred position of an entity within its network.
struct NetworkRole {
    std::string track;
    /// HANDLER / COURIER / ASSET / ASSOCIATE / UNKNOWN.
    ///
    /// ASSOCIATE was missing from this list while the classifier emitted it -
    /// it is the "has contacts but fits none of the three" case, and a consumer
    /// switching on the documented set would have fallen through on it.
    std::string role;
    int n_contacts{0};
    Real avg_speed_mps{0.0};
    Real betweenness{0.0};
    Real confidence{0.0};
    Severity severity{Severity::INFO};
};

/// A group of entities that keep being seen together.
struct Cluster {
    int cluster_id{0};
    std::vector<std::string> member_ids;
    Vec2 centre{};
    std::string hub_track;
    std::unordered_map<std::string, Real> betweenness;
    std::unordered_map<std::string, Real> weighted_degree;
    bool recurring{false};
    Severity significance{Severity::MEDIUM};
};

/// Dempster-Shafer fusion of the evidence behind one track.
struct Credibility {
    Real belief{0.5};
    Real plausibility{0.5};
    Real conflict{0.0};
};

/// Where the threat score came from. Reported alongside the score itself
/// because an operator acting on a number they cannot decompose is a liability.
struct ThreatBreakdown {
    Real existence{0.0};
    Real possibility{0.0};
    Real pol_anomaly{0.0};
    Real detection_density{0.0};
    Real hvl_proximity{0.0};
    Real motion{0.0};
    Real persistence{0.0};
    Real mismatch_penalty{0.0};
};

struct ThreatScore {
    Real mean{0.0};
    Real stddev{0.0};
    Real p90{0.0};
    Real p95{0.0};
    Priority priority{Priority::MONITOR};
    Real ema{0.0};
    int persistence{0};
    ThreatBreakdown breakdown{};
};

/// A single forecast step.
struct ForecastStep {
    Real timestamp{0.0};
    Vec2 position{};
    Real uncertainty_m{0.0};
};

/// Everything known about one confirmed track this scan.
struct TargetReport {
    std::string track_id;
    std::optional<std::string> parent_id;
    Real born_at{0.0};
    Real last_seen{0.0};
    Vec2 position{};
    Vec2 velocity_mps{};
    Real speed_mps{0.0};
    Real position_uncertainty_m{0.0};
    Real existence{0.0};
    Real possibility{0.0};
    Real possibility_mismatch{0.0};
    int age_scans{0};
    int hits{0};
    int misses{0};
    Real measurement_rate{0.0};
    Real observation_quality{0.0};
    std::string dominant_model;
    Credibility credibility{};
    ThreatScore threat{};
    std::vector<ForecastStep> forecast;
};

/// An escalation raised by watching a track's anomaly trend over time.
struct Alert {
    std::string track;
    std::string reason;          ///< SPIKE / ESCALATING / COUNTER_SURVEILLANCE
    Real score{0.0};
    Priority tier{Priority::MEDIUM};
};

/// A recommendation about where to point the next sensor.
struct CollectionTask {
    std::string track_id;
    Modality recommended_modality{Modality::GEOINT};
    Real expected_info_gain{0.0};
    Real current_uncertainty_m{0.0};
};

/// Two sensors that persistently disagree about the same entity.
struct SensorConflict {
    std::string source_a;
    std::string source_b;
    Real mean_disagreement_m{0.0};
    Real rate{0.0};
    int observations{0};
};

/// A sensor whose reports are consistently offset in one direction.
struct SensorBias {
    std::string source_id;
    Vec2 offset_m{};
    Real magnitude_m{0.0};
    Real significance{0.0};
    int samples{0};
    /// Residual points against the consensus - the signature of the sensor
    /// that is dragging the tracks rather than being dragged by them.
    bool minority_direction{false};
};

/// A sensor whose reports chronically match nothing.
struct SensorOrphaned {
    std::string source_id;
    Real unassigned_rate{0.0};
    int reports{0};
};

/// Cross-cutting observations that do not belong to a single detector.
struct OperationalIntel {
    std::vector<SensorBias> sensor_biases;
    std::vector<SensorOrphaned> orphaned_sources;
    std::vector<SensorConflict> sensor_conflicts;
    std::vector<std::string> possibility_mismatch_tracks;
    std::vector<std::string> high_speed_tracks;
    std::vector<std::string> dwelling_tracks;
    std::vector<std::string> boundary_tracks;
};

/// The complete output of one call to Engine::ingest.
struct ScanReport {
    int scan{0};
    Real timestamp{0.0};
    std::string domain;
    int n_observations{0};
    int n_tracks{0};
    int n_components{0};
    int n_dormant{0};
    Real clutter_rate{0.0};
    Real latency_ms{0.0};

    /// No source reported anything this scan, in a scene where sources had
    /// been reporting steadily. The engine treats that as a gap in coverage
    /// rather than as evidence that everything left, and says so here because
    /// the two are genuinely different and only the operator can confirm
    /// which.
    bool coverage_gap{false};
    /// Where the scan's time went, by stage. Present because the engine's cost
    /// profile is not obvious: tracking is linear in track count while some
    /// detectors are not, and which dominates decides whether a deployment is
    /// feasible at its intended scale.
    std::vector<std::pair<std::string, Real>> stage_ms;

    std::vector<TargetReport> targets;
    std::vector<RendezvousWarning> rendezvous;
    std::vector<Cluster> clusters;
    std::vector<DetectionEvent> events;      ///< all detector output, flattened
    std::vector<NetworkRole> network_roles;
    std::vector<Alert> alerts;
    std::vector<CollectionTask> sensor_schedule;
    OperationalIntel operational;

    /// Events from one named detector.
    [[nodiscard]] std::vector<const DetectionEvent*> events_from(
        std::string_view detector) const {
        std::vector<const DetectionEvent*> out;
        for (const auto& e : events) {
            if (e.detector == detector) out.push_back(&e);
        }
        return out;
    }

    /// Events of one type, regardless of which detector raised them.
    [[nodiscard]] std::vector<const DetectionEvent*> events_of_type(
        std::string_view type) const {
        std::vector<const DetectionEvent*> out;
        for (const auto& e : events) {
            if (e.type == type) out.push_back(&e);
        }
        return out;
    }
};

}  // namespace trace
