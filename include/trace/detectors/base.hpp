// TRACE — detector plugin interface.
//
// A detector reads the current track set and raises events. It owns whatever
// cross-scan state it needs (visit histories, separation series, streak
// counters) and is registered with the engine at runtime, so a deployment can
// add a domain-specific behaviour without touching the tracking core.
#pragma once

#include <algorithm>
#include <optional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "trace/core/profile.hpp"
#include "trace/core/report.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/track.hpp"

namespace trace {

/// Everything a detector may read about the current scan.
struct DetectorContext {
    Real timestamp{0.0};
    int scan_index{0};
    const DomainProfile* profile{nullptr};
    const std::vector<Vec2>* high_value_locations{nullptr};
    const std::unordered_map<std::string, Real>* betweenness{nullptr};
    const std::vector<Cluster>* clusters{nullptr};
    Rng* rng{nullptr};

    [[nodiscard]] Real betweenness_of(const std::string& tid) const {
        if (betweenness == nullptr) return 0.0;
        const auto it = betweenness->find(tid);
        return it != betweenness->end() ? it->second : 0.0;
    }
};

class Detector {
public:
    virtual ~Detector() = default;

    /// Stable identifier, used for registration and for tagging events.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Examine this scan's confirmed tracks and raise whatever fires.
    virtual std::vector<DetectionEvent> detect(
        const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) = 0;

    /// Rendezvous predictions, if this detector makes any. Kept separate from
    /// generic events because warnings carry an ETA and drive different UI.
    virtual std::vector<RendezvousWarning> rendezvous(
        const std::vector<TrackPtr>& /*tracks*/, const DetectorContext& /*ctx*/) {
        return {};
    }

    /// Role assignments, if this detector infers any.
    virtual std::vector<NetworkRole> roles(
        const std::vector<TrackPtr>& /*tracks*/, const DetectorContext& /*ctx*/) {
        return {};
    }
};

using DetectorPtr = std::unique_ptr<Detector>;

/// Helper for building an event without a wall of field assignments.
inline DetectionEvent make_event(std::string type, std::string detector,
                                 std::vector<std::string> tracks,
                                 Severity severity, Real timestamp) {
    DetectionEvent e;
    e.type = std::move(type);
    e.detector = std::move(detector);
    e.tracks = std::move(tracks);
    e.severity = severity;
    e.timestamp = timestamp;
    return e;
}

}  // namespace trace
