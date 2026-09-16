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
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "trace/core/profile.hpp"
#include "trace/core/report.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/spatial_index.hpp"
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
    /// Tracks binned by position, indexed in step with the `tracks` vector the
    /// detector is handed. Built once per scan and shared, so no detector has
    /// to walk every pair to find the nearby ones.
    const SpatialIndex* index{nullptr};

    /// Pairs of track indices within `radius`, using the shared index where
    /// one is available and falling back to all pairs where it is not.
    [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> near_pairs(
        Real radius, std::size_t n_tracks) const {
        if (index != nullptr && index->size() == n_tracks) {
            return index->pairs_within(radius);
        }
        std::vector<std::pair<std::size_t, std::size_t>> all;
        for (std::size_t i = 0; i < n_tracks; ++i) {
            for (std::size_t j = i + 1; j < n_tracks; ++j) all.emplace_back(i, j);
        }
        return all;
    }

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

    /// Drop per-track state for identities the engine is no longer carrying.
    ///
    /// Detectors keep history keyed by track id - visit deques, contact
    /// streaks, per-pair separation series, dedup sets - and a track id is
    /// never reused, so without this every one of those maps grows for the
    /// life of the process. Measured before the engine started calling it:
    /// resident memory rose about 10 MB per thousand scans with the live track
    /// count held flat, and never plateaued.
    ///
    /// `live` holds every id the engine still knows about, live or dormant.
    /// Anything else is gone for good and its state cannot be needed again.
    /// The default does nothing, for detectors that hold no per-track state.
    virtual void forget(const std::set<std::string>& /*live*/) {}
};

using DetectorPtr = std::unique_ptr<Detector>;

/// Erase every entry of `m` whose key is not a live track id.
template <typename Map>
void forget_by_id(Map& m, const std::set<std::string>& live) {
    for (auto it = m.begin(); it != m.end();) {
        it = live.count(it->first) != 0 ? std::next(it) : m.erase(it);
    }
}

/// Erase every entry of `m` keyed by a PAIR of track ids where either side has
/// gone. A pair is dead as soon as one of its members is.
template <typename Map>
void forget_by_pair(Map& m, const std::set<std::string>& live) {
    for (auto it = m.begin(); it != m.end();) {
        const bool keep = live.count(it->first.first) != 0 &&
                          live.count(it->first.second) != 0;
        it = keep ? std::next(it) : m.erase(it);
    }
}

/// Erase every entry of a dedup set whose tag embeds a dead track id.
///
/// The tags are built by the detectors themselves and carry the id as a
/// prefix up to `sep` - "A>B" for a transition between two tracks, "A@cell"
/// for a stop in a cell. Anything before the first separator is the id.
inline void forget_tags(std::set<std::string>& tags,
                        const std::set<std::string>& live, char sep) {
    for (auto it = tags.begin(); it != tags.end();) {
        const auto cut = it->find(sep);
        const std::string id = cut == std::string::npos ? *it : it->substr(0, cut);
        bool keep = live.count(id) != 0;
        // "A>B" names two tracks; both have to still exist.
        if (keep && sep == '>' && cut != std::string::npos) {
            keep = live.count(it->substr(cut + 1)) != 0;
        }
        it = keep ? std::next(it) : tags.erase(it);
    }
}

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
