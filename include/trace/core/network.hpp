// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — co-location network analysis.
//
// Who keeps being seen with whom, and which entity sits on the paths between
// others. Betweenness is the useful measure here rather than raw contact count:
// the entity that connects otherwise-separate groups is structurally important
// even when it meets fewer people than anyone else.
#pragma once

#include <algorithm>
#include <deque>
#include <cmath>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "trace/core/report.hpp"
#include "trace/core/spatial_index.hpp"
#include "trace/core/track.hpp"

namespace trace {

/// Brandes betweenness centrality on an unweighted graph given as adjacency
/// lists.
///
/// O(V*E). The earlier version took a dense adjacency matrix and scanned all V
/// columns for every dequeued vertex, making it O(V^3) - about 20 million
/// operations per scan at 270 tracks, which dominated the entire engine.
std::vector<Real> betweenness_centrality(
    const std::vector<std::vector<std::size_t>>& adjacency);

class NetworkAnalyser {
public:
    /// `memory_scans` is the half-life of a contact: how long a single
    /// proximity event goes on contributing to the graph. Without one the
    /// graph only ever gains edges, and in any scene that runs long enough
    /// every track ends up adjacent to every other - at which point
    /// betweenness is uniformly zero and the network says nothing. It also
    /// bounds the map, which otherwise keeps a row for every track that has
    /// ever existed.
    explicit NetworkAnalyser(Real coloc_distance, int memory_scans = 60)
        : coloc_dist_(coloc_distance),
          decay_(std::pow(0.5, 1.0 / std::max(memory_scans, 1))) {}

    /// Accumulate this scan's contacts and return the current clusters.
    std::vector<Cluster> analyse(const std::vector<TrackPtr>& tracks,
                                 Real timestamp,
                                 const SpatialIndex* index = nullptr);

    /// Betweenness by track id, as computed on the most recent scan.
    /// Drop per-track history for identities the manager no longer carries.
    /// `bc_history_` is keyed by track id and kept a deque for every track
    /// that ever existed; `adjacency_` ages out by weight, but only for pairs
    /// that keep being seen.
    void forget(const std::set<std::string>& live) {
        for (auto it = bc_history_.begin(); it != bc_history_.end();) {
            it = live.count(it->first) != 0 ? std::next(it) : bc_history_.erase(it);
        }
        for (auto row = adjacency_.begin(); row != adjacency_.end();) {
            if (live.count(row->first) == 0) { row = adjacency_.erase(row); continue; }
            for (auto e = row->second.begin(); e != row->second.end();) {
                e = live.count(e->first) != 0 ? std::next(e) : row->second.erase(e);
            }
            ++row;
        }
    }

    [[nodiscard]] const std::unordered_map<std::string, Real>& betweenness() const {
        return latest_betweenness_;
    }

private:
    Real coloc_dist_{350.0};
    Real decay_{1.0};
    std::unordered_map<std::string, std::unordered_map<std::string, Real>> adjacency_;
    std::unordered_map<std::string, std::deque<Real>> bc_history_;
    std::unordered_map<std::string, Real> latest_betweenness_;
};

/// Watches a track's anomaly series and raises escalations. A single high score
/// is noise; a rising trend, or a dip-then-spike, is a pattern.
class AnomalyEscalator {
public:
    std::vector<Alert> update(const std::string& track_id, Real score,
                              Priority tier);

    /// Drop the score history of identities that are gone. Keyed by track id
    /// and previously kept one deque per track ever scored.
    void forget(const std::set<std::string>& live) {
        for (auto it = history_.begin(); it != history_.end();) {
            it = live.count(it->first) != 0 ? std::next(it) : history_.erase(it);
        }
    }

private:
    std::unordered_map<std::string, std::deque<Real>> history_;
};

/// Recommends where to point the next collection asset.
std::vector<CollectionTask> schedule_collection(
    const std::vector<TrackPtr>& tracks, const DomainProfile& profile,
    std::size_t max_tasks = 3);

}  // namespace trace
