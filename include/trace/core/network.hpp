// TRACE — co-location network analysis.
//
// Who keeps being seen with whom, and which entity sits on the paths between
// others. Betweenness is the useful measure here rather than raw contact count:
// the entity that connects otherwise-separate groups is structurally important
// even when it meets fewer people than anyone else.
#pragma once

#include <algorithm>
#include <deque>
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
    explicit NetworkAnalyser(Real coloc_distance) : coloc_dist_(coloc_distance) {}

    /// Accumulate this scan's contacts and return the current clusters.
    std::vector<Cluster> analyse(const std::vector<TrackPtr>& tracks,
                                 Real timestamp,
                                 const SpatialIndex* index = nullptr);

    /// Betweenness by track id, as computed on the most recent scan.
    [[nodiscard]] const std::unordered_map<std::string, Real>& betweenness() const {
        return latest_betweenness_;
    }

private:
    Real coloc_dist_{350.0};
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

private:
    std::unordered_map<std::string, std::deque<Real>> history_;
};

/// Recommends where to point the next collection asset.
std::vector<CollectionTask> schedule_collection(
    const std::vector<TrackPtr>& tracks, const DomainProfile& profile,
    std::size_t max_tasks = 3);

}  // namespace trace
