// TRACE — truth-to-track assignment for scoring.
//
// Scoring a tracker means deciding which track corresponds to which real
// entity, and that decision is itself an assignment problem. Getting it wrong
// invents errors: assigning each entity to its nearest track independently lets
// one track "cover" several entities and reports identity switches that never
// happened. That is exactly what inflated the warehouse scenario's switch count,
// where eight entities converge inside the match radius.
#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "trace/core/types.hpp"

namespace trace::sim {

/// Result of matching rows (truth) to columns (tracks).
struct Assignment {
    /// For each row, the column it was matched to, or -1.
    std::vector<int> row_to_col;
    /// For each column, the row it was matched to, or -1.
    std::vector<int> col_to_row;
    Real total_cost{0.0};
    std::size_t n_matched{0};
};

/// Minimum-cost matching, gated: a pair whose cost exceeds `max_cost` is never
/// matched, which is what makes "close enough to count" meaningful.
///
/// Uses the Hungarian algorithm for small problems, where an exact optimum is
/// affordable, and falls back to globally-sorted greedy above `exact_limit`.
/// Greedy is not optimal but is order-independent and close in practice; the
/// alternative on a 150-person MOT20 frame is an O(n^3) solve per frame across
/// thousands of frames.
Assignment match(const std::vector<std::vector<Real>>& cost, Real max_cost,
                 std::size_t exact_limit = 64);

/// Convenience: build the cost matrix from two point sets and match them.
Assignment match_points(const std::vector<Vec2>& truth,
                        const std::vector<Vec2>& tracks, Real max_distance,
                        std::size_t exact_limit = 64);

}  // namespace trace::sim
