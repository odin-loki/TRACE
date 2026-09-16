// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/core/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>

namespace trace {
namespace {

constexpr Real kInf = std::numeric_limits<Real>::infinity();

/// Jonker-Volgenant style Hungarian algorithm on a rectangular matrix.
/// O(n^2 m) with n rows and m columns; exact.
///
/// Requires n <= m. The shortest-path search below grows one augmenting path
/// per row and needs a free column to terminate on; with more rows than
/// columns the later rows have none, `delta` stays infinite, and the search
/// breaks out having already shifted the potentials. The matching that comes
/// back still has the right NUMBER of pairs - every column is used - so
/// nothing downstream notices, but the pairs themselves are no longer the
/// cheapest ones. The `hungarian` wrapper below enforces that by transposing.
Assignment hungarian_le(const std::vector<std::vector<Real>>& cost, Real max_cost) {
    const std::size_t n = cost.size();
    const std::size_t m = n > 0 ? cost[0].size() : 0;
    Assignment out;
    out.row_to_col.assign(n, -1);
    out.col_to_row.assign(m, -1);
    if (n == 0 || m == 0) return out;

    // Forbidden pairs are replaced by a large FINITE cost for the duration of
    // the search, and dropped afterwards by the gate at the bottom.
    //
    // The search must always be able to reach a free column. Left as
    // infinities, a row with no admissible column makes `delta` infinite, and
    // the loop below then breaks out of a shortest-path search that never
    // terminated on a free column - with `j0` sitting on an occupied one, so
    // the sentinel test after the loop does not catch it and the augmentation
    // runs along a path that is not an augmenting path, evicting whichever row
    // already held that column regardless of cost. Worse, the potentials were
    // already shifted before the break, so the dual invariant is broken for
    // every row after it too. Measured against exhaustive search over random
    // gated matrices, 6.5% of 2x2 and 10.7% of 4x4 instances came back
    // strictly costlier, always with the right number of pairs - so no caller
    // could see it.
    //
    // `big_m` exceeds the total of every admissible cost in the matrix, so a
    // matching that uses one forbidden pair is dearer than any matching that
    // uses none. Minimising therefore takes as many admissible pairs as exist
    // first and the cheapest such matching second, which is the objective
    // `match` documents. Pairs outside the gate are forbidden here as well as
    // at the bottom: a row spent on a pair the gate will discard is a row that
    // could have been matched admissibly somewhere else.
    //
    // That argument needs the admissible costs to be non-negative, and they are
    // not always. Reacquisition scores a candidate with a Gaussian LOG-density
    // and negates it (src/core/pmbm.cpp:630), so its costs run negative
    // whenever the position sigma is below one metre - which is most of the
    // time. A negative total made `big_m` negative too, and a forbidden pair
    // then looked CHEAPER than every real one: on {{-3, inf}, {inf, -3}} the
    // solver took both infinities, the gate below dropped them, and two tracks
    // that should have been reacquired came back unmatched. Shifting the
    // admissible costs up by their own minimum restores the precondition. Each
    // candidate assignment fills all n rows, so subtracting a constant from
    // every admissible cell moves same-cardinality assignments by the same
    // amount and cannot reorder them; `total_cost` below is accumulated from
    // the ORIGINAL costs, so nothing the caller sees is shifted.
    const auto admissible = [max_cost](Real c) {
        // One predicate, used here and by the gate at the bottom. Two separate
        // tests could disagree - on a NaN `max_cost` both `c <= max_cost` and
        // `c > max_cost` are false, which forbade every pair during the search
        // and then dropped none of them afterwards.
        return std::isfinite(c) && c <= max_cost;
    };

    Real lowest = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < m; ++j) {
            if (admissible(cost[i][j])) lowest = std::min(lowest, cost[i][j]);
        }
    }
    const Real shift = lowest;   // <= 0; zero when nothing is negative

    Real admissible_total = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < m; ++j) {
            if (admissible(cost[i][j])) admissible_total += cost[i][j] - shift;
        }
    }
    // Strictly greater than the total, not merely one more than it: past 2^53
    // adding one is a no-op and the domination stops being strict.
    const Real big_m =
        admissible_total +
        std::max(1.0, admissible_total * 8.0 * std::numeric_limits<Real>::epsilon());

    std::vector<std::vector<Real>> w(n, std::vector<Real>(m, big_m));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < m; ++j) {
            if (admissible(cost[i][j])) w[i][j] = cost[i][j] - shift;
        }
    }

    // Potentials, indexed from 1 for the sentinel used by the shortest-path
    // search below. u for rows, v for columns.
    std::vector<Real> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<long> p(m + 1, -1), way(m + 1, 0);

    for (std::size_t i = 0; i < n; ++i) {
        p[m] = static_cast<long>(i);
        std::size_t j0 = m;
        std::vector<Real> minv(m + 1, kInf);
        std::vector<char> used(m + 1, 0);

        do {
            used[j0] = 1;
            const long i0 = p[j0];
            Real delta = kInf;
            std::size_t j1 = m;

            for (std::size_t j = 0; j < m; ++j) {
                if (used[j]) continue;
                const Real cur = w[static_cast<std::size_t>(i0)][j] -
                                 u[static_cast<std::size_t>(i0)] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = static_cast<long>(j0);
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            // Every entry of `w` is finite, so a free column is always
            // reachable and `delta` is always finite. The guard stays as an
            // assertion of that rather than as control flow the search relies
            // on; reaching it would mean the invariant above had been broken.
            if (!std::isfinite(delta)) break;

            for (std::size_t j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[static_cast<std::size_t>(p[j])] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != -1);

        if (j0 == m) continue;
        // Augment along the path found.
        while (j0 != m) {
            const std::size_t j1 = static_cast<std::size_t>(way[j0]);
            p[j0] = p[j1];
            j0 = j1;
        }
    }

    for (std::size_t j = 0; j < m; ++j) {
        if (p[j] < 0) continue;
        const auto i = static_cast<std::size_t>(p[j]);
        // The solver matches everything it can; the gate is applied afterwards,
        // so a pair that is technically optimal but too far apart is dropped.
        if (!admissible(cost[i][j])) continue;
        out.row_to_col[i] = static_cast<int>(j);
        out.col_to_row[j] = static_cast<int>(i);
        out.total_cost += cost[i][j];
        ++out.n_matched;
    }
    return out;
}

/// Exact matching for ANY shape, by solving the transpose when there are more
/// rows than columns.
///
/// Matching is symmetric - "which row for each column" is the same problem as
/// "which column for each row" - so transposing costs nothing but the copy and
/// puts the solver back inside its precondition. Measured against exhaustive
/// search over 4000 random matrices per shape: without this, 87% of 4x3 cases
/// and 96% of 6x2 cases came back sub-optimal, at a mean excess of 3.9 and 5.6
/// on a cost scale of 10. Square and wide cases were, and remain, exact.
Assignment hungarian(const std::vector<std::vector<Real>>& cost, Real max_cost) {
    const std::size_t n = cost.size();
    const std::size_t m = n > 0 ? cost[0].size() : 0;
    if (n <= m) return hungarian_le(cost, max_cost);

    std::vector<std::vector<Real>> t(m, std::vector<Real>(n, kInf));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < m; ++j) t[j][i] = cost[i][j];
    }
    Assignment a = hungarian_le(t, max_cost);

    // Rows of the transpose are the original columns, so the two maps swap.
    Assignment out;
    out.row_to_col = std::move(a.col_to_row);
    out.col_to_row = std::move(a.row_to_col);
    out.total_cost = a.total_cost;
    out.n_matched = a.n_matched;
    return out;
}

/// Globally-sorted greedy: consider every admissible pair cheapest-first and
/// take it if both sides are still free.
Assignment greedy(const std::vector<std::vector<Real>>& cost, Real max_cost) {
    const std::size_t n = cost.size();
    const std::size_t m = n > 0 ? cost[0].size() : 0;
    Assignment out;
    out.row_to_col.assign(n, -1);
    out.col_to_row.assign(m, -1);
    if (n == 0 || m == 0) return out;

    std::vector<std::tuple<Real, std::size_t, std::size_t>> pairs;
    pairs.reserve(n * m);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < m; ++j) {
            const Real c = cost[i][j];
            if (std::isfinite(c) && c <= max_cost) pairs.emplace_back(c, i, j);
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

    for (const auto& [c, i, j] : pairs) {
        if (out.row_to_col[i] >= 0 || out.col_to_row[j] >= 0) continue;
        out.row_to_col[i] = static_cast<int>(j);
        out.col_to_row[j] = static_cast<int>(i);
        out.total_cost += c;
        ++out.n_matched;
    }
    return out;
}

}  // namespace

Assignment match(const std::vector<std::vector<Real>>& cost, Real max_cost,
                 std::size_t exact_limit) {
    const std::size_t n = cost.size();
    const std::size_t m = n > 0 ? cost[0].size() : 0;
    if (n == 0 || m == 0) {
        Assignment empty;
        empty.row_to_col.assign(n, -1);
        empty.col_to_row.assign(m, -1);
        return empty;
    }
    if (std::max(n, m) <= exact_limit) return hungarian(cost, max_cost);
    return greedy(cost, max_cost);
}

Assignment match_points(const std::vector<Vec2>& truth,
                        const std::vector<Vec2>& tracks, Real max_distance,
                        std::size_t exact_limit) {
    std::vector<std::vector<Real>> cost(truth.size(),
                                        std::vector<Real>(tracks.size(), kInf));
    for (std::size_t i = 0; i < truth.size(); ++i) {
        for (std::size_t j = 0; j < tracks.size(); ++j) {
            const Real d = distance(truth[i], tracks[j]);
            // Gate here as well as in match(): an infinite cost keeps the
            // solver from ever considering an impossible pair.
            if (d <= max_distance) cost[i][j] = d;
        }
    }
    return match(cost, max_distance, exact_limit);
}

}  // namespace trace
