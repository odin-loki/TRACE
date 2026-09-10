#include "trace/sim/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>

namespace trace::sim {
namespace {

constexpr Real kInf = std::numeric_limits<Real>::infinity();

/// Jonker-Volgenant style Hungarian algorithm on a rectangular matrix.
/// O(n^2 m) with n rows and m columns; exact.
Assignment hungarian(const std::vector<std::vector<Real>>& cost, Real max_cost) {
    const std::size_t n = cost.size();
    const std::size_t m = n > 0 ? cost[0].size() : 0;
    Assignment out;
    out.row_to_col.assign(n, -1);
    out.col_to_row.assign(m, -1);
    if (n == 0 || m == 0) return out;

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
                const Real c = cost[static_cast<std::size_t>(i0)][j];
                const Real cur = (std::isfinite(c) ? c : kInf) -
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
            if (!std::isfinite(delta)) break;  // no augmenting path remains

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
        if (!std::isfinite(cost[i][j]) || cost[i][j] > max_cost) continue;
        out.row_to_col[i] = static_cast<int>(j);
        out.col_to_row[j] = static_cast<int>(i);
        out.total_cost += cost[i][j];
        ++out.n_matched;
    }
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

}  // namespace trace::sim
