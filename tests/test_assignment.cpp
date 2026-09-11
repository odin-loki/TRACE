// Scoring is itself an assignment problem. If it is wrong, every performance
// number in this repository is wrong with it, so it gets checked against cases
// whose answer can be worked out by hand.
#include "trace/core/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "trace/core/rng.hpp"

#include "test_harness.hpp"

using namespace trace;

namespace {

void test_one_to_one() {
    // Two entities, one track between them: exactly one may claim it.
    const Assignment a = match_points({{0, 0}, {10, 0}}, {{5, 0}}, 20.0);
    CHECK(a.n_matched == 1);
    CHECK((a.row_to_col[0] >= 0) != (a.row_to_col[1] >= 0));
}

void test_optimal_beats_nearest_first() {
    // Truth at 0 and 3; tracks at 1 and 4. Nearest-first for the first entity
    // grabs the track at 1, which happens to be optimal here; the real test is
    // that the total cost is minimal and the pairing is not crossed.
    const Assignment a = match_points({{0, 0}, {3, 0}}, {{1, 0}, {4, 0}}, 10.0);
    CHECK(a.row_to_col[0] == 0);
    CHECK(a.row_to_col[1] == 1);
    CHECK_NEAR(a.total_cost, 2.0, 1e-9);
}

void test_does_not_cross_a_pair() {
    // Two entities whose nearest tracks are each other's: the optimum keeps
    // them uncrossed, and a naive per-entity nearest search would too - but a
    // per-entity search that ran in the other order would not.
    const Assignment a = match_points({{0, 0}, {10, 0}}, {{9.5, 0}, {0.5, 0}}, 20.0);
    CHECK(a.row_to_col[0] == 1);
    CHECK(a.row_to_col[1] == 0);
    CHECK_NEAR(a.total_cost, 1.0, 1e-9);
}

void test_gate_excludes() {
    const Assignment a = match_points({{0, 0}}, {{100, 0}}, 10.0);
    CHECK(a.n_matched == 0);
    CHECK(a.row_to_col[0] == -1);
}

void test_empty_inputs() {
    CHECK(match_points({}, {{1, 1}}, 10.0).n_matched == 0);
    CHECK(match_points({{1, 1}}, {}, 10.0).n_matched == 0);
    CHECK(match_points({}, {}, 10.0).n_matched == 0);
}

void test_rectangular() {
    // More tracks than entities, and vice versa: everything that can be matched
    // is, and nothing is matched twice.
    const Assignment a = match_points({{0, 0}, {10, 0}},
                                      {{0, 1}, {10, 1}, {50, 50}, {60, 60}}, 5.0);
    CHECK(a.n_matched == 2);
    CHECK(a.col_to_row[2] == -1);
    CHECK(a.col_to_row[3] == -1);
}

void test_large_problem_stays_one_to_one() {
    // Above the exact limit the solver falls back to greedy; the result need
    // not be optimal but must still be a valid matching.
    std::vector<Vec2> truth, tracks;
    for (int i = 0; i < 200; ++i) {
        truth.push_back(Vec2{static_cast<Real>(i) * 10.0, 0.0});
        tracks.push_back(Vec2{static_cast<Real>(i) * 10.0 + 2.0, 0.0});
    }
    const Assignment a = match_points(truth, tracks, 20.0);

    std::vector<int> claimed(tracks.size(), 0);
    for (const int j : a.row_to_col) {
        if (j >= 0) ++claimed[static_cast<std::size_t>(j)];
    }
    for (const int c : claimed) CHECK(c <= 1);
    std::printf("  200x200 fallback: %zu matched, cost %.1f\n", a.n_matched,
                a.total_cost);
    CHECK(a.n_matched == 200);
}

void test_exact_and_greedy_agree_on_easy_problems() {
    // Where the answer is unambiguous, the two paths must not disagree.
    std::vector<std::vector<Real>> cost(8, std::vector<Real>(8, 100.0));
    for (std::size_t i = 0; i < 8; ++i) cost[i][i] = 1.0;

    const Assignment exact = match(cost, 50.0, /*exact_limit*/ 64);
    const Assignment approx = match(cost, 50.0, /*exact_limit*/ 1);
    CHECK(exact.n_matched == approx.n_matched);
    CHECK_NEAR(exact.total_cost, approx.total_cost, 1e-9);
    for (std::size_t i = 0; i < 8; ++i) {
        CHECK(exact.row_to_col[i] == static_cast<int>(i));
    }
}


/// Exhaustive search over every gated matching: maximum cardinality first,
/// cheapest total among those. That is the objective `match` claims - it pairs
/// everything it can, then minimises what the pairing costs - so it is the
/// objective the solver has to be checked against.
///
/// A column may go unmatched, which is not a detail: where columns outnumber
/// rows some column MUST, and a reference that insisted on a full matching
/// would report every wide problem as unsolvable rather than checking it.
Real brute_force_min(const std::vector<std::vector<Real>>& cost, Real max_cost,
                     std::size_t* out_matched = nullptr) {
    const std::size_t n = cost.size(), m = cost[0].size();
    std::vector<int> used(n, 0);
    std::size_t best_n = 0;
    Real best = 0.0;
    const std::function<void(std::size_t, std::size_t, Real)> rec =
        [&](std::size_t j, std::size_t cnt, Real acc) {
            if (j == m) {
                if (cnt > best_n || (cnt == best_n && acc < best)) {
                    best_n = cnt;
                    best = acc;
                }
                return;
            }
            rec(j + 1, cnt, acc);                      // leave column j unmatched
            for (std::size_t i = 0; i < n; ++i) {
                if (used[i] || !std::isfinite(cost[i][j]) || cost[i][j] > max_cost) continue;
                used[i] = 1;
                rec(j + 1, cnt + 1, acc + cost[i][j]);
                used[i] = 0;
            }
        };
    rec(0, 0, 0.0);
    if (out_matched != nullptr) *out_matched = best_n;
    return best;
}

void test_tall_matrix_is_optimal() {
    // More rows than columns. The Jonker-Volgenant search needs a free column
    // to terminate each augmenting path on, so with rows in excess it used to
    // break out mid-search having already shifted the potentials, and returned
    // a matching of the right SIZE built from the wrong PAIRS. Every column
    // was still used, so no caller could detect it.
    //
    // This is not a corner case. Truth-to-track scoring is tall exactly when
    // the tracker is under-reporting, which is the regime the metrics exist to
    // measure, and reacquisition is tall whenever more detections reappear at
    // once than there are dormant tracks to claim them.
    const std::vector<std::vector<Real>> cost{
        {8.0, 2.0, 7.0},
        {3.0, 9.0, 1.0},
        {6.0, 4.0, 5.0},
        {2.0, 7.0, 8.0},
    };
    const Assignment a = match(cost, 100.0, /*exact_limit*/ 64);
    CHECK(a.n_matched == 3);
    CHECK_NEAR(a.total_cost, brute_force_min(cost, 100.0), 1e-9);

    // And the maps still agree with each other after the transpose.
    for (std::size_t j = 0; j < 3; ++j) {
        const int i = a.col_to_row[j];
        CHECK(i >= 0);
        CHECK(a.row_to_col[static_cast<std::size_t>(i)] == static_cast<int>(j));
    }
}

void test_optimal_at_every_shape() {
    // Against exhaustive search, over shapes on both sides of the square.
    Rng rng(20240117);
    const std::pair<std::size_t, std::size_t> shapes[] = {
        {2, 2}, {3, 3}, {4, 3}, {5, 2}, {6, 3}, {3, 4}, {2, 5}, {3, 6}};
    for (const auto& [n, m] : shapes) {
        for (int trial = 0; trial < 200; ++trial) {
            std::vector<std::vector<Real>> cost(n, std::vector<Real>(m));
            for (auto& row : cost) {
                for (auto& v : row) v = rng.uniform(0.0, 10.0);
            }
            const Assignment a = match(cost, 10.0, /*exact_limit*/ 64);
            std::size_t want_n = 0;
            const Real want_cost = brute_force_min(cost, 10.0, &want_n);
            CHECK(want_n == std::min(n, m));
            CHECK(a.n_matched == want_n);
            CHECK_NEAR(a.total_cost, want_cost, 1e-9);
        }
    }
}

}  // namespace

int main() {
    test_one_to_one();
    test_optimal_beats_nearest_first();
    test_does_not_cross_a_pair();
    test_gate_excludes();
    test_empty_inputs();
    test_rectangular();
    test_large_problem_stays_one_to_one();
    test_exact_and_greedy_agree_on_easy_problems();
    test_tall_matrix_is_optimal();
    test_optimal_at_every_shape();
    return trace::test::summary("test_assignment");
}
