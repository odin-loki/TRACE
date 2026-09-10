// Scoring is itself an assignment problem. If it is wrong, every performance
// number in this repository is wrong with it, so it gets checked against cases
// whose answer can be worked out by hand.
#include "trace/sim/assignment.hpp"

#include <cstdio>

#include "test_harness.hpp"

using namespace trace;
using namespace trace::sim;

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
    return trace::test::summary("test_assignment");
}
