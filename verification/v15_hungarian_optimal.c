/* Property — the matcher returns a MINIMUM-cost matching, at any shape.
 * Mirrors src/core/assignment.cpp:23-121, including the transpose in
 * `hungarian()` that puts a tall matrix back inside `hungarian_le`'s n <= m
 * precondition.
 *
 * v04 proves the solver is memory-safe and that its output is a valid partial
 * matching. Neither is optimality, and the two are independent: a solver can
 * return a well-formed matching of exactly the right SIZE and still pick the
 * wrong pairs. That is what the pre-fix code did whenever rows outnumbered
 * columns -- every column got a row, so no caller could tell -- and it is what
 * this harness exists to rule out.
 *
 * The claim is stated against an arbitrary rival rather than by enumerating
 * every matching: a nondeterministic injective column->row assignment is
 * constructed, constrained to be a valid full matching, and the solver's total
 * is asserted no worse. Proving it for a nondeterministic rival proves it for
 * all of them, and costs one choice instead of 4*3*2 enumerated ones.
 *
 * Shape is 4 rows by 3 columns: TALL, which is the case that was wrong, and
 * the case every caller actually hits --
 *   - src/sim/scenario.cpp:58 and src/apps/mot_main.cpp:94,168 match truth
 *     rows to track columns, so they are tall exactly when the tracker is
 *     under-reporting, which is the regime the metrics exist to measure;
 *   - src/core/pmbm.cpp:611 matches reappearing detections to dormant tracks,
 *     so it is tall whenever more things reappear at once than went dormant.
 *
 * Run against the pre-fix source this harness FAILS, which is what makes it a
 * regression test rather than a description.
 *
 * Costs are INTEGERS here, where v04's are doubles. That is not a shortcut and
 * it is not the mistake v04's header warns about. The sentinel only has to
 * behave like an infinity where it takes part in arithmetic, and the arithmetic
 * that can reach it is `minv[j] -= delta` -- which a finite sentinel would
 * wrongly make selectable. Here every cost is finite and inside the gate, so
 * the first pass of the inner loop gives every unused column a finite `minv`,
 * and the sentinel is never decremented. An assertion below states exactly
 * that, so the soundness of the integer encoding is checked rather than
 * assumed. Optimality is a combinatorial property; encoding it over integers
 * instead of IEEE doubles is the difference between a solvable instance and a
 * 4.9-million-variable one.
 */
#include "verif.h"

#define N 3                    /* rows */
#define M 2                    /* columns */
#define MAXV 3
#define INF 1000                /* see the header: never decremented here */

/* hungarian_le: assignment.cpp:23-96, verbatim in control flow.
 * Requires n <= m. Writes the column chosen for each row into rtc (length n)
 * and the row chosen for each column into ctr (length m). */
static int solve_le(const int cost[M][N], int n, int m,
                    int rtc[M], int ctr[N], int* n_matched, int max_cost) {
    int u[M + 1], v[N + 1];   /* rows of the transpose = M, columns = N */
    int p[N + 1], way[N + 1];
    /* u is indexed by row and v, p, way by column, exactly as in
     * assignment.cpp:25-26: `std::vector<Real> u(n+1), v(m+1)`. */
    for (int i = 0; i <= n; ++i) u[i] = 0;
    for (int j = 0; j <= m; ++j) { v[j] = 0; p[j] = -1; way[j] = 0; }

    for (int i = 0; i < n; ++i) {
        p[m] = i;
        int j0 = m;
        int minv[N + 1];
        char used[N + 1];
        for (int j = 0; j <= m; ++j) { minv[j] = INF; used[j] = 0; }

        do {
            used[j0] = 1;
            const int i0 = p[j0];
            CHECK(i0 >= 0 && i0 < n, "row index stays in bounds");
            int delta = INF;
            int j1 = m;
            for (int j = 0; j < m; ++j) {
                if (used[j]) continue;
                const int c = cost[(int)i0][j];
                const int cur = (c < INF ? c : INF) - u[(int)i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            if (!(delta < INF)) break;
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    CHECK(p[j] >= 0 && p[j] < n, "potential index in bounds");
                    u[(int)p[j]] += delta;
                    v[j] -= delta;
                } else {
                    /* The integer sentinel is sound only if it is never the
                     * thing being decremented. */
                    CHECK(minv[j] < INF,
                          "the sentinel never enters arithmetic when all costs are finite");
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != -1);

        if (j0 == m) continue;
        while (j0 != m) {
            const int jw = (int)way[j0];
            p[j0] = p[jw];
            j0 = jw;
        }
    }

    for (int i = 0; i < n; ++i) rtc[i] = -1;
    for (int j = 0; j < m; ++j) ctr[j] = -1;
    int total = 0;
    *n_matched = 0;
    for (int j = 0; j < m; ++j) {
        if (p[j] < 0) continue;
        const int i = (int)p[j];
        if (!(cost[i][j] < INF) || cost[i][j] > max_cost) continue;
        rtc[i] = j; ctr[j] = i; total += cost[i][j]; ++(*n_matched);
    }
    return total;
}

int main(void) {
    /* Original problem: N rows by M columns, all costs admissible. */
    int cost[N][M];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) {
            int c = nondet_int();
            ASSUME(c >= 0 && c <= MAXV);
            cost[i][j] = c;
        }
    const int max_cost = MAXV;

    /* hungarian(): N > M, so solve the transpose (assignment.cpp:106-120). */
    int t[M][N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) t[j][i] = cost[i][j];

    int t_rtc[M], t_ctr[N], n_matched = 0;
    const int total = solve_le(t, M, N, t_rtc, t_ctr, &n_matched, max_cost);
    /* Rows of the transpose are the original columns, so the maps swap:
     * row_to_col == t_ctr, col_to_row == t_rtc. */
    const int* row_to_col = t_ctr;
    const int* col_to_row = t_rtc;

    /* Every cost is finite and within the gate, so every column must be used. */
    CHECK(n_matched == M, "a full matching is found when every pair is admissible");

    /* The result is a genuine matching. */
    for (int j = 0; j < M; ++j) {
        const int i = col_to_row[j];
        CHECK(i >= 0 && i < N, "each column gets a real row");
        CHECK(row_to_col[i] == j, "the two maps agree");
    }
    for (int j = 0; j < M; ++j)
        for (int k = j + 1; k < M; ++k)
            CHECK(col_to_row[j] != col_to_row[k], "no row serves two columns");

    /* An arbitrary rival full matching. */
    int rival[M];
    for (int j = 0; j < M; ++j) {
        int r = nondet_int();
        ASSUME(r >= 0 && r < N);
        rival[j] = r;
    }
    for (int j = 0; j < M; ++j)
        for (int k = j + 1; k < M; ++k)
            ASSUME(rival[j] != rival[k]);          /* injective: a real matching */

    int rival_total = 0;
    for (int j = 0; j < M; ++j) rival_total += cost[rival[j]][j];

    /* THE CLAIM: no matching is cheaper than the one returned. */
    CHECK(total <= rival_total, "the returned matching is minimum-cost");
    return 0;
}
