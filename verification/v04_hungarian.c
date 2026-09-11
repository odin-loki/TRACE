/* Property 3, corrected translation — the Jonker-Volgenant/Hungarian matcher
 * in src/core/assignment.cpp:15-88.
 *
 * The earlier harness (p3_hungarian.c) used a FINITE integer sentinel
 * INF = 1000000 for a non-finite cost. That is not what the source does:
 *
 *     constexpr Real kInf = std::numeric_limits<Real>::infinity();
 *                                                   (assignment.cpp:11)
 *
 * The difference is load-bearing. Line 62 of the source is
 *
 *     minv[j] -= delta;
 *
 * With a finite sentinel, repeatedly subtracting delta eventually drags an
 * "unreachable" column below the others and makes it selectable, so `way[j]`
 * can be read while still holding a stale value from a previous row. With a
 * true IEEE infinity, INF - delta == INF for every finite delta, so such a
 * column is never selected and the stale entry is never reached.
 *
 * This harness therefore uses `double` and INFINITY, exactly as the source
 * does. Costs are drawn from a small set of exactly-representable integral
 * values (or +inf) so the float reasoning stays tractable while the
 * infinity semantics are the real ones.
 *
 * Claims, over ALL cost matrices of this size:
 *   (a) every array index stays in bounds -- in particular u[p[j]] on
 *       line 58, which is u[SIZE_MAX] if p[j] is ever -1 when used[j] is set;
 *   (b) the augment loop at line 69 terminates;
 *   (c) the output is a valid partial matching: row_to_col and col_to_row
 *       agree, and no row or column is used twice;
 *   (d) no pair above max_cost, and no non-finite pair, is ever matched.
 */
#include "verif.h"

#define N 3
#define M 3
#define MAXC 10.0

static const double INF = 1.0 / 0.0;   /* true +inf, as kInf is */

int main(void) {
    double cost[N][M];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) {
            int c = nondet_int();
            ASSUME(c >= 0 && c <= 12);
            /* > MAXC models a gated-out pair, which match_points() writes as
             * kInf (assignment.cpp:141) and the solver sees as non-finite. */
            cost[i][j] = (c > 10) ? INF : (double)c;
        }
    const double max_cost = MAXC;

    double u[N + 1], v[M + 1];
    long p[M + 1], way[M + 1];
    for (int i = 0; i <= N; ++i) u[i] = 0.0;
    for (int j = 0; j <= M; ++j) { v[j] = 0.0; p[j] = -1; way[j] = 0; }

    for (int i = 0; i < N; ++i) {
        p[M] = i;
        int j0 = M;
        double minv[M + 1];
        char used[M + 1];
        for (int j = 0; j <= M; ++j) { minv[j] = INF; used[j] = 0; }

        do {
            used[j0] = 1;
            const long i0 = p[j0];
            /* (a) the index that would be SIZE_MAX if p[j0] were -1 */
            CHECK(i0 >= 0 && i0 < N, "i0 = p[j0] is a valid row index");
            double delta = INF;
            int j1 = M;

            for (int j = 0; j < M; ++j) {
                if (used[j]) continue;
                const double c = cost[(int)i0][j];
                const double cur = (c < INF ? c : INF) - u[(int)i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            if (!(delta < INF)) break;          /* !std::isfinite(delta) */

            for (int j = 0; j <= M; ++j) {
                if (used[j]) {
                    /* (a) the write on assignment.cpp:58 */
                    CHECK(p[j] >= 0 && p[j] < N, "u[p[j]] index is in bounds");
                    u[(int)p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != -1);

        if (j0 == M) continue;
        /* (b) this is the loop whose unwinding assertion failed under the
         * finite-sentinel translation. */
        while (j0 != M) {
            const int jw = (int)way[j0];
            CHECK(jw >= 0 && jw <= M, "way[j0] is a valid column index");
            p[j0] = p[jw];
            j0 = jw;
        }
    }

    int row_to_col[N], col_to_row[M];
    for (int i = 0; i < N; ++i) row_to_col[i] = -1;
    for (int j = 0; j < M; ++j) col_to_row[j] = -1;

    for (int j = 0; j < M; ++j) {
        if (p[j] < 0) continue;
        const int i = (int)p[j];
        CHECK(i >= 0 && i < N, "p[j] is a valid row index at extraction");
        if (!(cost[i][j] < INF) || cost[i][j] > max_cost) continue;
        row_to_col[i] = j;
        col_to_row[j] = i;
    }

    /* (c) the matching is mutual and injective both ways. */
    for (int i = 0; i < N; ++i) {
        if (row_to_col[i] < 0) continue;
        const int j = row_to_col[i];
        CHECK(j >= 0 && j < M, "matched column in range");
        CHECK(col_to_row[j] == i, "matching is mutual");
        /* (d) the gate holds. */
        CHECK(cost[i][j] <= max_cost, "no pair above max_cost is matched");
        CHECK(cost[i][j] < INF, "no non-finite pair is matched");
    }
    for (int j = 0; j < M; ++j) {
        if (col_to_row[j] < 0) continue;
        const int i = col_to_row[j];
        CHECK(i >= 0 && i < N, "matched row in range");
        CHECK(row_to_col[i] == j, "matching is mutual the other way");
    }
    return 0;
}
