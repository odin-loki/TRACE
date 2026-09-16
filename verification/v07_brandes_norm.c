// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — betweenness_centrality's normalisation.
 * Mirrors src/core/network.cpp:21-81. The source ends with
 *
 *     // Normalise to [0,1] for an undirected graph.
 *     const Real denom = std::max(static_cast<Real>((n-1)*(n-2)) / 2.0, 1.0);
 *     for (auto& v : bc) v /= denom;
 *
 * This harness checks exactly what that comment claims -- that every returned
 * centrality lies in [0,1] -- over ALL undirected graphs on four vertices
 * (2^6 = 64 of them, enumerated by six nondeterministic edge bits).
 *
 * It did not hold before the divisor was corrected. Brandes' accumulation runs
 * the outer loop over every source s, so on an undirected graph each unordered
 * pair {s,t} is counted twice, once from each end; the raw score must be
 * halved before being divided by the (n-1)(n-2)/2 unordered pairs a vertex
 * could lie between. Dividing by the pair count alone left every score at
 * twice its normalised value, and the star K_{1,3} was the witness: its hub
 * lies between all 3*2 = 6 ordered leaf pairs, giving raw 6 and 6/3 = 2.0.
 * Run against the pre-fix source this harness FAILS on exactly that graph.
 *
 * The consequence was confined to reporting. Both consumers are scale-free --
 * the role classifier in src/detectors/behaviour.cpp:501-516 thresholds
 * against the upper quartile of these same values, and the recurrence test at
 * src/core/network.cpp:204 asks only whether a score exceeds zero -- so a
 * uniform factor cancelled in both. NetworkReport, which publishes the number,
 * was the one place it did not.
 */
#include "verif.h"

#define N 4

int main(void) {
    /* Six nondeterministic bits: one per unordered pair of four vertices. */
    char adj[N][N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) adj[i][j] = 0;
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            int e = nondet_int();
            ASSUME(e == 0 || e == 1);
            adj[i][j] = (char)e;
            adj[j][i] = (char)e;
        }

    double bc[N];
    for (int i = 0; i < N; ++i) bc[i] = 0.0;

    double sigma[N], delta[N];
    long dist[N];
    char pred[N][N];                 /* pred[w][v] : v is a predecessor of w */
    int order[N], n_order;
    int queue[N], qh, qt;

    for (int s = 0; s < N; ++s) {
        int degree = 0;
        for (int j = 0; j < N; ++j) degree += adj[s][j];
        if (degree == 0) continue;             /* adjacency[s].empty() */

        for (int i = 0; i < N; ++i) {
            sigma[i] = 0.0; delta[i] = 0.0; dist[i] = -1;
            for (int j = 0; j < N; ++j) pred[i][j] = 0;
        }
        n_order = 0; qh = 0; qt = 0;

        sigma[s] = 1.0; dist[s] = 0; queue[qt++] = s;

        while (qh < qt) {
            const int v = queue[qh++];
            CHECK(n_order < N, "BFS order never exceeds the vertex count");
            order[n_order++] = v;
            for (int w = 0; w < N; ++w) {
                if (!adj[v][w]) continue;
                if (dist[w] < 0) {
                    dist[w] = dist[v] + 1;
                    CHECK(qt < N, "each vertex is enqueued at most once");
                    queue[qt++] = w;
                }
                if (dist[w] == dist[v] + 1) {
                    sigma[w] += sigma[v];
                    pred[w][v] = 1;
                }
            }
        }

        for (int idx = n_order - 1; idx >= 0; --idx) {
            const int w = order[idx];
            for (int v = 0; v < N; ++v) {
                if (!pred[w][v]) continue;
                if (sigma[w] > 0.0) delta[v] += sigma[v] / sigma[w] * (1.0 + delta[w]);
            }
            if (w != s) bc[w] += delta[w];
        }
    }

    double denom = (double)((N - 1) * (N - 2));
    if (denom < 1.0) denom = 1.0;
    for (int i = 0; i < N; ++i) bc[i] /= denom;

    for (int i = 0; i < N; ++i) {
        CHECK(bc[i] >= 0.0, "centrality is non-negative");
        CHECK(bc[i] <= 1.0, "centrality is normalised to [0,1] as documented");
    }
    return 0;
}
