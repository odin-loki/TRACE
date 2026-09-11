// Betweenness is the only graph statistic the engine publishes, and the only
// one whose value a reader is invited to interpret directly: NetworkReport
// carries it as a normalised score and the role classifier ranks on it. So it
// is checked two ways here - against an independently written reference, and
// exhaustively over every graph of a given size, which for small n is not a
// sample but a proof.
#include "trace/core/network.hpp"

#include <cmath>
#include <cstdio>
#include <queue>
#include <vector>

#include "test_harness.hpp"

using namespace trace;

namespace {

using Adjacency = std::vector<std::vector<std::size_t>>;

Adjacency from_bits(std::size_t n, unsigned bits) {
    Adjacency adj(n);
    unsigned b = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j, ++b) {
            if (bits & (1u << b)) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }
    return adj;
}

/// Betweenness from the definition, not from Brandes' accumulation.
///
/// For every ordered pair (s,t) and every other vertex v, v's share of that
/// pair is sigma_sv * sigma_vt / sigma_st when v lies on a shortest s-t path
/// and zero otherwise. Summing over ORDERED pairs counts each unordered pair
/// twice, so the divisor is (n-1)(n-2) rather than half of it.
///
/// This shares no code and no structure with the implementation: it counts
/// paths by BFS from every source and then combines them arithmetically, where
/// Brandes accumulates dependencies backwards along one BFS tree. If the two
/// agree on every graph of a size, the accumulation is right.
std::vector<Real> reference_betweenness(const Adjacency& adj) {
    const std::size_t n = adj.size();
    std::vector<Real> bc(n, 0.0);
    if (n < 3) return bc;

    // sigma[s][v] = number of shortest s-v paths; dist[s][v] their length.
    std::vector<std::vector<Real>> sigma(n, std::vector<Real>(n, 0.0));
    std::vector<std::vector<long>> dist(n, std::vector<long>(n, -1));
    for (std::size_t s = 0; s < n; ++s) {
        sigma[s][s] = 1.0;
        dist[s][s] = 0;
        std::queue<std::size_t> q;
        q.push(s);
        while (!q.empty()) {
            const std::size_t v = q.front();
            q.pop();
            for (const std::size_t w : adj[v]) {
                if (dist[s][w] < 0) {
                    dist[s][w] = dist[s][v] + 1;
                    q.push(w);
                }
                if (dist[s][w] == dist[s][v] + 1) sigma[s][w] += sigma[s][v];
            }
        }
    }

    for (std::size_t s = 0; s < n; ++s) {
        for (std::size_t t = 0; t < n; ++t) {
            if (s == t || dist[s][t] < 0) continue;
            for (std::size_t v = 0; v < n; ++v) {
                if (v == s || v == t) continue;
                if (dist[s][v] < 0 || dist[v][t] < 0) continue;
                if (dist[s][v] + dist[v][t] != dist[s][t]) continue;
                bc[v] += sigma[s][v] * sigma[v][t] / sigma[s][t];
            }
        }
    }

    const Real denom = static_cast<Real>((n - 1) * (n - 2));
    for (Real& v : bc) v /= denom;
    return bc;
}

void test_exhaustive_against_reference() {
    // Every undirected graph on 4, 5 and 6 vertices: 64, 1024 and 32768 of
    // them. Exhaustive at these sizes, so agreement is not evidence about the
    // implementation, it is the whole statement about it.
    int graphs = 0;
    for (std::size_t n : {4u, 5u, 6u}) {
        const unsigned edges = static_cast<unsigned>(n * (n - 1) / 2);
        for (unsigned bits = 0; bits < (1u << edges); ++bits) {
            const Adjacency adj = from_bits(n, bits);
            const std::vector<Real> got = betweenness_centrality(adj);
            const std::vector<Real> want = reference_betweenness(adj);
            for (std::size_t v = 0; v < n; ++v) {
                CHECK_NEAR(got[v], want[v], 1e-9);
                // And the range the header promises, which is what the
                // published score means.
                CHECK(got[v] >= 0.0);
                CHECK(got[v] <= 1.0 + 1e-12);
            }
            ++graphs;
        }
    }
    std::printf("  betweenness: %d graphs checked exhaustively (n = 4, 5, 6)\n", graphs);
}

void test_star_hub_is_exactly_one() {
    // The hub of a star lies between every pair of leaves and nothing else
    // does, so it is the maximum the normalisation is defined against. Any
    // value but 1.0 means the divisor is wrong, and the size at which it is
    // wrong tells you how: a constant 2.0 is the undirected double-count.
    for (std::size_t n : {4u, 5u, 7u, 11u, 21u}) {
        Adjacency adj(n);
        for (std::size_t i = 1; i < n; ++i) {
            adj[0].push_back(i);
            adj[i].push_back(0);
        }
        const std::vector<Real> bc = betweenness_centrality(adj);
        CHECK_NEAR(bc[0], 1.0, 1e-12);
        for (std::size_t i = 1; i < n; ++i) CHECK_NEAR(bc[i], 0.0, 1e-12);
    }
}

void test_path_matches_closed_form() {
    // On a path, vertex k lies between exactly the pairs that straddle it, so
    // its unnormalised score is k(n-1-k) and the normalised one follows. A
    // closed form the implementation cannot have been fitted to.
    for (std::size_t n : {5u, 9u, 12u}) {
        Adjacency adj(n);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            adj[i].push_back(i + 1);
            adj[i + 1].push_back(i);
        }
        const std::vector<Real> bc = betweenness_centrality(adj);
        const Real denom = static_cast<Real>((n - 1) * (n - 2)) / 2.0;
        for (std::size_t k = 0; k < n; ++k) {
            const Real want = static_cast<Real>(k * (n - 1 - k)) / denom;
            CHECK_NEAR(bc[k], want, 1e-9);
        }
    }
}

void test_complete_graph_is_all_zero() {
    // Everybody is adjacent to everybody, so no shortest path has an interior
    // and no vertex is between anything.
    for (std::size_t n : {4u, 6u, 9u}) {
        Adjacency adj(n);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                if (i != j) adj[i].push_back(j);
            }
        }
        const std::vector<Real> bc = betweenness_centrality(adj);
        for (std::size_t i = 0; i < n; ++i) CHECK_NEAR(bc[i], 0.0, 1e-12);
    }
}

void test_degenerate_sizes() {
    CHECK(betweenness_centrality({}).empty());
    CHECK(betweenness_centrality(Adjacency(1)).size() == 1);
    CHECK(betweenness_centrality(Adjacency(2)).size() == 2);
    // An isolated vertex lies on no path between others, and its presence must
    // not change anyone else's score beyond enlarging the normalisation.
    Adjacency adj(4);
    adj[0].push_back(1); adj[1].push_back(0);
    adj[1].push_back(2); adj[2].push_back(1);
    const std::vector<Real> bc = betweenness_centrality(adj);
    CHECK_NEAR(bc[3], 0.0, 1e-12);
    CHECK(bc[1] > 0.0);
}

}  // namespace

int main() {
    test_exhaustive_against_reference();
    test_star_hub_is_exactly_one();
    test_path_matches_closed_form();
    test_complete_graph_is_all_zero();
    test_degenerate_sizes();
    return trace::test::summary("test_network");
}
