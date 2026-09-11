/* Property — log_sum_exp is numerically safe.
 * Mirrors src/core/pmbm.cpp:39-46 and src/core/pattern_of_life.cpp:16-23
 * (the two copies are identical):
 *
 *     if (v.empty()) return -inf;
 *     const Real m = *std::max_element(v.begin(), v.end());
 *     if (!std::isfinite(m)) return m;
 *     Real acc = 0.0;
 *     for (const Real x : v) acc += std::exp(x - m);
 *     return m + std::log(acc);
 *
 * The hazard in any log-sum-exp is log(0), which is -inf and poisons every
 * downstream weight. The shift by the maximum is what rules it out: the
 * maximal element contributes exp(0) = 1 exactly, so acc >= 1 and log(acc) is
 * both finite and non-negative.
 *
 * exp() and log() are not modelled -- a bit-precise transcendental is beyond
 * any bounded model checker. What IS proven is the property the shift exists
 * to establish, using only two facts about exp that hold in IEEE-754:
 * exp(0) == 1 exactly, and exp(t) >= 0 for all t. Everything else about exp is
 * left nondeterministic, so the proof holds for any conforming implementation.
 *
 * Claims:
 *   (a) the shifted maximum is exactly zero, so its exponential is exactly 1;
 *   (b) acc >= 1, hence log(acc) is defined and >= 0;
 *   (c) the result is >= the maximum element (never loses the dominant term).
 */
#include "verif.h"

#define K 4

/* exp(t) constrained only by the two properties the proof relies on. */
static double exp_model(double t) {
    if (t == 0.0) return 1.0;              /* exp(0) == 1 exactly */
    double r = nondet_double();
    ASSUME(r == r);
    ASSUME(r >= 0.0);                      /* exp is non-negative */
    ASSUME(r <= 1.0);                      /* t < 0 here, so exp(t) <= 1 */
    return r;
}

int main(void) {
    double v[K];
    for (int i = 0; i < K; ++i) v[i] = bounded(-1e6, 1e6);

    /* std::max_element */
    double m = v[0];
    for (int i = 1; i < K; ++i) if (v[i] > m) m = v[i];
    CHECK(m == m, "max of finite entries is finite");

    int argmax_seen = 0;
    double acc = 0.0;
    for (int i = 0; i < K; ++i) {
        const double shifted = v[i] - m;
        CHECK(shifted <= 0.0, "no entry exceeds the maximum after shifting");
        if (shifted == 0.0) argmax_seen = 1;
        acc += exp_model(shifted);
    }

    /* (a) */
    CHECK(argmax_seen, "the maximal element shifts to exactly zero");
    /* (b) -- the whole point of the shift: log(acc) is never log(0). */
    CHECK(acc >= 1.0, "acc >= 1, so log(acc) is defined and non-negative");
    CHECK(acc <= (double)K, "acc <= K, so log(acc) <= log(K): no overflow");

    /* (c) m + log(acc) >= m, since log(acc) >= 0. Modelled the same way. */
    double log_acc = nondet_double();
    ASSUME(log_acc == log_acc);
    ASSUME(acc >= 1.0 ? log_acc >= 0.0 : 1);
    const double result = m + log_acc;
    CHECK(result >= m, "the result never falls below the dominant term");
    return 0;
}
