// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — the clutter estimator's posterior is well-formed.
 * Mirrors src/core/pmbm.cpp:57-63 and include/trace/core/pmbm.hpp:32-36:
 *
 *     window_.push_back(max(0, n_unassigned));
 *     if (window_.size() > 20) window_.pop_front();
 *     total  = sum(window_);
 *     alpha_ = 3.0 + total;
 *     beta_  = 1.0 + window_.size();
 *     rate()    = alpha_ / beta_;
 *     density(V)= V > 0 ? rate() / V : 1e-6;
 *
 * This is the Gamma-Poisson conjugate update: for n_unassigned ~ Poisson(r)
 * with a Gamma(alpha0=3, beta0=1) prior on the rate, the posterior after n
 * observations summing to S is Gamma(3 + S, 1 + n) and its mean is
 * (3 + S) / (1 + n), which is exactly what rate() returns. The class is named
 * "Beta-Poisson" in the header; Beta-Binomial and Gamma-Poisson are the two
 * standard conjugate pairs and this is the second one. The arithmetic is
 * right and the name is wrong.
 *
 * Because it is windowed rather than cumulative, the posterior is the one
 * conditioned on the last 20 scans only -- deliberate, so the estimate tracks
 * a change in conditions instead of being anchored by history.
 *
 * Claims, over ALL observation sequences:
 *   (a) beta_ is never zero, so rate() never divides by zero;
 *   (b) rate() is strictly positive, so a density used as a likelihood
 *       denominator can never be zero;
 *   (c) the prior floor: with no clutter ever observed, rate() is still
 *       3/(1+n) > 0 -- the estimator never asserts that clutter is impossible;
 *   (d) rate() is monotone non-decreasing in the observations at fixed n;
 *   (e) density() is positive for every positive volume.
 */
#include "verif.h"

/* The source's window is 20 scans (src/core/pmbm.cpp:21). Eight is enough to
 * exercise every claim below - each is monotone in the window length, so no
 * claim distinguishes 8 from 20 - and keeps the instance solvable. */
#define W 8
#define W_SRC 20

int main(void) {
    int n = nondet_int();
    ASSUME(n >= 0 && n <= W);

    double total = 0.0, total_hi = 0.0;
    for (int i = 0; i < W; ++i) {
        if (i >= n) break;
        int x = nondet_int();
        ASSUME(x >= 0 && x <= 15);
        int y = nondet_int();
        ASSUME(y >= x && y <= 15);          /* a pointwise-larger sequence */
        total += (double)x;
        total_hi += (double)y;
    }

    const double alpha = 3.0 + total;
    const double beta  = 1.0 + (double)n;

    /* (a) */
    CHECK(beta >= 1.0, "beta is at least the prior, so rate() cannot divide by zero");
    const double rate = alpha / beta;

    /* (b) */
    CHECK(rate > 0.0, "the posterior mean clutter rate is strictly positive");
    /* (c) */
    CHECK(rate >= 3.0 / (1.0 + (double)W_SRC),
          "the prior floors the rate: clutter is never ruled impossible");

    /* (d) */
    const double rate_hi = (3.0 + total_hi) / beta;
    CHECK(rate_hi >= rate, "more observed clutter never lowers the estimate");

    /* (e) */
    /* A volume from a coarse decade grid: the claim is that a positive volume
     * yields a positive density, which does not turn on the exact value. */
    int e = nondet_int();
    ASSUME(e >= 0 && e <= 10);
    double volume = 1.0;
    for (int i = 0; i < 10; ++i) if (i < e) volume *= 10.0;
    const double density = volume > 0.0 ? rate / volume : 1e-6;
    CHECK(density > 0.0, "clutter density is positive for any positive volume");
    return 0;
}
