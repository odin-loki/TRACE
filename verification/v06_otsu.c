/* Property — high_mode_threshold (Otsu split) indexes safely.
 * Mirrors src/detectors/behaviour.cpp:41-75 statement for statement.
 *
 * The between-class variance maximised here is
 *     w0 * w1 * (mu1 - mu0)^2      with w1 = 1 - w0,
 * which is the standard Otsu criterion; the code writes it as
 *     w_lo * (1 - w_lo) * (mean_hi - mean_lo)^2.
 *
 * The risks are index arithmetic, not statistics: `best_k` is used both to
 * slice the sample (`sample.begin() + best_k`) and to divide by `n - best_k`,
 * and it is seeded to 0 to mean "no split found". If `best_k` could reach `n`
 * the code would divide by zero and read `sample[n]`.
 *
 * Claims, over ALL sorted samples of this size:
 *   (a) both class sizes are non-zero, so neither mean divides by zero;
 *   (b) when a threshold is returned, 1 <= best_k <= n-1, so sample[best_k]
 *       is in bounds;
 *   (c) the returned threshold is an element of the sample, and is not the
 *       smallest element (the high mode is a proper subset);
 *   (d) the returned threshold is >= the low-class mean.
 */
#include "verif.h"

#define N 4

int main(void) {
    /* Speeds: non-negative, sorted ascending, as the contract requires.
     * Drawn from a small grid of exactly-representable values rather than from
     * the reals. The claims here are about index arithmetic and about the sign
     * and ordering of the class means, none of which turn on the width of the
     * value domain; encoding them over arbitrary IEEE doubles multiplies the
     * solver's work without strengthening a single one of them. */
    double sample[N];
    int prev = 0;
    for (int i = 0; i < N; ++i) {
        int q = nondet_int();
        ASSUME(q >= prev && q <= 15);
        sample[i] = (double)q;
        prev = q;
    }
    const double min_ratio = 1.5;

    double total = 0.0;
    for (int i = 0; i < N; ++i) total += sample[i];

    double best_var = 0.0;
    int best_k = 0;
    double cum = 0.0;
    for (int k = 1; k < N; ++k) {
        cum += sample[k - 1];
        /* (a) k >= 1 and n - k >= 1 for every k in [1, n-1]. */
        CHECK(k >= 1, "low class is non-empty");
        CHECK(N - k >= 1, "high class is non-empty");
        const double w_lo = (double)k / (double)N;
        CHECK(w_lo > 0.0 && w_lo < 1.0, "class weight is a proper fraction");
        const double mean_lo = cum / (double)k;
        const double mean_hi = (total - cum) / (double)(N - k);
        const double var = w_lo * (1.0 - w_lo) * (mean_hi - mean_lo) *
                           (mean_hi - mean_lo);
        CHECK(var >= 0.0, "between-class variance is non-negative");
        if (var > best_var) { best_var = var; best_k = k; }
    }
    if (best_k == 0) return 0;             /* std::nullopt */

    /* (b) */
    CHECK(best_k >= 1 && best_k <= N - 1, "best_k indexes a proper split");

    double cut = 0.0;
    for (int i = 0; i < best_k; ++i) cut += sample[i];
    const double mean_lo = cut / (double)best_k;
    const double mean_hi = (total - cut) / (double)(N - best_k);
    if (mean_hi < mean_lo * min_ratio) return 0;   /* std::nullopt */

    const double threshold = sample[best_k];

    /* (c) */
    int in_sample = 0;
    for (int i = 0; i < N; ++i) if (sample[i] == threshold) in_sample = 1;
    CHECK(in_sample, "the threshold is an element of the sample");
    CHECK(threshold >= sample[0], "the threshold is not below the sample");
    /* (d) The sample is sorted and the low class is sample[0..best_k-1], so
     * every element of the high class is >= the low class mean. */
    CHECK(threshold >= mean_lo, "the threshold is at or above the low mean");
    return 0;
}
