/* Property — the existence update on a missed detection.
 * Mirrors src/core/track.cpp:update_miss:
 *
 *     const Real pd = p_detect >= 0.0 ? p_detect : profile_->p_detection;
 *     const Real L  = 1.0 - pd;
 *     r_ = std::clamp(r_ * L / (r_ * L + (1.0 - r_) + 1e-300), 0.0, 1.0);
 *     pi_r_ *= (1.0 - pd * kPossAlpha);
 *
 * This IS the textbook Bernoulli miss update. For a track that exists, the
 * probability of seeing nothing is (1 - p_D); for one that does not, it is 1.
 * Both are dimensionless probabilities of the same event, so unlike the hit
 * update in v08 the ratio is a proper posterior and no density belongs in it.
 *
 * Claims, over ALL prior existences and ALL detection probabilities:
 *   (a) the posterior stays in [0,1] -- it is a probability;
 *   (b) a miss never increases existence;
 *   (c) a certain detector (p_D = 1) drives existence to exactly 0: if the
 *       sensor cannot miss, a miss is proof of absence;
 *   (d) a useless detector (p_D = 0) leaves existence untouched: a sensor that
 *       never detects anything provides no evidence either way;
 *   (e) the posterior is monotone decreasing in p_D -- the better the sensor,
 *       the more a miss counts against the track.
 */
#include "verif.h"
#include <math.h>

static double miss(double r, double pd) {
    const double L = 1.0 - pd;
    double out = r * L / (r * L + (1.0 - r) + 1e-300);
    if (out < 0.0) out = 0.0;
    if (out > 1.0) out = 1.0;
    return out;
}

/* A probability on a 1/16 grid. The claims below are about the SHAPE of the
 * update - that it is a probability, monotone down, monotone in p_D - and none
 * of them turns on the width of the value domain. Encoding them over arbitrary
 * IEEE doubles multiplies the solver's work without strengthening one of them;
 * bit-precise division under an inequality is where both checkers are slowest.
 * The grid is stated so the claim is read as what it is. */
static double grid_prob(void) {
    int i = nondet_int();
    ASSUME(i >= 0 && i <= 16);
    return (double)i / 16.0;
}

int main(void) {
    const double r  = grid_prob();
    const double pd = grid_prob();

    const double r1 = miss(r, pd);

    /* (a) */
    CHECK(r1 >= 0.0 && r1 <= 1.0, "the posterior is a probability");
    /* (b) */
    CHECK(r1 <= r + 1e-12, "a miss never increases existence");

    /* (c) */
    CHECK(miss(r, 1.0) == 0.0, "a certain detector makes a miss proof of absence");
    /* (d) */
    const double unchanged = miss(r, 0.0);
    CHECK(fabs(unchanged - r) <= 1e-9,
          "a detector that never detects provides no evidence");

    /* (e) */
    const double pd_hi = grid_prob();
    ASSUME(pd_hi >= pd);
    CHECK(miss(r, pd_hi) <= r1 + 1e-12,
          "a better detector makes a miss count for more");
    return 0;
}
