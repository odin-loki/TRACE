// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — the hit and miss existence updates agree in the limit.
 * Mirrors src/core/pmbm.cpp:895-899 against src/core/track.cpp:121-127:
 *
 *     const Real present = r * (L * g + (1.0 - L) * cd);
 *     const Real absent  = (1.0 - r) * cd;
 *     set_existence(clamp(present / (present + absent + 1e-300), 0, 0.9999));
 *
 *     r_ = clamp(r_ * L / (r_ * L + (1.0 - r_) + 1e-300), 0, 1);   // L = 1-pd
 *
 * A gated measurement admits two explanations if the entity is there: the
 * entity produced it, at density p_D g(z); or the entity was MISSED and the
 * measurement is clutter, at density (1 - p_D) lambda_c. Only the second is
 * available if the entity is not there. So
 *
 *     r' = r [ p_D g + (1-p_D) lambda ]
 *          -----------------------------------------
 *          r [ p_D g + (1-p_D) lambda ] + (1-r) lambda
 *
 * The second term in the numerator is the one this harness exists for. Without
 * it the update was DISCONTINUOUS at the quantity it is supposed to measure:
 * as g fell the coded ratio fell to zero, while the miss update - the same
 * entity, seen by nobody at all - settles at r(1-p_D) / (r(1-p_D) + (1-r)).
 * A track offered a badly-fitting detection was therefore punished HARDER than
 * a track offered nothing. Run against the pre-fix expression, claim (b) fails
 * on the first counterexample either checker reaches.
 *
 * Claims, over all prior existences, all detection probabilities, all clutter
 * densities and all measurement likelihoods:
 *   (a) the posterior is a probability;
 *   (b) a detection never leaves a track worse off than no detection at all,
 *       wherever the two updates' CEILINGS agree. They do not agree at r = 1:
 *       the hit update clamps to 0.9999 so that no track becomes unkillable,
 *       `update_miss` clamps to 1.0, and a miss therefore leaves r = 1 alone
 *       where a hit would lower it to 0.9999. That is unreachable - r is only
 *       ever set from `r_birth`, from the hit update's own 0.9999 ceiling, or
 *       lowered by a miss, and no shipped profile births at 1.0 - so the claim
 *       is stated where the ceilings do not disagree rather than weakened to
 *       something the counterexample would also satisfy;
 *   (c) at g = 0 the two updates are EQUAL, not merely ordered - the hit update
 *       degrades continuously into the miss update rather than falling off it,
 *       under the same ceiling guard;
 *   (d) the posterior is monotone non-decreasing in g: a better-fitting
 *       detection is never weaker evidence than a worse-fitting one;
 *   (e) a track that does not exist cannot be resurrected by any measurement.
 */
#include "verif.h"
#include <math.h>

static double hit(double r, double pd, double lambda, double g) {
    const double present = r * (pd * g + (1.0 - pd) * lambda);
    const double absent  = (1.0 - r) * lambda;
    double out = present / (present + absent + 1e-300);
    if (out < 0.0) out = 0.0;
    if (out > 0.9999) out = 0.9999;
    return out;
}

static double miss(double r, double pd) {
    const double L = 1.0 - pd;
    double out = r * L / (r * L + (1.0 - r) + 1e-300);
    if (out < 0.0) out = 0.0;
    if (out > 1.0) out = 1.0;
    return out;
}

/* A probability on a 1/4 grid, for the reason v14 gives, only more so: these
 * claims are about the SHAPE of the update, and each one compares TWO of these
 * updates, so the solver carries two bit-precise divisions under every
 * inequality. On a 1/8 grid neither checker finished inside fifteen minutes.
 * Nothing claimed below turns on the width of the value domain. */
static double grid_prob(void) {
    int i = nondet_int();
    ASSUME(i >= 0 && i <= 4);
    return (double)i / 4.0;
}

/* A strictly positive density. lambda is a clutter intensity per unit area and
 * g a Gaussian density in the same units; the claims are scale-free in them, so
 * a coarse positive grid is enough. */
static double grid_density(void) {
    int i = nondet_int();
    ASSUME(i >= 1 && i <= 4);
    return (double)i / 2.0;
}

int main(void) {
    const double r      = grid_prob();
    const double pd     = grid_prob();
    const double lambda = grid_density();

    int gi = nondet_int();
    ASSUME(gi >= 0 && gi <= 4);
    const double g = (double)gi / 2.0;   /* g = 0 is reachable: no fit at all */

    const double r_hit  = hit(r, pd, lambda, g);
    const double r_miss = miss(r, pd);

    /* (a) */
    CHECK(r_hit >= 0.0 && r_hit <= 1.0, "the posterior is a probability");

    /* (b) — the non-monotonicity the missing term created. Guarded by the
     * ceiling mismatch documented above, which is the one way the two can
     * legitimately differ and is not reachable from any shipped profile. */
    if (r_miss <= 0.9999) {
        CHECK(r_hit >= r_miss - 1e-12,
              "a detection never leaves a track worse off than no detection");
    }

    /* (c) — and at g = 0 the two are the same number, not merely ordered.
     * The clamp at 0.9999 is the one way they can legitimately differ, so it
     * is excluded rather than papered over. */
    const double r_zero = hit(r, pd, lambda, 0.0);
    if (r_miss <= 0.9999) {
        CHECK(fabs(r_zero - r_miss) <= 1e-9,
              "with nothing to explain the detection, a hit decays exactly like a miss");
    }

    /* (d) */
    int gj = nondet_int();
    ASSUME(gj >= gi && gj <= 4);
    const double g_hi = (double)gj / 2.0;
    CHECK(hit(r, pd, lambda, g_hi) >= r_hit - 1e-12,
          "a better-fitting detection is never weaker evidence");

    /* (e) */
    CHECK(hit(0.0, pd, lambda, g) == 0.0,
          "no measurement resurrects a track that does not exist");
    return 0;
}
