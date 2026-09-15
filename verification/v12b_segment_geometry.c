/* Property — the road-network projection lands on the road (part 2 of 2).
 * The other half of v12, carrying the two claims that do NOT discharge:
 *
 *   (b) for ANY parameter in [0,1], the affine combination `a + (b-a)t` lands
 *       inside the segment's bounding box;
 *   (c) the returned tangent is unit-length -- which align() depends on, since
 *       it computes dir * v.dot(dir), and that is a projection only if
 *       |dir| == 1. sqrt is modelled by its two defining properties, as in v05
 *       and v11, rather than encoded.
 *
 * Neither CBMC nor ESBMC closes either claim. It is not the domain: CBMC still
 * runs past four minutes with the coordinates narrowed from +-1e4 to +-10, so
 * the difficulty is intrinsic to comparing an IEEE product against a scaled
 * tolerance, which is the shape both claims have and the shape bit-blasting
 * handles worst. Narrowing further would not make the claim more true, only
 * less useful, so the domains are left where the engine actually works.
 *
 * Kept, marked SLOW, and reported by run.sh rather than quietly dropped. Both
 * claims are also covered concretely by tests/test_motion_constraint.
 */
#include "verif.h"
#include <math.h>

int main(void) {
    /* ---- (b) the affine combination, for any parameter the clamp can yield. */
    {
        const double ax = bounded(-1e4, 1e4), ay = bounded(-1e4, 1e4);
        const double bx = bounded(-1e4, 1e4), by = bounded(-1e4, 1e4);
        const double t  = bounded(0.0, 1.0);

        const double qx = ax + (bx - ax) * t;
        const double qy = ay + (by - ay) * t;

        const double lox = ax < bx ? ax : bx, hix = ax < bx ? bx : ax;
        const double loy = ay < by ? ay : by, hiy = ay < by ? by : ay;
        const double eps = 1e-9 * (1.0 + fabs(ax) + fabs(bx) + fabs(ay) + fabs(by));

        CHECK(qx >= lox - eps && qx <= hix + eps, "projection stays within the segment in x");
        CHECK(qy >= loy - eps && qy <= hiy + eps, "projection stays within the segment in y");
        /* The endpoints are reached exactly, which is what makes the clamped
         * parameter mean "the nearest point ON the segment" rather than near it. */
        if (t == 0.0) CHECK(qx == ax && qy == ay, "t = 0 is the first endpoint");
        if (t == 1.0) CHECK(qx == bx && qy == by, "t = 1 is the second endpoint");
    }

    /* ---- (c) the tangent is unit-length, which align() depends on: it returns
     * dir * v.dot(dir), and that is a projection only if |dir| == 1. */
    {
        const double abx = bounded(-1e4, 1e4), aby = bounded(-1e4, 1e4);
        const double len_sq = abx * abx + aby * aby;
        ASSUME(len_sq >= 1e-12);            /* the non-degenerate branch */

        /* len == sqrt(len_sq), modelled by non-negativity and by squaring back
         * to the argument within one rounding, so the proof holds for any
         * conforming square root rather than one implementation of it. */
        double len = nondet_double();
        ASSUME(len == len);
        ASSUME(len > 0.0);
        ASSUME(len * len >= len_sq * (1.0 - 1e-12));
        ASSUME(len * len <= len_sq * (1.0 + 1e-12));

        const double dx = abx / len, dy = aby / len;
        CHECK(dx == dx && dy == dy, "the tangent is never NaN");
        /* |d|^2 * len^2 == len_sq, to within the modelled rounding. */
        const double m2 = dx * dx + dy * dy;
        CHECK(fabs(m2 * (len * len) - len_sq) <= 1e-6 * (len_sq + 1.0),
              "the tangent is unit-length, so align() is a true projection");
    }

    return 0;
}
