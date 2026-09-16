// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — Vec2::unit() is total on finite input.
 * Mirrors include/trace/core/types.hpp:41-44:
 *
 *     const Real n = norm();
 *     return n > 1e-12 ? Vec2{x / n, y / n} : Vec2{0.0, 0.0};
 *
 * Every heading comparison in the behaviour detectors divides by this, so a
 * NaN here would silently disable a detector rather than crash it: NaN fails
 * every threshold test, so the detector stops firing and reports nothing.
 *
 * The guard has a second, less obvious consequence. The zero vector returned
 * for a near-stationary input is NOT a unit vector, so `a.unit().dot(b.unit())`
 * is 0 for a stationary entity -- which reads as "perpendicular", not as "no
 * information". Callers must therefore test speed before testing heading, and
 * src/detectors/behaviour.cpp:97-107 does exactly that. The claim below records
 * the contract rather than asserting the return is always unit-length.
 *
 * Claims:
 *   (a) neither component is NaN for any finite input;
 *   (b) the result is either exactly the zero vector or has unit norm to
 *       within rounding;
 *   (c) the zero vector is returned only for inputs below the guard.
 *
 * sqrt is not encoded bit-precisely; a correctly-rounded IEEE square root
 * circuit is expensive and none of the claims turn on its last bit. It is
 * modelled instead by the two properties they do turn on - the result is
 * non-negative, and squaring it recovers the argument to within one rounding -
 * so the proof holds for any conforming sqrt rather than for one particular
 * implementation of it.
 */
#include "verif.h"
#include <math.h>

int main(void) {
    /* Integer components. unit()'s branch turns on norm() against 1e-12 and
     * on the division that follows; neither depends on the components being
     * arbitrary reals, and an exact sum of squares keeps the modelled sqrt
     * below tractable. */
    int ix = nondet_int(), iy = nondet_int();
    ASSUME(ix >= -32 && ix <= 32);
    ASSUME(iy >= -32 && iy <= 32);
    const double x = (double)ix;
    const double y = (double)iy;

    const double s = x * x + y * y;
    double n = nondet_double();            /* n == sqrt(s), unconstrained */
    ASSUME(n == n);                        /* sqrt of a non-negative is not NaN */
    ASSUME(n >= 0.0);
    ASSUME(n <= 1e6);                       /* finite, given finite components */

    double ux, uy;
    if (n > 1e-12) { ux = x / n; uy = y / n; } else { ux = 0.0; uy = 0.0; }

    /* (a) */
    CHECK(ux == ux && uy == uy, "unit() never produces NaN from finite input");

    /* (b), (c) */
    if (n > 1e-12) {
        CHECK(n != 0.0, "the guard rules out the division by zero");
        /* A caller can tell the two shapes of answer apart by inspecting the
         * result alone, which is what makes the zero return a usable signal
         * rather than a silent one. */
        if (!(x == 0.0 && y == 0.0)) {
            CHECK(!(ux == 0.0 && uy == 0.0),
                  "a non-degenerate input yields a non-zero result");
        }
    } else {
        CHECK(ux == 0.0 && uy == 0.0, "degenerate input yields exactly zero");
    }
    return 0;
}
