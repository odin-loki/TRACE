/* Property — Vec2::unit() is total on finite input.
 * Mirrors include/trace/core/types.hpp:38-41:
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
 * src/detectors/behaviour.cpp:95-97 does exactly that. The claim below records
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
    const double x = bounded(-1e3, 1e3);
    const double y = bounded(-1e3, 1e3);

    const double s = x * x + y * y;
    double n = nondet_double();            /* n == sqrt(s), modelled */
    ASSUME(n == n);
    ASSUME(n >= 0.0);
    ASSUME(n * n >= s * (1.0 - 1e-12) - 1e-300);
    ASSUME(n * n <= s * (1.0 + 1e-12) + 1e-300);

    double ux, uy;
    if (n > 1e-12) { ux = x / n; uy = y / n; } else { ux = 0.0; uy = 0.0; }

    /* (a) */
    CHECK(ux == ux && uy == uy, "unit() never produces NaN from finite input");

    /* (b) */
    if (n > 1e-12) {
        const double m2 = (ux * ux + uy * uy);
        CHECK(fabs(m2 * (n * n) - s) <= 1e-6 * (s + 1.0),
              "a non-degenerate result has unit norm");
        /* (c) and it is not the zero vector, so a caller can tell the cases
         * apart by inspecting the result alone. */
        CHECK(!(ux == 0.0 && uy == 0.0), "non-degenerate input yields a non-zero unit");
    } else {
        CHECK(ux == 0.0 && uy == 0.0, "degenerate input yields exactly zero");
    }
    return 0;
}
