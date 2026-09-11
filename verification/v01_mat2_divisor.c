/* Property 1a — the singularity guard in Mat2::inverse.
 * include/trace/core/types.hpp:61-63, verbatim:
 *     Real d = aa * cc - b * b;
 *     if (std::abs(d) < 1e-15) d = (d < 0.0 ? -1e-15 : 1e-15);
 * Proven for an ARBITRARY d, which is stronger than deriving d from a,b,c:
 * whatever the determinant computation produces, the guard must leave a
 * divisor that cannot be zero. */
#include "verif.h"
#include <math.h>
int main(void) {
    double d = nondet_double();
    ASSUME(d == d);                       /* see p1b for the NaN case */
    if (fabs(d) < 1e-15) d = (d < 0.0 ? -1e-15 : 1e-15);
    CHECK(d != 0.0, "divisor non-zero");
    CHECK(fabs(d) >= 1e-15, "divisor bounded away from zero");
    return 0;
}
