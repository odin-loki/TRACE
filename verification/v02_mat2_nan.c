/* Property 1b — does the singularity guard sanitise a NaN determinant?
 * The guard is `if (fabs(d) < 1e-15) d = ...`. A NaN compares false against
 * everything, so it takes the else branch untouched. This harness asserts the
 * guard DOES sanitise, expecting a counterexample: the point is to establish
 * where NaN is and is not stopped, not to claim the code is wrong here. */
#include "verif.h"
#include <math.h>
int main(void) {
    double d = nondet_double();
    ASSUME(d != d);                        /* d IS NaN */
    if (fabs(d) < 1e-15) d = (d < 0.0 ? -1e-15 : 1e-15);
    CHECK(d == d, "guard sanitises a NaN determinant");
    return 0;
}
