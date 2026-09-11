/* Property 2 — std::clamp(v, lo, hi) is v < lo ? lo : (hi < v ? hi : v).
 * Track::update_hit/update_miss and PmbmManager both wrap the existence update
 * in std::clamp(..., 0.0, 1.0), which reads like a sanitiser. Assert that it
 * is one, expecting a counterexample. */
#include "verif.h"
static double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (hi < v ? hi : v);
}
int main(void) {
    double v = nondet_double();
    ASSUME(v != v);                        /* v IS NaN */
    const double r = clamp(v, 0.0, 1.0);
    CHECK(r >= 0.0 && r <= 1.0, "clamp confines a NaN to [0,1]");
    return 0;
}
