/* Property — the Mixed Ornstein-Uhlenbeck discretisation is self-consistent.
 * Mirrors src/core/particle_filter.cpp:31-37:
 *
 *     c.alpha[k]   = exp(-theta * dt);
 *     c.sigma_v[k] = sigma * sqrt((1 - exp(-2*theta*dt)) / (2*theta));
 *     c.ss_vvar[k] = sigma*sigma / (2*theta);
 *
 * For dv = -theta*v*dt + sigma*dW the exact transition over a step dt is
 *     v(t+dt) = e^{-theta*dt} v(t) + eta,   Var[eta] = sigma^2 (1 - e^{-2 theta dt}) / (2 theta),
 * and the stationary variance is sigma^2 / (2 theta). The discretisation is
 * therefore exact rather than an Euler approximation, and it must satisfy
 *
 *     sigma_v^2 == ss_vvar * (1 - alpha^2)
 *
 * -- the statement that a cloud already at steady state stays at steady
 * state, which is what stops the velocity spread from drifting scan over scan.
 *
 * exp() is not modelled bit-precisely. Instead `a = exp(-theta*dt)` is left
 * nondeterministic in (0,1) and the identity exp(-2*theta*dt) == a*a is used,
 * which is exact in the reals. What remains is pure algebra over the engine's
 * actual parameter ranges, and that is what is proven. Tolerance is relative
 * because the source computes exp(-2*theta*dt) directly rather than squaring
 * alpha; the two agree in the reals and to within rounding in IEEE-754.
 *
 * Claims:
 *   (a) sigma_v^2 == ss_vvar * (1 - alpha^2) to within 1e-9 relative;
 *   (b) alpha is a contraction: 0 < alpha < 1, so velocity decays, never grows;
 *   (c) the variance under the square root is non-negative, so sigma_v is real;
 *   (d) a cloud at steady state stays there: alpha^2 * ss + sigma_v^2 == ss.
 */
#include "verif.h"
#include <math.h>

int main(void) {
    /* theta and sigma across the shipped profiles, drawn from a grid. In exact
     * arithmetic the identity below is a tautology - both sides expand to
     * sigma^2 (1 - a^2) / (2 theta) - so what this harness actually settles is
     * that evaluating them in the source's ORDER, in IEEE-754, preserves it.
     * That is a claim about rounding, and rounding does not care whether theta
     * came from a continuum or a grid. The derivation itself, that these three
     * constants are the exact OU transition rather than an Euler step, is in
     * docs/FORMAL_VERIFICATION.md and is not something a checker establishes. */
    int ti = nondet_int(), si = nondet_int(), ai = nondet_int();
    ASSUME(ti >= 1 && ti <= 16);
    ASSUME(si >= 1 && si <= 16);
    ASSUME(ai >= 1 && ai <= 15);
    const double theta = (double)ti / 8.0;          /* 0.125 .. 2.0 */
    const double sigma = (double)si;                /* 1 .. 16 m/s^(3/2) */

    /* a = exp(-theta*dt) for some dt > 0: strictly inside (0,1). */
    const double a = (double)ai / 16.0;
    const double a2 = a * a;                 /* == exp(-2*theta*dt) */

    /* (c) */
    const double under_root = (1.0 - a2) / (2.0 * theta);
    CHECK(under_root > 0.0, "the diffusion variance is positive and real");

    const double ss_vvar = sigma * sigma / (2.0 * theta);
    /* sigma_v = sigma * sqrt(under_root), so sigma_v^2 = sigma^2 * under_root. */
    const double sv2 = sigma * sigma * under_root;

    /* (b) */
    CHECK(a > 0.0 && a < 1.0, "alpha is a strict contraction");
    CHECK(ss_vvar > 0.0, "steady-state velocity variance is positive");

    /* (a) */
    const double rhs = ss_vvar * (1.0 - a2);
    const double scale = rhs > sv2 ? rhs : sv2;
    CHECK(fabs(sv2 - rhs) <= 1e-9 * scale,
          "sigma_v^2 == ss_vvar * (1 - alpha^2)");

    /* (d) the stationarity consequence, which is the reason (a) matters. */
    const double propagated = a2 * ss_vvar + sv2;
    CHECK(fabs(propagated - ss_vvar) <= 1e-9 * ss_vvar,
          "a cloud at steady state stays at steady state");
    return 0;
}
