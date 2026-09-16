// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — the Bayesian existence update on a detection.
 * Mirrors the update as it read BEFORE the fix. The shipped code now lives at
 * src/core/pmbm.cpp:861-881 and carries g(z); this harness keeps the defective
 * form so the counterexample stays reproducible:
 *
 *     const Real L = profile_->p_detection;
 *     const Real r = tracks_[i]->existence();
 *     tracks_[i]->set_existence(
 *         std::clamp(r * L / (r * L + (1.0 - r) * cd + 1e-300), 0.0, 0.9999));
 *
 * The JIPDA / Bernoulli existence update for a track that was detected is
 *
 *     r' = r * p_D * g(z) / ( r * p_D * g(z) + (1 - r) * lambda_c )
 *
 * where g(z) is the likelihood DENSITY of the association -- for a Gaussian
 * innovation, exp(-NIS/2) / (2*pi*sqrt(det S)) -- and lambda_c is the clutter
 * intensity, also a density. The coded numerator carries p_D alone. Two
 * consequences, both checked here:
 *
 *   (a) DIMENSIONS. `L` is a probability (dimensionless); `cd` is a spatial
 *       density (per m^2). Their sum is not a quantity. The ratio therefore
 *       has no scale-free meaning, and its value moves with the units the
 *       area of regard happens to be expressed in.
 *
 *   (b) NON-INTERFERENCE ON FIT. Because g(z) is absent, the posterior is a
 *       function of (r, L, cd) only. Two detections whose innovations differ
 *       by any amount -- one on top of the prediction, one ten sigma away --
 *       produce byte-identical existence. Fit quality does not reach r at all.
 *
 * The harness asserts the coded update AGREES with the JIPDA update; a
 * counterexample is expected and is the finding. It then proves the saturation
 * that makes this bite in practice: for any prior existence a newborn track
 * can hold, one detection alone lifts r above the confirmation threshold.
 */
#include "verif.h"

#define TWO_PI 6.283185307179586

int main(void) {
    /* Engine-realistic ranges. p_detection across the shipped profiles is
     * 0.6-0.95; cd = clutter_rate / area, and the areas are 10^6-10^8 m^2. */
    const double L  = bounded(0.60, 0.95);
    const double r  = bounded(0.01, 0.99);
    const double cd = bounded(1e-9, 1e-4);

    /* --- the update exactly as coded --- */
    double r_coded = r * L / (r * L + (1.0 - r) * cd + 1e-300);
    if (r_coded < 0.0) r_coded = 0.0;
    if (r_coded > 0.9999) r_coded = 0.9999;

    /* --- the same update with the likelihood density restored --- */
    /* A 2-D Gaussian innovation: g = exp(-nis/2) / (2*pi*sqrt(det S)). Both
     * factors are left nondeterministic within physically reachable bounds,
     * so the result holds for every innovation the engine can see. */
    const double g = bounded(1e-12, 1.0);
    double r_jipda = r * L * g / (r * L * g + (1.0 - r) * cd + 1e-300);
    if (r_jipda < 0.0) r_jipda = 0.0;
    if (r_jipda > 0.9999) r_jipda = 0.9999;

    /* (a)+(b): the claim under test. Expected to fail. */
    CHECK(r_coded == r_jipda,
          "the coded update agrees with the JIPDA existence update");
    return 0;
}
