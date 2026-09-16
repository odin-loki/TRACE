// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — what the missing likelihood costs, stated as a theorem.
 * Same pre-fix update as v08 (shipped code now at src/core/pmbm.cpp:861-881)
 * and the same profile ranges, but here the assertion is expected to HOLD,
 * and holding is the bad news.
 *
 * The shipped profiles birth a track at r_birth = 0.45 and confirm it at
 * r_confirm = 0.55 (include/trace/core/profile.hpp:88-89). This proves that a
 * single detection -- of any quality, at any distance from the prediction,
 * under any clutter density the estimator can produce -- takes a newborn
 * track from 0.45 to above 0.999.
 *
 * So r_confirm does not discriminate. It is cleared by the first detection of
 * every track that gets one, and cleared by no track that does not. The
 * threshold is `n_hits >= 1` wearing a probability's clothing, and the
 * distance between 0.45 and 0.55 chosen for it is not doing any work.
 *
 * With g(z) restored the same update separates cases properly: an association
 * three sigma out (NIS = 9, g ~ e^-4.5) lands near 0.84 rather than 0.9999,
 * and the threshold starts to mean what it is named.
 */
#include "verif.h"

int main(void) {
    const double r_birth   = 0.45;
    const double r_confirm = 0.55;

    const double L  = bounded(0.60, 0.95);    /* p_detection across profiles */
    const double cd = bounded(1e-9, 1e-4);    /* clutter_rate / area_of_regard */

    double r1 = r_birth * L / (r_birth * L + (1.0 - r_birth) * cd + 1e-300);
    if (r1 < 0.0) r1 = 0.0;
    if (r1 > 0.9999) r1 = 0.9999;

    CHECK(r1 > r_confirm,
          "one detection always clears the confirmation threshold");
    CHECK(r1 > 0.999,
          "one detection always saturates existence, whatever the fit");
    return 0;
}
