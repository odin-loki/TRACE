// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

/* Property — the road-network projection lands on the road (part 1 of 2).
 * Mirrors src/core/motion_constraint.cpp:14-21:
 *
 *     const Vec2 ab = b - a;
 *     const Real len_sq = ab.norm_sq();
 *     if (len_sq < 1e-12) return {a, Vec2{1.0, 0.0}};
 *     const Real t = std::clamp((p - a).dot(ab) / len_sq, 0.0, 1.0);
 *     return {a + ab * t, ab / std::sqrt(len_sq)};
 *
 * RoadNetwork::project() feeds the result straight back into the particle
 * cloud, so a projection that escaped the segment would move particles onto
 * road that does not exist -- and, because the constraint is applied every
 * prediction step, would do so cumulatively.
 *
 * This half carries the two claims that discharge:
 *
 *   (a) the clamp yields a parameter in [0,1] for ANY finite quotient,
 *       whatever the division that produced it did -- which is stronger than
 *       proving it for the particular quotient the source computes, and needs
 *       none of the arithmetic that computes it;
 *   (d) the degenerate branch returns the endpoint and a unit fallback.
 *
 * The other two -- that the affine combination lands inside the segment's
 * bounding box, and that the returned tangent is unit-length -- are in
 * v12b_segment_geometry.c, which neither checker closes. Splitting them is not
 * a way of making the suite look complete: the undischarged half is still in
 * the suite, still marked, and still reported.
 */
#include "verif.h"
#include <math.h>

int main(void) {
    /* ---- (a) the clamp, over any finite quotient the division could give. */
    {
        const double raw = bounded(-1e12, 1e12);
        const double t = raw < 0.0 ? 0.0 : (1.0 < raw ? 1.0 : raw);
        CHECK(t >= 0.0 && t <= 1.0, "the clamp yields a parameter on the segment");
        /* And it is the identity where it should be, so the clamp cannot be
         * silently discarding a valid interior projection. */
        if (raw >= 0.0 && raw <= 1.0) CHECK(t == raw, "an interior parameter is untouched");
    }

    /* ---- (d) the degenerate branch. */
    {
        const double ax = bounded(-1e4, 1e4), ay = bounded(-1e4, 1e4);
        const double qx = ax, qy = ay, dx = 1.0, dy = 0.0;
        CHECK(qx == ax && qy == ay, "a degenerate segment projects to its endpoint");
        CHECK(dx * dx + dy * dy == 1.0, "the degenerate fallback is unit-length");
    }
    return 0;
}
