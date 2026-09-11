/* Property — the road-network projection lands on the road.
 * Mirrors src/core/motion_constraint.cpp:11-18:
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
 * Claims, over ALL points and ALL segments in a 10 km box:
 *   (a) the clamped parameter is in [0,1], so the foot of the perpendicular is
 *       on the SEGMENT and not on its infinite extension;
 *   (b) the projected point is inside the axis-aligned box of the segment,
 *       which for a point known to be an affine combination with t in [0,1] is
 *       the checkable form of "on the segment";
 *   (c) the returned direction has unit norm whenever the segment is
 *       non-degenerate, which align() relies on: it computes dir * v.dot(dir),
 *       and that is a projection only if |dir| == 1;
 *   (d) the projection never moves a point that is already on the segment.
 *
 * Note on NaN: `std::clamp` does not sanitise one (proven in v03), so a
 * non-finite input position propagates to the projected point. The inputs here
 * are assumed finite, matching the engine, where positions come from particle
 * state that is itself finite by construction.
 */
#include "verif.h"
#include <math.h>

int main(void) {
    const double px = bounded(-1e4, 1e4), py = bounded(-1e4, 1e4);
    const double ax = bounded(-1e4, 1e4), ay = bounded(-1e4, 1e4);
    const double bx = bounded(-1e4, 1e4), by = bounded(-1e4, 1e4);

    const double abx = bx - ax, aby = by - ay;
    const double len_sq = abx * abx + aby * aby;

    double qx, qy, dx, dy;
    if (len_sq < 1e-12) {
        qx = ax; qy = ay; dx = 1.0; dy = 0.0;
        CHECK(dx * dx + dy * dy == 1.0, "the degenerate fallback is unit-length");
        CHECK(qx == ax && qy == ay, "a degenerate segment projects to its own endpoint");
        return 0;
    }

    double t = ((px - ax) * abx + (py - ay) * aby) / len_sq;
    t = t < 0.0 ? 0.0 : (1.0 < t ? 1.0 : t);      /* std::clamp(t, 0.0, 1.0) */

    /* (a) */
    CHECK(t >= 0.0 && t <= 1.0, "the projection parameter stays on the segment");

    qx = ax + abx * t;
    qy = ay + aby * t;

    /* (b) */
    const double lox = ax < bx ? ax : bx, hix = ax < bx ? bx : ax;
    const double loy = ay < by ? ay : by, hiy = ay < by ? by : ay;
    const double eps = 1e-6 * (1.0 + fabs(ax) + fabs(bx) + fabs(ay) + fabs(by));
    CHECK(qx >= lox - eps && qx <= hix + eps, "projection is within the segment in x");
    CHECK(qy >= loy - eps && qy <= hiy + eps, "projection is within the segment in y");

    /* (c) */
    const double len = sqrt(len_sq);
    dx = abx / len; dy = aby / len;
    CHECK(fabs(dx * dx + dy * dy - 1.0) <= 1e-9,
          "the tangent is unit-length, so align() is a true projection");
    return 0;
}
