// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/sim/world.hpp"

#include <algorithm>
#include <cmath>

namespace trace::sim {

void World::step(Real dt) {
    time_ += dt;

    for (auto& e : entities_) {
        if (!e.active) continue;

        // Latch the cruise speed once, BEFORE anything can zero the velocity it
        // is latched from. The 1.4 m/s default is a walking pace for entities a
        // scenario never gave a velocity; it is meaningless in any other unit
        // system, so it must not become reachable by accident - and it was.
        // The dwell branch below sets velocity to zero and `continue`s past
        // this, so an entity that STARTED the run dwelling reached the latch
        // one step later with its configured speed already erased, fell to the
        // walking-pace default, and walked at 1.4 m/s for the rest of the run
        // whatever the scenario had asked for.
        if (e.cruise_mps <= 0.0) {
            e.cruise_mps = e.velocity.norm() > 1e-6 ? e.velocity.norm() : 1.4;
        }

        if (e.dwell_remaining_s > 0.0) {
            e.dwell_remaining_s -= dt;
            e.velocity = Vec2{0.0, 0.0};
            e.mode = "standing";
            continue;
        }

        if (e.waypoint_index >= e.waypoints.size()) {
            e.velocity = Vec2{0.0, 0.0};
            e.mode = "standing";
            continue;
        }

        // Spend the scan's travel budget along the route, waypoint by
        // waypoint, rather than stopping dead at the first one reached.
        // Reaching a waypoint mid-scan used to cost the remainder of that
        // scan, and a repeated waypoint - which any two concatenated path
        // segments produce at their join - cost a whole scan while moving
        // nowhere. Following consecutive segments stays exactly on the route,
        // so this still cannot cut a corner.
        Real remaining = e.cruise_mps * dt;
        while (remaining > 0.0 && e.waypoint_index < e.waypoints.size()) {
            const Vec2 delta = e.waypoints[e.waypoint_index] - e.position;
            const Real dist = delta.norm();
            if (dist < 1e-9) {
                ++e.waypoint_index;            // already there; costs nothing
                continue;
            }
            if (dist <= remaining) {
                e.position = e.waypoints[e.waypoint_index];
                remaining -= dist;
                ++e.waypoint_index;
            } else {
                e.position += delta.unit() * remaining;
                remaining = 0.0;
            }
        }

        if (e.waypoint_index < e.waypoints.size()) {
            const Vec2 heading = e.waypoints[e.waypoint_index] - e.position;
            if (heading.norm() > 1e-9) e.velocity = heading.unit() * e.cruise_mps;
        } else {
            e.velocity = Vec2{0.0, 0.0};
            e.mode = "standing";
        }
    }

    if (on_step) on_step(*this, dt);
}

}  // namespace trace::sim
