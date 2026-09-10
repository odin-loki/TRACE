#include "trace/sim/world.hpp"

#include <algorithm>
#include <cmath>

namespace trace::sim {

void World::step(Real dt) {
    time_ += dt;

    for (auto& e : entities_) {
        if (!e.active) continue;

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

        const Vec2 target = e.waypoints[e.waypoint_index];
        const Vec2 delta = target - e.position;
        const Real dist = delta.norm();
        const Real speed = e.velocity.norm() > 1e-6 ? e.velocity.norm() : 1.4;
        const Real travel = speed * dt;

        if (dist <= travel || dist < 1e-6) {
            // Arrived: snap to the waypoint and take the next one, so an entity
            // cannot overshoot a corner and cut through a wall.
            e.position = target;
            ++e.waypoint_index;
            if (e.waypoint_index < e.waypoints.size()) {
                const Vec2 next = e.waypoints[e.waypoint_index] - e.position;
                e.velocity = next.unit() * speed;
            } else {
                e.velocity = Vec2{0.0, 0.0};
                e.mode = "standing";
            }
        } else {
            const Vec2 dir = delta.unit();
            e.position += dir * travel;
            e.velocity = dir * speed;
        }
    }

    if (on_step) on_step(*this, dt);
}

}  // namespace trace::sim
