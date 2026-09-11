// TRACE — simulation ground truth.
//
// A World owns entities whose true positions are known. Sensors observe it
// imperfectly and produce Observations; the engine sees only those. Keeping
// truth and observation strictly separate is what makes the simulations
// meaningful — the engine can never accidentally read the answer.
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "trace/core/rng.hpp"
#include "trace/core/types.hpp"

namespace trace::sim {

/// One simulated entity, with truth the engine never sees.
struct Entity {
    std::string id;                ///< ground-truth id, e.g. "traveller_0"
    Vec2 position{};
    Vec2 velocity{};               ///< metres per second
    std::string mode{"walking"};   ///< true motion regime
    bool active{true};             ///< inactive entities emit no detections
    Vec2 goal{};
    std::vector<Vec2> waypoints;   ///< remaining path
    std::size_t waypoint_index{0};

    /// Cruise speed, latched from the initial `velocity` on the first step.
    /// Velocity carries both heading and speed, and heading is rewritten at
    /// every waypoint; deriving speed from it again each step means one
    /// degenerate waypoint can lose it permanently.
    Real cruise_mps{0.0};

    /// Free-form per-scenario state (dwell timers, intent flags).
    Real dwell_remaining_s{0.0};
    std::string role;              ///< scenario's own label, for scoring
};

/// The truth state at one instant.
struct WorldSnapshot {
    Real timestamp{0.0};
    std::vector<Entity> entities;
};

class World {
public:
    explicit World(std::uint64_t seed = 1234) : rng_(seed) {}

    Entity& add(Entity e) {
        entities_.push_back(std::move(e));
        return entities_.back();
    }

    /// Advance every entity by `dt` seconds along its waypoint path.
    void step(Real dt);

    [[nodiscard]] std::vector<Entity>& entities() { return entities_; }
    [[nodiscard]] const std::vector<Entity>& entities() const { return entities_; }
    [[nodiscard]] Real time() const { return time_; }
    [[nodiscard]] WorldSnapshot snapshot() const { return {time_, entities_}; }
    [[nodiscard]] Rng& rng() { return rng_; }

    /// Per-scenario hook run after every step (dwell logic, re-tasking).
    std::function<void(World&, Real)> on_step;

private:
    std::vector<Entity> entities_;
    Real time_{0.0};
    Rng rng_;
};

}  // namespace trace::sim
