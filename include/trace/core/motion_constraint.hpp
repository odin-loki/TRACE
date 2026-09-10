// TRACE — motion constraints.
//
// The MOU motion model assumes free space: an entity may move in any direction.
// That is right for a person in a plaza or a ship at sea, and wrong for anything
// confined to a network. A vehicle between two ANPR readers 400 m apart has not
// wandered off across the fields; it is on the road, and a tracker that does not
// know this coasts its estimate into a hedge.
//
// A MotionConstraint is an optional projection applied after each prediction
// step. It leaves the probabilistic machinery untouched - existence,
// association, behaviour detection are all unchanged - and simply restricts
// where the particles are allowed to be.
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "trace/core/types.hpp"

namespace trace {

class MotionConstraint {
public:
    virtual ~MotionConstraint() = default;

    /// Move a particle onto the nearest admissible position.
    [[nodiscard]] virtual Vec2 project(Vec2 position) const = 0;

    /// Align a velocity with the local direction of travel at `position`.
    /// Returning the velocity unchanged makes the constraint positional only.
    [[nodiscard]] virtual Vec2 align(Vec2 position, Vec2 velocity) const = 0;

    /// Project regardless of tolerance. Used once the caller has already
    /// established that the entity is on-network, so individual outlying
    /// particles must still be pulled back rather than escaping permanently.
    [[nodiscard]] virtual Vec2 project_unconditional(Vec2 position) const = 0;
    [[nodiscard]] virtual Vec2 align_unconditional(Vec2 position,
                                                   Vec2 velocity) const = 0;

    /// How far a position may be from the network before the constraint gives
    /// up and leaves it alone. Prevents a detection genuinely off-network -
    /// a vehicle in a car park, a pedestrian crossing a field - from being
    /// yanked onto a road it is not on.
    [[nodiscard]] virtual Real tolerance() const = 0;

    /// Is this position close enough to the network to be considered on it?
    [[nodiscard]] virtual bool on_network(Vec2 position) const = 0;
};

using MotionConstraintPtr = std::shared_ptr<const MotionConstraint>;

/// A network of straight segments: roads, rail, corridors, shipping lanes.
class RoadNetwork final : public MotionConstraint {
public:
    struct Segment {
        Vec2 a;
        Vec2 b;
    };

    RoadNetwork() = default;
    explicit RoadNetwork(std::vector<Segment> segments, Real tolerance_m = 30.0)
        : segments_(std::move(segments)), tolerance_(tolerance_m) {}

    void add(Vec2 a, Vec2 b) { segments_.push_back(Segment{a, b}); }

    /// Build a polyline: a chain of connected segments.
    static RoadNetwork from_polyline(const std::vector<Vec2>& points,
                                     Real tolerance_m = 30.0);

    /// Build a rectangular grid of streets.
    static RoadNetwork grid(Area area, int cols, int rows, Real tolerance_m = 30.0);

    [[nodiscard]] Vec2 project(Vec2 position) const override;
    [[nodiscard]] Vec2 align(Vec2 position, Vec2 velocity) const override;
    [[nodiscard]] Vec2 project_unconditional(Vec2 position) const override;
    [[nodiscard]] Vec2 align_unconditional(Vec2 position,
                                           Vec2 velocity) const override;
    [[nodiscard]] Real tolerance() const override { return tolerance_; }
    [[nodiscard]] bool on_network(Vec2 position) const override {
        return segments_.empty() ? false : distance_to(position) <= tolerance_;
    }

    [[nodiscard]] const std::vector<Segment>& segments() const { return segments_; }

    /// Distance from a point to the network - useful for diagnostics and for
    /// deciding whether an observation is plausibly on-network at all.
    [[nodiscard]] Real distance_to(Vec2 position) const;

private:
    /// Nearest point on the network, and the unit tangent there.
    [[nodiscard]] std::pair<Vec2, Vec2> nearest(Vec2 position) const;

    std::vector<Segment> segments_;
    Real tolerance_{30.0};
};

}  // namespace trace
