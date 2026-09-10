#include "trace/core/motion_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trace {
namespace {

/// Closest point to `p` on the segment ab, plus the segment's unit direction.
std::pair<Vec2, Vec2> closest_on_segment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const Real len_sq = ab.norm_sq();
    if (len_sq < 1e-12) return {a, Vec2{1.0, 0.0}};
    // Clamped projection parameter: a segment, not an infinite line.
    const Real t = std::clamp((p - a).dot(ab) / len_sq, 0.0, 1.0);
    return {a + ab * t, ab / std::sqrt(len_sq)};
}

}  // namespace

RoadNetwork RoadNetwork::from_polyline(const std::vector<Vec2>& points,
                                       Real tolerance_m) {
    std::vector<Segment> segs;
    for (std::size_t i = 1; i < points.size(); ++i) {
        segs.push_back(Segment{points[i - 1], points[i]});
    }
    return RoadNetwork(std::move(segs), tolerance_m);
}

RoadNetwork RoadNetwork::grid(Area area, int cols, int rows, Real tolerance_m) {
    std::vector<Segment> segs;
    for (int c = 0; c <= cols; ++c) {
        const Real x = area.xmin + area.width() * c / std::max(cols, 1);
        segs.push_back(Segment{{x, area.ymin}, {x, area.ymax}});
    }
    for (int r = 0; r <= rows; ++r) {
        const Real y = area.ymin + area.height() * r / std::max(rows, 1);
        segs.push_back(Segment{{area.xmin, y}, {area.xmax, y}});
    }
    return RoadNetwork(std::move(segs), tolerance_m);
}

std::pair<Vec2, Vec2> RoadNetwork::nearest(Vec2 position) const {
    Vec2 best_point = position;
    Vec2 best_dir{1.0, 0.0};
    Real best_d = std::numeric_limits<Real>::infinity();

    for (const auto& s : segments_) {
        const auto [pt, dir] = closest_on_segment(position, s.a, s.b);
        const Real d = distance(position, pt);
        if (d < best_d) {
            best_d = d;
            best_point = pt;
            best_dir = dir;
        }
    }
    return {best_point, best_dir};
}

Real RoadNetwork::distance_to(Vec2 position) const {
    if (segments_.empty()) return 0.0;
    return distance(position, nearest(position).first);
}

Vec2 RoadNetwork::project_unconditional(Vec2 position) const {
    if (segments_.empty()) return position;
    return nearest(position).first;
}

Vec2 RoadNetwork::align_unconditional(Vec2 position, Vec2 velocity) const {
    if (segments_.empty()) return velocity;
    const auto [pt, dir] = nearest(position);
    // Keep the along-road component and discard the across-road one, preserving
    // direction of travel. A vehicle's speed is its own; only its heading is
    // dictated by the road.
    return dir * velocity.dot(dir);
}

Vec2 RoadNetwork::project(Vec2 position) const {
    if (segments_.empty()) return position;
    const auto [pt, dir] = nearest(position);
    const Real d = distance(position, pt);
    // Beyond tolerance the entity is genuinely off-network; snapping it would
    // assert something we do not know.
    if (d > tolerance_) return position;
    return pt;
}

Vec2 RoadNetwork::align(Vec2 position, Vec2 velocity) const {
    if (segments_.empty()) return velocity;
    const auto [pt, dir] = nearest(position);
    if (distance(position, pt) > tolerance_) return velocity;

    // Keep the along-road component and discard the across-road one, preserving
    // direction of travel. A vehicle's speed is its own; only its heading is
    // dictated by the road.
    const Real along = velocity.dot(dir);
    return dir * along;
}

}  // namespace trace
