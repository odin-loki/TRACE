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

void RoadNetwork::find_junctions() {
    junctions_.clear();
    if (segments_.size() < 2) return;
    constexpr Real kSame = 1e-6;

    const auto remember = [&](Vec2 p) {
        for (const Vec2& j : junctions_) {
            if (distance(j, p) <= kSame) return;
        }
        junctions_.push_back(p);
    };

    // Endpoints that three or more segment ends meet at. Two ends meeting is
    // just a corner - the network still has one answer for which way to go.
    std::vector<Vec2> ends;
    ends.reserve(segments_.size() * 2);
    for (const Segment& s : segments_) {
        ends.push_back(s.a);
        ends.push_back(s.b);
    }
    for (std::size_t i = 0; i < ends.size(); ++i) {
        int n = 0;
        for (const Vec2& e : ends) {
            if (distance(ends[i], e) <= kSame) ++n;
        }
        if (n >= 3) remember(ends[i]);
    }

    // And crossings, which are junctions that share no endpoint. A grid of
    // streets is built from full-width and full-height lines that cross
    // without meeting end to end, so looking only at shared endpoints finds no
    // junctions in a city grid at all - which is the one layout where almost
    // every point of interest is a junction.
    //
    // O(n^2) in segments, once, at construction.
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        for (std::size_t k = i + 1; k < segments_.size(); ++k) {
            const Vec2 p = segments_[i].a;
            const Vec2 r = segments_[i].b - p;
            const Vec2 q = segments_[k].a;
            const Vec2 sd = segments_[k].b - q;
            const Real denom = r.x * sd.y - r.y * sd.x;
            if (std::abs(denom) < 1e-12) continue;        // parallel
            const Vec2 qp = q - p;
            const Real t = (qp.x * sd.y - qp.y * sd.x) / denom;
            const Real u = (qp.x * r.y - qp.y * r.x) / denom;
            // Inside both segments, and strictly inside at least one. A
            // crossing is a four-way junction and a T - one street ending on
            // the interior of another - is a three-way one; both qualify.
            // Only an endpoint meeting an endpoint is excluded, because that
            // is a plain corner where the network still has a single answer,
            // and the endpoint test above already counts those properly.
            constexpr Real kEdge = 1e-9;
            if (t < -kEdge || t > 1.0 + kEdge) continue;
            if (u < -kEdge || u > 1.0 + kEdge) continue;
            const bool t_interior = t > kEdge && t < 1.0 - kEdge;
            const bool u_interior = u > kEdge && u < 1.0 - kEdge;
            if (!t_interior && !u_interior) continue;
            remember(Vec2{p.x + r.x * t, p.y + r.y * t});
        }
    }
}

Vec2 RoadNetwork::project_unconditional(Vec2 position) const {
    if (segments_.empty()) return position;
    // At a junction the network has nothing to say about which way an entity
    // is going, and projecting to the nearest branch says it anyway - which
    // collapses a belief that ought to span both. Left alone here, the cloud
    // keeps whatever spread it has; once past the junction each particle is
    // pulled onto whichever branch it actually drifted towards, so the
    // multi-modality survives the one place it matters.
    if (junction_radius_ > 0.0 && at_junction(position)) return position;
    return nearest(position).first;
}

Vec2 RoadNetwork::align_unconditional(Vec2 position, Vec2 velocity) const {
    if (segments_.empty()) return velocity;
    // Same reasoning as the projection: at a junction there is no single local
    // direction of travel to align to, and picking one discards the very
    // information the next few scans will supply.
    if (junction_radius_ > 0.0 && at_junction(position)) return velocity;
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
