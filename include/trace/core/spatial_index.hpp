// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — uniform-grid spatial index.
//
// Five stages of the pipeline ask the same question every scan: which pairs of
// tracks are close enough to interact? Convergence prediction, co-location
// clustering, brush-pass detection, parallel-route detection and network
// contact accumulation all did it by examining every pair, which is quadratic
// in the track count and dominates everything else once there are more than a
// few dozen tracks — measured at n^2.03, against n^1.0 for tracking alone.
//
// Every one of those stages then immediately discards pairs beyond some radius.
// Binning the tracks and only visiting nearby bins answers the same question in
// time proportional to the number of pairs that actually matter.
#pragma once

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "trace/core/types.hpp"

namespace trace {

class SpatialIndex {
public:
    SpatialIndex() = default;

    /// Bin `points` into cells of `cell_size`. The cell size should be the
    /// largest interaction radius any caller will query with; smaller cells
    /// mean more cells to visit per query, larger cells mean more candidates
    /// to reject.
    SpatialIndex(const std::vector<Vec2>& points, Real cell_size)
        : cell_size_(std::max(cell_size, 1e-6)), points_(points) {
        cells_.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            cells_[key_of(points[i])].push_back(i);
        }
    }

    /// Indices of points within `radius` of `centre`, excluding `skip`.
    ///
    /// The radius may exceed the cell size; the search widens accordingly.
    [[nodiscard]] std::vector<std::size_t> within(Vec2 centre, Real radius,
                                                  std::size_t skip = kNone) const {
        std::vector<std::size_t> out;
        if (points_.empty()) return out;

        const auto reach = static_cast<long>(std::ceil(radius / cell_size_));
        const long cx = cell_coord(centre.x);
        const long cy = cell_coord(centre.y);
        const Real r2 = radius * radius;

        for (long dx = -reach; dx <= reach; ++dx) {
            for (long dy = -reach; dy <= reach; ++dy) {
                const auto it = cells_.find(pack(cx + dx, cy + dy));
                if (it == cells_.end()) continue;
                for (const std::size_t idx : it->second) {
                    if (idx == skip) continue;
                    if ((points_[idx] - centre).norm_sq() <= r2) out.push_back(idx);
                }
            }
        }
        return out;
    }

    /// Unordered pairs (i, j) with i < j that lie within `radius`.
    ///
    /// Returning i < j only keeps callers from processing each pair twice,
    /// matching the shape of the nested loops this replaces.
    [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> pairs_within(
        Real radius) const {
        std::vector<std::pair<std::size_t, std::size_t>> out;
        for (std::size_t i = 0; i < points_.size(); ++i) {
            for (const std::size_t j : within(points_[i], radius, i)) {
                if (i < j) out.emplace_back(i, j);
            }
        }
        return out;
    }

    [[nodiscard]] std::size_t size() const { return points_.size(); }
    [[nodiscard]] bool empty() const { return points_.empty(); }

    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

private:
    [[nodiscard]] long cell_coord(Real v) const {
        return static_cast<long>(std::floor(v / cell_size_));
    }
    [[nodiscard]] std::size_t key_of(Vec2 p) const {
        return pack(cell_coord(p.x), cell_coord(p.y));
    }
    /// Two signed cell coordinates into one hash key. The offset keeps
    /// negative coordinates distinct after the shift.
    static std::size_t pack(long x, long y) {
        constexpr long kBias = 1L << 20;
        return static_cast<std::size_t>((x + kBias) * (1L << 21) + (y + kBias));
    }

    Real cell_size_{1.0};
    std::vector<Vec2> points_;
    std::unordered_map<std::size_t, std::vector<std::size_t>> cells_;
};

}  // namespace trace
