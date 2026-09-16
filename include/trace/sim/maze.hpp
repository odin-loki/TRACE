// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — maze / camera-grid simulation.
//
// A maze is a cheap stand-in for a real camera estate. It gives you exactly the
// properties that make multi-camera tracking hard, with none of the cost:
//
//   * walls force travellers along constrained, non-linear routes, so a
//     straight-line predictor is not enough;
//   * the maze is partitioned into rectangular view panels - one camera each -
//     so a traveller crossing a boundary is a real inter-camera handoff;
//   * panels can be switched off, creating blind corridors where the engine
//     must hold identity with no evidence at all and then reacquire;
//   * two travellers inside one panel are exactly the case that breaks naive
//     association.
//
// Everything is rendered in the console, so a whole surveillance estate can be
// inspected frame by frame without a GUI.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <string>
#include <vector>

#include "trace/core/report.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/types.hpp"
#include "trace/sim/sensor.hpp"
#include "trace/sim/world.hpp"

namespace trace::sim {

/// A grid cell, addressed by column and row.
struct Cell {
    int col{0};
    int row{0};

    friend bool operator==(const Cell& a, const Cell& b) {
        return a.col == b.col && a.row == b.row;
    }
};

/// A rectangular maze with walls on cell edges.
class Maze {
public:
    Maze(int width, int height, Real cell_size_m = 10.0);

    /// Carve a perfect maze (exactly one path between any two cells).
    void generate(Rng& rng);

    /// Knock out extra walls to create loops, so travellers have route choices
    /// and a predictor cannot simply assume the unique path.
    void add_loops(Rng& rng, Real fraction);

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] Real cell_size() const { return cell_size_; }

    [[nodiscard]] bool in_bounds(Cell c) const {
        return c.col >= 0 && c.col < width_ && c.row >= 0 && c.row < height_;
    }

    /// Is there a wall between adjacent cells a and b?
    [[nodiscard]] bool wall_between(Cell a, Cell b) const;

    /// Cells reachable from `c` in one step.
    [[nodiscard]] std::vector<Cell> neighbours(Cell c) const;

    /// Shortest path through the maze, inclusive of both ends.
    [[nodiscard]] std::vector<Cell> path(Cell from, Cell to) const;

    /// Centre of a cell in world metres.
    [[nodiscard]] Vec2 centre_of(Cell c) const {
        return Vec2{(c.col + 0.5) * cell_size_, (c.row + 0.5) * cell_size_};
    }

    /// Which cell contains a world point.
    [[nodiscard]] Cell cell_at(Vec2 p) const {
        return Cell{static_cast<int>(std::floor(p.x / cell_size_)),
                    static_cast<int>(std::floor(p.y / cell_size_))};
    }

    [[nodiscard]] Area bounds() const {
        return Area{0.0, width_ * cell_size_, 0.0, height_ * cell_size_};
    }

    /// Convert a cell path into world-space waypoints.
    [[nodiscard]] std::vector<Vec2> waypoints(const std::vector<Cell>& cells) const;

    /// A random cell that is not inside any disabled region.
    [[nodiscard]] Cell random_cell(Rng& rng) const;

private:
    /// Wall bitmask per cell: north=1, east=2, south=4, west=8.
    [[nodiscard]] std::uint8_t& walls(Cell c) {
        return walls_[static_cast<std::size_t>(c.row) * width_ + c.col];
    }
    [[nodiscard]] std::uint8_t walls(Cell c) const {
        return walls_[static_cast<std::size_t>(c.row) * width_ + c.col];
    }

    int width_{0};
    int height_{0};
    Real cell_size_{10.0};
    std::vector<std::uint8_t> walls_;
};

/// The camera estate laid over a maze.
struct CameraGrid {
    /// Divide the maze into `cols` x `rows` panels, one camera each.
    static std::vector<CameraPanel::Config> partition(const Maze& maze, int cols,
                                                      int rows,
                                                      Real p_detect = 0.85,
                                                      Real pos_noise_m = 1.5);
};

/// Console renderer.
struct MazeRenderer {
    bool colour{true};
    bool show_walls{true};
    bool show_coverage{true};
    bool show_truth{true};
    bool show_tracks{true};

    /// One frame: maze, camera panels, true entities, and engine tracks.
    [[nodiscard]] std::string render(const Maze& maze,
                                     const std::vector<CameraPanel::Config>& panels,
                                     const std::vector<Entity>& entities,
                                     const std::vector<TargetReport>& tracks) const;

    /// Colour output honours both the struct flag and a NO_COLOR environment
    /// variable, so piping the sim to a file gives clean text.
    [[nodiscard]] bool colour_enabled() const;

    /// Legend explaining the glyphs.
    [[nodiscard]] std::string legend(
        const std::vector<CameraPanel::Config>& panels,
        const std::vector<Entity>& entities) const;
};

}  // namespace trace::sim
