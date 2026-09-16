// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/sim/maze.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace trace::sim {
namespace {

constexpr std::uint8_t kNorth = 1;
constexpr std::uint8_t kEast = 2;
constexpr std::uint8_t kSouth = 4;
constexpr std::uint8_t kWest = 8;
constexpr std::uint8_t kAllWalls = kNorth | kEast | kSouth | kWest;

/// Direction from a to b, as the wall bit that separates them.
std::uint8_t wall_bit(Cell a, Cell b) {
    if (b.col == a.col && b.row == a.row - 1) return kNorth;
    if (b.col == a.col + 1 && b.row == a.row) return kEast;
    if (b.col == a.col && b.row == a.row + 1) return kSouth;
    if (b.col == a.col - 1 && b.row == a.row) return kWest;
    return 0;
}

std::uint8_t opposite(std::uint8_t bit) {
    switch (bit) {
        case kNorth: return kSouth;
        case kSouth: return kNorth;
        case kEast:  return kWest;
        case kWest:  return kEast;
        default:     return 0;
    }
}

long cell_key(Cell c) {
    return static_cast<long>(c.row) * 100000L + c.col;
}

// ANSI palette for camera panels. Chosen to stay legible on both light and
// dark terminals, and to cycle before repeating in any realistic panel count.
constexpr std::array<const char*, 8> kPanelColours{
    "\033[38;5;39m",  "\033[38;5;208m", "\033[38;5;141m", "\033[38;5;42m",
    "\033[38;5;214m", "\033[38;5;170m", "\033[38;5;80m",  "\033[38;5;222m"};
constexpr const char* kReset = "\033[0m";
constexpr const char* kDim = "\033[38;5;238m";
constexpr const char* kWallColour = "\033[38;5;244m";
constexpr const char* kTruthColour = "\033[1;38;5;231m";
constexpr const char* kTrackColour = "\033[1;38;5;46m";
constexpr const char* kBlindColour = "\033[38;5;52m";

}  // namespace

// ---------------------------------------------------------------------------
// Maze
// ---------------------------------------------------------------------------

Maze::Maze(int width, int height, Real cell_size_m)
    : width_(std::max(width, 2)),
      height_(std::max(height, 2)),
      cell_size_(cell_size_m),
      walls_(static_cast<std::size_t>(width_) * height_, kAllWalls) {}

void Maze::generate(Rng& rng) {
    std::fill(walls_.begin(), walls_.end(), kAllWalls);

    // Recursive backtracker with an explicit stack: produces long winding
    // corridors, which is what we want - a traveller should spend several
    // scans inside one camera panel before handing off.
    std::vector<bool> visited(walls_.size(), false);
    std::vector<Cell> stack;

    Cell start{rng.uniform_int(0, width_ - 1), rng.uniform_int(0, height_ - 1)};
    visited[static_cast<std::size_t>(start.row) * width_ + start.col] = true;
    stack.push_back(start);

    while (!stack.empty()) {
        const Cell current = stack.back();

        std::vector<Cell> unvisited;
        for (const Cell n : {Cell{current.col, current.row - 1},
                             Cell{current.col + 1, current.row},
                             Cell{current.col, current.row + 1},
                             Cell{current.col - 1, current.row}}) {
            if (!in_bounds(n)) continue;
            if (visited[static_cast<std::size_t>(n.row) * width_ + n.col]) continue;
            unvisited.push_back(n);
        }

        if (unvisited.empty()) {
            stack.pop_back();
            continue;
        }

        const Cell next =
            unvisited[static_cast<std::size_t>(
                rng.uniform_int(0, static_cast<int>(unvisited.size()) - 1))];
        const std::uint8_t bit = wall_bit(current, next);
        walls(current) &= static_cast<std::uint8_t>(~bit);
        walls(next) &= static_cast<std::uint8_t>(~opposite(bit));

        visited[static_cast<std::size_t>(next.row) * width_ + next.col] = true;
        stack.push_back(next);
    }
}

void Maze::add_loops(Rng& rng, Real fraction) {
    // A perfect maze has exactly one route between any two points, which makes
    // route prediction trivially easy and unrealistic. Knocking out extra walls
    // gives travellers genuine choices.
    const int target = static_cast<int>(fraction * width_ * height_);
    for (int i = 0; i < target; ++i) {
        const Cell c{rng.uniform_int(0, width_ - 1), rng.uniform_int(0, height_ - 1)};
        std::vector<Cell> candidates;
        for (const Cell n : {Cell{c.col, c.row - 1}, Cell{c.col + 1, c.row},
                             Cell{c.col, c.row + 1}, Cell{c.col - 1, c.row}}) {
            if (in_bounds(n) && wall_between(c, n)) candidates.push_back(n);
        }
        if (candidates.empty()) continue;
        const Cell n = candidates[static_cast<std::size_t>(
            rng.uniform_int(0, static_cast<int>(candidates.size()) - 1))];
        const std::uint8_t bit = wall_bit(c, n);
        walls(c) &= static_cast<std::uint8_t>(~bit);
        walls(n) &= static_cast<std::uint8_t>(~opposite(bit));
    }
}

bool Maze::wall_between(Cell a, Cell b) const {
    if (!in_bounds(a) || !in_bounds(b)) return true;
    const std::uint8_t bit = wall_bit(a, b);
    if (bit == 0) return true;  // not adjacent
    return (walls(a) & bit) != 0;
}

std::vector<Cell> Maze::neighbours(Cell c) const {
    std::vector<Cell> out;
    for (const Cell n : {Cell{c.col, c.row - 1}, Cell{c.col + 1, c.row},
                         Cell{c.col, c.row + 1}, Cell{c.col - 1, c.row}}) {
        if (in_bounds(n) && !wall_between(c, n)) out.push_back(n);
    }
    return out;
}

std::vector<Cell> Maze::path(Cell from, Cell to) const {
    if (!in_bounds(from) || !in_bounds(to)) return {};
    if (from == to) return {from};

    std::unordered_map<long, Cell> came_from;
    std::queue<Cell> q;
    q.push(from);
    came_from[cell_key(from)] = from;

    while (!q.empty()) {
        const Cell c = q.front();
        q.pop();
        if (c == to) break;
        for (const Cell n : neighbours(c)) {
            if (came_from.contains(cell_key(n))) continue;
            came_from[cell_key(n)] = c;
            q.push(n);
        }
    }

    if (!came_from.contains(cell_key(to))) return {};

    std::vector<Cell> out;
    Cell c = to;
    while (!(c == from)) {
        out.push_back(c);
        c = came_from[cell_key(c)];
    }
    out.push_back(from);
    std::reverse(out.begin(), out.end());
    return out;
}

std::vector<Vec2> Maze::waypoints(const std::vector<Cell>& cells) const {
    std::vector<Vec2> out;
    out.reserve(cells.size());
    for (const Cell c : cells) out.push_back(centre_of(c));
    return out;
}

Cell Maze::random_cell(Rng& rng) const {
    return Cell{rng.uniform_int(0, width_ - 1), rng.uniform_int(0, height_ - 1)};
}

// ---------------------------------------------------------------------------
// CameraGrid
// ---------------------------------------------------------------------------

std::vector<CameraPanel::Config> CameraGrid::partition(const Maze& maze, int cols,
                                                       int rows, Real p_detect,
                                                       Real pos_noise_m) {
    std::vector<CameraPanel::Config> out;
    const Real w = maze.width() * maze.cell_size();
    const Real h = maze.height() * maze.cell_size();

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            CameraPanel::Config cfg;
            char name[32];
            std::snprintf(name, sizeof(name), "CAM_%c%d",
                          static_cast<char>('A' + r), c + 1);
            cfg.id = name;
            cfg.footprint = Area{w * c / cols, w * (c + 1) / cols,
                                 h * r / rows, h * (r + 1) / rows};
            cfg.p_detect = p_detect;
            cfg.pos_noise_m = pos_noise_m;
            cfg.false_alarm_rate = 0.08;
            cfg.modality = Modality::GEOINT;
            out.push_back(std::move(cfg));
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------

bool MazeRenderer::colour_enabled() const {
    if (!colour) return false;
    // Respect the no-color convention so redirected output stays readable.
    return std::getenv("NO_COLOR") == nullptr;
}

std::string MazeRenderer::render(const Maze& maze,
                                 const std::vector<CameraPanel::Config>& panels,
                                 const std::vector<Entity>& entities,
                                 const std::vector<TargetReport>& tracks) const {
    // Render grid is (2W+1) x (2H+1): odd coordinates are cells, even are walls.
    const int rw = 2 * maze.width() + 1;
    const int rh = 2 * maze.height() + 1;

    std::vector<std::string> glyph(static_cast<std::size_t>(rw) * rh, " ");
    std::vector<int> colour(static_cast<std::size_t>(rw) * rh, -1);  // panel index
    const auto at = [&](int x, int y) { return static_cast<std::size_t>(y) * rw + x; };

    // ---- Walls ------------------------------------------------------------
    if (show_walls) {
        for (int y = 0; y < rh; ++y) {
            for (int x = 0; x < rw; ++x) {
                const bool even_x = (x % 2) == 0;
                const bool even_y = (y % 2) == 0;
                if (even_x && even_y) {
                    glyph[at(x, y)] = "+";
                } else if (even_y) {
                    const Cell above{(x - 1) / 2, y / 2 - 1};
                    const Cell below{(x - 1) / 2, y / 2};
                    const bool blocked =
                        !maze.in_bounds(above) || !maze.in_bounds(below) ||
                        maze.wall_between(above, below);
                    glyph[at(x, y)] = blocked ? "-" : " ";
                } else if (even_x) {
                    const Cell left{x / 2 - 1, (y - 1) / 2};
                    const Cell right{x / 2, (y - 1) / 2};
                    const bool blocked =
                        !maze.in_bounds(left) || !maze.in_bounds(right) ||
                        maze.wall_between(left, right);
                    glyph[at(x, y)] = blocked ? "|" : " ";
                }
            }
        }
    }

    // ---- Camera coverage --------------------------------------------------
    if (show_coverage) {
        for (int row = 0; row < maze.height(); ++row) {
            for (int col = 0; col < maze.width(); ++col) {
                const Vec2 p = maze.centre_of(Cell{col, row});
                int panel = -1;
                for (std::size_t i = 0; i < panels.size(); ++i) {
                    if (panels[i].enabled && panels[i].footprint.contains(p)) {
                        panel = static_cast<int>(i);
                        break;
                    }
                }
                const std::size_t idx = at(2 * col + 1, 2 * row + 1);
                colour[idx] = panel;
                // A blind cell is marked, not blank: an operator must be able
                // to tell "nothing there" from "we cannot see there".
                glyph[idx] = panel >= 0 ? "." : "x";
            }
        }
    }

    // ---- Engine tracks ----------------------------------------------------
    // Drawn before truth so that where both coincide, truth wins the cell and
    // the reader can still see the track in the legend.
    if (show_tracks) {
        for (std::size_t i = 0; i < tracks.size(); ++i) {
            const Cell c = maze.cell_at(tracks[i].position);
            if (!maze.in_bounds(c)) continue;
            const std::size_t idx = at(2 * c.col + 1, 2 * c.row + 1);
            glyph[idx] = std::to_string(i % 10);
            colour[idx] = -2;  // track colour
        }
    }

    // ---- Ground truth -----------------------------------------------------
    if (show_truth) {
        for (std::size_t i = 0; i < entities.size(); ++i) {
            if (!entities[i].active) continue;
            const Cell c = maze.cell_at(entities[i].position);
            if (!maze.in_bounds(c)) continue;
            const std::size_t idx = at(2 * c.col + 1, 2 * c.row + 1);
            glyph[idx] = std::string(1, static_cast<char>('A' + (i % 26)));
            colour[idx] = -3;  // truth colour
        }
    }

    // ---- Emit -------------------------------------------------------------
    std::ostringstream os;
    for (int y = 0; y < rh; ++y) {
        for (int x = 0; x < rw; ++x) {
            const std::size_t idx = at(x, y);
            const std::string& g = glyph[idx];
            if (!colour_enabled()) {
                os << g;
                continue;
            }
            const int c = colour[idx];
            if (c == -3) {
                os << kTruthColour << g << kReset;
            } else if (c == -2) {
                os << kTrackColour << g << kReset;
            } else if (c >= 0) {
                os << kPanelColours[static_cast<std::size_t>(c) % kPanelColours.size()]
                   << g << kReset;
            } else if (g == "x") {
                os << kBlindColour << g << kReset;
            } else if (g == "|" || g == "-" || g == "+") {
                os << kWallColour << g << kReset;
            } else {
                os << kDim << g << kReset;
            }
        }
        os << "\n";
    }
    return os.str();
}

std::string MazeRenderer::legend(const std::vector<CameraPanel::Config>& panels,
                                 const std::vector<Entity>& entities) const {
    std::ostringstream os;
    os << "  legend  ";
    if (colour_enabled()) {
        os << kTruthColour << "A" << kReset << "=truth  " << kTrackColour << "0"
           << kReset << "=TRACE  " << kBlindColour << "x" << kReset << "=no coverage\n";
    } else {
        os << "A=truth  0=TRACE track  x=no coverage  .=covered\n";
    }

    os << "  cameras ";
    int shown = 0;
    for (std::size_t i = 0; i < panels.size(); ++i) {
        if (shown++ >= 12) { os << "..."; break; }
        const char* col = colour_enabled()
                              ? kPanelColours[i % kPanelColours.size()]
                              : "";
        const char* rst = colour_enabled() ? kReset : "";
        os << col << panels[i].id << rst << (panels[i].enabled ? " " : "(off) ");
    }
    os << "\n";

    os << "  entities ";
    for (std::size_t i = 0; i < entities.size() && i < 12; ++i) {
        os << static_cast<char>('A' + (i % 26)) << "=" << entities[i].id;
        if (!entities[i].role.empty()) os << "[" << entities[i].role << "]";
        os << "  ";
    }
    os << "\n";
    return os.str();
}

}  // namespace trace::sim
