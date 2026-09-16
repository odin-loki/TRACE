// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — the live map widget.
#pragma once

#include <QWidget>
#include <vector>

#include "trace/core/report.hpp"
#include "trace/sim/maze.hpp"
#include "trace/sim/world.hpp"

/// Draws the area of regard: camera footprints, walls, ground truth and the
/// engine's tracks, with an uncertainty ellipse per track.
///
/// Truth is drawn deliberately, in a distinct colour, because the whole point
/// of the console during development is seeing where the engine is wrong. A
/// deployment build would pass an empty truth vector and see only tracks.
class TrackView : public QWidget {
    Q_OBJECT

public:
    explicit TrackView(QWidget* parent = nullptr);

    void set_area(trace::Area area);
    void set_maze(const trace::sim::Maze* maze);
    void set_panels(const std::vector<trace::sim::CameraPanel::Config>* panels);
    void update_scan(const trace::ScanReport& report,
                     const std::vector<trace::sim::Entity>& truth);

    void set_show_truth(bool on) { show_truth_ = on; update(); }
    void set_show_uncertainty(bool on) { show_uncertainty_ = on; update(); }
    void set_show_forecast(bool on) { show_forecast_ = on; update(); }
    void set_show_trails(bool on) { show_trails_ = on; update(); }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    [[nodiscard]] QPointF to_screen(trace::Vec2 world) const;
    [[nodiscard]] double scale() const;

    trace::Area area_{};
    const trace::sim::Maze* maze_{nullptr};
    const std::vector<trace::sim::CameraPanel::Config>* panels_{nullptr};

    trace::ScanReport report_;
    std::vector<trace::sim::Entity> truth_;
    std::vector<std::vector<trace::Vec2>> trails_;
    std::vector<std::string> trail_ids_;

    bool show_truth_{true};
    bool show_uncertainty_{true};
    bool show_forecast_{true};
    bool show_trails_{true};
};
