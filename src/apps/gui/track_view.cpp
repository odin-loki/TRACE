#include "track_view.hpp"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

using namespace trace;

namespace {

/// Priority drives colour: an operator should be able to read urgency without
/// consulting a legend.
QColor priority_colour(Priority p) {
    switch (p) {
        case Priority::IMMEDIATE: return QColor(255, 82, 82);
        case Priority::HIGH:      return QColor(255, 167, 38);
        case Priority::MEDIUM:    return QColor(255, 213, 79);
        case Priority::LOW:       return QColor(129, 199, 132);
        default:                  return QColor(144, 164, 174);
    }
}

constexpr int kMaxTrail = 40;

}  // namespace

TrackView::TrackView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(600, 420);
    setAutoFillBackground(true);
}

void TrackView::set_area(Area area) {
    area_ = area;
    update();
}

void TrackView::set_maze(const sim::Maze* maze) {
    maze_ = maze;
    update();
}

void TrackView::set_panels(const std::vector<sim::CameraPanel::Config>* panels) {
    panels_ = panels;
    update();
}

void TrackView::update_scan(const ScanReport& report,
                            const std::vector<sim::Entity>& truth) {
    report_ = report;
    truth_ = truth;

    // Extend the trail for every track still present; drop trails for tracks
    // that have gone, so the display does not fill with history.
    std::vector<std::vector<Vec2>> next_trails;
    std::vector<std::string> next_ids;
    for (const auto& t : report_.targets) {
        const auto it = std::find(trail_ids_.begin(), trail_ids_.end(), t.track_id);
        std::vector<Vec2> trail;
        if (it != trail_ids_.end()) {
            trail = trails_[static_cast<std::size_t>(it - trail_ids_.begin())];
        }
        trail.push_back(t.position);
        if (trail.size() > kMaxTrail) trail.erase(trail.begin());
        next_trails.push_back(std::move(trail));
        next_ids.push_back(t.track_id);
    }
    trails_ = std::move(next_trails);
    trail_ids_ = std::move(next_ids);

    update();
}

double TrackView::scale() const {
    const double w = area_.width() > 0 ? area_.width() : 1.0;
    const double h = area_.height() > 0 ? area_.height() : 1.0;
    return std::min(width() / w, height() / h) * 0.92;
}

QPointF TrackView::to_screen(Vec2 world) const {
    const double s = scale();
    const double ox = (width() - area_.width() * s) * 0.5;
    const double oy = (height() - area_.height() * s) * 0.5;
    // Flip y: world coordinates grow upward, screen coordinates grow downward.
    return QPointF(ox + (world.x - area_.xmin) * s,
                   oy + (area_.ymax - world.y) * s);
}

void TrackView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(18, 20, 24));

    const double s = scale();

    // ---- Camera footprints ------------------------------------------------
    if (panels_ != nullptr) {
        for (const auto& panel : *panels_) {
            const QPointF tl = to_screen(Vec2{panel.footprint.xmin, panel.footprint.ymax});
            const QRectF r(tl, QSizeF(panel.footprint.width() * s,
                                      panel.footprint.height() * s));
            if (panel.enabled) {
                p.fillRect(r, QColor(40, 70, 100, 70));
                p.setPen(QPen(QColor(70, 130, 180, 130), 1));
            } else {
                // A disabled camera is drawn as a blind zone, not as absence:
                // "we cannot see here" is operationally different from "empty".
                p.fillRect(r, QColor(70, 30, 30, 90));
                p.setPen(QPen(QColor(160, 60, 60, 150), 1, Qt::DashLine));
            }
            p.drawRect(r);
            p.setPen(QColor(150, 170, 190, 160));
            p.drawText(r.adjusted(4, 2, -2, -2), Qt::AlignTop | Qt::AlignLeft,
                       QString::fromStdString(panel.id));
        }
    }

    // ---- Maze walls -------------------------------------------------------
    if (maze_ != nullptr) {
        p.setPen(QPen(QColor(120, 130, 140), 1.5));
        const double cs = maze_->cell_size();
        for (int row = 0; row < maze_->height(); ++row) {
            for (int col = 0; col < maze_->width(); ++col) {
                const sim::Cell c{col, row};
                const Vec2 bl{col * cs, row * cs};
                const Vec2 br{(col + 1) * cs, row * cs};
                const Vec2 tl{col * cs, (row + 1) * cs};
                if (maze_->wall_between(c, sim::Cell{col, row - 1})) {
                    p.drawLine(to_screen(bl), to_screen(br));
                }
                if (maze_->wall_between(c, sim::Cell{col - 1, row})) {
                    p.drawLine(to_screen(bl), to_screen(tl));
                }
            }
        }
    }

    // ---- Trails -----------------------------------------------------------
    if (show_trails_) {
        for (const auto& trail : trails_) {
            if (trail.size() < 2) continue;
            QPainterPath path(to_screen(trail.front()));
            for (std::size_t i = 1; i < trail.size(); ++i) {
                path.lineTo(to_screen(trail[i]));
            }
            p.setPen(QPen(QColor(80, 200, 140, 90), 1.4));
            p.drawPath(path);
        }
    }

    // ---- Ground truth -----------------------------------------------------
    if (show_truth_) {
        p.setPen(QPen(QColor(230, 230, 240, 190), 1.2));
        for (const auto& e : truth_) {
            if (!e.active) continue;
            const QPointF c = to_screen(e.position);
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(c, 7, 7);
            p.drawText(c + QPointF(9, -6), QString::fromStdString(e.id));
        }
    }

    // ---- Tracks -----------------------------------------------------------
    for (const auto& t : report_.targets) {
        const QPointF c = to_screen(t.position);
        const QColor col = priority_colour(t.threat.priority);

        if (show_uncertainty_) {
            // One-sigma position uncertainty, drawn to scale. An operator
            // needs to see confidence, not just a dot.
            const double r = std::max(t.position_uncertainty_m * s, 3.0);
            p.setPen(QPen(QColor(col.red(), col.green(), col.blue(), 90), 1));
            p.setBrush(QColor(col.red(), col.green(), col.blue(), 28));
            p.drawEllipse(c, r, r);
        }

        if (show_forecast_ && !t.forecast.empty()) {
            QPainterPath fp(c);
            for (const auto& f : t.forecast) fp.lineTo(to_screen(f.position));
            p.setPen(QPen(col, 1.2, Qt::DotLine));
            p.setBrush(Qt::NoBrush);
            p.drawPath(fp);
        }

        p.setPen(QPen(col, 1.6));
        p.setBrush(col);
        p.drawEllipse(c, 4, 4);

        p.setPen(QColor(220, 225, 235));
        p.drawText(c + QPointF(7, 12),
                   QString("%1  %2")
                       .arg(QString::fromStdString(t.track_id))
                       .arg(t.threat.mean, 0, 'f', 2));
    }

    // ---- Convergence warnings --------------------------------------------
    for (const auto& w : report_.rendezvous) {
        if (!w.location.has_value()) continue;
        const QPointF c = to_screen(*w.location);
        p.setPen(QPen(QColor(255, 90, 90, 200), 1.4, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(c, 10, 10);
        p.drawText(c + QPointF(12, 4),
                   QString("%1<->%2  %3 min")
                       .arg(QString::fromStdString(w.track_a))
                       .arg(QString::fromStdString(w.track_b))
                       .arg(w.eta_min(), 0, 'f', 1));
    }
}
