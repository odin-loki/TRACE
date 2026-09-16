// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — operator console main window.
#pragma once

#include <QMainWindow>
#include <memory>
#include <vector>

#include "trace/core/engine.hpp"
#include "trace/sim/maze.hpp"
#include "trace/sim/scenario.hpp"
#include "track_view.hpp"

class QTableWidget;
class QTextEdit;
class QLabel;
class QTimer;
class QPushButton;
class QSlider;
class QComboBox;

/// Live console: a map, a ranked track table, and an event log, driven by the
/// maze simulation so the whole pipeline can be watched end to end.
class ConsoleWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit ConsoleWindow(QWidget* parent = nullptr);
    ~ConsoleWindow() override;

private slots:
    void step();
    void toggle_run();
    void reset();
    void set_speed(int ms);
    void profile_changed(int index);

private:
    void build_ui();
    void build_scenario();
    void refresh_tables(const trace::ScanReport& report);

    std::unique_ptr<trace::sim::Maze> maze_;
    std::vector<trace::sim::CameraPanel::Config> panels_;
    std::unique_ptr<trace::sim::Scenario> scenario_;
    std::unique_ptr<trace::Engine> engine_;
    trace::sim::Metrics metrics_;
    trace::Rng rng_{20260910};

    TrackView* view_{nullptr};
    QTableWidget* track_table_{nullptr};
    QTextEdit* event_log_{nullptr};
    QLabel* status_{nullptr};
    QTimer* timer_{nullptr};
    QPushButton* run_button_{nullptr};
    QComboBox* profile_box_{nullptr};

    int scan_{0};
    bool running_{false};
};
