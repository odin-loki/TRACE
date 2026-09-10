#include "console_window.hpp"

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

using namespace trace;

namespace {

constexpr int kMazeW = 21;
constexpr int kMazeH = 13;
constexpr int kPanelCols = 4;
constexpr int kPanelRows = 3;
constexpr int kTravellers = 4;
constexpr double kCellSize = 8.0;

QString severity_text(Severity s) { return QString::fromStdString(std::string(to_string(s))); }

}  // namespace

ConsoleWindow::ConsoleWindow(QWidget* parent) : QMainWindow(parent) {
    build_ui();
    build_scenario();
}

ConsoleWindow::~ConsoleWindow() = default;

void ConsoleWindow::build_ui() {
    setWindowTitle("TRACE - operator console");
    resize(1400, 860);

    auto* central = new QWidget(this);
    auto* outer = new QVBoxLayout(central);

    // ---- Controls ---------------------------------------------------------
    auto* controls = new QHBoxLayout();
    run_button_ = new QPushButton("Run", this);
    auto* step_button = new QPushButton("Step", this);
    auto* reset_button = new QPushButton("Reset", this);

    profile_box_ = new QComboBox(this);
    for (const auto& name : profile_names()) {
        profile_box_->addItem(QString::fromStdString(name));
    }
    profile_box_->setCurrentText("CityCameraSurveillance");

    auto* speed = new QSlider(Qt::Horizontal, this);
    speed->setRange(16, 500);
    speed->setValue(120);
    speed->setMaximumWidth(180);

    controls->addWidget(run_button_);
    controls->addWidget(step_button);
    controls->addWidget(reset_button);
    controls->addSpacing(16);
    controls->addWidget(new QLabel("Profile:", this));
    controls->addWidget(profile_box_);
    controls->addSpacing(16);
    controls->addWidget(new QLabel("Frame delay:", this));
    controls->addWidget(speed);

    // Display toggles: an operator turns truth off in the field, but during
    // development seeing truth beside the estimate is the whole value.
    auto* truth_btn = new QPushButton("Truth", this);
    auto* unc_btn = new QPushButton("Uncertainty", this);
    auto* fc_btn = new QPushButton("Forecast", this);
    auto* trail_btn = new QPushButton("Trails", this);
    for (auto* b : {truth_btn, unc_btn, fc_btn, trail_btn}) {
        b->setCheckable(true);
        b->setChecked(true);
        controls->addWidget(b);
    }
    controls->addStretch();
    outer->addLayout(controls);

    // ---- Map + side panels ------------------------------------------------
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    view_ = new TrackView(this);
    splitter->addWidget(view_);

    auto* side = new QWidget(this);
    auto* side_layout = new QVBoxLayout(side);

    track_table_ = new QTableWidget(0, 6, this);
    track_table_->setHorizontalHeaderLabels(
        {"Track", "Priority", "Threat", "Exist", "Speed m/s", "Regime"});
    track_table_->horizontalHeader()->setStretchLastSection(true);
    track_table_->verticalHeader()->setVisible(false);
    track_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    auto* track_group = new QGroupBox("Tracks (ranked by threat)", this);
    auto* tg_layout = new QVBoxLayout(track_group);
    tg_layout->addWidget(track_table_);
    side_layout->addWidget(track_group, 3);

    event_log_ = new QTextEdit(this);
    event_log_->setReadOnly(true);
    auto* event_group = new QGroupBox("Events and convergence warnings", this);
    auto* eg_layout = new QVBoxLayout(event_group);
    eg_layout->addWidget(event_log_);
    side_layout->addWidget(event_group, 2);

    splitter->addWidget(side);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    outer->addWidget(splitter);

    setCentralWidget(central);
    status_ = new QLabel(this);
    statusBar()->addWidget(status_);

    timer_ = new QTimer(this);
    timer_->setInterval(120);

    connect(timer_, &QTimer::timeout, this, &ConsoleWindow::step);
    connect(run_button_, &QPushButton::clicked, this, &ConsoleWindow::toggle_run);
    connect(step_button, &QPushButton::clicked, this, &ConsoleWindow::step);
    connect(reset_button, &QPushButton::clicked, this, &ConsoleWindow::reset);
    connect(speed, &QSlider::valueChanged, this, &ConsoleWindow::set_speed);
    connect(profile_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &ConsoleWindow::profile_changed);
    connect(truth_btn, &QPushButton::toggled, view_, &TrackView::set_show_truth);
    connect(unc_btn, &QPushButton::toggled, view_, &TrackView::set_show_uncertainty);
    connect(fc_btn, &QPushButton::toggled, view_, &TrackView::set_show_forecast);
    connect(trail_btn, &QPushButton::toggled, view_, &TrackView::set_show_trails);
}

void ConsoleWindow::build_scenario() {
    rng_.reseed(20260910);
    maze_ = std::make_unique<sim::Maze>(kMazeW, kMazeH, kCellSize);
    maze_->generate(rng_);
    maze_->add_loops(rng_, 0.10);

    panels_ = sim::CameraGrid::partition(*maze_, kPanelCols, kPanelRows, 0.82,
                                         kCellSize * 0.15);
    for (auto& p : panels_) p.swap_probability = 0.08;
    panels_[static_cast<std::size_t>(rng_.uniform_int(0, static_cast<int>(panels_.size()) - 1))]
        .enabled = false;

    scenario_ = std::make_unique<sim::Scenario>(20260910);
    scenario_->n_scans = 1 << 30;  // driven by the timer, not a scan budget
    scenario_->match_radius_m = kCellSize * 1.5;

    DomainProfile profile = profile_by_name(profile_box_->currentText().toStdString());
    profile.scan_dt_s = 1.0;
    profile.pos_noise_m = kCellSize * 0.15;
    profile.meas_noise_var = profile.pos_noise_m * profile.pos_noise_m * 4.0;
    profile.rv_threshold_m = kCellSize * 1.2;
    profile.coloc_dist_m = kCellSize * 2.0;
    profile.brush_pass_m = kCellSize * 0.8;

    scenario_->engine_config.profile = profile;
    scenario_->engine_config.area = maze_->bounds();
    scenario_->engine_config.seed = 20260910;

    for (const auto& cfg : panels_) {
        scenario_->sensors.push_back(std::make_unique<sim::CameraPanel>(cfg));
    }

    for (int i = 0; i < kTravellers; ++i) {
        sim::Entity e;
        e.id = "traveller_" + std::to_string(i);
        const sim::Cell start = maze_->random_cell(rng_);
        const sim::Cell goal = maze_->random_cell(rng_);
        e.position = maze_->centre_of(start);
        e.waypoints = maze_->waypoints(maze_->path(start, goal));
        e.velocity = Vec2{1.3 + 0.2 * i, 0.0};
        scenario_->world.add(std::move(e));
    }

    sim::Maze* maze_ptr = maze_.get();
    Rng* rng_ptr = &rng_;
    scenario_->on_scan = [maze_ptr, rng_ptr](sim::Scenario& s, int) {
        for (auto& e : s.world.entities()) {
            if (e.waypoint_index < e.waypoints.size() || e.dwell_remaining_s > 0.0) {
                continue;
            }
            const sim::Cell from = maze_ptr->cell_at(e.position);
            const sim::Cell to = maze_ptr->random_cell(*rng_ptr);
            const auto cells = maze_ptr->path(from, to);
            if (cells.size() < 2) continue;
            e.waypoints = maze_ptr->waypoints(cells);
            e.waypoint_index = 1;
            e.dwell_remaining_s = rng_ptr->uniform(0.0, 6.0);
        }
    };

    engine_ = std::make_unique<Engine>(scenario_->engine_config);
    metrics_ = sim::Metrics{};
    scan_ = 0;

    view_->set_area(maze_->bounds());
    view_->set_maze(maze_.get());
    view_->set_panels(&panels_);
    event_log_->clear();
}

void ConsoleWindow::step() {
    if (scenario_->on_scan) scenario_->on_scan(*scenario_, scan_);
    scenario_->world.step(scenario_->engine_config.profile.scan_dt_s);

    const sim::WorldSnapshot truth = scenario_->world.snapshot();
    const auto obs = sim::collect(scenario_->sensors, truth, scenario_->rng);
    const ScanReport report = engine_->ingest(obs, truth.timestamp);

    sim::score_scan(metrics_, truth.entities, report.targets,
                    scenario_->match_radius_m);
    metrics_.latencies_ms.push_back(report.latency_ms);

    view_->update_scan(report, truth.entities);
    refresh_tables(report);
    ++scan_;
}

void ConsoleWindow::refresh_tables(const ScanReport& report) {
    track_table_->setRowCount(static_cast<int>(report.targets.size()));
    for (int i = 0; i < static_cast<int>(report.targets.size()); ++i) {
        const auto& t = report.targets[static_cast<std::size_t>(i)];
        const auto set = [&](int col, const QString& text) {
            track_table_->setItem(i, col, new QTableWidgetItem(text));
        };
        set(0, QString::fromStdString(t.track_id));
        set(1, QString::fromStdString(std::string(to_string(t.threat.priority))));
        set(2, QString("%1 ± %2").arg(t.threat.mean, 0, 'f', 3).arg(t.threat.stddev, 0, 'f', 3));
        set(3, QString::number(t.existence, 'f', 3));
        set(4, QString::number(t.speed_mps, 'f', 2));
        set(5, QString::fromStdString(t.dominant_model));
    }

    for (const auto& w : report.rendezvous) {
        event_log_->append(
            QString("[%1] CONVERGENCE %2<->%3 in %4 min  (%5, conf %6)")
                .arg(report.scan)
                .arg(QString::fromStdString(w.track_a))
                .arg(QString::fromStdString(w.track_b))
                .arg(w.eta_min(), 0, 'f', 1)
                .arg(QString::fromStdString(w.method))
                .arg(w.confidence, 0, 'f', 2));
    }
    for (const auto& e : report.events) {
        QString tracks;
        for (const auto& t : e.tracks) tracks += QString::fromStdString(t) + " ";
        event_log_->append(QString("[%1] %2 %3 [%4]")
                               .arg(report.scan)
                               .arg(QString::fromStdString(e.type))
                               .arg(tracks.trimmed())
                               .arg(severity_text(e.severity)));
    }

    status_->setText(
        QString("scan %1   tracks %2   dormant %3   %4 ms   |   detection %5%   "
                "pos err %6 m   id switches %7   ghosts %8")
            .arg(report.scan)
            .arg(report.n_tracks)
            .arg(report.n_dormant)
            .arg(report.latency_ms, 0, 'f', 2)
            .arg(100.0 * metrics_.detection_rate(), 0, 'f', 1)
            .arg(metrics_.mean_position_error(), 0, 'f', 2)
            .arg(metrics_.id_switches)
            .arg(metrics_.ghost_tracks));
}

void ConsoleWindow::toggle_run() {
    running_ = !running_;
    run_button_->setText(running_ ? "Pause" : "Run");
    if (running_) {
        timer_->start();
    } else {
        timer_->stop();
    }
}

void ConsoleWindow::reset() {
    timer_->stop();
    running_ = false;
    run_button_->setText("Run");
    build_scenario();
}

void ConsoleWindow::set_speed(int ms) { timer_->setInterval(ms); }

void ConsoleWindow::profile_changed(int) { reset(); }
