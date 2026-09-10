#include "trace/core/engine.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <sstream>

#include "trace/detectors/detectors.hpp"

namespace trace {
namespace {

constexpr std::size_t kObsCacheDepth = 20;
constexpr Real kObsCacheRadius = 120.0;

std::string fmt(const char* spec, auto... args) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), spec, args...);
    return std::string(buf);
}

}  // namespace

Engine::Engine(EngineConfig config)
    : config_(std::move(config)),
      pmbm_(config_.profile, config_.area, config_.seed),
      network_(config_.profile.coloc_dist_m),
      detectors_(default_detectors()),
      rng_(config_.seed ^ 0x1234ABCDULL) {}

Engine::~Engine() = default;
Engine::Engine(Engine&&) noexcept = default;
Engine& Engine::operator=(Engine&&) noexcept = default;

void Engine::register_detector(DetectorPtr detector) {
    if (detector == nullptr) return;
    const std::string name = detector->name();
    std::erase_if(detectors_, [&](const DetectorPtr& d) { return d->name() == name; });
    detectors_.push_back(std::move(detector));
}

bool Engine::unregister_detector(std::string_view name) {
    const auto before = detectors_.size();
    std::erase_if(detectors_, [&](const DetectorPtr& d) { return d->name() == name; });
    return detectors_.size() != before;
}

std::vector<std::string> Engine::detector_names() const {
    std::vector<std::string> out;
    out.reserve(detectors_.size());
    for (const auto& d : detectors_) out.push_back(d->name());
    return out;
}

ScanReport Engine::ingest(const std::vector<Observation>& observations,
                          Real timestamp) {
    const auto t_start = std::chrono::steady_clock::now();
    ++scan_count_;

    // ---- [T] + [A] tracking and association -------------------------------
    pmbm_.predict();
    pmbm_.update(observations, timestamp);
    const std::vector<TrackPtr> confirmed = pmbm_.confirmed();

    // Keep the recent evidence behind each track for the credibility fusion.
    for (const auto& t : confirmed) {
        auto& cache = obs_cache_[t->id()];
        for (const auto& o : observations) {
            if (!o.has_position()) continue;
            if (distance(*o.position, t->position()) < kObsCacheRadius) {
                cache.push_back(o);
            }
        }
        if (cache.size() > kObsCacheDepth) {
            cache.erase(cache.begin(),
                        cache.end() - static_cast<long>(kObsCacheDepth));
        }
    }

    // ---- Network structure, needed before roles can be inferred -----------
    const std::vector<Cluster> clusters = network_.analyse(confirmed, timestamp);

    // ---- Scoring ----------------------------------------------------------
    ScanReport report;
    report.targets.reserve(confirmed.size());

    for (const auto& t : confirmed) {
        TargetReport tr;
        tr.track_id = t->id();
        tr.parent_id = t->parent_id();
        tr.born_at = t->born_at();
        tr.last_seen = t->last_seen();
        tr.position = t->position();
        tr.velocity_mps = t->velocity_mps(config_.profile.scan_dt_s);
        tr.speed_mps = tr.velocity_mps.norm();
        tr.position_uncertainty_m = t->position_uncertainty();
        tr.existence = t->existence();
        tr.possibility = t->possibility();
        tr.possibility_mismatch = t->possibility_mismatch();
        tr.age_scans = t->age();
        tr.hits = t->hits();
        tr.misses = t->misses();
        tr.measurement_rate = t->measurement_rate();
        tr.observation_quality = t->mean_observation_quality();
        tr.dominant_model = t->dominant_model();
        tr.credibility = fuse_credibility(obs_cache_[t->id()], config_.profile);
        tr.threat = score_track(*t, timestamp, config_.high_value_locations,
                                config_.profile.hvl_radius_m, config_.profile, rng_);

        for (auto& a : escalator_.update(t->id(), tr.threat.breakdown.pol_anomaly,
                                         tr.threat.priority)) {
            report.alerts.push_back(std::move(a));
        }

        // Forecast only what warrants the compute.
        if (tr.threat.priority == Priority::IMMEDIATE ||
            tr.threat.priority == Priority::HIGH) {
            const Vec2 v = t->velocity();
            Vec2 p = t->position();
            const Real unc0 = t->position_uncertainty();
            for (int k = 1; k <= config_.forecast_horizon; ++k) {
                p += v;
                tr.forecast.push_back(ForecastStep{
                    timestamp + k * config_.profile.scan_dt_s, p,
                    // Uncertainty grows as sqrt(time) under a diffusion model.
                    unc0 * std::sqrt(static_cast<Real>(k) + 1.0)});
            }
        }
        report.targets.push_back(std::move(tr));
    }

    std::sort(report.targets.begin(), report.targets.end(),
              [](const TargetReport& a, const TargetReport& b) {
                  return a.threat.mean > b.threat.mean;
              });

    // ---- [C] + [E] convergence and events ---------------------------------
    DetectorContext ctx;
    ctx.timestamp = timestamp;
    ctx.scan_index = scan_count_;
    ctx.profile = &config_.profile;
    ctx.high_value_locations = &config_.high_value_locations;
    ctx.betweenness = &network_.betweenness();
    ctx.clusters = &clusters;
    ctx.rng = &rng_;

    for (auto& d : detectors_) {
        // A misbehaving detector must not take the engine down with it: a
        // deployment can register anything, and losing the tracking core
        // because one plugin threw is not an acceptable failure mode.
        try {
            for (auto& e : d->detect(confirmed, ctx)) report.events.push_back(std::move(e));
            for (auto& r : d->rendezvous(confirmed, ctx)) report.rendezvous.push_back(std::move(r));
            for (auto& r : d->roles(confirmed, ctx)) report.network_roles.push_back(std::move(r));
        } catch (const std::exception& ex) {
            auto e = make_event("DETECTOR_ERROR", d->name(), {}, Severity::LOW,
                                timestamp);
            e.note = ex.what();
            report.events.push_back(std::move(e));
        }
    }

    // ---- Cross-cutting operational flags ----------------------------------
    for (const auto& t : confirmed) {
        if (t->possibility_mismatch() > 0.4) {
            report.operational.possibility_mismatch_tracks.push_back(t->id());
        }
        const Real speed = t->speed_mps(config_.profile.scan_dt_s);
        if (speed > config_.profile.courier_speed_thresh * 3.0) {
            report.operational.high_speed_tracks.push_back(t->id());
        }
        if (speed < config_.profile.courier_speed_thresh * 0.1) {
            report.operational.dwelling_tracks.push_back(t->id());
        }
        const Vec2 p = t->position();
        const Real margin = std::min(config_.area.width(), config_.area.height()) * 0.05;
        if (p.x - config_.area.xmin < margin || config_.area.xmax - p.x < margin ||
            p.y - config_.area.ymin < margin || config_.area.ymax - p.y < margin) {
            report.operational.boundary_tracks.push_back(t->id());
        }
    }

    report.sensor_schedule = schedule_collection(confirmed, config_.profile);
    report.clusters = clusters;
    report.scan = scan_count_;
    report.timestamp = timestamp;
    report.domain = config_.profile.name;
    report.n_observations = static_cast<int>(observations.size());
    report.n_tracks = static_cast<int>(confirmed.size());
    report.n_components = static_cast<int>(pmbm_.all_tracks().size());
    report.n_dormant = static_cast<int>(pmbm_.dormant_count());
    report.clutter_rate = pmbm_.clutter_rate();

    const auto t_end = std::chrono::steady_clock::now();
    report.latency_ms =
        std::chrono::duration<Real, std::milli>(t_end - t_start).count();
    total_latency_ms_ += report.latency_ms;

    history_.push_back(report);
    return report;
}

std::string Engine::summary(const ScanReport& r) const {
    std::ostringstream os;

    os << fmt("== TRACE [%s] scan %04d  t=%.0fs  %.1f ms  clutter=%.1f  dormant=%d\n",
              r.domain.c_str(), r.scan, r.timestamp, r.latency_ms, r.clutter_rate,
              r.n_dormant);
    os << fmt("   obs=%d  confirmed=%d  components=%d\n", r.n_observations,
              r.n_tracks, r.n_components);

    if (!r.alerts.empty()) {
        os << "   ALERTS ";
        for (std::size_t i = 0; i < std::min<std::size_t>(r.alerts.size(), 5); ++i) {
            os << r.alerts[i].reason << "(" << r.alerts[i].track << ") ";
        }
        os << "\n";
    }

    if (!r.events.empty()) {
        std::map<std::string, int> by_type;
        for (const auto& e : r.events) ++by_type[e.type];
        os << "   EVENTS ";
        for (const auto& [type, n] : by_type) os << type << " x" << n << "  ";
        os << "\n";
    }

    if (!r.rendezvous.empty()) {
        os << "   -- convergence warnings --\n";
        for (std::size_t i = 0; i < std::min<std::size_t>(r.rendezvous.size(), 6); ++i) {
            const auto& w = r.rendezvous[i];
            os << fmt("   %s<->%s  ETA %6.1f min  sep %7.0f m  %-22s conf %.2f  %s\n",
                      w.track_a.c_str(), w.track_b.c_str(), w.eta_min(),
                      w.current_sep_m, w.method.c_str(), w.confidence,
                      std::string(to_string(w.priority)).c_str());
        }
    }

    if (!r.network_roles.empty()) {
        os << "   -- network roles --\n";
        for (std::size_t i = 0; i < std::min<std::size_t>(r.network_roles.size(), 5); ++i) {
            const auto& n = r.network_roles[i];
            os << fmt("   %-7s %-10s contacts=%-2d  spd=%5.2f m/s  bc=%.3f  conf=%.2f\n",
                      n.track.c_str(), n.role.c_str(), n.n_contacts,
                      n.avg_speed_mps, n.betweenness, n.confidence);
        }
    }

    if (!r.targets.empty()) {
        os << fmt("   %-7s %-10s %-16s %-6s %-22s %-11s %s\n", "ID", "PRIORITY",
                  "THREAT", "EXIST", "POSITION", "MODEL", "MR");
        for (const auto& t : r.targets) {
            os << fmt("   %-7s %-10s %.3f +/- %.3f  %.3f  [%8.0f,%8.0f]  %-11s %.2f\n",
                      t.track_id.c_str(),
                      std::string(to_string(t.threat.priority)).c_str(),
                      t.threat.mean, t.threat.stddev, t.existence, t.position.x,
                      t.position.y, t.dominant_model.c_str(), t.measurement_rate);
        }
    }

    if (!r.clusters.empty()) {
        os << "   -- clusters --\n";
        for (std::size_t i = 0; i < std::min<std::size_t>(r.clusters.size(), 3); ++i) {
            const auto& c = r.clusters[i];
            os << "   C" << c.cluster_id << ": ";
            for (const auto& m : c.member_ids) os << m << " ";
            os << " hub=" << c.hub_track << (c.recurring ? "  [recurring]" : "") << "\n";
        }
    }

    return os.str();
}

std::string Engine::performance_report() const {
    if (history_.empty()) return "TRACE: no scans ingested.\n";

    int peak = 0;
    int events = 0;
    int rvs = 0;
    int roles = 0;
    std::set<std::string> unique_ids;
    std::vector<Real> latencies;
    latencies.reserve(history_.size());

    for (const auto& r : history_) {
        peak = std::max(peak, r.n_tracks);
        events += static_cast<int>(r.events.size());
        rvs += static_cast<int>(r.rendezvous.size());
        roles += static_cast<int>(r.network_roles.size());
        latencies.push_back(r.latency_ms);
        for (const auto& t : r.targets) unique_ids.insert(t.track_id);
    }

    std::sort(latencies.begin(), latencies.end());
    const Real median = latencies[latencies.size() / 2];
    const Real p95 = latencies[static_cast<std::size_t>(
        std::min(latencies.size() - 1,
                 static_cast<std::size_t>(latencies.size() * 0.95)))];
    const Real mean = total_latency_ms_ / static_cast<Real>(history_.size());

    std::ostringstream os;
    os << fmt("TRACE session  domain=%s  scans=%d\n", config_.profile.name.c_str(),
              scan_count_);
    os << fmt("  tracks   peak=%d  unique=%zu  dormant-now=%d\n", peak,
              unique_ids.size(), history_.back().n_dormant);
    os << fmt("  findings events=%d  convergence=%d  roles=%d\n", events, rvs, roles);
    os << fmt("  latency  median=%.2f ms  mean=%.2f ms  p95=%.2f ms  max=%.2f ms\n",
              median, mean, p95, latencies.back());
    return os.str();
}

}  // namespace trace
