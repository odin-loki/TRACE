#include "trace/sim/sensor.hpp"

#include <algorithm>
#include <cmath>

namespace trace::sim {
namespace {

std::string make_id(const std::string& sensor_id, long n) {
    return sensor_id + "-" + std::to_string(n);
}

}  // namespace

namespace {

/// A stable per-entity descriptor, blended with fresh noise.
///
/// The signal part is a deterministic function of the entity's id, so the same
/// entity looks the same to every camera on every scan - which is exactly what
/// a re-identification embedding is for. `quality` is how much of the result
/// is that signal: at 1.0 it is an oracle, at 0.0 pure noise, and the range
/// between is where a real descriptor lives.
Descriptor entity_descriptor(const std::string& entity_id, Real quality, Rng& rng) {
    Descriptor d;
    std::uint64_t h = 1469598103934665603ULL;          // FNV-1a over the id
    for (const char c : entity_id) {
        h = (h ^ static_cast<unsigned char>(c)) * 1099511628211ULL;
    }
    Rng stable(h);
    const Real q = std::clamp(quality, 0.0, 1.0);
    for (std::size_t i = 0; i < kDescriptorDim; ++i) {
        d.v[i] = static_cast<float>(q * stable.normal() +
                                    (1.0 - q) * rng.normal());
    }
    d.present = true;
    d.normalise();
    return d;
}

}  // namespace

std::vector<Observation> CameraPanel::observe(const WorldSnapshot& truth, Rng& rng,
                                              DetectionLedger* ledger) {
    std::vector<Observation> out;
    if (!cfg_.enabled) return out;

    // Which entities are in this camera's footprint right now.
    std::vector<const Entity*> in_view;
    for (const auto& e : truth.entities) {
        if (e.active && cfg_.footprint.contains(e.position)) in_view.push_back(&e);
    }

    // Identity confusion: with two or more entities in one field of view, a
    // real re-ID stage sometimes attaches the wrong label. The engine gets the
    // positions but never the labels, so what this actually models is two
    // detections arriving in swapped order with each other's noise - enough to
    // pull a naive nearest-neighbour associator onto the wrong track.
    std::vector<Vec2> report_positions;
    report_positions.reserve(in_view.size());
    for (const auto* e : in_view) report_positions.push_back(e->position);

    if (in_view.size() >= 2 && cfg_.swap_probability > 0.0) {
        for (std::size_t i = 0; i + 1 < in_view.size(); ++i) {
            if (rng.bernoulli(cfg_.swap_probability)) {
                std::swap(report_positions[i], report_positions[i + 1]);
            }
        }
    }

    // A bias is not noise: it does not average out, and it moves every
    // detection from this sensor the same way.
    const Vec2 bias = cfg_.bias + cfg_.bias_drift_per_s * truth.timestamp;

    for (std::size_t i = 0; i < in_view.size(); ++i) {
        if (!rng.bernoulli(cfg_.p_detect)) continue;
        if (ledger != nullptr) ledger->insert(in_view[i]->id);
        const Vec2 noisy{
            report_positions[i].x + bias.x + rng.normal(0.0, cfg_.pos_noise_m),
            report_positions[i].y + bias.y + rng.normal(0.0, cfg_.pos_noise_m)};
        const Real conf = std::clamp(
            rng.normal(cfg_.confidence_mean, cfg_.confidence_sigma), 0.1, 1.0);
        Observation obs(make_id(cfg_.id, counter_++), truth.timestamp, noisy,
                        cfg_.modality, conf, cfg_.id);
        if (cfg_.appearance_quality > 0.0) {
            obs.descriptor = entity_descriptor(in_view[i]->id,
                                               cfg_.appearance_quality, rng);
        }
        out.push_back(std::move(obs));
    }

    // False alarms land anywhere in the footprint - reflections, foliage, a
    // discarded bag that the detector calls a person.
    const int n_false = rng.poisson(cfg_.false_alarm_rate);
    for (int i = 0; i < n_false; ++i) {
        const Vec2 p{rng.uniform(cfg_.footprint.xmin, cfg_.footprint.xmax),
                     rng.uniform(cfg_.footprint.ymin, cfg_.footprint.ymax)};
        out.emplace_back(make_id(cfg_.id, counter_++), truth.timestamp, p,
                         cfg_.modality, rng.uniform(0.15, 0.45), cfg_.id);
    }
    return out;
}

std::vector<Observation> GateReader::observe(const WorldSnapshot& truth, Rng& rng,
                                             DetectionLedger* ledger) {
    std::vector<Observation> out;
    if (!cfg_.enabled) return out;

    for (const auto& e : truth.entities) {
        if (!e.active) continue;
        if (distance(e.position, cfg_.position) > cfg_.radius_m) continue;
        if (!rng.bernoulli(cfg_.p_detect)) continue;
        if (ledger != nullptr) ledger->insert(e.id);

        // A gate reports its own location, not the entity's: that is precisely
        // what a badge reader or a turnstile knows.
        const Vec2 noisy{cfg_.position.x + rng.normal(0.0, cfg_.pos_noise_m),
                         cfg_.position.y + rng.normal(0.0, cfg_.pos_noise_m)};
        out.emplace_back(make_id(cfg_.id, counter_++), truth.timestamp, noisy,
                         cfg_.modality, cfg_.confidence, cfg_.id);
    }

    const int n_false = rng.poisson(cfg_.false_alarm_rate);
    for (int i = 0; i < n_false; ++i) {
        out.emplace_back(make_id(cfg_.id, counter_++), truth.timestamp,
                         cfg_.position, cfg_.modality, rng.uniform(0.1, 0.4),
                         cfg_.id);
    }
    return out;
}

std::vector<Observation> WideAreaReporter::observe(const WorldSnapshot& truth,
                                                   Rng& rng,
                                                   DetectionLedger* ledger) {
    std::vector<Observation> out;
    if (!cfg_.enabled) return out;

    for (const auto& e : truth.entities) {
        if (!e.active) continue;
        if (silenced.contains(e.id)) continue;  // gone dark
        if (!cfg_.footprint.contains(e.position)) continue;
        if (!rng.bernoulli(cfg_.p_detect)) continue;
        if (ledger != nullptr) ledger->insert(e.id);

        const Vec2 noisy{e.position.x + rng.normal(0.0, cfg_.pos_noise_m),
                         e.position.y + rng.normal(0.0, cfg_.pos_noise_m)};
        out.emplace_back(make_id(cfg_.id, counter_++), truth.timestamp, noisy,
                         cfg_.modality, cfg_.confidence, cfg_.id);
    }

    const int n_false = rng.poisson(cfg_.false_alarm_rate);
    for (int i = 0; i < n_false; ++i) {
        const Vec2 p{rng.uniform(cfg_.footprint.xmin, cfg_.footprint.xmax),
                     rng.uniform(cfg_.footprint.ymin, cfg_.footprint.ymax)};
        out.emplace_back(make_id(cfg_.id, counter_++), truth.timestamp, p,
                         cfg_.modality, rng.uniform(0.1, 0.35), cfg_.id);
    }
    return out;
}

std::vector<Observation> SpoofInjector::observe(const WorldSnapshot& truth,
                                                Rng& rng,
                                                DetectionLedger* ledger) {
    std::vector<Observation> out;
    if (!cfg_.enabled) return out;
    if (truth.timestamp < cfg_.start_time_s || truth.timestamp > cfg_.end_time_s) {
        return out;
    }
    if (!rng.bernoulli(cfg_.p_report)) return out;

    // Deliberately NOT recorded in the ledger: no real entity is behind this,
    // so counting it would corrupt the sensor-ceiling measurement that every
    // scenario reports.
    const Vec2 p = phantom_position(truth.timestamp);
    out.emplace_back(cfg_.id + "-" + std::to_string(counter_++), truth.timestamp,
                     Vec2{p.x + rng.normal(0.0, cfg_.pos_noise_m),
                          p.y + rng.normal(0.0, cfg_.pos_noise_m)},
                     cfg_.modality, cfg_.confidence, cfg_.id);
    return out;
}

std::vector<Observation> collect(const std::vector<SensorPtr>& sensors,
                                 const WorldSnapshot& truth, Rng& rng,
                                 DetectionLedger* ledger) {
    std::vector<Observation> out;
    for (const auto& s : sensors) {
        for (auto& o : s->observe(truth, rng, ledger)) out.push_back(std::move(o));
    }
    // Shuffle so the engine cannot infer identity from arrival order - a real
    // fusion layer receives detections in whatever order they land.
    for (std::size_t i = out.size(); i > 1; --i) {
        const auto j = static_cast<std::size_t>(rng.uniform_int(0, static_cast<int>(i) - 1));
        std::swap(out[i - 1], out[j]);
    }
    return out;
}

}  // namespace trace::sim
