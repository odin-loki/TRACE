// TRACE — simulated sensors.
//
// Every sensor here models the failure modes that make real tracking hard:
// limited footprint, missed detections, position error, false alarms, and
// identity confusion when two entities are in the same field of view. A
// simulation without those is only testing arithmetic.
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "trace/core/observation.hpp"
#include "trace/core/rng.hpp"
#include "trace/sim/world.hpp"

namespace trace::sim {

class Sensor {
public:
    virtual ~Sensor() = default;
    [[nodiscard]] virtual const std::string& id() const = 0;

    /// Produce this scan's detections from the true world state.
    virtual std::vector<Observation> observe(const WorldSnapshot& truth,
                                             Rng& rng) = 0;

    /// True if this sensor could see the point at all — used for coverage maps
    /// and for rendering, never by the engine.
    [[nodiscard]] virtual bool covers(Vec2 point) const = 0;
};

using SensorPtr = std::unique_ptr<Sensor>;

/// A fixed camera watching a rectangular footprint.
///
/// This is the workhorse of the maze simulation: a "view panel" is exactly one
/// of these. Detection probability, position noise and false-alarm rate are all
/// per-camera, so a scenario can mix a good overhead camera with a poor
/// street-level one and see how the tracker copes.
class CameraPanel final : public Sensor {
public:
    struct Config {
        std::string id{"CAM"};
        Area footprint{};
        Real p_detect{0.85};
        Real pos_noise_m{2.0};
        Real false_alarm_rate{0.15};  ///< Poisson mean per scan
        Modality modality{Modality::GEOINT};
        Real confidence_mean{0.88};
        Real confidence_sigma{0.06};
        /// Chance that two entities in view are reported at each other's
        /// position - the identity confusion a real re-ID stage suffers.
        Real swap_probability{0.0};
        bool enabled{true};
    };

    explicit CameraPanel(Config cfg) : cfg_(std::move(cfg)) {}

    [[nodiscard]] const std::string& id() const override { return cfg_.id; }
    [[nodiscard]] bool covers(Vec2 point) const override {
        return cfg_.enabled && cfg_.footprint.contains(point);
    }
    [[nodiscard]] const Config& config() const { return cfg_; }
    [[nodiscard]] Config& config() { return cfg_; }

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng) override;

private:
    Config cfg_;
    long counter_{0};
};

/// A reader that fires only when an entity crosses a line or enters a small
/// radius: RFID/BLE gates, ANPR at a junction, a turnstile. Produces sparse,
/// accurate, topologically-anchored detections rather than continuous tracks.
class GateReader final : public Sensor {
public:
    struct Config {
        std::string id{"GATE"};
        Vec2 position{};
        Real radius_m{5.0};
        Real p_detect{0.95};
        Real pos_noise_m{0.5};
        Modality modality{Modality::COMMS};
        Real confidence{0.92};
        Real false_alarm_rate{0.0};
        bool enabled{true};
    };

    explicit GateReader(Config cfg) : cfg_(std::move(cfg)) {}

    [[nodiscard]] const std::string& id() const override { return cfg_.id; }
    [[nodiscard]] bool covers(Vec2 point) const override {
        return cfg_.enabled && distance(point, cfg_.position) <= cfg_.radius_m;
    }
    [[nodiscard]] const Config& config() const { return cfg_; }

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng) override;

private:
    Config cfg_;
    long counter_{0};
};

/// A wide-area, low-rate, unreliable reporter: satellite AIS, a collar uplink,
/// an occasional human report. Can be told to go dark for a stretch, which is
/// the whole point of the dark-vessel scenario.
class WideAreaReporter final : public Sensor {
public:
    struct Config {
        std::string id{"WIDE"};
        Area footprint{};
        Real p_detect{0.5};
        Real pos_noise_m{150.0};
        Modality modality{Modality::SIGINT};
        Real confidence{0.7};
        Real false_alarm_rate{0.05};
        bool enabled{true};
    };

    explicit WideAreaReporter(Config cfg) : cfg_(std::move(cfg)) {}

    [[nodiscard]] const std::string& id() const override { return cfg_.id; }
    [[nodiscard]] bool covers(Vec2 point) const override {
        return cfg_.enabled && cfg_.footprint.contains(point);
    }
    [[nodiscard]] Config& config() { return cfg_; }

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng) override;

    /// Entities that this reporter will not report, by ground-truth id.
    std::set<std::string> silenced;

private:
    Config cfg_;
    long counter_{0};
};

/// Collect one scan from every sensor.
std::vector<Observation> collect(const std::vector<SensorPtr>& sensors,
                                 const WorldSnapshot& truth, Rng& rng);

}  // namespace trace::sim
