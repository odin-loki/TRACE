// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

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

#ifdef SIGINT
#undef SIGINT
#endif

namespace trace::sim {

/// Which ground-truth entities actually produced a detection this scan.
///
/// Used only for scoring, never passed to the engine. Without it a low
/// detection rate cannot be attributed: a tracker that reports nothing when the
/// sensors reported nothing has not failed at anything.
using DetectionLedger = std::set<std::string>;

class Sensor {
public:
    virtual ~Sensor() = default;
    [[nodiscard]] virtual const std::string& id() const = 0;

    /// Produce this scan's detections from the true world state.
    ///
    /// If `ledger` is non-null, record the ground-truth id behind each genuine
    /// detection (false alarms contribute nothing, having no entity behind
    /// them).
    virtual std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng,
                                             DetectionLedger* ledger = nullptr) = 0;

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
        /// Systematic position offset: a miscalibrated or knocked camera.
        /// Unlike noise, a bias does not average out, and every detection from
        /// the sensor is wrong in the same direction.
        Vec2 bias{};
        /// How fast the bias grows, in metres per second. Models a mount
        /// slowly slipping, which is harder to notice than a sudden knock.
        Vec2 bias_drift_per_s{};
        bool enabled{true};

        /// How distinctive this camera's appearance descriptors are, as the
        /// fraction of the descriptor that is signal rather than noise. Zero
        /// attaches no descriptor at all, which is the default and what every
        /// scenario written before this did.
        ///
        /// Appearance is the only evidence that survives a long gap intact -
        /// across one, position has decayed to a guess - so a scenario that
        /// asks whether identities survive an outage cannot answer it without
        /// this. It is a knob rather than a switch because the interesting
        /// question is never "does a perfect descriptor help" but "how good
        /// does one have to be".
        Real appearance_quality{0.0};
    };

    explicit CameraPanel(Config cfg) : cfg_(std::move(cfg)) {}

    [[nodiscard]] const std::string& id() const override { return cfg_.id; }
    [[nodiscard]] bool covers(Vec2 point) const override {
        return cfg_.enabled && cfg_.footprint.contains(point);
    }
    [[nodiscard]] const Config& config() const { return cfg_; }
    [[nodiscard]] Config& config() { return cfg_; }

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng,
                                     DetectionLedger* ledger = nullptr) override;

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

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng,
                                     DetectionLedger* ledger = nullptr) override;

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

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng,
                                     DetectionLedger* ledger = nullptr) override;

    /// Entities that this reporter will not report, by ground-truth id.
    std::set<std::string> silenced;

private:
    Config cfg_;
    long counter_{0};
};

/// A sensor that fabricates detections: a spoofed feed, an injected track, a
/// decoy. It reports a plausible entity that does not exist, at high confidence,
/// which is what makes it dangerous - a low-confidence lie is filtered by the
/// birth gate and never becomes a track.
///
/// This exists to test the possibility/probability mismatch diagnostic, which
/// until now fired constantly in the scenarios without ever being exercised
/// against a case where it *should* fire.
class SpoofInjector final : public Sensor {
public:
    struct Config {
        std::string id{"SPOOF"};
        Vec2 origin{};
        Vec2 velocity{};        ///< the phantom's apparent motion, m/s
        Real start_time_s{0.0};
        Real end_time_s{1e18};
        Real p_report{0.95};    ///< a fabricated feed is conveniently reliable
        Real pos_noise_m{1.0};
        Modality modality{Modality::GEOINT};
        Real confidence{0.95};
        bool enabled{true};
    };

    explicit SpoofInjector(Config cfg) : cfg_(std::move(cfg)) {}

    [[nodiscard]] const std::string& id() const override { return cfg_.id; }

    /// A spoofer covers NOTHING, and this used to return `enabled` - true at
    /// every point in the world.
    ///
    /// `covers` answers "could this source have seen something here", and the
    /// engine asks it two questions with that answer. `anyone_covers` decides
    /// whether a miss is evidence of absence or just nobody looking; with a
    /// spoofer in the estate it was true everywhere, so the coverage map that
    /// the `spoofing` scenario supplies told the engine the whole world was
    /// under observation and stopped doing its job. And the detection-rate
    /// estimator charged the spoofer with a miss every time any entity it had
    /// never fed walked anywhere, driving its estimated rate to the floor for
    /// a reason that has nothing to do with spoofing.
    ///
    /// A spoofer does not observe. It fabricates, at a place of its own
    /// choosing, and there is no location at which its silence means anything.
    [[nodiscard]] bool covers(Vec2) const override { return false; }
    [[nodiscard]] const Config& config() const { return cfg_; }

    /// Where the phantom would appear at `t`, whether or not it is reported.
    [[nodiscard]] Vec2 phantom_position(Real t) const {
        return cfg_.origin + cfg_.velocity * std::max(t - cfg_.start_time_s, 0.0);
    }

    std::vector<Observation> observe(const WorldSnapshot& truth, Rng& rng,
                                     DetectionLedger* ledger = nullptr) override;

private:
    Config cfg_;
    long counter_{0};
};

/// Collect one scan from every sensor.
std::vector<Observation> collect(const std::vector<SensorPtr>& sensors,
                                 const WorldSnapshot& truth, Rng& rng,
                                 DetectionLedger* ledger = nullptr);

}  // namespace trace::sim
