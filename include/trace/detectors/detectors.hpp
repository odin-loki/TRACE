// TRACE — the built-in detector set.
//
// Eight detectors ship by default. Each one turns a kinematic pattern into a
// named behaviour; the vocabulary below is the intelligence-domain naming, and
// docs/USE_CASES.md gives the civil reading of each (a "brush pass" in a
// warehouse is a custody handover; a "chokepoint" in a retail store is a
// display someone keeps returning to).
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <unordered_map>

#include "trace/detectors/base.hpp"

namespace trace {

/// Close approaches, surveillance-detection loops, and dead drops.
class TradecraftDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "Tradecraft"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    struct Visit {
        Real timestamp{0.0};
        Vec2 position{};
    };
    std::unordered_map<std::string, std::deque<Visit>> visits_;
    std::map<std::pair<std::string, std::string>, int> contact_streak_;
};

/// Predicts meetings ahead of time by stacking three independent methods:
/// straight-line intercept, closure-rate extrapolation, and pattern-of-life
/// cross-prediction. They fail in different circumstances, so running all three
/// and taking the most confident covers far more cases than any one alone.
class RendezvousWarner final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "RendezvousWarner"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;
    std::vector<RendezvousWarning> rendezvous(const std::vector<TrackPtr>& tracks,
                                              const DetectorContext& ctx) override;

private:
    struct SepSample {
        Real timestamp{0.0};
        Real separation{0.0};
    };
    std::map<std::pair<std::string, std::string>, std::deque<SepSample>> sep_history_;

    /// One track's pattern-of-life forecast across the warning horizon.
    ///
    /// Computed once per track per scan and reused for every pair that track
    /// appears in. Recomputing it inside the pair loop meant each track's
    /// forecast was rebuilt once for every other track - at 270 tracks that is
    /// 270 times over, and it made this detector 95% of the engine's runtime.
    struct PolForecast {
        std::vector<Vec2> position;
        std::vector<Real> uncertainty;
        bool valid{false};
    };

    std::optional<RendezvousWarning> geometric_intercept(
        const Track& a, const Track& b, const DetectorContext& ctx) const;
    std::optional<RendezvousWarning> separation_rate(
        const Track& a, const Track& b, const std::deque<SepSample>& hist,
        const DetectorContext& ctx) const;
    std::optional<RendezvousWarning> pol_cross_predict(
        const Track& a, const Track& b, const PolForecast& fa,
        const PolForecast& fb, const DetectorContext& ctx) const;
};

/// One entity shadowing another: same heading, steady lateral offset, held
/// across many consecutive scans.
class ParallelRouteDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "ParallelRoute"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    std::map<std::pair<std::string, std::string>, int> streak_;
};

/// A transport change: one track stops and a differently-moving track appears
/// beside it moments later. Vehicle-to-foot, foot-to-vehicle, pallet-to-truck.
class ModeTransitionDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "ModeTransition"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    struct Stop {
        std::string track_id;
        Vec2 position{};
        Real timestamp{0.0};
        std::string model;
    };
    std::deque<Stop> recent_stops_;
    std::set<std::string> reported_;
};

/// Dwelling far longer than this entity's own pattern of life would predict.
/// Comparing against the entity's own baseline rather than a global threshold
/// is what keeps a habitually stationary asset from alerting constantly.
class LoiterDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "Loiter"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    struct Dwell {
        Vec2 anchor{};
        Real since{0.0};
        bool reported{false};
    };
    std::unordered_map<std::string, Dwell> dwell_;
};

/// A stop that is nowhere in the entity's normal pattern, especially one close
/// to a location of interest.
class CoverStopDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "CoverStop"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    std::set<std::string> reported_;
};

/// Repeated passes through the same small cell — the signature of someone
/// watching a fixed point rather than travelling through it.
class ChokepointDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "Chokepoint"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

private:
    struct CellVisit {
        long cx{0};
        long cy{0};
        Real last_time{0.0};
        int count{0};
        bool reported{false};
    };
    std::unordered_map<std::string, std::vector<CellVisit>> cells_;
};

/// Assigns each entity a role from its movement and contact pattern:
/// a hub that others come to, a mover that connects hubs, or a leaf.
class NetworkRoleDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "NetworkRole"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;
    std::vector<NetworkRole> roles(const std::vector<TrackPtr>& tracks,
                                   const DetectorContext& ctx) override;

private:
    /// Who each track has been near, and when. Not a lifetime set: a lifetime
    /// set converges on "everyone has met everyone" in any scene that runs
    /// long enough, and gets there far sooner than that when transient tracks
    /// are present, since every one of them leaves a permanent mark on
    /// whatever it appeared next to. Entries age out, which also stops this
    /// map growing without bound in a long-running deployment.
    std::unordered_map<std::string, std::unordered_map<std::string, int>> contacts_;
    std::unordered_map<std::string, std::deque<std::string>> role_history_;
    std::unordered_map<std::string, std::deque<Real>> speed_history_;
    std::unordered_map<std::string, int> last_seen_;
    int scan_{0};
};

/// Construct the default eight-detector pipeline.
std::vector<DetectorPtr> default_detectors();

}  // namespace trace
