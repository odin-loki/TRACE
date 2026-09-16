// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

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
    void forget(const std::set<std::string>& live) override {
        forget_by_id(visits_, live);
        forget_by_pair(contact_streak_, live);
    }

private:
    std::unordered_map<std::string, std::deque<Visit>> visits_;
    std::map<std::pair<std::string, std::string>, int> contact_streak_;
};

/// Predicts meetings ahead of time by stacking three independent methods:
/// straight-line intercept, closure-rate extrapolation, and pattern-of-life
/// cross-prediction. They fail in different circumstances, so running all three
/// and taking the most confident covers far more cases than any one alone.
/// Least-squares velocity over a track's recent history, in metres per SCAN.
///
/// Declared here rather than kept private to rendezvous.cpp because it carries
/// a units contract - metres per scan, from a history that is appended once per
/// DETECTION rather than once per scan - and a units contract that cannot be
/// tested directly is a units contract that drifts. See the definition for what
/// regressing against the sample index instead of the sample timestamp cost.
Vec2 fitted_velocity(const Track& t, Real scan_dt, std::size_t window = 6);

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
    void forget(const std::set<std::string>& live) override {
        forget_by_pair(sep_history_, live);
    }

private:
    std::map<std::pair<std::string, std::string>, std::deque<SepSample>> sep_history_;
    int scans_since_prune_{0};

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
    void forget(const std::set<std::string>& live) override {
        forget_by_pair(streak_, live);
    }

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

    void forget(const std::set<std::string>& live) override {
        // Only the dedup set, whose tag is "<from>><to>" and which grows
        // without limit. `recent_stops_` is deliberately left alone: it is
        // already bounded twice over, by a time cutoff and by a hard cap, and
        // a stop recorded by a track that has since been retired is still a
        // historical fact within that window. Dropping it would suppress
        // exactly the handover this detector exists to find.
        forget_tags(reported_, live, '>');
    }

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
    void forget(const std::set<std::string>& live) override {
        forget_by_id(dwell_, live);
    }

private:
    std::unordered_map<std::string, Dwell> dwell_;
};

/// A stop that is nowhere in the entity's normal pattern, especially one close
/// to a location of interest.
class CoverStopDetector final : public Detector {
public:
    [[nodiscard]] std::string name() const override { return "CoverStop"; }
    std::vector<DetectionEvent> detect(const std::vector<TrackPtr>& tracks,
                                       const DetectorContext& ctx) override;

    void forget(const std::set<std::string>& live) override {
        // The dedup tag is "<id>@<cell>".
        forget_tags(reported_, live, '@');
    }

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
    void forget(const std::set<std::string>& live) override {
        forget_by_id(cells_, live);
    }

private:
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
    void forget(const std::set<std::string>& live) override {
        forget_by_id(contacts_, live);
        for (auto& [id, peers] : contacts_) forget_by_id(peers, live);
        forget_by_id(role_history_, live);
        forget_by_id(speed_history_, live);
        forget_by_id(last_seen_, live);
    }

private:
    std::unordered_map<std::string, std::unordered_map<std::string, int>> contacts_;
    std::unordered_map<std::string, std::deque<std::string>> role_history_;
    std::unordered_map<std::string, std::deque<Real>> speed_history_;
    std::unordered_map<std::string, int> last_seen_;
    int scan_{0};
};

/// Construct the default eight-detector pipeline.
std::vector<DetectorPtr> default_detectors();

}  // namespace trace
