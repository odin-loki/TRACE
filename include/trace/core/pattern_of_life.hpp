// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — Pattern of Life.
//
// A per-entity Gaussian mixture over [hour-of-day, x, y]. It answers three
// questions the kinematic filter cannot:
//   1. is this entity where it usually is, at this time of day?  (anomaly)
//   2. where will it probably be at time t?                      (prediction)
//   3. if it vanished, where should we look for it?              (reacquisition)
//
// Fitted by EM once `min_obs` sightings have accumulated, then refitted every
// `refit_interval` further sightings. Until then it reports a neutral score:
// an entity with no baseline is not thereby suspicious.
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>
#include <array>
#include <optional>
#include <vector>

#include "trace/core/profile.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/types.hpp"

namespace trace {

/// Dimensionality of the pattern-of-life feature space: [hour, x, y].
inline constexpr int kPolDim = 3;

using Vec3 = std::array<Real, kPolDim>;

/// Lower-triangular 3x3 Cholesky factor, stored dense for simplicity.
struct Chol3 {
    std::array<std::array<Real, kPolDim>, kPolDim> L{};
    Real log_det{0.0};
    bool valid{false};

    /// Factorise a symmetric positive-definite matrix, adding a ridge until it
    /// succeeds. A degenerate component means every sighting landed in one
    /// spot, which is legitimate — a stationary asset does exactly that.
    static Chol3 factor(const std::array<std::array<Real, kPolDim>, kPolDim>& A,
                        Real ridge = 1e-6);

    /// Solve L y = b in place semantics (forward substitution).
    [[nodiscard]] Vec3 forward_solve(const Vec3& b) const;

    /// L z, used to draw a correlated sample from a standard normal z.
    [[nodiscard]] Vec3 apply(const Vec3& z) const;
};

/// One mixture component.
struct PolComponent {
    Vec3 mean{};
    std::array<std::array<Real, kPolDim>, kPolDim> cov{};
    Real weight{0.0};
    Chol3 chol{};
};

class PatternOfLife {
public:
    PatternOfLife() = default;
    explicit PatternOfLife(const DomainProfile& profile);

    /// Record a sighting. Triggers a refit when enough have accumulated.
    void add(Real timestamp, Vec2 position);

    /// Inherit a fitted baseline from a related entity — used when a track
    /// splits off an existing group, so the child does not start blind.
    void clone_from(const PatternOfLife& other);

    [[nodiscard]] bool fitted() const { return fitted_; }
    [[nodiscard]] std::size_t n_observations() const { return obs_.size(); }

    /// 0 = perfectly ordinary, 1 = never seen this behaviour.
    /// Returns 0.5 when unfitted: unknown, not alarming.
    [[nodiscard]] Real anomaly_score(Real timestamp, Vec2 position) const;

    /// Expected position at `timestamp`, with a 1-sigma radius. The radius is
    /// 999 m when unfitted, which callers use as "no opinion".
    struct Prediction {
        Vec2 position{};
        Real uncertainty_m{999.0};
    };
    [[nodiscard]] Prediction predict_location(Real timestamp, Rng& rng,
                                              int n_mc = 60) const;

    /// Hour-of-day windows in which this entity is normally active.
    [[nodiscard]] std::vector<std::pair<Real, Real>> active_windows() const;

    /// Typical dwell radius — how tightly clustered the baseline is in space.
    /// Used by the loiter detector to decide what counts as "stopped for this
    /// entity" rather than applying one global threshold.
    [[nodiscard]] Real spatial_spread() const;

    [[nodiscard]] const std::vector<PolComponent>& components() const {
        return components_;
    }

private:
    void em_fit();
    /// Log density of the whole MIXTURE at `x` - every component's weight is
    /// already inside it.
    [[nodiscard]] Real log_prob(const Vec3& x) const;
    /// Log density of ONE component at `x`, with no weight in it. What a
    /// responsibility is built from; `log_prob` is not.
    [[nodiscard]] Real component_log_prob(std::size_t c, const Vec3& x) const;

    static Vec3 featurise(Real timestamp, Vec2 position) {
        // Hour of day folds a multi-day history onto one 24-hour cycle, which
        // is what makes "he is never here at 3am" expressible.
        Real hour = std::fmod(timestamp, 86400.0) / 3600.0;
        if (hour < 0.0) hour += 24.0;
        return Vec3{hour, position.x, position.y};
    }

    const DomainProfile* profile_{nullptr};
    int k_{5};
    int min_obs_{15};
    int refit_interval_{5};

    std::vector<Vec3> obs_;
    std::vector<PolComponent> components_;
    Real baseline_nll_{4.0};
    bool fitted_{false};
    int obs_since_refit_{0};

    mutable Rng rng_{0xC0FFEE};
};

}  // namespace trace
