// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — appearance descriptors.
//
// Kinematics alone cannot say which of two people who just crossed is which.
// Nothing in the position, velocity or existence of two tracks distinguishes
// them at the moment their paths intersect, and that is where identity is lost.
// A descriptor carries whatever non-kinematic evidence a sensor can supply -
// a re-identification embedding from a camera, a transponder's reported vessel
// class, a tag's RSSI signature, or simply the geometry of a detection box.
//
// The engine is deliberately incurious about where the numbers come from. It
// only requires that similar entities produce similar vectors, and that the
// vector be L2-normalised so cosine similarity is meaningful.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "trace/core/types.hpp"

namespace trace {

/// Fixed width, so an Observation stays allocation-free on the hot path.
///
/// Thirty-two dimensions is far smaller than a raw re-identification embedding
/// (typically 128-2048). Reduce a learned embedding with PCA before handing it
/// over: the discriminative power survives the reduction, and a per-detection
/// heap allocation in the association inner loop would not be worth what the
/// extra dimensions buy.
inline constexpr std::size_t kDescriptorDim = 32;

struct Descriptor {
    std::array<float, kDescriptorDim> v{};
    bool present{false};

    [[nodiscard]] bool valid() const { return present; }

    /// Scale to unit length. Cosine similarity is only meaningful afterwards,
    /// so this is applied on construction rather than trusted to the caller.
    void normalise() {
        double sum = 0.0;
        for (const float x : v) sum += static_cast<double>(x) * x;
        if (sum <= 1e-20) {
            present = false;
            return;
        }
        const auto inv = static_cast<float>(1.0 / std::sqrt(sum));
        for (float& x : v) x *= inv;
        present = true;
    }

    /// Cosine similarity in [-1, 1]; 0 when either side is absent, which reads
    /// as "no evidence" rather than "dissimilar".
    [[nodiscard]] Real similarity(const Descriptor& other) const {
        if (!present || !other.present) return 0.0;
        double dot = 0.0;
        for (std::size_t i = 0; i < kDescriptorDim; ++i) {
            dot += static_cast<double>(v[i]) * other.v[i];
        }
        return std::clamp(dot, -1.0, 1.0);
    }

    /// Blend an observation into a running model. Momentum near 1 keeps a long
    /// memory, which is what makes the descriptor useful across an occlusion;
    /// too long and it stops tracking genuine change in appearance.
    void blend(const Descriptor& obs, Real momentum) {
        if (!obs.present) return;
        if (!present) {
            *this = obs;
            return;
        }
        const auto m = static_cast<float>(std::clamp(momentum, 0.0, 1.0));
        for (std::size_t i = 0; i < kDescriptorDim; ++i) {
            v[i] = m * v[i] + (1.0f - m) * obs.v[i];
        }
        normalise();
    }
};

/// Build a descriptor by softly binning a set of scalar features.
///
/// Cosine similarity on a handful of raw scalars is nearly meaningless - every
/// vector points in almost the same direction. Spreading each feature across a
/// band of bins as a Gaussian bump turns "these two values are close" into
/// "these two vectors overlap", which is what cosine actually measures.
class SoftBinner {
public:
    struct Feature {
        Real value;
        Real lo;      ///< start of the represented range
        Real hi;      ///< end of the represented range
        int bins;     ///< bins allotted to this feature
        Real width;   ///< bump width, in bins
    };

    static Descriptor encode(const std::vector<Feature>& features) {
        Descriptor d;
        std::size_t offset = 0;
        for (const auto& f : features) {
            const auto n = static_cast<std::size_t>(std::max(f.bins, 1));
            if (offset + n > kDescriptorDim) break;

            const Real span = std::max(f.hi - f.lo, 1e-9);
            const Real pos = std::clamp((f.value - f.lo) / span, 0.0, 1.0) * (n - 1);
            const Real w = std::max(f.width, 0.25);

            for (std::size_t b = 0; b < n; ++b) {
                const Real dist = (static_cast<Real>(b) - pos) / w;
                d.v[offset + b] = static_cast<float>(std::exp(-0.5 * dist * dist));
            }
            offset += n;
        }
        d.normalise();
        return d;
    }
};

}  // namespace trace
