// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — deterministic random number generation.
//
// Every stochastic step in the engine draws from an explicitly seeded Rng so a
// scenario replays bit-for-bit. That matters more here than usual: the whole
// point of the simulation suite is that a regression shows up as a changed
// track, not as noise.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>

#include "trace/backend/simd.hpp"

namespace trace {

/// xoshiro256++ — small, fast, and good enough for Monte-Carlo integration.
class Rng {
public:
    explicit Rng(std::uint64_t seed = 0x9E3779B97F4A7C15ULL) { reseed(seed); }

    void reseed(std::uint64_t seed) {
        // SplitMix64 to spread a single seed across the four state words.
        std::uint64_t z = seed;
        for (auto& s : s_) {
            z += 0x9E3779B97F4A7C15ULL;
            std::uint64_t t = z;
            t = (t ^ (t >> 30)) * 0xBF58476D1CE4E5B9ULL;
            t = (t ^ (t >> 27)) * 0x94D049BB133111EBULL;
            s = t ^ (t >> 31);
        }
        has_spare_ = false;
    }

    std::uint64_t next_u64() {
        const std::uint64_t result = rotl(s_[0] + s_[3], 23) + s_[0];
        const std::uint64_t t = s_[1] << 17;
        s_[2] ^= s_[0];
        s_[3] ^= s_[1];
        s_[1] ^= s_[2];
        s_[0] ^= s_[3];
        s_[2] ^= t;
        s_[3] = rotl(s_[3], 45);
        return result;
    }

    /// Uniform in [0, 1).
    double uniform() {
        return static_cast<double>(next_u64() >> 11) * 0x1.0p-53;
    }

    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

    /// Standard normal via Marsaglia polar, caching the spare deviate.
    double normal() {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }
        double u, v, s;
        do {
            u = 2.0 * uniform() - 1.0;
            v = 2.0 * uniform() - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);
        const double f = std::sqrt(-2.0 * std::log(s) / s);
        spare_ = v * f;
        has_spare_ = true;
        return u * f;
    }

    double normal(double mean, double stddev) { return mean + stddev * normal(); }

    /// Poisson deviate (Knuth for small lambda, normal approximation above 30).
    int poisson(double lambda) {
        if (lambda <= 0.0) return 0;
        if (lambda > 30.0) {
            const double v = normal(lambda, std::sqrt(lambda));
            return v < 0.0 ? 0 : static_cast<int>(v + 0.5);
        }
        const double limit = std::exp(-lambda);
        double p = 1.0;
        int k = 0;
        do {
            ++k;
            p *= uniform();
        } while (p > limit);
        return k - 1;
    }

    /// Index sampled from an unnormalised weight vector.
    std::size_t categorical(const double* weights, std::size_t n) {
        double total = 0.0;
        for (std::size_t i = 0; i < n; ++i) total += weights[i];
        if (total <= 0.0) return 0;
        double u = uniform() * total;
        for (std::size_t i = 0; i < n; ++i) {
            u -= weights[i];
            if (u <= 0.0) return i;
        }
        return n - 1;
    }

    bool bernoulli(double p) { return uniform() < p; }

    int uniform_int(int lo, int hi_inclusive) {
        if (hi_inclusive <= lo) return lo;
        const auto span = static_cast<std::uint64_t>(hi_inclusive - lo + 1);
        return lo + static_cast<int>(next_u64() % span);
    }

    /// Gamma(shape, 1) via Marsaglia-Tsang; the Beta sampler is built on it.
    double gamma(double shape) {
        if (shape < 1.0) {
            const double u = uniform();
            return gamma(shape + 1.0) * std::pow(u, 1.0 / shape);
        }
        const double d = shape - 1.0 / 3.0;
        const double c = 1.0 / std::sqrt(9.0 * d);
        for (;;) {
            double x, v;
            do {
                x = normal();
                v = 1.0 + c * x;
            } while (v <= 0.0);
            v = v * v * v;
            const double u = uniform();
            if (u < 1.0 - 0.0331 * x * x * x * x) return d * v;
            if (std::log(u) < 0.5 * x * x + d * (1.0 - v + std::log(v))) return d * v;
        }
    }

    double beta(double a, double b) {
        const double x = gamma(a);
        const double y = gamma(b);
        const double s = x + y;
        return s > 0.0 ? x / s : 0.5;
    }

private:
    static std::uint64_t rotl(std::uint64_t x, int k) {
        return (x << k) | (x >> (64 - k));
    }

    std::array<std::uint64_t, 4> s_{};
    double spare_{0.0};
    bool has_spare_{false};
};

/// Lane-parallel xoshiro256++ feeding the particle kernels.
///
/// Each lane carries its own independent stream, seeded by jumping the scalar
/// generator. Normals come from a vectorised Box-Muller: one call fills two
/// batches, which is exactly the shape the OU velocity update wants (x and y).
class SimdRng {
public:
    explicit SimdRng(std::uint64_t seed = 0xDEADBEEFCAFEF00DULL) { reseed(seed); }

    void reseed(std::uint64_t seed) {
        Rng seeder(seed);
        for (auto& lane_state : s_) {
            for (std::size_t lane = 0; lane < simd::kLanes; ++lane) {
                lane_state[lane] = seeder.next_u64();
            }
        }
    }

    /// Uniform in [0, 1) per lane, via the exponent-splice trick: build a
    /// double in [1, 2) from raw mantissa bits and subtract one. Avoids an
    /// unsigned-to-double conversion, which AVX2 lacks.
    simd::Batch uniform() {
        const simd::BatchU bits = next_bits();
        const simd::BatchU mantissa =
            (bits >> 12) | simd::BatchU{0x3FF0000000000000ULL};
        return simd::bits_to_double(mantissa) - simd::Batch{1.0};
    }

    /// Two independent standard-normal batches per call (Box-Muller).
    void normal_pair(simd::Batch& z0, simd::Batch& z1) {
        // Nudge u1 off zero so the log stays finite.
        const simd::Batch u1 = uniform() * simd::Batch{0.9999999} + simd::Batch{1e-12};
        const simd::Batch u2 = uniform();
        const simd::Batch r = simd::sqrt_(simd::Batch{-2.0} * simd::log_(u1));
        const simd::Batch theta = simd::Batch{2.0 * std::numbers::pi} * u2;
        z0 = r * simd::cos_(theta);
        z1 = r * simd::sin_(theta);
    }

private:
    simd::BatchU next_bits() {
        // Explicit loads: xsimd's operators are templates, so the LaneState
        // conversion operator is not a candidate during overload resolution.
        simd::BatchU s0 = s_[0].batch();
        simd::BatchU s1 = s_[1].batch();
        simd::BatchU s2 = s_[2].batch();
        simd::BatchU s3 = s_[3].batch();

        const simd::BatchU result = rotl(s0 + s3, 23) + s0;
        const simd::BatchU t = s1 << 17;
        s2 = s2 ^ s0;
        s3 = s3 ^ s1;
        s1 = s1 ^ s2;
        s0 = s0 ^ s3;
        s2 = s2 ^ t;
        s3 = rotl(s3, 45);

        s_[0].store(s0);
        s_[1].store(s1);
        s_[2].store(s2);
        s_[3].store(s3);
        return result;
    }

    static simd::BatchU rotl(const simd::BatchU& x, int k) {
        return (x << k) | (x >> (64 - k));
    }

#if defined(TRACE_WITH_XSIMD)
    // xsimd batches are register types; hold state in memory and reload.
    struct LaneState {
        alignas(64) std::array<std::uint64_t, simd::kLanes> v{};
        std::uint64_t& operator[](std::size_t i) { return v[i]; }
        [[nodiscard]] simd::BatchU batch() const {
            return simd::BatchU::load_aligned(v.data());
        }
        void store(const simd::BatchU& b) { b.store_aligned(v.data()); }
    };
#else
    struct LaneState {
        std::array<std::uint64_t, 1> v{};
        std::uint64_t& operator[](std::size_t i) { return v[i]; }
        [[nodiscard]] simd::BatchU batch() const { return simd::BatchU{v[0]}; }
        void store(const simd::BatchU& b) { v[0] = b.v; }
    };
#endif
    std::array<LaneState, 4> s_{};
};

}  // namespace trace
