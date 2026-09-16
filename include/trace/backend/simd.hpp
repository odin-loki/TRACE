// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — SIMD backend selection.
//
// Particle propagation - `src/core/particle_filter.cpp`, and the RNG it draws
// from - is written once against these aliases. With xsimd present it compiles
// to AVX2/AVX-512; without it `Batch` degrades to a one-lane type and the same
// source runs scalar.
//
// That is the whole of it. This used to claim "likelihood evaluation" and "GMM
// responsibilities" too; `grep -rl 'simd::' src include` finds particle_filter,
// rng.hpp and one line of bench_main, and nothing else. The GMM E step in
// pattern_of_life.cpp is scalar. Vectorising it is a reasonable thing to do and
// a different thing from saying it has been done.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstddef>

#include "trace/core/types.hpp"

#if defined(TRACE_WITH_XSIMD)
#include <xsimd/xsimd.hpp>
#endif

namespace trace::simd {

#if defined(TRACE_WITH_XSIMD)

using Batch  = xsimd::batch<Real>;
using BatchU = xsimd::batch<std::uint64_t>;

inline constexpr std::size_t kLanes = Batch::size;
inline constexpr bool kEnabled = true;

inline Batch load_u(const Real* p)            { return Batch::load_unaligned(p); }
inline void  store_u(Real* p, const Batch& b) { b.store_unaligned(p); }
inline Batch sqrt_(const Batch& b)            { return xsimd::sqrt(b); }
inline Batch log_(const Batch& b)             { return xsimd::log(b); }
inline Batch exp_(const Batch& b)             { return xsimd::exp(b); }
inline Batch sin_(const Batch& b)             { return xsimd::sin(b); }
inline Batch cos_(const Batch& b)             { return xsimd::cos(b); }
inline Batch fma_(const Batch& a, const Batch& b, const Batch& c) {
    return xsimd::fma(a, b, c);
}
inline Real  hadd_(const Batch& b)            { return xsimd::reduce_add(b); }

/// Reinterpret 64 raw bits per lane as a double without converting.
inline Batch bits_to_double(const BatchU& u) {
    return xsimd::bitwise_cast<Real>(u);
}

/// The architecture `Batch` actually resolved to, from xsimd itself.
///
/// This used to interpolate `TRACE_XSIMD_ARCH_NAME`, a CMake variable set to
/// the literal "generic" at the top of CMakeLists.txt and never derived from
/// anything - so an AVX-512 build reported "xsimd/generic" and `trace_bench`
/// printed it. `Batch` is `xsimd::batch<Real>`, which is
/// `xsimd::batch<Real, xsimd::default_arch>`, so the arch is already knowable
/// here and does not need telling.
inline constexpr const char* backend_name() {
    return xsimd::batch<Real>::arch_type::name();
}

#else  // ---- scalar fallback -------------------------------------------------

/// One-lane stand-in with just enough of the batch interface for the kernels.
struct Batch {
    Real v{0.0};
    static constexpr std::size_t size = 1;

    Batch() = default;
    Batch(Real x) : v(x) {}  // NOLINT(google-explicit-constructor)

    friend Batch operator+(Batch a, Batch b) { return Batch{a.v + b.v}; }
    friend Batch operator-(Batch a, Batch b) { return Batch{a.v - b.v}; }
    friend Batch operator*(Batch a, Batch b) { return Batch{a.v * b.v}; }
    friend Batch operator/(Batch a, Batch b) { return Batch{a.v / b.v}; }
};

struct BatchU {
    std::uint64_t v{0};
    static constexpr std::size_t size = 1;

    BatchU() = default;
    BatchU(std::uint64_t x) : v(x) {}  // NOLINT(google-explicit-constructor)

    friend BatchU operator^(BatchU a, BatchU b) { return BatchU{a.v ^ b.v}; }
    friend BatchU operator|(BatchU a, BatchU b) { return BatchU{a.v | b.v}; }
    friend BatchU operator&(BatchU a, BatchU b) { return BatchU{a.v & b.v}; }
    friend BatchU operator+(BatchU a, BatchU b) { return BatchU{a.v + b.v}; }
    friend BatchU operator*(BatchU a, BatchU b) { return BatchU{a.v * b.v}; }
    friend BatchU operator<<(BatchU a, int s)   { return BatchU{a.v << s}; }
    friend BatchU operator>>(BatchU a, int s)   { return BatchU{a.v >> s}; }
};

inline constexpr std::size_t kLanes = 1;
inline constexpr bool kEnabled = false;

inline Batch load_u(const Real* p)            { return Batch{*p}; }
inline void  store_u(Real* p, const Batch& b) { *p = b.v; }
inline Batch sqrt_(const Batch& b)            { return Batch{std::sqrt(b.v)}; }
inline Batch log_(const Batch& b)             { return Batch{std::log(b.v)}; }
inline Batch exp_(const Batch& b)             { return Batch{std::exp(b.v)}; }
inline Batch sin_(const Batch& b)             { return Batch{std::sin(b.v)}; }
inline Batch cos_(const Batch& b)             { return Batch{std::cos(b.v)}; }
inline Batch fma_(const Batch& a, const Batch& b, const Batch& c) {
    return Batch{std::fma(a.v, b.v, c.v)};
}
inline Real  hadd_(const Batch& b)            { return b.v; }

inline Batch bits_to_double(const BatchU& u) {
    Real out;
    std::memcpy(&out, &u.v, sizeof(out));
    return Batch{out};
}

inline constexpr const char* backend_name() { return "scalar"; }

#endif

}  // namespace trace::simd
