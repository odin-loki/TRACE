// TRACE — Tracking, Re-identification, Association, Convergence & Events
// Core value types shared across the engine.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace trace {

using Real = double;

inline constexpr int kStateDim = 4;   // [x, y, vx, vy]
inline constexpr int kObsDim   = 2;   // [x, y]

/// Fixed 2-vector. Small enough that every operation inlines to registers.
struct Vec2 {
    Real x{0.0};
    Real y{0.0};

    constexpr Vec2() = default;
    constexpr Vec2(Real x_, Real y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(Real s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(Real s) const { return {x / s, y / s}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }

    [[nodiscard]] constexpr Real dot(Vec2 o) const { return x * o.x + y * o.y; }
    [[nodiscard]] Real norm() const { return std::sqrt(x * x + y * y); }
    [[nodiscard]] constexpr Real norm_sq() const { return x * x + y * y; }
    [[nodiscard]] Vec2 unit() const {
        const Real n = norm();
        return n > 1e-12 ? Vec2{x / n, y / n} : Vec2{0.0, 0.0};
    }
};

inline Real distance(Vec2 a, Vec2 b) { return (a - b).norm(); }

/// Symmetric 2x2 matrix, stored as [a b; b c]. Used for position covariance
/// and innovation covariance, where an explicit inverse beats a general solver.
struct Mat2 {
    Real a{0.0};  // (0,0)
    Real b{0.0};  // (0,1) == (1,0)
    Real c{0.0};  // (1,1)

    [[nodiscard]] constexpr Real det() const { return a * c - b * b; }
    [[nodiscard]] constexpr Real trace_() const { return a + c; }

    /// Inverse with ridge regularisation; a singular covariance is a modelling
    /// failure upstream, not something the caller should have to branch on.
    [[nodiscard]] Mat2 inverse(Real ridge = 1e-9) const {
        const Real aa = a + ridge;
        const Real cc = c + ridge;
        Real d = aa * cc - b * b;
        if (std::abs(d) < 1e-15) d = (d < 0.0 ? -1e-15 : 1e-15);
        return Mat2{cc / d, -b / d, aa / d};
    }

    /// x^T M x
    [[nodiscard]] constexpr Real quad(Vec2 v) const {
        return v.x * (a * v.x + b * v.y) + v.y * (b * v.x + c * v.y);
    }
};

/// Intelligence-discipline label on an observation. The names are inherited
/// from the intelligence domain, but the engine only uses them to look up a
/// reliability prior — civil domains remap them freely (see docs/USE_CASES.md).
enum class Modality : std::uint8_t {
    GEOINT = 0,  // imagery / camera / overhead
    SIGINT,      // emitter geolocation
    COMMS,       // device / CDR
    HUMINT,      // human report
    OSINT,       // open source
    Count
};

inline constexpr std::array<std::string_view, static_cast<std::size_t>(Modality::Count)>
    kModalityNames{"GEOINT", "SIGINT", "COMMS", "HUMINT", "OSINT"};

inline std::string_view to_string(Modality m) {
    return kModalityNames[static_cast<std::size_t>(m)];
}

inline Modality modality_from_string(std::string_view s) {
    for (std::size_t i = 0; i < kModalityNames.size(); ++i) {
        if (kModalityNames[i] == s) return static_cast<Modality>(i);
    }
    return Modality::OSINT;
}

/// Priority tier assigned by the threat scorer.
enum class Priority : std::uint8_t { MONITOR = 0, LOW, MEDIUM, HIGH, IMMEDIATE };

inline std::string_view to_string(Priority p) {
    switch (p) {
        case Priority::IMMEDIATE: return "IMMEDIATE";
        case Priority::HIGH:      return "HIGH";
        case Priority::MEDIUM:    return "MEDIUM";
        case Priority::LOW:       return "LOW";
        default:                  return "MONITOR";
    }
}

/// Severity attached to a detector event.
enum class Severity : std::uint8_t { INFO = 0, LOW, MEDIUM, HIGH, CRITICAL };

inline std::string_view to_string(Severity s) {
    switch (s) {
        case Severity::CRITICAL: return "CRITICAL";
        case Severity::HIGH:     return "HIGH";
        case Severity::MEDIUM:   return "MEDIUM";
        case Severity::LOW:      return "LOW";
        default:                 return "INFO";
    }
}

/// Rectangular area of regard, in metres.
struct Area {
    Real xmin{-5000.0};
    Real xmax{5000.0};
    Real ymin{-5000.0};
    Real ymax{5000.0};

    [[nodiscard]] constexpr Real width()  const { return xmax - xmin; }
    [[nodiscard]] constexpr Real height() const { return ymax - ymin; }
    [[nodiscard]] constexpr Real volume() const { return width() * height(); }
    [[nodiscard]] constexpr bool contains(Vec2 p) const {
        return p.x >= xmin && p.x <= xmax && p.y >= ymin && p.y <= ymax;
    }
};

}  // namespace trace
