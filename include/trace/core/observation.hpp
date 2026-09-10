// TRACE — one sighting from one sensor at one instant.
#pragma once

#include <optional>
#include <string>

#include "trace/core/descriptor.hpp"
#include "trace/core/types.hpp"

namespace trace {

/// A single sensor report.
///
/// `position` is optional: a COMMS or SIGINT hit can establish that an entity
/// was active without pinning it in space. Position-less observations still
/// feed source credibility and the report's observation count, but are skipped
/// by the association step.
struct Observation {
    std::string          obs_id;
    Real                 timestamp{0.0};   // seconds
    std::optional<Vec2>  position;
    Modality             modality{Modality::GEOINT};
    Real                 confidence{1.0};  // 0..1, sensor's own confidence
    std::string          source_id;        // e.g. "CAM_NORTH_01"

    /// Optional non-kinematic evidence about *which* entity this is. Absent by
    /// default: most sensors supply position and nothing else, and the engine
    /// works without it. Where it exists it is the only thing that can separate
    /// two entities whose paths have just crossed.
    Descriptor           descriptor{};

    Observation() = default;

    Observation(std::string id, Real ts, Vec2 pos, Modality mod,
                Real conf = 1.0, std::string src = {})
        : obs_id(std::move(id)), timestamp(ts), position(pos),
          modality(mod), confidence(conf), source_id(std::move(src)) {}

    /// Position-less variant (device seen, location unknown).
    Observation(std::string id, Real ts, Modality mod, Real conf,
                std::string src)
        : obs_id(std::move(id)), timestamp(ts), position(std::nullopt),
          modality(mod), confidence(conf), source_id(std::move(src)) {}

    [[nodiscard]] bool has_position() const { return position.has_value(); }
    [[nodiscard]] bool has_descriptor() const { return descriptor.valid(); }

    /// Attach an appearance descriptor, normalising it on the way in.
    Observation& with_descriptor(Descriptor d) {
        d.normalise();
        descriptor = d;
        return *this;
    }
    [[nodiscard]] Vec2 pos_or_zero() const { return position.value_or(Vec2{}); }
};

}  // namespace trace
