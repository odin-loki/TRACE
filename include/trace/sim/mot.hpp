// TRACE — MOTChallenge replay.
//
// Every other number in this repository comes from TRACE's own simulator, which
// is evidence the code does what it is meant to and no evidence about the
// world. This harness replays real sequences from MOTChallenge (MOT17, MOT20)
// so the tracker meets detections produced by real detectors on real video:
// genuine false positives, genuine misses, genuine occlusion.
//
// TRACE consumes detections, not pixels, so only the label archives are needed
// (~10 MB) rather than the full image sets. See scripts/fetch_mot.sh.
//
// Coordinates are image pixels, and the "ground plane" proxy for a person is
// the bottom-centre of their bounding box - the standard choice, and the one
// point on a box that stays put as the box grows with proximity to the camera.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "trace/core/observation.hpp"
#include "trace/core/profile.hpp"
#include "trace/core/types.hpp"
#include "trace/sim/world.hpp"

namespace trace::sim {

/// One annotated box in one frame.
struct MotBox {
    int frame{0};
    int id{-1};          ///< ground-truth identity, -1 for detections
    Real x{0.0};         ///< left
    Real y{0.0};         ///< top
    Real w{0.0};
    Real h{0.0};
    Real score{1.0};     ///< detector confidence, or GT "consider" flag
    int  cls{1};         ///< GT class: 1 = pedestrian
    Real visibility{1.0};

    /// Bottom-centre: the conventional ground-contact proxy.
    [[nodiscard]] Vec2 foot() const { return Vec2{x + w * 0.5, y + h}; }
};

/// A loaded MOTChallenge sequence.
struct MotSequence {
    std::string name;
    int frame_rate{30};
    int length{0};
    int width{1920};
    int height{1080};

    /// Public detections, indexed by frame (1-based, as MOT numbers them).
    std::map<int, std::vector<MotBox>> detections;
    /// Ground truth, indexed by frame.
    std::map<int, std::vector<MotBox>> truth;

    [[nodiscard]] bool valid() const { return length > 0 && !detections.empty(); }
    [[nodiscard]] Area area() const {
        // Pad the frame: a person's feet can sit slightly outside the image, and
        // a track approaching the edge should not be clipped by the area gate.
        return Area{-0.2 * width, 1.2 * width, -0.2 * height, 1.2 * height};
    }

    /// Detections for one frame, as engine observations.
    ///
    /// `min_score` rejects the weakest detections. MOT's three public detectors
    /// (DPM, FRCNN, SDP) use wildly different score ranges, so the threshold is
    /// applied after normalising each sequence's own score distribution.
    [[nodiscard]] std::vector<Observation> observations_for(
        int frame, Real timestamp, Real min_score = 0.0) const;

    /// Ground-truth entities for one frame, filtered to real pedestrians.
    [[nodiscard]] std::vector<Entity> truth_for(int frame) const;

    /// Percentile of the detection score distribution, for thresholding.
    [[nodiscard]] Real score_percentile(Real q) const;
};

/// Load a sequence directory (one containing det/, gt/ and seqinfo.ini).
/// Returns an invalid sequence if the layout is not there.
MotSequence load_mot_sequence(const std::string& directory);

/// List sequence directories under a MOT split directory (e.g. .../train).
std::vector<std::string> find_mot_sequences(const std::string& root);

/// A profile tuned for pedestrian tracking in image pixels at `frame_rate`.
///
/// Everything is in pixels rather than metres, which the engine does not care
/// about - it only needs the units to be consistent. Speeds come from what
/// pedestrians actually do in 1080p footage.
DomainProfile MotPedestrianPixels(int frame_rate, Real typical_px_per_s = 90.0);

}  // namespace trace::sim
