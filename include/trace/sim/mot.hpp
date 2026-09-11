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

#include "trace/core/descriptor.hpp"
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

    /// A descriptor built from the box's own geometry.
    ///
    /// TRACE has no access to pixels here, but a detection box is not nothing:
    /// height is a strong depth cue and is stable per person over short
    /// intervals, and aspect ratio separates a standing adult from a crouching
    /// one or a pushchair. It is far weaker than a learned re-identification
    /// embedding - it cannot tell two similarly-sized strangers apart - but it
    /// is honest evidence obtainable from public detections alone, and it
    /// exercises exactly the code path a real embedding would use.
    [[nodiscard]] Descriptor geometry_descriptor() const;
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

    /// Robust bounds of this sequence's own detection-score distribution,
    /// computed once at load. Detectors do not share a scale - DPM emits
    /// roughly -1..+3, FRCNN and SDP quite different ranges again, and MOT20's
    /// differ from all of them - so a score threshold only means anything
    /// after normalising against the sequence that produced it.
    Real score_lo{0.0};
    Real score_hi{1.0};

    /// Whether the score column carries usable information at all. MOT20's
    /// public detections ship with it unset - 175,303 of MOT20-03's 177,347
    /// rows are scored exactly 0 - so its 2nd and 98th percentiles coincide
    /// and every detection normalises to the same value. A score threshold
    /// against that scale is not a strict filter, it is an arbitrary one, and
    /// at the default it discarded 96% of MOT20's detections.
    bool scores_informative{true};

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
    /// How detections should be tagged with appearance evidence.
    enum class Appearance {
        None,      ///< no descriptor at all
        Geometry,  ///< from the detection box's own width and height
        Oracle,    ///< from the ground-truth identity the box belongs to
    };

    /// `Appearance::Oracle` is a deliberate upper-bound study, not a result. It
    /// hands the tracker a perfect, noiseless identity descriptor by looking up
    /// which real person each detection belongs to. No real system can do this.
    /// Its purpose is to measure precisely how much a learned re-identification
    /// embedding would be worth, without having to train one - the gap between
    /// Geometry and Oracle is the headroom.
    [[nodiscard]] std::vector<Observation> observations_for(
        int frame, Real timestamp, Real min_score = 0.0,
        Appearance appearance = Appearance::Geometry) const;

    /// Descriptor encoding a ground-truth identity, for the oracle study.
    [[nodiscard]] static Descriptor identity_descriptor(int gt_id);

    /// The ground-truth box nearest to a detection, or nullptr.
    [[nodiscard]] const MotBox* nearest_truth(int frame, Vec2 foot,
                                              Real max_distance) const;

    /// Ground-truth entities for one frame, filtered to real pedestrians.
    [[nodiscard]] std::vector<Entity> truth_for(int frame) const;

    /// Percentile of the detection score distribution, for thresholding.
    /// Quantile of the loaded detection scores. Used at load time to fill
    /// `score_lo` / `score_hi`; the result was previously memoised in a
    /// function-local static keyed on `this`, which is not an identity -
    /// sequences are replayed one at a time from the same stack slot, so every
    /// sequence after the first silently normalised against the first one's
    /// score distribution.
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
