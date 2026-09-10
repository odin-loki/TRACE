// TRACE — MOTChallenge replay.
//
// The only numbers in this repository not produced by TRACE's own simulator.
// Replays real sequences with real public detections and scores with the
// standard CLEAR-MOT measures, so the result is comparable to published work
// rather than self-referential.
//
//   ./trace_mot /path/to/MOT17/train                 # every sequence
//   ./trace_mot /path/to/MOT17/train/MOT17-02-FRCNN  # one sequence
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "trace/core/engine.hpp"
#include "trace/sim/assignment.hpp"
#include "trace/sim/mot.hpp"

using namespace trace;
using namespace trace::sim;

namespace {

/// CLEAR-MOT accumulators.
///
/// MOTA folds misses, false positives and identity switches into one number.
/// It is the standard headline figure and it is also famously dominated by
/// recall, so precision and MOTP are reported alongside rather than hidden.
struct ClearMot {
    long gt_total{0};
    long true_positives{0};
    long false_positives{0};
    long false_negatives{0};
    long id_switches{0};
    Real distance_sum{0.0};

    /// Last track id seen for each ground-truth identity.
    std::map<int, std::string> last_match;
    /// Frames each ground-truth identity was present / matched.
    std::map<int, long> gt_frames;
    std::map<int, long> matched_frames;

    Real latency_sum{0.0};
    long frames{0};
    int peak_tracks{0};

    [[nodiscard]] Real mota() const {
        if (gt_total == 0) return 0.0;
        return 1.0 - static_cast<Real>(false_negatives + false_positives + id_switches) /
                         static_cast<Real>(gt_total);
    }
    [[nodiscard]] Real motp() const {
        return true_positives > 0 ? distance_sum / static_cast<Real>(true_positives) : 0.0;
    }
    [[nodiscard]] Real precision() const {
        const long denom = true_positives + false_positives;
        return denom > 0 ? static_cast<Real>(true_positives) / static_cast<Real>(denom) : 0.0;
    }
    [[nodiscard]] Real recall() const {
        return gt_total > 0 ? static_cast<Real>(true_positives) / static_cast<Real>(gt_total) : 0.0;
    }

    /// Mostly-tracked / mostly-lost: fraction of identities followed for more
    /// than 80% / less than 20% of their life. These say something MOTA cannot:
    /// whether the tracker holds an identity or merely finds it repeatedly.
    [[nodiscard]] std::pair<Real, Real> mt_ml() const {
        if (gt_frames.empty()) return {0.0, 0.0};
        long mt = 0, ml = 0;
        for (const auto& [id, total] : gt_frames) {
            const auto it = matched_frames.find(id);
            const Real frac = it == matched_frames.end()
                                  ? 0.0
                                  : static_cast<Real>(it->second) / static_cast<Real>(total);
            if (frac >= 0.8) ++mt;
            if (frac < 0.2) ++ml;
        }
        const auto n = static_cast<Real>(gt_frames.size());
        return {static_cast<Real>(mt) / n, static_cast<Real>(ml) / n};
    }
};

void accumulate(ClearMot& m, const std::vector<MotBox>& gt,
                const std::vector<TargetReport>& tracks, Real match_radius) {
    std::vector<Vec2> gt_pts;
    gt_pts.reserve(gt.size());
    for (const auto& b : gt) gt_pts.push_back(b.foot());

    std::vector<Vec2> tr_pts;
    tr_pts.reserve(tracks.size());
    for (const auto& t : tracks) tr_pts.push_back(t.position);

    const Assignment a = match_points(gt_pts, tr_pts, match_radius);

    m.gt_total += static_cast<long>(gt.size());
    for (const auto& b : gt) ++m.gt_frames[b.id];

    for (std::size_t i = 0; i < gt.size(); ++i) {
        const int j = a.row_to_col[i];
        if (j < 0) {
            ++m.false_negatives;
            continue;
        }
        const auto& track = tracks[static_cast<std::size_t>(j)];
        ++m.true_positives;
        ++m.matched_frames[gt[i].id];
        m.distance_sum += distance(track.position, gt[i].foot());

        const auto it = m.last_match.find(gt[i].id);
        if (it == m.last_match.end()) {
            m.last_match[gt[i].id] = track.track_id;
        } else if (it->second != track.track_id) {
            ++m.id_switches;
            it->second = track.track_id;
        }
    }

    for (std::size_t j = 0; j < tracks.size(); ++j) {
        if (a.col_to_row[j] < 0) ++m.false_positives;
    }
}

void print_result(const std::string& name, const ClearMot& m) {
    const auto [mt, ml] = m.mt_ml();
    std::printf(
        "  %-22s MOTA %6.1f%%  MOTP %5.1f px  Rcll %5.1f%%  Prcn %5.1f%%  "
        "IDs %5ld  MT %4.1f%%  ML %4.1f%%  %5.2f ms/frame\n",
        name.c_str(), 100.0 * m.mota(), m.motp(), 100.0 * m.recall(),
        100.0 * m.precision(), m.id_switches, 100.0 * mt, 100.0 * ml,
        m.frames > 0 ? m.latency_sum / static_cast<Real>(m.frames) : 0.0);
}

/// What the input allows.
///
/// Score the raw detections directly against ground truth, as if a perfect
/// tracker simply reported every detection it was handed. No tracker consuming
/// these detections can exceed this recall, so it is the honest denominator for
/// judging the one that does. MOT's public detections are deliberately weak -
/// on MOT17-02 they supply about 14 boxes per frame against 31 real people -
/// and quoting a tracker's recall without this number invites a comparison
/// against detectors, not trackers.
struct DetectorCeiling {
    long gt_total{0};
    long matched{0};
    long detections{0};
    [[nodiscard]] Real recall() const {
        return gt_total > 0 ? static_cast<Real>(matched) / static_cast<Real>(gt_total) : 0.0;
    }
    [[nodiscard]] Real precision() const {
        return detections > 0 ? static_cast<Real>(matched) / static_cast<Real>(detections) : 0.0;
    }
};

DetectorCeiling detector_ceiling(const MotSequence& seq, Real min_score,
                                 Real match_radius) {
    DetectorCeiling c;
    for (int frame = 1; frame <= seq.length; ++frame) {
        const auto gt_it = seq.truth.find(frame);
        if (gt_it == seq.truth.end()) continue;

        const auto obs = seq.observations_for(frame, 0.0, min_score);
        std::vector<Vec2> gt_pts;
        for (const auto& b : gt_it->second) gt_pts.push_back(b.foot());
        std::vector<Vec2> det_pts;
        for (const auto& o : obs) det_pts.push_back(*o.position);

        const Assignment a = match_points(gt_pts, det_pts, match_radius);
        c.gt_total += static_cast<long>(gt_pts.size());
        c.detections += static_cast<long>(det_pts.size());
        c.matched += static_cast<long>(a.n_matched);
    }
    return c;
}

ClearMot run_sequence(const std::string& dir, Real min_score, Real match_radius,
                      bool verbose) {
    ClearMot m;
    const MotSequence seq = load_mot_sequence(dir);
    if (!seq.valid()) {
        std::printf("  %-22s (no det/gt found)\n",
                    dir.substr(dir.find_last_of('/') + 1).c_str());
        return m;
    }

    EngineConfig cfg;
    cfg.profile = MotPedestrianPixels(seq.frame_rate);
    cfg.area = seq.area();
    cfg.seed = 20260910;
    Engine engine(cfg);

    const Real dt = 1.0 / std::max(seq.frame_rate, 1);
    for (int frame = 1; frame <= seq.length; ++frame) {
        const Real t = static_cast<Real>(frame - 1) * dt;
        const auto obs = seq.observations_for(frame, t, min_score);
        const ScanReport r = engine.ingest(obs, t);

        m.latency_sum += r.latency_ms;
        ++m.frames;
        m.peak_tracks = std::max(m.peak_tracks, r.n_tracks);

        const auto gt_it = seq.truth.find(frame);
        if (gt_it != seq.truth.end()) {
            accumulate(m, gt_it->second, r.targets, match_radius);
        }

        if (verbose && frame % 200 == 0) {
            std::printf("    frame %4d/%d  obs %3zu  tracks %3d  %.2f ms\n", frame,
                        seq.length, obs.size(), r.n_tracks, r.latency_ms);
        }
    }
    return m;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts(
            "usage: trace_mot <MOT sequence or split directory> [options]\n"
            "\n"
            "  --min-score F   drop detections below this normalised score (default 0.15)\n"
            "  --radius PX     match radius in pixels                      (default 100)\n"
            "  --verbose       per-frame progress\n"
            "\n"
            "Fetch the label archives with scripts/fetch_mot.sh - only the\n"
            "annotations are needed, not the images.");
        return 1;
    }

    const std::string root = argv[1];
    Real min_score = 0.15;
    Real radius = 100.0;
    bool verbose = false;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--min-score") == 0 && i + 1 < argc) {
            min_score = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--radius") == 0 && i + 1 < argc) {
            radius = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        }
    }

    auto sequences = find_mot_sequences(root);
    if (sequences.empty()) sequences.push_back(root);

    std::printf("TRACE MOTChallenge replay  (min-score %.2f, match radius %.0f px)\n\n",
                min_score, radius);

    ClearMot overall;
    DetectorCeiling ceiling_all;
    for (const auto& dir : sequences) {
        const ClearMot m = run_sequence(dir, min_score, radius, verbose);
        if (m.frames == 0) continue;
        print_result(dir.substr(dir.find_last_of('/') + 1), m);

        const MotSequence seq = load_mot_sequence(dir);
        const DetectorCeiling c = detector_ceiling(seq, min_score, radius);
        ceiling_all.gt_total += c.gt_total;
        ceiling_all.matched += c.matched;
        ceiling_all.detections += c.detections;

        overall.gt_total += m.gt_total;
        overall.true_positives += m.true_positives;
        overall.false_positives += m.false_positives;
        overall.false_negatives += m.false_negatives;
        overall.id_switches += m.id_switches;
        overall.distance_sum += m.distance_sum;
        overall.latency_sum += m.latency_sum;
        overall.frames += m.frames;
        for (const auto& [id, n] : m.gt_frames) overall.gt_frames[id] += n;
        for (const auto& [id, n] : m.matched_frames) overall.matched_frames[id] += n;
    }

    if (overall.frames > 0) {
        std::printf("\n");
        print_result("OVERALL", overall);
        std::printf("\n  %ld frames, %ld ground-truth boxes\n", overall.frames,
                    overall.gt_total);

        // The input's own limit, and how much of it the tracker recovered.
        std::printf("\n  detector ceiling       Rcll %5.1f%%  Prcn %5.1f%%  "
                    "(%ld detections for %ld ground-truth boxes)\n",
                    100.0 * ceiling_all.recall(), 100.0 * ceiling_all.precision(),
                    ceiling_all.detections, ceiling_all.gt_total);
        if (ceiling_all.recall() > 0.0) {
            std::printf("  TRACE recovered %.1f%% of the recall the detections allow\n",
                        100.0 * overall.recall() / ceiling_all.recall());
        }
    }
    return 0;
}
