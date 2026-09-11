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
#include <unordered_map>
#include <set>
#include <string>
#include <vector>

#include "trace/core/engine.hpp"
#include "trace/core/assignment.hpp"
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

    // CLEAR-MOT matches in two passes, and the order is the whole point.
    //
    // First, every correspondence from the previous frame that is STILL valid
    // is kept: if ground-truth g was matched to hypothesis h last frame and h
    // is still within the radius of g now, that pairing stands. Only what is
    // left over goes to the optimal matcher.
    //
    // Without the first pass a fresh optimum is computed every frame, and two
    // hypotheses that fit two ground-truth identities about equally well are
    // free to swap between them whenever the arithmetic tips - which scores
    // two identity switches for a scene in which nothing happened. The metric
    // is supposed to count the tracker changing its mind, not the scorer
    // changing its mind. Bernardin & Stiefelhagen specify the continuity pass
    // for exactly this reason.
    std::vector<int> gt_to_track(gt.size(), -1);
    std::vector<char> gt_taken(gt.size(), 0), tr_taken(tracks.size(), 0);

    std::unordered_map<std::string, std::size_t> by_id;
    by_id.reserve(tracks.size());
    for (std::size_t j = 0; j < tracks.size(); ++j) by_id[tracks[j].track_id] = j;

    for (std::size_t i = 0; i < gt.size(); ++i) {
        const auto prev = m.last_match.find(gt[i].id);
        if (prev == m.last_match.end()) continue;
        const auto hit = by_id.find(prev->second);
        if (hit == by_id.end() || tr_taken[hit->second]) continue;
        if (distance(tracks[hit->second].position, gt[i].foot()) > match_radius) continue;
        gt_to_track[i] = static_cast<int>(hit->second);
        gt_taken[i] = 1;
        tr_taken[hit->second] = 1;
    }

    // Second pass: optimal matching over whatever the first pass left free.
    std::vector<std::size_t> free_gt, free_tr;
    std::vector<Vec2> free_gt_pts, free_tr_pts;
    for (std::size_t i = 0; i < gt.size(); ++i) {
        if (!gt_taken[i]) { free_gt.push_back(i); free_gt_pts.push_back(gt_pts[i]); }
    }
    for (std::size_t j = 0; j < tracks.size(); ++j) {
        if (!tr_taken[j]) { free_tr.push_back(j); free_tr_pts.push_back(tr_pts[j]); }
    }
    const Assignment a = match_points(free_gt_pts, free_tr_pts, match_radius);
    for (std::size_t k = 0; k < free_gt.size(); ++k) {
        const int c = a.row_to_col[k];
        if (c >= 0) gt_to_track[free_gt[k]] = static_cast<int>(free_tr[static_cast<std::size_t>(c)]);
    }

    m.gt_total += static_cast<long>(gt.size());
    for (const auto& b : gt) ++m.gt_frames[b.id];

    for (std::size_t i = 0; i < gt.size(); ++i) {
        const int j = gt_to_track[i];
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

    std::vector<char> matched_track(tracks.size(), 0);
    for (std::size_t i = 0; i < gt.size(); ++i) {
        if (gt_to_track[i] >= 0) matched_track[static_cast<std::size_t>(gt_to_track[i])] = 1;
    }
    for (std::size_t j = 0; j < tracks.size(); ++j) {
        if (!matched_track[j]) ++m.false_positives;
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
                      bool verbose, MotSequence::Appearance appearance,
                      Real appearance_weight, bool adaptive_noise) {
    ClearMot m;
    const MotSequence seq = load_mot_sequence(dir);
    if (!seq.valid()) {
        std::printf("  %-22s (no det/gt found)\n",
                    dir.substr(dir.find_last_of('/') + 1).c_str());
        return m;
    }

    EngineConfig cfg;
    cfg.profile = MotPedestrianPixels(seq.frame_rate);
    cfg.profile.appearance_weight = appearance_weight;
    cfg.profile.adaptive_meas_noise = adaptive_noise;
    cfg.area = seq.area();
    cfg.seed = 20260910;
    Engine engine(cfg);

    const Real dt = 1.0 / std::max(seq.frame_rate, 1);
    for (int frame = 1; frame <= seq.length; ++frame) {
        const Real t = static_cast<Real>(frame - 1) * dt;
        const auto obs = seq.observations_for(frame, t, min_score, appearance);
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
            "  --adaptive-noise  learn each sensor's measurement noise from residuals\n"
            "  --radius PX     match radius in pixels                      (default 100)\n"
            "  --verbose       per-frame progress\n"
            "  --appearance M  none | geometry | oracle   (default geometry)\n"
            "                  oracle is an upper-bound study: it hands the\n"
            "                  tracker perfect identity evidence, which no real\n"
            "                  system has. The gap to geometry is the headroom a\n"
            "                  learned re-identification embedding would unlock.\n"
            "  --appearance-weight F   override the profile's weight\n"
            "\n"
            "Fetch the label archives with scripts/fetch_mot.sh - only the\n"
            "annotations are needed, not the images.");
        return 1;
    }

    const std::string root = argv[1];
    Real min_score = 0.15;
    bool adaptive_noise = false;
    Real radius = 100.0;
    bool verbose = false;
    auto appearance = MotSequence::Appearance::Geometry;
    Real appearance_weight = -1.0;  // negative: use the profile's own default
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--adaptive-noise") == 0) {
            adaptive_noise = true;
        } else if (std::strcmp(argv[i], "--min-score") == 0 && i + 1 < argc) {
            min_score = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--radius") == 0 && i + 1 < argc) {
            radius = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (std::strcmp(argv[i], "--appearance") == 0 && i + 1 < argc) {
            const std::string mode = argv[++i];
            if (mode == "none") appearance = MotSequence::Appearance::None;
            else if (mode == "oracle") appearance = MotSequence::Appearance::Oracle;
            else appearance = MotSequence::Appearance::Geometry;
        } else if (std::strcmp(argv[i], "--appearance-weight") == 0 && i + 1 < argc) {
            appearance_weight = std::atof(argv[++i]);
        }
    }

    auto sequences = find_mot_sequences(root);
    if (sequences.empty()) sequences.push_back(root);

    std::printf("TRACE MOTChallenge replay  (min-score %.2f, match radius %.0f px)\n\n",
                min_score, radius);

    ClearMot overall;
    DetectorCeiling ceiling_all;
    for (const auto& dir : sequences) {
        const Real w = appearance_weight >= 0.0
                           ? appearance_weight
                           : MotPedestrianPixels(30).appearance_weight;
        const ClearMot m = run_sequence(
            dir, min_score, radius, verbose,
            appearance == MotSequence::Appearance::None ? appearance : appearance,
            appearance == MotSequence::Appearance::None ? 0.0 : w, adaptive_noise);
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
