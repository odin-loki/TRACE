#include "trace/sim/mot.hpp"

#include <algorithm>
#include <set>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace trace::sim {
namespace {

namespace fs = std::filesystem;

/// MOT files are comma-separated with no header and a variable column count.
std::vector<Real> parse_csv_line(const std::string& line) {
    std::vector<Real> out;
    const char* p = line.c_str();
    char* end = nullptr;
    while (*p != '\0') {
        const Real v = std::strtod(p, &end);
        if (end == p) break;
        out.push_back(v);
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\r') ++p;
    }
    return out;
}

std::map<int, std::vector<MotBox>> load_boxes(
    const fs::path& file, bool is_gt,
    std::map<int, std::vector<MotBox>>* ignore = nullptr) {
    std::map<int, std::vector<MotBox>> out;
    std::ifstream in(file);
    if (!in) return out;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto f = parse_csv_line(line);
        if (f.size() < 6) continue;

        MotBox b;
        b.frame = static_cast<int>(f[0]);
        b.id = static_cast<int>(f[1]);
        b.x = f[2];
        b.y = f[3];
        b.w = f[4];
        b.h = f[5];
        b.score = f.size() > 6 ? f[6] : 1.0;
        b.cls = f.size() > 7 ? static_cast<int>(f[7]) : 1;
        b.visibility = f.size() > 8 ? f[8] : 1.0;

        if (is_gt && (b.score < 0.5 || b.cls != 1)) {
            // Not ground truth, but not nothing either: see MotSequence::ignore.
            // Routed to the caller's second map rather than discarded, so the
            // scorer can decline to charge a track that sits on one.
            if (ignore != nullptr) (*ignore)[b.frame].push_back(b);
            continue;
        }
        out[b.frame].push_back(b);
    }
    return out;
}

std::string read_ini_value(const fs::path& file, const std::string& key) {
    std::ifstream in(file);
    if (!in) return {};
    std::string line;
    while (std::getline(in, line)) {
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        if (line.substr(0, eq) != key) continue;
        std::string v = line.substr(eq + 1);
        while (!v.empty() && (v.back() == '\r' || v.back() == '\n' || v.back() == ' ')) {
            v.pop_back();
        }
        return v;
    }
    return {};
}

}  // namespace

Descriptor MotBox::geometry_descriptor() const {
    if (w <= 0.0 || h <= 0.0) return Descriptor{};
    // Log scale: a 20-pixel difference means something quite different on a
    // 40-pixel box than on a 400-pixel one.
    const Real log_h = std::log(std::max(h, 1.0));
    const Real log_aspect = std::log(std::max(w, 1.0) / std::max(h, 1.0));
    return SoftBinner::encode({
        {log_h,      std::log(20.0),  std::log(600.0), 20, 1.2},
        {log_aspect, std::log(0.15),  std::log(1.5),   12, 1.2},
    });
}

Descriptor MotSequence::identity_descriptor(int gt_id) {
    // Spread identities around a circle in a two-feature soft-binned space, so
    // distinct ids give near-orthogonal descriptors and the same id always
    // gives the same one.
    const Real a = static_cast<Real>((gt_id * 2654435761u) % 10000) / 10000.0;
    const Real b = static_cast<Real>((gt_id * 40503u) % 10000) / 10000.0;
    return SoftBinner::encode({{a, 0.0, 1.0, 16, 0.6}, {b, 0.0, 1.0, 16, 0.6}});
}

const MotBox* MotSequence::nearest_truth(int frame, Vec2 foot,
                                         Real max_distance) const {
    const auto it = truth.find(frame);
    if (it == truth.end()) return nullptr;
    const MotBox* best = nullptr;
    Real best_d = max_distance;
    for (const auto& b : it->second) {
        const Real d = distance(b.foot(), foot);
        if (d < best_d) {
            best_d = d;
            best = &b;
        }
    }
    return best;
}

std::vector<Observation> MotSequence::observations_for(int frame, Real timestamp,
                                                       Real min_score,
                                                       Appearance appearance) const {
    std::vector<Observation> out;
    const auto it = detections.find(frame);
    if (it == detections.end()) return out;

    // Normalise this sequence's score range onto 0..1, against the bounds
    // measured from this sequence at load.
    const Real span = std::max(score_hi - score_lo, 1e-6);
    const Real lo = score_lo;

    int n = 0;
    for (const auto& b : it->second) {
        // An absent score is not a low score. Where the column is a validity
        // flag rather than a confidence, deriving confidence from it asserts
        // something the file never said - and it asserted the worst case:
        // every MOT20 detection came through at the 0.3 confidence floor,
        // which after the modality weight is 0.285 against a birth threshold
        // of 0.25. The entire sequence balanced on that 0.035, and any
        // credibility multiplier below 0.877 shut track birth off outright.
        const Real norm =
            scores_informative ? std::clamp((b.score - lo) / span, 0.0, 1.0) : 1.0;
        if (scores_informative && norm < min_score) continue;
        // Confidence floors at 0.3: a detection that survives the threshold is
        // still worth something, and zero-confidence observations are ignored
        // by the birth gate entirely.
        const Real conf = std::clamp(0.3 + 0.65 * norm, 0.1, 0.99);
        Observation obs("d" + std::to_string(frame) + "_" + std::to_string(n++),
                        timestamp, b.foot(), Modality::GEOINT, conf, "MOT_DET");
        switch (appearance) {
            case Appearance::Geometry:
                obs.with_descriptor(b.geometry_descriptor());
                break;
            case Appearance::Oracle: {
                // A false positive belongs to nobody, so it gets no descriptor
                // rather than a fabricated one - otherwise the study would be
                // measuring clutter suppression as well as re-identification.
                if (const MotBox* gt = nearest_truth(frame, b.foot(), 50.0)) {
                    obs.with_descriptor(identity_descriptor(gt->id));
                }
                break;
            }
            case Appearance::None:
                break;
        }
        out.push_back(std::move(obs));
    }
    return out;
}

std::vector<Entity> MotSequence::truth_for(int frame) const {
    std::vector<Entity> out;
    const auto it = truth.find(frame);
    if (it == truth.end()) return out;
    for (const auto& b : it->second) {
        Entity e;
        e.id = "gt" + std::to_string(b.id);
        e.position = b.foot();
        e.active = true;
        out.push_back(std::move(e));
    }
    return out;
}

std::size_t MotSequence::distinct_scores() const {
    std::set<Real> values;
    for (const auto& [f, boxes] : detections) {
        for (const auto& b : boxes) {
            values.insert(b.score);
            if (values.size() > 2) return values.size();   // enough to decide
        }
    }
    return values.size();
}

Real MotSequence::score_percentile(Real q) const {
    std::vector<Real> sorted;
    for (const auto& [f, boxes] : detections) {
        for (const auto& b : boxes) sorted.push_back(b.score);
    }
    std::sort(sorted.begin(), sorted.end());
    if (sorted.empty()) return 0.0;
    const auto idx = static_cast<std::size_t>(
        std::clamp(q * static_cast<Real>(sorted.size() - 1), 0.0,
                   static_cast<Real>(sorted.size() - 1)));
    return sorted[idx];
}

MotSequence load_mot_sequence(const std::string& directory) {
    MotSequence seq;
    const fs::path dir(directory);
    if (!fs::is_directory(dir)) return seq;

    seq.name = dir.filename().string();
    seq.detections = load_boxes(dir / "det" / "det.txt", false);
    seq.truth = load_boxes(dir / "gt" / "gt.txt", true, &seq.ignore);

    const fs::path info = dir / "seqinfo.ini";
    const auto as_int = [&](const std::string& key, int fallback) {
        const std::string v = read_ini_value(info, key);
        return v.empty() ? fallback : std::atoi(v.c_str());
    };
    seq.frame_rate = as_int("frameRate", 30);
    seq.length = as_int("seqLength", 0);
    seq.width = as_int("imWidth", 1920);
    seq.height = as_int("imHeight", 1080);

    if (seq.length == 0 && !seq.detections.empty()) {
        seq.length = seq.detections.rbegin()->first;
    }

    // Measure the score distribution once, here, where the sequence's own
    // detections are the only ones in scope.
    seq.score_lo = seq.score_percentile(0.02);
    seq.score_hi = seq.score_percentile(0.98);
    seq.scores_informative =
        seq.distinct_scores() > 2 && (seq.score_hi - seq.score_lo) > 1e-6;
    return seq;
}

std::vector<std::string> find_mot_sequences(const std::string& root) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(root, ec)) return out;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (!entry.is_directory()) continue;
        if (fs::exists(entry.path() / "det" / "det.txt")) {
            out.push_back(entry.path().string());
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

DomainProfile MotPedestrianPixels(int frame_rate, Real typical_px_per_s) {
    DomainProfile p;
    p.name = "MotPedestrianPixels";
    p.scan_dt_s = 1.0 / std::max(frame_rate, 1);

    // Public detections are noisy in position as well as existence; the box
    // bottom-centre wobbles by several pixels frame to frame even on a
    // stationary person.
    p.pos_noise_m = 6.0;
    p.meas_noise_var = 36.0 * 4.0;

    // Public detectors miss a great deal, especially under occlusion.
    p.p_detection = 0.55;
    p.r_birth = 0.40;
    p.r_confirm = 0.55;
    p.dormant_timeout = static_cast<int>(frame_rate * 4);
    // How long a window is safe depends entirely on whether there is anything
    // to break the tie. Without a descriptor, a crowd offers dozens of
    // plausible reappearances and the honest setting is to reacquire only
    // across the briefest occlusions. With one, a longer window becomes an
    // asset rather than a liability - see docs/VALIDATION.md.
    p.reacquire_kinematic_s = 1.0;
    p.reacquire_min_similarity = 0.35;
    p.max_coast_s = 1.0;               // one second of occlusion, then retire

    p.gibbs_sweeps = 10;
    p.n_particles = 192;              // crowds mean many filters at once
    p.max_tracks = 400;               // MOT20 puts 200+ people in one frame

    //                       name        holds heading (s)   speed (px/s)
    p.mou_models = {{motion("standing",           1.0, typical_px_per_s * 0.06),
                     motion("walking",            2.5, typical_px_per_s),
                     motion("hurrying",           2.0, typical_px_per_s * 2.2),
                     motion("drifting",           5.0, typical_px_per_s * 0.4)}};
    p.model_trans = {{{{0.80, 0.14, 0.02, 0.04}},
                      {{0.10, 0.80, 0.06, 0.04}},
                      {{0.03, 0.22, 0.70, 0.05}},
                      {{0.10, 0.20, 0.03, 0.67}}}};

    // Pixel-space equivalents of the behavioural thresholds.
    p.rv_threshold_m = 60.0;
    p.rv_warning_horizon_s = 3.0;
    p.brush_pass_m = 40.0;
    p.parallel_route_m = 80.0;
    p.parallel_scans = static_cast<int>(frame_rate * 0.8);
    p.coloc_dist_m = 150.0;
    p.chokepoint_m = 60.0;
    p.courier_speed_thresh = typical_px_per_s * 0.5;
    p.hvl_radius_m = 300.0;
    p.loiter_min_s = 4.0;
    p.sdr_window = static_cast<int>(frame_rate * 2);
    p.pol_min_obs = 40;               // a few seconds before judging "normal"

    // Merge gate scaled to how far apart two people can genuinely be while
    // their boxes still overlap in a crowd.
    p.merge_distance_m = 25.0;

    // Appearance is switched OFF here, and the reason is a measurement rather
    // than an assumption.
    //
    // Box geometry was tried as a descriptor and gave nothing (MOTA 39.7% ->
    // 39.5%). More tellingly, so did a perfect *oracle* descriptor built from
    // ground-truth identity: identity switches moved 1039 -> 1012, and no
    // combination of weight, reacquisition window or dormancy changed that.
    //
    // The arithmetic says why. On MOT17-02-FRCNN the MOTA penalty is 89%
    // missed detections, 9% identity switches, 1.5% false positives. Missed
    // detections are capped by the detector - which TRACE already exceeds by
    // coasting - so appearance can only address a ninth of the penalty, and
    // eliminating every switch would be worth 5.6 MOTA points. The switches
    // that remain are fragmentation: a person undetected for seconds, whose
    // coasted track has drifted too far to be recognised as theirs.
    //
    // The mechanism is implemented, tested and available; on this benchmark it
    // is not what is limiting. A domain where descriptors are genuinely
    // discriminative should turn it on. See docs/VALIDATION.md.
    p.appearance_weight = 0.0;
    p.appearance_sigma = 0.30;
    p.appearance_momentum = 0.85;
    p.reacquire_min_similarity = -1.0;
    return p;
}

}  // namespace trace::sim
