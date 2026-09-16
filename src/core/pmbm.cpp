#include "trace/core/pmbm.hpp"

#include "trace/core/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <map>
#include <numbers>
#include <tuple>
#include <unordered_set>

namespace trace {
namespace {

constexpr std::size_t kClutterWindow = 20;
constexpr Real kCredDecay = 0.98;
constexpr Real kCredGood = 0.95;
constexpr Real kCredBad = 0.10;
constexpr Real kCredDefault = 0.80;
constexpr Real kCredLoglikThreshold = -5.0;

constexpr Real kGibbsClutterLoglik = -11.5129;  // log(1e-5)

constexpr Real kMinBirthWeight = 0.25;

/// Group-spawn heuristics: a track whose particles disagree sharply about
/// velocity while still being fed a steady stream of detections is probably
/// two entities travelling together, not one.
constexpr Real kGroupSpawnRadius = 80.0;
constexpr Real kGroupMeasRateThresh = 0.85;
/// Velocity dispersion across the particle cloud, in (m/s)^2. Expressed as a
/// multiple of the domain's speed scale so it travels between profiles.
constexpr Real kGroupVelVarScale = 1.5;
constexpr Real kProximityParentRadius = 60.0;

Real log_sum_exp(const std::vector<Real>& v) {
    if (v.empty()) return -std::numeric_limits<Real>::infinity();
    const Real m = *std::max_element(v.begin(), v.end());
    if (!std::isfinite(m)) return m;
    Real acc = 0.0;
    for (const Real x : v) acc += std::exp(x - m);
    return m + std::log(acc);
}

}  // namespace

// ---------------------------------------------------------------------------
// ClutterEstimator
// ---------------------------------------------------------------------------

void ClutterEstimator::update(int n_unassigned) {
    window_.push_back(std::max(0, n_unassigned));
    if (window_.size() > kClutterWindow) window_.pop_front();
    const Real total = std::accumulate(window_.begin(), window_.end(), Real{0.0});
    alpha_ = 3.0 + total;
    beta_ = 1.0 + static_cast<Real>(window_.size());
}

// ---------------------------------------------------------------------------
// DetectionRateEstimator
// ---------------------------------------------------------------------------

namespace {

constexpr Real kPdDecay = 0.98;
constexpr int kPdMinSamples = 40;
/// A sensor that fed a track this recently is assumed to still cover it. Long
/// enough to survive an ordinary run of misses, short enough that an entity
/// which has genuinely walked out of shot stops counting against the sensor.
constexpr int kFeederWindow = 10;
/// The estimate never says a sensor is hopeless: that would make every miss
/// uninformative and every track immortal.
constexpr Real kPdFloor = 0.05;
constexpr Real kPdCeil = 0.99;

}  // namespace

void DetectionRateEstimator::record(const std::string& source_id, bool detected) {
    if (source_id.empty()) return;
    auto& st = by_source_[source_id];
    const Real x = detected ? 1.0 : 0.0;
    st.ema = st.ema < 0.0 ? x : kPdDecay * st.ema + (1.0 - kPdDecay) * x;
    ++st.samples;
}

Real DetectionRateEstimator::rate(const std::string& source_id,
                                  Real fallback) const {
    const auto it = by_source_.find(source_id);
    if (it == by_source_.end() || it->second.samples < kPdMinSamples ||
        it->second.ema < 0.0) {
        return fallback;
    }
    return std::clamp(it->second.ema, kPdFloor, kPdCeil);
}

std::vector<std::pair<std::string, Real>> DetectionRateEstimator::rates() const {
    std::vector<std::pair<std::string, Real>> out;
    for (const auto& [id, st] : by_source_) {
        if (st.samples < kPdMinSamples || st.ema < 0.0) continue;
        out.emplace_back(id, std::clamp(st.ema, kPdFloor, kPdCeil));
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ---------------------------------------------------------------------------
// MeasurementNoiseEstimator
// ---------------------------------------------------------------------------

namespace {

/// Expected normalised innovation squared for a consistent 2-D filter.
constexpr Real kNisTarget = 2.0;
constexpr Real kNisDecay = 0.97;
/// One gross outlier - a detection that belongs to something else entirely -
/// must not move the estimate far, so a single sample is capped.
constexpr Real kNisSampleCap = kNisTarget * 25.0;
/// Enough samples that the mean means something before anything is changed.
constexpr int kNisMinSamples = 40;
constexpr Real kNoiseScaleMin = 0.25;
constexpr Real kNoiseScaleMax = 16.0;

}  // namespace

void MeasurementNoiseEstimator::observe(const std::string& source_id, Real nis) {
    if (source_id.empty() || !std::isfinite(nis) || nis < 0.0) return;
    auto& st = by_source_[source_id];
    st.mean_nis = kNisDecay * st.mean_nis +
                  (1.0 - kNisDecay) * std::min(nis, kNisSampleCap);
    ++st.samples;
    if (st.samples < kNisMinSamples) return;

    // The correction is MULTIPLICATIVE on the scale already in force, and that
    // is the whole of it.
    //
    // The NIS arriving here was measured against an innovation covariance that
    // already carries `st.scale`. If the source's true variance is k times the
    // profile's assertion, then under an applied scale s the expected NIS is
    // 2k/s rather than 2k. Setting the scale to mean_nis/target - reading the
    // symptom as the answer - therefore solves s = k/s, and the estimator
    // settles at the SQUARE ROOT of the ratio it is trying to find. Measured
    // across true variance ratios of 2.25 to 9, the fitted exponent was 0.51.
    // A source genuinely four times noisier than claimed had its assumed
    // variance widened 1.66-fold, and one 25 times noisier, 2.37-fold.
    //
    // Correcting the applied scale instead solves mean_nis = target, whose
    // fixed point is s = k: the definition of a consistent filter, and what
    // the target was chosen to express.
    //
    // Damped at the same rate as the NIS average it is driven by. The two are
    // a coupled pair - the scale moves the NIS that moves the scale - and
    // stepping straight to the implied value would ring.
    const Real implied = st.scale * st.mean_nis / kNisTarget;
    st.scale = std::clamp(kNisDecay * st.scale + (1.0 - kNisDecay) * implied,
                          kNoiseScaleMin, kNoiseScaleMax);
}

Real MeasurementNoiseEstimator::scale(const std::string& source_id) const {
    const auto it = by_source_.find(source_id);
    if (it == by_source_.end() || it->second.samples < kNisMinSamples) return 1.0;
    return it->second.scale;
}

std::vector<std::pair<std::string, Real>> MeasurementNoiseEstimator::scales() const {
    std::vector<std::pair<std::string, Real>> out;
    for (const auto& [id, st] : by_source_) {
        if (st.samples < kNisMinSamples) continue;
        out.emplace_back(id, st.scale);
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ---------------------------------------------------------------------------
// SourceCredibility
// ---------------------------------------------------------------------------

void SourceCredibility::update(const std::string& source_id, Real obs_loglik,
                               Real threshold) {
    if (source_id.empty()) return;
    auto [it, inserted] = scores_.try_emplace(source_id, kCredDefault);
    const Real target = obs_loglik > threshold ? kCredGood : kCredBad;
    it->second = kCredDecay * it->second + (1.0 - kCredDecay) * target;
}

void SourceCredibility::record_pairwise_conflict(const std::string& a,
                                                 const std::string& b,
                                                 Real disagreement_m,
                                                 Real expected_m) {
    if (a.empty() || b.empty() || a == b) return;
    const auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    auto& c = conflicts_[key];
    c.source_a = key.first;
    c.source_b = key.second;
    ++c.observations;
    if (disagreement_m > 3.0 * std::max(expected_m, 1e-6)) ++c.disagreements;
    // Running mean of the disagreement magnitude.
    c.mean_disagreement_m +=
        (disagreement_m - c.mean_disagreement_m) / static_cast<Real>(c.observations);
}

std::vector<SourceCredibility::Conflict> SourceCredibility::conflicts(
    Real min_rate) const {
    std::vector<Conflict> out;
    for (const auto& [key, c] : conflicts_) {
        if (c.observations >= 10 && c.rate() >= min_rate) out.push_back(c);
    }
    std::sort(out.begin(), out.end(), [](const Conflict& x, const Conflict& y) {
        return x.mean_disagreement_m > y.mean_disagreement_m;
    });
    return out;
}

void SourceCredibility::update_against_peers(const std::string& source_id,
                                             Real disagreement_m,
                                             Real expected_m) {
    if (source_id.empty()) return;
    auto [it, inserted] = scores_.try_emplace(source_id, kCredDefault);

    // Independent reports of one entity differ by roughly sqrt(2) sigma. Three
    // times that is well outside noise and indicates a systematic offset.
    const Real ratio = disagreement_m / std::max(expected_m, 1e-6);
    const Real target = ratio < 3.0 ? kCredGood : kCredBad;

    // Peer evidence moves credibility faster than the fit-to-track test,
    // because it is the test that can actually be trusted.
    constexpr Real kPeerDecay = 0.92;
    it->second = kPeerDecay * it->second + (1.0 - kPeerDecay) * target;
}

void SourceCredibility::note_residual(const std::string& source_id,
                                      Vec2 residual, Real pos_noise_m) {
    if (source_id.empty()) return;
    auto& r = residuals_[source_id];
    ++r.n;
    r.noise = pos_noise_m;
    // Running mean of the signed offset. Noise cancels; a bias does not.
    r.mean += (residual - r.mean) / static_cast<Real>(r.n);

    // Standard error of the mean falls as 1/sqrt(n), so a persistent offset
    // becomes significant even when it is small compared with single-shot
    // noise. Below thirty samples the estimate is too loose to act on.
    if (r.n < 30) return;
    const Real stderr_m = std::max(pos_noise_m, 1e-6) /
                          std::sqrt(static_cast<Real>(r.n));
    const Real significance = r.mean.norm() / stderr_m;

    auto [it, inserted] = scores_.try_emplace(source_id, kCredDefault);
    const Real target = significance > 6.0 ? kCredBad : kCredGood;
    constexpr Real kBiasDecay = 0.97;
    it->second = kBiasDecay * it->second + (1.0 - kBiasDecay) * target;
}

std::vector<SourceCredibility::Bias> SourceCredibility::biases(
    Real min_significance) const {
    std::vector<Bias> out;
    for (const auto& [id, r] : residuals_) {
        if (r.n < 30) continue;
        const Real stderr_m = std::max(r.noise, 1e-6) /
                              std::sqrt(static_cast<Real>(r.n));
        const Real significance = r.mean.norm() / stderr_m;
        if (significance < min_significance) continue;
        out.push_back(Bias{id, r.mean, r.mean.norm(), significance, r.n, false});
    }

    // Which direction does the bulk of the evidence point? Weighting by sample
    // count keeps a thinly-observed sensor from casting a large vote.
    Vec2 consensus{};
    for (const auto& b : out) {
        consensus += b.offset_m.unit() * static_cast<Real>(b.samples);
    }
    const Vec2 consensus_dir = consensus.unit();

    for (auto& b : out) {
        // The sound sensors form the majority and all point the same way,
        // because the track has been dragged away from them. The odd one out,
        // pointing back towards where it is pulling, is the biased sensor.
        b.minority_direction = b.offset_m.unit().dot(consensus_dir) < 0.0;
    }

    std::sort(out.begin(), out.end(), [](const Bias& a, const Bias& b) {
        if (a.minority_direction != b.minority_direction) {
            return a.minority_direction;   // suspects first
        }
        return a.significance > b.significance;
    });
    return out;
}

void SourceCredibility::note_assignment(const std::string& source_id,
                                        bool assigned) {
    if (source_id.empty()) return;
    auto& a = assignment_[source_id];
    ++a.total;
    if (!assigned) ++a.unassigned;
}

std::vector<SourceCredibility::Orphaned> SourceCredibility::orphaned_sources(
    Real min_rate) const {
    std::vector<Orphaned> out;
    for (const auto& [id, a] : assignment_) {
        if (a.total < 30) continue;
        const Real rate = static_cast<Real>(a.unassigned) / a.total;
        if (rate >= min_rate) out.push_back(Orphaned{id, rate, a.total});
    }
    std::sort(out.begin(), out.end(), [](const Orphaned& x, const Orphaned& y) {
        return x.unassigned_rate > y.unassigned_rate;
    });
    return out;
}

Vec2 SourceCredibility::mean_residual(const std::string& source_id) const {
    const auto it = residuals_.find(source_id);
    // Below a handful of samples the mean is noise itself, and subtracting it
    // would remove real signal.
    if (it == residuals_.end() || it->second.n < 8) return Vec2{0.0, 0.0};
    return it->second.mean;
}

void SourceCredibility::note_source(const std::string& source_id) {
    if (!source_id.empty()) seen_sources_.insert(source_id);
}

Real SourceCredibility::get(const std::string& source_id) const {
    // No adjustment, rather than the default prior. kCredDefault is 0.80 - a
    // reasonable opening guess about a source that has peers to be compared
    // against, and a flat 20% penalty when it has none. The score multiplies
    // into the birth gate, so that penalty is not cosmetic: it is the
    // difference between a detection founding a track and being discarded.
    if (seen_sources_.size() < 2) return 1.0;
    const auto it = scores_.find(source_id);
    return it != scores_.end() ? it->second : kCredDefault;
}

// ---------------------------------------------------------------------------
// GibbsAssigner
// ---------------------------------------------------------------------------

std::unordered_map<int, int> GibbsAssigner::assign(
    const std::vector<TrackPtr>& tracks,
    const std::vector<const Observation*>& observations,
    const DomainProfile& profile, Rng& rng) const {
    std::unordered_map<int, int> out;
    const int n_t = static_cast<int>(tracks.size());
    const int n_o = static_cast<int>(observations.size());
    if (n_t == 0 || n_o == 0) return out;

    // Gated log-likelihood matrix. Anything outside the chi-square gate is
    // impossible, not merely unlikely, so it never enters the sampler.
    const Real neg_inf = -std::numeric_limits<Real>::infinity();
    std::vector<Real> loglik(static_cast<std::size_t>(n_t) * n_o, neg_inf);

    for (int i = 0; i < n_t; ++i) {
        for (int j = 0; j < n_o; ++j) {
            const Observation& o = *observations[j];
            if (!o.has_position()) continue;
            const Real m2 = tracks[i]->filter().mahalanobis_sq(*o.position);
            if (m2 >= profile.gate_chi2) continue;

            const Real w = profile.modality_weight(o.modality) * o.confidence;
            Real ll = -0.5 * m2 + std::log(w + 1e-300);

            // Appearance, where the sensor supplies it. This is the only term
            // that can separate two entities at the moment their paths cross,
            // which is precisely when position tells you nothing and identity
            // is lost. Treated as a Gaussian on (1 - cosine similarity), so it
            // enters the log-likelihood on the same footing as the Mahalanobis
            // term rather than as an ad-hoc bonus.
            if (profile.appearance_weight > 0.0 && o.has_descriptor()) {
                const Descriptor& track_app = tracks[i]->appearance();
                if (track_app.valid()) {
                    const Real d = 1.0 - track_app.similarity(o.descriptor);
                    const Real sigma = std::max(profile.appearance_sigma, 1e-6);
                    ll -= profile.appearance_weight * 0.5 * (d / sigma) * (d / sigma);
                }
            }

            loglik[static_cast<std::size_t>(i) * n_o + j] = ll;
        }
    }

    // Seed with the greedy solution, then let the sweeps redistribute.
    // Greedy one-to-one seed: best pairs first, each detection claimed once.
    std::vector<int> asgn(static_cast<std::size_t>(n_t), -1);
    std::vector<bool> taken(static_cast<std::size_t>(n_o), false);
    std::vector<std::tuple<Real, int, int>> pairs;
    pairs.reserve(static_cast<std::size_t>(n_t) * n_o);
    for (int i = 0; i < n_t; ++i) {
        for (int j = 0; j < n_o; ++j) {
            const Real v = loglik[static_cast<std::size_t>(i) * n_o + j];
            if (std::isfinite(v)) pairs.emplace_back(v, i, j);
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
    for (const auto& [v, i, j] : pairs) {
        if (asgn[static_cast<std::size_t>(i)] >= 0) continue;
        if (taken[static_cast<std::size_t>(j)]) continue;
        asgn[static_cast<std::size_t>(i)] = j;
        taken[static_cast<std::size_t>(j)] = true;
    }

    std::vector<int> order(static_cast<std::size_t>(n_t));
    std::iota(order.begin(), order.end(), 0);
    std::vector<int> valid_j;
    std::vector<Real> cand;

    for (int sweep = 0; sweep < sweeps_; ++sweep) {
        // Shuffle so no track has a permanent claim on a contested detection.
        for (int i = n_t - 1; i > 0; --i) {
            std::swap(order[static_cast<std::size_t>(i)],
                      order[static_cast<std::size_t>(rng.uniform_int(0, i))]);
        }

        for (const int i : order) {
            // Proper Gibbs conditional for a one-to-one matching: sample this
            // track's assignment given every other track's current claim. A
            // detection already claimed by another track is not a candidate.
            //
            // The reference merely penalised conflicts by a constant, which is
            // far too weak to enforce exclusivity - two tracks sitting on one
            // entity would both be fed the same detection every scan, so
            // neither ever decayed and duplicates accumulated without bound.
            valid_j.clear();
            for (int j = 0; j < n_o; ++j) {
                if (!std::isfinite(loglik[static_cast<std::size_t>(i) * n_o + j])) {
                    continue;
                }
                bool claimed = false;
                for (int k = 0; k < n_t; ++k) {
                    if (k != i && asgn[static_cast<std::size_t>(k)] == j) {
                        claimed = true;
                        break;
                    }
                }
                if (!claimed) valid_j.push_back(j);
            }
            if (valid_j.empty()) {
                asgn[static_cast<std::size_t>(i)] = -1;
                continue;
            }

            cand.clear();
            cand.push_back(kGibbsClutterLoglik);  // option 0: this track is unseen
            for (const int j : valid_j) {
                cand.push_back(loglik[static_cast<std::size_t>(i) * n_o + j]);
            }

            const Real norm = log_sum_exp(cand);
            std::vector<Real> probs(cand.size());
            for (std::size_t c = 0; c < cand.size(); ++c) {
                probs[c] = std::exp(cand[c] - norm);
            }
            const std::size_t choice = rng.categorical(probs.data(), probs.size());
            asgn[static_cast<std::size_t>(i)] =
                choice == 0 ? -1 : valid_j[choice - 1];
        }
    }

    for (int i = 0; i < n_t; ++i) {
        if (asgn[static_cast<std::size_t>(i)] >= 0) {
            out[i] = asgn[static_cast<std::size_t>(i)];
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// PmbmManager
// ---------------------------------------------------------------------------

PmbmManager::PmbmManager(const DomainProfile& profile, Area area,
                         std::uint64_t seed, MotionConstraintPtr constraint,
                         SensorCoveragePtr coverage)
    : profile_(&profile),
      area_(area),
      constraint_(std::move(constraint)),
      coverage_(std::move(coverage)),
      mou_(MouConstants::from(profile)),
      gibbs_(profile.gibbs_sweeps),
      rng_(seed),
      seed_(seed) {}

std::string PmbmManager::next_id() {
    ++track_counter_;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "T%04d", track_counter_);
    return std::string(buf);
}

void PmbmManager::predict() {
    for (auto& t : tracks_) t->predict();
    ++scan_;
}

std::optional<Real> PmbmManager::reacquire_score(const Observation& obs,
                                                 std::size_t d,
                                                 Real timestamp) {
    if (d >= dormant_.size() || !obs.has_position()) return std::nullopt;
    const Vec2 pos = *obs.position;
    const auto& entry = dormant_[d];

    // Score a dormant track by how well its learned pattern of life explains a
    // sighting here, now. This is what turns "we lost him three days ago" into
    // "that is him, at his usual place, at his usual hour". Two complementary
    // cues, for two very different timescales.
    //
    //   pattern of life - "this is where he is at this hour, most days";
    //   kinematics      - "he went that way two seconds ago".
    //
    // Only the first was implemented originally, and it is gated on having a
    // fitted baseline. A track that went dormant before accumulating one could
    // therefore never be reacquired, so every reappearance became a new track
    // and a guaranteed identity switch. That is most tracks in any short
    // sequence, where hour-of-day carries no information at all.
    const Real kinematic_window =
        profile_->reacquire_kinematic_s > 0.0
            ? profile_->reacquire_kinematic_s
            : std::max(profile_->scan_dt_s * 10.0, 1e-6);

    // Both cues, and the better one wins. They used to be an if/else on whether
    // a pattern of life had been fitted, which quietly made them exclusive: a
    // track with a fitted baseline was judged on hour-of-day alone, even when
    // it had vanished four seconds ago and its velocity said exactly where it
    // went. Over any run short enough that hour-of-day carries no information,
    // the fitted baseline predicts the middle of the entity's path, the gate
    // rejects the reappearance, and every entity comes back as a new track.
    // That is the opposite of what the comment above describes, and it is worth
    // being explicit: these are two estimates of the same quantity, so the
    // right combination is whichever explains the sighting better, not
    // whichever was checked first.
    //
    // A Gaussian log-likelihood with its normalisation, so that the two are
    // comparable at all. The score was -dist/sigma, which tends to zero - the
    // best score available - as sigma grows, so the vaguest candidate won
    // every detection it was gated for. Being uncertain is not evidence.
    const Real gate_floor = 2.0 * profile_->pos_noise_m;
    auto evaluate = [&](Vec2 predicted, Real uncertainty) -> std::optional<Real> {
        const Real dist = distance(pos, predicted);
        if (dist > std::max(uncertainty * 3.0, gate_floor)) return std::nullopt;
        const Real sigma = std::max(uncertainty, profile_->pos_noise_m * 0.5);
        return -0.5 * (dist / sigma) * (dist / sigma) - std::log(sigma);
    };

    std::optional<Real> best;
    if (entry.track->pol().fitted()) {
        const auto pred = entry.track->pol().predict_location(timestamp, rng_);
        best = evaluate(pred.position, pred.uncertainty_m);
    }

    const Real elapsed = timestamp - entry.track->last_seen();
    if (elapsed >= 0.0 && elapsed <= kinematic_window) {
        // Carry the last velocity forward from where the state actually
        // stopped, not from the last detection. A track keeps being propagated
        // for several scans after its final hit - that is what coasting is -
        // and only freezes when it goes dormant, so its stored position is
        // already advanced. Extrapolating again from `last_seen` re-applies
        // the coast the filter had already applied.
        const Real since_frozen =
            static_cast<Real>(scan_ - entry.dormant_since_scan) * profile_->scan_dt_s;
        const Vec2 predicted = entry.track->position() +
                               entry.track->velocity() * std::max(since_frozen, 0.0);
        // Uncertainty grows with the whole gap since the last sighting: that is
        // how long it has been since anything was confirmed.
        const Real uncertainty = entry.track->position_uncertainty() +
                                 profile_->courier_speed_thresh * 2.0 * elapsed +
                                 profile_->pos_noise_m;
        if (const auto k = evaluate(predicted, uncertainty)) {
            if (!best.has_value() || *k > *best) best = k;
        }
    }

    if (!best.has_value()) return std::nullopt;
    Real score = *best;

    // Appearance is worth more here than anywhere else in the engine.
    // Association only has to choose between candidates competing in the same
    // scan, where position usually settles it; reacquisition has to decide
    // whether somebody appearing now is somebody who vanished earlier, and
    // across a gap position has decayed to a guess. This is the
    // re-identification problem proper, and a descriptor is the only evidence
    // that survives the gap intact.
    if (profile_->appearance_weight > 0.0 && obs.has_descriptor()) {
        const Descriptor& remembered = entry.track->appearance();
        if (remembered.valid()) {
            const Real sim = remembered.similarity(obs.descriptor);
            score += profile_->appearance_weight *
                     profile_->reacquire_appearance_gain * sim;
            // A confident mismatch is grounds for refusal, not merely a lower
            // score: resurrecting the wrong identity is worse than starting a
            // new track.
            if (sim < profile_->reacquire_min_similarity) return std::nullopt;
        }
    }
    return score;
}

TrackPtr PmbmManager::revive(std::size_t d, const Observation& obs) {
    TrackPtr revived = dormant_[d].track;
    dormant_.erase(dormant_.begin() + static_cast<long>(d));

    // Re-seed the particle cloud on the new detection; the old kinematic state
    // is stale, but the pattern of life and the identity carry over.
    revived->filter().init(*obs.position);
    revived->set_existence(profile_->r_birth);
    return revived;
}

std::unordered_map<const Observation*, TrackPtr> PmbmManager::reacquire_batch(
    const std::vector<const Observation*>& unassigned, Real timestamp) {
    std::unordered_map<const Observation*, TrackPtr> out;
    if (dormant_.empty() || unassigned.empty()) return out;

    // Rows are reappearing detections, columns are dormant tracks. `match`
    // minimises cost, and the score is "higher is better", so cost is its
    // negation; a pair that cannot go together is infinite and gated out.
    constexpr Real kNoMatch = std::numeric_limits<Real>::infinity();
    std::vector<std::vector<Real>> cost(unassigned.size(),
                                        std::vector<Real>(dormant_.size(), kNoMatch));
    bool any = false;
    for (std::size_t r = 0; r < unassigned.size(); ++r) {
        for (std::size_t c = 0; c < dormant_.size(); ++c) {
            if (const auto sc = reacquire_score(*unassigned[r], c, timestamp)) {
                cost[r][c] = -*sc;
                any = true;
            }
        }
    }
    if (!any) return out;

    const Assignment a = match(cost, kNoMatch);

    // Revive in descending column order: reviving erases from `dormant_`, which
    // would invalidate the indices of every column after it.
    std::vector<std::pair<std::size_t, std::size_t>> pairs;   // (column, row)
    for (std::size_t r = 0; r < unassigned.size(); ++r) {
        if (a.row_to_col[r] >= 0) {
            pairs.emplace_back(static_cast<std::size_t>(a.row_to_col[r]), r);
        }
    }
    std::sort(pairs.begin(), pairs.end(), std::greater<>());
    for (const auto& [col, row] : pairs) {
        out[unassigned[row]] = revive(col, *unassigned[row]);
    }
    return out;
}

void PmbmManager::try_group_spawn(Track& fresh) {
    for (const auto& existing : tracks_) {
        if (existing->id() == fresh.id()) continue;
        const Real sep = distance(fresh.position(), existing->position());
        if (sep > kGroupSpawnRadius) continue;

        // Velocity dispersion across the particle cloud: high dispersion under
        // a steady detection rate means the filter is straddling two entities.
        const auto& pf = existing->filter();
        const Vec2 mean_v = existing->velocity();
        Real var = 0.0;
        const auto& vxs = pf.vxs();
        const auto& vys = pf.vys();
        for (std::size_t i = 0; i < vxs.size(); ++i) {
            const Real dvx = vxs[i] - mean_v.x;
            const Real dvy = vys[i] - mean_v.y;
            var += dvx * dvx + dvy * dvy;
        }
        var /= static_cast<Real>(std::max<std::size_t>(vxs.size(), 1));

        const Real vel_var_thresh =
            std::pow(profile_->courier_speed_thresh * kGroupVelVarScale, 2.0);
        const bool is_group = existing->measurement_rate() > kGroupMeasRateThresh &&
                              var > vel_var_thresh && existing->age() > 5;
        if (is_group) {
            fresh.set_parent(existing->id());
            fresh.pol().clone_from(existing->pol());
            return;
        }
        if (sep < kProximityParentRadius && existing->age() > 3) {
            fresh.set_parent(existing->id());
            return;
        }
    }
}

void PmbmManager::update(const std::vector<Observation>& observations,
                         Real timestamp) {
    std::vector<const Observation*> valid;
    valid.reserve(observations.size());
    for (const auto& o : observations) {
        if (o.has_position()) valid.push_back(&o);
        cred_.note_source(o.source_id);
    }

    // A scan with nothing in it is genuinely ambiguous: either nothing is
    // there, or nobody is looking. Treating it as evidence of absence is only
    // right in the first case, and it is expensive to get wrong - a camera
    // estate that drops out for four scans loses every track it holds, because
    // at p_detection 0.9 four consecutive misses is overwhelming evidence that
    // the entity has gone. `max_coast_s` and `dormant_timeout` never come into
    // it; existence has already collapsed.
    //
    // The engine is never told which sensors are live, but it does not need to
    // be: a scene that has been producing detections every scan and abruptly
    // produces none is far more likely to have lost its sensors than to have
    // lost every entity simultaneously. Silence from everything at once is
    // not evidence about any one thing.
    constexpr std::size_t kCoverageWindow = 10;
    const bool was_reporting =
        std::count(scan_had_detections_.begin(), scan_had_detections_.end(), true) >=
        static_cast<long>(scan_had_detections_.size()) / 2;
    scan_had_detections_.push_back(!valid.empty());
    while (scan_had_detections_.size() > kCoverageWindow) {
        scan_had_detections_.pop_front();
    }
    coverage_gap_ = valid.empty() && was_reporting && !tracks_.empty();

    if (valid.empty()) {
        // Time still passes in a coverage gap - tracks age, and dormancy and
        // pruning are on wall-clock, so an outage cannot hold a track open for
        // ever. What is withheld is the *evidential* penalty for not being
        // seen by an instrument that was not looking.
        if (!coverage_gap_) {
            for (auto& t : tracks_) t->update_miss();
        }
        // Deliberately not recording detection trials here. A scan with no
        // detections at all says nothing about any individual sensor's
        // detection probability - it is the scene, or the estate, not the
        // sensor - and counting it would drive every estimate towards zero
        // during exactly the outages the estimate is supposed to survive.
        prune(timestamp);
        return;
    }

    // Association runs once per source, not once over all detections.
    //
    // Exclusivity is a fact about a sensor, not about the world: one camera
    // reports a given entity at most once per scan, but two overlapping cameras
    // both report it, and that second report is corroboration rather than a
    // second entity. Enforcing one-detection-per-track globally left every
    // such corroborating report unassigned, where it promptly founded a
    // duplicate track - 11 ghost tracks per scan in a two-sensor scenario, and
    // a steady drizzle of them in every overlapping-camera setup.
    //
    // Multi-source fusion is the whole point of the observation model, so this
    // is the case that has to work.
    std::map<std::string, std::vector<const Observation*>> by_source;
    for (const auto* o : valid) by_source[o->source_id].push_back(o);

    // track index -> the detections assigned to it this scan, at most one per
    // source.
    std::unordered_map<int, std::vector<const Observation*>> track_hits;
    std::unordered_set<const Observation*> assigned;

    for (const auto& [source, group] : by_source) {
        const auto group_asgn = gibbs_.assign(tracks_, group, *profile_, rng_);
        for (const auto& [track_idx, obs_idx] : group_asgn) {
            const Observation* o = group[static_cast<std::size_t>(obs_idx)];
            track_hits[track_idx].push_back(o);
            assigned.insert(o);
        }
    }

    clutter_.update(static_cast<int>(valid.size()) -
                    static_cast<int>(assigned.size()));
    const Real cd = clutter_.density(area_.volume());

    // How often does a sensor actually report the tracks it has been feeding?
    // One trial per (track, recent feeder) pair per scan. Pooled per source in
    // the estimator, so one spurious track cannot move it far.
    const auto effective_pd = [&](const std::string& track_id) {
        if (!profile_->adaptive_p_detection) return -1.0;
        const auto fit = feeders_.find(track_id);
        if (fit == feeders_.end()) return -1.0;
        // The best chance any sensor still covering this track had of seeing
        // it. Taking the best rather than an average is the conservative
        // reading of a miss: if one good sensor should have seen it and did
        // not, that is the informative failure.
        Real best = -1.0;
        for (const auto& [src, last] : fit->second) {
            if (scan_ - last > kFeederWindow) continue;
            best = std::max(best, detect_rate_.rate(src, profile_->p_detection));
        }
        return best;
    };

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        const auto it = track_hits.find(static_cast<int>(i));
        const std::string& tid = tracks_[i]->id();

        if (profile_->adaptive_p_detection) {
            std::set<std::string> hit_by;
            if (it != track_hits.end()) {
                for (const Observation* o : it->second) hit_by.insert(o->source_id);
            }
            auto& fed = feeders_[tid];
            const Vec2 where = tracks_[i]->position();
            for (auto f = fed.begin(); f != fed.end();) {
                if (scan_ - f->second > kFeederWindow) {
                    f = fed.erase(f);
                    continue;
                }
                // Only a sensor that was looking at this track can be said to
                // have missed it. Without a coverage map the engine assumes a
                // recent feeder still covers it, which holds for a wide-area
                // sensor and not at all for a gate reader watching a few
                // metres - every point sensor then estimates out at the floor,
                // having been charged with a miss every time an entity walked
                // away from it.
                const bool looking =
                    coverage_ == nullptr || coverage_->covers(f->first, where);
                if (looking) {
                    detect_rate_.record(f->first, hit_by.count(f->first) != 0);
                }
                ++f;
            }
            for (const std::string& src : hit_by) fed[src] = scan_;
        }

        if (it == track_hits.end()) {
            // A miss is evidence of absence only where somebody was looking.
            // With a coverage map that is a fact rather than an inference, and
            // it is the per-track version of the whole-scan guess made above:
            // one track can walk out of the estate while the rest stay in it.
            if (coverage_ != nullptr &&
                !coverage_->anyone_covers(tracks_[i]->position())) {
                continue;
            }
            tracks_[i]->update_miss(effective_pd(tid));
            continue;
        }

        // Existence is updated once per scan however many sources reported the
        // entity: the detections are corroborating evidence about one scan, not
        // independent scans. Counting each one separately would let a track
        // watched by four cameras become four times as certain as the same
        // track watched by one.
        // The Bernoulli/JIPDA update for a track that WAS detected is
        //
        //     r' = r p_D g(z) / ( r p_D g(z) + (1-r) lambda_c )
        //
        // and the factor that is easy to drop is g(z), the likelihood DENSITY
        // of the detection under this track's own innovation covariance.
        // Without it the numerator carried a bare probability while the
        // denominator carried a density per square metre, so the ratio was not
        // a quantity at all - its value moved with the units the area of
        // regard happened to be written in.
        //
        // It also made the update blind to fit. Two detections, one on top of
        // the prediction and one at the very edge of the gate, produced
        // byte-identical existence, so r measured only that SOMETHING had been
        // associated. verification/v08 finds the counterexample immediately and
        // v09 proves the consequence: a newborn track went from r_birth = 0.45
        // to above 0.999 on its first detection whatever that detection looked
        // like, which left r_confirm = 0.55 clearing on every track that got
        // one and no track that did not.
        const Real L = profile_->p_detection;
        const Real r = tracks_[i]->existence();
        Real best_nis = std::numeric_limits<Real>::infinity();
        for (const Observation* o : it->second) {
            if (!o->has_position()) continue;
            best_nis =
                std::min(best_nis, tracks_[i]->filter().mahalanobis_sq(*o->position));
        }
        if (std::isfinite(best_nis)) {
            // 2-D Gaussian density at the innovation: exp(-NIS/2) / (2 pi sqrt|S|).
            // The best-fitting member of the group, for the same reason the
            // whole group updates existence once: these are one scan's evidence
            // about one entity, and the best explanation of it is what counts.
            const Mat2 S = tracks_[i]->filter().innovation_covariance();
            const Real det_s = std::max(S.det(), 1e-12);
            const Real g = std::exp(-0.5 * best_nis) /
                           (2.0 * std::numbers::pi * std::sqrt(det_s));
            const Real lik = r * L * g;
            tracks_[i]->set_existence(
                std::clamp(lik / (lik + (1.0 - r) * cd + 1e-300), 0.0, 0.9999));
        }

        // Where several sensors reported this entity in this scan, each can be
        // checked against the others. That is the only non-circular test
        // available: a sensor's fit to a track it has itself been steering says
        // nothing about whether the sensor is right.
        const auto& group = it->second;
        const Real expected = profile_->pos_noise_m * std::numbers::sqrt2;

        if (group.size() >= 3) {
            // Three or more reports: the majority pulls the peer mean towards
            // the truth, so a biased sensor's disagreement is roughly twice
            // any sound sensor's and attribution works.
            for (const Observation* o : group) {
                Vec2 peer_sum{};
                int peers = 0;
                for (const Observation* other : group) {
                    if (other == o) continue;
                    peer_sum += *other->position;
                    ++peers;
                }
                const Vec2 peer_mean = peer_sum / static_cast<Real>(peers);
                cred_.update_against_peers(o->source_id,
                                           distance(*o->position, peer_mean),
                                           expected);
            }
        } else if (group.size() == 2) {
            // Two reports and no tie-breaker. The disagreement is identical
            // from both sides, so punishing either is a coin flip - and doing
            // so demonstrably punished the sound sensor as hard as the drifting
            // one. Record the conflict instead: "these two disagree" is
            // actionable even when "this one is wrong" is not knowable.
            cred_.record_pairwise_conflict(
                group[0]->source_id, group[1]->source_id,
                distance(*group[0]->position, *group[1]->position), expected);
        }

        for (const Observation* o : group) {
            // Fit to the assigned track: weak, and circular for a sensor that
            // has been steering that track, but it still catches a gross
            // outlier where no peer evidence exists at all.
            const Real nis = tracks_[i]->filter().mahalanobis_sq(*o->position);
            const Real obs_ll = -0.5 * nis;
            cred_.update(o->source_id, obs_ll, kCredLoglikThreshold);
            // The same residual says something else as well: how far the
            // sensor's actual error is from the profile's claim about it. But
            // only its *spread* does. Measured about zero, a sensor whose mount
            // has drifted reads as a noisy one and has its association gate
            // widened, which is precisely the response that lets its wrong
            // detections keep hold of tracks - on the sensor-drift scenario
            // that cost twelve points of recovery. Measured about the source's
            // own estimated offset, bias goes to the credibility machinery
            // where it belongs and only spread reaches this.
            //
            // And only from a track whose prediction is one scan old. After a
            // long coast the residual is dominated by where the track was
            // guessed to be, not by where the sensor said it was, and feeding
            // those in tells the estimator the sensor is noisy when what is
            // actually uncertain is the prediction. In the warehouse scenario -
            // patchy readers, long dormancy - that cost sixteen points of
            // recovery on its own. A sensor's noise is measured on a target
            // that is being tracked well.
            const bool prediction_is_fresh =
                timestamp - tracks_[i]->last_seen() <= profile_->scan_dt_s * 1.5;
            if (profile_->adaptive_meas_noise && prediction_is_fresh) {
                const Vec2 centred = *o->position - cred_.mean_residual(o->source_id);
                noise_.observe(o->source_id,
                               tracks_[i]->filter().mahalanobis_sq(centred));
            }
            // The residual's direction is the durable evidence: noise cancels
            // over many samples, a miscalibration does not.
            cred_.note_residual(o->source_id,
                                *o->position - tracks_[i]->position(),
                                profile_->pos_noise_m);
            Observation adjusted = *o;
            adjusted.confidence = o->confidence * cred_.get(o->source_id);
            if (profile_->adaptive_meas_noise) {
                tracks_[i]->filter().set_noise_scale(noise_.scale(o->source_id));
            }
            tracks_[i]->update_hit(adjusted, profile_->scan_dt_s);
        }
        for (const Observation* o : it->second) {
            tracks_[i]->note_hit_scan(scan_, o->source_id);
        }
    }

    // How far could a real entity have moved since the previous scan? Anything
    // beyond this cannot be the same object, so it cannot corroborate a birth.
    Real birth_gate = profile_->birth_gate_m;
    if (birth_gate <= 0.0) {
        // The furthest an entity can travel between scans, by the profile's own
        // account of how its entities move.
        //
        // Each MOU regime is built by `motion(name, heading_hold_s,
        // typical_speed_mps)`, which sets sigma = typical_speed * sqrt(2 theta)
        // - so sigma / sqrt(2 theta) recovers that typical speed exactly, and
        // the fastest regime is the domain's own statement of its top speed.
        //
        // This used to be derived from `courier_speed_thresh`, which is a
        // network-analysis parameter - how fast a courier walks - and sits
        // between 0.3 and 4 m/s in every shipped profile. Multiplied by four it
        // still described nobody: an aircraft profile whose own motion model
        // declares 300 m/s got a gate built from 12 m/s. No profile sets
        // `birth_gate_m`, so the derivation is what every one of them used, and
        // where a scan's travel exceeded the gate no track could ever be born
        // at all. Measured on a clean single-target feed, three profiles never
        // formed a track: Airspace (1,000 m per scan against a 210 m gate),
        // VehicleConvoy (150 m against 129 m) and Maritime (32.4 km against
        // 29.4 km). Slowing the target below the gate made tracks appear
        // immediately in all three.
        //
        // No multiplier: the fast regime is already the fast case, and the
        // factor of four was there to compensate for a speed scale that was an
        // order of magnitude too small.
        //
        // The gate has two constraints pulling against each other. It must be
        // at least one scan of travel, or a fast entity can never be
        // corroborated; and no wider than the spacing between entities, or a
        // detection is corroborated by its neighbour and two objects become
        // one. This derivation serves the first. Where a domain packs its
        // entities closer together than one scan of travel - WarehouseAssets
        // does, which is why the fix costs it about five points of recovery
        // while improving its identity switches - the profile should say so by
        // setting `birth_gate_m` directly. No profile currently does.
        Real fastest = 0.0;
        for (const auto& m : profile_->mou_models) {
            fastest = std::max(fastest,
                               m.sigma / std::sqrt(2.0 * std::max(m.theta, 1e-6)));
        }
        birth_gate = fastest * profile_->scan_dt_s + 3.0 * profile_->pos_noise_m;
    }

    std::vector<Vec2> unassigned_now;

    // Leftover detections: either a dormant entity resurfacing, or a birth.
    // Resurfacing is settled first and for all of them at once, because which
    // dormant track a detection belongs to depends on what the other
    // detections claim. Deciding it one detection at a time gives the answer
    // away to whichever happened to be considered first.
    std::vector<const Observation*> leftover;
    for (const Observation* o : valid) {
        if (assigned.contains(o)) continue;
        const Real weight = profile_->modality_weight(o->modality) * o->confidence;
        if (weight * cred_.get(o->source_id) < kMinBirthWeight) continue;
        leftover.push_back(o);
    }
    auto revivals = reacquire_batch(leftover, timestamp);

    for (std::size_t j = 0; j < valid.size(); ++j) {
        const Observation& obs = *valid[j];
        if (assigned.contains(&obs)) continue;
        unassigned_now.push_back(*obs.position);

        const Real weight = profile_->modality_weight(obs.modality) * obs.confidence;
        if (weight * cred_.get(obs.source_id) < kMinBirthWeight) continue;

        if (const auto it = revivals.find(&obs); it != revivals.end()) {
            TrackPtr revived = std::move(it->second);
            revivals.erase(it);
            revived->update_hit(obs, profile_->scan_dt_s);
            revived->note_hit_scan(scan_, obs.source_id);
            tracks_.push_back(std::move(revived));
            continue;
        }

        // Two-point initiation. A moving entity leaves a trail of unassigned
        // detections in consecutive scans; an isolated false alarm does not.
        if (profile_->two_point_initiation) {
            bool corroborated = false;
            for (const Vec2& prev : prev_unassigned_) {
                if (distance(prev, *obs.position) <= birth_gate) {
                    corroborated = true;
                    break;
                }
            }
            if (!corroborated) continue;
        }

        if (weight > kMinBirthWeight) {
            auto fresh = std::make_shared<Track>(
                next_id(), profile_->r_birth, *profile_, mou_, timestamp,
                seed_ + static_cast<std::uint64_t>(track_counter_) * 7919ULL,
                constraint_);
            fresh->filter().init(*obs.position);
            // No existence update on birth: this detection is already the
            // reason the track exists, and counting it again would confirm
            // every false alarm on the spot. Corroboration must come from a
            // *subsequent* scan, which is what r_birth < r_confirm encodes.
            fresh->update_hit(obs, profile_->scan_dt_s);
            fresh->note_hit_scan(scan_, obs.source_id);
            try_group_spawn(*fresh);
            tracks_.push_back(std::move(fresh));
        }
    }

    for (const auto* o : valid) {
        cred_.note_assignment(o->source_id, assigned.contains(o));
    }

    prev_unassigned_ = std::move(unassigned_now);
    merge_duplicates();
    prune(timestamp);
}

// Two tracks on one entity is the failure mode that quietly ruins everything
// downstream: the pair look like a meeting, the network analyser invents a
// contact, and identity continuity breaks even though the positions are good.
// The existence update alone cannot fix it - a duplicate that wins a detection
// every few scans keeps resetting its own existence and never decays away.
void PmbmManager::merge_duplicates() {
    if (tracks_.size() < 2) return;

    // The gate has to scale with how far an entity travels between scans,
    // because that is roughly how far from the original a duplicate is born.
    // A gate fixed to sensor noise alone was ~2% of one scan of motion in the
    // sparse domains (hourly AIS, four-hourly collar fixes) and duplicates
    // simply never came within it, while being ~75% of it in the maze - which
    // is why merging appeared to work there and nowhere else.
    //
    // Widening it is safe because distance is only a cheap prefilter here. The
    // test that actually decides is whether the two tracks were ever fed their
    // own detection in the same scan: two real entities are, two tracks on one
    // entity cannot be.
    const Real merge_dist =
        profile_->merge_distance_m > 0.0
            ? profile_->merge_distance_m
            : std::max({3.0 * profile_->pos_noise_m, 0.5 * profile_->brush_pass_m,
                        0.3 * profile_->courier_speed_thresh * 4.0 *
                            profile_->scan_dt_s});

    // Better-established tracks first, so survivors are the ones with history.
    std::sort(tracks_.begin(), tracks_.end(),
              [](const TrackPtr& a, const TrackPtr& b) {
                  if (a->hits() != b->hits()) return a->hits() > b->hits();
                  return a->existence() > b->existence();
              });

    std::vector<bool> absorbed(tracks_.size(), false);

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        if (absorbed[i]) continue;
        for (std::size_t j = i + 1; j < tracks_.size(); ++j) {
            if (absorbed[j]) continue;

            const Real d = distance(tracks_[i]->position(), tracks_[j]->position());
            if (d > merge_dist) continue;

            // Statistical test under the summed covariances: close in metres is
            // not enough when both estimates are uncertain.
            const Mat2 Pi = tracks_[i]->filter().position_covariance();
            const Mat2 Pj = tracks_[j]->filter().position_covariance();
            const Mat2 S{Pi.a + Pj.a + profile_->meas_noise_var,
                         Pi.b + Pj.b,
                         Pi.c + Pj.c + profile_->meas_noise_var};
            const Vec2 delta = tracks_[i]->position() - tracks_[j]->position();
            if (S.inverse().quad(delta) > profile_->merge_chi2) continue;

            // Two discriminators, because one entity seen twice and two
            // entities seen once look alike from a distance.
            //
            // First: did any single sensor feed both tracks in one scan? A
            // sensor reports a given entity once per scan, so that settles it.
            if (tracks_[i]->shares_hit_scan_with(*tracks_[j])) continue;

            // Second: are both tracks being fed steadily by the same set of
            // sensors, just never in the same scan? One entity cannot produce
            // two independent streams of detections from one sensor, so this is
            // two entities whose detections happen to alternate. Without this,
            // two genuinely distinct entities merge whenever source identifiers
            // carry no spatial meaning - and collapsing a real pair is a worse
            // error than the duplicate it was meant to prevent.
            const bool both_well_fed = tracks_[i]->measurement_rate() > 0.55 &&
                                       tracks_[j]->measurement_rate() > 0.55 &&
                                       tracks_[i]->age() > 8 && tracks_[j]->age() > 8;
            if (both_well_fed && tracks_[i]->shares_source_with(*tracks_[j])) continue;

            tracks_[i]->absorb(*tracks_[j]);
            absorbed[j] = true;
        }
    }

    std::vector<TrackPtr> keep;
    keep.reserve(tracks_.size());
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        if (!absorbed[i]) keep.push_back(std::move(tracks_[i]));
    }
    tracks_ = std::move(keep);

    // Per-track state for identities that are gone for good. `feeders_` is
    // keyed by track id and was never cleared, so it kept a row for every
    // track ever created.
    {
        const std::set<std::string> live = known_ids();
        for (auto it = feeders_.begin(); it != feeders_.end();) {
            it = live.count(it->first) != 0 ? std::next(it) : feeders_.erase(it);
        }
    }
}

void PmbmManager::prune(Real timestamp) {
    std::vector<TrackPtr> keep;
    keep.reserve(tracks_.size());

    const Real max_coast =
        profile_->max_coast_s > 0.0
            ? profile_->max_coast_s
            : profile_->dormant_timeout * profile_->scan_dt_s;

    // Note who has ever been reportable before deciding what to discard.
    for (auto& t : tracks_) {
        if (t->existence() >= profile_->r_confirm) t->mark_confirmed();
    }

    for (auto& t : tracks_) {
        const Real coasted = timestamp - t->last_seen();
        if (coasted > max_coast) {
            // Out of time rather than out of evidence, but still a recognisable
            // identity for as long as one of the reacquisition cues holds.
            dormant_.push_back(DormantEntry{std::move(t), scan_});
            continue;
        }
        if (t->existence() > profile_->r_prune) {
            keep.push_back(std::move(t));
        } else if (t->ever_confirmed()) {
            // Retire it from the live set but keep the identity: it was a real
            // entity, and it may well come back.
            //
            // Dormancy used to require existence to land inside the band
            // between r_dormant and r_prune - typically 0.01 wide, which a
            // decaying existence falls straight through in a single scan, and
            // which two shipped profiles had inverted so the band was empty and
            // dormancy impossible. It also demanded a fitted pattern of life,
            // which a short-lived track can never have. Whether an identity is
            // worth remembering is a question about the track's history, not
            // about where a decaying number happened to stop.
            dormant_.push_back(DormantEntry{std::move(t), scan_});
        }
        // A track that was never confirmed was probably clutter; drop it.
    }
    tracks_ = std::move(keep);

    const auto cap = static_cast<std::size_t>(std::max(profile_->max_tracks, 1));
    if (tracks_.size() > cap) {
        std::partial_sort(tracks_.begin(), tracks_.begin() + static_cast<long>(cap),
                          tracks_.end(),
                          [](const TrackPtr& a, const TrackPtr& b) {
                              return a->existence() > b->existence();
                          });
        tracks_.resize(cap);
    }

    std::erase_if(dormant_, [&](const DormantEntry& d) {
        return scan_ - d.dormant_since_scan >= profile_->dormant_timeout;
    });
}

std::vector<TrackPtr> PmbmManager::confirmed() const {
    std::vector<TrackPtr> out;
    out.reserve(tracks_.size());
    for (const auto& t : tracks_) {
        if (t->existence() >= profile_->r_confirm) out.push_back(t);
    }
    std::sort(out.begin(), out.end(), [](const TrackPtr& a, const TrackPtr& b) {
        return a->existence() > b->existence();
    });
    return out;
}

}  // namespace trace
