#include "trace/core/pmbm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
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
constexpr std::size_t kMaxTracks = 80;

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
// SourceCredibility
// ---------------------------------------------------------------------------

void SourceCredibility::update(const std::string& source_id, Real obs_loglik,
                               Real threshold) {
    if (source_id.empty()) return;
    auto [it, inserted] = scores_.try_emplace(source_id, kCredDefault);
    const Real target = obs_loglik > threshold ? kCredGood : kCredBad;
    it->second = kCredDecay * it->second + (1.0 - kCredDecay) * target;
}

Real SourceCredibility::get(const std::string& source_id) const {
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
            if (m2 < profile.gate_chi2) {
                const Real w = profile.modality_weight(o.modality) * o.confidence;
                loglik[static_cast<std::size_t>(i) * n_o + j] =
                    -0.5 * m2 + std::log(w + 1e-300);
            }
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
                         std::uint64_t seed, MotionConstraintPtr constraint)
    : profile_(&profile),
      area_(area),
      constraint_(std::move(constraint)),
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

TrackPtr PmbmManager::try_reacquire(const Observation& obs, Real timestamp) {
    if (dormant_.empty() || !obs.has_position()) return nullptr;
    const Vec2 pos = *obs.position;

    // Score each dormant track by how well its learned pattern of life
    // explains a sighting here, now. This is what turns "we lost him three
    // days ago" into "that is him, at his usual place, at his usual hour".
    // Two complementary cues, for two very different timescales.
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

    Real best_score = -std::numeric_limits<Real>::infinity();
    long best_idx = -1;

    for (std::size_t i = 0; i < dormant_.size(); ++i) {
        const auto& entry = dormant_[i];
        Vec2 predicted{};
        Real uncertainty = 0.0;
        bool usable = false;

        if (entry.track->pol().fitted()) {
            const auto pred = entry.track->pol().predict_location(timestamp, rng_);
            predicted = pred.position;
            uncertainty = pred.uncertainty_m;
            usable = true;
        } else {
            const Real elapsed = timestamp - entry.track->last_seen();
            if (elapsed >= 0.0 && elapsed <= kinematic_window) {
                // Carry the last velocity forward, and let the gate widen with
                // the gap: after a second of not looking, we know much less.
                predicted = entry.track->position() + entry.track->velocity() * elapsed;
                uncertainty = entry.track->position_uncertainty() +
                              profile_->courier_speed_thresh * 2.0 * elapsed +
                              profile_->pos_noise_m;
                usable = true;
            }
        }
        if (!usable) continue;

        const Real dist = distance(pos, predicted);
        if (dist > std::max(uncertainty * 3.0, 2.0 * profile_->pos_noise_m)) continue;
        const Real score = -dist / std::max(uncertainty, 1.0);
        if (score > best_score) {
            best_score = score;
            best_idx = static_cast<long>(i);
        }
    }

    if (best_idx < 0) return nullptr;

    TrackPtr revived = dormant_[static_cast<std::size_t>(best_idx)].track;
    dormant_.erase(dormant_.begin() + best_idx);

    // Re-seed the particle cloud on the new detection; the old kinematic state
    // is stale, but the pattern of life and the identity carry over.
    revived->filter().init(pos);
    revived->set_existence(profile_->r_birth);
    return revived;
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
    }

    if (valid.empty()) {
        for (auto& t : tracks_) t->update_miss();
        prune(timestamp);
        return;
    }

    const auto asgn = gibbs_.assign(tracks_, valid, *profile_, rng_);

    std::unordered_set<int> assigned_obs;
    assigned_obs.reserve(asgn.size());
    for (const auto& [ti, oj] : asgn) assigned_obs.insert(oj);

    clutter_.update(static_cast<int>(valid.size()) -
                    static_cast<int>(assigned_obs.size()));
    const Real cd = clutter_.density(area_.volume());

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        const auto it = asgn.find(static_cast<int>(i));
        if (it == asgn.end()) {
            tracks_[i]->update_miss();
            continue;
        }

        const Observation& obs = *valid[static_cast<std::size_t>(it->second)];
        const Real obs_ll = -0.5 * tracks_[i]->filter().mahalanobis_sq(*obs.position);
        cred_.update(obs.source_id, obs_ll, kCredLoglikThreshold);
        const Real trust = cred_.get(obs.source_id);

        // The single point where Bayesian existence is updated. A detection in
        // a noisy scan is weaker evidence than the same detection in a clean
        // one, which is exactly what the adaptive clutter density expresses.
        const Real L = profile_->p_detection;
        const Real r = tracks_[i]->existence();
        tracks_[i]->set_existence(
            std::clamp(r * L / (r * L + (1.0 - r) * cd + 1e-300), 0.0, 0.9999));

        Observation adjusted = obs;
        adjusted.confidence = obs.confidence * trust;
        tracks_[i]->update_hit(adjusted, profile_->scan_dt_s);
        tracks_[i]->note_hit_scan(scan_);
    }

    // How far could a real entity have moved since the previous scan? Anything
    // beyond this cannot be the same object, so it cannot corroborate a birth.
    Real birth_gate = profile_->birth_gate_m;
    if (birth_gate <= 0.0) {
        // Derive from the domain's own speed scale, not from the fastest MOU
        // model: a low-theta regime has an enormous steady-state velocity
        // variance that says nothing about how far a real entity travels in
        // one scan, and using it opened the gate to the whole area of regard.
        const Real speed_scale = profile_->courier_speed_thresh * 4.0;
        birth_gate = speed_scale * profile_->scan_dt_s + 3.0 * profile_->pos_noise_m;
    }

    std::vector<Vec2> unassigned_now;

    // Leftover detections: either a dormant entity resurfacing, or a birth.
    for (std::size_t j = 0; j < valid.size(); ++j) {
        if (assigned_obs.contains(static_cast<int>(j))) continue;
        const Observation& obs = *valid[j];
        unassigned_now.push_back(*obs.position);

        const Real weight = profile_->modality_weight(obs.modality) * obs.confidence;
        if (weight * cred_.get(obs.source_id) < kMinBirthWeight) continue;

        if (TrackPtr revived = try_reacquire(obs, timestamp)) {
            revived->update_hit(obs, profile_->scan_dt_s);
            revived->note_hit_scan(scan_);
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
            fresh->note_hit_scan(scan_);
            try_group_spawn(*fresh);
            tracks_.push_back(std::move(fresh));
        }
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

            // The discriminator. Two people walking side by side each generate
            // their own detection in the same scan; two tracks on one person
            // can only take turns. Without this test a genuine pair would be
            // collapsed into one track, which is a worse error than the one
            // being fixed.
            if (tracks_[i]->shares_hit_scan_with(*tracks_[j])) continue;

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

    if (tracks_.size() > kMaxTracks) {
        std::partial_sort(tracks_.begin(),
                          tracks_.begin() + static_cast<long>(kMaxTracks),
                          tracks_.end(),
                          [](const TrackPtr& a, const TrackPtr& b) {
                              return a->existence() > b->existence();
                          });
        tracks_.resize(kMaxTracks);
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
