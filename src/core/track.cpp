#include "trace/core/track.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace trace {
namespace {

/// Existence update constants, ported from the reference implementation.
/// How much of its possibility a track retains between updates. Slightly below
/// one, so a track fed steadily weaker evidence sags towards that weaker level
/// rather than holding its best-ever moment forever.
constexpr Real kPossRetain = 0.97;
constexpr Real kPossAlpha = 0.25;      ///< possibilistic decay on a miss
constexpr Real kThreatEmaAlpha = 0.3;
constexpr Real kThreatPersistThresh = 0.62;
constexpr std::size_t kMaxHistory = 50;
constexpr std::size_t kMaxThreatHistory = 20;

}  // namespace

Track::Track(std::string id, Real r, const DomainProfile& profile,
             const MouConstants& mou, Real t0, std::uint64_t seed,
             MotionConstraintPtr constraint)
    : id_(std::move(id)),
      r_(std::clamp(r, 0.0, 1.0)),
      pi_r_(std::clamp(r, 0.0, 1.0)),
      pf_(profile, mou, seed, std::move(constraint)),
      pol_(profile),
      profile_(&profile),
      born_at_(t0),
      last_seen_(t0) {}

void Track::predict() {
    r_ *= profile_->p_survival;
    pi_r_ *= profile_->p_survival;
    pf_.predict();
    ++age_;
}

void Track::update_hit(const Observation& obs, Real scan_dt) {
    if (!obs.has_position()) return;
    const Vec2 pos = *obs.position;

    const Real weight = profile_->modality_weight(obs.modality) * obs.confidence;

    // A low-confidence source should move the filter less, so its assumed
    // measurement noise is inflated rather than its likelihood truncated.
    pf_.update(pos, 1.0 / (weight + 0.1));

    // If this sighting follows the previous one by about one scan period, the
    // implied velocity is informative. The reference compared against a fixed
    // 1-second step, which never matched a 60-second scan and silently
    // disabled this path; the profile's own scan period is the right yardstick.
    const Real gap = obs.timestamp - last_obs_ts_;
    const bool consecutive =
        last_obs_pos_.has_value() && std::abs(gap - scan_dt) < scan_dt * 0.25;
    if (consecutive) {
        pf_.update_trajectory(pos, *last_obs_pos_, gap);
    }

    // Bayesian existence is deliberately NOT updated here. It depends on the
    // current clutter density, which only the manager knows, and the manager
    // applies it once per association. Updating it here as well double-counted
    // every detection, and drove a newborn track to near-certainty using the
    // very detection that created it - which made the birth threshold dead
    // code. See PmbmManager::update.

    // Possibilistic existence: the degree to which the evidence *permits* this
    // track to exist, as opposed to the Bayesian r, which measures how much
    // evidence has accumulated. The two answer different questions, and their
    // divergence is the diagnostic.
    //
    // The reference computed this as a running product of factors that are
    // always <= 1, so it could only ever fall, while r rose to ~1. Every
    // long-lived track therefore converged to a mismatch of 1.0 no matter what
    // evidence it was built from, and the diagnostic was pure noise - it fired
    // on 119 of 119 scans for a fabricated track and equally for every real
    // one. A possibility measure must be able to rise: good evidence makes a
    // hypothesis *more* permissible, not less.
    //
    // Normalising by the best weight any source could supply puts quality on a
    // 0..1 scale, so pi_r converges to the typical quality of this track's
    // evidence while r converges to 1 on sheer count. A wide gap means many
    // weak detections have been laundered into false certainty.
    const Real best_weight = profile_->modality_weight(Modality::GEOINT);
    const Real quality = std::clamp(weight / std::max(best_weight, 1e-6), 0.0, 1.0);
    pi_r_ = std::clamp(std::max(pi_r_ * kPossRetain, quality), 0.0, 1.0);

    poss_mismatch_ = std::abs(r_ - pi_r_) / (std::max(r_, pi_r_) + 1e-6);

    // Fold this detection's appearance into the running model. Done on every
    // hit, so the model reflects the entity across viewpoints rather than
    // whichever frame happened to be first.
    if (obs.has_descriptor()) {
        appearance_.blend(obs.descriptor, profile_->appearance_momentum);
    }

    last_seen_ = obs.timestamp;
    ++n_hit_;

    obs_weights_.push_back(weight);
    if (obs_weights_.size() > kMaxHistory) obs_weights_.pop_front();

    pol_.add(obs.timestamp, pos);

    history_.push_back(TrackSample{obs.timestamp, pf_.position(), pf_.velocity()});
    if (history_.size() > kMaxHistory) history_.pop_front();

    last_obs_pos_ = pos;
    last_obs_ts_ = obs.timestamp;
    // The rate itself is updated in note_hit_scan, which is the only place
    // that knows which scan this detection belongs to.
}

void Track::update_miss(Real p_detect) {
    const Real pd = p_detect >= 0.0 ? p_detect : profile_->p_detection;
    const Real L = 1.0 - pd;
    r_ = std::clamp(r_ * L / (r_ * L + (1.0 - r_) + 1e-300), 0.0, 1.0);
    pi_r_ *= (1.0 - pd * kPossAlpha);
    ++n_miss_;
}

void Track::update_threat(Real score) {
    threat_ema_ = kThreatEmaAlpha * score + (1.0 - kThreatEmaAlpha) * threat_ema_;
    threat_history_.push_back(score);
    if (threat_history_.size() > kMaxThreatHistory) threat_history_.pop_front();
    if (score > kThreatPersistThresh) {
        ++threat_persistence_;
    } else {
        threat_persistence_ = std::max(0, threat_persistence_ - 1);
    }
}

void Track::note_hit_scan(int scan, const std::string& source) {
    // One scan counts once, however many sensors reported it in that scan.
    if (scan != last_hit_scan_) {
        last_hit_scan_ = scan;
        ++n_hit_scans_;
    }
    hit_scans_.push_back(HitRecord{scan, std::hash<std::string>{}(source)});
    if (hit_scans_.size() > 24) hit_scans_.pop_front();
}

bool Track::shares_hit_scan_with(const Track& other) const {
    // A scan in which one sensor fed both tracks proves they are distinct
    // entities, however close together they are moving: a sensor reports a
    // given entity once per scan. The same scan via two different sensors
    // proves nothing of the kind - that is one entity under overlapping
    // coverage, which is the case that must still be allowed to merge.
    for (const HitRecord& a : hit_scans_) {
        for (const HitRecord& b : other.hit_scans_) {
            if (a == b) return true;
        }
    }
    return false;
}

bool Track::shares_source_with(const Track& other) const {
    for (const HitRecord& a : hit_scans_) {
        for (const HitRecord& b : other.hit_scans_) {
            if (a.source == b.source) return true;
        }
    }
    return false;
}

void Track::absorb(const Track& other) {
    // Keep the richer identity: the surviving track inherits whichever
    // pattern-of-life baseline is better established, and the hit counts add,
    // because both were evidence about one entity all along.
    if (!pol_.fitted() && other.pol_.fitted()) {
        pol_.clone_from(other.pol_);
    }
    if (!appearance_.valid() && other.appearance_.valid()) {
        appearance_ = other.appearance_;
    }
    n_hit_ += other.n_hit_;
    // The merged track existed for the union of the two spans, so the scans it
    // was seen in is at most the sum and at least the larger. Taking the max is
    // the conservative reading: it never claims a detection rate the evidence
    // does not support.
    n_hit_scans_ = std::max(n_hit_scans_, other.n_hit_scans_);
    last_hit_scan_ = std::max(last_hit_scan_, other.last_hit_scan_);
    born_at_ = std::min(born_at_, other.born_at_);
    last_seen_ = std::max(last_seen_, other.last_seen_);
    r_ = std::max(r_, other.r_);
    pi_r_ = std::max(pi_r_, other.pi_r_);
    if (!parent_id_.has_value() && other.parent_id_.has_value()) {
        parent_id_ = other.parent_id_;
    }
}

Real Track::mean_observation_quality() const {
    if (obs_weights_.empty()) return 0.5;
    const Real sum = std::accumulate(obs_weights_.begin(), obs_weights_.end(), Real{0.0});
    return sum / static_cast<Real>(obs_weights_.size());
}

Vec2 Track::smoothed_velocity(std::size_t n) const {
    if (history_.empty()) return velocity();
    const std::size_t take = std::min(n, history_.size());
    Vec2 acc{};
    for (std::size_t i = history_.size() - take; i < history_.size(); ++i) {
        acc += history_[i].velocity;
    }
    return acc / static_cast<Real>(take);
}

Real Track::path_length(std::size_t n) const {
    if (history_.size() < 2) return 0.0;
    const std::size_t take = std::min(n, history_.size());
    const std::size_t start = history_.size() - take;
    Real acc = 0.0;
    for (std::size_t i = start + 1; i < history_.size(); ++i) {
        acc += distance(history_[i].position, history_[i - 1].position);
    }
    return acc;
}

Real Track::net_displacement(std::size_t n) const {
    if (history_.size() < 2) return 0.0;
    const std::size_t take = std::min(n, history_.size());
    const std::size_t start = history_.size() - take;
    return distance(history_.back().position, history_[start].position);
}

Real Track::winding(std::size_t n) const {
    if (history_.size() < 3) return 0.0;
    const std::size_t take = std::min(n, history_.size());
    const std::size_t start = history_.size() - take;

    Real total = 0.0;
    for (std::size_t i = start + 2; i < history_.size(); ++i) {
        const Vec2 a = history_[i - 1].position - history_[i - 2].position;
        const Vec2 b = history_[i].position - history_[i - 1].position;
        const Real na = a.norm();
        const Real nb = b.norm();
        if (na < 1e-6 || nb < 1e-6) continue;
        // Signed turn angle via cross and dot; summing keeps left and right
        // turns cancelling, so a straight zigzag does not look like a loop.
        const Real cross = a.x * b.y - a.y * b.x;
        const Real dot = a.dot(b);
        total += std::atan2(cross, dot);
    }
    return total;
}

}  // namespace trace
