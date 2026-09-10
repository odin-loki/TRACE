#include "trace/core/threat.hpp"

#include <algorithm>
#include <cmath>

namespace trace {
namespace {

/// Tier thresholds, ported from the reference implementation.
constexpr Real kImmediate = 0.82;
constexpr Real kHigh = 0.62;
constexpr Real kMedium = 0.42;
constexpr Real kLow = 0.22;

/// Concentration multipliers per dimension. A larger multiplier means the
/// engine treats that evidence as more firmly established, so its Beta draw is
/// tighter. Existence gets the largest because it is the best-founded quantity
/// in the whole system; the mismatch penalty gets the smallest because it is a
/// heuristic diagnostic, not a measurement.
constexpr std::array<Real, 8> kConcentration{20.0, 8.0, 8.0, 8.0,
                                             6.0, 6.0, 8.0, 4.0};

Real percentile(std::vector<Real>& v, Real q) {
    if (v.empty()) return 0.0;
    const auto idx = static_cast<std::size_t>(
        std::clamp(q * static_cast<Real>(v.size() - 1), 0.0,
                   static_cast<Real>(v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + static_cast<long>(idx), v.end());
    return v[idx];
}

}  // namespace

Priority priority_for(Real score) {
    if (score >= kImmediate) return Priority::IMMEDIATE;
    if (score >= kHigh) return Priority::HIGH;
    if (score >= kMedium) return Priority::MEDIUM;
    if (score >= kLow) return Priority::LOW;
    return Priority::MONITOR;
}

ThreatScore score_track(Track& track, Real timestamp,
                        const std::vector<Vec2>& hvls, Real hvl_radius,
                        const DomainProfile& profile, Rng& rng) {
    const Vec2 pos = track.position();

    // ---- The eight evidence dimensions ------------------------------------
    const Real existence = track.existence();
    const Real pol = track.pol().fitted()
                         ? track.pol().anomaly_score(timestamp, pos)
                         : 0.5;
    const Real det_density = track.detection_density();

    // Proximity to a high-value location, decaying exponentially with range.
    Real hvl = 0.0;
    for (const Vec2& h : hvls) {
        hvl = std::max(hvl, std::exp(-distance(pos, h) / std::max(hvl_radius, 1.0)));
    }

    // Speed relative to the domain's own scale rather than a fixed 30 m/s: a
    // vessel and a sprinter should not be measured against the same yardstick.
    const Real speed = track.speed_mps(profile.scan_dt_s);
    const Real speed_scale = std::max(profile.courier_speed_thresh * 4.0, 1e-3);
    const Real motion = std::clamp(speed / speed_scale, 0.0, 1.0);

    const Real persistence =
        std::clamp(static_cast<Real>(track.threat_persistence()) / 10.0, 0.0, 1.0);
    const Real ema = std::clamp(track.threat_ema(), 0.0, 1.0);
    const Real mismatch_penalty =
        std::clamp(1.0 - track.possibility_mismatch(), 0.0, 1.0);

    const std::array<Real, 8> evidence{existence, pol,         det_density, hvl,
                                       motion,    persistence, ema,         mismatch_penalty};

    // ---- Beta-Monte-Carlo integration -------------------------------------
    // Each dimension becomes Beta(e*c + 1, (1-e)*c + 1): the evidence sets the
    // mean, the concentration sets how sure we are of it.
    const int n_mc = std::max(profile.threat_mc_samples, 16);
    std::vector<Real> scores(static_cast<std::size_t>(n_mc));

    for (int s = 0; s < n_mc; ++s) {
        Real acc = 0.0;
        for (std::size_t d = 0; d < 8; ++d) {
            const Real c = kConcentration[d];
            const Real a = evidence[d] * c + 1.0;
            const Real b = (1.0 - evidence[d]) * c + 1.0;
            acc += profile.threat_weights[d] * rng.beta(a, b);
        }
        scores[static_cast<std::size_t>(s)] = acc;
    }

    Real mean = 0.0;
    for (const Real s : scores) mean += s;
    mean /= static_cast<Real>(scores.size());

    Real var = 0.0;
    for (const Real s : scores) var += (s - mean) * (s - mean);
    var /= static_cast<Real>(scores.size());

    ThreatScore out;
    out.mean = mean;
    out.stddev = std::sqrt(var);
    out.p90 = percentile(scores, 0.90);
    out.p95 = percentile(scores, 0.95);
    out.priority = priority_for(mean);

    track.update_threat(mean);
    out.ema = track.threat_ema();
    out.persistence = track.threat_persistence();

    out.breakdown = ThreatBreakdown{existence,   track.possibility(),
                                    pol,         det_density,
                                    hvl,         motion,
                                    persistence, mismatch_penalty};
    return out;
}

Credibility fuse_credibility(const std::vector<Observation>& evidence,
                             const DomainProfile& profile) {
    Credibility out;
    if (evidence.empty()) return out;

    // Dempster's rule over {hypothesis, negation, uncertain}. Tracking the
    // conflict mass K separately is what distinguishes "we have little
    // evidence" from "our sources contradict each other".
    Real m_h = 1.0, m_not_h = 1.0, m_theta = 1.0;
    Real conflict = 0.0;

    const std::size_t start = evidence.size() > 8 ? evidence.size() - 8 : 0;
    for (std::size_t i = start; i < evidence.size(); ++i) {
        const auto& obs = evidence[i];
        const Real r = profile.modality_weight(obs.modality) * obs.confidence;
        const Real mh = r * 0.85;
        const Real mnh = (1.0 - r) * 0.10;
        const Real mt = 1.0 - mh - mnh;

        Real k = m_h * mnh + m_not_h * mh;
        k = std::min(k, 0.999);
        const Real denom = 1.0 - k;

        const Real new_h = (m_h * mh + m_h * mt + m_theta * mh) / denom;
        const Real new_not_h = (m_not_h * mnh + m_not_h * mt + m_theta * mnh) / denom;
        const Real new_theta = (m_theta * mt) / denom;

        m_h = new_h;
        m_not_h = new_not_h;
        m_theta = new_theta;
        conflict = k;
    }

    out.belief = std::clamp(m_h, 0.0, 1.0);
    out.plausibility = std::clamp(m_h + m_theta, 0.0, 1.0);
    out.conflict = std::clamp(conflict, 0.0, 1.0);
    return out;
}

}  // namespace trace
