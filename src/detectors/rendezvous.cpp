// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

#include "trace/detectors/detectors.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace trace {
namespace {

constexpr std::size_t kMaxSepHistory = 24;
constexpr int kPolMaxHorizonSteps = 20;
constexpr int kPolMonteCarlo = 8;

std::pair<std::string, std::string> pair_key(const std::string& a,
                                             const std::string& b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

Priority priority_from_eta(Real eta_s, Real confidence) {
    if (eta_s < 300.0 && confidence > 0.5) return Priority::IMMEDIATE;
    if (eta_s < 900.0 && confidence > 0.4) return Priority::HIGH;
    if (confidence > 0.3) return Priority::MEDIUM;
    return Priority::LOW;
}

}  // namespace

/// Least-squares velocity from a track's recent history, in metres per SCAN --
/// the intercept solve below works in scans, so this converts at the end rather
/// than returning SI. More robust than the instantaneous filter velocity, which
/// jitters with each resample and would make the intercept time swing wildly.
///
/// The regression is against each sample's TIMESTAMP, not its index in the
/// history. `Track::history_` is appended once per accepted detection, not once
/// per scan, so the index is not a clock: a track under two sensors gets two
/// samples per scan at the same instant, and one that goes unseen for a while
/// gets none at all. Regressing against the index then reads a slope of metres
/// per sample and calls it metres per scan.
///
/// Measured on a 2 m/s target, truth 120 m per 60 s scan: one sensor gave
/// 123 m/scan, two gave 45.8 and four gave 31.0 - under-reading the speed by
/// nearly four times under the overlapping coverage this detector is most
/// likely to be used in. Convergence ETAs are computed from this, so the
/// warning said an hour where the truth was a quarter of that.
///
/// Regressing against the timestamp removes the bias, and taking the window as
/// a span of TIME rather than a count of samples removes what was left. The
/// same three cases now read 1.01, 1.02 and 1.03 times the truth on a
/// vectorised build and 0.96, 1.01 and 1.00 on a scalar one - where with a
/// sample-count window they read 1.13 and 1.75 respectively for four sensors,
/// which is how the difference was noticed at all.
Vec2 fitted_velocity(const Track& t, Real scan_dt, std::size_t window) {
    const auto& h = t.history();
    if (h.size() < 3) return t.velocity() * scan_dt;

    // `window` counts SCANS, not samples. History is appended once per
    // detection, so a sample count spans however many scans the sensors
    // happened to supply: six samples is six scans under one sensor and a
    // scan and a half under four. A velocity fit over a scan and a half is
    // dominated by measurement noise, and the answer then depends on how many
    // sensors are watching and even on the SIMD width the binary was compiled
    // for - the scalar build read a four-sensor target at 1.75x its true speed
    // where the vectorised one read 1.13x.
    //
    // Taking a fixed span of TIME instead makes the fit mean the same thing
    // whoever is watching. Everything inside the span is used, so more sensors
    // now buy a better-conditioned fit rather than a shorter one.
    const Real span = static_cast<Real>(window) * scan_dt;
    const Real newest = h.back().timestamp;
    std::size_t start = h.size();
    while (start > 0 && newest - h[start - 1].timestamp <= span) --start;
    // Three points minimum, whatever the span says, or there is no line to fit.
    if (h.size() - start < 3) start = h.size() - std::min<std::size_t>(3, h.size());
    const std::size_t take = h.size() - start;
    const auto n = static_cast<Real>(take);

    // Times relative to the first sample kept, so the normal equations stay
    // well conditioned whatever the absolute timestamps are.
    const Real t0 = h[start].timestamp;

    Real sum_t = 0.0, sum_tt = 0.0;
    Vec2 sum_p{}, sum_tp{};
    for (std::size_t k = 0; k < take; ++k) {
        const Real ti = h[start + k].timestamp - t0;
        const Vec2 p = h[start + k].position;
        sum_t += ti;
        sum_tt += ti * ti;
        sum_p += p;
        sum_tp += p * ti;
    }
    const Real denom = n * sum_tt - sum_t * sum_t;
    // Degenerate when every sample shares a timestamp, which is exactly what
    // several sensors reporting one scan produces. There is no slope to fit
    // through a single instant, so fall back rather than invent one.
    if (std::abs(denom) < 1e-9) return t.velocity() * scan_dt;

    // Metres per second from the fit, then to metres per scan for the caller.
    return Vec2{(n * sum_tp.x - sum_t * sum_p.x) / denom,
                (n * sum_tp.y - sum_t * sum_p.y) / denom} * scan_dt;
}

// ---------------------------------------------------------------------------
// Method 1 — geometric intercept
// ---------------------------------------------------------------------------
// Treat both tracks as constant-velocity rays and solve for the time of closest
// approach analytically. Exact when people are actually walking towards each
// other; useless when either is manoeuvring.
std::optional<RendezvousWarning> RendezvousWarner::geometric_intercept(
    const Track& a, const Track& b, const DetectorContext& ctx) const {
    const DomainProfile& p = *ctx.profile;
    if (a.history().size() < 3 || b.history().size() < 3) return std::nullopt;

    const Vec2 va = fitted_velocity(a, p.scan_dt_s);
    const Vec2 vb = fitted_velocity(b, p.scan_dt_s);
    const Vec2 dv = va - vb;
    const Vec2 dp = a.position() - b.position();

    const Real dv2 = dv.norm_sq();
    if (dv2 < 1e-6) return std::nullopt;  // parallel: never converge

    const Real t_cpa = -dp.dot(dv) / dv2;  // in scans
    if (t_cpa <= 0.0) return std::nullopt;  // already diverging

    const Real eta_s = t_cpa * p.scan_dt_s;
    if (eta_s > p.rv_warning_horizon_s) return std::nullopt;

    const Vec2 a_cpa = a.position() + va * t_cpa;
    const Vec2 b_cpa = b.position() + vb * t_cpa;
    const Real cpa_sep = distance(a_cpa, b_cpa);
    if (cpa_sep > p.rv_threshold_m * 2.0) return std::nullopt;

    RendezvousWarning w;
    w.track_a = a.id();
    w.track_b = b.id();
    w.eta_s = eta_s;
    w.current_sep_m = dp.norm();
    w.method = "GEOMETRIC_INTERCEPT";
    w.confidence = std::clamp(1.0 - cpa_sep / p.rv_threshold_m, 0.1, 1.0);
    w.location = (a_cpa + b_cpa) * 0.5;
    w.priority = priority_from_eta(eta_s, w.confidence);
    return w;
}

// ---------------------------------------------------------------------------
// Method 2 — closure-rate extrapolation
// ---------------------------------------------------------------------------
// Ignore the geometry; fit a line to the separation series and ask when it
// reaches contact range. Catches convergence along a curving route, where the
// straight-line intercept gives nothing.
std::optional<RendezvousWarning> RendezvousWarner::separation_rate(
    const Track& a, const Track& b, const std::deque<SepSample>& hist,
    const DetectorContext& ctx) const {
    const DomainProfile& p = *ctx.profile;
    const std::size_t window =
        std::min(hist.size(), static_cast<std::size_t>(p.rv_sep_rate_window));
    if (window < 3) return std::nullopt;

    const std::size_t start = hist.size() - window;
    const auto n = static_cast<Real>(window);

    Real sum_t = 0.0, sum_tt = 0.0, sum_s = 0.0, sum_ts = 0.0;
    const Real t0 = hist[start].timestamp;
    for (std::size_t k = start; k < hist.size(); ++k) {
        const Real t = (hist[k].timestamp - t0) / p.scan_dt_s;
        const Real s = hist[k].separation;
        sum_t += t;
        sum_tt += t * t;
        sum_s += s;
        sum_ts += t * s;
    }
    const Real denom = n * sum_tt - sum_t * sum_t;
    if (std::abs(denom) < 1e-9) return std::nullopt;

    const Real slope = (n * sum_ts - sum_t * sum_s) / denom;  // metres per scan
    if (slope >= 0.0) return std::nullopt;                    // separating

    const Real intercept = (sum_s - slope * sum_t) / n;
    const Real current_sep = hist.back().separation;

    Real scans_to_rv = (current_sep - p.rv_threshold_m) / (-slope);
    if (scans_to_rv <= 0.0) scans_to_rv = 1.0;
    const Real eta_s = scans_to_rv * p.scan_dt_s;
    if (eta_s > p.rv_warning_horizon_s) return std::nullopt;

    // Confidence is the fit's R^2: a noisy separation series should not be
    // extrapolated with the same conviction as a clean one.
    Real ss_res = 0.0, ss_tot = 0.0;
    const Real mean_s = sum_s / n;
    for (std::size_t k = start; k < hist.size(); ++k) {
        const Real t = (hist[k].timestamp - t0) / p.scan_dt_s;
        const Real pred = slope * t + intercept;
        const Real resid = hist[k].separation - pred;
        ss_res += resid * resid;
        ss_tot += (hist[k].separation - mean_s) * (hist[k].separation - mean_s);
    }
    const Real r2 = 1.0 - ss_res / (ss_tot + 1e-9);

    RendezvousWarning w;
    w.track_a = a.id();
    w.track_b = b.id();
    w.eta_s = eta_s;
    w.current_sep_m = current_sep;
    w.method = "SEPARATION_RATE";
    w.confidence = std::clamp(r2, 0.05, 0.99);
    w.location = (a.position() + b.position()) * 0.5;
    w.priority = priority_from_eta(eta_s, w.confidence);
    return w;
}

// ---------------------------------------------------------------------------
// Method 3 — pattern-of-life cross-prediction
// ---------------------------------------------------------------------------
// Neither kinematic method can see a meeting that has not started yet. This one
// asks each entity's learned routine where it will be at a series of future
// times, and looks for a moment when both routines put them in the same place.
// It is the only method that fires while the two are still stationary.
std::optional<RendezvousWarning> RendezvousWarner::pol_cross_predict(
    const Track& a, const Track& b, const PolForecast& fa, const PolForecast& fb,
    const DetectorContext& ctx) const {
    const DomainProfile& p = *ctx.profile;
    if (!fa.valid || !fb.valid) return std::nullopt;

    const std::size_t steps = std::min(fa.position.size(), fb.position.size());
    for (std::size_t k = 0; k < steps; ++k) {
        const Real sep = distance(fa.position[k], fb.position[k]);
        if (sep >= p.rv_threshold_m) continue;

        const Real total_unc = fa.uncertainty[k] + fb.uncertainty[k];
        RendezvousWarning w;
        w.track_a = a.id();
        w.track_b = b.id();
        w.eta_s = static_cast<Real>(k + 1) * p.scan_dt_s;
        w.current_sep_m = distance(a.position(), b.position());
        w.method = "POL_CROSS_PREDICT";
        w.confidence =
            std::clamp(1.0 - total_unc / (p.rv_threshold_m * 4.0), 0.1, 0.95);
        w.location = (fa.position[k] + fb.position[k]) * 0.5;
        w.priority = priority_from_eta(w.eta_s, w.confidence);
        return w;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------

std::vector<RendezvousWarning> RendezvousWarner::rendezvous(
    const std::vector<TrackPtr>& tracks, const DetectorContext& ctx) {
    std::vector<RendezvousWarning> out;
    const DomainProfile& p = *ctx.profile;

    // Each track's pattern-of-life forecast, computed once and shared across
    // every pair it takes part in.
    // Clamped as a double BEFORE the cast. With the 1e-6 floor on the scan
    // period, the default one-hour window divides out to 3.6e9, which does not
    // fit in an int, and the conversion is undefined - so a profile with a
    // degenerate scan period took an undefined branch rather than a short one.
    //
    // Note also what `kPolMaxHorizonSteps` does to `rv_pol_window_s`: the
    // lookahead is min(rv_pol_window_s, 20 * scan_dt_s), so under any profile
    // scanning faster than once every three minutes the 20-step cap decides and
    // the documented window does not. On CityCameraSurveillance that is 20
    // seconds against the hour the profile asks for. The cap is deliberate -
    // each step is a Monte-Carlo forecast per track - but the knob should not
    // read as though it were in charge.
    const Real raw_steps = p.rv_pol_window_s / std::max(p.scan_dt_s, 1e-6);
    const int steps = static_cast<int>(
        std::min(raw_steps, static_cast<Real>(kPolMaxHorizonSteps)));
    std::vector<PolForecast> forecasts(tracks.size());
    if (steps >= 1 && ctx.rng != nullptr) {
        for (std::size_t i = 0; i < tracks.size(); ++i) {
            if (!tracks[i]->pol().fitted()) continue;
            auto& f = forecasts[i];
            f.position.reserve(static_cast<std::size_t>(steps));
            f.uncertainty.reserve(static_cast<std::size_t>(steps));
            for (int k = 1; k <= steps; ++k) {
                const auto pred = tracks[i]->pol().predict_location(
                    ctx.timestamp + k * p.scan_dt_s, *ctx.rng, kPolMonteCarlo);
                f.position.push_back(pred.position);
                f.uncertainty.push_back(pred.uncertainty_m);
            }
            f.valid = true;
        }
    }

    // Two entities cannot meet inside the warning horizon unless they are
    // already within reach of each other at the domain's own speed scale.
    // Walking every pair to discover that was the single largest cost in the
    // engine at realistic track counts.
    const Real reach = p.rv_threshold_m +
                       p.courier_speed_thresh * 4.0 * p.rv_warning_horizon_s;

    // That bound uses four times the domain's speed scale for both parties
    // over the whole horizon, which in a dense scene is wider than the scene:
    // the index then returns every pair and the gate does nothing. Measured at
    // 400 tracks this detector was still 56% of the engine. Each pair's own
    // speeds give a far tighter bound, and it costs two norms to apply - so
    // apply it before anything that allocates, which the separation history
    // does, twice, for every pair on every scan.
    std::vector<Real> speeds(tracks.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        speeds[i] = tracks[i]->speed_mps(p.scan_dt_s);
    }

    for (const auto& [i, j] : ctx.near_pairs(reach, tracks.size())) {
        {
            const Track& a = *tracks[i];
            const Track& b = *tracks[j];
            const Real sep = distance(a.position(), b.position());

            // Closing at their combined speed is the fastest they can converge.
            const Real closing_reach =
                p.rv_threshold_m +
                (speeds[i] + speeds[j]) * p.rv_warning_horizon_s;
            if (sep > closing_reach) continue;

            auto& hist = sep_history_[pair_key(a.id(), b.id())];
            hist.push_back(SepSample{ctx.timestamp, sep});
            if (hist.size() > kMaxSepHistory) hist.pop_front();

            // Run all three, keep the most confident. They disagree often, and
            // the disagreement is informative: a geometric hit with a low-R^2
            // separation fit means someone changed course.
            std::optional<RendezvousWarning> best;
            const std::array<std::optional<RendezvousWarning>, 3> candidates{
                geometric_intercept(a, b, ctx),
                separation_rate(a, b, hist, ctx),
                pol_cross_predict(a, b, forecasts[i], forecasts[j], ctx)};

            int n_agree = 0;
            for (const auto& c : candidates) {
                if (!c.has_value()) continue;
                ++n_agree;
                if (!best.has_value() || c->confidence > best->confidence) best = c;
            }
            if (!best.has_value()) continue;

            // Independent methods agreeing is real corroboration, so it lifts
            // confidence; but never past certainty.
            if (n_agree > 1) {
                best->confidence =
                    std::min(0.99, best->confidence * (1.0 + 0.15 * (n_agree - 1)));
                best->method += "+" + std::to_string(n_agree - 1);
            }
            if (best->eta_s <= p.rv_warning_horizon_s) out.push_back(*best);
        }
    }

    std::sort(out.begin(), out.end(),
              [](const RendezvousWarning& x, const RendezvousWarning& y) {
                  return x.eta_s < y.eta_s;
              });
    // A pair that has stopped being reported stops being a pair. Without this
    // the history keeps a deque for every pair of tracks that has ever been
    // near another, which in a long run is every pair that has ever existed.
    //
    // Swept periodically rather than every scan: an entry cannot go stale in
    // less than the history window anyway, and walking the whole map each scan
    // cost more than the memory it reclaimed - 12 ms of a 126 ms scan at 400
    // tracks, which is worse than the leak.
    if (++scans_since_prune_ >= static_cast<int>(kMaxSepHistory)) {
        scans_since_prune_ = 0;
        const Real stale_before =
            ctx.timestamp - static_cast<Real>(kMaxSepHistory) * p.scan_dt_s;
        for (auto it = sep_history_.begin(); it != sep_history_.end();) {
            it = (it->second.empty() || it->second.back().timestamp < stale_before)
                     ? sep_history_.erase(it)
                     : std::next(it);
        }
    }

    return out;
}

std::vector<DetectionEvent> RendezvousWarner::detect(
    const std::vector<TrackPtr>& /*tracks*/, const DetectorContext& /*ctx*/) {
    return {};  // this detector speaks through rendezvous(), not events
}

}  // namespace trace
