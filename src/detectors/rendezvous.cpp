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

/// Least-squares velocity from a track's recent history, in metres per SCAN --
/// the intercept solve below works in scans, so this deliberately does not
/// convert to SI. More robust than the instantaneous filter velocity, which
/// jitters with each resample and would make the intercept time swing wildly.
Vec2 fitted_velocity(const Track& t, Real scan_dt, std::size_t window = 6) {
    const auto& h = t.history();
    if (h.size() < 3) return t.velocity() * scan_dt;

    const std::size_t take = std::min(window, h.size());
    const std::size_t start = h.size() - take;
    const auto n = static_cast<Real>(take);

    Real sum_i = 0.0, sum_ii = 0.0;
    Vec2 sum_p{}, sum_ip{};
    for (std::size_t k = 0; k < take; ++k) {
        const auto i = static_cast<Real>(k);
        const Vec2 p = h[start + k].position;
        sum_i += i;
        sum_ii += i * i;
        sum_p += p;
        sum_ip += p * i;
    }
    const Real denom = n * sum_ii - sum_i * sum_i;
    if (std::abs(denom) < 1e-9) return t.velocity() * scan_dt;
    return Vec2{(n * sum_ip.x - sum_i * sum_p.x) / denom,
                (n * sum_ip.y - sum_i * sum_p.y) / denom};
}

Priority priority_from_eta(Real eta_s, Real confidence) {
    if (eta_s < 300.0 && confidence > 0.5) return Priority::IMMEDIATE;
    if (eta_s < 900.0 && confidence > 0.4) return Priority::HIGH;
    if (confidence > 0.3) return Priority::MEDIUM;
    return Priority::LOW;
}

}  // namespace

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
    const int steps = std::min(
        static_cast<int>(p.rv_pol_window_s / std::max(p.scan_dt_s, 1e-6)),
        kPolMaxHorizonSteps);
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

    for (const auto& [i, j] : ctx.near_pairs(reach, tracks.size())) {
        {
            const Track& a = *tracks[i];
            const Track& b = *tracks[j];
            const Real sep = distance(a.position(), b.position());

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
    return out;
}

std::vector<DetectionEvent> RendezvousWarner::detect(
    const std::vector<TrackPtr>& /*tracks*/, const DetectorContext& /*ctx*/) {
    return {};  // this detector speaks through rendezvous(), not events
}

}  // namespace trace
