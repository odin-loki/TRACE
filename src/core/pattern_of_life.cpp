#include "trace/core/pattern_of_life.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace trace {
namespace {

constexpr int kEmIterations = 35;
constexpr std::size_t kMaxHistory = 300;   // EM window
constexpr std::size_t kBaselineTail = 40;  // sightings used to calibrate "normal"
constexpr Real kCovRegular = 5e-3;

Real log_sum_exp(const std::vector<Real>& v) {
    if (v.empty()) return -std::numeric_limits<Real>::infinity();
    const Real m = *std::max_element(v.begin(), v.end());
    if (!std::isfinite(m)) return m;
    Real acc = 0.0;
    for (const Real x : v) acc += std::exp(x - m);
    return m + std::log(acc);
}

}  // namespace

Chol3 Chol3::factor(const std::array<std::array<Real, kPolDim>, kPolDim>& A,
                    Real ridge) {
    Chol3 out;
    for (int attempt = 0; attempt < 8; ++attempt) {
        const Real r = ridge * std::pow(10.0, attempt);
        bool ok = true;
        std::array<std::array<Real, kPolDim>, kPolDim> L{};

        for (int i = 0; i < kPolDim && ok; ++i) {
            for (int j = 0; j <= i; ++j) {
                Real sum = A[i][j] + (i == j ? r : 0.0);
                for (int k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
                if (i == j) {
                    if (sum <= 0.0) { ok = false; break; }
                    L[i][j] = std::sqrt(sum);
                } else {
                    L[i][j] = sum / L[j][j];
                }
            }
        }
        if (ok) {
            out.L = L;
            out.log_det = 0.0;
            for (int i = 0; i < kPolDim; ++i) out.log_det += 2.0 * std::log(L[i][i]);
            out.valid = true;
            return out;
        }
    }
    // Fall back to a wide isotropic component rather than propagating a NaN.
    out.L = {};
    for (int i = 0; i < kPolDim; ++i) out.L[i][i] = 1.0;
    out.log_det = 0.0;
    out.valid = false;
    return out;
}

Vec3 Chol3::forward_solve(const Vec3& b) const {
    Vec3 y{};
    for (int i = 0; i < kPolDim; ++i) {
        Real sum = b[i];
        for (int k = 0; k < i; ++k) sum -= L[i][k] * y[k];
        y[i] = sum / (L[i][i] != 0.0 ? L[i][i] : 1.0);
    }
    return y;
}

Vec3 Chol3::apply(const Vec3& z) const {
    Vec3 out{};
    for (int i = 0; i < kPolDim; ++i) {
        Real sum = 0.0;
        for (int k = 0; k <= i; ++k) sum += L[i][k] * z[k];
        out[i] = sum;
    }
    return out;
}

PatternOfLife::PatternOfLife(const DomainProfile& profile)
    : profile_(&profile),
      k_(profile.pol_components),
      min_obs_(profile.pol_min_obs),
      refit_interval_(profile.pol_refit_interval) {}

void PatternOfLife::add(Real timestamp, Vec2 position) {
    obs_.push_back(featurise(timestamp, position));
    if (obs_.size() > kMaxHistory * 2) {
        obs_.erase(obs_.begin(), obs_.begin() + static_cast<long>(kMaxHistory));
    }
    ++obs_since_refit_;
    if (static_cast<int>(obs_.size()) >= min_obs_ &&
        obs_since_refit_ >= refit_interval_) {
        em_fit();
        obs_since_refit_ = 0;
    }
}

void PatternOfLife::clone_from(const PatternOfLife& other) {
    obs_ = other.obs_;
    components_ = other.components_;
    baseline_nll_ = other.baseline_nll_;
    fitted_ = other.fitted_;
    obs_since_refit_ = 0;
    if (other.profile_ != nullptr) {
        profile_ = other.profile_;
        k_ = other.k_;
        min_obs_ = other.min_obs_;
        refit_interval_ = other.refit_interval_;
    }
}

void PatternOfLife::em_fit() {
    const std::size_t take = std::min(obs_.size(), kMaxHistory);
    const std::vector<Vec3> data(obs_.end() - static_cast<long>(take), obs_.end());
    const std::size_t n = data.size();

    // One component per three sightings at most, so K never outruns the data.
    const int k = std::min(k_, static_cast<int>(n / 3));
    if (k < 1) return;

    // k-means++ seeding: spread the initial centres by squared distance.
    std::vector<Vec3> means;
    means.push_back(data[rng_.uniform_int(0, static_cast<int>(n) - 1)]);
    std::vector<Real> d2(n, 0.0);
    for (int c = 1; c < k; ++c) {
        for (std::size_t i = 0; i < n; ++i) {
            Real best = std::numeric_limits<Real>::infinity();
            for (const auto& m : means) {
                Real acc = 0.0;
                for (int d = 0; d < kPolDim; ++d) {
                    const Real diff = data[i][d] - m[d];
                    acc += diff * diff;
                }
                best = std::min(best, acc);
            }
            d2[i] = best;
        }
        means.push_back(data[rng_.categorical(d2.data(), n)]);
    }

    std::vector<PolComponent> comps(static_cast<std::size_t>(k));
    for (int c = 0; c < k; ++c) {
        comps[c].mean = means[c];
        // Hours vary over ~1h, positions over ~500m: a diagonal prior with
        // those scales keeps the first E-step from collapsing on one axis.
        comps[c].cov = {};
        comps[c].cov[0][0] = 1.0;
        comps[c].cov[1][1] = 500.0;
        comps[c].cov[2][2] = 500.0;
        comps[c].weight = 1.0 / k;
        comps[c].chol = Chol3::factor(comps[c].cov);
    }

    std::vector<std::vector<Real>> resp(n, std::vector<Real>(k, 0.0));

    for (int iter = 0; iter < kEmIterations; ++iter) {
        // ---- E step -------------------------------------------------------
        for (std::size_t i = 0; i < n; ++i) {
            std::vector<Real> lp(static_cast<std::size_t>(k));
            for (int c = 0; c < k; ++c) {
                const auto& comp = comps[c];
                Vec3 diff{};
                for (int d = 0; d < kPolDim; ++d) diff[d] = data[i][d] - comp.mean[d];
                const Vec3 y = comp.chol.forward_solve(diff);
                Real maha = 0.0;
                for (int d = 0; d < kPolDim; ++d) maha += y[d] * y[d];
                lp[c] = -0.5 * (maha + comp.chol.log_det +
                                kPolDim * std::log(2.0 * std::numbers::pi)) +
                        std::log(comp.weight + 1e-300);
            }
            const Real norm = log_sum_exp(lp);
            for (int c = 0; c < k; ++c) resp[i][c] = std::exp(lp[c] - norm);
        }

        // ---- M step -------------------------------------------------------
        for (int c = 0; c < k; ++c) {
            Real nk = 1e-6;
            for (std::size_t i = 0; i < n; ++i) nk += resp[i][c];

            Vec3 mean{};
            for (std::size_t i = 0; i < n; ++i) {
                for (int d = 0; d < kPolDim; ++d) mean[d] += resp[i][c] * data[i][d];
            }
            for (int d = 0; d < kPolDim; ++d) mean[d] /= nk;

            std::array<std::array<Real, kPolDim>, kPolDim> cov{};
            for (std::size_t i = 0; i < n; ++i) {
                Vec3 diff{};
                for (int d = 0; d < kPolDim; ++d) diff[d] = data[i][d] - mean[d];
                for (int a = 0; a < kPolDim; ++a) {
                    for (int b = 0; b < kPolDim; ++b) {
                        cov[a][b] += resp[i][c] * diff[a] * diff[b];
                    }
                }
            }
            for (int a = 0; a < kPolDim; ++a) {
                for (int b = 0; b < kPolDim; ++b) cov[a][b] /= nk;
                cov[a][a] += kCovRegular;
            }

            comps[c].mean = mean;
            comps[c].cov = cov;
            comps[c].weight = nk / static_cast<Real>(n);
            // Refactoring every iteration is wasted work; the E step only
            // needs an up-to-date factor periodically and at the end.
            if (iter % 5 == 4 || iter == kEmIterations - 1) {
                comps[c].chol = Chol3::factor(cov);
            }
        }
    }

    components_ = std::move(comps);
    fitted_ = true;

    // Calibrate "normal" against the most recent sightings, so the anomaly
    // score is relative to this entity's own recent behaviour.
    const std::size_t tail = std::min(data.size(), kBaselineTail);
    Real acc = 0.0;
    for (std::size_t i = data.size() - tail; i < data.size(); ++i) {
        acc += log_prob(data[i]);
    }
    baseline_nll_ = -acc / static_cast<Real>(tail);
}

Real PatternOfLife::log_prob(const Vec3& x) const {
    if (components_.empty()) return -4.0;
    std::vector<Real> lp(components_.size());
    for (std::size_t c = 0; c < components_.size(); ++c) {
        const auto& comp = components_[c];
        Vec3 diff{};
        for (int d = 0; d < kPolDim; ++d) diff[d] = x[d] - comp.mean[d];
        const Vec3 y = comp.chol.forward_solve(diff);
        Real maha = 0.0;
        for (int d = 0; d < kPolDim; ++d) maha += y[d] * y[d];
        lp[c] = -0.5 * (maha + comp.chol.log_det +
                        kPolDim * std::log(2.0 * std::numbers::pi)) +
                std::log(comp.weight + 1e-300);
    }
    return log_sum_exp(lp);
}

Real PatternOfLife::anomaly_score(Real timestamp, Vec2 position) const {
    if (!fitted_) return 0.5;
    const Real nll = -log_prob(featurise(timestamp, position));
    // Normalise the surprise against the entity's own baseline, then squash.
    // Dividing by the baseline is what makes one threshold work for a courier
    // criss-crossing a city and a moored vessel.
    const Real x = (nll - baseline_nll_) / std::max(std::abs(baseline_nll_), 1.0);
    return std::clamp(1.0 / (1.0 + std::exp(-x)), 0.0, 1.0);
}

PatternOfLife::Prediction PatternOfLife::predict_location(Real timestamp,
                                                          Rng& rng,
                                                          int n_mc) const {
    Prediction out;
    if (!fitted_ || components_.empty()) return out;

    Real hour = std::fmod(timestamp, 86400.0) / 3600.0;
    if (hour < 0.0) hour += 24.0;

    // Reweight components by how well this hour matches each one, so the
    // prediction is "where is he at 09:00" not "where is he on average".
    //
    // The component's mixing weight enters ONCE. It used to enter twice: added
    // as `log(weight)` when building the log-weights, and multiplied in again
    // when exponentiating them - so a component's influence went as the square
    // of its weight, and the hour-conditioned prediction collapsed towards
    // whichever component was heaviest overall rather than whichever one
    // explains this hour.
    const std::size_t k = components_.size();
    std::vector<Real> lw(k);
    for (std::size_t c = 0; c < k; ++c) {
        const Vec3 probe{hour, components_[c].mean[1], components_[c].mean[2]};
        lw[c] = log_prob(probe) + std::log(components_[c].weight + 1e-300);
    }
    const Real norm = log_sum_exp(lw);
    std::vector<Real> w(k);
    Real wsum = 0.0;
    for (std::size_t c = 0; c < k; ++c) {
        w[c] = std::exp(lw[c] - norm);
        wsum += w[c];
    }
    if (wsum <= 0.0) return out;
    for (auto& v : w) v /= wsum;

    std::vector<Vec2> samples;
    samples.reserve(static_cast<std::size_t>(n_mc));
    for (int i = 0; i < n_mc; ++i) {
        const std::size_t c = rng.categorical(w.data(), k);
        const Vec3 z{rng.normal(), rng.normal(), rng.normal()};
        const Vec3 shift = components_[c].chol.apply(z);
        samples.push_back(Vec2{components_[c].mean[1] + shift[1],
                               components_[c].mean[2] + shift[2]});
    }

    Vec2 mean{};
    for (const auto& s : samples) mean += s;
    mean = mean / static_cast<Real>(samples.size());

    Real var = 0.0;
    for (const auto& s : samples) {
        const Real d = distance(s, mean);
        var += d * d;
    }
    out.position = mean;
    out.uncertainty_m = std::sqrt(var / static_cast<Real>(samples.size()));
    return out;
}

std::vector<std::pair<Real, Real>> PatternOfLife::active_windows() const {
    std::vector<std::pair<Real, Real>> out;
    if (!fitted_ || components_.empty()) return out;
    const Real min_weight = 0.5 / static_cast<Real>(components_.size());
    for (const auto& c : components_) {
        if (c.weight <= min_weight) continue;
        const Real centre = c.mean[0];
        const Real spread = std::sqrt(std::max(c.cov[0][0], 0.0));
        out.emplace_back(centre - spread, centre + spread);
    }
    return out;
}

Real PatternOfLife::spatial_spread() const {
    if (!fitted_ || components_.empty()) return 0.0;
    Real acc = 0.0;
    for (const auto& c : components_) {
        acc += c.weight * std::sqrt(std::max(c.cov[1][1] + c.cov[2][2], 0.0));
    }
    return acc;
}

}  // namespace trace
