#include "trace/core/network.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <stack>

namespace trace {
namespace {

constexpr std::size_t kBcHistory = 20;
constexpr Real kEscalationThreshold = 0.72;
constexpr std::size_t kEscalationWindow = 5;

}  // namespace

std::vector<Real> betweenness_centrality(
    const std::vector<std::vector<std::size_t>>& adjacency) {
    const std::size_t n = adjacency.size();
    std::vector<Real> bc(n, 0.0);
    if (n < 3) return bc;

    // Reused across sources so the inner loop allocates nothing.
    std::vector<Real> sigma(n), delta(n);
    std::vector<long> dist(n);
    std::vector<std::vector<std::size_t>> preds(n);
    std::vector<std::size_t> order;
    order.reserve(n);
    std::queue<std::size_t> q;

    for (std::size_t s = 0; s < n; ++s) {
        // An isolated vertex lies on no shortest path between others.
        if (adjacency[s].empty()) continue;

        std::fill(sigma.begin(), sigma.end(), 0.0);
        std::fill(delta.begin(), delta.end(), 0.0);
        std::fill(dist.begin(), dist.end(), -1);
        for (auto& p : preds) p.clear();
        order.clear();

        sigma[s] = 1.0;
        dist[s] = 0;
        q.push(s);

        while (!q.empty()) {
            const std::size_t v = q.front();
            q.pop();
            order.push_back(v);
            // Only actual neighbours, rather than every vertex in the graph.
            for (const std::size_t w : adjacency[v]) {
                if (dist[w] < 0) {
                    dist[w] = dist[v] + 1;
                    q.push(w);
                }
                if (dist[w] == dist[v] + 1) {
                    sigma[w] += sigma[v];
                    preds[w].push_back(v);
                }
            }
        }

        for (auto it = order.rbegin(); it != order.rend(); ++it) {
            const std::size_t w = *it;
            for (const std::size_t v : preds[w]) {
                if (sigma[w] > 0.0) {
                    delta[v] += sigma[v] / sigma[w] * (1.0 + delta[w]);
                }
            }
            if (w != s) bc[w] += delta[w];
        }
    }

    // Normalise to [0,1] for an undirected graph.
    const Real denom = std::max(static_cast<Real>((n - 1) * (n - 2)) / 2.0, 1.0);
    for (auto& v : bc) v /= denom;
    return bc;
}

std::vector<Cluster> NetworkAnalyser::analyse(const std::vector<TrackPtr>& tracks,
                                              Real /*timestamp*/,
                                              const SpatialIndex* index) {
    latest_betweenness_.clear();
    if (tracks.size() < 2) return {};

    // Accumulate contact weight over time so a network is built from repeated
    // proximity, not from one scan's geometry.
    // Only pairs already within the co-location radius can contribute, so the
    // index answers this directly instead of the all-pairs walk it replaces.
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    if (index != nullptr && index->size() == tracks.size()) {
        candidates = index->pairs_within(coloc_dist_);
    } else {
        for (std::size_t i = 0; i < tracks.size(); ++i) {
            for (std::size_t j = i + 1; j < tracks.size(); ++j) candidates.emplace_back(i, j);
        }
    }
    // Age the existing graph before adding this scan's contacts, and drop
    // edges that have faded past the point of meaning anything. An edge only
    // stays if the proximity keeps recurring.
    constexpr Real kEdgeFloor = 0.05;
    for (auto row = adjacency_.begin(); row != adjacency_.end();) {
        for (auto e = row->second.begin(); e != row->second.end();) {
            e->second *= decay_;
            e = e->second < kEdgeFloor ? row->second.erase(e) : std::next(e);
        }
        row = row->second.empty() ? adjacency_.erase(row) : std::next(row);
    }

    for (const auto& [i, j] : candidates) {
        const Real d = distance(tracks[i]->position(), tracks[j]->position());
        if (d >= coloc_dist_) continue;
        const Real w = std::max(0.1, 1.0 - d / coloc_dist_);
        adjacency_[tracks[i]->id()][tracks[j]->id()] += w;
        adjacency_[tracks[j]->id()][tracks[i]->id()] += w;
    }

    const std::size_t n = tracks.size();

    // Adjacency lists, not an n-by-n matrix. Two dense matrices were allocated
    // and filled every scan purely to be read sparsely afterwards.
    std::vector<std::vector<std::size_t>> adjacency(n);
    std::vector<Real> weighted_degree(n, 0.0);
    std::unordered_map<std::string, std::size_t> index_of;
    index_of.reserve(n);
    for (std::size_t i = 0; i < n; ++i) index_of[tracks[i]->id()] = i;

    for (std::size_t i = 0; i < n; ++i) {
        const auto row = adjacency_.find(tracks[i]->id());
        if (row == adjacency_.end()) continue;
        for (const auto& [other_id, weight] : row->second) {
            const auto it = index_of.find(other_id);
            if (it == index_of.end() || it->second == i) continue;
            adjacency[i].push_back(it->second);
            weighted_degree[i] += weight;
        }
    }

    const std::vector<Real> bc = betweenness_centrality(adjacency);

    for (std::size_t i = 0; i < n; ++i) {
        latest_betweenness_[tracks[i]->id()] = bc[i];
        auto& h = bc_history_[tracks[i]->id()];
        h.push_back(bc[i]);
        if (h.size() > kBcHistory) h.pop_front();
    }

    // Connected components over the accumulated graph.
    std::vector<Cluster> clusters;
    std::vector<bool> visited(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        if (visited[i]) continue;
        std::vector<std::size_t> members;
        std::stack<std::size_t> stack;
        stack.push(i);
        visited[i] = true;
        while (!stack.empty()) {
            const std::size_t v = stack.top();
            stack.pop();
            members.push_back(v);
            for (const std::size_t w : adjacency[v]) {
                if (!visited[w]) {
                    visited[w] = true;
                    stack.push(w);
                }
            }
        }
        if (members.size() < 2) continue;

        Cluster c;
        c.cluster_id = static_cast<int>(clusters.size());
        Vec2 centre{};
        Real best_degree = -1.0;
        bool recurring = false;

        for (const std::size_t m : members) {
            c.member_ids.push_back(tracks[m]->id());
            centre += tracks[m]->position();
            c.betweenness[tracks[m]->id()] = bc[m];
            c.weighted_degree[tracks[m]->id()] = weighted_degree[m];
            if (weighted_degree[m] > best_degree) {
                best_degree = weighted_degree[m];
                c.hub_track = tracks[m]->id();
            }
            const auto& h = bc_history_[tracks[m]->id()];
            if (std::count_if(h.begin(), h.end(), [](Real v) { return v > 0.0; }) > 3) {
                recurring = true;
            }
        }
        c.centre = centre / static_cast<Real>(members.size());
        c.recurring = recurring;
        c.significance = recurring ? Severity::HIGH : Severity::MEDIUM;
        clusters.push_back(std::move(c));
    }

    return clusters;
}

std::vector<Alert> AnomalyEscalator::update(const std::string& track_id,
                                            Real score, Priority tier) {
    auto& h = history_[track_id];
    h.push_back(score);
    if (h.size() > kEscalationWindow) h.pop_front();

    std::vector<Alert> alerts;
    if (score > kEscalationThreshold) {
        alerts.push_back(Alert{track_id, "SPIKE", score, tier});
    }

    if (h.size() >= kEscalationWindow) {
        bool monotonic = true;
        for (std::size_t i = 1; i < h.size(); ++i) {
            if (h[i] <= h[i - 1]) monotonic = false;
        }
        if (monotonic) alerts.push_back(Alert{track_id, "ESCALATING", score, tier});
    }

    // High, then conspicuously quiet, then high again. Someone who noticed they
    // were being watched, went still, and resumed.
    if (h.size() >= 3) {
        const std::size_t n = h.size();
        if (h[n - 3] > 0.5 && h[n - 2] < 0.3 && h[n - 1] > 0.6) {
            alerts.push_back(Alert{track_id, "COUNTER_SURVEILLANCE", score, tier});
        }
    }
    return alerts;
}

std::vector<CollectionTask> schedule_collection(const std::vector<TrackPtr>& tracks,
                                                const DomainProfile& profile,
                                                std::size_t max_tasks) {
    std::vector<CollectionTask> tasks;
    // Expected information gain: point the most reliable sensor at the track
    // that matters most and is currently least well localised.
    for (const auto& t : tracks) {
        const Real unc = std::max(t->position_uncertainty(), 1.0);
        Modality best = Modality::GEOINT;
        Real best_gain = -1.0;
        for (std::size_t m = 0; m < static_cast<std::size_t>(Modality::Count); ++m) {
            const auto mod = static_cast<Modality>(m);
            const Real gain = profile.modality_weight(mod) * t->existence() / unc;
            if (gain > best_gain) {
                best_gain = gain;
                best = mod;
            }
        }
        tasks.push_back(CollectionTask{t->id(), best, best_gain, unc});
    }

    std::sort(tasks.begin(), tasks.end(),
              [](const CollectionTask& a, const CollectionTask& b) {
                  return a.expected_info_gain > b.expected_info_gain;
              });
    if (tasks.size() > max_tasks) tasks.resize(max_tasks);
    return tasks;
}

}  // namespace trace
