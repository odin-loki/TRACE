// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — Bayesian threat scoring.
//
// Eight evidence dimensions, each expressed as a Beta distribution rather than
// a point estimate, combined by weighted Monte-Carlo integration. The output is
// a distribution: a mean, a spread, and upper percentiles. That spread is the
// point — "0.7 plus or minus 0.30" and "0.7 plus or minus 0.02" call for very
// different responses, and a scalar score hides the difference.
#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "trace/core/profile.hpp"
#include "trace/core/report.hpp"
#include "trace/core/rng.hpp"
#include "trace/core/track.hpp"

namespace trace {

/// Score one track. Mutates the track's threat EMA and persistence counter.
ThreatScore score_track(Track& track, Real timestamp,
                        const std::vector<Vec2>& high_value_locations,
                        Real hvl_radius, const DomainProfile& profile, Rng& rng);

/// Map a score onto a priority tier.
Priority priority_for(Real score);

/// Dempster-Shafer combination over a track's recent evidence.
Credibility fuse_credibility(const std::vector<Observation>& evidence,
                             const DomainProfile& profile);

}  // namespace trace
