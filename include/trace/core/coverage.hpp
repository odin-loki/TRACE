// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — what each sensor can see, when the deployment knows.
//
// The engine is handed detections and nothing else, so it has to infer things
// a real estate simply knows. Two separate findings converged on the same
// missing input:
//
//   - A sensor's detection probability is estimated from how often it reports
//     the tracks it has been feeding. For a wide-area sensor that works. For a
//     gate reader covering a few metres it does not, because the estimator
//     cannot distinguish "the reader missed it" from "the entity walked out of
//     coverage", and every point sensor estimates out at the floor.
//
//   - A scan with no detections is either an empty scene or a dead estate, and
//     the engine guesses between them from whether anything reported recently.
//     That guess is right often enough to be worth making and is still a guess.
//
// Both are answered by knowing which sensors could have seen a given point.
// This interface is how a deployment says so. It is optional: without one the
// engine falls back on the inferences above, which is what it did before.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "trace/core/types.hpp"

namespace trace {

class SensorCoverage {
public:
    virtual ~SensorCoverage() = default;

    /// Could this source have detected something at this point, this scan?
    /// False for a sensor that is down as well as for one looking elsewhere.
    [[nodiscard]] virtual bool covers(const std::string& source_id,
                                      Vec2 point) const = 0;

    /// Every source that is working this scan, whether or not it reported.
    [[nodiscard]] virtual std::vector<std::string> live_sources() const = 0;

    /// Is any working sensor looking at this point? A miss here is evidence of
    /// absence; a miss anywhere else is evidence of nothing.
    [[nodiscard]] bool anyone_covers(Vec2 point) const {
        for (const std::string& s : live_sources()) {
            if (covers(s, point)) return true;
        }
        return false;
    }
};

using SensorCoveragePtr = std::shared_ptr<const SensorCoverage>;

}  // namespace trace
