// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — CUDA backend stub.
//
// Built when TRACE_WITH_CUDA is off, so callers can query the backend
// unconditionally and get an honest "no" rather than a link error.
#if !defined(TRACE_WITH_CUDA)

#include "trace/backend/cuda.hpp"

namespace trace::cuda {

bool available() { return false; }

const char* device_name() { return "CUDA not compiled in"; }

bool propagate_batches(ParticleBatch*, std::size_t, Real, Real, std::uint64_t) {
    return false;
}

bool gmm_log_responsibilities(const Real*, std::size_t, const Real*, const Real*,
                              const Real*, const Real*, std::size_t, Real*) {
    return false;
}

bool pairwise_distances(const Real*, const Real*, std::size_t, Real*) {
    return false;
}

}  // namespace trace::cuda

#endif
