// TRACE — CUDA backend.
//
// The engine has a complete CPU path; CUDA is an accelerator for the two places
// where the work is genuinely wide:
//
//   * particle propagation - every track's 320 particles advance independently,
//     and with many tracks in flight that is tens of thousands of lanes;
//   * pattern-of-life EM - the E step evaluates K Gaussians against N points,
//     which is a dense matrix of independent evaluations.
//
// Below roughly 30 simultaneous tracks the launch overhead dominates and the
// CPU path is faster; trace_available() plus the batch size decide at runtime.
#pragma once

#include <cstddef>
#include <cstdint>

#include "trace/core/types.hpp"

namespace trace::cuda {

/// True if the binary was built with CUDA and a usable device is present.
bool available();

/// Human-readable device description, or why there isn't one.
const char* device_name();

/// Below this many concurrently-propagated particles the CPU wins.
inline constexpr std::size_t kMinParticlesForGpu = 8192;

/// One track's particle block, laid out structure-of-arrays to match the CPU.
struct ParticleBatch {
    Real* x{nullptr};
    Real* y{nullptr};
    Real* vx{nullptr};
    Real* vy{nullptr};
    const Real* alpha{nullptr};    ///< per-particle OU velocity retention
    const Real* sigma_v{nullptr};  ///< per-particle OU velocity diffusion
    std::size_t n{0};
};

/// Advance many tracks' particles in one launch.
///
/// Returns false if CUDA is unavailable or the batch is too small to be worth
/// it, in which case the caller must run the CPU path. Never a hard failure:
/// the GPU is an optimisation, not a dependency.
bool propagate_batches(ParticleBatch* batches, std::size_t n_batches, Real dt,
                       Real jitter_m, std::uint64_t seed);

/// Gaussian-mixture log-responsibilities for the pattern-of-life E step.
///
/// `data` is n_points x 3 row-major; `chol` is k x 9 (lower-triangular 3x3
/// factors); `log_det` and `log_weight` are length k. Writes n_points x k
/// log-responsibilities into `out`.
bool gmm_log_responsibilities(const Real* data, std::size_t n_points,
                              const Real* means, const Real* chol,
                              const Real* log_det, const Real* log_weight,
                              std::size_t k, Real* out);

/// Pairwise distances between all confirmed track positions - the input to
/// co-location clustering and the convergence detector. O(n^2) and trivially
/// parallel, which matters once track counts reach the hundreds.
bool pairwise_distances(const Real* xs, const Real* ys, std::size_t n, Real* out);

}  // namespace trace::cuda
