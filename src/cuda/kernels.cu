// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — CUDA kernels.
//
// Compiled only when TRACE_WITH_CUDA is on and nvcc is present. The CPU path in
// src/core/ is the reference behaviour; these kernels must match it, including
// the exact OU position integral and the SI unit convention.

#include <cuda_runtime.h>

#include <cmath>
#include <cstdio>
#include <vector>

#include "trace/backend/cuda.hpp"

namespace trace::cuda {
namespace {

constexpr int kBlockSize = 256;

/// Device-side xoshiro256++, one independent stream per thread.
struct DeviceRng {
    unsigned long long s[4];

    __device__ explicit DeviceRng(unsigned long long seed) {
        // SplitMix64 to decorrelate the four state words.
        unsigned long long z = seed;
        for (int i = 0; i < 4; ++i) {
            z += 0x9E3779B97F4A7C15ULL;
            unsigned long long t = z;
            t = (t ^ (t >> 30)) * 0xBF58476D1CE4E5B9ULL;
            t = (t ^ (t >> 27)) * 0x94D049BB133111EBULL;
            s[i] = t ^ (t >> 31);
        }
    }

    __device__ static unsigned long long rotl(unsigned long long x, int k) {
        return (x << k) | (x >> (64 - k));
    }

    __device__ unsigned long long next() {
        const unsigned long long result = rotl(s[0] + s[3], 23) + s[0];
        const unsigned long long t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return result;
    }

    __device__ double uniform() {
        return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
    }

    /// Box-Muller: two normals per call, which is exactly what the x/y
    /// velocity update consumes.
    __device__ void normal_pair(double& z0, double& z1) {
        const double u1 = fmax(uniform(), 1e-12);
        const double u2 = uniform();
        const double r = sqrt(-2.0 * log(u1));
        const double theta = 6.283185307179586 * u2;
        z0 = r * cos(theta);
        z1 = r * sin(theta);
    }
};

/// One thread per particle across a flattened set of tracks.
__global__ void propagate_kernel(ParticleBatch* batches, std::size_t n_batches,
                                 const std::size_t* offsets, std::size_t total,
                                 double dt, double jitter_m,
                                 unsigned long long seed) {
    const std::size_t gid = blockIdx.x * blockDim.x + threadIdx.x;
    if (gid >= total) return;

    // Locate this thread's batch by binary search over the prefix sums.
    std::size_t lo = 0, hi = n_batches - 1, b = 0;
    while (lo <= hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (offsets[mid] <= gid) {
            b = mid;
            lo = mid + 1;
        } else {
            if (mid == 0) break;
            hi = mid - 1;
        }
    }
    const std::size_t i = gid - offsets[b];
    ParticleBatch& pb = batches[b];
    if (i >= pb.n) return;

    DeviceRng rng(seed ^ (static_cast<unsigned long long>(gid) * 0x9E3779B97F4A7C15ULL));

    double ex, ey, jx, jy;
    rng.normal_pair(ex, ey);
    rng.normal_pair(jx, jy);

    const double a = pb.alpha[i];
    const double s = pb.sigma_v[i];
    const double m = pb.x_mean[i];
    const double c1 = pb.x_sig1[i];
    // The jitter floor is folded in here rather than drawn separately, exactly
    // as ParticleFilter::predict does it, so both paths consume two normals
    // per axis and produce the same distribution.
    const double c2 = sqrt(pb.x_sig2[i] * pb.x_sig2[i] + jitter_m * jitter_m);
    const double vx = pb.vx[i];
    const double vy = pb.vy[i];

    // Must match ParticleFilter::predict exactly: OU velocity step, then the
    // exact integral of that velocity over the real scan period. `dt` no
    // longer appears: the scan period is baked into alpha, sigma_v and the
    // three position constants when MouConstants is built, and taking it from
    // two places at once is how the CPU and GPU paths would drift apart.
    (void)dt;
    const double nvx = fma(a, vx, s * ex);
    const double nvy = fma(a, vy, s * ey);

    pb.x[i] += fma(m, vx, fma(c1, ex, c2 * jx));
    pb.y[i] += fma(m, vy, fma(c1, ey, c2 * jy));
    pb.vx[i] = nvx;
    pb.vy[i] = nvy;
}

/// One thread per (point, component) pair.
__global__ void gmm_kernel(const double* data, std::size_t n_points,
                           const double* means, const double* chol,
                           const double* log_det, const double* log_weight,
                           std::size_t k, double* out) {
    const std::size_t gid = blockIdx.x * blockDim.x + threadIdx.x;
    if (gid >= n_points * k) return;

    const std::size_t p = gid / k;
    const std::size_t c = gid % k;

    double d[3];
    for (int i = 0; i < 3; ++i) d[i] = data[p * 3 + i] - means[c * 3 + i];

    // Forward substitution against the 3x3 Cholesky factor.
    const double* L = chol + c * 9;
    double y[3];
    for (int i = 0; i < 3; ++i) {
        double sum = d[i];
        for (int j = 0; j < i; ++j) sum -= L[i * 3 + j] * y[j];
        y[i] = sum / L[i * 3 + i];
    }

    double maha = 0.0;
    for (int i = 0; i < 3; ++i) maha += y[i] * y[i];

    constexpr double kLog2Pi = 1.8378770664093455;
    out[gid] = -0.5 * (maha + log_det[c] + 3.0 * kLog2Pi) + log_weight[c];
}

__global__ void pairwise_kernel(const double* xs, const double* ys,
                                std::size_t n, double* out) {
    const std::size_t gid = blockIdx.x * blockDim.x + threadIdx.x;
    if (gid >= n * n) return;
    const std::size_t i = gid / n;
    const std::size_t j = gid % n;
    const double dx = xs[i] - xs[j];
    const double dy = ys[i] - ys[j];
    out[gid] = sqrt(dx * dx + dy * dy);
}

bool check(cudaError_t e, const char* what) {
    if (e == cudaSuccess) return true;
    std::fprintf(stderr, "TRACE cuda: %s failed: %s\n", what, cudaGetErrorString(e));
    return false;
}

int device_count() {
    int n = 0;
    if (cudaGetDeviceCount(&n) != cudaSuccess) return 0;
    return n;
}

}  // namespace

bool available() { return device_count() > 0; }

const char* device_name() {
    static char name[256] = {0};
    if (name[0] != '\0') return name;
    if (device_count() == 0) {
        std::snprintf(name, sizeof(name), "no CUDA device");
        return name;
    }
    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, 0) == cudaSuccess) {
        std::snprintf(name, sizeof(name), "%s (sm_%d%d, %zu MB)", prop.name,
                      prop.major, prop.minor, prop.totalGlobalMem / (1024 * 1024));
    } else {
        std::snprintf(name, sizeof(name), "CUDA device 0");
    }
    return name;
}

bool propagate_batches(ParticleBatch* batches, std::size_t n_batches, Real dt,
                       Real jitter_m, std::uint64_t seed) {
    if (!available() || n_batches == 0) return false;

    std::vector<std::size_t> offsets(n_batches);
    std::size_t total = 0;
    for (std::size_t b = 0; b < n_batches; ++b) {
        offsets[b] = total;
        total += batches[b].n;
    }
    // Not worth a launch: the transfer and setup cost exceeds the work.
    if (total < kMinParticlesForGpu) return false;

    ParticleBatch* d_batches = nullptr;
    std::size_t* d_offsets = nullptr;
    if (!check(cudaMalloc(&d_batches, n_batches * sizeof(ParticleBatch)), "malloc batches")) {
        return false;
    }
    if (!check(cudaMalloc(&d_offsets, n_batches * sizeof(std::size_t)), "malloc offsets")) {
        cudaFree(d_batches);
        return false;
    }
    cudaMemcpy(d_batches, batches, n_batches * sizeof(ParticleBatch),
               cudaMemcpyHostToDevice);
    cudaMemcpy(d_offsets, offsets.data(), n_batches * sizeof(std::size_t),
               cudaMemcpyHostToDevice);

    const std::size_t blocks = (total + kBlockSize - 1) / kBlockSize;
    propagate_kernel<<<blocks, kBlockSize>>>(d_batches, n_batches, d_offsets, total,
                                             dt, jitter_m, seed);
    const bool ok = check(cudaGetLastError(), "propagate launch") &&
                    check(cudaDeviceSynchronize(), "propagate sync");

    cudaFree(d_batches);
    cudaFree(d_offsets);
    return ok;
}

bool gmm_log_responsibilities(const Real* data, std::size_t n_points,
                              const Real* means, const Real* chol,
                              const Real* log_det, const Real* log_weight,
                              std::size_t k, Real* out) {
    if (!available() || n_points == 0 || k == 0) return false;

    const std::size_t total = n_points * k;
    const std::size_t blocks = (total + kBlockSize - 1) / kBlockSize;
    gmm_kernel<<<blocks, kBlockSize>>>(data, n_points, means, chol, log_det,
                                       log_weight, k, out);
    return check(cudaGetLastError(), "gmm launch") &&
           check(cudaDeviceSynchronize(), "gmm sync");
}

bool pairwise_distances(const Real* xs, const Real* ys, std::size_t n, Real* out) {
    if (!available() || n == 0) return false;
    const std::size_t total = n * n;
    const std::size_t blocks = (total + kBlockSize - 1) / kBlockSize;
    pairwise_kernel<<<blocks, kBlockSize>>>(xs, ys, n, out);
    return check(cudaGetLastError(), "pairwise launch") &&
           check(cudaDeviceSynchronize(), "pairwise sync");
}

}  // namespace trace::cuda
