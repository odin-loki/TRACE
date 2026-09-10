// The whole simulation suite rests on these draws being correct and
// reproducible, so they get checked directly.
#include "trace/core/rng.hpp"

#include <vector>

#include "test_harness.hpp"

using namespace trace;

namespace {

void test_scalar_moments() {
    Rng rng(12345);
    const int n = 200000;
    double sum = 0.0, sum_sq = 0.0, umin = 1.0, umax = 0.0;
    for (int i = 0; i < n; ++i) {
        const double u = rng.uniform();
        umin = std::min(umin, u);
        umax = std::max(umax, u);
        const double z = rng.normal();
        sum += z;
        sum_sq += z * z;
    }
    CHECK(umin >= 0.0);
    CHECK(umax < 1.0);
    CHECK_NEAR(sum / n, 0.0, 0.02);          // mean
    CHECK_NEAR(sum_sq / n, 1.0, 0.02);       // variance
}

void test_determinism() {
    Rng a(999), b(999);
    for (int i = 0; i < 1000; ++i) {
        CHECK(a.next_u64() == b.next_u64());
    }
    Rng c(1000);
    Rng d(999);
    CHECK(c.next_u64() != d.next_u64());
}

void test_simd_normals() {
    // Box-Muller across lanes must produce the same moments as the scalar path,
    // otherwise the particle cloud is silently mis-scaled.
    SimdRng srng(4242);
    const int iters = 20000;
    double sum = 0.0, sum_sq = 0.0;
    long count = 0;
    std::vector<double> buf(simd::kLanes);
    for (int i = 0; i < iters; ++i) {
        simd::Batch z0, z1;
        srng.normal_pair(z0, z1);
        simd::store_u(buf.data(), z0);
        for (double v : buf) { sum += v; sum_sq += v * v; ++count; }
        simd::store_u(buf.data(), z1);
        for (double v : buf) { sum += v; sum_sq += v * v; ++count; }
    }
    CHECK(count > 0);
    CHECK_NEAR(sum / static_cast<double>(count), 0.0, 0.02);
    CHECK_NEAR(sum_sq / static_cast<double>(count), 1.0, 0.02);
}

void test_simd_lane_independence() {
    // If lanes shared a stream, every particle in a batch would move together.
    SimdRng srng(7);
    simd::Batch z0, z1;
    srng.normal_pair(z0, z1);
    std::vector<double> buf(simd::kLanes);
    simd::store_u(buf.data(), z0);
    if (simd::kLanes > 1) {
        bool all_same = true;
        for (std::size_t i = 1; i < simd::kLanes; ++i) {
            if (std::abs(buf[i] - buf[0]) > 1e-12) all_same = false;
        }
        CHECK(!all_same);
    }
}

void test_poisson_and_beta() {
    Rng rng(31337);
    const int n = 50000;
    double psum = 0.0;
    for (int i = 0; i < n; ++i) psum += rng.poisson(3.0);
    CHECK_NEAR(psum / n, 3.0, 0.08);

    double bsum = 0.0;
    for (int i = 0; i < n; ++i) bsum += rng.beta(2.0, 3.0);
    CHECK_NEAR(bsum / n, 0.4, 0.01);  // mean of Beta(2,3) = 2/5
}

}  // namespace

int main() {
    std::printf("simd backend: %s (%zu lanes)\n", simd::backend_name(),
                simd::kLanes);
    test_scalar_moments();
    test_determinism();
    test_simd_normals();
    test_simd_lane_independence();
    test_poisson_and_beta();
    return trace::test::summary("test_rng");
}
