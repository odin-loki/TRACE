// Minimal assertion harness - keeps the suite dependency-free so it runs
// anywhere the engine builds.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace trace::test {

inline int g_failures = 0;
inline int g_checks = 0;

inline void report(bool ok, const char* expr, const char* file, int line,
                   const std::string& extra = {}) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL %s:%d  %s%s%s\n", file, line, expr,
                    extra.empty() ? "" : "  ", extra.c_str());
    }
}

inline int summary(const char* name) {
    std::printf("%s: %d checks, %d failures\n", name, g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

}  // namespace trace::test

#define CHECK(expr) \
    ::trace::test::report(static_cast<bool>(expr), #expr, __FILE__, __LINE__)

#define CHECK_NEAR(a, b, tol)                                              \
    ::trace::test::report(std::abs((a) - (b)) <= (tol), #a " ~= " #b,      \
                          __FILE__, __LINE__,                              \
                          "got " + std::to_string(a) + " vs " +            \
                              std::to_string(b))
