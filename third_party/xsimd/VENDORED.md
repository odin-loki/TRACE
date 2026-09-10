# Vendored xsimd

- **Version:** 13.0.0
- **Source:** https://github.com/xtensor-stack/xsimd
- **License:** BSD 3-Clause (see `LICENSE`)

Header-only, so only `include/` is vendored; the upstream tests, docs and CI
configuration have been removed. Vendoring keeps `cmake --build` working with no
network access and no package manager, which matters for a project whose whole
point is edge deployment.

TRACE does not require xsimd. Building with `-DTRACE_WITH_XSIMD=OFF` selects a
scalar fallback that runs the same kernels one lane at a time; the test suite
passes identically either way. To use a system xsimd instead, delete this
directory and CMake will `find_package(xsimd)`.
