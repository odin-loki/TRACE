// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — shim so one harness runs under both CBMC and ESBMC unchanged.
//
// Each harness in this directory is a self-contained C translation of one
// function from the engine. C rather than C++ because neither checker can
// parse libstdc++'s headers; self-contained because a harness that pulled in
// the real header would drag in the whole engine. The cost of that choice is
// that a harness can drift from the source it mirrors, so every one names the
// file and line range it was translated from, and reproduces the control flow
// statement for statement.
//
// That fidelity is not a formality. An earlier version of the Hungarian
// harness used a finite integer sentinel where the source uses
// `std::numeric_limits<Real>::infinity()`, and the checker duly reported a
// non-terminating loop that does not exist in the engine: with a true IEEE
// infinity, `minv[j] -= delta` leaves an unreachable column unreachable. If
// you change a harness, change it towards the source.
#ifndef TRACE_VERIF_H
#define TRACE_VERIF_H

#if defined(__ESBMC__) || defined(USE_ESBMC)
void __ESBMC_assume(_Bool);
#define ASSUME(c) __ESBMC_assume(c)
#else
void __CPROVER_assume(_Bool);
#define ASSUME(c) __CPROVER_assume(c)
#endif

#include <assert.h>
#define CHECK(c, msg) assert((c) && msg)

double nondet_double(void);
int nondet_int(void);

/* A finite double in [lo,hi]. */
static double bounded(double lo, double hi) {
    double v = nondet_double();
    ASSUME(v == v);              /* not NaN */
    ASSUME(v >= lo && v <= hi);
    return v;
}
#endif
