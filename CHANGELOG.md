<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->
<!-- SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki> -->

# Changelog

Kept short on purpose. This repository documents its own corrections in more
detail than a changelog can carry, so each entry points at where the working is:
[docs/AUDIT.md](docs/AUDIT.md) for the pre-release audit,
[docs/FORMAL_VERIFICATION.md](docs/FORMAL_VERIFICATION.md) for the mathematics,
[docs/PORTING_NOTES.md](docs/PORTING_NOTES.md) for the port, and
[docs/VALIDATION.md](docs/VALIDATION.md) for every measurement.

## 0.2.0 — 2026-09-16

The first release that can be licensed, installed, consumed and audited.

### Licensing

- **AGPL-3.0-or-later**, with the full text in [`LICENSE`](LICENSE) and an SPDX
  tag on every first-party file. The repository had claimed this licence in its
  README since the port began and had never carried the text.
- [`THIRD-PARTY.md`](THIRD-PARTY.md) records vendored xsimd (BSD-3-Clause,
  compatible, unmodified), optional Qt (LGPL-3.0-or-later or GPL-3.0-or-later,
  linked only by `trace_console`, off by default) and the separate
  non-commercial terms on the MOTChallenge annotations, which are fetched
  rather than redistributed. Those terms require citation, which the file
  described and did not give; the citations are now in it and in
  [docs/VALIDATION.md](docs/VALIDATION.md), where the numbers are.

### Packaging

- `install()` and `export()` rules. There were none, while `trace_core`
  declared an `$<INSTALL_INTERFACE:>` that only means something to an install.
  `find_package(TRACE)` and `TRACE::core` now work from another project, and CI
  proves it by building one.
- `trace::abi::compatible()`, because `-march=native` is PUBLIC and the SIMD
  lane count decides `sizeof(simd::Batch)`. An archive built on one
  microarchitecture and consumed on another disagrees with its own headers,
  silently. Build installed packages with `-DTRACE_NATIVE_ARCH=OFF`. The guard
  itself had to be hardened: it reported the mismatch at -O2 and missed it at
  -O0, where the linker was free to resolve its inline helper to the
  consumer's copy. CI now checks that it says no when it should, as well as
  yes.
- `-DTRACE_WITH_CUDA=ON` configures again. A bare source path in
  `trace_cuda`'s include directories, added with the install rules above, made
  CMake refuse to generate at all.

### Corrected results

Everything here was reproduced before it was fixed and measured after.
[docs/AUDIT.md](docs/AUDIT.md) has the numbers.

- The assignment solver's forbidden-pair marker did not dominate for negative
  costs, which is the reacquisition path's whole cost range. 94–100% of gated
  negative-cost matchings returned the wrong cardinality.
- The Bernoulli existence update dropped `(1-p_D)λ_c`, so a badly-fitting
  detection was punished harder than no detection at all.
- `predict_location` reweighted by the mixture density rather than the
  component's, so an hour-conditioned prediction was pulled toward whichever
  component was heaviest overall.
- `absorb()` dropped `age_`, `ever_confirmed_` and the hit records, putting
  `measurement_rate` back above 1 and letting a merge un-confirm an identity.
- **Position was integrated trapezoidally over an exact OU velocity step.**
  Right only while `theta dt` is small; the mean displacement per scan was 8%
  too far at 1, 31% at 2 and a factor of five at 10, and ten regime/profile
  pairs sit at 1 or above. Now the exact integral of the process the velocity
  step already uses, noise and correlation included. MOT17 MOTA unchanged,
  identity switches down 8%.
- **Source credibility was multiplied into an observation's confidence before
  it reached the possibility measure**, so `possibility_mismatch` — whose whole
  purpose is to separate weak evidence from strong — fired on 476 of 496
  real-entity scans and scored real entities as more suspicious than a
  deliberately convincing phantom. Trust now travels beside the observation
  rather than inside it. Now 0 of 496.
- **`Engine`'s move operations were defaulted** while PmbmManager, every Track
  and each Track's filter hold a pointer into the Engine's own config.
  Use-after-free on the third scan after the moved-from engine was destroyed.
- **The per-track forecast was a straight line beside a filter that does not go
  straight**, with an interval this repository had already written down as "an
  order-of-magnitude indication and not a calibrated interval". Both halves now
  run the filter's own mean and covariance recursion forward, which needed two
  quantities the particle cloud was not computing — its velocity variance and
  its position/velocity covariance. The interval widens a great deal, which is
  the result: 252 m one scan ahead for a walker, against 15 m before.
- The engine's reported mean latency divided an all-time total by a bounded
  history, and excluded the per-scan retirement sweep.
- `SDR_PATTERN` re-emitted on every scan and counted position noise as
  circling; `CHOKEPOINT` counted jitter across a cell boundary as passage.
- `PARALLEL_ROUTE` and `DEAD_DROP` could not fire at all under four profiles.
- Dempster's conflict kept only the last pairwise K.

### Corrected measurements

- **The don't-care amnesty in the MOTChallenge scorer was wider than the
  benchmark's own rule**, worth 1.3 points of MOTA on MOT17 and 4.2 at the
  `--min-score 0` operating point. Withdrawn, and every figure restated.
- CLEAR-MOT continuity preserved the last-*ever* match rather than the previous
  frame's, concealing identity switches and inflating MOTP.
- Published figures at this release: MOT17 train **53.0%** MOTA, 21.3 px MOTP,
  58.9% recall, 2,442 identity switches, 108.2% of the detector ceiling; MOT20
  train **62.5%** MOTA, 6,101 identity switches and 114.7% of ceiling. Both
  MOTA figures are lower than 0.1.0 reported, and the reason is that the
  scoring was wrong in the tracker's favour.
- Every scenario result in [docs/SIMULATIONS.md](docs/SIMULATIONS.md) is now a
  median over twelve seeds taken at this head. Several had drifted — the
  weather scenario's fog-phase recovery by 18 points, `anpr-corridor` carried
  three different recovery figures across three files, and `metro`'s constraint
  A/B claimed a dominance that no longer holds.
- Two published measurements could not be produced by the commands printed
  beside them: `trace_bench`'s sweep overshot its own `--max` and stopped short
  of the 400-track row README publishes, and `run_anpr_corridor` ignored
  `--no-constraint` while SIMULATIONS.md quoted an A/B that needs it. Both
  tools fixed rather than both numbers deleted.

### Tests, proofs and CI

- The **sanitizer CI entry had never configured** — an unquoted flag in a
  folded YAML scalar. Fixed, leak detection enabled, `halt_on_error` set; the
  suite is clean under ASan and UBSan.
- The proof runner counted a checker timeout as a proof failure and reported
  success for a run that checked nothing. Both fixed. Seventeen harnesses,
  sixteen discharged, `v16_existence_continuity` new. The default per-harness
  budget is now 1800 s: at 900 the suite could not discharge its own
  `v13_clutter_rate`, which takes about 1200.
- Eleven tests that could not fail were rewritten against what the code should
  produce, and each was run against the pre-fix source to confirm it fails
  there. Among the later five: a profile test that asserted finiteness inside a
  loop over targets, so a profile forming no track at all executed no checks; a
  track-cap test whose 120 uniform points per scan produced three tracks
  against a cap of 25; a waypoint test satisfied by the diagonal it exists to
  rule out; a particle-spread test with a sixtyfold band; and a
  false-accusation test that permitted a larger false accusation than the true
  one it measures elsewhere.
- Regression tests for three defects that had none: the coast timeout, the
  scorer's one-to-one matching, and dormancy being reachable in every profile.
- `tests/test_detectors.cpp` is new: the detector layer had no direct tests at
  all.
- A test whose source goes missing, or a test source nobody listed, is now a
  configuration error rather than a quietly smaller suite.

### Documentation

- [docs/AUDIT.md](docs/AUDIT.md) records the audit: method, findings,
  refutations and what is still open.
- CUDA is documented as an unfinished branch rather than a backend; nothing
  calls it and the kernels pass host pointers to kernel launches.
- `simd::backend_name()` reported `xsimd/generic` on every build and now
  reports the architecture the batch type resolved to.
- Three defect counts across two files disagreed with each other and with the
  file being counted; `--help` for `trace_sim` listed four of its nine flags.
  Both fixed, and `--junction-radius` is new, so the metro scenario's
  constraint A/B can be run rather than only quoted.
- **`PARALLEL_ROUTE` was documented as firing in none of the fourteen
  scenarios.** That was one seed. Over thirteen seeds of `anpr-corridor` it
  raises six events on five seeds — and exactly one of the six is on the
  target/tail pair, which nothing was counting and the report labelled
  "(tail)" regardless. The scenario now counts the two separately, and the
  limitation is restated as what it is: a single carriageway, where every
  vehicle holds the same heading, is the wrong place to demonstrate "matched
  heading at a fixed offset".

## 0.1.0

The C++23 port of ARIA-INTEL. See [docs/PORTING_NOTES.md](docs/PORTING_NOTES.md).
