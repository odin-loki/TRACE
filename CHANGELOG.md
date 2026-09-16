<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->
<!-- SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki> -->

# Changelog

Kept short on purpose. This repository documents its own corrections in more
detail than a changelog can carry, so each entry points at where the working is:
[docs/AUDIT.md](docs/AUDIT.md) for the pre-release audit,
[docs/FORMAL_VERIFICATION.md](docs/FORMAL_VERIFICATION.md) for the mathematics,
[docs/PORTING_NOTES.md](docs/PORTING_NOTES.md) for the port, and
[docs/VALIDATION.md](docs/VALIDATION.md) for every measurement.

## 0.2.0

The first release that can be licensed, installed, consumed and audited.

### Licensing

- **AGPL-3.0-or-later**, with the full text in [`LICENSE`](LICENSE) and an SPDX
  tag on every first-party file. The repository had claimed this licence in its
  README since the port began and had never carried the text.
- [`THIRD-PARTY.md`](THIRD-PARTY.md) records vendored xsimd (BSD-3-Clause,
  compatible, unmodified) and the separate non-commercial terms on the
  MOTChallenge annotations, which are fetched rather than redistributed.

### Packaging

- `install()` and `export()` rules. There were none, while `trace_core`
  declared an `$<INSTALL_INTERFACE:>` that only means something to an install.
  `find_package(TRACE)` and `TRACE::core` now work from another project, and CI
  proves it by building one.
- `trace::abi::compatible()`, because `-march=native` is PUBLIC and the SIMD
  lane count decides `sizeof(simd::Batch)`. An archive built on one
  microarchitecture and consumed on another disagrees with its own headers,
  silently. Build installed packages with `-DTRACE_NATIVE_ARCH=OFF`.

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
- Published figures at this release: MOT17 train **53.0%** MOTA, 20.6 px MOTP,
  59.0% recall, 2,656 identity switches, 108.5% of the detector ceiling; MOT20
  train **62.5%** MOTA and 114.7% of ceiling. Both are lower than 0.1.0
  reported, and the reason is that the scoring was wrong in the tracker's
  favour.

### Tests, proofs and CI

- The **sanitizer CI entry had never configured** — an unquoted flag in a
  folded YAML scalar. Fixed, leak detection enabled, `halt_on_error` set; the
  suite is clean under ASan and UBSan.
- The proof runner counted a checker timeout as a proof failure and reported
  success for a run that checked nothing. Both fixed. Seventeen harnesses,
  sixteen discharged, `v16_existence_continuity` new.
- Six tests that could not fail were rewritten against what the code should
  produce, and each was run against the pre-fix source to confirm it fails
  there. `tests/test_detectors.cpp` is new: the detector layer had no direct
  tests at all.
- A test whose source goes missing, or a test source nobody listed, is now a
  configuration error rather than a quietly smaller suite.

### Documentation

- [docs/AUDIT.md](docs/AUDIT.md) records the audit: method, findings,
  refutations and what is still open.
- CUDA is documented as an unfinished branch rather than a backend; nothing
  calls it and the kernels pass host pointers to kernel launches.
- `simd::backend_name()` reported `xsimd/generic` on every build and now
  reports the architecture the batch type resolved to.

## 0.1.0

The C++23 port of ARIA-INTEL. See [docs/PORTING_NOTES.md](docs/PORTING_NOTES.md).
