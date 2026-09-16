<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->
<!-- SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki> -->

# The release audit

This document records an adversarial audit of the whole engine, run before
release, and what it found. It is here for the same reason
[FORMAL_VERIFICATION.md](FORMAL_VERIFICATION.md) is: a repository that reports
its own corrections is easier to trust than one that reports only its results.

---

## Method

Fourteen areas, each read by an agent given the source and told that a finding
must be a **defect** rather than a preference — wrong mathematics, a wrong
result, undefined behaviour, a crash, a leak, a vacuous test, a documented
claim the code does not support, or a portability break — and told to report
what it checked and found **correct** as well.

Every finding was then put to two independent readers with different jobs:

- a **source** lens, which opens the named file, checks the quoted evidence is
  verbatim, and traces the call sites to see whether the failure is reachable;
- a **domain** lens, which asks whether the "correct" behaviour the finding
  demands is actually correct for PMBM, JIPDA, CLEAR-MOT, Ornstein–Uhlenbeck
  processes or Jonker–Volgenant, and whether the defect has any observable
  consequence.

Both were told to default to refuting when unsure, on the stated grounds that a
false finding costs more than a missed one. A finding survived only if neither
lens refuted it.

| | |
|---|---|
| Areas audited | 14 |
| Findings raised | 122 |
| Things checked and reported correct | 244 |
| Adversarial verdicts cast | 186 |
| Findings refuted | 112 |

The refutation rate is the point of the exercise, not an embarrassment to it.
Most refutations were "already fixed at HEAD" — the audit ran while its own
findings were being repaired — but a substantial minority were findings that
did not survive contact with the source or the domain. Two examples, both of
which would have been plausible in a report:

- the claim that `best_nis` should be a sum rather than a maximum over the
  gated set. Correct for IPDA with one sensor's several returns; wrong here,
  because the assigner returns at most one observation per source per track, so
  the set is a singleton by construction and the two agree.
- the claim that `net_displacement`'s sibling `smoothed_velocity` was safe at
  `n == 0`. It was not — it divided by zero — which the verifier caught while
  refuting a different part of the same finding.

---

## What it found

Everything below was reproduced before it was fixed, and every fix that changes
a number has that number measured rather than asserted.

### Wrong results

| | Effect |
|---|---|
| Big-M did not dominate for negative costs | 94–100% of gated negative-cost matchings returned the wrong cardinality, measured against brute force over 96,000 instances. Live on the reacquisition path, whose costs are negated log-densities. |
| The existence update dropped `(1−p_D)λ_c` | A badly-fitting detection was punished harder than no detection at all. `verification/v16` proves the two updates now agree in the limit. |
| `predict_location` reweighted by the mixture density | Asked where an entity is at the hour it is always at work, over seven seeds: mean 372 against a truth of 1000, now 824. |
| `absorb()` left `age_` behind | `measurement_rate` returned 5.250 after a merge — the defect it had just been rewritten to remove. |
| Mean latency divided by a bounded history | Every session over 256 scans reported a mean inflated by `scans/256`: the 420-scan `warehouse` read 5.83 ms against a true 3.55. |
| CLEAR-MOT continuity used the last-EVER match | A stale claim outranked a live one. MOTP improved 1.2 px on MOT17 and 4.4 px on MOT20 once fixed; 204 and 1,270 concealed identity switches appeared. |
| `SDR_PATTERN` re-emitted on every scan | 4,358 events across the suite became 154. It was 65% of every event the engine raised and 96.5% of it was duplicates. |
| CHOKEPOINT counted jitter as passage | 57% of the events in the scenario that reports them. |
| The credibility evidence set was 120 m in every domain | 0.001× to 112× each profile's own gate. Six tracks, one badly sourced: all six reported belief 1.000. |
| Dempster's conflict kept only the last pairwise K | Sources that contradicted each other early and agreed later reported no conflict. |
| Forecasts published sub-sensor precision | 12.8% of all forecasts came from a cloud tighter than the sensor's noise; the tightest ratio was 0.000. |
| `PARALLEL_ROUTE`'s window was empty under two profiles | `VehicleConvoy` had its bounds the wrong way round; `WarehouseAssets` inherited an urban 80 m. |
| `DEAD_DROP` was unreachable under two profiles | Their scan periods exceed the whole dead-drop window. |
| The detection ledger did not follow its own swap | The denominator of every recovery figure credited the wrong entity. |
| A spoofer claimed to cover the world | `anyone_covers` was true everywhere, so the coverage map stopped distinguishing anything. |
| `Chol3::factor` returned NaN as valid | `sum <= 0.0` is false for NaN. |
| The world forgot a dwelling entity's speed | It fell to the 1.4 m/s default its own comment says must not be reachable by accident. |

### Scoring, and a number withdrawn

The don't-care amnesty was wider than MOTChallenge's own rule: as well as
containment it forgave any track within `match_radius` of the region's foot
point, which is a 100 px disc hung off one edge. It was worth **1.3 points of
MOTA on MOT17 and 1.1 on MOT20** without changing anything the tracker found,
and **4.2 points** at the `--min-score 0` operating point, where the tracker
emits most false positives. Withdrawn; every MOTChallenge figure in this
repository is restated against containment alone.

### Things that could not fail

Six tests asserted only conditions that hold for any input — `!windows.empty()`
and `spread > 0` for any fitted model, `A || B` where B was true by
construction, a `during` count computed and printed and never checked. Each is
now written against what the code should actually produce, and each was run
against the pre-fix source to confirm it fails there.

### Claims the code did not support

CUDA was advertised as a backend for three engine stages; nothing calls it and
the kernels pass host pointers to kernel launches. `simd::backend_name()`
reported `xsimd/generic` on every build. `DomainProfile::rv_horizon_scans` was
documented and read by nothing. `smoothed_velocity` was documented in metres
per scan and returns metres per second. `NetworkRole::role` omitted a role the
classifier emits. `docs/SIMULATIONS.md` claimed bit-for-bit determinism that
`docs/VALIDATION.md` spends a section qualifying.

### The tooling that was supposed to catch all this

The `sanitizers` CI entry had **never configured**: an unquoted flag in a
folded YAML scalar made cmake exit 1, so the ASan/UBSan coverage the matrix
advertises had never once run. Once fixed it found that the memory test's RSS
assertion is meaningless under ASan, and nothing else — the engine is clean
under both, with leak detection on.

The proof runner counted a checker running out of budget as a proof FAILURE,
and reported success for a run in which nothing was checked. Both fixed; the
second was not hypothetical, because the licensing pass had just dropped the
runner's executable bit and the job went on passing.

`tests/CMakeLists.txt` silently skipped any test whose source was missing.

---

## What it did not find

244 specific things were checked and reported correct. The audit did not find a
memory error, a leak, or undefined behaviour in the engine's own execution: the
full suite passes under ASan and UBSan with leak detection enabled and
`halt_on_error` set. It did not find a defect in the Hungarian solver's
shortest-path search, the OU *velocity* discretisation, the log-sum-exp, the
Otsu threshold, or the Brandes accumulation — all of which carry proof
harnesses that still discharge. It did find one in the OU *position*
integration, which had no harness; see below.

## The second wave

This document was written while the audit's findings were still being worked
through. Everything below was found by the audit and fixed after the first
draft, and each one is recorded in full in
[PORTING_NOTES.md](PORTING_NOTES.md), [CHANGELOG.md](../CHANGELOG.md) or the
commit that made it.

| | Effect |
|---|---|
| The forecast was a straight line beside a filter that does not go straight | `position + velocity * elapsed` contradicts a filter whose predict step decays velocity; the interval beside it scaled the current uncertainty by `sqrt(k+1)` and had nothing to do with the process noise. Both now run the filter's own recursion forward. Closed the open item this document raised against itself. |
| Position was integrated trapezoidally over an exact OU velocity step | The mean displacement per scan exceeded the correct one by 8% at `theta dt` of 1, 31% at 2, and a factor of five at 10. Ten regime/profile pairs sit at 1 or above, `OrganisedCrimeNetwork`'s stationary regime at 10. Fixed by integrating exactly, noise included; MOT17 MOTA unchanged at 53.0% and 214 fewer identity switches. |
| Trust was folded into the evidence-quality measure | `possibility_mismatch` fired on 476 of 496 real-entity scans in the `spoofing` scenario and scored real entities as more suspicious than a phantom built to be convincing. The engine-level test could not see it: it fed one source, and credibility returns 1.0 when only one source has ever reported. Now 0 of 496. |
| `Engine`'s move operations were defaulted | PmbmManager, every Track and each Track's filter hold a `const DomainProfile*` into the Engine's own config, which a defaulted move relocated. Use-after-free on the third scan after the source was destroyed, and a silently hollowed-out profile before that. |
| The ABI guard reported a mismatch at -O2 and missed it at -O0 | `library_tag()` called an inline function whose out-of-line copy the linker was free to take from the consumer. Debug is where somebody chasing an ABI bug looks first. |
| `-DTRACE_WITH_CUDA=ON` did not configure | A bare source path in `trace_cuda`'s include directories, added with the install rules; CMake refuses to generate an export containing one. |
| Qt was absent from THIRD-PARTY.md | `trace_console` links LGPL/GPL Qt, and the file the README points at as the list of what is not this project's work did not mention it. |
| The MOTChallenge citation was described, not given | The data terms require citation; the file said so and cited nothing. |
| Two published measurements could not be reproduced | `trace_bench`'s sweep overshot its own `--max` and stopped at 270 while README published a 400-track row; `run_anpr_corridor` ignored `--no-constraint` while SIMULATIONS.md quoted an A/B that needs it. Both tools fixed, both measurements re-taken. |
| Documented counts and results had drifted | Three different defect counts across two files; a "four of the seven inherited" that survived there stopping being seven; `anpr-corridor` at three different recovery figures; the weather scenario's fog-phase recovery 18 points out; `metro`'s constraint A/B claiming a dominance that no longer holds. Every scenario figure in SIMULATIONS.md is now a median over twelve seeds taken at this head. |
| The proof suite's default budget could not discharge its own suite | `v13_clutter_rate` needs about 1200 s and the default was 900. |

---

## What is still open

- `PARALLEL_ROUTE` fires, on five of thirteen seeds of `anpr-corridor`, and
  one of those six events is on the target/tail pair the scenario built for it.
  The audit reported it as firing in none of the fourteen scenarios; that was
  one seed. The distance windows are fixed. What remains is that a
  single-carriageway corridor is the wrong place to demonstrate "matched
  heading at a fixed offset", because everything there matches heading. See
  "What is still missing" in [VALIDATION.md](VALIDATION.md).
- `v12b_segment_geometry` is not discharged by either checker.
- The exact OU position integral costs `anpr-corridor` about three points of
  recovery, because a sparse scenario was benefiting from over-prediction. The
  integral is right and the scenario figure is now lower; whether the profile's
  heading-hold times are the ones its author meant is a separate question
  nobody has asked.
- Three profiles have a `PARALLEL_ROUTE` window narrower than their own
  separation noise. `tests/test_detectors` prints them on every run rather than
  leaving it to be rediscovered.
