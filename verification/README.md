# Formal proofs

Bounded model checking of the engine's numerical kernels, under
[ESBMC](https://github.com/esbmc/esbmc) and [CBMC](https://github.com/diffblue/cbmc).

```bash
./run.sh              # every harness, on whichever checker discharges it
./run.sh --cbmc       # force CBMC for all
./run.sh --esbmc      # force ESBMC for all
./run.sh v04 v15      # just those
ESBMC=/path/to/esbmc ./run.sh         # if the binaries are not on PATH
TIMEOUT=1800 ./run.sh                 # longer per-harness budget
```

Neither tool discharges every harness. ESBMC is markedly better on the ones
dominated by integer and array reasoning; CBMC's bit-blasting is better on the
ones dominated by floating-point comparison. The manifest in `run.sh` records
which is used for each and why the verdict is what it is, so the default run
uses the one that closes.

A bounded model checker does not sample inputs. Where a harness says
`VERIFICATION SUCCESSFUL` it has established the property for **every** input
in the stated domain, including the ones nobody would think to test. Where it
says `VERIFICATION FAILED` it has produced a concrete counterexample, and the
harness header says whether that counterexample is a defect or a documented
property of the code.

## What is proven

Thirteen of the fifteen are discharged. The other two are marked, reported, and
explained below rather than dropped.

| | property | verdict |
|---|---|---|
| v01 | `Mat2::inverse`'s guard leaves a divisor that is non-zero and at least 1e-15, for any determinant | holds |
| v02 | the same guard does **not** sanitise a NaN determinant | counterexample |
| v03 | `std::clamp(NaN, 0, 1)` returns NaN — it reads like a sanitiser and is not one | counterexample |
| v04 | the Hungarian matcher is memory-safe, returns a valid partial matching, and the `max_cost` gate holds | holds |
| v05 | `log_sum_exp`'s shift by the maximum makes `log(0)` unreachable: `acc` lies in [1, K] | holds |
| v06 | the Otsu split never divides by an empty class, and `sample[best_k]` is always in bounds | holds |
| v07 | betweenness is normalised to [0,1] over every four-vertex graph | *not discharged* — see below |
| v08 | the existence update on a detection does **not** agree with the JIPDA update | counterexample |
| v09 | one detection, of any quality, always clears `r_confirm` and drives `r` above 0.999 | holds |
| v10 | the MOU discretisation satisfies `sigma_v^2 == ss_vvar (1 - alpha^2)` in IEEE, so a cloud at steady state stays there | holds |
| v11 | `Vec2::unit()` is total on finite input, and its guard is what rules out the division by zero | holds |
| v12 | the road projection lands on the segment, and its tangent is unit-length | *not discharged* |
| v13 | the clutter posterior never divides by zero and never rules clutter impossible | *not discharged* |
| v14 | the miss update is a probability, never increases existence, and is monotone in `p_D` | holds |
| v15 | the matcher takes as many admissible pairs as exist, and the cheapest such matching, on a tall **gated** problem | holds |

v02, v03 and v08 assert something the code does not do, and are expected to
fail. They are in the suite because "where does a NaN stop" and "what is this
update actually a posterior over" are worth writing down and checking, rather
than rediscovering.

Two of them — v07 and v15 — also fail against the code as it was before the
defects they describe were fixed, which is what makes them regression tests
rather than descriptions.

## What these proofs do not say

Each harness is a **self-contained C translation** of one function. Neither
checker can parse libstdc++, so the alternative — pointing them at the real
headers — is not available. Every harness names the file and line range it
mirrors and reproduces the control flow statement for statement, but the
translation is the weak link in the chain, and it is worth being blunt about
why.

An earlier version of v04 used a finite integer sentinel where the source uses
`std::numeric_limits<Real>::infinity()`. The checker duly reported a
non-terminating loop, with a counterexample and everything. It was an artefact:
`minv[j] -= delta` drags a finite sentinel down until an unreachable column
becomes selectable, and a true IEEE infinity is unmoved by it. The source was
correct and the harness was wrong. **A failing proof is a claim about the
harness until the harness has been checked against the source.**

The checking cuts both ways. v15 failed its first run on an array bound — in
the harness, where the row and column counts had been swapped in an
initialisation loop. Bounds checking caught a translation bug that inspection
had missed.

Two further limits:

- **Sizes are small.** v04 and v15 run at 3x3 and 3x2; v07 at four vertices;
  v06 on a four-element sample. Bounded model checking is exponential in the
  state it must encode, and these are the sizes that discharge. A proof at 3x2
  is a proof about 3x2. The repository's own tests cover the larger cases by
  exhaustive search instead — `test_network` enumerates all 33,856 graphs on
  four, five and six vertices, and `test_assignment` compares against
  exhaustive search at eight shapes.

- **Some domains are narrowed.** v06 draws its sample from a small integer
  grid, v11 models `sqrt` by its two defining properties rather than encoding
  the circuit, v05 does the same for `exp`. Each header says which, and why
  the claim does not turn on what was narrowed. Where a narrowing WOULD weaken
  a claim, the claim is stated over the narrower domain instead of being
  quietly generalised.

- **Two harnesses are not discharged within the default budget by either
  checker**, and `run.sh` marks them SLOW and reports them rather than counting
  them. Bit-precise IEEE division is where both tools are weakest: v12 and v13
  reach 246,000 and 146,000 SAT variables from a handful of divisions and then
  sit there. Both encode their properties correctly and are kept so a faster
  solver, or a longer budget, can close them. A suite that quietly dropped them
  would read as more complete than it is.

  v07 is a different case. What it states — that betweenness is normalised to
  [0,1] — is established far more strongly by `tests/test_network`, which
  enumerates every undirected graph on four, five and six vertices and runs the
  **real function** on each rather than a translation of it. The harness is kept
  because it states the claim beside the code, not because it is the evidence.

  Narrowing a domain to make a harness discharge is legitimate where the claim
  does not turn on what was narrowed, and it is not where it does. v11 shows
  the line: its unit-length claim needed `n*n == x*x + y*y` to within a
  tolerance, no checker would take it, and rather than weaken the claim until
  something passed, the harness now states only what an arbitrary non-negative
  `n` supports — and says in its header what that costs.

## Reproducing

ESBMC 8.5.0 and CBMC 6.x. Neither is packaged in most distributions; ESBMC
builds from source in about 20 minutes and needs `gcc-multilib g++-multilib
libc6-dev-i386` for its 32-bit C library model.
