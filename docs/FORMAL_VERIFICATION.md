# Mathematical verification

Two questions, asked of the engine's numerical core:

1. **Is the mathematics right?** Does each formula follow from the model it
   claims to implement — and do its units agree?
2. **Does the code do what the mathematics says?** Established by bounded model
   checking under [ESBMC](https://github.com/esbmc/esbmc) and
   [CBMC](https://github.com/diffblue/cbmc), not by sampling inputs. The
   harnesses are in [`verification/`](../verification), which also documents
   what the proofs do **not** cover.

Ten derivations came back sound. Six did not, and are set out below with the
evidence. Five are fixed; the sixth is a calibration decision that belongs to
whoever owns the engine, and is reported rather than patched over.

---

## Sound

### The Mixed Ornstein–Uhlenbeck discretisation

`src/core/particle_filter.cpp:31-37`. Velocity follows

    dv = -theta v dt + sigma dW

whose exact transition over a step `dt` is
`v(t+dt) = e^{-theta dt} v(t) + eta` with

    Var[eta] = sigma^2 * (1 - e^{-2 theta dt}) / (2 theta),

and whose stationary variance is `sigma^2 / (2 theta)`. The code computes all
three, so the discretisation is **exact rather than an Euler approximation** —
it is right at any step size, which matters here because scan periods range
from one second to an hour across the profiles.

The three constants are not independent. They must satisfy

    sigma_v^2 == ss_vvar * (1 - alpha^2)

or a cloud already at steady state would drift away from it every scan, and the
velocity spread would inflate or collapse over a long run. They do
(`verification/v10`, proven).

One cosmetic note: the source computes `exp(-2 theta dt)` directly rather than
squaring `alpha`. The two agree in the reals and to within a rounding in
IEEE-754. Not a defect.

### `log_sum_exp`

`src/core/pmbm.cpp:39` and `src/core/pattern_of_life.cpp:16` (identical). The
hazard in any log-sum-exp is `log(0)`, which is `-inf` and poisons every
downstream weight. Shifting by the maximum rules it out: the maximal element
contributes `exp(0) = 1` exactly, so the accumulator is at least 1 and at most
K. Proven for any conforming `exp` (`v05`), by modelling only the two
properties the argument uses — `exp(0) == 1` and `exp <= 1` below zero — rather
than a particular implementation.

Non-finite inputs are handled before the shift, so an all `-inf` vector returns
`-inf` rather than `NaN`.

### The Otsu high-mode split

`src/detectors/behaviour.cpp:41`. The criterion maximised is

    w_lo * (1 - w_lo) * (mean_hi - mean_lo)^2

which is the standard Otsu between-class variance `w0 w1 (mu1 - mu0)^2`. The
risk here is index arithmetic rather than statistics: `best_k` is used both to
slice the sample and as a divisor for `n - best_k`, and is seeded to 0 to mean
"no split found". Proven that `1 <= best_k <= n-1` whenever a threshold is
returned, so neither class is ever empty and `sample[best_k]` is always in
bounds (`v06`).

### The existence update on a **miss**

`src/core/track.cpp`. This one is the textbook Bernoulli update and is right:
the probability of seeing nothing is `(1 - p_D)` if the track exists and `1` if
it does not, both dimensionless probabilities of the same event, so no density
belongs in the ratio. Proven to stay in [0,1], never to increase existence, to
drive existence to exactly zero for a detector that cannot miss, to leave it
untouched for a detector that never detects, and to be monotone in `p_D`
(`v14`).

It is worth stating because the update on a **hit** is the same shape and is
not right — see below.

### The clutter posterior

`src/core/pmbm.cpp:54`. For `n_unassigned ~ Poisson(r)` with a `Gamma(3, 1)`
prior on the rate, the posterior after `n` observations summing to `S` is
`Gamma(3 + S, 1 + n)`, mean `(3 + S) / (1 + n)` — exactly what `rate()`
returns. Windowed rather than cumulative, deliberately, so the estimate tracks
a change in conditions instead of being anchored by history. Proven never to
divide by zero, to stay strictly positive, and to be monotone in the
observations (`v13`).

The class is named `Beta-Poisson`. Beta–Binomial and Gamma–Poisson are the two
standard conjugate pairs and this is the second one; the arithmetic is right
and the name is not.

### `Mat2::inverse`, `Vec2::unit`, the road projection

- The ridge guard in `Mat2::inverse` leaves a divisor that is non-zero and at
  least `1e-15` for **any** determinant, not merely for the ones a covariance
  produces (`v01`).
- `Vec2::unit()` is total on finite input: never NaN, and either exactly the
  zero vector or unit-length (`v11`). The zero return for a near-stationary
  input is not a unit vector, so `a.unit().dot(b.unit())` reads as
  "perpendicular" rather than "no information" — callers must test speed before
  heading, and `behaviour.cpp:95` does.
- `closest_on_segment` clamps its projection parameter to [0,1], so the foot of
  the perpendicular is on the **segment** and not on its infinite extension,
  and the tangent it returns is unit-length — which `align()` depends on, since
  `dir * v.dot(dir)` is a projection only if `|dir| == 1` (`v12`).

### Where NaN is *not* stopped

Two guards read like sanitisers and are not. The `Mat2` ridge guard is
`if (fabs(d) < 1e-15)`, and a NaN compares false against everything, so it
takes the else branch untouched (`v02`). `std::clamp(v, 0, 1)` is
`v < lo ? lo : (hi < v ? hi : v)`, which returns NaN unchanged (`v03`) —
and every existence update in the engine is wrapped in one.

Neither is a defect on its own: nothing upstream is known to produce a NaN. Both
are worth having written down, because "the clamp will catch it" is the kind of
thing that is assumed rather than checked.

---

## Defects

### 1. The matcher was not optimal on tall problems — **fixed**

`src/core/assignment.cpp`. The Jonker–Volgenant search grows one augmenting
path per row and needs a free column to terminate on. With more rows than
columns the later rows have none: `delta` stays infinite, the search breaks
out, and the potentials it had already shifted stay shifted.

What comes back is still a valid matching of exactly the right **size** —
every column gets a row — so no caller could detect it. The pairs are simply
not the cheapest ones. Against exhaustive search over 4000 random matrices per
shape:

| shape | sub-optimal, before | mean cost excess (scale of 10) | after |
|---|---|---|---|
| 3x3 square | 0 / 4000 | — | 0 / 4000 |
| 4x3 tall | 3497 / 4000 | 3.9 | 0 / 4000 |
| 5x3 tall | 3818 / 4000 | 4.8 | 0 / 4000 |
| 6x2 tall | 3850 / 4000 | 5.6 | 0 / 4000 |
| 3x4 wide | 0 / 4000 | — | 0 / 4000 |
| 3x6 wide | 0 / 4000 | — | 0 / 4000 |

Every caller hits the tall case. Truth-to-track scoring
(`sim/scenario.cpp:58`, `apps/mot_main.cpp:94,168`) is tall exactly when the
tracker is under-reporting, which is the regime the metrics exist to measure;
reacquisition (`core/pmbm.cpp:611`) is tall whenever more detections reappear
at once than there are dormant tracks to claim them, and that one is engine
behaviour rather than scoring.

Measured on MOT17-02-FRCNN, same detections, the matcher the only change:

| | before | after |
|---|---|---|
| MOTA | 38.0% | **42.0%** |
| MOTP | 40.2 px | **15.5 px** |
| Identity switches | 947 | **198** |
| Recall / Precision | 44.1 / 97.8 | 44.1 / 97.8 |

Recall and precision do not move, which is the signature of the fault: the
number of matched pairs was always right. MOTP and identity switches depend on
**which** pairs, so localisation error was overstated 2.6x and identity
switches 4.8x.

Matching is symmetric, so the fix is to solve the transpose when rows outnumber
columns and swap the two maps back. Proven minimum-cost at 3x2 (`v15`); checked
against exhaustive search at eight shapes on both sides of square in
`tests/test_assignment.cpp`.

### 2. Betweenness was normalised to [0,2] — **fixed**

`src/core/network.cpp:74`. Brandes' accumulation runs its outer loop over every
source, so on an undirected graph each unordered pair `{s,t}` is counted twice,
once from each end. The raw score has to be halved before it is divided by the
`(n-1)(n-2)/2` unordered pairs a vertex could lie between. This divided by the
pair count alone, leaving every score at twice its normalised value: the hub of
a star scored **2.0** at every size, where a star's hub is the definition of
1.0.

Nothing downstream broke, and that is worth saying rather than implying a
behavioural fix. Both consumers are scale-free — the role classifier thresholds
against the upper quartile of these same values, and the recurrence test asks
only whether a score exceeds zero — so a uniform factor cancelled in both.
`NetworkReport`, which publishes the number for a reader to interpret, was the
one place it did not.

`tests/test_network.cpp` now checks the implementation against an independently
written reference that counts shortest paths by BFS and combines them
arithmetically, sharing no structure with Brandes' backward accumulation, over
**every** undirected graph on four, five and six vertices — 33,856 of them.
606,050 checks pass; 132,810 of them fail against the old divisor.

### 3. The existence update on a hit is not a posterior — **reported, not fixed**

`src/core/pmbm.cpp:816`. The code is

    r' = r p_D / ( r p_D + (1-r) cd )

where `cd` is the estimated clutter **density**, per square metre. The
Bernoulli/JIPDA update for a track that was detected is

    r' = r p_D g(z) / ( r p_D g(z) + (1-r) lambda_c )

where `g(z)` is the likelihood density of the detection under the track's own
innovation covariance. Two consequences:

- **The units do not agree.** The numerator carries a bare probability while
  the denominator carries a density, so the ratio has no scale-free meaning —
  its value moves with the units the area of regard happens to be written in.

- **The update is blind to fit.** `g(z)` is the only term carrying the
  innovation, so two detections — one on top of the prediction, one at the very
  edge of the gate — produce byte-identical existence. ESBMC finds the
  counterexample immediately (`v08`).

The practical effect is proven in `v09`: a newborn track at `r_birth = 0.45`
goes above **0.999** on its first detection, whatever that detection looks
like, for every `p_detection` and clutter density the engine can produce. So
`r_confirm = 0.55` clears on every track that gets a detection and no track
that does not. **The confirmation threshold is `n_hits >= 1` wearing a
probability's clothing**, and the gap between 0.45 and 0.55 chosen for it is
not doing any work.

This is reported rather than fixed, and the reason is worth recording, because
restoring `g(z)` is a dozen lines and was implemented and measured before being
withdrawn.

With the likelihood restored, existence still saturates on a well-fitting
detection — correctly, since where clutter is sparse a good detection really is
near-conclusive. What changes is the *ceiling*: `r` lands at 0.9962 instead of
being pinned at the clamp's 0.9999. That difference is small and it is load
bearing, because the number of consecutive misses a track survives before
`r_prune` retires it is set by where `r` starts. The old update pinned `r` at
the clamp ceiling on every hit, which bought the longest coast available. So
**coast duration was being set by an arithmetic artefact of the saturating
update rather than by the profile's prune threshold.**

Correcting the update alone therefore shortens the coast, and the scenarios
that depend on coasting lose recovery:

| domain | recovery before | after | identity switches before | after |
|---|---|---|---|---|
| CityCameraSurveillance | 93.7% | 68.8% | 104 | 79 |
| CounterTerrorism | 86.1% | 75.3% | 86 | 53 |
| IndoorVenue | 124.7% | 116.0% | 1058 | 1033 |
| WarehouseAssets | 108.2% | 105.2% | 21 | 22 |
| VehicleConvoy, Maritime | unchanged | | unchanged | |
| TransactionSpace | 98.1% | 98.6% | 13 | 12 |

Identity switches improve almost everywhere, which is what a fit-sensitive
existence should do. Recovery falls, in one case by 25 points.

The mathematics is not in question; the calibration is. `r_birth`, `r_confirm`
and `r_prune` were all chosen against the saturating dynamics, and correcting
the update without re-deriving them ships a regression in exchange for a
correctness argument. That trade is the engine owner's to make, and doing it
properly means re-deriving the lifecycle thresholds — and probably the coast
timeout — against the corrected update, then re-measuring. It is not a drive-by
change, and pretending otherwise by tuning thresholds until the numbers came
back would be fitting to the scenarios rather than fixing anything.

### 4. Dempster's rule started from a mass vector summing to three — **fixed**

`src/core/threat.cpp:118`. `fuse_credibility` combines evidence over
`{H, not-H, Theta}`. The rule itself is written correctly. The accumulator it
started from was `(1, 1, 1)`.

A mass function sums to one. This summed to three, and the excess propagated
through every combination that followed, so **belief and plausibility came out
at exactly 1.0000 for every possible input**:

| evidence | belief, before | belief, after | conflict, before | after |
|---|---|---|---|---|
| one report at r = 0.05 | 1.0000 | 0.0425 | 0.1375 | 0.0000 |
| one report at r = 0.9 | 1.0000 | 0.7650 | 0.7750 | 0.0000 |
| three at r = 0.9 | 1.0000 | 0.9866 | 0.9990 | 0.0130 |
| strong, then twice contradicted | 1.0000 | 0.7492 | 0.9990 | 0.0734 |

One near-worthless report and eight corroborating ones were indistinguishable,
and the conflict mass sat at its 0.999 cap from the third observation onwards —
so the one distinction that tracking `K` separately exists to make, between
"our sources contradict each other" and "we have a lot of evidence", could not
be made either. The clamps on the way out hid it: a belief pinned at 1.0 is
still a number in [0,1], and every range check the code had passed.

The fix is the identity element of the rule — all mass on the frame, none
committed either way, which is what "no evidence yet" means. `Credibility` is
carried on `TargetReport` and read by no decision in the engine, so this
corrects a number an operator is invited to interpret rather than a control
path. It had no tests; it has four now.

### 5. Collection tasking ranked against its own objective — **fixed**

`src/core/network.cpp:243`. The comment says it points a sensor at "the track
that matters most and is currently least well localised". The score was

    modality_weight * existence / uncertainty

which is monotone the wrong way in the one term meant to drive it. For a
Gaussian estimate of prior variance `P` under sensor noise `R`, a measurement
leaves `PR/(P+R)`, so the entropy it removes is

    dH = 0.5 * log(1 + P/R)

— **increasing** in `P`, because there is more to learn about a track you have
localised badly. Dividing by uncertainty ranks the best-localised tracks first,
which is the exact inverse of a collection plan's purpose.

The modality loop in the same function needed stating rather than fixing: every
factor that varies between tracks is common to all five modalities, so the
arg-max never depended on the track. It always returned whichever modality the
profile weights highest, dressed up as a per-track choice. Choosing per track
would need a per-modality accuracy and the profile carries only a per-modality
reliability weight.

### 6. The adaptive noise estimator converged to a square root — **fixed**

`src/core/pmbm.cpp:128`. `MeasurementNoiseEstimator` learns a multiplier on
each source's assumed measurement **variance** from the normalised innovation
squared its detections produce. The multiplier was set to `mean_nis / target`.

The NIS arriving at the estimator was measured against an innovation covariance
that already carries the multiplier. If a source's true variance is `k` times
the profile's assertion, then under an applied scale `s`,

    E[NIS] = trace(S^-1 R_true) = 2k/s

so setting `s = E[NIS]/target = k/s` solves `s^2 = k`. The estimator settles at
the **square root** of the ratio it exists to find. Reading the symptom as the
answer, when the symptom is already damped by the answer.

Measured by sweeping a source's true variance ratio and fitting the exponent —
`learned = ratio^alpha`, where a correct multiplier on a variance gives
`alpha = 1`:

| true variance ratio | learned, before | after | correct |
|---|---|---|---|
| 2.25 | 1.24 | 2.36 | 2.25 |
| 4 | 1.66 | 4.07 | 4 |
| 9 | 2.25 | 8.80 | 9 |
| 16 | 2.32 | 15.25 | 16 |
| 25 | 2.37 | 16.00 (ceiling) | 25 |
| **fitted alpha** | **0.51** | **1.03** | 1.0 |

A source genuinely four times noisier than claimed had its assumed variance
widened 1.66-fold. The gap widens with the fault: at 25 times, 2.37-fold. So
the mechanism was weakest exactly where it was needed most, and the residual
`alpha` below 0.5 at the top of the range is the sample cap and scale ceiling
binding on top of the square root.

The correction is multiplicative on the scale already in force, which solves
`mean_nis = target` — the definition of a consistent filter, and what the
target was chosen to express. It is damped at the same rate as the NIS average
driving it, because the two are a coupled pair and stepping straight to the
implied value rings.

`adaptive_meas_noise` is off in every shipped profile and reached only through
`--adaptive-noise`, so no default measurement in this repository changes. What
changes is that the opt-in feature now does what it claims.

---

## Noted in passing, not chased

Writing the test for section 5 turned up two things that are outside this
exercise but should not go unrecorded.

`Track`'s constructor does not seed its particle filter — `PmbmManager` does
that at birth — and an unseeded filter reports **zero** position uncertainty
rather than an undefined or maximal one. A caller who builds a `Track` directly
gets a track that claims perfect localisation.

More seriously, a filter fed detections that jump 40 m either side of its
prediction each scan also settles at **exactly zero** reported uncertainty:
resampling collapses the cloud onto a single ancestor, and the spread of one
point is zero. The filter reports maximum confidence at precisely the moment it
is most confused. That is a particle-filter degeneracy rather than a formula
error, so it is not in scope here, but anything reading
`position_uncertainty()` as a confidence — the collection plan in section 5
among them — inherits it.

---

## Method, and its limits

Bounded model checking does not sample. Where a harness reports
`VERIFICATION SUCCESSFUL`, the property holds for **every** input in the stated
domain. Where it reports a counterexample, the counterexample is concrete.

The weak link is that neither checker can parse libstdc++, so each harness is a
self-contained C translation of one function rather than the function itself.
[`verification/README.md`](../verification/README.md) sets out what that costs,
including the case where a translation error produced a convincing report of a
defect that did not exist, and the case where bounds checking caught a
translation error that inspection had missed. Sizes are small — 3x2 and 3x3
matchings, four-vertex graphs — and the larger cases are covered by exhaustive
search in the test suite instead, which at those sizes is not a sample either.
