# Mathematical verification

Two questions, asked of the engine's numerical core:

1. **Is the mathematics right?** Does each formula follow from the model it
   claims to implement — and do its units agree?
2. **Does the code do what the mathematics says?** Established by bounded model
   checking under [ESBMC](https://github.com/esbmc/esbmc) and
   [CBMC](https://github.com/diffblue/cbmc), not by sampling inputs. The
   harnesses are in [`verification/`](../verification), which also documents
   what the proofs do **not** cover.

Ten derivations came back sound. Seven did not, and are set out below with the
evidence. Six are fixed; the seventh is a calibration decision that belongs to
whoever owns the engine, and is reported rather than patched over.

Three of the seven are in the **scorer** rather than the engine — the code that
decides which track corresponds to which real entity, and what counts as the
tracker changing its mind. None of them changes how TRACE tracks anything. All
three change what this repository was reporting about it.

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

### 1. The matcher was not optimal — **fixed, in two goes**

`src/core/assignment.cpp`. Two independent defects in one function, and the
second was only found because the first fix was checked against the wrong
population.

**(a) Tall matrices.** The Jonker–Volgenant search grows one augmenting path
per row and needs a free column to terminate on. With more rows than columns
the later rows have none: `delta` goes infinite, the search breaks out, and the
potentials it had already shifted stay shifted. Matching is symmetric, so the
fix is to solve the transpose when rows outnumber columns and swap the two maps
back.

**(b) Forbidden pairs.** The same infinite `delta` arises, for the same reason,
whenever a row has no admissible column — which needs no particular shape at
all, only a gated-out pair. Worse, when the dead end is reached on the second or
later iteration of the search, `j0` is sitting on an *occupied* column rather
than the sentinel, so the guard after the loop does not fire and the
augmentation runs along a path that never reached a free column, evicting
whichever row already held that column irrespective of cost.

The fix for (b) is to replace every forbidden pair — non-finite, or finite but
outside the gate — with a large finite `big_m` before the search, and apply the
gate afterwards as before. `big_m` exceeds the total of every admissible cost,
so a matching using one forbidden pair is dearer than any matching using none;
minimising therefore takes as many admissible pairs as exist first and the
cheapest such matching second, which is exactly the objective `match`
documents.

**How (b) was missed, and then found.** The probe that validated (a) drew every
cost from `U(0, 10)` with the gate at 10, so every pair was admissible and no
infinity ever entered the search. It reported 0 of 4000 suboptimal at every
shape, and on that basis the matcher was called exact. It was exact — on
matrices the engine never builds. `match_points` writes an infinity for every
pair outside the radius and `reacquire_batch` fills its whole matrix with
infinity before scoring, so a gated matrix is not an edge case here, it is the
only case. A separate adversarial audit raised (b) with a two-by-two
counterexample, which reproduced immediately.

Against exhaustive search, 20,000 random instances per shape:

| | ungated, before (a) | after (a) | gated, before (b) | after (b) |
|---|---|---|---|---|
| 2x2 | exact | exact | 6.5% costlier | exact |
| 3x3 | exact | exact | 10.5% costlier | exact |
| 4x4 | exact | exact | 10.7% costlier | exact |
| 4x3 tall | 87% costlier | exact | 2.6% costlier | exact |
| 6x2 tall | 96% costlier | exact | — | exact |
| 3x4 wide | exact | exact | 2.5% costlier | exact |

Cardinality was always right, under both defects, which is why neither was
visible downstream. The minimal case for (b) is two by two:
`match({{1, inf}, {2, inf}})` returned column 0 to row 1 at cost 2, leaving row
0 — for whom it costs 1 — unmatched.

Every caller is exposed. Truth-to-track scoring (`sim/scenario.cpp`,
`apps/mot_main.cpp`) is tall exactly when the tracker is under-reporting, which
is the regime the metrics exist to measure, and gated on every frame where
anything falls outside the match radius. Reacquisition (`core/pmbm.cpp`) is
tall whenever more detections reappear at once than went dormant, and is gated
by construction — that one is engine behaviour, not scoring.

Measured on MOT17 train, all 21 sequences, the matcher the only thing changing:

| | original | after (a) | after (b) |
|---|---|---|---|
| MOTA | 48.2% | 50.5% | **51.3%** |
| MOTP | 27.4 px | 15.7 px | **13.7 px** |
| Identity switches | 19,156 | 11,621 | **8,686** |
| Recall / Precision | 60.0 / 90.8 | 60.0 / 90.9 | 59.9 / 90.8 |

Recall and precision never move, across both fixes. That is the signature of
the whole class of fault: the number of matched pairs was always right, and
only which pairs were wrong. (Finding 7 moves these figures again, and MOTP
further; the table in VALIDATION.md carries the final ones.)

The tests now compare against exhaustive search at eight shapes on both sides
of square, and at seven shapes again with a quarter of entries infinite and
half the rest above the gate. The gated cases fail 568 times against a build
with only (a) fixed. `verification/v15` proves the same objective outright for
a tall gated 3x2 — as many admissible pairs as exist, and the cheapest such
matching — and is sensitive to the substitution rather than merely to its
presence: a `big_m` of zero makes it fail on exactly those two claims.

The episode is the clearest thing in this document about what verification is
worth. Both defects were in code that had passed every test in the repository,
and the first fix was validated by a probe that measured the right property on
the wrong inputs — 4000 trials per shape, all reporting exact, none of them
resembling a matrix the engine builds. What caught the second was an
independent adversarial pass that produced a two-by-two counterexample, and
what settled it was reproducing that counterexample rather than accepting the
report. A measurement that cannot fail is not evidence, however many trials it
runs.

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
`--adaptive-noise`, so no default measurement in this repository changes — all
twelve scenarios produce byte-identical output. What changes is the opt-in
path, and there the correction is not uniformly a win:

| domain, recovery | default | `--adaptive-noise`, before | after |
|---|---|---|---|
| CityCameraSurveillance | 93.7% | 90.0% | **105.0%** |
| IndoorVenue | 124.7% | 115.9% | 107.0% |
| WarehouseAssets | 108.2% | 114.9% | 111.6% |
| TransactionSpace | 98.1% | 98.6% | 99.3% |
| the other three | — | unchanged | unchanged |

Which is what a working mechanism should look like rather than a broken one.
Learning a scale of 8.8 instead of 2.25 for a genuinely noisy source widens its
association gate by what the noise actually warrants, and that is worth 15
points where noise is the binding constraint (city cameras) and costs 9 where
a wider gate mostly buys confusion (a dense indoor venue). Before the fix the
estimator barely moved the gate at all, so it could neither help nor hurt much.

Whether to switch it on is therefore a domain judgement, which is why it is a
flag. The defect was that the flag did not do what its name said; it now does,
and the trade-off it exposes is a real one rather than an artefact.

### 7. Identity switches were counted without match continuity — **fixed**

`src/apps/mot_main.cpp`. CLEAR-MOT matches hypotheses to ground truth in two
passes, and the order is the whole point. A correspondence from the previous
frame that is **still valid** — the same hypothesis, still within the match
radius of the same ground-truth identity — is kept; only what is left over goes
to the optimal matcher.

The scorer ran one pass. A fresh optimum was computed every frame, which means
two hypotheses fitting two ground-truth identities about equally well were free
to swap between them whenever the arithmetic tipped by a pixel. Each swap
scored two identity switches for a scene in which nothing had happened. The
metric is supposed to count the *tracker* changing its mind, not the scorer
changing its mind.

Restoring the continuity pass takes MOT17's identity-switch total from 8,686 to
**2,445** — and this is on top of the two matcher fixes above, so against the
original scorer it is 19,156 to 2,445, a factor of 7.8.

Two figures move in the unflattering direction, and both are correct:

| MOT17 train | one-pass | two-pass |
|---|---|---|
| MOTA | 51.3% | 51.6% |
| MOTP | 13.7 px | **23.0 px** |
| Recall | 59.9% | 59.1% |
| Precision | 90.8% | 89.7% |
| Identity switches | 8,686 | **2,445** |

MOTP rises because it now averages over the correspondences actually
maintained, rather than over the best pairing available in each frame in
isolation. Keeping a correspondence that has drifted to 40 px instead of
re-matching to a hypothesis 10 px away is precisely what CLEAR-MOT is defined
to measure, and a scorer free to re-optimise every frame reports a localisation
error no tracker achieved. Recall and precision fall slightly for the same
reason: a maintained correspondence occupies a hypothesis that a fresh optimum
would have spent elsewhere.

The detector-ceiling pass is deliberately left one-pass. It asks what a perfect
echo of the detections would score, and an echo has no identity to maintain, so
a per-frame optimum is the right question there.

This one changes the case against appearance descriptors rather than making it.
Identity switches were 9.3% of the MOTA penalty on MOT17-02-FRCNN as previously
measured; they are 0.3% of it now. Eliminating every switch would buy 0.2 MOTA
points.

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

Thirteen of the fifteen harnesses are discharged. Two are not: bit-precise IEEE
division is where both tools are weakest, and they reach a quarter of a
million SAT variables from a handful of divisions. They are marked and reported
rather than dropped, because a suite that hid them would read as more complete
than it is.

Of the seven findings, exactly one — the existence update — was found by a
checker rather than by reading. The rest came from derivation: writing down what
the formula is supposed to compute and comparing. Model checking earned its
place by settling things reading could not, in both directions. It proved the
matcher's optimality and the saturation theorem outright; it also produced one
convincing report of a non-terminating loop that turned out to be an artefact of
the harness, which is the standing reminder that a failing proof is a claim about
the harness until the harness has been checked against the source.
