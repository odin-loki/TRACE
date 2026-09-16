# Mathematical verification

Two questions, asked of the engine's numerical core:

1. **Is the mathematics right?** Does each formula follow from the model it
   claims to implement — and do its units agree?
2. **Does the code do what the mathematics says?** Established by bounded model
   checking under [ESBMC](https://github.com/esbmc/esbmc) and
   [CBMC](https://github.com/diffblue/cbmc), not by sampling inputs. The
   harnesses are in [`verification/`](../verification), which also documents
   what the proofs do **not** cover.

Ten derivations came back sound. Thirteen did not, and are set out below with
the evidence. All thirteen are now fixed.

**Five of the thirteen are in the scorer, not the engine** — the code that
decides which track corresponds to which real entity, what counts as the
tracker changing its mind, which ground-truth identity is which, and what to do
about the places the benchmark declined to annotate. Not one of them changes
how TRACE tracks anything. All five change what this repository was reporting
about it, and together they move MOT17 MOTA from 48.2% to 54.3% without a
single line of the engine being touched.

That ratio is the most useful thing here: close to half the defects were in the
instrument rather than in the thing being measured. Of the eight that were in
the engine, five were errors of dimension — a probability compared against a
density, metres per second integrated as metres per scan, a walking pace used
as an aircraft's speed, a count of detections divided by a count of scans, a slope per sample
reported as a slope per scan —
which is what makes "do the units agree" worth asking of every formula rather
than only of the ones that look suspicious.

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

### 3. The existence update on a hit was not a posterior — **fixed**

`src/core/pmbm.cpp`. The code was

    r' = r p_D / ( r p_D + (1-r) cd )

where `cd` is the estimated clutter **density**, per square metre. The
Bernoulli/JIPDA update for a track that was detected is

    r' = r p_D g(z) / ( r p_D g(z) + (1-r) lambda_c )

where `g(z)` is the likelihood density of the detection under the track's own
innovation covariance. Two things followed from dropping it:

- **The units did not agree.** The numerator carried a bare probability while
  the denominator carried a density, so the ratio had no scale-free meaning —
  its value moved with the units the area of regard happened to be written in.

- **The update was blind to fit.** `g(z)` is the only term carrying the
  innovation, so a detection on top of the prediction and one at the very edge
  of the gate produced byte-identical existence. ESBMC finds the counterexample
  immediately (`v08`), and `v09` proves what it cost: a newborn track at
  `r_birth = 0.45` went above **0.999** on its first detection whatever that
  detection looked like, for every `p_detection` and clutter density the engine
  can produce. So `r_confirm = 0.55` cleared on every track that got a
  detection and no track that did not — **`n_hits >= 1` wearing a probability's
  clothing**, with the gap between 0.45 and 0.55 doing no work at all.

Restoring `g(z)` is a dozen lines; both `mahalanobis_sq` and
`innovation_covariance` were already computed at that call site. A newborn
track's first detection now lands at 0.949 rather than pinned at the 0.9999
clamp, and a poorer one lands lower.

**What it costs and what it buys.** Measured over 14 scenarios and 7 seeds,
medians throughout:

| | as shipped | with g(z) |
|---|---|---|
| Ghost tracks | 2,912 | **2,375 (−18.4%)** |
| Recovery (sum over 14 scenarios) | 1538.2 | 1499.3 (−38.9, ≈ −2.5%) |
| Mean position error (sum) | 2443.6 | **2412.7** |
| Identity switches | 1,409 | 1,448 (+2.8%) |

Two scenarios are outright wins — `coordinated-evasion` drops from 128 ghost
tracks to 43 and `mule-network` from 50 to 11, both with fewer identity
switches and no loss of recovery. `metro` is the sharpest trade: half the ghost
tracks (360 to 178) and mean position error from 20.5 m to 15.1 m, for nine
points of recovery.

On real data the correction is close to free:

| | before | after |
|---|---|---|
| MOT17 MOTA | 54.3% | 54.3% |
| MOT17 MOTP | 23.0 px | **22.2 px** |
| MOT17 identity switches | 2,445 | **2,362** |
| MOT20 MOTA | 64.0% | 63.8% |
| MOT20 MOTP | 26.2 px | **25.8 px** |
| MOT20 identity switches | 4,906 | **4,738** |

**No threshold recovers the difference**, and that was checked rather than
assumed. `r_prune` was swept over a 500x range and moved total recovery by
under 1% (730.4 to 736.6 across five settings); `r_confirm` was swept over a 2x
range and moved it by 0.35%. The trade is a property of the corrected
posterior, not of a calibration that happens to be stale: fewer, better-founded
tracks that coast less far.

**A false trail worth recording.** The first measurement of this change said it
cost 25 points of recovery on one scenario and was a plain regression, and that
is what an earlier version of this document reported. It was wrong twice over.
It was taken through the matcher of finding 1, before either half of that was
fixed — and the scenario scorer is the matcher, so the comparison was made with
a broken instrument. And it looked only at recovery, which is the half of the
trade that gets worse. With the matcher fixed and ghost tracks counted, the
same scenario is within half a point.

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

### 8. Pooled mostly-tracked and mostly-lost merged people across sequences — **fixed**

`src/apps/mot_main.cpp`. Both figures are fractions over ground-truth
identities: an identity is mostly-tracked if it was matched in at least 80% of
the frames it appeared in, and mostly-lost below 20%. The per-sequence
accumulator keyed those frame counts by the raw ground-truth id, and the
OVERALL accumulator summed them on the same key.

MOTChallenge numbers its identities from 1 within each sequence. Person 1 of
MOT17-02 and person 1 of MOT17-04 are different people wearing the same
integer, so the 2,388 identities of the MOT17 train split were being added
together into **188** buckets — a collapse of nearly thirteen to one. A bucket
that blends a well-tracked person with an untracked one lands in the middle
band, neither above 80% nor below 20%, and almost every bucket did:

| MOT17 train, pooled | before | after |
|---|---|---|
| Mostly tracked | 6.5% | **30.3%** |
| Mostly lost | 6.5% | **26.1%** |

Both were understated by about a factor of four, in opposite directions —
the merge manufactured an implausibly tidy result in which hardly any identity
was either well tracked or lost. Keying by (sequence, identity) separates them.

Nothing else moves: MOTA, MOTP, recall, precision and identity switches do not
depend on these maps, and the per-sequence figures were always right, since
within one sequence the raw id is unique. Only the pooled line was wrong, which
is why it survived — every sequence in the table above it was correct.

### 9. Regions MOT declines to annotate were charged as false positives — **fixed**

`src/sim/mot.cpp` and `src/apps/mot_main.cpp`. MOTChallenge's ground-truth file
carries rows scored below 0.5 or of a non-pedestrian class. They mark places
where something **is** present and the benchmark declines to say what: a
reflection, a person on a bicycle, a figure in a poster, a crowd too dense to
annotate individually. They are not a minor category — **277,212 of the 614,103
rows in the MOT17 train file, 45% of it.**

The loader dropped them, which is right: scoring a tracker against a region
nobody labelled would be meaningless. But dropping them from ground truth is
only half the protocol. A track sitting on one then has nothing to match
against, so it fell through to the false-positive count — which penalises the
tracker for finding exactly what the annotator saw and chose not to label. MOT's
own protocol removes such hypotheses from the hypothesis set before counting.

The regions are now kept in a separate map and an unmatched track that sits on
one is discarded rather than charged. "Sits on one" has two arms, because a
don't-care region is a rectangle while the rest of this scorer works in
ground-contact points: the track's foot point is inside the rectangle, or it is
within `match_radius` of the rectangle's own foot point — the same standard
applied to real ground truth.

| | before | after |
|---|---|---|
| MOT17 MOTA | 51.6% | **54.3%** |
| MOT17 precision | 89.7% | **93.5%** |
| MOT20 MOTA | 61.7% | **64.0%** |
| MOT20 precision | 96.2% | **99.6%** |

Recall, MOTP, identity switches, mostly-tracked and mostly-lost do not move at
all, on either benchmark — only the false-positive count was ever wrong.

8,974 track-frames are discarded across the MOT17 train split. That number is
**printed beside the precision it raises**, per sequence and overall, because a
change that improves a score by discarding evidence should not be able to hide
inside the score.

---

### 10. The forecast advanced one second per scan — **fixed**

`src/core/engine.cpp`. `Track::velocity()` is metres per **second**, and each
forecast step is stamped one scan period into the future — but the step added
was `p += v`, one second of travel. Right only where the scan period happens
to be one second, which is one of the ten shipped profiles.

Everywhere else the forecast was out by the scan period: a vessel at 9 m/s
predicted an hour ahead was placed 9 m from where it started rather than 32 km,
and at the other end a 25 fps profile threw the prediction twenty-five times
too far. The fix is `p += v * scan_dt_s`.

### 11. Three profiles could not form a track at all — **fixed**

`src/core/pmbm.cpp`. Two-point initiation corroborates an unassigned detection
against the unassigned detections of the previous scan: if one is within
`birth_gate` metres, the pair is a birth rather than two false alarms. So the
gate is a question about travel — how far could this entity have moved since
the last scan.

It was derived from `courier_speed_thresh`, which is a network-analysis
parameter describing how fast a courier walks, and sits between 0.3 and 4 m/s
in every shipped profile. **No profile sets `birth_gate_m`**, so that
derivation is what all of them used. Where a scan's travel exceeded the gate,
no track could ever be born:

| profile | travel per scan | gate | tracks formed |
|---|---|---|---|
| Airspace | 1,000 m | 210 m | **none, ever** |
| Maritime | 32,400 m | 29,400 m | **none, ever** |
| VehicleConvoy | 150 m | 129 m | **none, ever** |

Slowing the target below the gate made tracks appear immediately in all three,
which is what confirms the gate as the cause rather than anything else about
those profiles.

The gate now comes from the profile's own motion model. Each MOU regime is
built by `motion(name, heading_hold_s, typical_speed_mps)`, which sets
`sigma = typical_speed * sqrt(2 theta)`, so `sigma / sqrt(2 theta)` recovers
that speed exactly and the fastest regime is the domain's own statement of how
fast its entities go — 300 m/s for Airspace against the 12 m/s the old
derivation supplied. All ten profiles now form tracks.

It costs something, and the cost is understood rather than waved past. Over
seven seeds and fourteen scenarios: total recovery 1499.3 to 1491.3 (−0.5%),
ghost tracks 2,375 to 2,389 (+0.6%), identity switches 1,448 to 1,362 (−6%).
On MOT it is within rounding — MOT17 MOTA 54.3% to 54.4%, MOTP 22.2 to 22.1 px.
Almost all of the recovery loss is one scenario — warehouse, 117.0 to 110.8 —
and the mechanism is the tension the gate cannot resolve alone: it must be at
least one scan of travel or fast entities cannot be born, and no wider than the
spacing between entities or a detection is corroborated by its neighbour and
two objects become one. A warehouse packs its pallets closer together than a
forklift travels in a scan.

That difference was checked rather than assumed. Perturbing the gate by one
millimetre and by one centimetre leaves warehouse recovery identical to three
decimal places across three seeds, while the old gate moves it by five points —
so the change is genuinely attributable and not the chaotic divergence these
Monte Carlo scenarios are prone to. The remedy for a domain in that position is
`birth_gate_m`, which exists for exactly this and which no profile sets;
setting it here to recover the number would be fitting to the scenario rather
than to the domain.

### 12. A rate that could reach eight — **fixed**

`include/trace/core/track.hpp`. `measurement_rate()` is documented and read as
the fraction of scans a track has existed for in which it was detected, so it
cannot exceed 1. Two independent errors let it reach 8.

`update_hit` runs once for every observation in a scan's group, so a track
under four overlapping sensors scored four hits against one scan of age. And
the denominator was `age_`, which is zero on the scan a track is born in even
though the track was detected in that scan — the birth scan counted in the
numerator and not the denominator. That second one alone put a
perfectly-detected single-sensor track at 2.0 on its second scan and kept it
above 1 for life.

Measured on a track detected in every scan:

| sensors on the track | before | after |
|---|---|---|
| 1 | 2.0 | **1.000** |
| 2 | 4.0 | **1.000** |
| 4 | 8.0 | **1.000** |

The consequence is that both thresholds tested against it were vacuous. The
group-spawn test asks for a measurement rate above 0.85 and the merge test for
one above 0.55; every track with more than one sensor on it passed both,
whatever its detection history, and so did every single-sensor track past its
second scan. Counting hit scans rather than detections, and dividing by the
scans the track has actually existed for, makes both mean what they say.

Almost nothing moves as a result — over seven seeds, total recovery 1491.3 to
1491.2, ghost tracks 2,389 to 2,379, identity switches 1,362 to 1,350 — because
the shipped scenarios mostly feed a track from one sensor at a time. The value
of the fix is not in those numbers; it is that two decisions which were being
taken unconditionally are now taken on evidence.

### 13. The velocity fit regressed against an index, not a clock — **fixed**

`src/detectors/rendezvous.cpp`. `fitted_velocity` fits a line to a track's
recent positions and returns the slope in metres per **scan** — the intercept
solve downstream works in scans. It regressed position against the sample's
**index** in the history.

`Track::history_` is appended once per accepted detection, not once per scan,
so the index is not a clock. Two sensors reporting the same scan add two
samples at the same instant; a track that goes unseen for a while adds none at
all. Regressing against the index reads a slope of metres per *sample* and
calls it metres per scan.

Measured on a 2 m/s target with a 60 s scan — truth 120 m per scan:

| sensors on the track | before | after |
|---|---|---|
| 1 | 123.0 m/scan (1.02×) | 123.0 (1.02×) |
| 2 | 45.8 m/scan (0.38×) | **100.6 (0.84×)** |
| 4 | 31.0 m/scan (0.26×) | **135.5 (1.13×)** |

So under the overlapping coverage this detector is most likely to be used in,
the closing speed read at about a quarter of the truth — and convergence ETAs
are computed from it, so the warning offered four times the time that actually
remained. The samples already carry a timestamp; the fit now uses it, and
converts to metres per scan at the end.

What is left after the fix is spread, not bias: a six-sample window spans fewer
scans when several sensors report each one, so the baseline is shorter and the
fit noisier. Widening the window to span a fixed number of scans rather than
samples would reduce it.

**Nothing measurable moves.** Not one of the fourteen scenarios changes by a
single ghost track or identity switch over seven seeds, and MOT17 is identical
to the digit. That is not evidence the fix is unnecessary — it is evidence that
neither the scenarios nor MOTChallenge put two sensors on one track, which is
the only configuration the defect bites in. The engine supports overlapping
coverage, several profiles assume it, and a deployment with it would have been
getting convergence warnings offering four times the time that remained. A
defect the test suite cannot see is worse than one it can.

`fitted_velocity` is now declared in the detectors header rather than kept
private to the translation unit. A function whose contract is a unit — metres
per scan, from a history counted in detections — needs to be reachable from a
test, or the contract drifts. An end-to-end test through the rendezvous
detector was tried first and rejected: the geometric intercept is one of three
methods and the others masked the difference, so the test passed against the
defect.

---

## Not a formula error, and fixed anyway

The engine leaked memory without bound, which is not a mathematical defect and
would not have been found by any of the work above. It came out of an
adversarial audit run alongside it, and is recorded here because the
measurement is the same kind of thing: a property stated, checked, and held to.

Thirteen maps across the engine, the detectors, the contact graph and the
escalator are keyed by track id, and track ids are never reused, so every one
of them grew for the life of the process. On top of that the engine retained a
full `ScanReport` — the scan's targets, clusters, events and warnings — for
every scan it had ever run.

Measured with the live track count held flat at eight, entities arriving and
leaving so that ids kept being minted: resident memory rose from 4.3 MB to
**44.7 MB over 4,000 scans**, linear, no plateau. Two thirds of that was the
retained reports.

Both are fixed. `performance_report()` accumulates its statistics per scan
rather than walking a retained history, so the history can be bounded to the
last 256 scans without a single reported number changing; and the engine sweeps
per-track state for ids the track manager no longer knows, live or dormant.
The same load now plateaus at **8.7 MB**. Scenario output is byte-identical
before and after, which is the point: the sweep only drops state that can never
be consulted again, because an id that is gone cannot come back.

One thing was deliberately left growing. `recent_stops_` in the mode-transition
detector is already bounded twice, by a time cutoff and a hard cap, and a stop
recorded by a track that has since been retired is still a historical fact
inside that window — dropping it would suppress exactly the handover the
detector exists to find. Pruning everything that *can* be pruned is not the
same as pruning everything that should be.

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

Fifteen of the sixteen harnesses are discharged. The one that is not compares
an IEEE product against a scaled tolerance, which is the shape bit-blasting
handles worst; it is marked and reported rather than dropped, because a suite
that hid it would read as more complete than it is. It reached a quarter of a
million SAT variables as originally written, and splitting its four claims so
each is stated over exactly the inputs it needs — "for any parameter in [0,1]"
rather than "for the parameter this division produces" — discharged two of
them. Quantifying over a superset of the reachable values is both the cheaper
encoding and the stronger statement, which is the one generalisable technique
to come out of this exercise.

Of the thirteen findings, exactly one — the existence update — was found by a
checker rather than by reading. The rest came from derivation: writing down what
the formula is supposed to compute and comparing. Model checking earned its
place by settling things reading could not, in both directions. It proved the
matcher's optimality and the saturation theorem outright; it also produced one
convincing report of a non-terminating loop that turned out to be an artefact of
the harness, which is the standing reminder that a failing proof is a claim about
the harness until the harness has been checked against the source.
