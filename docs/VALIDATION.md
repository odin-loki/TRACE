# Validation against real data

Every other number in this repository comes from TRACE's own simulator. That is
evidence the code does what it is meant to and no evidence at all about the
world. This document contains the exception: replays of real
[MOTChallenge](https://motchallenge.net) sequences, with real detections
produced by real detectors on real video.

```bash
./scripts/fetch_mot.sh ./data/mot        # ~30 MB, annotations only
./build/src/apps/trace_mot ./data/mot/train
```

TRACE consumes detections rather than pixels, so only the label archives are
needed. The ground-plane proxy for a person is the bottom-centre of their
bounding box — the one point on a box that stays put as the box grows with
proximity to the camera.

---

## Result: MOT17 train, all 21 sequences

Public detections, no re-identification network, no offline processing. One
causal pass over 336,891 ground-truth boxes. Appearance descriptors are
supported but switched off here, for reasons measured below.

| | |
|---|---|
| **MOTA** | **53.0%** |
| MOTP | 20.6 px |
| Recall | 59.0% |
| Precision | 91.8% |
| Mostly tracked | 29.8% |
| Mostly lost | 26.2% |
| Identity switches | 2,656 |
| Track-frames on not-to-be-considered regions, discarded | 4,263 |
| Throughput | 5.8 ms/frame, one core |

At the tool's defaults, which is what the command above runs. An earlier
version of this table reported a different operating point (`--min-score 0`,
`--radius 120`) than the command printed beside it; that configuration gives
52.8% MOTA and 109.1% of ceiling. Dropping the score filter buys recall — 66.5%
against 59.0% — and pays for it in precision, 84.0% against 91.8%, so on MOTA
the two cancel to within two tenths of a point while identity switches go from
2,656 to 3,621. It is worth six tenths of a point of ceiling recovery and
nothing else.

That conclusion is the reverse of the one this document carried until the
don't-care amnesty was narrowed, and the reversal is the clearest single
illustration of why that mattered. At `--min-score 0` the tracker emits far
more false positives, the disc forgave a large share of them, and this
operating point scored **4.2 points of MOTA above** what it does now — against
1.3 at the default. An amnesty that grows with the number of false positives it
is forgiving will always flatter the setting that produces most of them.

### These numbers moved, and it was the scorer

Every MOTChallenge figure in this document was re-measured after a defect was
found in the matcher that decides which track corresponds to which ground-truth
box — the tables above and below, the per-detector ranges, the threshold sweep
and the appearance comparison. The Jonker-Volgenant solver is only valid when
rows do not outnumber columns, and
truth-to-track scoring is tall exactly when the tracker is under-reporting —
which is most frames of most sequences. The matching it returned had the right
number of pairs and the wrong pairs.

So the figures that depend on **how many** pairs were matched did not move, and
the figures that depend on **which** pairs did:

| MOT17 train, all 21 sequences | broken scorer | scorer fixed | now |
|---|---|---|---|
| Recall | 60.0% | 59.0% | 59.0% |
| Precision | 90.8% | 93.7% | 91.8% |
| MOTA | 48.2% | **54.4%** | **53.0%** |
| MOTP | 27.4 px | 21.8 px | 20.6 px |
| Identity switches | 19,156 | **2,326** | 2,656 |

The third column is the current head. Two things separate it from the second,
both measured on their own further down: the second correction to the
existence update, under "What the clutter term costs and buys", and the
withdrawal of an over-wide don't-care amnesty, under "The amnesty that was
worth 1.3 MOTA". The columns are kept apart so the scorer's original effect is
neither credited with, nor blamed for, something else.

**Eight defects: five in the scorer, three in the engine.** Two
in the matcher that decides which track corresponds to which ground-truth box —
one for matrices with more rows than columns, one for matrices containing
gated-out pairs — one in the identity-switch rule, one in how ground-truth
identities are pooled, and one in the handling of regions MOT flags as
not-to-be-considered. Those five leave the engine's own code untouched; what
changed is the instrument. The sixth is in the engine: the Bayesian existence
update dropped the measurement likelihood, which made it blind to how well a
detection fitted the track it was given, trading about 2.5% of recovery on the
synthetic scenarios for 18% fewer ghost tracks and costing nothing here. The
seventh is the birth gate, which was derived from a walking pace and left three
profiles - Airspace, Maritime and VehicleConvoy - unable to form a track at
all; on this benchmark it moves MOTA by a tenth of a point. The eighth is
`measurement_rate()`, which counted detections against scans and so could reach
8.0 for a four-sensor track, making both thresholds tested against it vacuous. All are derived in
[FORMAL_VERIFICATION.md](FORMAL_VERIFICATION.md).

Precision rises from 90.8% to 93.7%, almost all of it from the last of the
five scorer defects. Nearly half
the MOT17 ground-truth file — 277,212 rows of 614,103 — marks places where
something is present and the benchmark declined to annotate it: a reflection, a
cyclist, a crowd too dense to separate. Those rows were dropped from ground
truth, correctly, but a track sitting on one was then charged as a false
positive, which penalises the tracker for finding exactly what the annotator
saw and chose not to label. MOT's own protocol discards such hypotheses before
counting, and now so does this. 4,263 track-frames are discarded across the
train split, and the count is printed beside the precision it raises rather
than folded silently into it.

The identity-switch count is the largest single correction, from 19,156 to
2,656. CLEAR-MOT matches in two passes: a correspondence from the previous
frame that is still within the radius is **kept**, and only what is left over
goes to the optimal matcher. Without that first pass a fresh optimum is
computed every frame, so two hypotheses that fit two identities about equally
well swap whenever the arithmetic tips — scoring two switches for a scene in
which nothing happened. The metric is meant to count the tracker changing its
mind, not the scorer changing its mind.

**MOTP rises when match continuity is restored — 13.7 px to 23.0 px at that
step, 20.6 px at the current head — and that is the correct direction.** Under CLEAR-MOT it averages over the correspondences actually
maintained, not over the best pairing available each frame; keeping a
correspondence that has drifted to 40 px rather than re-matching to a hypothesis
10 px away is exactly what the metric is defined to measure. The 27.4 px this
document used to report was neither — it was a per-frame optimum computed by a
broken matcher. Recall falls slightly for the same reason.

Mostly-tracked and mostly-lost moved further than anything else, from 10.0%
and 3.5% to **29.8%** and **26.2%**, and that is a fourth defect rather than a
consequence of the first three. Both figures are pooled over ground-truth
identities, and MOTChallenge numbers its identities from 1 within each
sequence — so person 1 of MOT17-02 and person 1 of MOT17-04 were being added
together. The 1,638 identities this scorer counts across the train split
collapsed into 170 buckets, and a bucket blending a well-tracked person with an
untracked one lands in the middle band that is neither mostly-tracked nor
mostly-lost. Almost everything
ended up in that band, which is why both figures were small. Keying by
(sequence, identity) separates them again. The per-sequence figures in the
tables below were never affected; only the pooled ones were.

(This paragraph used to say 2,388 and 188. Those are the identity counts over
every row of the gt.txt files; a row whose flag or class is not 1 goes to the
ignore set rather than to the ground truth, so 1,638 and 170 are what the
scorer actually pools. A 9.6:1 collapse rather than a 12.7:1 one, and the same
conclusion — but the figure quoted should be the one the code produces.)

The simulator figures elsewhere in this document are affected far less, and
were checked rather than assumed: across all twelve scenarios the fix moves
recovery by at most 0.4 points and identity-switch counts by a handful in
either direction. Those scenarios carry five to seventeen entities, so their
cost matrices are rarely tall by much; MOT frames carry tens to hundreds.

The derivation, the exhaustive-search evidence and the fix are in
[FORMAL_VERIFICATION.md](FORMAL_VERIFICATION.md).

### Continuity means the previous frame, not the last time it worked

CLEAR-MOT's first matching pass preserves the previous frame's correspondence
where it is still valid, and only what is left over goes to the optimal
matcher. This scorer ran that pass off `last_match` — the last track id EVER
seen for a ground-truth identity — which is the right structure for counting
identity switches and the wrong one for continuity.

The difference shows when a ground-truth object goes unmatched for a stretch.
Under `last_match` it keeps its old claim the whole time, and the first pass
honours that claim the moment the old hypothesis comes back within the radius,
whatever is closer now. Preserving a correspondence that does not exist is not
continuity; it is a stale claim outranking a live one.

Splitting the two — `prev_frame_match` for the continuity pass, `last_match`
for the switch count — moves the figures in the direction that says the stale
claims were doing real damage:

| | last-ever | previous frame |
|---|---|---|
| MOT17 MOTA | 52.9% | **53.0%** |
| MOT17 MOTP | 21.8 px | **20.6 px** |
| MOT17 identity switches | 2,452 | 2,656 |
| MOT20 MOTA | 62.6% | **62.5%** |
| MOT20 MOTP | 25.9 px | **21.5 px** |
| MOT20 identity switches | 4,911 | 6,181 |

MOTP improves by 1.2 px on MOT17 and 4.4 px on MOT20 — exactly what you would
expect from no longer holding onto a pairing that has drifted when a better one
is available. The switch counts rise, by 204 and 1,270, because an identity
that returns on a different hypothesis after a gap is a switch and was being
concealed by the stale claim. MOT20 moves more than MOT17 on both, which also
fits: denser scenes produce more gaps and more competing hypotheses to be
wrong about.

Recall and ceiling recovery do not move at all, which is again the tell — this
is the scorer changing its mind about which pairing to keep, not the tracker
finding anything different.

### The amnesty that was worth 1.3 MOTA

MOTChallenge marks regions its annotators declined to label, and removes
hypotheses that fall in them before counting — otherwise a tracker is charged
for finding exactly what the annotator saw and chose not to write down. This
scorer does the same, and for a while it did rather more than the same.

The benchmark's own rule is box overlap: a hypothesis box against a don't-care
box. This scorer has no track box, only a ground-contact point, so the faithful
analogue is whether the point is inside the rectangle. It also amnestied any
track within `match_radius` — 100 px — of the rectangle's own foot point, on
the reasoning that such a track would have counted as a match had the region
been annotated.

The reasoning does not survive being measured. A don't-care region is an area
and its foot point is one point, so the second rule is a 100 px disc hung off
the bottom edge: it forgives tracks well outside a large rectangle and does
nothing for a track inside the top of one. What it was worth:

| MOT17 train | containment only | + the 100 px disc |
|---|---|---|
| MOTA | 52.9% | 54.3% |
| Precision | 91.7% | 93.7% |
| Track-frames discarded | 4,267 | 8,626 |

(Both columns measured before the continuity correction described next, so the
difference between them is the amnesty alone.)

| MOT20 train | containment only | + the 100 px disc |
|---|---|---|
| MOTA | 62.6% | 63.7% |
| Precision | 97.9% | 99.6% |

Recall, identity switches and the ceiling recovery do not move at all, which is
the tell: the disc changes nothing about what the tracker found, only about
what it was charged for. **1.3 points of MOTA on MOT17 and 1.1 on MOT20, from
a scoring rule rather than from tracking.**

A figure that size should not rest on a rule the benchmark does not have, so
the disc is gone and every MOTChallenge number in this document is the
containment-only one. The headline MOTA drops from 54.4% to 52.9%, and roughly
a point of that is this and the rest is the engine changes described below.

If a tolerance is wanted — and there is a real argument for one, since a track
50 px from an unannotated person is being charged for a person nobody wrote
down — the defensible shape is a uniform dilation of the *rectangle*, "close
enough to have matched somebody inside it", not a disc at one corner of it.
That is a change with a number attached and it has not been made here.

### What the clutter term costs and buys

The existence update was corrected twice. The first correction restored the
measurement likelihood `g(z)`, which had been dropped entirely; that is the
sixth defect above. The second restored `(1 - p_D) lambda_c`, and it is worth
separating because it is the one change in this document that made a headline
figure slightly **worse**.

A gated measurement admits two explanations if the entity is really there: the
entity produced it, at density `p_D g(z)`; or the entity was missed and the
measurement is clutter, at density `(1 - p_D) lambda_c`. Only the second is
available if the entity is not there. The coded update carried the first and
not the second, which made it discontinuous at the quantity it exists to
measure — as `g(z)` fell, the posterior fell to zero, while `update_miss` (the
same entity, seen by nobody at all) settles at `r(1-p_D) / (r(1-p_D) + (1-r))`.
**A track offered a badly-fitting detection was punished harder than a track
offered nothing**, with the crossover at `p_D g < (1 - p_D) lambda_c` — at
`p_D = 0.9`, any detection less than a ninth as dense as the clutter around it.
`verification/v16` proves the two updates now agree in the limit, and fails on
the pre-fix expression.

Restoring the term keeps tracks alive through evidence that does not fit, so it
helps most where detections are sparse and costs most where clutter is dense:

| MOT17 train | before | after |
|---|---|---|
| MOTA | 54.4% | 54.3% |
| MOTP | 21.8 px | 22.9 px |
| Recall | 59.0% | 58.9% |
| Identity switches | 2,326 | 2,415 |
| Mostly tracked / lost | 29.9% / 26.5% | 29.5% / 26.1% |
| Ceiling recovered | 108.4% | 108.3% |

| MOT20 train | before | after |
|---|---|---|
| MOTA | 63.7% | 63.7% |
| Recall | 64.3% | 64.4% |
| Mostly lost | 10.3% | 10.2% |
| Ceiling recovered | 114.6% | 114.7% |

MOT is flat to a tenth of a point either way, except for 89 more identity
switches and a point of MOTP. The synthetic scenarios, which include two with
genuinely sparse sensing, move a great deal more:

| Scenario | detection rate | recovery | identity switches | ghosts |
|---|---|---|---|---|
| wildlife (4 h revisit) | 31.7% → **43.5%** | 68.8% → **94.6%** | 79 → 99 | 185 → 250 |
| warehouse (BLE/RFID) | 52.9% → **57.9%** | 109.7% → **120.0%** | 1,046 → 1,137 | 432 → 426 |
| metro | 21.4% → 20.2% | 77.1% → **72.7%** | 44 → 48 | 195 → 220 |
| coordinated-evasion | 96.6% → 96.5% | 109.2% → 109.0% | 21 → **14** | 45 → **40** |
| weather | 80.2% → 80.5% | 105.1% → 105.6% | 17 → **14** | 3 → 3 |
| mule-network | 98.6% → 98.4% | 107.0% → 106.8% | 42 → **40** | 11 → **10** |
| *mean of all 14* | 75.1% → **76.2%** | 106.7% → **109.0%** | 106 → 113 | 179 → 185 |

The eight scenarios not listed do not move at all. Two of the fourteen moved
again afterwards, under the pattern-of-life and `absorb` corrections that
followed — `metro` to 20.6% / 74.4% / 59 / 260 and `weather` to 80.1% /
105.0% / 23 / 10 — so the "after" column above is the clutter term's effect in
isolation rather than the current head, which is what it is there to measure.
The head's own means are 76.2% detection and 109.1% recovery. With both sensor estimates
switched on (`--adaptive-noise`) the mean recovery goes 106.6% → 107.0% and
`coordinated-evasion` improves sharply — 167 → 101 identity switches and 428 →
251 ghosts — because a learned `p_D` and the clutter term are the two halves of
the same reading of a miss.

The trade is real and it is not free: **+2.3 points of mean recovery and +1.1
of mean detection rate, for 7 more identity switches and 6 more ghosts per
scenario on average, and a tenth of a point of MOTA on MOT17.** It is kept
because the previous behaviour was not a different trade, it was incoherent: a
detection that does not fit is evidence that the entity was probably missed and
the measurement is probably clutter, which is precisely the miss case, and an
update that instead drove existence to zero was answering a question nobody
asked. The scenarios where it costs are the ones with dense clutter, where the
right remedy is the pruning threshold rather than a broken posterior.

### The number that matters more than MOTA

MOT's public detections are deliberately weak. Matching them **directly**
against ground truth — as though a perfect tracker simply echoed every
detection it was handed — gives:

| | |
|---|---|
| Detector ceiling, recall | **54.4%** |
| TRACE, recall | **59.0%** |
| **TRACE recovered** | **108.5% of the recall the detections allow** |

No tracker consuming these detections can exceed 54.4% recall by reporting
them. TRACE exceeds it by *coasting through frames the detector missed*, and
those coasted positions still match ground truth. That is precisely what a
tracker is for, and it is the single clearest evidence in this repository that
the engine works on real data.

Quoting a tracker's raw recall without the ceiling invites a comparison against
detectors rather than trackers, so both are reported here and in the tool's own
output.

### Per-sequence, by detector

Performance tracks detector quality closely, which is the correct behaviour for
a kinematics-only tracker.

| Detector | MOTA range | Character |
|---|---|---|
| **SDP** (strongest) | 60.2 – **77.6%** | Best result: MOT17-04-SDP |
| **FRCNN** | 42.3 – 67.1% | Precision routinely above 97% |
| **DPM** (oldest) | 19.1 – 45.9% | Its false positives get promoted to tracks |

DPM's range was 4–38% before the source-credibility work; discounting a source
whose reports disagree with its peers is worth roughly ten MOTA points on the
sequences where the detector is unreliable, and nothing at all where it is not.

The detection threshold has a shallow optimum and falls away either side of it.
On MOT17-02-DPM, sweeping it gives 22.5% MOTA at 0.0, 19.1% at the default
0.15, and 16.9% at 0.30. **The default is no longer the best of the three.**
Earlier versions of this document reported an optimum at 0.15, and that was
measured through the broken matcher; with it corrected, filtering nothing beats
filtering a little by 3.4 points, because recall dominates MOTA and the
precision the filter buys is not worth the recall it costs. The default has not
been changed here — moving it is a tuning decision that should be taken across
all 21 sequences rather than from one — but it is no longer defensible as
measured, and that is worth saying rather than leaving the old justification in
place. An earlier version of this document reported the same sweep as a
monotone loss (20.7% → 12.0%); that no longer reproduces. DPM remains a weak
detector and no threshold rescues it.

## Scalability: MOT20, dense crowds

MOT20 is the crowd split — 62 to 226 people per frame, where association is
hardest and the cost of the engine's pairwise work starts to show. All four
train sequences, on the same pedestrian profile MOT17 uses, with nothing tuned
for them:

| Sequence | People/frame | MOTA | Precision | Recall | Mostly lost | ms/frame |
|---|---|---|---|---|---|---|
| MOT20-01 | 62 | **64.8%** | 98.4% | 66.6% | 8.1% | 12 |
| MOT20-02 | 72 | 57.9% | 98.1% | 59.4% | 5.9% | 20 |
| MOT20-03 | 148 | 62.3% | 98.2% | 64.0% | 11.5% | 47 |
| MOT20-05 | 226 | 63.6% | 97.7% | 65.8% | 9.1% | 78 |
| **Overall** | **127** | **62.5%** | **97.9%** | **64.4%** | **9.4%** | **49** |

| | |
|---|---|
| Detector ceiling, recall | 56.2% |
| TRACE, recall | 64.4% |
| **TRACE recovered** | **114.7% of the recall the detections allow** |

The spread across these four is itself a measurement. MOT20-01 gained 9.3 MOTA
from the matcher fix described below and MOT20-05 gained 0.1, because the exact
solver is only used below 64 rows or columns: at 62 people per frame MOT20-01
is almost always inside that limit and at 226 MOT20-05 is almost never. The
sequences that moved are exactly the ones that were using the code that
changed.

**MOT20 scores higher than MOT17**, on the same profile, and recovers more of
its ceiling. That is not the expected direction and the reason is the
detections: MOT20's are 96% precise where MOT17's average 95% but include DPM
at far less. Density hurts association, but it is a smaller effect than
detection quality, and the denser sequences also give the coasting mechanism
more to work with — a crowd that thins for a few frames is still a crowd.

The cost, though, is real: 78 ms/frame at 226 people. That is 12 frames per
second on one core, so a 25 fps camera at that density needs the area
partitioned across workers. See the scaling measurements below.

These numbers are all much better than the ones this document carried
previously, and the reason is a pair of defects that only a long dense sequence
could expose — written up next, because the way they were found is more useful
than the numbers.

---

## The failure that only a long sequence could show

MOT20's two largest sequences ran at 18% MOTA and 75% mostly-lost while
recovering a quarter of what their detections allowed. That was not a density
limit. Watching the track count against the detection count over the sequence
showed it plainly:

| Frame of MOT20-03 | Detections offered | Tracks held |
|---|---|---|
| 200 | 58 | 45 |
| 800 | 70 | 31 |
| 1400 | 78 | 19 |
| 2000 | 89 | 13 |
| 2400 | 87 | 10 |

Detections steady, tracks decaying to nothing. Tracks were dying and none were
replacing them, so the fault was in birth, and two things were sitting on it.

**Credibility was discounting the only sensor there was.** `SourceCredibility`
multiplies into the birth gate, and its own header already says the fit-to-track
test is circular — a sensor steering a track will fit it however wrong it is —
and that peer disagreement is the only test that is not. MOT has one source, so
the only signal available was the circular one, and in a dense scene it falls
steadily for a reason that has nothing to do with the sensor: ambiguous
association is not the detector's fault. The score fell from 0.80 to 0.42 over
the sequence. Credibility is a *relative* judgement and now returns the neutral
default when only one source has ever reported — there is nothing to compare a
lone sensor against, and nothing left if you disbelieve it. Where peers do
exist the mechanism is untouched; `sensor-drift` still discounts the drifting
camera to 0.434 against a sound neighbour's 0.605 and flags it as against
consensus.

**An absent score was being read as a low score.** MOT20 ships its score column
unset. Every detection therefore came through at the 0.3 confidence floor,
which after the modality weight is 0.285, against a birth threshold of 0.25.
The entire benchmark balanced on that 0.035, and any credibility multiplier
below 0.877 shut birth off outright. Deriving a confidence from a validity flag
asserts something the file never said, and it asserted the worst case.

| MOT20-03 | MOTA | Mostly lost | Recovery of ceiling |
|---|---|---|---|
| before | 18.4% | 74.8% | 26% |
| **after** | **63.4%** | **13.1%** | **117%** |

The same two fixes are worth 2.7 points of ceiling recovery on MOT17, and took
`wildlife` — the sparsest scenario in the suite, and the only one that had ever
recovered *less* than its sensors produced — from 88% to 107%.

What made this findable was a ratio that should have been stable and was not.
Neither number is alarming alone: a dense sequence scoring badly is
unsurprising, and a track count of 10 is only wrong next to 87 detections. It
also could not have been found on MOT17, whose sequences are 600–1050 frames —
short enough that the decay never has time to bite.

---

## When is a velocity estimate worth anything?

Coasting through a gap means predicting where something went, which needs a
velocity. Reacquiring it afterwards needs the same. Both are only as good as
that estimate, and there turns out to be a single ratio that decides whether it
is worth having at all:

```
    typical speed  x  heading-hold time
    ──────────────────────────────────────
           position noise
```

The numerator is how far the entity travels while still going the same way —
the MOU model's own velocity correlation time. If that distance is not large
compared with the measurement error, the filter can never accumulate enough
evidence to measure the speed, because mean-reversion discards the older
evidence before it adds up.

Measured directly, on an entity travelling at a constant 1.50 m/s, over nine
seeds and 200 scans each:

| Ratio | Median estimated speed | 10th–90th percentile |
|---|---|---|
| 0.8 | 3.24 | 1.18 – 6.47 |
| 1.5 | 2.39 | 0.88 – 4.54 |
| 2.4 | 1.21 | 0.56 – 2.21 |
| 4.8 | 1.31 | 0.68 – 2.18 |
| 12 | 1.36 | 0.82 – 2.03 |
| 36 | 1.43 | 1.04 – 1.85 |
| **108** | **1.46** | **1.20 – 1.72** |

Below about 5 the estimate is not an estimate. Note the *direction* of the
error: at low ratios the reported speed is far too **high**, because speed is
the norm of a noisy vector and noise cannot make a norm smaller.

Checked across the shipped profiles, only one was in the bad region:

| Profile | Worst travelling regime |
|---|---|
| VehicleConvoy | 500 |
| Maritime | 432 |
| UrbanHUMINT, CounterTerrorism, OrganisedCrime, Fugitive, Wildlife | 288 |
| Airspace | 144 |
| WarehouseAssets, SportsPitch | 12 |
| IndoorVenue | 8.7 |
| BorderPatrol | 7.2 |
| **CityCameraSurveillance** | **1.4 (walking)** |

A walking pedestrian was given an eight-second heading hold against eight
metres of position noise. That is a defensible description of someone browsing
a concourse and a poor one of someone walking down a corridor, and under it the
velocity estimate is noise — which is why the blackout scenario carries its own
motion models rather than inheriting them.

This is a modelling constraint rather than a bug: a genuinely twisty target
observed by a coarse sensor *has* no measurable velocity, and no amount of
filtering invents one. What it means is that a profile has to be checked
against it before any claim about coasting or reacquisition is worth making.

---

## When the network helps and when it lies

A `MotionConstraint` confines a track to a road, a rail line or a corridor, and
`anpr-corridor` shows it earning its place: it cut the ghost rate from 7.87 to
4.75 per scan and is what makes a tail detectable at all. The `metro` scenario
shows where the same idea goes wrong.

Away from a junction the network has one answer to "which way is it going
here", and projecting to that answer beats free space. At a junction every
branch is admissible, and projecting to the **nearest** is the single worst
thing available: it collapses a belief that ought to span both branches onto
whichever the particle cloud happened to sit closest to, and where that is the
wrong branch the track is lost.

Rebuilding the same metro network with three disjoint lines isolates it:

| | Constraint on | Off | Cost of constraining |
|---|---|---|---|
| No junctions | 95.4% | 99.9% | 4.5 |
| Four junctions | 76.0% | 90.8% | **14.8** |

`RoadNetwork` now finds junctions — three or more segment ends meeting, so a
plain corner is not one — and leaves positions and headings alone within one
tolerance of them. Measured over seven seeds it strictly dominates both
alternatives:

| Metro, median of 7 seeds | Recovery | Ghosts/scan |
|---|---|---|
| junctions projected like any other point | 81.7% | 0.94 |
| **junction-aware** | **85.9%** | 1.83 |
| no constraint at all | 81.7% | 2.01 |

Junctions are found three ways, because a network has three kinds. Three or
more segment *ends* meeting is the metro case. A *crossing*, where two streets
pass through each other sharing no endpoint, is the city-grid case — and
looking only at shared endpoints found no junctions at all in a grid, which is
the one layout where nearly every point of interest is one. A *T*, where one
street ends on the interior of another, is missed by both of those tests and is
a three-way junction all the same. Only an endpoint meeting an endpoint is
excluded: that is a plain corner, where the network still has a single answer.
A 3×2 grid comes out at eight junctions and four corners, which is right.

`anpr-corridor`, whose network is a polyline, is unchanged to a decimal.

---

## Learning what a sensor is actually like

The clutter rate is learned from unassigned detections. `meas_noise_var` beside
it is asserted by the profile and never checked, which costs most exactly when
it matters — conditions change under a deployment and the profile goes on
asserting what it always did.

The evidence is already being computed. The normalised innovation squared for a
detection against its assigned track has expectation equal to the measurement
dimension when the filter's assumptions are right; persistently above that means
the sensor is noisier than claimed, and the ratio is how much. Run
`trace_sim --adaptive-noise` or set `adaptive_meas_noise` to switch it on.

**It ships off, and the measurement is why.** Over nine seeds:

| Scenario | Asserted | Learned | Change |
|---|---|---|---|
| weather | 108.6% | **112.4%** | **+3.8** |
| dark-vessel | 101.8% | 101.8% | — |
| coordinated-evasion | 109.9% | 108.3% | −1.6 |
| wildlife | 98.1% | 95.6% | −2.5 |
| warehouse | 118.8% | 108.8% | **−10.0** |

The estimator is *right about the noise* in every one of these — warehouse
learns that one of its two BLE readers is twelve times noisier than claimed,
and it is. Acting on that widens the association gate, which helps where
detections are sparse and isolated and hurts where they are dense and
confusable. So the capability exists, with the guidance recorded, rather than a
default chosen by hope.

### Two things it had to get right first

**Bias is not noise.** They are different moments of the same residual and call
for opposite responses: a biased sensor should be *distrusted*, a noisy one
merely believed less precisely. Measured about zero, a sensor whose mount has
drifted reads as a noisy one and has its gate widened — the one response that
helps its wrong detections keep hold of tracks. On `sensor-drift` that cost
twelve points. Spread is now measured about the source's own estimated offset:

| Sensor | Learned noise scale |
|---|---|
| sound | 0.82 |
| **biased by six sigma** | **0.82** |
| genuinely 3× noisier | 2.25 |

**A stale prediction is not a noisy sensor.** After a long coast the residual is
dominated by where the track was *guessed* to be. Feeding those in tells the
estimator the sensor is noisy when what is uncertain is the prediction; in
`warehouse` that cost sixteen points on its own. Innovations are now sampled
only from tracks whose prediction is one scan old.

### The same question asked of `p_detection`

`p_detection` decides how much a miss counts against a track, and it was
asserted the same way. The evidence for it is equally available: how often does
a sensor report the tracks it has recently been feeding?
`--adaptive-pd` switches it on.

| Scenario (median of 7 seeds) | Asserted | Learned |
|---|---|---|
| weather | 108.9% | **121.1%** |
| warehouse | 118.8% | 118.8% |
| anpr-corridor | 107.1% | 107.1% |
| transit-hub | 118.7% | 118.7% |
| blackout | 119.7% | 119.5% |
| dark-vessel | 101.8% | 101.5% |
| wildlife | 98.1% | 97.3% |

A better shape than the noise estimate: neutral almost everywhere, and a large
win where conditions genuinely change. MOT17-02-FRCNN is identical in every
figure. Ghost rates are flat or slightly better except on `weather`, where they
go from 0.04 to 0.46 per scan — the cost of keeping tracks alive through
degradation. With both estimates on, `weather` reaches 122.1%.

It is pooled **per source, not per track**, and that is the whole design. A
per-track estimate is circular: a track nothing detects would learn that nothing
detects it, conclude its own misses were uninformative, and become immortal. It
is also floored, so the estimate never says a sensor is hopeless.

**What the estimates look like is informative about the sensors, not just the
engine.** Wide-area sensors come out close to their asserted value — `wildlife`'s
collars at 0.51 against 0.45, `blackout`'s cameras at 0.59–0.77 against 0.70.
Point sensors sit near the floor, because a gate reader stops covering a track
that walks away and the estimator cannot tell that from a miss. That is a real
limit of the measurement, and it is exactly why the scenarios full of gate
readers come out unchanged rather than improved.

### The input both estimates were missing

Both of the above converged on the same gap: **the engine is never told what
each sensor can see.** A gate reader's detection probability could not be
estimated, because the engine cannot distinguish "the reader missed it" from
"the entity walked out of its few metres of coverage". And a silent scan was
read as a dead estate or an empty scene by guessing from whether anything had
reported recently.

`EngineConfig::coverage` is how a deployment says so — optional, and without it
the engine falls back on exactly the inferences it made before. A sensor is now
charged with a miss only where it was actually looking, and a track missed
outside everybody's coverage takes no existence penalty at all.

The estimates it unlocks are checkable against the scenarios' own
configurations:

| Sensor | Truly | Learned, blind | Learned, told what it sees |
|---|---|---|---|
| `blackout` cameras | 0.90 | 0.59 – 0.77 | **0.89 – 0.92** |
| `anpr-corridor` readers | 0.15 | floor (noise) | **0.12 – 0.14** |
| `warehouse` BLE readers | ~0.60 | 0.05 (all at floor) | **0.55 – 0.76** |

Supplying the map alone, with no estimate switched on, is worth **+5.9 points**
of recovery on `warehouse` — patchy reader coverage, which is the case it is
for — and is neutral on every other scenario, at a cost of 0.04 ghosts per scan.

**What bounds any estimate of this kind.** It is conditioned on the track still
existing. With a single sensor, a track that is missed dies and stops producing
evidence against that sensor, so the estimate is biased *upward* — 0.95 against
a true 0.60, in the two-sensor test above run with one sensor removed. It is
accurate where something else keeps tracks alive, which is the case it is
useful in anyway. That was found writing the test, not reasoning about the
design.

### And one that caught me

On its default seed `wildlife` appeared to gain **eighteen points**, and that
was nearly the headline of this section. The median over nine seeds is a loss of
two and a half. It is the same single-seed trap recorded under "A note on test
thresholds" in [PORTING_NOTES.md](PORTING_NOTES.md), fallen into again while
measuring the fix for something else — which is the argument for the
median-over-seeds convention rather than an anecdote about it.

---

## Cost: how the engine scales with crowd size

`trace_bench` sweeps entity count with density held constant, so it measures
scaling in track count rather than the separate effect of packing entities
closer together.

An earlier version of this document claimed latency scaled "close to linearly
… because the chi-square gate keeps the association matrix sparse". That was
wrong. Measured, it was **n^1.82** — and the gate had nothing to do with it.

| Tracks | Median ms/scan | Tracking only | µs per track (tracking) |
|---|---|---|---|
| 10 | 2.0 | 1.5 | 149 |
| 40 | 7.9 | 6.0 | 151 |
| 120 | 28.0 | 18.6 | 155 |
| 270 | 74.6 | 44.4 | 165 |
| 400 | 125.3 | 67.2 | 168 |

**Cost grows as about n^1.12 over the full range — effectively linear.**
Tracking alone is flat at 149–168 µs per track from 10 tracks to 400, which is
the part that had to be linear and is. What growth remains is in the detector
pipeline. Wall-clock figures move a few percent between runs on the same
machine; the exponent does not.

Getting there needed one measurement and two wrong guesses. The obvious
suspects — the all-pairs detector loops, and a betweenness implementation that
turned out to be O(V³) — were both fixed and neither mattered. Adding per-stage
timing to `ScanReport` found the real cost immediately:

| Stage at 270 tracks | Before | After |
|---|---|---|
| **RendezvousWarner** | **751 ms** | **43 ms** |
| score + forecast | 26 ms | 26 ms |
| track + associate | 13 ms | 16 ms |
| everything else combined | 2 ms | 3 ms |

The convergence detector was rebuilding each track's pattern-of-life forecast
*inside* its pair loop, so every track's forecast was recomputed once for every
other track. Hoisting it out cut total scan latency at 270 tracks from 875 ms
to 73 ms. Nothing about it was incorrect, and no correctness test could have
caught it — `tests/test_scaling.cpp` now guards the exponent.

A second ceiling surfaced in the same sweep: the engine tracked exactly 80
entities no matter how many were offered, because `kMaxTracks` was a file-scope
constant. It is now a profile field, and the per-stage breakdown is part of
every report.

**And it was still the dominant term afterwards, because its gate was inert.**
Hoisting the forecast bought a factor of twenty in the constant, not a better
exponent. The same fix gated the pair enumeration on the spatial index, which
looked like the quadratic term dealt with — but the gate's radius was four
times the domain's speed scale, for both parties, over the whole warning
horizon. In any dense scene that is wider than the scene, so the index returned
every pair. A gate whose radius exceeds the area of regard is not a gate, and
it reads exactly like one.

Each pair's own speeds give a far tighter bound — two tracks cannot converge
faster than the sum of their speeds — and it costs two norms to apply, so it is
applied before anything that allocates:

| Stage at 400 tracks | Before | After |
|---|---|---|
| **RendezvousWarner** | **102.1 ms** (56.4%) | **54.7 ms** (43.6%) |
| score + forecast | 38.4 ms | 38.3 ms |
| track + associate | 26.8 ms | 27.0 ms |
| everything else combined | 13.8 ms | 5.3 ms |
| **total scan** | **181.1 ms** | **125.3 ms** |

The capability is unchanged: `transit-hub` still raises its first convergence
warning at the same 15 s lead time, and `evader` is bit-identical.

### Above 400 tracks

Earlier versions of this document said scaling above 400 tracks "has not been
measured". The reason it had not is worth recording, because it looked like a
measurement and was not: the bench's profile caps tracks at 400, so every sweep
point above that measured the same 400 tracks and the curve obediently
flattened. `trace_bench --max-tracks N` raises the cap.

| Tracks | Median ms/scan | µs per track |
|---|---|---|
| 405 | 136.8 | 338 |
| 607 | 239.0 | 394 |
| 910 | 399.2 | 439 |
| **1365** | **674.2** | **494** |

**n^1.23 over the full 10–1365 range.** Per-track cost roughly triples from 10
tracks to 1365 — the detectors' pairwise work, not the tracking — but nothing
falls off a cliff. At 1365 simultaneous tracks the engine manages 1.5 scans per
second on one core.

**What this means in practice.** At 400 simultaneous tracks the engine runs at
about 7 scans per second on one core, and at 226 people per frame on MOT20-05
at 12 frames per second. Comfortable for a 1 Hz camera estate, not for 25 fps
without partitioning the area across workers.

---

## What an appearance model is actually worth

TRACE gained a descriptor field, a per-track appearance model, and an appearance
term in both the association likelihood and the reacquisition score. The
question was how much it buys. The answer on MOT is: **nothing**, and the
measurements are worth recording because the conclusion is counter-intuitive.

The MOT profile sets `appearance_weight` to zero, so `--appearance oracle`
alone changes nothing — it selects a descriptor the association never consults,
and reproduces the `none` row exactly. The weight has to be given explicitly
for the mechanism to run at all:

| Descriptor on MOT17-02-FRCNN | MOTA | Identity switches |
|---|---|---|
| `--appearance none` | **43.2%** | 37 |
| `--appearance oracle` (profile weight, 0.0) | **43.2%** | 37 |
| `--appearance oracle --appearance-weight 0.15` | 42.9% | 34 |
| `--appearance oracle --appearance-weight 0.35` | 42.8% | **27** |

A *perfect* descriptor, handed the true identity of every detection, removes 10
identity switches out of 37 — and **costs** four tenths of a point of MOTA doing
it, because the switches it prevents are worth less than the associations it
perturbs. No reacquisition window or dormancy setting changed that.

The arithmetic explains it, and explains it more starkly than before the
identity-switch rule was corrected. On MOT17-02-FRCNN, with 18,581
ground-truth boxes and 43.2% MOTA, the penalty decomposes as:

| Component | Count | Share of penalty |
|---|---|---|
| Missed detections | ~10,460 | **99.1%** |
| False positives | ~56 | 0.5% |
| Identity switches | 37 | **0.4%** |

Missed detections are capped by the detector — which TRACE already exceeds by
coasting — so appearance can address four parts in a thousand of the penalty.
Eliminating *every* identity switch would be worth 0.2 MOTA points. The old
version of this table put identity switches at 9.3% of the penalty and the
conclusion was already that appearance was not worth it; with the scorer
corrected the case is an order of magnitude stronger. And the
switches that remain are not the kind appearance fixes: they are fragmentation,
where a person goes undetected for seconds and their coasted track has drifted
too far to be recognised as theirs.

(The counts are derived from the reported rates rather than printed directly,
so the two approximate ones carry a rounding band of about twenty. The
conclusion does not turn on it.)

**The mechanism does work where descriptors are discriminative**, and the
`decoy-split` scenario is the cleanest case: a subject hands off to a lookalike
who then leaves along the subject's original heading at the subject's speed.

| Descriptor quality | Followed the subject | Followed the decoy |
|---|---|---|
| none | 9 of 12 seeds | **3 of 12** |
| 0.5 | **12 of 12** | 0 |

A *modest* descriptor closes a gap that a *perfect* one could not touch on MOT,
and the reason is the whole lesson: a mechanism is worth what the failure mode
it addresses is worth. MOT's penalty is 89% missed detections, which appearance
cannot help. `decoy-split`'s single error is a confusion, which is the only
thing appearance addresses.

The same holds for six entities converging on one point, milling within
measurement noise of each other, then dispersing along swapped paths:

Mean over seven seeds, because a single run of this is noisy:

| | identities lost |
|---|---|
| without appearance | 5.7 of 6 |
| with appearance | **2.7 of 6** |

So appearance is implemented, tested and available, and is switched **off** in
the MOT profile — because it was measured there rather than assumed. An earlier
version of this document called an appearance cue "the obvious next step". That
was wrong for this benchmark, and the measurement above is what disproved it.

---

## How to read this against published work

Published MOT17 results using public detections generally sit around 50–60%
MOTA. TRACE at 53.0% sits inside that range, at the lower end, and the reason
it is not higher is worth stating plainly rather than explaining away:

**TRACE has no *learned* appearance model.** Methods at the top of the MOT
leaderboards
lean heavily on re-identification embeddings — learned visual descriptors that
say "this is the same person" when kinematics cannot. MOT's central difficulty
is crowds, where people pass each other constantly and the only reliable
discriminator is what they look like. TRACE is kinematics and behaviour only,
by design, because it is built for domains where there is no image at all: AIS
transponders, RFID readers, collar uplinks, cell-tower hits.

So the honest reading is: on the metric that measures what TRACE actually does —
recovering entity trajectories from a stream of noisy point detections — it
exceeds the input's own ceiling. On the metric that rewards visual
re-identification, it is beaten by methods that do visual re-identification.

The identity switches were the same story — or so it appeared until they were
measured. See "What an appearance model is actually worth" above: on this
benchmark they are dominated by fragmentation rather than by association error,
and even perfect appearance evidence barely moves them.

A related finding, recorded because it was counter-intuitive: making dormant
tracks *easier* to reacquire raised recall against the ceiling but **lowered**
MOTA. In a dense crowd a dormant track has dozens of plausible reappearances
within any generous window, and resurrecting the wrong person costs both a false
positive and an identity switch. The MOT profile therefore reacquires only
across the briefest occlusions (`reacquire_kinematic_s = 1.0`, swept). In sparse
domains, where candidates are few and a lost identity may not resurface for
days, the opposite setting is correct — which is exactly the sort of thing a
`DomainProfile` exists to express.

---

## The same lesson in the synthetic scenarios

Measuring the detector ceiling on MOT prompted the same question of the
simulations: how much of what the *sensors actually produced* did TRACE recover?
Every scenario now reports it, and the sensors' own detection ledger is kept
strictly separate from anything the engine can see.

Medians over twelve seeds, with the spread, because a single run of any of
these is a draw from a high-variance process and reporting one is how a
document ends up describing its luckiest seed. `wildlife` alone spans 88–107%.

| Scenario | Sensors produced | TRACE reported | Recovery | Spread over 12 seeds |
|---|---|---|---|---|
| evader | 70.1% | 94.8% | **134%** | 130 – 138% |
| blackout | 66.6% | 80.2% | **120%** | 118 – 122% |
| transit-hub | 81.3% | 97.5% | **120%** | 118 – 121% |
| spoofing | 84.8% | 98.0% | **116%** | 112 – 118% |
| warehouse | 48.0% | 53.8% | **113%** | 102 – 131% |
| coordinated-evasion | 88.1% | 96.3% | **109%** | 108 – 113% |
| anpr-corridor | 19.9% | 21.4% | **108%** | 94 – 123% |
| decoy-split | 92.0% | 98.8% | **107%** | 106 – 109% |
| mule-network | 91.8% | 98.3% | **107%** | 106 – 108% |
| weather | 77.2% | 80.9% | **105%** | 103 – 107% |
| dark-vessel | 76.0% | 77.2% | **102%** | 96 – 105% |
| sensor-drift | 98.5% | 97.4% | 99% | 98 – 100% |
| wildlife | 46.0% | 41.7% | 90% | 71 – 100% |
| metro | 28.0% | 21.2% | 75% | 71 – 79% |

Above 100% means the engine reported a usable track in scans where no sensor
detected the entity at all, by coasting through the gap.

Eleven of the fourteen recover more than their sensors produced, which is what a
tracker is for. The three that do not are the ones with the least to work with,
in opposite directions. `sensor-drift`'s sensors detect 98% of everything, so
there are almost no gaps left to coast through and 99% is close to the ceiling
of what is available. `wildlife` has four animals reporting every four hours,
and `metro` sees an entity in roughly one scan in four: below about a third of
scans there is not enough of a track to coast from, and both sit under 100%
with a wide spread — `wildlife` spans 71–100% over these twelve seeds, which is
a reminder of how little a single run of it is worth.

These numbers are re-measured at the current head and several moved from the
figures this table used to carry, `metro` and `wildlife` most of all — the
existence update's clutter term and the pattern-of-life correction both change
how long a sparsely-sensed track survives. Nothing here is a single run: all
fourteen are medians over the same twelve seeds, `--seed 1` through `--seed 12`.

**`metro` at 84% is the one real shortfall in the suite**, and unlike
`dark-vessel`'s former 70% it is not an artefact. Position is observed only at
turnstiles and is identical for two travellers standing at one, so the two cues
that carry almost every other scenario — where something is, and how far it is
from everything else — are both absent at exactly the moments anything is
observed. The junction work below recovers part of it; what remains would need
the filter to represent which *branch* an entity is on as a discrete state,
rather than inferring it from a position that is only occasionally available.

This changed the assessment of three scenarios materially. `anpr-corridor` had
been documented as the weakest of the seven on a 20% detection rate; the
sensors only ever produced a detection in 19.9% of truth-scans, because ANPR
readers 400 m apart cover about a fifth of the corridor. It was never a tracking
failure. The same applies to `warehouse` and `wildlife`.

One of them measures the same thing twice under different conditions.
`weather` degrades its sensors mid-run without telling the engine — detection
probability from 0.90 to 0.30, position error from 3 m to 12 m — and the engine
recovers 109% of what the sensors produce in the clear and 103% in fog. The
tracker's grip barely moves; what it loses is the margin coasting gave it,
because a coasted position is only as good as a velocity measured through four
times the noise.

### The shortfall that was not one

An earlier version of this document recorded `dark-vessel` at 70% recovery as
"the one genuine shortfall", and explained it as physics:

> Vessels at 12 knots sampled hourly move 21.6 km between scans, and the
> motion model's own one-scan prediction uncertainty is around 13 km. That is
> a real limit and it is not a tuning problem.

**That explanation was wrong, and the arithmetic in it was an invitation to
stop looking.** The scenario now recovers 105% with a mean position error of
1.7 km, and nothing about the motion model, the scan period or the vessel speed
changed. What changed was a defect in the *simulator*, three layers away from
anything this document was measuring.

`World::step` moved an entity toward its next waypoint and stopped there for
the remainder of the scan. On arriving it took the next waypoint and set a
heading from it — and `Vec2::unit()` of a zero-length delta is `{0, 0}`, so a
*repeated* waypoint set the entity's velocity to zero. Concatenating two path
segments produces a repeated waypoint at the join, which is how every route in
`sim_main.cpp` is built. From then on the step function fell through to its
`1.4` m/s default — a walking pace, meaningless in knots or in the abstract
units of `mule-network`, where it worked out at 5,040 units per scan. The
entity tore through its entire remaining route in a few scans and then stood
still, because a route with no waypoints left is a stationary entity.

So the ground truth the engine was being scored against had entities that
teleported and then stopped. Some scenarios were barely touched; `dark-vessel`
was scored almost entirely against vessels that were not moving as the scenario
said they were.

Fixed, the step function spends its whole travel budget along the route,
waypoint by waypoint, so reaching one mid-scan no longer costs the rest of that
scan and a repeated waypoint costs nothing; and cruise speed is latched once at
first use rather than re-derived from a heading vector that gets rewritten at
every corner. Two things are worth taking from it:

- **The scenarios got harder, not easier.** Entities now traverse their full
  routes, so there is more ground to cover and more handoffs to get wrong.
  `anpr-corridor` fell from 111% recovery to 107%, and `wildlife` from 94% to
  88% before later fixes took it to 97%; the maze, at its larger 21x11
  configuration, went from 83.3% detection and zero identity switches to 78.2%
  and five. Those are the honest numbers for a harder problem, and they are
  reported here rather than the flattering ones.
- **A tidy physical explanation for a bad number is the most expensive kind of
  wrong.** The 21.6 km against 13 km was arithmetic that happened to be true
  and had nothing to do with the result it was explaining.

---

## Reproducing

```bash
./scripts/fetch_mot.sh ./data/mot
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build

./build/src/apps/trace_mot ./data/mot/train                      # full MOT17 train
./build/src/apps/trace_mot ./data/mot/train/MOT17-04-SDP         # the best case
./build/src/apps/trace_mot ./data/mot/MOT20Labels/train          # dense crowds
```

Deterministic under a fixed seed **on a given build**: the same command gives
the same numbers every time. Across builds it is not quite, and the reason is
worth stating because it bounds how precisely any of these figures can be read.

`TRACE_NATIVE_ARCH` is on by default, so the core compiles with `-march=native`
and the SIMD width follows whatever the build machine supports. The particle
filter draws its process noise a vector at a time, so a different lane count
consumes the random stream in a different order: the same algorithm, the same
seed, a different sample. Building with `-DTRACE_NATIVE_ARCH=OFF` gives a
portable binary and a stream that does not depend on the host.

Measured, on this machine, native against generic:

| MOT17 train, 336,891 boxes | native | generic |
|---|---|---|
| MOTA | 53.0% | 53.0% |
| Recall | 59.0% | 59.0% |
| Precision | 91.8% | 91.9% |
| MOTP | 20.6 px | 20.7 px |
| Identity switches | 2,656 | 2,577 |
| Mostly tracked | 29.8% | 30.5% |
| Recovery of ceiling | 108.5% | 108.4% |

So the headline figures are stable to the precision printed, and the finer ones
move by about a percent — MOTA and recall identical, MOTP by a tenth of a pixel,
identity switches by 3%.

**The synthetic scenarios are a different matter.** Same seed, same source, the
two builds side by side:

| | native | generic |
|---|---|---|
| `wildlife` recovery | 94.6% | 91.4% |
| `warehouse` recovery | 120.0% | 117.0% |
| `anpr-corridor` ghost tracks | 969 | 790 |
| `mule-network` ghost tracks | 10 | 33 |
| `coordinated-evasion` identity switches | 14 | 32 |
| `warehouse` identity switches | 1,137 | 989 |

Recovery moves by up to 3.2 points and the ghost and switch counts by more than
a factor of two in either direction. These scenarios carry five to seventeen
entities over a few hundred scans, so a single run is a Monte Carlo sample and
not a measurement. That is why every scenario comparison in this repository is
a median over several seeds, and why a single-seed difference of a few points
should be read as nothing at all — including the ones in this table.

The tool's defaults are what every table here reports; `--min-score` and
`--radius` change the operating point, and the sweep of the first is in
"Per-sequence, by detector" above.

## What is still missing

- **No test-split submission.** These are train-split numbers, scored locally.
  Numbers comparable to the public leaderboard require submitting to the
  evaluation server.
- **No appearance cue**, as discussed above.
- **MOT20 has not been tuned for.** All four sequences now replay on the same
  pedestrian profile MOT17 uses, and score higher than MOT17 does on it; a
  profile fitted to dense crowds has not been tried.
- **Nothing above 226 people per frame has been measured on real data**, and at
  that density one core manages 12 frames per second. Synthetically the engine
  has now been measured to 1365 tracks.
- **Both sensor estimates ship off.** `adaptive_meas_noise` and
  `adaptive_p_detection` are measured above and are opt-in; for a deployment
  whose conditions vary they are likely worth turning on, and for one tuned
  against fixed assumptions they are not free.
- **Sensor coverage is optional, and without it the engine infers.** Supply
  `EngineConfig::coverage` and a miss is known to have happened inside
  somebody's field of view; leave it out and the engine guesses, which it does
  reasonably and still guesses.
- **Detection-rate estimates are conditioned on track survival**, so they are
  biased upward wherever a missed track simply dies. Accurate where something
  else keeps tracks alive.
- **`PARALLEL_ROUTE` fires in none of the fourteen scenarios.** The detector
  wants `brush_pass_m < separation <= parallel_route_m` and a matched heading,
  held for `parallel_scans` scans in a row. Two profiles could not satisfy the
  distance window at all and are fixed — `VehicleConvoy` had the two bounds
  the wrong way round, at 20 and 15, so the window was empty; `WarehouseAssets`
  set `brush_pass_m` to 1.5 m and left `parallel_route_m` inheriting 80 m from
  the urban preset, in a facility whose `coloc_dist_m` is 4 m. Neither was what
  kept the detector quiet.

  `anpr-corridor` exists to demonstrate it "on a genuine tail", overrides the
  window to 15–60 m and the streak to 8, and raises nothing. Instrumented over
  that scenario: 180 pair-scans evaluated, 102 with the heading cosine above
  threshold, 38 satisfying all three conditions at once, and **the longest
  unbroken run is 4** against the 8 it asks for. So the matching is real and
  intermittent, and either the detector is stricter than a tail at a 15%
  detection rate can satisfy or the simulated tail does not hold station well
  enough to be one. That has not been established, and the threshold has
  deliberately not been lowered to the observed maximum, which would be tuning
  to the test rather than fixing anything.
