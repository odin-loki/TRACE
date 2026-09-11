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
| **MOTA** | **48.2%** |
| MOTP | 27.4 px |
| Recall | 60.0% |
| Precision | 90.8% |
| Mostly tracked | 10.0% |
| Mostly lost | 3.5% |
| Identity switches | 19,156 |
| Throughput | 5.0 ms/frame, one core |

At the tool's defaults, which is what the command above runs. An earlier
version of this table reported a different operating point (`--min-score 0`,
`--radius 120`) than the command printed beside it; that configuration gives
47.1% MOTA and 103.7% of ceiling, so the choice is worth about a point.

### The number that matters more than MOTA

MOT's public detections are deliberately weak. Matching them **directly**
against ground truth — as though a perfect tracker simply echoed every
detection it was handed — gives:

| | |
|---|---|
| Detector ceiling, recall | **54.4%** |
| TRACE, recall | **60.0%** |
| **TRACE recovered** | **110.2% of the recall the detections allow** |

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
| **SDP** (strongest) | 49.4 – **70.1%** | Best result: MOT17-04-SDP |
| **FRCNN** | 38.6 – 60.4% | Precision routinely above 95% |
| **DPM** (oldest) | 18.8 – 38.4% | Its false positives get promoted to tracks |

DPM's range was 4–38% before the source-credibility work; discounting a source
whose reports disagree with its peers is worth roughly ten MOTA points on the
sequences where the detector is unreliable, and nothing at all where it is not.

The detection threshold has a shallow optimum and falls away either side of it.
On MOT17-02-DPM, sweeping it gives 18.4% MOTA at 0.0, 18.9% at the default
0.15, and 15.5% at 0.30: a little filtering pays for itself, more does not,
because recall dominates MOTA and beyond that point precision is being bought
with it. An earlier version of this document reported the same sweep as a
monotone loss (20.7% → 12.0%); that no longer reproduces. DPM remains a weak
detector and no threshold rescues it.

## Scalability: MOT20, dense crowds

MOT20 is the crowd split — 62 to 226 people per frame, where association is
hardest and the cost of the engine's pairwise work starts to show. All four
train sequences, on the same pedestrian profile MOT17 uses, with nothing tuned
for them:

| Sequence | People/frame | MOTA | Precision | Recall | Mostly lost | ms/frame |
|---|---|---|---|---|---|---|
| MOT20-01 | 62 | 55.9% | 98.6% | 67.4% | 2.7% | 13 |
| MOT20-02 | 72 | 51.2% | 95.5% | 59.7% | 3.3% | 20 |
| MOT20-03 | 148 | 60.8% | 96.7% | 64.1% | 12.0% | 50 |
| MOT20-05 | 226 | **61.6%** | 95.7% | 65.7% | 10.2% | 83 |
| **Overall** | **127** | **59.9%** | **96.0%** | **64.5%** | **5.0%** | **52** |

| | |
|---|---|
| Detector ceiling, recall | 56.2% |
| TRACE, recall | 64.5% |
| **TRACE recovered** | **114.8% of the recall the detections allow** |

**MOT20 scores higher than MOT17**, on the same profile, and recovers more of
its ceiling. That is not the expected direction and the reason is the
detections: MOT20's are 96% precise where MOT17's average 95% but include DPM
at far less. Density hurts association, but it is a smaller effect than
detection quality, and the denser sequences also give the coasting mechanism
more to work with — a crowd that thins for a few frames is still a crowd.

The cost, though, is real: 83 ms/frame at 226 people. That is 12 frames per
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
| **after** | **60.8%** | **12.0%** | **117%** |

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

| Descriptor on MOT17-02-FRCNN | MOTA | Identity switches |
|---|---|---|
| none | 39.7% | 1039 |
| detection-box geometry | 39.5% | 1055 |
| **oracle — perfect ground-truth identity** | **39.7%** | **1012** |

A *perfect* descriptor, handed the true identity of every detection, moves
identity switches by 2.6% and MOTA not at all. No weight, reacquisition window
or dormancy setting changed that.

The arithmetic explains it. On MOT17-02-FRCNN the MOTA penalty decomposes as:

| Component | Count | Share of penalty |
|---|---|---|
| Missed detections | 9,997 | **89.2%** |
| Identity switches | 1,039 | 9.3% |
| False positives | 166 | 1.5% |

Missed detections are capped by the detector — which TRACE already exceeds by
coasting — so appearance can only address a ninth of the penalty. Eliminating
*every* identity switch would be worth 5.6 MOTA points. And the switches that
remain are not the kind appearance fixes: they are fragmentation, where a person
goes undetected for seconds and their coasted track has drifted too far to be
recognised as theirs.

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
MOTA. TRACE at 48.2% is just under that range, and the reason is worth stating
plainly rather than explaining away:

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
| evader | 70.1% | 94.8% | **134%** | 131 – 138% |
| blackout | 66.6% | 80.2% | **120%** | 119 – 122% |
| transit-hub | 81.5% | 97.7% | **120%** | 118 – 121% |
| spoofing | 84.8% | 98.0% | **116%** | 112 – 118% |
| warehouse | 48.0% | 55.4% | **115%** | 101 – 127% |
| mule-network | 91.8% | 98.8% | **107%** | 106 – 108% |
| decoy-split | 92.0% | 98.8% | **107%** | 106 – 109% |
| coordinated-evasion | 88.1% | 96.8% | **110%** | 109 – 113% |
| weather | 77.2% | 83.7% | **109%** | 106 – 111% |
| anpr-corridor | 19.9% | 21.3% | **107%** | 95 – 124% |
| dark-vessel | 76.0% | 77.2% | **102%** | 98 – 105% |
| sensor-drift | 98.5% | 97.5% | 99% | 98 – 100% |
| wildlife | 46.0% | 44.3% | 97% | 89 – 107% |

Above 100% means the engine reported a usable track in scans where no sensor
detected the entity at all, by coasting through the gap.

Eleven of the thirteen recover more than their sensors produced, which is what a
tracker is for. The two that do not are the two with the least to work with in
opposite directions: `sensor-drift`'s sensors detect 98% of everything, so
there are almost no gaps left to coast through, and `wildlife` has four animals
reporting every four hours — its spread crosses 100% and the median sits just
below it.

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

Deterministic under a fixed seed: the same command gives the same numbers. The
tool's defaults are what every table here reports; `--min-score` and `--radius`
change the operating point, and the sweep of the first is in "Per-sequence, by
detector" above.

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
- **A point sensor's detection rate cannot be estimated this way.** The
  estimator cannot distinguish "the reader missed it" from "the entity walked
  out of the reader's few metres of coverage", so gate and ANPR sources sit at
  the floor. Knowing each sensor's footprint would fix it; the engine is not
  told footprints.
- **Sensor availability is inferred, not known.** A coverage gap is guessed at
  from whether anything reported at all. A real deployment knows which cameras
  are down; there is no interface for it to say so.
