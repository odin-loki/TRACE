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
| **MOTA** | **48.0%** |
| MOTP | 27.4 px |
| Recall | 58.1% |
| Precision | 92.3% |
| Mostly tracked | 7.6% |
| Mostly lost | 4.1% |
| Identity switches | 17,876 |
| Throughput | 5.6 ms/frame, one core |

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
| TRACE, recall | **58.1%** |
| **TRACE recovered** | **106.9% of the recall the detections allow** |

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
| **SDP** (strongest) | 48.6 – **70.2%** | Best result: MOT17-04-SDP |
| **FRCNN** | 38.6 – 60.5% | Precision routinely above 95% |
| **DPM** (oldest) | 14.0 – 39.2% | Its false positives get promoted to tracks |

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

MOT20 is the crowd split — tens to hundreds of people per frame, where
association is hardest and the O(n^2) parts of the engine start to matter.

| Sequence | People/frame | MOTA | Precision | Recall | Recovery of ceiling | ms/frame |
|---|---|---|---|---|---|---|
| MOT20-01 | ~46 | **51.8%** | 98.9% | 66.2% | 105.2% | 12.2 |
| MOT20-02 | ~56 | 37.0% | 95.2% | 41.9% | 74.8% | 13.9 |

The two sequences disagree, and the disagreement is the finding. MOT20-01
beats the MOT17 average and exceeds its detector ceiling (105.2%); MOT20-02
recovers only **74.8%** of what its detections allow — the first sequence in
this repository where TRACE falls materially short of its input.

Precision stays at 95.2%, so the engine is not inventing tracks; it is failing
to hold them. Mostly-lost rises from 0.0% to 23.0% and identity switches to
4,182 across 2,782 frames. MOT20-02 is the same scene as MOT20-01 at higher
density and longer duration, which points at the association step rather than
at the detections: with ~56 people in frame, a track that loses its detection
for a few frames has many plausible continuations, and the gate admits several
of them. This is the same mechanism documented under reacquisition below,
arriving through a different door — and unlike the MOT17 case it costs recall
outright rather than trading it.

It has not been tuned for, deliberately: `MotPedestrianPixels` is one profile
shared by every MOT sequence here, and fitting it to MOT20-02 would make the
MOT17 numbers a different kind of claim.

---

## Cost: how the engine scales with crowd size

`trace_bench` sweeps entity count with density held constant, so it measures
scaling in track count rather than the separate effect of packing entities
closer together.

An earlier version of this document claimed latency scaled "close to linearly
… because the chi-square gate keeps the association matrix sparse". That was
wrong. Measured, it was **n^1.82** — and the gate had nothing to do with it.

| Tracks | Median ms/scan | Tracking only | µs per track |
|---|---|---|---|
| 10 | 2.2 | 1.5 | 220 |
| 40 | 6.4 | 5.3 | 161 |
| 120 | 22.3 | 16.4 | 186 |
| 270 | 72.7 | 39.9 | 269 |
| 400 | 135.3 | 76.8 | 338 |

**Cost now grows as about n^1.14 — effectively linear.** Tracking alone is flat
at ~145 µs per track across the whole range; the residual growth is in the
detector pipeline.

Getting there needed one measurement and two wrong guesses. The obvious
suspects — the all-pairs detector loops, and a betweenness implementation that
turned out to be O(V³) — were both fixed and neither mattered. Adding per-stage
timing to `ScanReport` found the real cost immediately:

| Stage at 270 tracks | Before | After |
|---|---|---|
| **RendezvousWarner** | **751 ms** | **34 ms** |
| score + forecast | 26 ms | 26 ms |
| track + associate | 13 ms | 13 ms |
| everything else combined | 2 ms | 2 ms |

The convergence detector was rebuilding each track's pattern-of-life forecast
*inside* its pair loop, so every track's forecast was recomputed once for every
other track. Hoisting it out cut total scan latency at 270 tracks from 875 ms
to 73 ms. Nothing about it was incorrect, and no correctness test could have
caught it — `tests/test_scaling.cpp` now guards the exponent.

A second ceiling surfaced in the same sweep: the engine tracked exactly 80
entities no matter how many were offered, because `kMaxTracks` was a file-scope
constant. It is now a profile field, and the per-stage breakdown is part of
every report.

**What this means in practice.** At 400 simultaneous tracks the engine runs at
about 7 scans per second on one core: comfortable for a 1 Hz camera estate,
not for 25 fps without partitioning the area across workers. Above 400 has not
been measured.

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

**The mechanism does work where descriptors are discriminative.** Six entities
converging on one point, milling within measurement noise of each other, then
dispersing along swapped paths:

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
MOTA. TRACE at 46.2% is at the low end of that, and the reason is worth stating plainly
rather than explaining away:

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

| Scenario | Sensors produced | TRACE reported | Recovery |
|---|---|---|---|
| evader | 71.5% | 96.3% | **135%** |
| warehouse | 49.0% | 61.5% | **126%** |
| transit-hub | 81.7% | 97.9% | **120%** |
| spoofing | 85.9% | 98.0% | **114%** |
| mule-network | 92.0% | 98.7% | **107%** |
| anpr-corridor | 19.9% | 20.8% | **104%** |
| dark-vessel | 76.8% | 78.8% | **103%** |
| sensor-drift | 98.1% | 97.3% | 99% |
| wildlife | 45.0% | 39.4% | 88% |

Above 100% means the engine reported a usable track in scans where no sensor
detected the entity at all, by coasting through the gap.

This changed the assessment of three scenarios materially. `anpr-corridor` had
been documented as the weakest of the seven on a 20% detection rate; the
sensors only ever produced a detection in 19.9% of truth-scans, because ANPR
readers 400 m apart cover about a fifth of the corridor. It was never a tracking
failure. The same applies to `warehouse` and `wildlife`.

### The shortfall that was not one

An earlier version of this document recorded `dark-vessel` at 70% recovery as
"the one genuine shortfall", and explained it as physics:

> Vessels at 12 knots sampled hourly move 21.6 km between scans, and the
> motion model's own one-scan prediction uncertainty is around 13 km. That is
> a real limit and it is not a tuning problem.

**That explanation was wrong, and the arithmetic in it was an invitation to
stop looking.** The scenario now recovers 103% with a mean position error of
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
  `wildlife` fell from 94% recovery to 88% and `anpr-corridor` from 111% to
  104%; the maze went from 83.3% detection and zero identity switches to 78.2%
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

Deterministic under a fixed seed: the same command gives the same numbers.

## What is still missing

- **MOT20 has been loaded but not tuned for.** It is far denser — up to 200+
  people per frame — and is the natural scalability test.
- **No test-split submission.** These are train-split numbers, scored locally.
  Numbers comparable to the public leaderboard require submitting to the
  evaluation server.
- **No appearance cue**, as discussed above.
