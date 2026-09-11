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
| **MOTA** | **46.2%** |
| MOTP | 27.1 px |
| Recall | 62.0% |
| Precision | 86.3% |
| Mostly tracked | 11.8% |
| Mostly lost | 2.4% |
| Identity switches | 20,233 |
| Throughput | 6.3 ms/frame, one core |

### The number that matters more than MOTA

MOT's public detections are deliberately weak. Matching them **directly**
against ground truth — as though a perfect tracker simply echoed every
detection it was handed — gives:

| | |
|---|---|
| Detector ceiling, recall | **59.9%** |
| TRACE, recall | **62.0%** |
| **TRACE recovered** | **103.6% of the recall the detections allow** |

No tracker consuming these detections can exceed 59.9% recall by reporting
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
| **SDP** (strongest) | 50–71% | Best result: **MOT17-04-SDP at 71.3% MOTA** |
| **FRCNN** | 40–62% | Precision routinely above 95% |
| **DPM** (oldest) | 4–38% | Precision falls to 54–77%; its false positives get promoted |

Raising the detection threshold to suppress DPM's false positives makes MOTA
*worse* (20.7% → 12.0% on MOT17-02-DPM): recall dominates MOTA, so trading it
for precision is a losing exchange. DPM is simply a weak detector and there is
no tuning that rescues it.

## Scalability: MOT20, dense crowds

MOT20 is the crowd split — tens to hundreds of people per frame, where
association is hardest and the O(n^2) parts of the engine start to matter.

| Sequence | People/frame | MOTA | Precision | Recall | Recovery of ceiling | ms/frame |
|---|---|---|---|---|---|---|
| MOT20-01 | ~46 | **51.8%** | 98.9% | 66.2% | 105.2% | 12.0 |
| MOT20-02 | ~56 | 49.2% | 96.0% | 59.9% | 107.1% | 50.9* |

Both score *higher* than the MOT17 average, because MOT20's detections are
cleaner (98–99% precision at the ceiling). MOT20-01 loses no identity for more
than 80% of its life at all: **mostly-lost 0.0%**.

\* MOT20-02 was measured before the cost work described below; expect roughly
a third of that figure now, as MOT20-01 went from 33.4 to 12.0 ms/frame.

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
| evader | 71.6% | 96.8% | **135%** |
| warehouse | 46.7% | 57.1% | **122%** |
| transit-hub | 81.5% | 97.7% | **120%** |
| anpr-corridor | 21.2% | 23.4% | **111%** |
| wildlife | 45.0% | 42.1% | 94% |
| dark-vessel | 76.8% | 53.8% | 70% |

Above 100% means the engine reported a usable track in scans where no sensor
detected the entity at all, by coasting through the gap.

This changed the assessment of three scenarios materially. `anpr-corridor` had
been documented as the weakest of the seven on a 20% detection rate; the
sensors only ever produced a detection in 21.2% of truth-scans, because ANPR
readers 400 m apart cover about a fifth of the corridor. It was never a tracking
failure. The same applies to `warehouse` and `wildlife`.

**`dark-vessel` at 70% is the one genuine shortfall.** Vessels at 12 knots
sampled hourly move 21.6 km between scans, and the motion model's own one-scan
prediction uncertainty is around 13 km. That is a real limit and it is not a
tuning problem: at that sampling rate relative to that speed, the information
simply is not there.

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
