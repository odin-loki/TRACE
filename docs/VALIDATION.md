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

Public detections, no appearance model, no re-identification network, no
offline processing. One pass, causal, 336,891 ground-truth boxes.

| | |
|---|---|
| **MOTA** | **43.9%** |
| MOTP | 27.3 px |
| Recall | 65.5% |
| Precision | 81.8% |
| Mostly tracked | 21.2% |
| Mostly lost | 2.4% |
| Identity switches | 23,456 |
| Throughput | 12.5 ms/frame, one core |

### The number that matters more than MOTA

MOT's public detections are deliberately weak. Matching them **directly**
against ground truth — as though a perfect tracker simply echoed every
detection it was handed — gives:

| | |
|---|---|
| Detector ceiling, recall | **59.9%** |
| TRACE, recall | **65.5%** |
| **TRACE recovered** | **109.3% of the recall the detections allow** |

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

---

## How to read this against published work

Published MOT17 results using public detections generally sit around 50–60%
MOTA. TRACE at 43.9% is below that, and the reason is worth stating plainly
rather than explaining away:

**TRACE has no appearance model.** Methods at the top of the MOT leaderboards
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

The 23,456 identity switches are the same story. Without appearance there is
nothing to break the tie when two people cross, and adding an appearance cue is
the obvious next step for anyone wanting to use TRACE on camera data
specifically. The `Observation` type would need a descriptor field and the
association likelihood a term for it; nothing structural stands in the way.

A related finding, recorded because it was counter-intuitive: making dormant
tracks *easier* to reacquire raised recall to 110.5% of ceiling but **lowered**
MOTA. In a dense crowd a dormant track has dozens of plausible reappearances
within any generous window, and resurrecting the wrong person costs both a false
positive and an identity switch. The MOT profile therefore reacquires only
across the briefest occlusions (`reacquire_kinematic_s = 0.4`). In sparse
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
