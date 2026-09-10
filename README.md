# TRACE

**T**racking · **R**e-identification · **A**ssociation · **C**onvergence · **E**vents

A domain-neutral C++23 engine that turns sparse, uncertain, multi-source
sightings into persistent identities, learned behaviour, and early warning of
interactions.

Most trackers stop at "track id + position". TRACE keeps going: it holds
identity through coverage gaps, learns what is normal for each entity
individually, predicts meetings before they happen, and ranks what deserves
attention — with the uncertainty attached to every number.

Nothing in the core knows what it is tracking. Cameras, ships, pallets, animals
and players are all the same problem with a different `DomainProfile`.

---

## Quick start

```bash
git clone <this repo> && cd TRACE
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

./build/src/apps/trace_maze          # watch it track through a maze
./build/src/apps/trace_sim --all     # six more scenarios
ctest --test-dir build               # the test suite
```

Only a C++23 compiler and CMake are required. xsimd is vendored. CUDA and Qt are
optional and off by default.

```bash
cmake -S . -B build -DTRACE_WITH_CUDA=ON -DTRACE_WITH_QT=ON
./build/src/apps/gui/trace_console   # live operator console
```

---

## The maze

The headline simulation, and the quickest way to see what the engine does. A
maze is partitioned into camera view panels; travellers walk through it; some
panels are switched off to create blind corridors. Ground truth and the engine's
estimate are drawn together, because the useful view during development is where
they disagree.

```
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|.|. . .|. . .|. . .|. . .|. .|
+ + +-+ +-+ + +-+-+ + + + + + +
|.|. .|. . .|.|. .|. .|.|. .|.|          A   ground truth
+ +-+ +-+-+-+ + + + +-+ +-+-+ +          0   TRACE track
|.|. .|. . . .|.|.|. . .|. . A|          x   no camera coverage
+ +-+-+ + + + + + +-+-+-+ +-+ +          .   covered, coloured by panel
|. .|x x|x|x|.|.|.|. . .|.|. .|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

```bash
./trace_maze --width 31 --height 17 --panels 6x4 --travellers 6 --blind 3
./trace_maze --swap 0.3 --pd 0.6      # make it hard
```

**15×9 maze, 3×3 cameras, 1 blind, 3 travellers, 60 scans:** 83% detection,
1.6 m mean error, **0 identity switches**, 0.08 ghost tracks/scan, 0.29 ms
median scan latency on one core.

---

## Validated on real data

Replays of MOTChallenge sequences — real detections from real detectors on real
video — are the only numbers here not produced by TRACE's own simulator.

**MOT17 train, all 21 sequences, 336,891 ground-truth boxes:**

| | |
|---|---|
| MOTA | 43.9% |
| Recall | 65.5% |
| **Detector ceiling, recall** | **59.9%** |
| **TRACE recovered** | **109.3% of the recall the detections allow** |

The ceiling is what a perfect tracker would get by simply echoing every
detection it was handed. TRACE beats it by coasting through frames the detector
missed — which is the entire job. Best single sequence: **71.3% MOTA**
(MOT17-04-SDP).

TRACE has **no appearance model**, deliberately: it is built for domains with no
image at all — transponders, RFID readers, collar uplinks, cell-tower hits. On
MOT, where crowds make visual re-identification the deciding factor, that costs
it against methods that have one. [The full analysis is in
docs/VALIDATION.md](docs/VALIDATION.md).

The same question asked of the simulations — how much of what the *sensors*
produced did the engine recover? — reframed three of them. `anpr-corridor` had
been the weakest scenario on a 20% detection rate; its readers only ever produce
a detection in 21.2% of truth-scans, and TRACE recovers 111% of that.

```bash
./scripts/fetch_mot.sh ./data/mot        # ~30 MB, annotations only
./build/src/apps/trace_mot ./data/mot/train
```

---

## What one scan produces

```cpp
#include "trace/core/engine.hpp"

trace::EngineConfig cfg;
cfg.profile = trace::CityCameraSurveillance();
cfg.area = {0, 500, 0, 300};
cfg.high_value_locations = {{250.0, 150.0}};

trace::Engine engine(cfg);

std::vector<trace::Observation> scan{
    {"obs-1", 1710000000.0, {120.0, 84.0}, trace::Modality::GEOINT, 0.92, "CAM_NORTH"},
};

const trace::ScanReport report = engine.ingest(scan, 1710000000.0);
std::puts(engine.summary(report).c_str());
```

`ScanReport` is a typed struct, not a dictionary — adding a field is a compile
error at every consumer rather than a silent lookup failure.

| Field | What it carries |
|---|---|
| `targets` | Per-track position, velocity, uncertainty, existence, threat score with breakdown, forecast |
| `rendezvous` | Predicted meetings: who, when, where, by which method, with what confidence |
| `events` | `BRUSH_PASS`, `SDR_PATTERN`, `DEAD_DROP`, `PARALLEL_ROUTE`, `MODE_TRANSITION`, `LOITER`, `COVER_STOP`, `CHOKEPOINT` |
| `network_roles` | Hub / mover / leaf inference from contact structure |
| `clusters` | Co-location groups with betweenness centrality |
| `alerts` | Escalations from watching a track's anomaly trend |
| `sensor_schedule` | Where to point the next collection asset |
| `operational` | Possibility mismatch, high-speed, dwelling and boundary flags |

---

## How it works

```
observations (camera / gate / transponder / human report)
      |
  [T] PMBM tracker ......... Bernoulli existence + 320-particle MOU filter
  [A] Gibbs association .... one-to-one matching, 14 sweeps, then duplicate merge
  [R] re-identification .... dormant tracks reacquired via pattern of life
      |
  pattern of life .......... per-entity GMM over [hour, x, y]
      |
  [C] convergence .......... three independent predictors, stacked
  [E] detectors ............ eight behaviours, hot-swappable at runtime
      |
  threat scoring ........... 8 evidence dimensions, Beta-Monte-Carlo
      |
  ScanReport
```

**Why three convergence predictors.** Geometric intercept is exact when two
entities are walking towards each other and useless when either manoeuvres.
Closure-rate extrapolation catches convergence along a curving route. Pattern-of-life
cross-prediction is the only one that can fire while both parties are
still stationary — it asks each entity's learned routine where it will be later
and looks for a moment when both routines agree. They fail in different
circumstances, so all three run and the most confident wins; agreement between
independent methods raises confidence.

**Confining motion to a network.** The MOU model assumes free space, which is
right for a person in a plaza and wrong for a vehicle between two ANPR readers.
An optional `MotionConstraint` projects the particle cloud onto a road, rail or
corridor network after each prediction step, leaving existence, association and
behaviour detection untouched. Over 30 coasting scans an unconstrained cloud
spreads 2,481 m sideways; a constrained one stays on the carriageway.

```cpp
cfg.motion_constraint = std::make_shared<RoadNetwork>(
    RoadNetwork::from_polyline({{0, 400}, {6000, 400}}, /*tolerance*/ 60.0));
```

**Why probabilistic existence.** Every track carries `r`, the probability it
exists at all, separate from where it is. That is what lets a track survive an
occlusion instead of being deleted on the first missed scan. A second,
possibilistic existence is propagated under different assumptions; when the two
diverge sharply the evidence is internally inconsistent, which is the signature
of a spoofed or failing sensor rather than a moving entity.

---

## Performance

Measured on one core of the development container (AVX-512), Release build.

| Scenario | Tracks | Median | p95 |
|---|---|---|---|
| maze, 3 travellers, 9 cameras | 3–4 | 0.35 ms | 0.86 ms |
| transit-hub, 10 entities | 10–12 | 4.1 ms | 6.6 ms |
| evader, 6 entities | 6–7 | 1.7 ms | 3.4 ms |

Backends:
- **xsimd** (vendored, on by default) vectorises particle propagation — 8 lanes
  under AVX-512. Degrades to identical scalar code when unavailable.
- **CUDA** (`-DTRACE_WITH_CUDA=ON`) for particle propagation, the GMM E step and
  pairwise distances. An accelerator, never a dependency: with no device the
  engine runs the CPU path and says so.
- **Qt 6** (`-DTRACE_WITH_QT=ON`) builds the operator console — live map with
  uncertainty ellipses, forecast paths, ranked track table and event log.

---

## Repository layout

```
include/trace/{core,detectors,backend,sim}/   headers
src/{core,detectors,sim,cuda,apps}/           implementation and applications
scripts/fetch_mot.sh                          fetch MOTChallenge annotations
tests/                                        dependency-free test suite
docs/                                         see below
reference/                                    the original Python implementation
third_party/xsimd/                            vendored
```

| Document | Contents |
|---|---|
| [docs/NAMING.md](docs/NAMING.md) | What TRACE stands for, and the event vocabulary in both intelligence and civil readings |
| [docs/USE_CASES.md](docs/USE_CASES.md) | What this can be retrofitted to do, in three tiers by distance from shipped code |
| [docs/SIMULATIONS.md](docs/SIMULATIONS.md) | Every simulation, what failure mode each one stresses, and further ones worth building |
| [docs/VALIDATION.md](docs/VALIDATION.md) | MOTChallenge replay results, the detector-ceiling method, and how to read them against published work |
| [docs/PORTING_NOTES.md](docs/PORTING_NOTES.md) | Twelve defects found and fixed, why each was invisible, and what changed |

---

## Origin

TRACE is a C++23 port of **ARIA-INTEL**, a single-file Python engine for
intelligence work, kept verbatim under `reference/`. The port generalises the
framing — intelligence is now one domain pack among thirteen — and fixes seven
substantive algorithmic defects found while building the simulations. Four of
them were invisible in the original's own metrics, because it reported peak
track counts but never identity continuity. [The full list is
here](docs/PORTING_NOTES.md); the shortest summary is that association was not
one-to-one, so duplicate tracks were fed the same detection forever and never
decayed.

---

## Honest limitations

- **Nothing here is validated against real sensor data.** Every number in this
  repository comes from its own simulations. That is evidence the code does what
  it is meant to, and no evidence at all about the world. Replaying against
  MOT17 or WILDTRACK is the most valuable next step.
- **Point-sensor domains need a road-network motion model.** Free-space motion
  does not know a vehicle is confined to a road; see the `anpr-corridor`
  scenario, which is the weakest of the seven and documented rather than tuned.
- **Pattern of life needs enough sightings.** Below `pol_min_obs` the anomaly
  score returns 0.5 — unknown, deliberately not alarming.
- **Motion regime identification** requires the per-scan motion difference
  between regimes to exceed the measurement noise. Where it does not, the
  posterior correctly falls back on the transition prior.
- **No detector or re-ID model.** TRACE consumes detections; producing them is
  someone else's job.
- **No audit logging, access control or retention policy.** Several of the
  Tier 1 use cases are mass-surveillance capabilities, and anyone deploying
  against people needs that scaffolding built around this — deliberately not
  provided as a default.

## License

AGPL-3.0, inherited from the reference implementation.
