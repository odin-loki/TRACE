# Simulation catalogue

A tracker cannot be tested on its output alone — you need ground truth the
engine never sees. Every simulation here owns a `World` of entities whose true
positions are known, and `Sensor`s that observe it badly. Only `Observation`s
reach the engine; the truth is used solely for scoring.

## Design principle

**A simulation that only tests the happy path tests nothing.** Each scenario
below is built around a specific way tracking fails in the field:

| Failure mode | Why it breaks trackers | Exercised by |
|---|---|---|
| Coverage gaps | Track must coast blind, then be recognised on the far side | maze (blind panels), dark-vessel |
| Identity confusion | Two entities in one field of view swap labels | maze (`--swap`), transit-hub |
| Clutter at full weight | False detections spawn confirmed tracks | maze, evader |
| Sparse detection | A miss carries almost no information, so nothing decays | anpr-corridor, wildlife |
| Dense co-location | Association is genuinely ambiguous | transit-hub, warehouse |
| Constrained topology | Straight-line prediction is wrong by construction | maze |
| Deliberate evasion | Behaviour designed to defeat naive tracking | evader |
| Overlapping sensors | Two sensors report one entity; the second looks like a new one | transit-hub, mule-network |
| Fabricated evidence | A feed reports something that is not there | spoofing |
| Non-physical space | "Position" is behavioural, not geographic | mule-network |
| Miscalibrated sensor | A bias does not average out the way noise does | sensor-drift |
| Crowd scale | Cost per track, and the hard ceiling on track count | trace_bench |

---

## 1. Maze and camera grid — `trace_maze`

**The headline simulation.** A maze partitioned into rectangular camera view
panels, with travellers walking through it, rendered in the console.

A maze is a cheap stand-in for a real camera estate that reproduces the
properties that actually make multi-camera tracking hard: walls force
non-linear routes so straight-line prediction fails; panel boundaries are real
inter-camera handoffs; disabled panels create blind corridors; two travellers
in one panel is precisely the case that defeats nearest-neighbour association.

```bash
./trace_maze                                        # animated, 21x11, 4x3 cameras
./trace_maze --width 31 --height 17 --panels 6x4 --travellers 6 --blind 3
./trace_maze --swap 0.3 --pd 0.6                    # hostile conditions
./trace_maze --scans 200 --quiet                    # metrics only
NO_COLOR=1 ./trace_maze --no-animate                # plain ASCII, final frame
```

```
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|.|. . .|. . .|. . .|. . .|. .|
+ + +-+ +-+ + +-+-+ + + + + + +
|.|. .|. . .|.|. .|. .|.|. .|.|
+ +-+ +-+-+-+ + + + +-+ +-+-+ +
|.|. .|. . . .|.|.|. . .|. . A|      A     ground truth
+ +-+-+ + + + + + +-+-+-+ +-+ +      0     TRACE track
|. .|. .|.|.|.|.|.|. . .|.|. .|      x     no camera coverage
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      .     covered, panel-coloured
```

Every knob is exposed because the interesting behaviour is at the edges: raise
`--swap`, drop `--pd`, disable more panels, and watch where it breaks.

**Typical result** (15×9, 3×3 panels, 1 blind, 3 travellers, 60 scans):
detection 83%, mean position error 1.6 m, **0 identity switches**, 0.08 ghost
tracks/scan, 0.35 ms median latency.

---

## 2–7. The scenario suite — `trace_sim`

```bash
./trace_sim --list        # descriptions and what each stresses
./trace_sim --all
./trace_sim transit-hub --seed 99
```

### 2. `transit-hub` — concourse with overlapping ceiling cameras and ticket gates
Two subjects converge on an atrium from opposite ends while eight commuters
cross the space. Stresses dense co-location and short-range convergence
prediction. The pattern-of-life cross-predictor is the only one of the three
methods that can see the meeting coming while both parties are still walking
ordinary routes.
*Result: 97.7% detection, 0.91 m error.*

### 3. `dark-vessel` — a ship switches off its transponder mid-transit
Hourly satellite AIS over a 400 × 300 km box; one vessel goes silent for 25
scans and comes back. Stresses long scan periods, existence decay over a real
gap, and pattern-of-life reacquisition. This is the scenario that exposed the
SI-units defect — it ran at 15% detection until the motion models were fixed.
*Result: 82% detection, 1 identity switch, reacquired after resurfacing.*

### 4. `anpr-corridor` — plate readers along a road, one vehicle tailing another
Fifteen readers 400 m apart. Stresses sparse point observations, and uses a
`RoadNetwork` motion constraint to confine tracks to the carriageway between
readers. That constraint cut the ghost rate from 7.87 to 4.75 per scan and is
what makes the tail detectable at all — `PARALLEL_ROUTE` fires only with it.

Long documented as the weakest scenario on a 20% detection rate; the readers
cover about a fifth of the corridor, and TRACE recovers **111% of what they
actually produce**. See "Reading a detection rate" below.

### 5. `warehouse` — BLE-tagged pallets and forklifts with door readers
Patchy reader coverage, two pallets deliberately stalled on the floor. Stresses
low detection probability, long dormancy, and chokepoint counting. `LOITER`
fires on exactly the stalled stock — the civil reading of a detector written for
counter-surveillance.

### 6. `evader` — a subject walking a surveillance-detection route
A closed loop around an objective while civilians cross the area. Stresses the
winding-number test and counter-surveillance escalation. Also the scenario that
proved the SDR window must scale with the domain: at a fixed 12 samples the test
was mathematically incapable of reaching threshold on any loop taking longer
than 12 scans.
*Result: 97.2% detection, 0.04 ghosts/scan, 159 SDR detections.*

### 7. `wildlife` — GPS collars reporting every four hours for twenty days
Animals looping between den sites and a waterhole. Stresses extreme sparsity,
single-sighting track birth, and pattern-of-life on thin data.

---

## Further simulations worth building

Sketched with what each would newly stress. The `Scenario`, `World` and `Sensor`
pieces are in place; each is roughly one file.

**Sensor and environment**
- **Stadium egress.** Thousands of entities in minutes. `trace_bench` now
  measures cost to 400 tracks (about 7 scans/second on one core); beyond that
  needs partitioning the area across workers, which is not implemented.
- **Metro network.** Turnstile taps only: purely topological observation with no
  metric position at all. Forces the question of what the filter means when
  "distance" is graph hops.
- **Multi-floor building.** Genuine 3-D, where two entities one metre apart
  vertically are on different floors and cannot interact.
- **Adverse weather.** Detection probability and position noise varying *over
  time* rather than being fixed per sensor.
- **Intermittent sensor failure.** A camera that drops out and returns, rather
  than one that drifts — `sensor-drift` covers the drifting case.

**Adversarial** — spoofing is now implemented; see scenario 8.
- **Decoy and split.** A subject who hands off to a lookalike mid-route.
- **Coordinated evasion.** A team deliberately breaking co-location so the
  network analyser cannot connect them.

**Non-geographic** — the transaction-space case is now implemented; see
scenario 9, including its negative finding about role inference.
- **Lateral movement on a network graph**, with "position" as a service
  embedding. The `MotionConstraint` hook is the natural place to express the
  graph.

**Validation** — MOTChallenge replay is now implemented; see
[VALIDATION.md](VALIDATION.md) and `trace_mot`.
- **MOT20 tuning.** The dense-crowd split loads but has not been tuned for; at
  200+ people per frame it is the natural scalability test.
- **An appearance cue.** `Observation` would need a descriptor field and the
  association likelihood a term for it. The largest available improvement for
  camera domains, irrelevant to every other one.

---

## Reading a detection rate

Every scenario reports three numbers, not one:

```
  sensor detections  46.7%   (1766 of 3780 truth-scans produced a detection)
  detection rate     57.1%   (2160 of 3780 truth-scans had a track)
  recovery          122.3%   of what the sensors made possible
```

The first is the sensors' own ledger — which ground-truth entities actually
produced a detection. It never reaches the engine; it exists so that a low
detection rate can be attributed correctly. A tracker that reports nothing when
the sensors reported nothing has not failed at anything.

Recovery above 100% means the engine reported a usable track in scans where no
sensor detected the entity at all, by coasting through the gap.

This reframed three scenarios. `anpr-corridor` was documented as the weakest of
the seven on a 20% detection rate; its readers, 400 m apart, only ever produce a
detection in 21.2% of truth-scans, and TRACE recovers 111% of that. The same
applies to `warehouse` and `wildlife`. Only `dark-vessel`, at 70%, is a genuine
shortfall.

## Metrics, and why these ones

`sim::Metrics` reports detection rate, mean and max position error, **identity
switches**, and **ghost tracks**.

The last two matter most, and their absence from the reference implementation's
metrics is why four of its seven defects went unnoticed. Peak track count and
position error look fine while a tracker quietly maintains three duplicate
tracks per entity and swaps between them every few scans. Identity switches
catch that immediately.

Everything is deterministic under a seed — the same `--seed` replays bit for
bit — so a regression shows up as a changed number rather than as noise.

**Read the numbers in context.** A high identity-switch count in `warehouse` is
partly the metric's fault: eight entities converge inside the 4 m match radius,
so truth-to-track assignment is genuinely ambiguous there. Metrics measure the
scenario and the tracker together.
