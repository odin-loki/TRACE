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
Fifteen readers 400 m apart. Stresses sparse point observations. **This is the
scenario TRACE handles worst**, and the reason is documented rather than tuned
away: free-space motion has no idea a vehicle is confined to a road, so tracks
coast off the carriageway between readers. Point-sensor domains want a
graph-constrained motion model, which is not implemented.

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
- **Stadium egress.** Thousands of entities in minutes — the scalability wall,
  where the O(n²) convergence check and Brandes betweenness stop being free.
- **Metro network.** Turnstile taps only: purely topological observation with no
  metric position at all. Forces the question of what the filter means when
  "distance" is graph hops.
- **Multi-floor building.** Genuine 3-D, where two entities one metre apart
  vertically are on different floors and cannot interact.
- **Adverse weather.** Detection probability and position noise varying *over
  time* rather than being fixed per sensor.
- **Sensor drift and miscalibration.** One camera slowly developing a position
  bias — the case `SourceCredibility` exists for and which nothing currently
  tests.

**Adversarial**
- **Spoofing and injection.** Fabricated detections designed to create a
  plausible false track. The direct test of the possibility/probability mismatch
  diagnostic, which currently fires constantly without being validated against a
  scenario where it *should*.
- **Decoy and split.** A subject who hands off to a lookalike mid-route.
- **Coordinated evasion.** A team deliberately breaking co-location so the
  network analyser cannot connect them.

**Non-geographic** — the Tier 3 cases in [USE_CASES.md](USE_CASES.md)
- **Lateral movement on a network graph**, with "position" as a service
  embedding.
- **Transaction-space mule network**, to see whether the role classifier finds
  couriers without being told what a mule is.

**Validation**
- **Replay against public datasets.** MOT17/MOT20, WILDTRACK or DukeMTMC would
  give the first numbers in this project not generated by its own simulator.
  This is the single most valuable thing on this list.

---

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
