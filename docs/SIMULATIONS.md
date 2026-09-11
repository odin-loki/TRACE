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
detection 80% — **117% of what the cameras actually produced** — mean position
error 1.5 m, **0 identity switches**, **0 ghost tracks**, 0.65 ms median
latency. At 21×11 with 12 panels and 400 scans it holds 78% detection and five
identity switches across three travellers.

---

## 2–10. The scenario suite — `trace_sim`

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
*Result: 97.7% detection, 0.85 m error, 120% of what the sensors produced.*

### 3. `dark-vessel` — a ship switches off its transponder mid-transit
Hourly satellite AIS over a 400 × 300 km box; one vessel goes silent for 25
scans and comes back. Stresses long scan periods, existence decay over a real
gap, and pattern-of-life reacquisition. This is the scenario that exposed the
SI-units defect — it ran at 15% detection until the motion models were fixed.
*Result: 77% detection, 1 identity switch, 102% of what the sensors
produced, reacquired after resurfacing. Long documented at 70% recovery and
explained as a hard limit of vessel speed against scan period; that was a
simulator defect, not physics. See [VALIDATION.md](VALIDATION.md).*

### 4. `anpr-corridor` — plate readers along a road, one vehicle tailing another
Fifteen readers 400 m apart. Stresses sparse point observations, and uses a
`RoadNetwork` motion constraint to confine tracks to the carriageway between
readers. That constraint cut the ghost rate from 7.87 to 4.75 per scan and is
what makes the tail detectable at all — `PARALLEL_ROUTE` fires only with it.

Long documented as the weakest scenario on a 20% detection rate; the readers
cover about a fifth of the corridor, and TRACE recovers **107% of what they
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
*Result: 95% detection, 0.08 ghosts/scan, 134% of what the sensors produced.*

### 7. `wildlife` — GPS collars reporting every four hours for twenty days
Animals looping between den sites and a waterhole. Stresses extreme sparsity,
single-sighting track birth, and pattern-of-life on thin data. The thinnest
input in the suite, and the scenario most improved by not discounting a lone
sensor: with one collar per animal there are no peers to judge it against, and
the engine had been steadily disbelieving its only source.
*Result: 44% detection, **97% of what the sensors produced** — median over
twelve seeds, spread 89–107%. Four animals reporting every four hours is the
thinnest input here, and the widest spread: a single run of this scenario says
little.*

### 8. `spoofing` — a fabricated track reported by a single source
One entity is reported by several independent sensors; another is reported by
one, confidently and consistently. Stresses Dempster–Shafer credibility fusion
and the possibility/necessity pair, which is where a claim no other source
corroborates is supposed to show up.
*Result: 98% detection, 116% of what the sensors produced, 0 identity
switches.*

### 9. `mule-network` — accounts in a behavioural space, not a physical one
"Position" is a two-dimensional behaviour embedding: transaction size against
counterparty diversity. Accounts drift as their behaviour changes; observations
are periodic transaction reports, which are noisy, incomplete and irregular in
exactly the way sensor detections are. Six mules shuttle between three
collection accounts and their own cash-out profiles, among eight ordinary
retail accounts. Nothing in the engine is told what any of it means.

Stresses whether the kinematic and behavioural layers mean anything outside a
metric space. They do: 107% recovery (spread 106–108% over twelve seeds, the
tightest in the suite), and the role classifier recovers mules as
`COURIER` (64% of assignments) and retail accounts as `ASSET` (81%) from
behaviour alone.

Collection accounts split between `HANDLER` and `ASSET`, and the reason is
topological rather than a threshold. The scenario was written expecting the
collection accounts to be the hubs, because every mule deals with one. But
betweenness measures who lies *between* others, and a mule joining one
collection account to its own cash-out profile is the node in the middle of
that path; the collection account sits at one end. The classifier reports the
graph it was given, and the graph disagrees with the scenario's premise.

This scenario is also what turned up the `World::step` waypoint defect, because
its units are not metres — the step function's hardcoded 1.4 m/s fallback came
to 5,040 units per scan here, which was too absurd to explain away.
*Result: 98.8% detection, 0.33 unit error, 107% of what the sensors produced.*

### 10. `sensor-drift` — a camera whose mount slowly slips
One sensor's reports acquire a growing systematic offset while its peers stay
sound. The case `SourceCredibility` exists for, and which nothing tested until
it was written; testing it found the mechanism inert, and fixing that is worth
roughly ten MOTA points on MOT17's weakest detector.
*Result: 97.5% detection, 99% of what the sensors produced.*

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
scenario 9.
- **Lateral movement on a network graph**, with "position" as a service
  embedding. The `MotionConstraint` hook is the natural place to express the
  graph.

**Validation** — MOTChallenge replay is now implemented; see
[VALIDATION.md](VALIDATION.md) and `trace_mot`.
- **MOT20 tuning.** All four sequences now replay, on the shared pedestrian
  profile; none has been tuned for. At 200+ people per frame it is the natural
  scalability test.
- **An appearance cue.** `Observation` would need a descriptor field and the
  association likelihood a term for it. The largest available improvement for
  camera domains, irrelevant to every other one.

---

## Reading a detection rate

Every scenario reports three numbers, not one:

```
  sensor detections  49.0%   (1852 of 3780 truth-scans produced a detection)
  detection rate     61.5%   (2325 of 3780 truth-scans had a track)
  recovery          125.5%   of what the sensors made possible
```

The first is the sensors' own ledger — which ground-truth entities actually
produced a detection. It never reaches the engine; it exists so that a low
detection rate can be attributed correctly. A tracker that reports nothing when
the sensors reported nothing has not failed at anything.

Recovery above 100% means the engine reported a usable track in scans where no
sensor detected the entity at all, by coasting through the gap.

This reframed three scenarios. `anpr-corridor` was documented as the weakest of
the seven on a 20% detection rate; its readers, 400 m apart, only ever produce a
detection in 19.9% of truth-scans, and TRACE recovers 107% of that. The same
applies to `warehouse` and `wildlife`.

`dark-vessel` was documented here as the one genuine shortfall at 70%, with a
physical explanation attached. It now recovers 105%, and nothing about the
motion model or the scan period changed — the shortfall was a defect in
`World::step`, which let a repeated waypoint zero an entity's velocity and drop
it onto a hardcoded walking-pace default. The write-up is in
[VALIDATION.md](VALIDATION.md); the short version is that a tidy physical
explanation for a bad number is the most expensive kind of wrong.

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
