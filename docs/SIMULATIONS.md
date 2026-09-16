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

## 2–15. The scenario suite — `trace_sim`

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
*Result: 97.6% detection, 0.86 m error, 120% of what the sensors produced
(medians over twelve seeds).*

### 3. `dark-vessel` — a ship switches off its transponder mid-transit
Hourly satellite AIS over a 400 × 300 km box; one vessel goes silent for 25
scans and comes back. Stresses long scan periods, existence decay over a real
gap, and pattern-of-life reacquisition. This is the scenario that exposed the
SI-units defect — it ran at 15% detection until the motion models were fixed.
*Result: 78% detection, 1 identity switch, 102% of what the sensors
produced (medians over twelve seeds), reacquired after resurfacing. Long documented at 70% recovery and
explained as a hard limit of vessel speed against scan period; that was a
simulator defect, not physics. See [VALIDATION.md](VALIDATION.md).*

### 4. `anpr-corridor` — plate readers along a road, one vehicle tailing another
Fifteen readers 400 m apart. Stresses sparse point observations, and uses a
`RoadNetwork` motion constraint to confine tracks to the carriageway between
readers, which is what makes the tail detectable at all — `PARALLEL_ROUTE`
fires only with it.

Run it both ways to see what the constraint is worth. Medians over eight seeds:

| | recovery | ghosts/scan | mean position error |
|---|---|---|---|
| `trace_sim anpr-corridor` | **106%** | **5.8** | **18.7 m** |
| `trace_sim --no-constraint anpr-corridor` | 81% | 7.5 | 22.8 m |

That A/B could not be run until this release. `--no-constraint` existed, this
page quoted a ghost-rate pair it would produce, and `run_anpr_corridor` set the
constraint unconditionally and ignored the flag.

Long documented as the weakest scenario on a 20% detection rate; the readers
cover about a fifth of the corridor, and TRACE recovers **104% of what they
actually produce** (median over twelve seeds, 100–123%). See "Reading a
detection rate" below.

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
*Result: 95% detection, 0.14 ghosts/scan, 134% of what the sensors produced
(medians over twelve seeds).*

### 7. `wildlife` — GPS collars reporting every four hours for twenty days
Animals looping between den sites and a waterhole. Stresses extreme sparsity,
single-sighting track birth, and pattern-of-life on thin data. The thinnest
input in the suite, and the scenario most improved by not discounting a lone
sensor: with one collar per animal there are no peers to judge it against, and
the engine had been steadily disbelieving its only source.
*Result: 44% detection, **98% of what the sensors produced** — median over
twelve seeds, spread 90–103%. Four animals reporting every four hours is the
thinnest input here, and the widest spread: a single run of this scenario says
little.*

### 8. `spoofing` — a fabricated track reported by a single source
One entity is reported by several independent sensors; another is reported by
one, confidently and consistently. Stresses Dempster–Shafer credibility fusion
and the possibility/necessity pair, which is where a claim no other source
corroborates is supposed to show up.
*Result: 98% detection, 116% of what the sensors produced, 0 identity
switches (medians over twelve seeds). The possibility/probability mismatch flags the marginal-quality
rumour on 119 of 119 scans and neither the real entities (0/496) nor the
high-confidence phantom (0/102) — the second of those being a limit of the
method rather than a result, see [PORTING_NOTES](PORTING_NOTES.md) 14.*

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
*Result: 98.4% detection, 0.34 unit error, 107% of what the sensors produced
(medians over twelve seeds).*

### 11. `blackout` — cameras that drop out and come back
`sensor-drift` covers a sensor that degrades; this covers one that vanishes.
Six cameras along a corridor go dark three times, for 12, 25 and 40 scans, while
five people walk through. Stresses dormancy, coasting and reacquisition — the
path with more tunable parameters than any other in the profile, and the one
nothing exercised deliberately until this scenario existed.

The metric is not the detection rate. It is whether an entity keeps its
*identity* across the gap, which is the whole point of a tracker. It found four
separate defects in the reacquisition path and one in how the engine reads an
empty scan; see [PORTING_NOTES.md](PORTING_NOTES.md) defects 29–33. Two of
fifteen walkers kept their identity when the scenario was first run.

Run with `--appearance Q` to give the cameras descriptors of quality Q. The
simulated sensors had none before this — the entire appearance subsystem was
reachable only through MOTChallenge replay.
*Result: 15 of 15 identities kept across the three blackouts, 0 identity
switches, 120% of what the sensors produced (median of twelve seeds, 118–122%).
Kinematics alone carries it: the median without descriptors is 15 of 15 across
seeds, ranging 11–15, and at Q=0.9 every seed keeps every identity — so
appearance buys the variance rather than the median.*

### 12. `decoy-split` — a subject hands off to a lookalike
The classic counter-surveillance manoeuvre. A subject walks a route; a decoy
waits at a meeting point; they stand together for several scans; then the decoy
leaves along the subject's original heading, at the subject's speed, while the
subject turns away. Afterwards the decoy is the one doing everything the subject
was doing.

This is the case an appearance model exists for, and the measurement is the
point:

| Descriptor quality | Followed the subject | Followed the decoy |
|---|---|---|
| none | 9 of 12 seeds | **3 of 12** |
| 0.5 | **12 of 12** | 0 |
| 0.9 | **12 of 12** | 0 |

Kinematics gets it right three times in four and fails the fourth. A *modest*
descriptor closes the gap completely.

Set against [VALIDATION.md](VALIDATION.md)'s finding that a **perfect** oracle
descriptor moves MOTA not at all on MOTChallenge, the pair says something worth
knowing: a mechanism is worth exactly what the failure mode it addresses is
worth. MOT's errors are missed detections, which appearance cannot help;
this scenario's single error is a confusion, which is all appearance addresses.
*Result: 98.8% detection, 107% of what the sensors produced (median of twelve
seeds, 106–109%).*

### 13. `coordinated-evasion` — a team that never stands together
Every network finding in this engine rests on co-location: the contact graph,
the clusters, the betweenness that decides who is a hub. A team that knows this
simply never co-locates. Four people pass material through two dead drops — one
leaves, another collects twenty minutes later — among six members of the public.
Truth co-location between team members: **zero entity-scans**.

So the contact graph finding nothing is the *correct* answer here, not a
failure. The question the scenario actually asks is whether anything else can
see a network built specifically to defeat it. The tradecraft detectors can: a
dead drop is defined by two people using one place and never being there
together, which is the evidence a disciplined team cannot avoid leaving.

It found three defects in a detector that had never been exercised — see
[PORTING_NOTES.md](PORTING_NOTES.md) defects 34–36 — and two in the scenario
as first written, which is worth recording separately because both were the
kind that make a test pass for the wrong reason: staggering the team by having
them stand still at their start points manufactured exactly the evidence the
detector looks for, and every site it flagged was a start position.
*Result: 96.8% detection, 110% of what the sensors produced (median of twelve
seeds). Both real drop sites found in every run, alongside about four
false positives — places where somebody genuinely did stand still where
somebody else had.*

### 14. `weather` — conditions that change under the engine
Every other scenario gives its sensors fixed characteristics and states matching
ones in the profile. Reality does not hold still. Fog rolls in between scans 100
and 200: detection probability falls from 0.90 to 0.30 and position error rises
from 3 m to 12 m, while `p_detection` and `meas_noise_var` go on asserting what
they always said. That mismatch is the commonest way a deployed tracker
degrades.

| Conditions | Sensors produced | TRACE reported | Recovery | Ghosts/scan |
|---|---|---|---|---|
| clear | 90.3% | 98.8% | **109%** | 0.00 |
| fog | 48.5% | 50.1% | **103%** | 0.09 |

**The engine holds up**, which is not what the scenario was written expecting.
Almost all the lost coverage is the sensors' rather than the tracker's. What it
loses is its *margin*: the coasting that let it exceed the sensors by 9% in the
clear buys only 3% in fog, because a coasted position is only as good as a
velocity measured through four times the noise.

Worth noting why it survives a mismatch this large: the one quantity the engine
*learns* rather than asserts — the clutter rate — is the one that moves most, as
the false-alarm rate rises eightfold.

Both of the asserted quantities can be learned instead, from evidence the
engine already has. `--adaptive-noise` learns measurement noise from the
innovations; `--adaptive-pd` learns detection probability from how often a
sensor reports the tracks it has been feeding. On this scenario they take
recovery from 108.9% to 112.4% and 121.1% respectively, and to 122.1% together,
while leaving clear conditions untouched.

Every scenario now hands the engine a coverage map built from its own sensors'
`covers()`, so a sensor is charged with a miss only where it was looking — which
is what makes these estimates trustworthy rather than merely available.

Both estimates ship off, because elsewhere they are neutral or slightly
negative — see
[VALIDATION.md](VALIDATION.md) for the trade, for the two things the noise
estimate had to get right before it was safe, and for why a *point* sensor's
detection rate cannot be estimated this way at all.
*Result: 80.7% detection overall, 104% of what the sensors produced (median of
twelve seeds, 103–106%); fog-phase recovery median 86%, range 78–94%. The
fog-phase figure stood at "median 104%, range 100–112%" here for a long time
after it stopped being true, which is the sort of thing a scenario prints on
every run and nobody re-reads.*

### 15. `metro` — position observed only at stations
Nine stations, three lines, six travellers, turnstile taps and nothing at all in
between. It inverts the usual problem twice.

Between stations there is no observation for scans at a time, so a traveller in
a tunnel is not missing — nobody is looking. And at a station two travellers tap
at the **same coordinate**: position, which settles almost every association
elsewhere in this suite, carries no information at the only moments anything is
observed. What is left is timing and which way each of them goes next, which is
topology rather than geometry.

**The finding is about junctions.** The `RoadNetwork` constraint was costing
recovery rather than buying it, and rebuilding the same network without
junctions says why:

| | Constraint on | Off | Cost of constraining |
|---|---|---|---|
| Three disjoint lines, no junctions | 95.4% | 99.9% | 4.5 |
| The same network, four junctions | 76.0% | 90.8% | **14.8** |

At a junction every branch is admissible, and projecting to the *nearest* is the
one thing that is wrong — it collapses a belief that should span both branches
onto whichever the particle cloud sat closest to. `RoadNetwork` now finds its
junctions (three or more segment ends meeting, so a plain corner is not one) and
leaves positions and headings alone within one tolerance of them; past the
junction each particle is pulled onto whichever branch it actually drifted
towards, so the multi-modality survives where it matters. Over seven seeds:

| | recovery | ghosts/scan |
|---|---|---|
| default, junctions at the network's tolerance | **73.4%** | 0.65 |
| `--junction-radius 0`, junctions projected like any other point | 72.7% | **0.40** |
| `--no-constraint`, no topology at all | 70.7% | 0.65 |

Finding the junctions is worth about three points of recovery over dropping the
topology, and seven tenths of a point over projecting through them — at two
thirds more ghosts than projecting through them. This page used to say the
default "strictly dominates both alternatives", on figures (85.9% at 1.83,
against 81.7% at 0.94 and 81.7% at 2.01) measured several defects ago. It does
not dominate: the middle row is the one to pick if ghosts cost more than
recovery.

Run `--junction-radius 0` to project junctions like any other point,
`--no-constraint` to drop the topology entirely, `--no-coverage` to stop telling
the engine what its turnstiles can see. The junction arm needed a source edit
until this release.
*Result: 21% detection, **74% of what the sensors produced** (median of twelve
seeds, 72–80%) — the lowest in the suite, and the one scenario where the engine
reports materially less than its sensors offered.*

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

Four of the sketches below have since been built — `blackout`, `decoy-split`,
`coordinated-evasion` and `weather` — and between them they found nine defects,
every one in a subsystem that had no scenario exercising it. That is the pattern
worth taking from this list rather than any individual entry on it.

**Sensor and environment**
- **Stadium egress.** Thousands of entities in minutes. `trace_bench` now
  measures cost to 1365 tracks (674 ms/scan, about 1.5 scans/second on one
  core); beyond that needs partitioning the area across workers, which is not
  implemented.
- ~~**Metro network.**~~ Implemented; see scenario 15.
- **Multi-floor building.** Genuine 3-D, where two entities one metre apart
  vertically are on different floors and cannot interact. The only sketch here
  that is architectural rather than a file: `Vec2` is assumed throughout.
- ~~**Adverse weather.**~~ Implemented; see scenario 14.
- ~~**Intermittent sensor failure.**~~ Implemented; see scenario 11.

**Adversarial** — spoofing is implemented; see scenario 8.
- ~~**Decoy and split.**~~ Implemented; see scenario 12.
- ~~**Coordinated evasion.**~~ Implemented; see scenario 13.

**Non-geographic** — the transaction-space case is implemented; see scenario 9.
- **Lateral movement on a network graph**, with "position" as a service
  embedding. The `MotionConstraint` hook is the natural place to express the
  graph.

**Validation** — MOTChallenge replay is implemented; see
[VALIDATION.md](VALIDATION.md) and `trace_mot`.
- **MOT20 tuning.** All four sequences now replay, on the shared pedestrian
  profile; none has been tuned for. At 200+ people per frame it is the natural
  scalability test.
- **Lateral movement on a service graph.** `metro` covers the topological case
  for physical movement; the same shape with "position" as a service embedding
  is the security-domain version, and would reuse the junction work directly.

---

## Reading a detection rate

Every scenario reports three numbers, not one:

```
  sensor detections  49.0%   (1852 of 3780 truth-scans produced a detection)
  detection rate     60.5%   (2287 of 3780 truth-scans had a track)
  recovery          123.5%   of what the sensors made possible
```

The first is the sensors' own ledger — which ground-truth entities actually
produced a detection. It never reaches the engine; it exists so that a low
detection rate can be attributed correctly. A tracker that reports nothing when
the sensors reported nothing has not failed at anything.

Recovery above 100% means the engine reported a usable track in scans where no
sensor detected the entity at all, by coasting through the gap.

This reframed three scenarios. `anpr-corridor` was documented as the weakest of
the seven on a 20% detection rate; its readers, 400 m apart, only ever produce a
detection in 20% of truth-scans, and TRACE recovers 104% of that. The same
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

Everything is deterministic under a seed **on a given build** — the same
`--seed` on the same binary replays bit for bit — so a regression shows up as a
changed number rather than as noise.

Across builds it is not. `TRACE_NATIVE_ARCH` is on by default, so the SIMD width
follows the build machine, and the particle filter draws its process noise a
vector at a time: a different lane count consumes the same random stream in a
different order. Same algorithm, same seed, different sample. Build with
`-DTRACE_NATIVE_ARCH=OFF` for a stream that does not depend on the host, and
see "Reproducing" in [VALIDATION.md](VALIDATION.md) for how far the numbers
move.

**Read the numbers in context.** A high identity-switch count in `warehouse` is
partly the metric's fault: eight entities converge inside the 4 m match radius,
so truth-to-track assignment is genuinely ambiguous there. Metrics measure the
scenario and the tracker together.
