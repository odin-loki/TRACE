# What TRACE can be retrofitted to do

The core answers five questions about **entities observed intermittently by
imperfect sensors**:

1. Where is each one, and how sure are we?
2. Is this the same one we saw before?
3. What is normal for *this* one, and is this normal?
4. Which two are about to come together?
5. Which of these is worth a human's attention first?

Any problem shaped like that is a candidate — the entities do not have to be
people, the sensors do not have to be cameras, and "position" does not have to
be geographic. What follows is organised by how far each use case sits from the
shipped code.

---

## Tier 1 — works today by choosing a profile

Thirteen profiles ship. Construct the engine with one and feed it observations.

| Profile | Domain | Scan | Notes |
|---|---|---|---|
| `UrbanHUMINT` | Foot and vehicle surveillance | 60 s | The reference default |
| `CityCameraSurveillance` | Multi-camera estates | 1 s | Coverage gaps, re-ID handoff |
| `CounterTerrorism` | Pre-attack behaviour | 60 s | Long warning horizon, early dwell trip-wire |
| `OrganisedCrimeNetwork` | Network mapping | 300 s | Long baselines, richer PoL before judging |
| `FugitiveTracking` | Rare-sighting search | 60 s | Single-sighting births, very long dormancy |
| `BorderPatrol` | Crossings and smuggling | 30 s | Lying-up regime, vehicle-to-foot handover |
| `Maritime` | Vessels, AIS | 1 h | Dark-vessel gaps, at-sea rendezvous |
| `Airspace` | Aircraft, radar | 5 s | High detection, fast regimes |
| `VehicleConvoy` | Fleets and tails | 10 s | Tight thresholds, GPS-quality noise |
| `IndoorVenue` | Retail, transit, hospital | 2 s | Metre-scale everything |
| `WarehouseAssets` | RFID/BLE asset tracking | 5 s | Patchy readers, long dormancy |
| `WildlifeTelemetry` | Collars, satellite uplinks | 4 h | Extreme sparsity |
| `SportsPitch` | Player tracking | 40 ms | Dense, fast, heavy occlusion |

### Security and public safety

- **Multi-camera person-of-interest tracking.** The headline case: hold identity
  across cameras and through the gaps between them. `CityCameraSurveillance`.
- **Counter-surveillance.** Detect a team watching a fixed point: `CHOKEPOINT`
  for repeated passes, `PARALLEL_ROUTE` for a mobile tail, `SDR_PATTERN` for
  closed-loop routes.
- **Fugitive search.** Sightings arrive weeks apart. Pattern-of-life
  reacquisition matches a new sighting to a dormant identity by *where and when*
  that person used to be seen.
- **Maritime interdiction.** A vessel switching off AIS is a track that must
  coast on prediction and be reacquired on the far side of the gap. The
  `dark-vessel` simulation is exactly this.
- **Border crossing detection.** Vehicle stops near the line, foot track appears
  beside it: `MODE_TRANSITION`.
- **Crowd and public order.** Cluster analysis identifies the entity others
  route through — `betweenness`, not raw contact count.

### Civil, commercial and scientific

- **Warehouse and yard asset tracking.** Pallets and forklifts on patchy BLE.
  `LOITER` means stalled stock; `CHOKEPOINT` means a door bottleneck;
  `BRUSH_PASS` means a custody handover. Implemented as the `warehouse` sim.
- **Retail analytics.** Dwell time against each shopper's own baseline, queue
  detection, staff-to-customer contact. `IndoorVenue`.
- **Hospital logistics.** Where is the infusion pump, which ward does it live
  in, why has it not moved in three days.
- **Wildlife conservation.** Collar fixes every few hours. Pattern-of-life finds
  den sites and migration corridors; a `COVER_STOP` far from the animal's
  baseline is a kill site, an injury, or a poacher.
- **Livestock and herd health.** An animal that stops following the herd's
  pattern is showing the earliest sign of illness.
- **Sports analytics.** Players through occlusion; `PARALLEL_ROUTE` is marking,
  convergence prediction is an anticipated tackle or interception.
- **Air traffic and space situational awareness.** Convergence prediction is
  conflict detection; for satellites it is conjunction assessment. Same maths,
  different scale — the two-body geometry is a straight substitution for the
  intercept solver.
- **Port and airport ground movement.** Aircraft, tugs and baggage trains on one
  apron, where the expensive failure is two things converging.
- **Search and rescue.** Sparse, unreliable sightings of one missing person is
  precisely the low-`p_detection`, high-value-of-a-single-report regime that
  `FugitiveTracking` encodes.

---

## Tier 2 — needs an adapter, not a new engine

The state is still `[x, y, vx, vy]`; the work is turning domain data into
`Observation`s.

- **Phone CDR / device location.** A cell sector is a position with a large,
  known uncertainty. Feed the sector centroid with `pos_noise_m` set to the
  sector radius and `Modality::COMMS`. Co-location clustering then reconstructs
  a contact network from call records alone.
- **Transit smartcard and turnstile data.** `GateReader` already models this:
  sparse, accurate, identity-anchored point observations.
- **ANPR and toll gantries.** Implemented as `anpr-corridor`, with a
  `RoadNetwork` motion constraint confining tracks to the carriageway between
  readers — without it, free-space motion coasts the estimate into the verge.
  The same constraint serves rail, shipping lanes, corridors and street grids.
- **Ship AIS and aircraft ADS-B.** Direct feeds; `Maritime` and `Airspace`.
- **Drone and robot fleets.** `VehicleConvoy` with tighter thresholds.
- **Contact tracing.** Co-location clustering *is* contact tracing. The
  rendezvous stack additionally predicts an exposure before it happens.

---

## Tier 3 — the interesting ones: non-geographic "position"

Nothing in the filter requires the two dimensions to be metres. Anything with a
meaningful distance and a notion of continuous movement can be tracked, and the
whole behavioural layer comes along.

- **Network security / lateral movement.** Position is a host's location in a
  service-graph embedding. An account "moving" through infrastructure is a
  track; two accounts converging on one asset is a rendezvous; a compromised
  account deviating from its own pattern of life is exactly what the anomaly
  score measures. `SDR_PATTERN` becomes scanning behaviour; `HANDLER` becomes a
  pivot host.
- **Financial crime.** Position is an account's location in a
  transaction-feature space. `COURIER` is a money mule — fast, many contacts,
  low centrality — and the role classifier finds it without being told what a
  mule is. `DEAD_DROP` is an asynchronous shared-account layering pattern.
- **Supply chain.** Containers through ports and depots, tracked in a
  network-position space; `LOITER` is a customs hold.
- **Industrial process monitoring.** A machine's position in sensor space drifts
  as it wears. Pattern of life learns each machine's own normal — which is the
  point, since no two identical machines behave identically.
- **Epidemiology.** Cases as entities in a geographic-plus-time space; cluster
  analysis and convergence prediction do outbreak detection and forecasting.
- **Content and account abuse.** Accounts in a behavioural embedding;
  coordinated inauthentic behaviour looks like `PARALLEL_ROUTE` — many entities
  moving in lockstep at a fixed offset.

Several Tier 3 domains have a natural network structure — a service graph, a
transaction graph, a rail network — and `MotionConstraint` is the hook for it.
`RoadNetwork` handles straight segments; a general graph constraint is the same
interface with a different projection.

**The caveat for Tier 3, now measured rather than guessed.** The
`mule-network` scenario implements the transaction-space case:

- **Tracking transfers.** The engine recovers 107% of available detections in a
  purely behavioural space, with no notion of what the axes mean, and that
  figure is the tightest in the suite across seeds (106–108%).
- **The behavioural detectors transfer.** `BRUSH_PASS` fires on direct
  transfers between accounts.
- **Role inference transfers, once the classifier stops using absolute
  thresholds.** Mules come out as `COURIER` and ordinary accounts as `ASSET`
  from behaviour alone, with nothing in the engine told what a mule is. This
  was documented here as a failure to transfer, and the diagnosis was wrong:
  the classifier's thresholds needed to be relative to the population rather
  than recalibrated per domain, and the scenario itself had made the quantity
  the classifier consumes unobservable. See
  [VALIDATION.md](VALIDATION.md) and [SIMULATIONS.md](SIMULATIONS.md).

What genuinely does not transfer is an assumption about *topology*. Collection
accounts come out split between `HANDLER` and `ASSET` because betweenness
measures who lies between others, and in this network the mules are the bridges
— each joins one collection account to its own cash-out profile. The classifier
reports the graph it was given; the scenario's premise about which nodes were
hubs was what was wrong.

The MOU motion model still assumes continuous movement with inertia — good for
things that move through space, questionable for things that jump
discontinuously through an abstract one. Expect to consider replacing
`MotionModel`; the tracking, association and role layers carry over as they
stand.

---

## What TRACE is not for

- **Single-target tracking with perfect detection.** A Kalman filter is simpler,
  faster and sufficient.
- **Anything needing frame-level detection.** TRACE consumes detections; it does
  not produce them. There is no detector or re-ID model here.
- **Domains with no meaningful distance metric.** If "close" is undefined,
  association has nothing to work with.
- **Legal or safety-critical automated decisions.** Nothing here is validated
  against real data. Every number in this repository comes from its own
  simulations, which is evidence the code does what it is meant to and no
  evidence at all about the world.

---

## Deployment note on surveillance domains

Several Tier 1 use cases are mass-surveillance capabilities. The engine has no
audit logging, no access control, no retention policy and no legal-basis
enforcement, and pretending otherwise would be worse than saying so. Anyone
deploying it against people needs those things built around it, and they are
deliberately not sketched here — that is a decision for whoever is accountable
for the deployment, not a default to be inherited from a framework.
