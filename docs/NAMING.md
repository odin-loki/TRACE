# TRACE

**T**racking · **R**e-identification · **A**ssociation · **C**onvergence · **E**vents

The acronym is the pipeline, in pipeline order. Each letter names a stage that
does real work and can be pointed at in the code.

| Letter | Stage | What it does | Where it lives |
|---|---|---|---|
| **T** | Tracking | Where is each entity, with explicit uncertainty | `core/particle_filter.*`, `core/track.*` |
| **R** | Re-identification | Recognising an entity that dropped out and came back | `PmbmManager::try_reacquire`, `core/pattern_of_life.*` |
| **A** | Association | Which sighting belongs to whom | `GibbsAssigner`, `PmbmManager::merge_duplicates` |
| **C** | Convergence | Who is about to meet whom, and when | `detectors/rendezvous.cpp` |
| **E** | Events | Behaviour worth a human's attention | `detectors/behaviour.cpp`, `detectors/tradecraft.cpp` |

## What the thing actually is

> A domain-neutral engine that turns sparse, uncertain, multi-source sightings
> into persistent identities, learned behaviour, and early warning of
> interactions.

Every word there is load-bearing:

- **sparse and uncertain** — it is built for missed detections, position error,
  clutter and coverage gaps, not for clean data;
- **multi-source** — cameras, gates, transponders and human reports fuse into
  one picture, each weighted by its own reliability;
- **persistent identities** — the output is a *thread* through time, not a
  per-frame detection list;
- **learned behaviour** — each entity gets its own baseline, so "unusual" means
  unusual *for it*;
- **early warning of interactions** — the convergence stack predicts meetings
  before they happen, which is the part most trackers do not attempt.

## Domain-neutral by construction

Nothing in the core knows what it is tracking. Every domain-specific quantity —
scan rate, sensor noise, motion regimes, what counts as a meeting, what counts
as loitering — lives in a `DomainProfile`. Swapping that object retargets the
whole engine. Thirteen profiles ship; see [USE_CASES.md](USE_CASES.md).

## A note on the name

TRACE is a common word, which is good for explaining and bad for searching. For
anything public-facing, a qualifier disambiguates without changing the code:
**TRACE-RFS** (random finite set, the mathematical family the tracker belongs
to) or **OpenTRACE**. The namespace, CLI and library stay `trace` either way.

## Relationship to ARIA-INTEL

This engine is a C++23 port of ARIA-INTEL (*Algebraic Rendezvous & Intelligence
Analyser*), a single-file Python implementation aimed squarely at intelligence
work. TRACE keeps the algorithms and generalises the framing: ARIA-INTEL's
subject matter is now one domain among several, expressed as the intelligence
profile pack rather than baked into the engine.

The port is not a translation. Seven substantive algorithmic defects were found
and fixed along the way, most of them invisible in the original's own metrics —
see [PORTING_NOTES.md](PORTING_NOTES.md). The reference implementation is kept
verbatim under `reference/` for comparison.

## Vocabulary

The intelligence-domain event names are kept because they are precise and
already understood in that field. Each has a plain reading in civil domains:

| Event | Intelligence reading | Civil reading |
|---|---|---|
| `BRUSH_PASS` | A covert handover | Two entities made contact — a custody handover, a passenger transfer |
| `DEAD_DROP` | A drop site used asynchronously | One location used by several entities who never meet — a shared pickup point |
| `SDR_PATTERN` | A surveillance-detection route | A closed loop — circling, patrolling, or lost |
| `PARALLEL_ROUTE` | A mobile tail | Two entities travelling together — a convoy, an escort, a pair |
| `MODE_TRANSITION` | A vehicle handoff | A change of transport — pallet to forklift, passenger to platform |
| `LOITER` | Dwelling with intent | Stalled: stuck stock, a queue, a fault |
| `COVER_STOP` | An unexplained halt | A deviation from this entity's normal route |
| `CHOKEPOINT` | Watching a fixed point | Repeated passes — a bottleneck, a favourite spot, a search pattern |
| `HANDLER` / `COURIER` / `ASSET` | Network roles | Hub / mover / leaf |
