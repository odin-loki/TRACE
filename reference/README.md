# Asset Tracking Algorithm — ARIA-INTEL multi-target tracking and tactical-intelligence engine

A single-file, edge-deployable intelligence engine that converts raw, uncertain, multi-source location observations into structured, actionable intelligence products — track states with explicit uncertainty, pattern-of-life models, 30-minute rendezvous warnings, eight tradecraft detectors, network role inference, and Bayesian threat scores — in **~28 ms median scan latency on one CPU core, no GPU**. The same codebase retargets to urban HUMINT, maritime, airspace, vehicle convoy, city-camera, border-patrol, and fugitive-tracking domains by swapping a single `DomainProfile` configuration object. This folder ships the reference implementation (`aria_intel.py`), three curated research documents, and a comprehensive law-enforcement / intelligence deployment brief covering camera integration, operational use cases, new domain profiles, and new detector specifications.

---

> **ARIA-INTEL = Algebraic Rendezvous & Intelligence Analyser.** A Poisson Multi-Bernoulli Mixture (PMBM) random-finite-set tracker with Mixed Ornstein–Uhlenbeck (MOU) motion, spatio-temporal pattern-of-life GMMs, three independent rendezvous-prediction methods stacked in parallel, eight tradecraft detectors, Dempster–Shafer multimodal fusion, and Beta–Monte-Carlo threat scoring — fused into one Python module designed for tactical edge hardware.
>
> Six MLPs this is not. This is a principled Bayesian multi-target filter extended with intelligence-specific subsystems: the tracker does not just tell you *where a target is*, it tells you *what they are doing*, *whether two targets are about to meet*, *whether their behaviour matches known tradecraft*, and *how confident you should be* — with every probability explicit and every threshold operationally motivated.

---

## Quick start

```python
import aria_intel as aria
import numpy as np

# Build the engine (Urban HUMINT preset, 9 km × 9 km area)
eng = aria.ARIAIntelEngineV6(
    profile=aria.UrbanHUMINT(),
    area=(-4500, 4500, -4500, 4500),
    high_value_locations=[np.array([1200.0, 800.0])],
)

# Feed one scan of observations
obs = [
    aria.Observation("obs001", 0.0, np.array([100.0, 200.0]),
                     "GEOINT", 0.92, "CAM_NORTH_01"),
]
report = eng.ingest(obs, timestamp=0.0)
print(eng.summary(report))

# Or run the built-in synthetic scenario generator
all_obs, true_traj = aria.generate_scenario(n_scans=20, n_targets=5, seed=42)
for scan_idx, scan_obs in enumerate(all_obs):
    report = eng.ingest(scan_obs, float(scan_idx * 60))
print(eng.performance_report())
```

Each call to `eng.ingest()` returns a structured report dictionary with per-track threat scores, rendezvous warnings, tradecraft events, network roles, clusters, sensor scheduling recommendations, and operational intelligence flags. See [Programmatic API](#programmatic-api) below for every field.

---

## What this project is

### The problem it solves

Most multi-target trackers stop at "track ID + position estimate." Real surveillance operations need more: targets go missing behind buildings and must be reacquired; multiple people close together scramble naive nearest-neighbour assignment; clutter generates fake tracks; a distance check fires only when people are *already* meeting; and you have no principled way to fuse GEOINT, SIGINT, COMMS, HUMINT, and OSINT observations with different reliability and update rates.

ARIA-INTEL addresses all of these in one framework:

- **Probabilistic existence** — every track carries a Bernoulli existence probability `r`; candidates below `r = 0.55` are not reported; tracks survive observation gaps via particle prediction and PoL-based reacquisition.
- **Principled data association** — Gibbs-sampled measurement-to-track assignment (14 sweeps) rather than greedy nearest-neighbour.
- **30-minute rendezvous warning** — three independent methods (geometric velocity intercept, separation-rate extrapolation, PoL cross-prediction) stacked in parallel; validated at **100 % recall (20/20 scenarios)** with **28.1 min mean lead time** on the synthetic test suite.
- **Pattern-of-life modelling** — `K = 5` GMM in `[hour, x, y]` space, fitted after 15 observations, refitted every 5; anomaly scoring against the target's own baseline.
- **Eight tradecraft detectors** — brush pass, SDR winding-number, dead drop, parallel-route surveillance, mode transition (vehicle handoff), loiter anomaly, cover stop, chokepoint surveillance — plus network role inference (HANDLER / COURIER / ASSET).
- **Bayesian threat scoring** — eight evidence dimensions, 250-sample Beta–Monte-Carlo integration, priority tiers IMMEDIATE / HIGH / MEDIUM / LOW / MONITOR.

### Three documents, one engine

| Document | Audience | What it covers |
|---|---|---|
| [`papers/Paper1_Research_Paper.md`](papers/Paper1_Research_Paper.md) | Researchers / reviewers | Academic exposition: PMBM theory, MOU motion, PoL GMM, rendezvous stack, detector registry, validated performance tables, literature positioning |
| [`papers/Paper2_LE_Intel_Brief.md`](papers/Paper2_LE_Intel_Brief.md) | Operators / integrators / policy | **1,400-line comprehensive brief** in five parts: (1) engine foundations explained for non-specialists, (2) camera Re-ID pipeline and city-scale architecture, (3) law-enforcement use cases (CT, organised crime, drugs, fugitives, border, financial crime), (4) intelligence-agency use cases (HUMINT tradecraft, SIGINT CDR, counter-intelligence, safe houses, maritime LE), (5) integration & deployment — new domain profiles as Python code, new detector specs as Python code, sensor architecture, three deployment models, operator API, implementation roadmap |
| [`papers/Technical_Reference.md`](papers/Technical_Reference.md) | Developers | Complete API reference: every class, every report field, every detector, performance optimisation history, file structure map |

Read [`papers/Paper2_LE_Intel_Brief.md`](papers/Paper2_LE_Intel_Brief.md) first if you are deploying for law enforcement or intelligence. Read [`papers/Technical_Reference.md`](papers/Technical_Reference.md) if you are extending the code. Read [`papers/Paper1_Research_Paper.md`](papers/Paper1_Research_Paper.md) if you need the mathematical justification.

---

## Folder structure

```
Asset Tracking Algorithm/
├── README.md                          ← you are here
├── aria_intel.py                      ← reference implementation (2,363 lines)
└── papers/
    ├── Paper1_Research_Paper.md       ← academic research paper
    ├── Paper2_LE_Intel_Brief.md     ← law-enforcement & intelligence deployment brief
    └── Technical_Reference.md       ← developer API reference
```

The implementation lives in a single file at the top level so it can be copied onto edge hardware without resolving package dependencies beyond NumPy and SciPy. All curated documentation is in `papers/`.

> **Naming note.** Older documents cite `aria_intel_v6.py`; the file on disk is `aria_intel.py`. They describe the same engine (`ARIAIntelEngineV6` class). Import as `import aria_intel as aria`.

---

## Where to start reading

**If you have 10 minutes:** read this README's [Quick start](#quick-start) and [Headline findings](#headline-findings), then skim [Paper 2 §5.8](papers/Paper2_LE_Intel_Brief.md) (what a complete deployment delivers).

**If you are an operator or integrator:** read [Paper 2](papers/Paper2_LE_Intel_Brief.md) linearly — Part 1 (engine), Part 2 (cameras), then the use-case part relevant to your domain (Part 3 for LE, Part 4 for intelligence), then Part 5 (integration code).

**If you are a developer:** read [Technical Reference §2–3](papers/Technical_Reference.md) (invocation + PMBM filter), then explore `aria_intel.py` with the [file structure map](papers/Technical_Reference.md) (§16).

**If you are a researcher:** read [Paper 1](papers/Paper1_Research_Paper.md) for the full mathematical exposition and validated benchmark tables.

---

## Installation

```bash
pip install numpy scipy
```

Requirements: Python ≥ 3.10, NumPy ≥ 1.24, SciPy ≥ 1.10. No GPU, no network connection at runtime. The entire engine is self-contained in `aria_intel.py`.

---

## Programmatic API

### Engine construction

```python
import aria_intel as aria
import numpy as np

eng = aria.ARIAIntelEngineV6()                          # defaults: UrbanHUMINT, 9 km area
eng = aria.ARIAIntelEngineV6(profile=aria.Maritime())   # maritime preset
eng = aria.ARIAIntelEngineV6(
    profile=aria.UrbanHUMINT(),
    area=(-4500, 4500, -4500, 4500),                   # xmin, xmax, ymin, ymax (metres)
    high_value_locations=[np.array([1200., 800.])],     # optional HVL list
)
```

### Per-scan ingestion

```python
report = eng.ingest(observations: List[Observation], timestamp: float)
print(eng.summary(report))           # formatted text output
print(eng.performance_report())      # session summary
```

### Observation schema

```python
aria.Observation(
    obs_id="obs001",                          # unique string
    timestamp=1710000000.0,                   # Unix seconds
    position=np.array([100.0, 200.0]),        # shape (2,), metres
    modality="GEOINT",                        # GEOINT | SIGINT | COMMS | HUMINT | OSINT
    confidence=0.92,                          # 0.0 – 1.0
    source_id="CAM_NORTH_01",
)
```

Modality reliability priors (used in Dempster–Shafer fusion): GEOINT 0.90, SIGINT 0.78, COMMS 0.70, HUMINT 0.62, OSINT 0.48.

### Report dictionary — key fields

| Field | Type | Description |
|---|---|---|
| `scan` | int | Monotonic scan counter |
| `timestamp` | float | Scan timestamp |
| `domain` | str | Active `DomainProfile` name |
| `n_obs` | int | Observations ingested this scan |
| `n_tracks` | int | Confirmed tracks (`r ≥ 0.55`) |
| `targets` | List[Dict] | Per-track threat scores, position, velocity, priority tier |
| `rendezvous` | List[Dict] | Rendezvous warnings with ETA, method, confidence |
| `tradecraft` | List[Dict] | Tradecraft events from all detectors |
| `network_roles` | List[Dict] | HANDLER / COURIER / ASSET role assignments |
| `clusters` | List[Dict] | Co-location clusters with betweenness centrality |
| `alerts` | List[Dict] | AnomalyEscalator alerts (SPIKE, ESCALATING, COUNTER_SURVEILLANCE) |
| `sensor_schedule` | List[Dict] | Top recommended collection tasks |
| `operational` | Dict | Velocity analysis, dwell flags, Possibility-PMBM mismatch alarms |
| `all_detections` | Dict[str, List] | Raw per-detector output keyed by detector name |

Full field reference: [Technical Reference §13](papers/Technical_Reference.md).

### Domain profiles (built-in presets)

| Preset | Scan period | RV horizon | Spatial gate | Motion models |
|---|---|---|---|---|
| `UrbanHUMINT()` | 60 s | 30 min | 150 m | foot, vehicle, stationary, fast |
| `Maritime()` | 3 600 s | 120 min | 2 000 m | drifting, transiting, anchored, fast_craft |
| `Airspace()` | 5 s | 10 min | 1 000 m | hovering, fixed_wing, gliding, fast_jet |
| `VehicleConvoy()` | 10 s | 5 min | 30 m | stopped, slow_roll, highway, sprint |

Paper 2 §5.1 adds five more profiles as copy-paste Python code: `CityCameraSurveillance()`, `CounterTerrorism()`, `OrganisedCrimeNetwork()`, `FugitiveTracking()`, `BorderPatrol()`.

### Detector registry

```python
eng.list_detectors()                              # active detector names
eng.register_detector(MyDetector(eng.profile))    # hot-swap at runtime
eng.unregister_detector("MyDetector")
```

Default registered detectors (8): `ExtendedRendezvousWarner`, `ParallelRouteSurveillanceDetector`, `ModeTransitionDetector`, `LoiterAnomalyDetector`, `CoverStopDetector`, `ChokepointSurveillanceDetector`, `NetworkRoleInference`, `LegacyTradecraftDetector`.

Paper 2 §5.2 specifies additional LE/intelligence detectors (`PreAttackPatternDetector`, `AssociateNetworkDetector`, `CDRPatternDetector`, etc.) as copy-paste Python implementations following the `BaseDetector` interface.

### Synthetic scenario generator

```python
all_obs, true_traj = aria.generate_scenario(
    n_scans=35, n_targets=7, area=4000.0, seed=77
)
# all_obs:     list of n_scans observation lists
# true_traj:   (n_scans, n_targets, 4) ground-truth [x, y, vx, vy]
```

Used for benchmarking and unit testing. Parameters match the filter's design `P_DETECTION = 0.85` and Poisson(3.0) clutter rate.

---

## The pipeline at a glance

```
Multi-source observations (GEOINT / SIGINT / COMMS / HUMINT / OSINT)
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  PMBM tracker                                             │
│  Gibbs assignment (14 sweeps) → BernoulliTrack per target  │
│  320-particle MOU filter per track                        │
│  Possibility-PMBM dual existence (r + π_r)                │
└─────────────────────────┬─────────────────────────────────┘
                          ▼
┌───────────────────────────────────────────────────────────┐
│  Pattern-of-Life GMM (K=5, EM, refit every 5 obs)         │
│  Anomaly scoring · location prediction · reacquisition    │
└─────────────────────────┬─────────────────────────────────┘
                          ▼
┌───────────────────────────────────────────────────────────┐
│  Detector pipeline (8 default, hot-swappable)               │
│  Rendezvous warner (3 methods) · tradecraft · network roles │
└─────────────────────────┬─────────────────────────────────┘
                          ▼
┌───────────────────────────────────────────────────────────┐
│  Threat scoring (8-dim Beta-MC) · DS fusion · scheduler    │
└─────────────────────────┬─────────────────────────────────┘
                          ▼
              Structured report dictionary
```

---

## Performance

Author-reported benchmarks on the synthetic `generate_scenario` test suite (20 seeds × 50 scans, 7 targets, 8-detector pipeline active). **Not third-party validated.**

| Metric | Value |
|---|---|
| Median scan latency | **28 ms** (non-PoL scans) |
| Mean scan latency | 51 ms |
| P95 scan latency | 210 ms (PoL cross-predict scans) |
| Max scan latency | 325 ms |
| Throughput | ~20 scans/s, single CPU core |
| Mean position error | 21.8 m |
| Rendezvous warning recall | **100 % (20/20 scenarios)** |
| Mean rendezvous lead time | 28.1 min; 100 % ≥ 20 min; 95 % ≥ 25 min |
| Detection at P_D = 0.40 | 100 % |
| Detection at P_D = 0.25 | 91 % |
| False alarm rate | 0.098 / scan |
| Reacquisition (8-scan gap) | 100 % (10/10) |

Full tables: [Paper 1 §10](papers/Paper1_Research_Paper.md), [Technical Reference §10](papers/Technical_Reference.md).

---

## Headline findings

1. **Whole-pipeline output from one engine.** Kinematics → behaviour → rendezvous → tradecraft → network roles → threat scores, not just track positions.
2. **30-minute rendezvous warning with three stacked methods.** Geometric intercept alone caught 26/39 events in testing; the full stack achieved 100 % recall on 20 independent scenarios.
3. **Edge-deployable.** 28 ms median on one CPU core, no GPU. Scales linearly to 50+ tracks.
4. **Domain-polymorphic.** Same code, different `DomainProfile` — urban HUMINT, maritime, airspace, convoy, and five additional LE/intelligence profiles specified in Paper 2 §5.1.
5. **Composable detectors.** `BaseDetector` plugin interface; register/unregister at runtime without restarting the engine.
6. **Possibility-PMBM mismatch diagnostic.** Dual-track of Bayesian `r` and possibilistic `π_r`; alarms when `|r − π_r| / max(r, π_r) > 0.4` — flags sensor deception or model failure that single-existence trackers cannot see.

---

## Law-enforcement & intelligence brief (Paper 2)

[`papers/Paper2_LE_Intel_Brief.md`](papers/Paper2_LE_Intel_Brief.md) is a self-contained 1,400-line document written for readers new to Bayesian filtering. It is organised in five parts:

| Part | Title | Contents |
|---|---|---|
| **1** | The Engine | Problem statement, PMBM filter, particle filter, MOU motion, PoL, rendezvous warning, eight detectors, network analysis, threat scoring, domain profiles, performance — all explained without assuming prior knowledge |
| **2** | The Camera Problem | Re-ID pipeline, camera topology graph, city-scale mass surveillance architecture, watchlist tracking (London NPPV model), privacy and legal architecture |
| **3** | Law Enforcement Use Cases | Counter-terrorism, organised crime, drug trafficking, fugitive tracking, vehicle surveillance, public order, border control, financial crime — each with domain-profile tuning and detector configuration guidance |
| **4** | Intelligence Agency Use Cases | HUMINT tradecraft detection, SIGINT/phone CDR tracking, counter-intelligence (hostile surveillance teams), safe house mapping, foreign intelligence networks, maritime LE, counter-proliferation |
| **5** | Integration & Deployment | Five new domain profiles as Python code, six new detector specs as Python code, sensor integration architecture, three deployment models (edge laptop / server cluster / federated), operator control interface, implementation roadmap (Phases 1–6), capability summary |

Paper 2 §5.1 domain profiles (`CityCameraSurveillance`, `CounterTerrorism`, `OrganisedCrimeNetwork`, `FugitiveTracking`, `BorderPatrol`) are **specifications** — copy into `aria_intel.py` or load as custom `DomainProfile` instances. They are not yet in the shipped code; the four built-in presets (`UrbanHUMINT`, `Maritime`, `Airspace`, `VehicleConvoy`) are.

---

## Limitations

What this project does **not** do, and is honest about:

- **Not third-party validated.** All benchmark numbers come from the author's synthetic scenario generator. Real-world performance on live sensor feeds is unknown.
- **PoL needs 15 observations.** Tracks observed intermittently (e.g. at P_D = 0.25) may take 60+ scans before PoL-driven detectors activate.
- **P95 latency spikes on PoL scans.** Budget ~325 ms for worst-case scans, not 28 ms.
- **SDR winding-number threshold (0.65) false-positives on circular patrol routes.**
- **Network role classification degenerates for n_tracks < 3.**
- **Paper 2 deployment profiles and detectors are specifications, not shipped code.** The LE/intelligence brief describes how to extend the engine; the reference implementation ships four domain presets and eight default detectors.
- **No camera Re-ID pipeline in the shipped code.** Paper 2 Part 2 describes the architecture; integration is a deployment task.
- **No audit-trail / legal-compliance logging in the shipped code.** Paper 2 §5.5 describes the operator control interface; implementation is a deployment task.

---

## Related work in this repo

| Folder | Relationship |
|---|---|
| [`../Filtering/`](../Filtering/) | GH-SR-IMM heavy-tailed multi-target tracking — complementary filter family; shares the IMM motion-model idea |
| [`../Statistical Generation/`](../Statistical%20Generation/) | Universal Statistical Generator — underpins distributional reasoning in threat scoring |
| [`../Battle Sim/`](../Battle%20Sim/) | Tactical reasoning over tracks ARIA-INTEL would deliver |
| [`../Threat Asessments/`](../Threat%20Asessments/) | Threat-assessment portfolio adjacent to the tradecraft/threat-scoring output |

---

## License

AGPL-3.0 — see [`../modified-license.md`](../modified-license.md) at the repository root.

[← Back to main README](../README.md)
