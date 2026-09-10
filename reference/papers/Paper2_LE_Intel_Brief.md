# ARIA-INTEL: Law Enforcement & Intelligence Systems
## Comprehensive Technical Brief

**Author:** Odin Loch  
**System:** ARIA-INTEL v6 — Algebraic Rendezvous & Intelligence Analyser  
**Basis:** `../aria_intel.py` — 2,362 lines, single-file, single-core, edge-deployable  
**Companion docs:** [`Paper1_Research_Paper.md`](Paper1_Research_Paper.md) (academic exposition) · [`Technical_Reference.md`](Technical_Reference.md) (developer API reference)

---

## Document Structure

This brief is written for a reader new to ARIA-INTEL and to the technical foundations of multi-target tracking. It is self-contained. No prior knowledge of Bayesian filtering, stochastic processes, or surveillance systems is assumed.

The document is organised in five parts:

**Part 1 — The Engine:** What ARIA-INTEL is and how every component works, from the mathematical foundations to the operational output. Read this first.

**Part 2 — The Camera Problem:** How to extend ARIA-INTEL to camera-based surveillance networks, including the Re-ID pipeline, city-scale architecture, and legal frameworks.

**Part 3 — Law Enforcement Use Cases:** Counter-terrorism, organised crime, drug trafficking, fugitive tracking, vehicle surveillance, public order, border control, financial crime.

**Part 4 — Intelligence Agency Use Cases:** HUMINT tradecraft detection, SIGINT-driven tracking, counter-intelligence, safe house mapping, foreign intelligence networks, maritime LE.

**Part 5 — Integration and Deployment:** Complete new domain profiles (as Python code), new detector implementations (as Python code), sensor integration architecture, deployment models, operator API, and full capability summary.

---

# ARIA-INTEL Law Enforcement & Intelligence Technical Brief
## Part 1 — The Engine: What ARIA-INTEL Is and How It Works

*Author: Odin Loch*

---

## 1.1 The Problem This Solves

Before explaining what ARIA-INTEL does, it is worth being precise about the problem it solves — because "tracking people" sounds simple until you try to do it correctly at scale.

Imagine you are running surveillance on a network of eight suspected drug traffickers. You have camera feeds from twenty intersections, informant reports coming in irregularly, phone location pings arriving every few minutes, and the occasional eyeballed sighting from a plainclothes officer. Each of these information sources has different reliability, different update rates, and different accuracy. The phone location might be within ten metres; the informant report might be two hours old. Some of your targets are deliberately trying to evade surveillance — taking indirect routes, switching vehicles, meeting briefly on corners. Two of your targets are probably about to meet somewhere, and you want to know where and when before it happens.

Now multiply this across a whole city. Or a fleet of vessels at sea. Or an airspace full of slow-moving drones.

The classical approach to this — the one that most systems still use — is to manually assign observations to tracks by nearest-neighbour matching, and to flag meetings by checking distances between track positions. This works in textbooks. It fails badly in the real world because:

- Observations go missing (a target walks behind a building, or a camera is offline), and the track needs to survive the gap.
- Multiple targets can be close together, and a naive nearest-neighbour assignment will scramble their identities.
- Clutter — false alarms, background pedestrians, noise — will generate fake tracks unless handled explicitly.
- A "near miss" distance check for meetings gives no warning in advance. It fires when people are already together.
- You have no idea whether a track represents a confirmed person or just a noise artefact.

ARIA-INTEL solves all of these problems simultaneously, in a single principled framework, running in 28 milliseconds per update on one CPU core.

---

## 1.2 The Core Idea: Tracking as Probability

The fundamental insight in ARIA-INTEL is to represent every track not as a definite object with a known position, but as a probability distribution. Specifically, two probabilities:

**Existence probability (r):** The probability that this track corresponds to a real target, as opposed to a clutter artefact. When a track is first born from an unassigned observation, it gets r = 0.65 — there is a 65% chance it is a real person, 35% chance it is noise. As more observations arrive that are consistent with the track, r rises. If observations go missing, r falls. A track is only reported to operators once r exceeds 0.55 (the confirmation threshold). It is pruned from memory when r falls below 0.05.

**State distribution:** Even for a track we believe exists (r ≈ 0.9), we do not know exactly where the person is. The track's state — position [x, y] and velocity [vx, vy] — is represented as a cloud of 320 particles, each representing a plausible location and velocity. The particles are weighted by how well each one explains the incoming observations. The weighted average of all particles gives the best position estimate; the spread of the particles gives the uncertainty.

This probabilistic foundation has direct operational consequences:

- A track that vanishes behind a building (no observations for several scans) does not immediately die. Its existence probability decays slowly (each missed scan multiplies r by P_survival = 0.995), and its position particles drift forward according to the motion model. When the target reappears, the particles that predicted the correct reappearance location get high weight, and the track reacquires cleanly.
- A clutter observation spawns a new candidate track at r = 0.65, but with no subsequent corroborating observations it receives only missed-scan updates, and r decays rapidly. It never crosses the confirmation threshold and is pruned silently.
- The uncertainty in position is explicit and carried forward — operators can see how confident the position estimate is, not just where the system thinks the target is.

---

## 1.3 The PMBM Filter: Why This Specific Mathematics

The mathematical framework underlying ARIA-INTEL's tracker is the Poisson Multi-Bernoulli Mixture (PMBM) filter. This is not an arbitrary choice. It is the theoretically optimal solution to the multi-target tracking problem under the assumptions of the Random Finite Set (RFS) framework, which was established by Ronald Mahler in 2003 and has since become the dominant theoretical foundation for academic multi-target tracking research.

To understand what "optimal" means here, consider the assignment problem. When you have six tracks and seven observations arriving at the same time, how do you decide which observation belongs to which track? A greedy approach — assign each observation to the nearest track — works most of the time but catastrophically scrambles identities when two targets pass close together. An exact enumeration of all possible assignments is theoretically perfect but scales as the factorial of the number of observations, which is computationally impossible for more than about eight tracks.

The PMBM filter solves this by representing all plausible global data association hypotheses simultaneously, as a mixture distribution, and weighting each hypothesis by its Bayesian likelihood. It does not commit to a single assignment; it maintains uncertainty about who saw whom, and resolves that uncertainty progressively as subsequent scans arrive.

In ARIA-INTEL's implementation, assignment is handled by a Gibbs sampler running 14 sweep passes per scan. The Gibbs sampler is a Markov Chain Monte Carlo technique that draws samples from the joint association probability by iteratively sampling each track's assignment conditioned on all others. After 14 sweeps, the samples represent the high-probability region of the association space. This gives near-optimal association in O(N_tracks × N_observations) time rather than the O(N!) time of exact enumeration.

The "Mixture" in PMBM is important. The system maintains a collection of Bernoulli components — one per tracked hypothesis — and each component carries its own existence probability and particle filter. This is what allows the system to handle uncertain cardinality: it does not commit to "there are exactly six people here"; it maintains a distribution over the number of targets, letting each Bernoulli component vote on whether its associated observations represent a real person.

---

## 1.4 The Particle Filter: Tracking Through Uncertainty

Each confirmed track in ARIA-INTEL maintains a Sequential Importance Resampling (SIR) particle filter. Understanding what this does is important for understanding what the system can and cannot track.

A particle filter represents a probability distribution as a collection of samples — "particles" — each associated with a weight. In ARIA-INTEL, each track has 320 particles. Each particle represents one hypothesis about the target's current state: a position [x, y] and velocity [vx, vy].

At each scan, three things happen:

**Prediction:** Every particle is propagated forward in time according to the motion model. The particles spread out — position uncertainty grows because the target might have moved in any direction. This represents "I know where they were; I don't know exactly where they are now."

**Update:** When a new observation arrives, each particle is scored by how well it predicts the observation. A particle that predicted the target would be near the observed position gets its weight increased. One that predicted the target would be on the other side of the city gets its weight reduced toward zero. After normalisation, the particle cloud has shifted toward the observed position.

**Resampling:** When the effective sample size — a measure of how spread the weights are — drops below 40% of the particle count, the filter resamples: particles with high weight are duplicated, particles with near-zero weight are discarded. This prevents the filter from "collapsing" to a single hypothesis.

The key advantage of a particle filter over a Kalman filter (the classical alternative) is that it makes no assumption about the shape of the distribution. A Kalman filter assumes positions are Gaussian — symmetric, bell-curved. Real targets make sharp turns, accelerate suddenly, and stop without warning. The particle filter handles all of these naturally because each particle can be anywhere; the filter's accuracy degrades gracefully rather than catastrophically when the Gaussian assumption breaks down.

**The auxiliary trajectory update:** When two consecutive observations are available, ARIA-INTEL performs a second update pass weighting particles by how well their velocity explains the change in observed position. This distinguishes between a target that is moving north and one that is stationary — two very different operational situations that a position-only update cannot separate quickly.

---

## 1.5 Motion Models: Teaching the System How People Move

Every particle filter needs a motion model — a rule for predicting where a particle will be at the next time step. The choice of motion model determines how well the tracker handles different target behaviours.

ARIA-INTEL uses the Mixed Ornstein-Uhlenbeck (MOU) process rather than the standard constant-velocity or constant-acceleration models. This is one of the most important algorithmic choices in the system and deserves a clear explanation.

**The problem with constant-velocity:** A constant-velocity model says "the target is moving at [vx, vy], so at the next time step they will be at [x + vx·dt, y + vy·dt], plus some noise." This works for aircraft and vehicles on motorways. It fails for pedestrians, who stop at traffic lights, turn corners, and reverse direction without warning. Over a 60-second scan interval, a pedestrian's velocity is essentially unpredictable. Adding large process noise to handle this makes the filter imprecise. Adding small noise makes it brittle.

**The Ornstein-Uhlenbeck process:** The OU process is a mean-reverting stochastic process, originally from physics (Brownian motion with friction). Instead of "velocity stays the same plus noise," it says "velocity tends back toward zero, with some randomness." The speed of reversion is controlled by θ (theta), and the amount of randomness by σ (sigma).

For a pedestrian: high θ (say 0.30), low σ (say 2.0 m/s). Velocity reverts to zero quickly — which is physically correct because pedestrians often stop and turn. The steady-state velocity variance σ²/(2θ) = 4/0.6 ≈ 6.7 m²/s², consistent with typical pedestrian speeds.

For a vehicle: low θ (0.10), high σ (8.0 m/s). Velocity persists for much longer before reverting — vehicles maintain speed between intersections. Steady-state variance = 64/0.2 = 320 m²/s², consistent with urban driving.

For a stationary observer/lookout: very high θ (2.00), very low σ (0.5 m/s). Velocity reverts to zero almost instantly — the entity is expected to stay put. Steady-state variance ≈ 0.063 m²/s², essentially stationary.

The discrete-time update for each particle at scan interval dt is:

```
alpha = exp(-theta * dt)
sigma_v = sigma * sqrt((1 - exp(-2*theta*dt)) / (2*theta))

V(t+dt) = alpha * V(t) + sigma_v * noise
X(t+dt) = X(t) + dt * (V(t) + V(t+dt)) / 2   [trapezoidal integration]
```

The trapezoidal integration rule (averaging old and new velocity) prevents the position lag that affects Euler integration at long scan intervals.

**Model mixing (IMM-style):** ARIA-INTEL does not assign a single motion model to a track. Instead, each particle carries a model weight vector μ representing the probability that the target is currently in each motion class. At each prediction step, a model index is sampled per particle from μ, and the appropriate θ, σ are applied. The model weights evolve via the model transition matrix MODEL_TRANS — for example, a foot-model particle has an 85% chance of remaining in foot model next scan, 10% chance of switching to vehicle, etc. This allows the tracker to handle a target that gets into a car: over several scans, the vehicle-class particles will start fitting the observations better, and the model weight distribution will shift from foot to vehicle automatically.

The dominant_model field in every track output reports which model currently has the highest weight. This is operationally useful: it tells you not just where a target is, but how they are moving.

---

## 1.6 Pattern of Life: Learning What "Normal" Looks Like

The Pattern-of-Life (PoL) subsystem is arguably ARIA-INTEL's most operationally powerful capability. It answers the question: "Is this person's current behaviour consistent with their normal routine, or are they doing something unusual?"

Every confirmed track maintains a PatternOfLife object that learns the target's habits by fitting a Gaussian Mixture Model (GMM) to their observation history — but with a crucial twist.

**The 3D spatio-temporal representation:** The GMM is not fitted in 2D space [x, y]. It is fitted in 3D space [hour_of_day, x, y]. The temporal coordinate is the fractional hour within a 24-hour cycle (timestamp mod 86400 divided by 3600). This means the model jointly learns when and where a target typically appears.

Why does this matter? Consider a target who always visits a particular building between 8:00 and 9:00 AM. If you tracked only position, you would learn "this person visits location X." But if someone else has their phone, they might also visit location X — you would not flag that as anomalous. With the spatio-temporal model, visiting location X at 3:00 AM is highly anomalous, even if visiting it at 8:30 AM is completely routine. The model captures the joint routine, not just the spatial one.

**Fitting the model:** The GMM uses 5 Gaussian components, fitted via the Expectation-Maximisation (EM) algorithm. EM alternates between:
- E-step: given current Gaussians, assign each observation a soft membership probability for each component
- M-step: given the soft memberships, update each Gaussian's mean, covariance, and weight to maximise likelihood

This is initialised with K-means++ seeding (avoids degenerate initialisations where two components end up at the same location) and runs for 35 iterations with regularisation to prevent covariance matrices from collapsing.

The model is fitted after the first 15 observations, and refitted every 5 new observations thereafter. This means a target observed once per minute builds a full PoL model within 15 minutes. A target observed once per day (perhaps via check-in reports) builds a model within 15 days.

**Anomaly scoring:** The anomaly score for any observation (timestamp, position) is:

```
NLL = -log p_GMM([hour, x, y])           # negative log-likelihood of this point
x = (NLL - baseline_NLL) / |baseline_NLL|  # normalise against recent history
score = sigmoid(x) = 1 / (1 + exp(-x))   # map to [0, 1]
```

The baseline NLL is the mean NLL of the last 40 in-model observations. This makes the threshold adaptive: if a target normally visits locations with moderate PoL likelihood, the baseline calibrates to that level. If observations become sparser or more irregular (perhaps the target is being careful), the baseline adjusts accordingly.

Score near 0.5: consistent with routine. Score near 1.0: highly anomalous. Score exactly 0.5: PoL model not yet fitted (system doesn't know yet).

**Location prediction:** Given a future timestamp, the PoL model can predict where a target is likely to be. It does this by: (1) weighting each GMM component by its likelihood at that hour of day, (2) sampling component indices from those weights, (3) drawing position samples from the selected components' 2D position marginals, (4) returning the mean predicted position and a spread (standard deviation of distances from mean) as uncertainty. This prediction is used by the dormant reacquisition system and the third rendezvous warning method.

**Dormant reacquisition:** When a track's existence probability drops below the prune threshold but the track has a fitted PoL model, it enters dormant state rather than being deleted — stored for up to 40 scans. When new unassigned observations arrive, the system checks each dormant track's PoL prediction: if any observation falls within 3 standard deviations of where the dormant track is predicted to be at the current time, the track is reacquired with its full identity, PoL history, and threat history intact. This is operationally critical: a target who goes off-grid for 40 minutes and then reappears is immediately re-identified rather than treated as a new unknown.

---

## 1.7 Rendezvous Warning: Predicting Meetings Before They Happen

The rendezvous warning system answers what is often the single most operationally valuable question: "Are two of my targets about to meet?"

Most surveillance systems detect meetings after they happen — a distance threshold fires when two tracked entities are less than X metres apart. ARIA-INTEL warns 30 minutes in advance, with 100% detection across all validated scenarios and a mean lead time of 28.1 minutes. This is architecturally different and operationally transformative: 30 minutes is enough time to position surveillance, seek a warrant, or intercept the meeting.

The system runs three independent prediction methods in parallel on every confirmed track pair at every scan. It reports the method that produces the longest valid warning time within the horizon.

**Method 1 — Geometric Velocity Intercept:**

For each track, a velocity vector is estimated by least-squares linear regression on the last 8 position history points. This is cached: if a track's history hasn't changed, the velocity fit is reused across all pairs it appears in (reducing computation from 42 polyfit calls per scan to 7 for 7 tracks).

Given positions p_i, p_j and velocities v_i, v_j, the time to closest approach (CPA) is:

```
delta_v = v_i - v_j
delta_p = p_i - p_j
t_CPA = -dot(delta_p, delta_v) / dot(delta_v, delta_v)   [scans]
```

If t_CPA > 0 (converging), the CPA positions are computed and the CPA separation is checked against twice the meeting threshold. If they will actually be close enough to meet, the ETA is t_CPA × scan_dt seconds. Confidence = 1 - CPA_sep / threshold, clipped to [0.1, 1.0].

This was the dominant method in validation — responsible for 26 of 39 rendezvous warning events. It works best when targets are on steady headings.

**Method 2 — Separation Rate Extrapolation:**

Rather than working with velocity vectors directly, this method tracks the scalar separation between the pair over the last 8 scans and fits a linear trend. If the slope is negative (separating distance decreasing), the ETA is:

```
ETA_scans = (current_sep - meeting_threshold) / |slope|
```

Confidence is the R² of the linear fit. This method is robust when headings are noisy or when targets are approaching along curved routes, because it operates on the much smoother separation time series rather than raw positions. A target orbiting around another in a closing spiral — hard for Method 1 — is handled cleanly here.

**Method 3 — PoL Cross-Prediction:**

Once both tracks have fitted PoL models (minimum 15 observations each), this method queries each target's habitual routine independently to predict where they will be at future times t_now + k·dt for k = 1 to 20 horizon steps. If the predicted positions converge within the meeting threshold at any step, the time of minimum predicted separation is the warning.

This method is the only one that can warn about a scheduled meeting when neither target is yet moving toward the other. If two targets meet at a particular café every Tuesday at 11 AM — from pure habit — Method 3 will predict this meeting on Tuesday morning before either target has left home. Methods 1 and 2 cannot do this: they require the targets to already be approaching each other. Method 3 is throttled to run every 5 scans (PoL patterns change slowly) and uses minimal Monte Carlo samples for efficiency.

**The stacking logic:** The three methods have independent failure modes. Geometric intercept fails when headings are erratic or targets are manoeuvring. Separation rate fails when the closing rate is non-monotonic. PoL cross-prediction requires sufficient observation history. By reporting whichever method produces the earliest valid warning, the system achieves redundancy: if any one method can detect the meeting, the warning is issued. This stacking architecture is novel — no known operational system uses all three methods simultaneously.

Priority tiers for warnings: IMMEDIATE (<5 min), HIGH (<15 min), MEDIUM (<30 min), LOW (otherwise).

---

## 1.8 The Eight Tradecraft Detectors

ARIA-INTEL's tradecraft detector registry contains eight plugins, each implementing a BaseDetector interface with a single `detect(tracks, context)` method. They can be added, removed, or swapped at runtime without restarting the engine. All eight run on every scan.

**Detector 1 — Brush Pass (LegacyTradecraftDetector):**

When two confirmed tracks come within 60m of each other (configurable as `brush_pass_m`), a BRUSH_PASS event fires — but only on the first scan of contact. This prevents a meeting from generating hundreds of events; it fires once, on onset. After the tracks separate and re-approach, it fires again. Operationally, this catches hand-to-hand exchanges: documents, money, drugs, physical signals.

**Detector 2 — Surveillance Detection Route (LegacyTradecraftDetector):**

A person running a Surveillance Detection Route (SDR) deliberately takes a winding path to detect whether they are being followed. The detector computes the winding number of each track's last 12 position history points: the total angular sweep of the path around its centroid, divided by 2π. A winding number ≥ 0.65 (roughly two-thirds of a full loop) fires an SDR_PATTERN event.

The winding number is a concept from topology — it counts how many times a curve winds around a point. Its application here is novel: no published operational system is known to use topological winding analysis for tradecraft detection. The threshold of 0.65 was chosen to catch operationally realistic SDR loops (which typically cover a partial loop in any given observation window) while being robust to minor directional corrections. Known false-positive: circular patrol routes.

**Detector 3 — Dead Drop (LegacyTradecraftDetector):**

A dead drop is when one person leaves something at a location and another person retrieves it later, without the two ever meeting. The detector divides the area into 200×200m grid cells and tracks which tracks have visited each cell within a configurable time window (60 seconds to 30 minutes). A cell with two or more visitors — with visits spread across the window but no simultaneous presence (no two visitors within 30 seconds of each other) — fires a DEAD_DROP event at IMMEDIATE severity.

Throttled to every 3 scans for efficiency. Operationally, this catches: document dead drops, drug pick-ups, weapons caches, message depositories.

**Detector 4 — Parallel Route Surveillance (ParallelRouteSurveillanceDetector):**

Detects mobile surveillance (a "tail") where one track follows another along a parallel route. For each track pair, the detector checks: velocity heading similarity (cosine ≥ 0.97, meaning headings within ~14°), lateral offset (perpendicular distance 0–80m), and longitudinal separation (within 3× the lateral offset). All three conditions must hold for 6 consecutive scans before firing PARALLEL_SURVEILLANCE at HIGH severity.

The 6-scan requirement prevents false alarms from momentary co-directional travel. The lateral offset window of 0–80m (rather than 0–0) is realistic: a professional tail does not shadow directly behind the subject; they maintain an offset to avoid recognition. Output includes `subject` and `surveillant` track IDs, lateral distance, heading cosine, and duration.

**Detector 5 — Mode Transition (ModeTransitionDetector):**

Detects vehicle-to-foot or foot-to-vehicle handoffs. A vehicle-class track (dominant MOU model with σ > 3.0 m/s, indicating high-speed motion) that decelerates below 2.5 m/s is recorded as a vehicle stop event. If within 2 scans, a new foot-class track appears within 50m of the stop location, a MODE_TRANSITION event fires at HIGH severity with both track IDs, stop position, and delay in scans.

Operationally: detects driver handoffs, dismounts from cover vehicles, foot-follows beginning from a parked car, counter-surveillance change of transportation mode, and swap of a physical asset between vehicles.

**Detector 6 — Loiter Anomaly (LoiterAnomalyDetector):**

Detects when a track dwells at a location for significantly longer than its normal behaviour would predict, and that location is anomalous according to the track's PoL model. The dwell radius is 4× position noise (typically 20–30m). When dwell duration exceeds `loiter_mult` (default 3×) times the track's PoL baseline dwell at a location with anomaly score > 0.65, LOITER_ANOMALY fires.

The PoL baseline dwell is estimated as the median inter-observation gap from the track's history — a rough measure of how long the target typically spends in any location. Without a PoL model (fewer than 15 observations), the detector falls back to an absolute duration threshold (4× the minimum loiter time), at LOW severity.

Operationally: detects static observation posts, pre-operation reconnaissance, patient waiting for a target, and target assessment behaviour.

**Detector 7 — Cover Stop (CoverStopDetector):**

The cover stop is a sophisticated intelligence tradecraft technique: the operative regularly visits an innocuous, routine-seeming location (a café, a park bench, a shop) near a high-value target (an embassy, a government building, an executive's home). The location appears legitimate because it is part of the operative's routine.

The detector identifies this by cross-correlating the PoL model with high-value location (HVL) proximity. A visit to a PoL-consistent location (anomaly score < 0.80, meaning it is within the track's normal routine) that is within `cover_stop_hvl_m` (default 800m) of an HVL, and that has a history of similar visits, fires COVER_STOP. Severity escalates with repeat count: HIGH on second visit, IMMEDIATE on third.

This detector requires HVLs to be configured — the operator provides a list of sensitive locations. In LE deployment: police stations, courts, protected witnesses, drug markets, financial institutions, government buildings.

**Detector 8 — Chokepoint Surveillance (ChokepointSurveillanceDetector):**

Detects a track repeatedly passing through the same narrow location in both directions — indicating the track is monitoring who passes through a chokepoint (a doorway, an alley entrance, a bridge, a vehicle checkpoint, a market entrance).

The detector tracks visits to grid cells (configurable cell size, default = `chokepoint_m` = 40m). For each cell with ≥ `chokepoint_n` (default 3) visits from a single track, it computes heading circular variance across visits. Low variance means always going the same direction (just a regular route). High circular variance (≥ 0.3) with visits spread across multiple sessions (gaps > 10 scans between visits) fires CHOKEPOINT_SURVEILLANCE at HIGH severity.

Operationally: countersurveillance at a meeting location, hostile intelligence collection at an embassy entrance, criminal monitoring of a drug territory boundary, surveillance of law enforcement facilities.

---

## 1.9 Network Analysis: Mapping Relationships

The DynamicNetworkAnalyser builds a relationship graph across the entire session. Every scan, pairs of tracks within `coloc_dist_m` (default 350m) accumulate edge weight proportional to their proximity (1 - distance/coloc_dist_m). This weighted adjacency matrix grows throughout the session, reflecting the cumulative history of who has been near whom.

**Betweenness centrality:** Computed using Brandes' algorithm (O(N×E), exact, not approximated) from the binary adjacency matrix. Betweenness centrality measures how often a node appears on the shortest path between other node pairs. In intelligence terms, high betweenness = this person is the hub through which information flows. The handler in a spy network, the organiser in a criminal gang, the coordinator in a trafficking ring — all will have high betweenness centrality because they connect otherwise separate network members.

**Clusters:** Groups of tracks that have been co-located with each other are identified as clusters. Each cluster output includes: member track IDs, the hub track (highest weighted degree — the most centrally connected member within the cluster), betweenness centrality scores per member, and a `recurring` flag indicating whether any cluster members have been co-located in previous scans (a habitual association vs a one-off encounter).

**Network roles (NetworkRoleInferenceDetector):** Using relative percentile ranks of speed and contact count within the current track set:

- COURIER: top tercile of speed AND contact count. Fast movers who see many people. Drug runners, message carriers, liaison agents.
- HANDLER: bottom tercile of speed, middle-to-high contact count. Slow, stable, seen by many. The command node.
- ASSET: high PoL anomaly, low contact count. Irregular activity, meets only 1–2 tracks. The active operative or primary target.
- UNKNOWN: insufficient data or doesn't fit the taxonomy.

Role stability requires 3 consecutive scans of the same classification before the role is reported — preventing noise-driven flickering. The percentile-based design means the classifier works the same whether the network has 3 members or 80.

---

## 1.10 Threat Scoring: From Evidence to Priority

Every confirmed track receives a threat score each scan via the Beta-Monte Carlo scorer. This turns the collection of quantitative evidence about a track into a single calibrated priority.

**Why Beta distributions?** Each piece of evidence is a proportion — a number between 0 and 1. The existence probability r is directly a probability. The PoL anomaly score, detection density, HVL proximity are all bounded in [0, 1]. The natural Bayesian distribution for a proportion is the Beta distribution, parameterised by positive evidence count (alpha) and negative evidence count (beta). When alpha is large relative to beta, the Beta distribution is concentrated near 1 (high threat evidence). When beta is large, it is concentrated near 0. The Beta-Binomial conjugate pair is one of the oldest and best-validated tools in Bayesian statistics.

ARIA-INTEL fits each of 8 dimensions to a Beta distribution:

| Dimension | What it measures | Weight |
|---|---|---|
| Existence | How certain we are this is a real person (PMBM r) | 0.23 |
| PoL Anomaly | How far from their normal routine they are | 0.18 |
| Detection Density | What fraction of scans they've been observed | 0.13 |
| HVL Proximity | How close they are to a high-value location | 0.13 |
| Threat EMA | Exponential moving average of past threat scores | 0.10 |
| Motion Score | Speed normalised to 30 m/s | 0.08 |
| Persistence | How many consecutive high-score scans | 0.08 |
| Poss Match | Inverse of Possibility-PMBM mismatch (anomaly penalty) | 0.07 |

Then 250 Monte Carlo samples are drawn from the weighted Beta mixture, giving a full posterior distribution over threat scores. The output includes mean, standard deviation, P90 (90th percentile), and P95. The mean drives priority tier assignment; the P95 provides an upper-confidence bound for conservative analyst alerting.

**Priority tiers:**
- IMMEDIATE (≥ 0.82): Requires immediate analyst action.
- HIGH (≥ 0.62): Priority collection and analysis.
- MEDIUM (≥ 0.42): Active monitoring.
- LOW (≥ 0.22): Background tracking.
- MONITOR (< 0.22): Passive — observed but not prioritised.

The Anomaly Escalator adds three secondary alert types: SPIKE (single-scan score > 0.72), ESCALATING (monotonic increase over 5 consecutive scans), and COUNTER_SURVEILLANCE — the last being particularly powerful: it fires when a track's score drops below 0.3 after being above 0.5, then spikes above 0.6 again, the sawtooth pattern characteristic of a target deliberately varying their behaviour to flush surveillance.

---

## 1.11 Supporting Systems

**Source Credibility Tracker:** Every observation carries a source_id. The system tracks per-source reliability using an exponential moving average (decay = 0.98). Each time a source's observation is assigned to a track, the observation log-likelihood under the particle filter is compared to a threshold. Sources whose observations consistently fail this test — i.e., their reported positions are systematically inconsistent with the physics of motion — have their weights reduced. This is automatic deception detection at the ingestion layer: a compromised informant or a sensor being fed false data will self-identify by producing observations that contradict the filter's predictions.

**Credibility Fuser:** Combines evidence across multiple simultaneous observations of the same target using Dempster-Shafer Theory (DST). Rather than simple averaging, DST explicitly represents the conflict between sources. The conflict coefficient K measures how much the observations contradict each other. K near 1.0 means the sources are telling contradictory stories — a potential indicator of disinformation, or of two different real targets being confused. Reliability priors: GEOINT 0.90, SIGINT 0.78, COMMS 0.70, HUMINT 0.62, OSINT 0.48.

**Sensor Scheduler:** Produces a ranked list of (track, recommended_modality) pairs ordered by expected information gain, approximated as modality_weight × track.r / position_uncertainty. This recommends the most reliable sensor modality for the track with the highest combination of confidence and positional uncertainty. Operationally: if a SIGINT track has high uncertainty because it has only been observed via phone location (COMMS modality, reliability 0.70), the scheduler recommends deploying a GEOINT asset (reliability 0.90) to reduce uncertainty before the next expected meeting.

**Route Predictor:** For IMMEDIATE and HIGH priority tracks, generates an 8-step trajectory forecast. Propagates the particle ensemble forward using the MOU model mixture. At each step, a PoL blend (increasing from 0 to 50% of influence over the horizon) nudges the ensemble toward the PoL-predicted location for that future time. This captures both short-term trajectory inertia and long-term behavioural tendencies. Output: list of {step, time_ahead_s, position, uncertainty_m, confidence} for each step.

**Forward-Backward Smoother:** Maintains a 6-scan lag history of particle ensembles per track. Computes a Gaussian-kernel-weighted average of historical particle means, with the kernel centred on the most recent scan. This smooths the displayed track history, reducing the visual noise that makes it harder for operators to identify trajectory patterns.

---

## 1.12 Domain Profiles: One Engine for Every Domain

Every number in ARIA-INTEL lives in a DomainProfile dataclass. There is no domain-specific conditional code anywhere in the engine. Swap the DomainProfile, and the entire system retunes: scan intervals, position noise, motion models, meeting thresholds, warning horizons, colocation distances, all detection thresholds.

The four built-in profiles:

**UrbanHUMINT:** 60-second scans. Meeting threshold 150m. Foot/vehicle/stationary/fast motion models. 30-minute rendezvous warning. Designed for tracking human subjects at pedestrian and vehicle speeds in an urban environment.

**Maritime:** 3600-second (1-hour) scans. Meeting threshold 2,000m. Drifting/transiting/anchored/fast_craft motion models. 2-hour rendezvous warning. Designed for vessel tracking with satellite or AIS refresh rates.

**Airspace:** 5-second scans. Meeting threshold 1,000m. Hovering/fixed_wing/gliding/fast_jet motion models. 10-minute rendezvous warning. Designed for radar-rate air track processing.

**VehicleConvoy:** 10-second scans. Meeting threshold 30m. Stopped/slow_roll/highway/sprint motion models. 5-minute rendezvous warning. Designed for GPS-quality vehicle fleet tracking.

New domain profiles for law enforcement use cases (CityCamera, BorderPatrol, MaritimeLE, etc.) are specified in Parts 2–5 of this brief.

---

## 1.13 Performance: What the Numbers Mean Operationally

28 ms median scan latency on a single CPU core means the engine can process 20 complete intelligence updates per second with full 8-detector pipeline active, on hardware as modest as a Raspberry Pi 4 or a laptop. The mean is 51 ms (PoL cross-predict scans cost more) and P95 is 210 ms. At a 60-second scan rate (UrbanHUMINT), even the P95 latency represents a vanishingly small fraction of the scan interval.

100% rendezvous detection at mean lead time 28.1 minutes across 20 independent scenarios is not a cherry-picked result — each scenario is a different random seed generating different target trajectories, different clutter levels, different modalities. The consistency demonstrates that the three-method stacked architecture is genuinely robust.

100% detection rate at P_D = 0.40 means the system confirms all targets even when only 4 in 10 observations are correct. This is critical for LE deployment: camera occlusion, sensor outages, informant unreliability — all translate to a reduced effective detection probability. The system handles this gracefully.

False alarm rate of 0.098/scan at clutter density 40/scan means with 40 false observations per scan (extremely noisy sensor), fewer than 0.1 false tracks per scan make it to confirmation. For context: if you ran the system for 1,000 scans (about 17 hours at 60-second scan rate), you would expect approximately 98 false track confirmations — against an average of 7 true targets per scan. The false alarm rate is manageable and predictable.

The competing graph-based multi-target tracking implementations (nearest-neighbour association, JPDA, Hungarian algorithm) do not achieve this level of performance under high clutter and low detection probability. Their O(N²·E) assignment costs scale catastrophically with track count; ARIA-INTEL's O(N·K·P) Gibbs-sampled PMBM scales linearly. At 50 simultaneous confirmed tracks, a graph tracker requires ~2,500× more association computation than the PMBM approach. This is why the system runs single-core while competitors require GPU acceleration or distributed compute.

---

*Part 1 complete. Part 2 covers the camera front-end and city-scale deployment.*
# ARIA-INTEL Law Enforcement & Intelligence Technical Brief
## Part 2 — The Camera Problem: Bridging Visual Surveillance to the Engine

*Author: Odin Loch*

---

## 2.1 Why Cameras Are Different

ARIA-INTEL was designed around a generic Observation schema: a timestamped position with a modality label and a confidence score. Every sensor — a phone location ping, an informant sighting, a radar return — fits this schema naturally, because each produces a position estimate with some uncertainty.

Cameras do not produce position estimates. They produce images.

A camera does not say "subject X is at coordinates [1200, 800] at time t." It produces a 3840×2160 pixel frame containing somewhere between zero and fifty people, none of whom are labelled. To turn that into ARIA-INTEL observations, a pipeline must:

1. Detect all people in the frame (object detection)
2. Assign each person a persistent identity — the same identity across frames and across different cameras (re-identification, or Re-ID)
3. Convert pixel coordinates to real-world coordinates (homographic projection)
4. Package each labelled, geolocated detection as an Observation and pass it to the engine

Steps 1 and 3 are well-solved problems. Step 2 — the Re-ID problem — is the hard one, and it is what separates a camera-based deployment of ARIA-INTEL from all its other sensor modes.

---

## 2.2 The Re-ID Pipeline: Creating Persistent Identities from Images

Re-identification is the task of matching a person appearing in one camera (or one frame) to the same person appearing in another camera (or a later frame). It is hard for several reasons:

- The same person looks different from different angles, at different distances, under different lighting, wearing different clothes across days.
- Different people in similar clothing are easily confused by appearance alone.
- Occlusions, crowd overlap, and reflective surfaces create partial and misleading observations.
- A city camera network may have tens of thousands of cameras, requiring real-time matching across a vast gallery of known appearances.

The engineering solution is a Re-ID model: a neural network trained to produce a compact embedding vector (typically 512 or 2048 dimensions) from a cropped person image, such that embeddings of the same person are close together in embedding space regardless of camera, angle, or lighting, and embeddings of different people are far apart.

**Architecture of the Re-ID bridge:**

```
Camera frame → Object detector (YOLOv8/RTMDet) 
             → Person crop extractor
             → Re-ID model (OSNet / BoT / LightMBN / custom)
             → Embedding vector (512-d float32)
             → Identity matching (cosine similarity against gallery)
             → Persistent person ID assignment
             → Homographic projection (pixel → real-world metres)
             → Observation(obs_id, timestamp, position, "GEOINT", confidence, camera_id)
             → ARIA-INTEL engine.ingest()
```

**Object detection:** Standard single-stage detectors (YOLOv8n or RTMDet-tiny) run at 20–60 FPS on GPU, or 5–15 FPS on CPU-only hardware. For city camera deployments, GPU hardware at each camera node is unrealistic; the typical approach is to stream compressed frames to regional processing nodes, each handling 20–50 cameras, at reduced frame rates (1–2 FPS is sufficient for the 60-second ARIA-INTEL scan rate). Person crops are extracted from detections with confidence > 0.5.

**Re-ID model selection:** Several off-the-shelf options exist. For deployment at scale:
- **OSNet (Omni-Scale Network, 2019):** Lightweight, fast, excellent cross-domain performance. Suitable for real-time processing.
- **BoT (Bag of Tricks, 2019):** Higher accuracy, more compute. Suitable for overnight batch processing.
- **LightMBN (2021):** Designed explicitly for low-resolution surveillance footage. Useful for older camera infrastructure.
- A custom model fine-tuned on local camera network footage will outperform any off-the-shelf model, because appearance distributions are location-specific (clothing styles, ethnicity demographics, lighting conditions).

**Gallery management:** The Re-ID gallery stores appearance embeddings for known subjects (the watchlist) and for unknown persons encountered during a session. For watchlist tracking: each watchlisted person's gallery entry contains multiple embeddings from reference images (multiple angles, lighting conditions), and a match is declared when cosine similarity between a detected embedding and any gallery embedding exceeds a threshold (typically 0.85–0.92 depending on false-alarm tolerance).

For mass tracking (every person in the camera network): gallery entries are created for all detected persons, and the matching threshold can be lower because temporal and spatial continuity constraints provide additional disambiguation power — the same person cannot appear at two locations 5km apart in 30 seconds.

**Confidence mapping:** Re-ID match confidence (cosine similarity score, normalised to [0.5, 1.0]) maps directly to the Observation.confidence field. ARIA-INTEL's source credibility tracker will then automatically learn that high-similarity matches from camera X are reliable (camera is in good condition, well-lit, good angle) and that low-similarity matches from camera Y are unreliable (camera is at low resolution or in shadow), adjusting weights accordingly.

---

## 2.3 The Camera Topology Graph

A city camera network is not a continuous 2D space — it is a graph. Camera A at intersection 1 has a field of view covering roughly 20×15 metres. Camera B at intersection 2, 200 metres away, has no visual coverage of the street between them. A person who disappears from camera A's view and appears in camera B's view has undergone a "gap" — a period of zero observations — during their transit.

This is structurally identical to ARIA-INTEL's standard dormancy/reacquisition problem. The dormancy system handles it without modification:

- Person exits camera A's field of view → track receives missed-scan updates, existence probability r decays
- Track enters dormancy if r drops below R_PRUNE but PoL model exists
- Person appears in camera B → unassigned observation spawns a candidate track, which is checked against dormant track PoL predictions
- If the new observation falls within the predicted location for the dormant track, the track is reacquired

The PoL prediction here is using the spatio-temporal model to predict "this person left camera A heading east 4 minutes ago; they should appear at camera B, C, or D within the next 2–6 minutes depending on whether they walked or took a vehicle." The model learns these transit patterns from historical data.

**Travel time priors:** For cameras with known geometry (which is the case for any properly surveyed deployment), the dormant timeout should be set based on the maximum credible transit time between the farthest camera pair in the network, divided by the scan rate. For urban camera networks with inter-camera spacing of 100–500m, a dormant timeout of 40 scans at 60-second scan rate (40 minutes) is conservative and appropriate.

**New domain profile — CityCamera:**

```python
CityCamera = DomainProfile(
    name             = "CityCamera",
    scan_dt_s        = 60.0,          # 1-minute aggregation window
    pos_noise_m      = 8.0,           # homographic projection error
    p_detection      = 0.75,          # camera occlusion, Re-ID failures
    rv_threshold_m   = 100.0,         # meeting distance in pedestrian context
    rv_warning_horizon_s = 1800.0,    # 30-minute warning
    brush_pass_m     = 50.0,
    coloc_dist_m     = 200.0,         # same-block co-location
    hvl_radius_m     = 400.0,
    dormant_timeout  = 40,
    mou_models = {
        "pedestrian":  {"theta": 0.30, "sigma": 1.5},  # walking
        "transit":     {"theta": 0.08, "sigma": 6.0},  # on public transport
        "stationary":  {"theta": 2.50, "sigma": 0.3},  # standing, waiting
        "cycling":     {"theta": 0.15, "sigma": 3.5},  # cycling speed
    },
)
```

---

## 2.4 Mass Surveillance Architecture (City-Scale)

The largest-scale camera-based surveillance deployments in the world — China's Skynet and Sharp Eyes programs, Singapore's SafeCity initiative, and large Western city deployments — share a common architecture: a hierarchy of processing nodes feeding a central intelligence layer.

At this scale, ARIA-INTEL does not track every person in the city. It tracks every person against a probabilistic model of expected behaviour. The distinction matters: in the mass surveillance model, ARIA-INTEL maintains a PoL model for every tracked individual, and the anomaly score is what drives escalation to human analyst attention.

**Tier 1 — Camera nodes:** Each camera or camera cluster runs a lightweight Re-ID front-end (object detection + embedding extraction). Outputs: timestamped (person_id, position, confidence) tuples. No ARIA-INTEL here — just fast, lightweight detection.

**Tier 2 — Zone processors:** Each city zone (covering, say, 5×5km and 200–500 cameras) runs a dedicated ARIA-INTEL engine instance. All observations from cameras in that zone flow into one engine. The zone engine maintains tracks for all persons detected in the zone, running the full PMBM/PoL/tradecraft/rendezvous pipeline.

**Tier 3 — City aggregator:** A federation node ingests confirmed track outputs from all zone processors and maintains cross-zone track continuity. When a person moves from zone A to zone B, zone A's engine allows the track to enter dormancy; zone B's engine picks up the reacquisition. The aggregator reconciles track IDs across zones. This is architecturally similar to the distributed military fusion architecture in ARIA-INTEL-MIL, using track-level (not observation-level) data sharing between nodes.

**Tier 4 — Analyst interface:** The central intelligence layer receives the aggregated confirmed tracks, rendezvous warnings, tradecraft events, and threat scores from all zone processors. Analysts receive only the IMMEDIATE and HIGH priority items, plus any events matching their active watchlists. The full track database is available for historical query.

**Population-scale PoL:** At city scale, every person with enough observation history has a PoL model. The city as a whole has a de facto "normal" — the aggregate of all individual routines. A person breaking their own routine is detectable via their individual anomaly score. A large-scale anomaly (many tracks simultaneously breaking routine, e.g. civil unrest) appears as a cluster of high anomaly scores in a geographic area — detectable as a geographic anomaly aggregate even without individual watchlisting.

---

## 2.5 Watchlist Tracking Architecture (London NPPV Model)

The alternative to mass surveillance is targeted watchlist tracking: a fixed set of persons of interest (POIs), identified by Re-ID gallery entries, tracked throughout the camera network. This is the model used by London's Metropolitan Police and National Police Video (NPV) deployments, and it requires far less compute and raises fewer legal concerns than blanket mass tracking.

**The watchlist:** A database of person embeddings for each POI, derived from booking photos, operational photographs, CCTV stills from previous incidents, or reference images from open sources. Each POI entry may contain dozens of embeddings (different clothing, lighting conditions, angles) to maximise Re-ID recall.

**The matching pipeline:** On each camera frame, detected person embeddings are compared against all watchlist entries using approximate nearest-neighbour search (FAISS or similar). Matches above the recognition threshold generate Observations with the POI's persistent ID. Non-matching detections are discarded (in the pure watchlist model) or tracked as unknowns for network mapping purposes.

**ARIA-INTEL configuration for watchlist tracking:** Relatively few confirmed tracks (one per POI, perhaps 10–100 across an active investigation). The engine can be configured to run at very high fidelity — large particle count, frequent PoL refit, aggressive rendezvous warning — because the computational budget is not constrained by track count.

**Key operational output — Cover Stop detection:** For watchlist tracking against high-value locations (embassy row, a crown court, a protected witness's residence, a financial institution under fraud investigation), the CoverStopDetector is the primary operational driver. Each POI's visits near the HVLs are accumulated; the repeated-routine-visit pattern near an HVL fires at IMMEDIATE severity after two visits and IMMEDIATE on the third. This automates the analyst task of correlating movement patterns with sensitive locations.

---

## 2.6 Privacy and Legal Architecture

Any LE/intelligence deployment of a camera-tracking system operates under legal constraints that vary by jurisdiction but share common themes: lawful authority for collection, purpose limitation, data retention limits, oversight and audit, and proportionality. ARIA-INTEL's architecture must incorporate these constraints natively rather than as an afterthought.

**Audit trail:** Every confirmed track, every event generated, every analyst action, and every source observation used must be logged to an immutable audit record. This is the evidentiary foundation for any prosecution and the accountability record for oversight review. The audit record should include: track ID, source observations used to confirm the track, all events generated, timestamps, and (in watchlist mode) the identity matching record showing which gallery entry matched.

**Warrant gating for IMMEDIATE actions:** In jurisdictions requiring judicial authority for interception or physical surveillance, ARIA-INTEL outputs at IMMEDIATE priority should be routed through a warrant approval workflow before triggering operational responses. The Cover Stop detector and Rendezvous Warning system are specifically designed to provide sufficient lead time (28+ minutes for rendezvous) to seek authorisation before deployment.

**Data minimisation:** In mass surveillance mode, track data for non-watchlisted persons should be held only for the duration needed to build PoL models and detect anomalies — not retained indefinitely. Configurable retention windows per track (e.g., dormant tracks purged after 24 hours for non-watchlisted persons vs 90 days for POIs) should be implemented as a retention policy layer above the engine.

**Classification tiers:** Different outputs carry different legal authority requirements:
- Observation data (raw camera tracks): standard CCTV retention policies
- Behavioural analysis (PoL, anomaly scores): higher authority — analytical product
- Watchlist match alerts: warrantable output requiring legal authority record
- IMMEDIATE tier events triggering physical surveillance: operational authorisation required

**Misidentification risk:** Re-ID false positives (system believes it sees POI X, but it is actually an innocent bystander with similar appearance) are a material legal risk. The Possibility-PMBM mismatch diagnostic helps here: a track generated from a Re-ID match but whose subsequent behaviour is inconsistent with the known POI's PoL will generate a mismatch alarm, flagging the possible misidentification to an analyst before any action is taken.

---

*Part 2 complete. Part 3 covers specific law enforcement use cases.*
# ARIA-INTEL Law Enforcement & Intelligence Technical Brief
## Part 3 — Law Enforcement Use Cases

*Author: Odin Loch*

---

## 3.1 Counter-Terrorism Surveillance

### The Operational Problem

CT surveillance involves the hardest version of the tracking problem: subjects are often trained in counter-surveillance, the consequences of both false positives and missed detections are severe, observations may come from a mix of technical and human sources across weeks or months, and the critical action — the attack or the facilitation meeting — may be preceded by a long and varied preparatory phase during which the operational indicators are subtle.

### What ARIA-INTEL Provides Directly

**Cover Stop Detection as the primary CT alarm:** CT subjects routinely conduct pre-operational reconnaissance of their intended targets. A subject who repeatedly visits a location near a crowded venue, a government building, or a transit hub — locations consistent with their normal pattern of life, providing operational cover — will be detected by the CoverStopDetector. The detector specifically catches this scenario: routine-seeming visits (PoL anomaly < 0.80) to locations within 800m of an HVL, accumulating over multiple visits. After the second visit to the same cover stop near the same HVL, a HIGH event is raised. After the third, IMMEDIATE.

Configuring the HVL list for CT operations: major transportation hubs, crowded event venues, government buildings, diplomatic facilities, military installations, utility infrastructure, financial institutions. The CoverStopDetector will then automatically flag any watchlisted subject who visits locations within the configured radius of these sites more than once.

**Rendezvous Warning for cell activity:** CT cells typically have a handler (recruiter/controller), couriers (facilitators), and active operatives (the subjects directly involved in planning or execution). The NetworkRoleInferenceDetector will classify these roles as HANDLER, COURIER, and ASSET respectively based on their movement patterns and contact frequency. The rendezvous warning system provides 30-minute advance warning of cell member meetings, enabling surveillance assets to be pre-positioned.

**SDR Pattern detection for surveillance-aware subjects:** Trained CT operatives conduct surveillance detection routes before sensitive meetings. The SDR winding number detector will flag any track in the watchlist executing this pattern, providing both an indication that the subject is operationally aware and a signal that a sensitive meeting may be imminent (SDRs are typically conducted immediately before meetings with handlers or cell members).

**Dead Drop detection:** CT networks increasingly use dead drops (physical or digital dead letter boxes) to avoid direct meetings between handler and operative. The dead drop detector flags sequential visits to the same small geographic area (200×200m cell) with no simultaneous presence — exactly the dead drop operational pattern.

**PoL anomaly as pre-operational indicator:** A CT subject in the preparation phase will break their normal routine in ways that are individually innocuous but collectively anomalous. They may conduct reconnaissance at unusual hours, visit unfamiliar parts of the city, make unusual transit patterns. The PoL anomaly score will rise as these breaks from routine accumulate, even before any specific event is detected. An ESCALATING alert (monotonic threat score increase over 5 scans) is a strong indicator that a subject's behaviour is changing in a systematic way.

### New Detector: Pre-Attack Pattern Detector

For CT-specific deployment, a new detector builds on the existing LoiterAnomalyDetector and CoverStopDetector:

```
PreAttackPatternDetector:
  Combines:
    - Consecutive cover stops near same HVL (CoverStop history)
    - Increasing visit frequency near HVL (acceleration in cover stop rate)
    - SDR execution within 48 hours of cover stop visit
    - Rendezvous with HANDLER-role track within 72 hours of cover stop

  Fires: ATTACK_PREPARATION_PATTERN at IMMEDIATE severity
  
  Output includes: HVL targeted, timeline of preparatory indicators,
                   network structure of associated tracks
```

### Domain Profile for CT Operations

```python
CTSurveillance = DomainProfile(
    name                  = "CTSurveillance",
    scan_dt_s             = 60.0,
    pos_noise_m           = 8.0,
    p_detection           = 0.70,          # covert collection has gaps
    rv_warning_horizon_s  = 3600.0,        # 1-hour warning for handler meetings
    rv_threshold_m        = 100.0,
    brush_pass_m          = 40.0,          # tight threshold for CT contacts
    cover_stop_hvl_m      = 1000.0,        # wider HVL radius for venue recon
    coloc_dist_m          = 150.0,
    hvl_radius_m          = 800.0,
    dormant_timeout       = 80,            # long dormancy — subjects go quiet
    loiter_mult           = 2.0,           # aggressive loiter detection
)
```

---

## 3.2 Organised Crime and Gang Network Mapping

### The Operational Problem

Organised crime investigations require understanding the network structure before targeting individuals. Arresting the street-level courier does nothing if the handler recruits a replacement within days. The investigative priority is identifying the command structure: who gives orders, who carries information, who handles money, how the network is organised. Traditional surveillance achieves this slowly, through manual correlation of observations over weeks. ARIA-INTEL makes it automatic.

### Network Topology as Automatic Intelligence Product

**Betweenness centrality as command node identifier:** The gang's organisational hierarchy maps directly onto betweenness centrality. The gang leader — or the cell commander, or the district-level manager in a large criminal organisation — occupies the highest betweenness position in the network: all information and orders flow through them. The DynamicNetworkAnalyser computes this automatically from the co-location history. After a sufficient observation period (typically 2–3 weeks of daily contact data), the command structure will be visible in the centrality scores without a single analyst manually connecting any dots.

**HANDLER/COURIER/ASSET roles in gang context:** The gang command structure maps to ARIA-INTEL's network roles:
- HANDLER = gang lieutenant, cell commander, wholesaler. Stable location (low mobility), receives multiple people, does not travel to initiate contacts.
- COURIER = runner, dealer, delivery driver. High mobility, many unique contacts, routine routes (low PoL anomaly once the route is established).
- ASSET = active operative, enforcer, lookout. Irregular activity (high PoL anomaly), meets only their specific handler.

**Cluster identification for cell structure:** Distinct criminal cells within a larger organisation will form distinct clusters in the co-location graph — members of cell A have high edge weights among themselves, members of cell B likewise, and the inter-cell connections (if any) reveal the liaison structure. The cluster's recurring flag distinguishes habitual cell associations from one-off contacts.

### Specific Detector Applications

**Drug distribution networks:** A drug wholesale distribution network typically has: the wholesaler (HANDLER, stationary, receives couriers), street-level dealers (COURIER, mobile, many contacts), and customers (not usually tracked, but their presence inflates the dealer's contact count). The Mode Transition detector catches the classic sequence: a vehicle arrives at a location (the car is the "bank"), multiple persons approach and leave on foot (deals), the vehicle departs. Multiple MODE_TRANSITION events from the same location at regular intervals identify the dealing location and the vehicle used.

**Gang territory monitoring:** The ChokepointSurveillanceDetector identifies gang lookouts monitoring territory entry points — alley entrances, the entrance to a housing estate, a bridge over a road. A lookout who makes multiple passes through the same chokepoint in both directions, spread across multiple sessions, is flagged automatically.

**Brush pass for drug hand-to-hand deals:** The BRUSH_PASS detector fires when two tracks come within 60m — but more importantly, it fires only on the first scan of contact. A high-volume street dealer may have 20–30 brush passes per hour. The event log provides a precise record of every contact: when, where, with which other track (if on the watchlist), at what time. This is both an intelligence product and an evidentiary record.

**Dead drop drug pick-ups:** Some drug distribution models use dead drops: drugs left at a location, payment left at the same location by the customer. The dead drop detector fires on sequential visits by different tracks to a small geographic cell with no simultaneous presence. Combined with the watchlist, this identifies the stash location and the persons using it.

### Network Disruption Intelligence

Once the network structure is mapped via betweenness centrality and cluster analysis, ARIA-INTEL provides natural targeting prioritisation:

- **Maximum network disruption:** Target the highest-betweenness track. Removing this node breaks all connections between clusters.
- **Cell isolation:** Target all cross-cluster liaison tracks. This fragments the network into cells that cannot coordinate.
- **Supply chain interdiction:** Target COURIER-role tracks who connect to known supply locations (HVLs configured as known warehousing sites, stash houses).

The route predictor for HIGH/IMMEDIATE tracks provides the 8-step trajectory forecast needed to pre-position arrest teams.

---

## 3.3 Drug Trafficking Interdiction

### The Operational Problem

Drug trafficking surveillance combines the network mapping challenge (identifying the supply chain structure) with a time-critical interdiction challenge (intercepting consignments in transit). The operational window for interdiction is narrow: a consignment that reaches a stash house is difficult to link to the supply chain; a consignment in transit can be seized and its carrier tracked back to both the supplier and the distribution network.

### Supply Chain Mapping with ARIA-INTEL

**Transit pattern recognition:** Drug couriers (runners carrying consignments) exhibit a characteristic MOU model signature: high-speed directed motion (vehicle or fast model dominant) for the transit phase, brief stops at stash locations (stationary model), and then rapid departure. The Mode Transition detector catches every vehicle stop with a foot-track appearing nearby — each of which is a potential transfer point.

**Route PoL for courier prediction:** A courier who makes the same run twice a week will develop a PoL model reflecting their route. The Route Predictor will project their 8-step trajectory, enabling intercept team positioning. The PoL cross-prediction rendezvous method will warn of the courier's scheduled arrival at a distribution point based on their habitual timing, even before they depart — purely from historical routine.

**Vehicle-to-vehicle transfer detection:** The Mode Transition detector's vehicle stop logic captures car-to-car transfers when a vehicle-class track stops and another vehicle-class track (rather than a foot-class track) appears nearby. This requires adding a vehicle-to-vehicle transfer event type:

```python
# New event in ModeTransitionDetector:
# VEHICLE_TO_VEHICLE_TRANSFER: vehicle stops, another vehicle appears within 
# mode_trans_m within mode_trans_scans, both vehicle-class dominant models.
```

**Stash house identification:** The LoiterAnomalyDetector identifies stash houses as anomalous dwell locations. A courier who regularly stops at a particular location outside their PoL routine, for durations inconsistent with their usual behaviour, will generate a LOITER_ANOMALY event. After multiple occurrences, the cover stop detector's logic reinforces this: the same anomalous location is visited repeatedly, clustering near a point that is not in the PoL model (i.e., it is a dedicated operational location, not a personal one).

### New Detector: Drug Corridor Monitor

```
DrugCorridorMonitor(BaseDetector):
  Maintains a list of known transit corridors (road segments or routes).
  Detects tracks exhibiting directed high-speed movement along a corridor,
  consistent with courier behaviour:
    - Vehicle-class dominant model
    - Speed > threshold for corridor type
    - Heading within 20 degrees of corridor bearing
    - No significant deviation (a courier does not sightsee)
    - PoL model showing prior transits of same corridor (repeat trips)
  
  Fires: TRANSIT_CORRIDOR_DETECTED at MEDIUM severity (first time)
         REPEAT_COURIER_TRANSIT at HIGH severity (subsequent detections)
  
  Output: corridor ID, track ID, estimated delivery ETA based on speed,
          known terminus locations from PoL
```

### VehicleConvoy Profile for Vehicle Fleet Surveillance

For vehicle-centric drug trafficking surveillance, the existing VehicleConvoy DomainProfile is appropriate with minor modifications:

```python
DrugTrafficking = DomainProfile(
    name                 = "DrugTrafficking",
    scan_dt_s            = 30.0,            # GPS tracker update rate
    pos_noise_m          = 5.0,             # GPS quality
    p_detection          = 0.92,
    rv_threshold_m       = 50.0,            # vehicle-to-vehicle meeting
    rv_warning_horizon_s = 900.0,           # 15-minute warning for intercept
    brush_pass_m         = 30.0,
    coloc_dist_m         = 100.0,
    hvl_radius_m         = 200.0,           # known stash houses as HVLs
    mode_trans_m         = 30.0,
    dormant_timeout      = 60,              # vehicles go underground for hours
    mou_models = {
        "parked":   {"theta": 5.0,  "sigma": 0.2},
        "urban":    {"theta": 0.20, "sigma": 5.0},
        "highway":  {"theta": 0.05, "sigma": 12.0},
        "sprint":   {"theta": 0.02, "sigma": 20.0},
    },
)
```

---

## 3.4 Fugitive Tracking and Reacquisition

### The Operational Problem

A fugitive is a tracked individual who has deliberately broken contact with the tracking system — escaped custody, gone underground, relocated. The operational challenge is reacquisition: finding them again after a gap that may range from hours to months, and doing so faster than they can establish a new cover identity.

### ARIA-INTEL's Dormant Reacquisition System

The dormant track mechanism was designed for exactly this scenario. When a track's existence probability drops below the prune threshold but a PoL model exists, the track enters dormant state and persists for up to 40 scans (configurable). The PoL model preserves everything learned about the fugitive's habits: which areas they frequent at which times, which routes they take, which locations they visit.

For fugitive tracking, the dormant timeout should be extended significantly beyond 40 scans:

```python
# Extend dormant timeout for fugitive operations
FugitiveProfile = DomainProfile(
    name            = "FugitiveTracking",
    scan_dt_s       = 3600.0,        # 1-hour aggregation (informant reports)
    dormant_timeout = 720,           # 30 days of hourly scans
    p_detection     = 0.20,          # fugitive actively avoiding detection
    pos_noise_m     = 50.0,          # coarse position from informant reports
    rv_warning_horizon_s = 43200.0,  # 12-hour warning for known associate meetings
    hvl_radius_m    = 500.0,
)
```

### Predictive Reacquisition

The PoL location prediction provides the most operationally valuable output for fugitive tracking: given the fugitive's historical routine, where are they likely to be at a specific future time?

A fugitive who previously worked a day job, visited family on weekends, and frequented specific venues will — under the psychological pressure of being a fugitive — tend to revert to familiar locations and routines. The PoL model captures these tendencies. The predict_location(t) method can be queried for any future timestamp, providing a probability distribution over likely locations.

Operational use: "Based on the last 3 months of observation data, this fugitive is most likely to be in the eastern suburbs between 6–9 PM on weekdays, with highest probability near location X (their former residence) between 7–8 PM on Fridays." This generates a targeted surveillance deployment rather than a citywide search.

### Wide-Area Movement Flag

The OperationalIntelligence system flags tracks whose total displacement exceeds 3km across their position history as WIDE_AREA_MOVEMENT. For fugitive tracking, this flag identifies a subject who has moved to a new area — potentially a new city or jurisdiction — as opposed to one who is lying low in familiar territory. This distinction affects the surveillance deployment strategy significantly.

### Associate Network Prediction

A fugitive will eventually make contact with their support network — family, criminal associates, former colleagues. These associates may themselves be tracked (on the watchlist). The rendezvous warning system will provide 30-minute advance warning when the fugitive approaches a known associate's PoL-predicted location — even if the fugitive has not been directly observed for days, because the warning fires when the fugitive's own PoL prediction places them near the associate's predicted location via Method 3 (PoL cross-prediction).

---

## 3.5 Vehicle Surveillance and Fleet Tracking

### The Operational Problem

Vehicle surveillance covers: surveillance of specific vehicles under investigation, convoy/motorcade tracking for protection operations, fleet monitoring for compliance, and vehicle identification in a wider surveillance context.

### VehicleConvoy Profile Applications

The existing VehicleConvoy DomainProfile was designed for military use but translates directly to law enforcement vehicle tracking:

**Surveillance vehicle detection:** A vehicle that maintains a consistent heading with, and lateral offset from, a subject vehicle for more than 6 consecutive scans triggers PARALLEL_SURVEILLANCE. This detects hostile surveillance of law enforcement vehicles and protected persons, as well as identifying surveillance vehicles in criminal networks.

**Driver swap detection:** The Mode Transition detector fires when a vehicle stops and a foot-class track appears nearby within a configured window. In the vehicle-surveillance context, this identifies: driver swaps (the vehicle is transferred to a new driver to avoid surveillance recognition), drop-offs of persons or packages, and staging locations where a vehicle crew prepares for an operation.

**Vehicle convoy integrity monitoring:** For protective operations (VIP convoy, prison transfer, evidence transport), the VehicleConvoy profile monitors the gap structure of the convoy. A vehicle leaving the convoy cluster (separation exceeds coloc_dist_m = 100m for multiple scans) fires a STATIONARY_DWELL alert via the OperationalIntelligence system. An unscheduled vehicle approaching the convoy (within rv_threshold_m = 30m) fires RENDEZVOUS_WARNING. An intercept vehicle approaching the convoy on a converging heading fires within minutes due to the 5-minute warning horizon of the VehicleConvoy profile.

**Stolen vehicle network tracking:** A criminal network running stolen vehicles will exhibit characteristic patterns: vehicles from the same source cluster (appearing at the same location shortly after theft is reported), being transferred via Mode Transition events (driver handoffs), and converging on chop shops (LOITER_ANOMALY at a fixed industrial location where the vehicle remains stationary for hours — inconsistent with any legitimate vehicle operation at that location and time).

---

## 3.6 Public Order and Protest Intelligence

### The Operational Problem

Public order operations require understanding crowd structure and dynamics before and during mass gatherings. Key questions: Who is organising the event? Are there embedded groups planning disruption? Are there organised criminal elements (looters, agitators) operating within a legitimate protest? Are there intelligence gatherers (hostile state operatives) documenting participants?

ARIA-INTEL's public order use raises significant proportionality concerns and must be deployed under clear legal authority. The engine's capabilities should be applied narrowly to persons of existing interest within the gathering, not to general surveillance of protest participants.

### Organiser Identification

In a mass gathering, the event organisers — stewards, marshals, coordinators — will exhibit the highest betweenness centrality in the co-location graph. They move between all groups, connect separate clusters, and receive a steady stream of brief contacts. The HANDLER role in ARIA-INTEL's network taxonomy maps to this function. Plotting betweenness centrality across the crowd automatically identifies the coordination structure.

A professional agitator (someone embedded in a legitimate protest to direct disruption) will have an unusual PoL profile — their presence at this location and this time will be anomalous against their historical routine (which is at their home city, not this location). Combined with the ASSET role classification (irregular activity, few contacts), this creates a distinctive signature.

### Forward Scout and Advance Reconnaissance

Individuals who arrive significantly before the main gathering and systematically move through the area — examining potential entry and exit points, identifying police positions, assessing crowd control infrastructure — exhibit the chokepoint surveillance pattern. The ChokepointSurveillanceDetector will flag these tracks if they make multiple passes through the same narrow geographic areas (doorways, bridge entry points, route constriction points) with bidirectional heading variance.

### Coordinated Group Movement Detection

New detector for public order context:

```
CoordinatedGroupDetector(BaseDetector):
  Identifies multiple tracks (3+) moving in tight formation with:
    - Similar headings (cosine similarity > 0.95 for all pairs)
    - Consistent inter-track separations (within 10m of initial formation spacing)
    - Duration > coordination_scans (5 consecutive scans)
    - No PoL model (new to the area — not local residents)
  
  Fires: COORDINATED_GROUP_MOVEMENT at HIGH severity
  Output: group track IDs, formation centre, heading, speed, estimated size
```

This fires for organised groups moving in coordinated fashion through a crowd — consistent with professional agitators, organised criminal elements, or hostile intelligence collection teams working in formation.

---

## 3.7 Border Control and Smuggling

### The Operational Problem

Border control tracking covers land border crossings, maritime entry points, and inland check-points. The smuggling problem requires detecting vehicles and persons making repeated crossings in suspicious patterns, or operating outside normal crossing behaviour.

### CrossBorder Domain Profile

```python
BorderPatrol = DomainProfile(
    name                 = "BorderPatrol",
    scan_dt_s            = 300.0,          # 5-minute observation windows
    pos_noise_m          = 15.0,
    p_detection          = 0.80,
    rv_threshold_m       = 500.0,          # vehicle rendezvous in border zone
    rv_warning_horizon_s = 3600.0,         # 1-hour warning
    brush_pass_m         = 100.0,
    cover_stop_hvl_m     = 2000.0,
    coloc_dist_m         = 1000.0,
    hvl_radius_m         = 1000.0,         # crossing points as HVLs
    dormant_timeout      = 288,            # 24 hours at 5-minute scan rate
    mou_models = {
        "on_foot":    {"theta": 0.30,  "sigma": 1.5},
        "vehicle":    {"theta": 0.05,  "sigma": 10.0},
        "stationary": {"theta": 3.0,   "sigma": 0.2},
        "offroad":    {"theta": 0.10,  "sigma": 6.0},
    },
)
```

### Smuggling Pattern Detection

**Repeat crossing without documentation:** A vehicle or person who repeatedly appears at unofficial crossing points (not at designated checkpoints) will accumulate PoL observations reflecting these crossings. The PoL anomaly detector will flag the crossing location as a routine for that entity — at which point it is anomalous not because the crossing is irregular but because a routine at an unofficial crossing is itself suspicious.

**Cross-border vehicle rendezvous:** Two vehicles meeting at a border zone — one from each side — with no legitimate commercial reason is a classic smuggling handoff. The rendezvous warning system provides advance warning of these meetings. Configuring known unofficial crossing routes as HVLs enables the CoverStopDetector to catch vehicles that stop near crossing points as part of a transit routine.

**New Detector: CrossBorderMovementDetector:**

```
CrossBorderMovementDetector(BaseDetector):
  Requires: border_geometry (list of line segments)
  
  Tracks position history of all confirmed tracks.
  When a track's position history crosses a border segment:
    - Fires CROSS_BORDER_MOVEMENT at HIGH severity
    - Records: crossing point, crossing time, approach heading, speed
  
  Loitering within border_buffer_m (default 2km) of a crossing point
  for > loiter_scans (default 3) fires BORDER_STAGING_SUSPECTED at MEDIUM.
  
  Repeat crossings > crossing_threshold (configurable) fire
  REPEAT_BORDER_CROSSING at IMMEDIATE severity.
```

---

## 3.8 Financial Crime and Asset Tracking

### The Operational Problem

Financial crime surveillance — money laundering, fraud, corruption — requires tracking the movement of both people and assets through a network. ARIA-INTEL's spatial tracking applies when financial crime has a physical component: cash couriers, property inspections before fraudulent transactions, repeated visits to financial institutions or legal offices.

### Mapping Physical Financial Crime Networks

The pattern is similar to drug trafficking but the "product" being moved is cash, documents, or information. The NetworkRoleInferenceDetector's COURIER/HANDLER/ASSET taxonomy maps to money mule couriers, money laundering operators, and the criminal principals directing the operation.

**Cash courier detection:** A cash courier makes regular runs between collection points (legitimate businesses used as fronts), processing locations (exchange houses, accountants), and banking facilities. The MOU model will classify them as vehicle-class transit during runs and stationary at processing points. The PoL model will capture the regular run schedule. The Mode Transition detector will fire at each handoff point where cash is transferred from one carrier to another.

**Cover Stop near financial institutions:** The CoverStopDetector configured with known financial institutions, law offices, estate agents, and corporate registry offices as HVLs will flag persons making routine visits to these locations at a frequency inconsistent with legitimate business.

**Transaction-correlated movement (new capability):**

For financial crime investigations where transaction records are available alongside movement data, a new FusionLayer can correlate temporal patterns:

```
FinancialCrimeFusion:
  Ingests: financial transaction events (timestamp, amount, institution)
           alongside ARIA-INTEL observation stream

  Correlates: does a track arrive near institution X within
              [pre_window] minutes BEFORE a transaction, and/or
              depart within [post_window] minutes AFTER?
  
  Accumulates: correlation score per track-institution pair
  
  Fires: TRANSACTION_CORRELATED_MOVEMENT when correlation score
         exceeds threshold across multiple transactions
```

This is implemented as a custom BaseDetector injecting transaction data via the context dictionary. The correlation is statistical: a single coincidence is noise; consistent correlation across 5+ transactions is a strong operational indicator.

---

*Part 3 complete. Part 4 covers intelligence agency use cases.*
# ARIA-INTEL Law Enforcement & Intelligence Technical Brief
## Part 4 — Intelligence Agency Use Cases

*Author: Odin Loch*

---

## 4.1 HUMINT Case Officer Tradecraft Detection

### The Operational Problem

This is the original use case ARIA-INTEL was designed for. A HUMINT (Human Intelligence) case officer running an agent network meets their sources in person, using professional tradecraft to avoid detection by hostile services or law enforcement. Counter-intelligence operations — identifying foreign intelligence officers operating in-country — require detecting this tradecraft.

Equally, a law enforcement counter-intelligence unit monitoring a subject who may be in contact with a foreign intelligence service needs to detect the specific operational patterns that distinguish intelligence tradecraft from ordinary behaviour.

### How ARIA-INTEL Was Built for This

Every tradecraft detector in ARIA-INTEL was designed with the HUMINT case officer scenario in mind:

**The SDR:** Before every meeting with a source, a professional case officer runs a Surveillance Detection Route — a deliberate, circuitous path designed to flush surveillance. The SDR Pattern detector fires when the winding number of a track's recent history exceeds 0.65. This is not an accident or a calibration choice: 0.65 was chosen to match the operationally realistic SDR loop geometry that a trained officer would execute in an urban environment.

**The Brush Pass:** The first contact with a source may be a brief, apparently accidental physical contact — a brush pass of documents, cash, or a device. The brush_pass detector fires on the first scan of contact (two tracks within 60m), not repeatedly, because the real operational significance is the initial contact event, not the sustained co-location.

**The Dead Drop:** Case officers and their agents often exchange material without meeting at all, using dead drops. The detector fires on sequential visits to the same small geographic cell with no simultaneous presence — exactly the dead drop protocol.

**Mode Transition:** Case officers change transportation mode to defeat vehicle surveillance — arriving by car, switching to foot, switching again to public transport. The Mode Transition detector fires on each vehicle-to-foot transition.

**Parallel Surveillance:** Hostile services may have the case officer under surveillance using mobile surveillance teams. The Parallel Route Surveillance detector fires when a track maintains a consistent lateral offset and matching heading with the case officer for multiple consecutive scans — the signature of a mobile surveillance team.

**Cover Stop:** The case officer's cover stop is a location they visit routinely and legitimately, near their actual operational target area. The CoverStopDetector cross-correlates routine-seeming visits with HVL proximity.

**Chokepoint Surveillance:** A hostile service case officer conducting reconnaissance of a potential meeting site will make multiple passes through the same narrow geographic area — a doorway, an alley entrance, a bridge — to assess surveillance presence and map the operational environment. The Chokepoint Surveillance detector catches this.

### Counter-Intelligence Network Structure

In a hostile intelligence network operating domestically:
- The **Rezident** (senior intelligence officer operating under diplomatic cover) = HANDLER in ARIA-INTEL's taxonomy. High betweenness centrality, low mobility, receives couriers.
- **Case officers** = COURIER role. Moderate mobility, multiple contacts, conduct SDRs.
- **Recruited agents** = ASSET role. High PoL anomaly (their intelligence activities are outside their normal routine), low contact count (meet only their handler).

The DynamicNetworkAnalyser will automatically construct this structure from the co-location history and betweenness centrality computation. A counter-intelligence analyst can then visualise the network hierarchy rather than manually correlating individual surveillance logs.

### Multi-Source HUMINT Fusion

A HUMINT operation will typically have multiple source types:
- Technical collection (phone location, licence plate readers): SIGINT/COMMS modality
- Human informant reports: HUMINT modality (lower reliability, 0.62)
- Open-source intelligence: OSINT modality (lowest reliability, 0.48)
- Physical surveillance: GEOINT modality (highest reliability, 0.90)

The CredibilityFuser combines all of these using Dempster-Shafer Theory. When a phone location says "subject is at location A" and an informant says "subject is at location B" simultaneously, the conflict coefficient K will be elevated — alerting the analyst that one of the sources may be compromised or that the subject has been separated from their phone.

---

## 4.2 SIGINT-Driven Tracking (Phone CDR and Device Location)

### The Operational Problem

Signals Intelligence (SIGINT) against mobile devices produces two primary location data types: Call Detail Records (CDRs) from mobile operators (giving cell tower location, accurate to hundreds of metres, available after the fact with legal process), and active device location from intercept (more accurate, available in near-real-time with operational authorities).

Both can be fed to ARIA-INTEL as SIGINT modality observations.

### CDR as ARIA-INTEL Input

A CDR record typically contains: device identifier (IMSI/IMEI), timestamp, cell tower ID (convertible to approximate coordinates), and event type (call, SMS, data session). Converting to ARIA-INTEL observations:

```python
# CDR → Observation conversion
for record in cdr_records:
    position = cell_tower_to_coordinates(record.tower_id)  # median tower footprint
    obs = Observation(
        obs_id    = record.event_id,
        timestamp = record.unix_timestamp,
        position  = np.array(position),
        modality  = "SIGINT",
        confidence = tower_radius_to_confidence(record.tower_radius),  
                     # large towers → low confidence (0.4); small towers → high (0.85)
        source_id = record.imsi   # per-device source tracking
    )
```

The confidence mapping from tower radius to confidence value is critical. A dense urban network with small cell towers (200m radius) gives a confidence of ~0.85. A rural tower covering a 5km radius gives confidence ~0.40. The particle filter handles this correctly: wide position uncertainty from a low-confidence observation spreads the particles across the tower's coverage area; subsequent observations narrow them down.

**PoL from CDR history:** If months of historical CDR data are available for a subject, ARIA-INTEL can be initialised with this history to immediately start with a fitted PoL model rather than waiting for 15 new observations. The PoL anomaly score becomes immediately operational. Historical CDR data ingested at accelerated time scale (1 hour of real time processed per second) builds the PoL model in minutes.

### Multi-Device Network Analysis

A criminal or intelligence network will use multiple devices — often deliberately rotating SIM cards to avoid persistent tracking. ARIA-INTEL handles this via the Source Credibility Tracker and the Gibbs assigner:

- Observations from different device identifiers (source_ids) that are spatially and temporally consistent with the same track will be assigned to the same track by the Gibbs assigner.
- The Source Credibility Tracker learns the reliability of each device identifier as a source. A device that consistently produces observations consistent with the track gets high credibility; a spoofed or rotated device will produce observations that periodically conflict with the track's predicted position, triggering the Possibility-PMBM mismatch diagnostic.

**IMSI Catcher / Cell Site Simulator Integration:** Active SIGINT collection via an IMSI catcher produces higher-accuracy location estimates (accurate to tens of metres) for devices within range. These observations enter as high-confidence SIGINT with source_id = IMSI_CATCHER_DEVICE_ID. The source credibility tracker will quickly learn that this source is highly reliable, and its observations will carry high weight in the particle filter update.

---

## 4.3 Counter-Intelligence: Detecting Hostile Surveillance Teams

### The Operational Problem

A hostile intelligence service conducting surveillance of a government facility, a protected person, or an intelligence officer will deploy a mobile surveillance team — typically 6–10 people operating as a coordinated unit, taking turns to "cover" the subject. This is the Mobile Surveillance Team (MST) pattern.

Detecting MST operations requires identifying: multiple individuals operating in a coordinated fashion, maintaining surveillance coverage of a single subject, cycling through different positions and routes to avoid triggering static surveillance detection.

### MST Detection with ARIA-INTEL

**Parallel Route Surveillance as MST component detection:** Each MST member, when in the "cover" position, maintains a consistent heading and offset from the subject. The ParallelRouteSurveillanceDetector will fire for each MST member during their active cover phase.

**Coordinated rotation detection (new):**

```
MSTRotationDetector(BaseDetector):
  Tracks which pairs of tracks have had recent PARALLEL_SURVEILLANCE events.
  When the same subject track appears with different surveillant tracks
  in rapid succession (rotation_window_scans, default 5), with the
  outgoing surveillant increasing separation and the incoming one
  closing from a different direction, fires:
  
  MOBILE_SURVEILLANCE_TEAM_DETECTED at IMMEDIATE severity
  
  Output: subject track, list of MST member track IDs, team size estimate,
          last 10 scan positions of all MST members
```

**Loiter Anomaly at known surveillance positions:** MST members frequently use static observation posts (OPs) during non-cover phases — a parked vehicle, a café, a public space with good sightlines. Their presence at these locations will be anomalous against their PoL (these are not their normal locations) and they will dwell for durations inconsistent with any legitimate purpose. The LoiterAnomalyDetector fires on these OPs.

**Chokepoint coverage pattern:** Before the subject departs a location, MST members pre-position at likely exit routes (the chokepoints). The ChokepointSurveillanceDetector will flag these members as they pass through the chokepoints in preparation for covering the subject's departure.

**Brush pass as team communication:** MST teams using physical signalling or dead drop communication between members (passing a physical signal device or documentation on the team's assignment) will produce BRUSH_PASS events between MST members. Combined with the network graph showing these members as a cluster with a shared subject under surveillance, this identifies the team structure.

---

## 4.4 Safe House and Dead Drop Infrastructure Mapping

### The Operational Problem

Intelligence operations require identifying the infrastructure that supports an adversary's operations: safe houses (locations used as secure meeting points, weapons storage, personnel accommodation), dead drops (fixed locations for exchanging material), and communications nodes (locations from which covert communications are made).

### Safe House Identification

A safe house exhibits a characteristic signature across multiple tracks:
1. Multiple different ASSET-role tracks visit the location at different times (not simultaneously)
2. HANDLER-role tracks visit the location regularly at specific times
3. The location appears in the PoL models of all these tracks — it is a shared routine node
4. The location is not an obvious public venue (low legitimate cover explanation)

ARIA-INTEL's cluster analysis identifies locations where multiple tracks have significant co-location history. The cover stop detector identifies locations that are routinely visited by tracks near high-value locations. Combining these two produces a safe house signature:

```
SafeHouseDetector(BaseDetector):
  Maintains: location grid (200×200m cells)
  Per-cell accumulation: set of track IDs that have visited, 
                          with timestamps and PoL anomaly scores at visit time
  
  When a cell accumulates:
    - >= safe_house_track_min (default 3) distinct tracks
    - all visit anomaly_scores < 0.6 (location is "routine" for all visitors)
    - visits spread across >= safe_house_session_min (3) distinct sessions
    - no two visits from different tracks simultaneously
  
  Fires: SAFE_HOUSE_SUSPECTED at HIGH severity
         SAFE_HOUSE_CONFIRMED (IMMEDIATE) after >= 5 tracks, 5+ sessions
  
  Output: location, visitor track IDs, visit history, dominant visiting times
```

### Dead Drop Mapping

The existing DEAD_DROP detector identifies sequential visits to the same 200×200m cell with no simultaneous presence. For dead drop infrastructure mapping (as opposed to single event detection), a new accumulation layer is needed:

```
DeadDropMapBuilder:
  Accumulates all DEAD_DROP events by location cell
  
  A cell that generates > dead_drop_event_threshold (default 3) 
  DEAD_DROP events across the session is flagged as 
  PERSISTENT_DEAD_DROP_LOCATION at IMMEDIATE severity
  
  Output: location, list of all participating track pairs,
          frequency distribution of visit times,
          likely dead drop schedule (peak hours)
```

---

## 4.5 Foreign Intelligence Network Mapping

### The Operational Problem

A foreign intelligence service operating domestically will have a structured hierarchy: the Rezident (senior officer, diplomatic cover), case officers (mid-level, diplomatic or commercial cover), agents (recruited locals), and support elements (logistics, communications). The operational goal is mapping this structure to identify: who the case officers are, who their local agents are, what facilities they are using, and what their collection priorities appear to be (inferred from what HVLs their cover stops approach).

### Automated Network Reconstruction

ARIA-INTEL's combination of network role inference, betweenness centrality, cluster analysis, and cover stop detection enables automated reconstruction of the intelligence network structure:

**Step 1 — Role identification:**
- Rezident: HANDLER role (low mobility, high betweenness, stable PoL at diplomatic premises)
- Case officers: COURIER role (moderate mobility, multiple contacts, SDR pattern)
- Agents: ASSET role (high PoL anomaly, low contact count, meets only their case officer)

**Step 2 — Cluster structure as cell structure:**
Each case officer and their agents form a distinct cluster in the network graph. Multiple clusters connected through the Rezident (high betweenness) constitute the overall network. The HANDLER node in each cluster is the case officer; the Rezident is the node connecting case officer clusters.

**Step 3 — Collection priority inference from cover stops:**
Each case officer's CoverStop events reveal what their service is prioritising. A case officer whose cover stops cluster near defence ministry buildings is running political/military collection. One whose cover stops cluster near technology company offices is running industrial espionage. The HVL configuration encodes the analyst's model of what constitutes a sensitive target.

**Step 4 — Communication infrastructure from dead drops and brush passes:**
The dead drop map identifies the operational infrastructure. Brush pass events between a case officer and an agent track the exchange of tasking and reporting materials.

**Step 5 — Counter-surveillance footprint from SDR and chokepoint events:**
Case officers who consistently execute SDRs before meeting their agents, and who conduct chokepoint surveillance before meeting locations, are operating with high professional tradecraft. Case officers who do not are either poorly trained or operating in a low-threat environment and taking shortcuts. This distinction is relevant to the counter-intelligence risk assessment.

---

## 4.6 Maritime Law Enforcement (Coast Guard, Customs, Fisheries)

### The Operational Problem

Maritime LE involves tracking vessels over large areas at long time intervals, with periods where vessels go "dark" (AIS transponder switched off, which is itself suspicious for commercial vessels required to carry AIS). The intelligence questions: Is this vessel following a route consistent with its declared purpose? Has it made undeclared rendezvous at sea? Is it transiting in a pattern consistent with smuggling (drugs, people, weapons)?

### Maritime Domain Profile

The existing Maritime() DomainProfile was built for this context. Key parameters:
- 3600-second (1-hour) scan rate matches AIS update frequency
- Position noise 200m matches AIS position accuracy
- P_D = 0.75 models AIS dark periods
- Meeting threshold 2,000m for vessel-to-vessel rendezvous
- 2-hour rendezvous warning horizon

### AIS Dark Vessel Detection

When a vessel switches off its AIS transponder, the track receives missed-scan updates and existence probability r decays. If the vessel has a PoL model (sufficient historical track), it enters dormancy rather than being pruned. The PoL model preserves the vessel's typical routes and anchorage patterns.

When the AIS transponder is switched back on, the observation is tested against the dormant track's PoL prediction. If the vessel has remained roughly where its PoL predicted (e.g., at its typical anchorage), the track reacquires normally. If the vessel has moved significantly from its predicted position during the dark period — it was somewhere unexpected — the reacquisition position contains operational intelligence: the vessel visited a location outside its PoL, which is the dead drop equivalent for maritime smuggling.

**AIS Dark Detector (new):**
```
AISDarkDetector(BaseDetector):
  When a maritime track transitions from confirmed to dormant
  (r drops below R_PRUNE after a run of missed observations):
    Records: vessel ID, last known position, last timestamp
  
  When track reacquires:
    Computes: displacement from PoL-predicted position
    If displacement > dark_displacement_threshold (default 10 nautical miles):
      Fires: UNEXPECTED_AIS_DARK_EXCURSION at HIGH severity
    Output: last known position, reacquisition position, displacement,
            dark duration, PoL-predicted position at reacquisition time
```

### At-Sea Rendezvous Detection

The rendezvous warning system is directly applicable: two vessels converging at sea, especially outside shipping lanes and away from anchorages, is a strong smuggling indicator. The Maritime DomainProfile's 2-hour warning horizon provides sufficient lead time to vector a patrol vessel or aircraft toward the predicted meeting location.

The PoL cross-prediction method (Method 3) is particularly valuable for maritime contraband operations where vessels follow regular supply schedules — a supply vessel that makes the same run every 10 days and meets the same receiving vessel at the same area of ocean will have a PoL model that predicts this meeting before either vessel has departed their respective ports.

**New Detector — Maritime Loitering:**
```
MaritimeLoiterDetector(BaseDetector):
  Identifies vessels loitering in international waters or off shipping lanes,
  outside any HVL (port, anchorage, fishing ground):
    - Vehicle drifting or stationary model dominant
    - PoL anomaly > 0.7 (not their normal location)
    - Dwell > maritime_loiter_hours (default 4 hours)
  
  Fires: SUSPICIOUS_MARITIME_LOITERING at MEDIUM severity
         Elevated to HIGH if another vessel approaches (rendezvous warning)
```

---

## 4.7 Counter-Proliferation and Special Category Tracking

### The Operational Problem

Counter-proliferation operations track persons involved in the procurement and transport of materials, technologies, or weapons subject to export controls or international treaty prohibition. The operational challenge is identifying which persons in a complex procurement network are making the critical linkages between foreign suppliers, domestic front companies, and end users.

### Network Role in Proliferation Context

The ARIA-INTEL network taxonomy maps to proliferation networks:
- **HANDLER:** The proliferation agent or acquisition manager — coordinates the network, maintains long-term relationships with both foreign suppliers and domestic clients.
- **COURIER:** The freight forwarder, the customs broker, the logistics coordinator — physically moves or documents the transaction.
- **ASSET:** The technical specialist providing procurement expertise, the front company director, the scientist with access.

The betweenness centrality identifies the critical nodes: removing the HANDLER disrupts the entire network, as they are the sole bridge between foreign supply and domestic demand.

### Cover Stops at Sensitive Technical Facilities

For counter-proliferation, HVLs are configured as:
- Research institutions with dual-use technology programmes
- Port and airport freight facilities
- Front company registered offices
- Foreign diplomatic missions (for liaison with procurement agents)
- Export control enforcement offices (which may themselves be under observation)

A procurement agent's visits to these locations — even if apparently routine (they may have legitimate cover business) — will be flagged by the CoverStopDetector when the pattern becomes established.

---

*Part 4 complete. Part 5 covers system integration, new profiles, and the full deployment brief.*
# ARIA-INTEL Law Enforcement & Intelligence Technical Brief
## Part 5 — Integration, Deployment, and Full System Architecture

*Author: Odin Loch*

---

## 5.1 Complete New Domain Profiles

The following domain profiles extend ARIA-INTEL for LE/intelligence deployments. Each is a direct instantiation of the existing DomainProfile dataclass — no code changes required.

```python
# ─────────────────────────────────────────────────────────────
#  CITY CAMERA SURVEILLANCE
# ─────────────────────────────────────────────────────────────
def CityCamera() -> DomainProfile:
    return DomainProfile(
        name                  = "CityCamera",
        scan_dt_s             = 60.0,
        pos_noise_m           = 8.0,
        p_detection           = 0.72,     # Re-ID misses + occlusion
        p_survival            = 0.992,
        rv_threshold_m        = 80.0,
        rv_warning_horizon_s  = 1800.0,
        brush_pass_m          = 40.0,
        parallel_route_m      = 60.0,
        parallel_vel_cos      = 0.96,
        parallel_scans        = 5,
        cover_stop_hvl_m      = 600.0,
        coloc_dist_m          = 150.0,
        hvl_radius_m          = 350.0,
        dormant_timeout       = 60,
        loiter_mult           = 2.5,
        chokepoint_m          = 25.0,
        chokepoint_n          = 3,
        mou_models = {
            "pedestrian":  {"theta": 0.35, "sigma": 1.5},
            "cycling":     {"theta": 0.18, "sigma": 3.5},
            "vehicle":     {"theta": 0.10, "sigma": 7.0},
            "stationary":  {"theta": 3.00, "sigma": 0.3},
        },
        model_trans = np.array([
            [0.80, 0.05, 0.08, 0.07],
            [0.04, 0.85, 0.06, 0.05],
            [0.05, 0.04, 0.86, 0.05],
            [0.20, 0.05, 0.05, 0.70],
        ]),
    )


# ─────────────────────────────────────────────────────────────
#  COUNTER-TERRORISM SURVEILLANCE
# ─────────────────────────────────────────────────────────────
def CTSurveillance() -> DomainProfile:
    return DomainProfile(
        name                  = "CTSurveillance",
        scan_dt_s             = 60.0,
        pos_noise_m           = 10.0,
        p_detection           = 0.65,     # covert multi-source collection
        rv_warning_horizon_s  = 3600.0,   # 1-hour warning for handler meetings
        rv_threshold_m        = 100.0,
        brush_pass_m          = 40.0,
        cover_stop_hvl_m      = 1000.0,
        coloc_dist_m          = 150.0,
        hvl_radius_m          = 800.0,
        dormant_timeout       = 120,      # subjects go quiet for 2 hours
        loiter_mult           = 2.0,
        dead_drop_spread      = (120.0, 3600.0),  # wider time window
        mou_models = {
            "foot":        {"theta": 0.30, "sigma": 2.0},
            "vehicle":     {"theta": 0.10, "sigma": 8.0},
            "stationary":  {"theta": 2.00, "sigma": 0.5},
            "fast":        {"theta": 0.05, "sigma": 15.0},
        },
    )


# ─────────────────────────────────────────────────────────────
#  ORGANISED CRIME NETWORK MAPPING
# ─────────────────────────────────────────────────────────────
def OrgCrime() -> DomainProfile:
    return DomainProfile(
        name                  = "OrgCrime",
        scan_dt_s             = 300.0,    # 5-minute aggregation windows
        pos_noise_m           = 20.0,     # phone location accuracy
        p_detection           = 0.55,     # irregular informant/tech coverage
        rv_warning_horizon_s  = 1800.0,
        rv_threshold_m        = 150.0,
        brush_pass_m          = 60.0,
        cover_stop_hvl_m      = 500.0,
        coloc_dist_m          = 300.0,
        hvl_radius_m          = 400.0,
        dormant_timeout       = 288,      # 24 hours at 5-min scan rate
        loiter_mult           = 3.0,
        courier_speed_thresh  = 5.0,
        courier_contact_n     = 4,
        handler_contact_max   = 3,
        mou_models = {
            "foot":        {"theta": 0.30, "sigma": 2.0},
            "vehicle":     {"theta": 0.10, "sigma": 8.0},
            "stationary":  {"theta": 2.00, "sigma": 0.5},
            "fast":        {"theta": 0.05, "sigma": 15.0},
        },
    )


# ─────────────────────────────────────────────────────────────
#  FUGITIVE TRACKING (LOW P_D, LONG DORMANCY)
# ─────────────────────────────────────────────────────────────
def FugitiveTracking() -> DomainProfile:
    return DomainProfile(
        name                  = "FugitiveTracking",
        scan_dt_s             = 3600.0,   # 1-hour report aggregation
        pos_noise_m           = 80.0,     # coarse informant reports
        p_detection           = 0.15,     # actively evading
        p_survival            = 0.998,    # track must persist through gaps
        rv_warning_horizon_s  = 43200.0,  # 12-hour warning for associate meetings
        rv_threshold_m        = 300.0,
        dormant_timeout       = 720,      # 30 days of hourly scans
        hvl_radius_m          = 500.0,
        coloc_dist_m          = 500.0,
        cover_stop_hvl_m      = 1000.0,
        mou_models = {
            "foot":        {"theta": 0.25, "sigma": 2.5},
            "vehicle":     {"theta": 0.08, "sigma": 10.0},
            "stationary":  {"theta": 1.50, "sigma": 0.8},
            "transit":     {"theta": 0.12, "sigma": 6.0},
        },
    )


# ─────────────────────────────────────────────────────────────
#  BORDER PATROL AND SMUGGLING
# ─────────────────────────────────────────────────────────────
def BorderPatrol() -> DomainProfile:
    return DomainProfile(
        name                  = "BorderPatrol",
        scan_dt_s             = 300.0,
        pos_noise_m           = 20.0,
        p_detection           = 0.78,
        rv_threshold_m        = 500.0,
        rv_warning_horizon_s  = 3600.0,
        brush_pass_m          = 100.0,
        cover_stop_hvl_m      = 2000.0,
        coloc_dist_m          = 800.0,
        hvl_radius_m          = 1000.0,
        dormant_timeout       = 288,
        mou_models = {
            "on_foot":    {"theta": 0.30,  "sigma": 1.5},
            "vehicle":    {"theta": 0.05,  "sigma": 10.0},
            "stationary": {"theta": 3.00,  "sigma": 0.2},
            "offroad":    {"theta": 0.12,  "sigma": 6.0},
        },
    )
```

---

## 5.2 Complete New Detector Specifications

The following detectors are new BaseDetector implementations for LE/intelligence use cases. Each follows the standard interface: `detect(tracks, context) -> List[Dict]`.

### 5.2.1 PreAttackPatternDetector (CT)

```python
class PreAttackPatternDetector(BaseDetector):
    """
    Detects pre-attack preparation patterns by combining:
      - Cover stops near HVLs (from CoverStop event history)
      - Acceleration in cover stop frequency
      - SDR execution near cover stop timing
      - Rendezvous with HANDLER-role track
    
    Requires context to carry 'all_detections' from prior detectors.
    """
    name = "PreAttackPattern"

    def __init__(self, profile):
        super().__init__(profile)
        self._cover_stop_history: Dict[str, List[Dict]] = defaultdict(list)
        self._sdr_history:        Dict[str, List[float]] = defaultdict(list)

    def detect(self, tracks, context) -> List[Dict]:
        ts = context['timestamp']
        all_det = context.get('all_detections', {})
        events = []

        # Ingest CoverStop events from this scan
        for ev in all_det.get('CoverStop', []):
            tid = ev['track']
            self._cover_stop_history[tid].append(ev)
            if len(self._cover_stop_history[tid]) > 20:
                self._cover_stop_history[tid].pop(0)

        # Ingest SDR events
        for ev in all_det.get('LegacyTradecraft', []):
            if ev.get('type') == 'SDR_PATTERN':
                self._sdr_history[ev['track']].append(ts)
                if len(self._sdr_history[ev['track']]) > 10:
                    self._sdr_history[ev['track']].pop(0)

        # Score each track
        for t in tracks:
            cs_hist = self._cover_stop_history.get(t.tid, [])
            sdr_ts  = self._sdr_history.get(t.tid, [])
            if not cs_hist: continue

            n_cover_stops = len(cs_hist)
            # Frequency acceleration: are cover stops getting more frequent?
            if n_cover_stops >= 3:
                intervals = np.diff([h['timestamp'] for h in cs_hist[-5:]])
                accel = float(np.polyfit(range(len(intervals)), intervals, 1)[0])
            else:
                accel = 0.0

            # SDR within 2 scan intervals of any cover stop
            sdr_near_cover = any(
                abs(s - h['timestamp']) < self.profile.scan_dt_s * 2
                for s in sdr_ts for h in cs_hist
            )

            if n_cover_stops >= 2 and (sdr_near_cover or accel < -50):
                severity = 'IMMEDIATE' if (n_cover_stops >= 3 and sdr_near_cover) else 'HIGH'
                events.append({
                    'type':            'ATTACK_PREPARATION_PATTERN',
                    'track':           t.tid,
                    'n_cover_stops':   n_cover_stops,
                    'freq_accel':      round(accel, 0),
                    'sdr_correlated':  sdr_near_cover,
                    'last_hvl':        cs_hist[-1].get('hvl'),
                    'timestamp':       ts,
                    'severity':        severity,
                    'interpretation':  'ESCALATING_RECONNAISSANCE_PATTERN',
                })
        return events
```

### 5.2.2 SafeHouseDetector

```python
class SafeHouseDetector(BaseDetector):
    """
    Identifies locations visited by multiple ASSET/HANDLER tracks
    in a non-simultaneous pattern — safe house signature.
    """
    name = "SafeHouse"

    def __init__(self, profile, cell_m=200.0, min_tracks=3, min_sessions=3):
        super().__init__(profile)
        self.cell_m = cell_m
        self.min_tracks = min_tracks
        self.min_sessions = min_sessions
        self._cell_log: Dict[str, Dict] = defaultdict(
            lambda: {'tids': defaultdict(list), 'events': 0}
        )

    def detect(self, tracks, context) -> List[Dict]:
        ts     = context['timestamp']
        events = []
        for t in tracks:
            cell = f"{int(t.pos[0]/self.cell_m)}_{int(t.pos[1]/self.cell_m)}"
            self._cell_log[cell]['tids'][t.tid].append(ts)

        for cell, data in self._cell_log.items():
            tids = list(data['tids'].keys())
            if len(tids) < self.min_tracks: continue
            # Check non-simultaneous: no two tracks' visits overlap within 30s
            all_visits = [(tid, t) for tid, ts_list in data['tids'].items()
                          for t in ts_list]
            simultaneous = sum(1 for i,(a,at) in enumerate(all_visits)
                               for b,bt in all_visits[i+1:]
                               if a != b and abs(at-bt) < 30)
            if simultaneous > 0: continue
            # Session count
            all_ts = sorted(t for ts_list in data['tids'].values() for t in ts_list)
            if len(all_ts) < 2: continue
            gaps = np.diff(all_ts)
            n_sessions = int(np.sum(gaps > self.profile.scan_dt_s * 10)) + 1
            if n_sessions < self.min_sessions: continue

            severity = 'IMMEDIATE' if len(tids) >= 5 and n_sessions >= 5 else 'HIGH'
            # Reconstruct approximate location from cell key
            ci, cj = map(int, cell.split('_'))
            approx_pos = [ci * self.cell_m + self.cell_m/2,
                          cj * self.cell_m + self.cell_m/2]
            events.append({
                'type':        'SAFE_HOUSE_' + ('CONFIRMED' if severity == 'IMMEDIATE' else 'SUSPECTED'),
                'cell':        cell,
                'position':    approx_pos,
                'visitor_ids': tids,
                'n_visitors':  len(tids),
                'n_sessions':  n_sessions,
                'timestamp':   ts,
                'severity':    severity,
            })
        return events
```

### 5.2.3 CrossBorderMovementDetector

```python
class CrossBorderMovementDetector(BaseDetector):
    """
    Detects tracks crossing defined border line segments.
    border_segments: List of ((x0,y0),(x1,y1)) tuples in the same
    coordinate system as track positions.
    """
    name = "CrossBorderMovement"

    def __init__(self, profile, border_segments, buffer_m=2000.0,
                 repeat_threshold=3):
        super().__init__(profile)
        self.segments = border_segments
        self.buffer_m = buffer_m
        self.repeat_threshold = repeat_threshold
        self._last_pos: Dict[str, np.ndarray] = {}
        self._crossing_count: Dict[str, int] = defaultdict(int)

    @staticmethod
    def _segments_intersect(p0, p1, q0, q1):
        """Returns True if segments p0-p1 and q0-q1 intersect."""
        def cross2d(a, b): return a[0]*b[1] - a[1]*b[0]
        r = p1 - p0; s = q1 - q0
        denom = float(cross2d(r, s))
        if abs(denom) < 1e-9: return False
        t = float(cross2d(q0 - p0, s)) / denom
        u = float(cross2d(q0 - p0, r)) / denom
        return 0 <= t <= 1 and 0 <= u <= 1

    def detect(self, tracks, context) -> List[Dict]:
        ts = context['timestamp']
        events = []
        for t in tracks:
            prev = self._last_pos.get(t.tid)
            curr = t.pos.copy()
            if prev is not None:
                for seg in self.segments:
                    q0 = np.array(seg[0]); q1 = np.array(seg[1])
                    if self._segments_intersect(prev, curr, q0, q1):
                        self._crossing_count[t.tid] += 1
                        n = self._crossing_count[t.tid]
                        events.append({
                            'type':      ('REPEAT_BORDER_CROSSING' if n >= self.repeat_threshold
                                          else 'CROSS_BORDER_MOVEMENT'),
                            'track':     t.tid,
                            'crossing_count': n,
                            'position':  curr.tolist(),
                            'heading':   float(np.degrees(np.arctan2(
                                             curr[1]-prev[1], curr[0]-prev[0]))),
                            'timestamp': ts,
                            'severity':  ('IMMEDIATE' if n >= self.repeat_threshold else 'HIGH'),
                        })

            # Staging detection: loitering near border
            min_border_dist = min(
                float(np.min(np.linalg.norm(
                    np.array([(1-s)*np.array(seg[0]) + s*np.array(seg[1])
                               for s in np.linspace(0,1,20)]) - curr, axis=1)))
                for seg in self.segments
            )
            if min_border_dist < self.buffer_m and float(np.linalg.norm(t.vel)) < 0.5:
                events.append({
                    'type':              'BORDER_STAGING_SUSPECTED',
                    'track':             t.tid,
                    'border_dist_m':     round(min_border_dist, 0),
                    'timestamp':         ts,
                    'severity':          'MEDIUM',
                })
            self._last_pos[t.tid] = curr

        return events
```

### 5.2.4 AISDarkDetector (Maritime LE)

```python
class AISDarkDetector(BaseDetector):
    """
    Detects vessels that go AIS-dark and reacquire at an unexpected position,
    indicating a covert excursion during the dark period.
    """
    name = "AISDark"

    def __init__(self, profile, dark_displacement_nm=10.0):
        super().__init__(profile)
        self.dark_displacement_m = dark_displacement_nm * 1852.0  # nm to metres
        self._last_confirmed: Dict[str, Dict] = {}
        self._went_dark: Dict[str, Dict] = {}

    def detect(self, tracks, context) -> List[Dict]:
        ts      = context['timestamp']
        events  = []
        # We monitor via the engine's pmbm dormant list (passed via context)
        # Here we approximate by tracking existence probability changes
        for t in tracks:
            tid = t.tid
            if tid not in self._last_confirmed:
                self._last_confirmed[tid] = {'r': t.r, 'pos': t.pos.copy(), 'ts': ts}
                continue
            prev = self._last_confirmed[tid]
            # Detect dark period ending: track was recently dormant/low-r, now confirmed
            if prev['r'] < 0.40 and t.r >= 0.55:
                dark_rec = self._went_dark.get(tid)
                if dark_rec and t.pol._fitted:
                    predicted, spread = t.pol.predict_location(ts)
                    actual_disp = float(np.linalg.norm(t.pos - dark_rec['pos']))
                    pol_disp    = float(np.linalg.norm(t.pos - predicted))
                    if actual_disp > self.dark_displacement_m:
                        events.append({
                            'type':               'UNEXPECTED_AIS_DARK_EXCURSION',
                            'track':              tid,
                            'dark_start_pos':     dark_rec['pos'].tolist(),
                            'reacquire_pos':      t.pos.tolist(),
                            'dark_duration_s':    round(ts - dark_rec['ts'], 0),
                            'displacement_m':     round(actual_disp, 0),
                            'pol_deviation_m':    round(pol_disp, 0),
                            'timestamp':          ts,
                            'severity':           'HIGH',
                            'interpretation':     'VESSEL_MADE_COVERT_EXCURSION_WHILE_AIS_DARK',
                        })
                self._went_dark.pop(tid, None)
            elif t.r < 0.40 and tid not in self._went_dark:
                self._went_dark[tid] = {'pos': prev['pos'].copy(), 'ts': ts}
            self._last_confirmed[tid] = {'r': t.r, 'pos': t.pos.copy(), 'ts': ts}
        return events
```

---

## 5.3 Sensor Integration Architecture

### 5.3.1 Sensor Front-End Summary

| Deployment type | Primary sensor | Secondary sensors | Observation modality | Front-end required |
|---|---|---|---|---|
| City camera | CCTV network | ANPR, WIFI probe | GEOINT, SIGINT | Re-ID pipeline + homography |
| CT watchlist | Covert CCTV | Phone location, informant | GEOINT, COMMS, HUMINT | Re-ID pipeline (targeted) |
| Organised crime | Phone CDR | Informant, CCTV | SIGINT, HUMINT, GEOINT | CDR ingestor |
| Drug trafficking | GPS trackers | ANPR, informant | GEOINT, HUMINT | GPS ingestor |
| Fugitive | Phone CDR | Informant reports | SIGINT, HUMINT | CDR ingestor + manual entry |
| Border patrol | LIDAR/radar | CCTV, ground sensors | GEOINT, SIGINT | Radar-to-position converter |
| Maritime LE | AIS | Radar, SIGINT | AIS, GEOINT | AIS stream parser |
| HUMINT case officer | Informant reports | Phone, CCTV | HUMINT, SIGINT | Manual + automated |
| Counter-intelligence | Multi-source | All available | GEOINT, SIGINT, HUMINT | All front-ends |

### 5.3.2 Standard Ingestor Interface

All front-end ingestors produce the same output type — a list of ARIA-INTEL Observation objects — regardless of source. This is the key architectural decision that makes ARIA-INTEL sensor-agnostic. The front-end is responsible for the translation; the engine knows only about Observations.

```python
class SensorIngestor(ABC):
    """
    Abstract base for all sensor-to-Observation converters.
    Every sensor type gets one concrete implementation.
    """
    @abstractmethod
    def ingest(self, raw_data) -> Tuple[List[Observation], float]:
        """
        Returns (observations, timestamp_unix_seconds).
        Called once per scan interval by the controller.
        """
        ...

class CDRIngestor(SensorIngestor):
    """Converts mobile operator CDR records to Observations."""
    def __init__(self, tower_db: Dict[str, Tuple[float, float, float]]):
        self.tower_db = tower_db  # tower_id → (lat, lon, radius_m)

    def ingest(self, cdr_batch) -> Tuple[List[Observation], float]:
        obs = []
        ts = max(r.timestamp for r in cdr_batch)
        for r in cdr_batch:
            if r.tower_id not in self.tower_db: continue
            lat, lon, radius = self.tower_db[r.tower_id]
            pos = latlon_to_metres(lat, lon)
            conf = float(np.clip(1.0 - radius / 5000.0, 0.25, 0.90))
            obs.append(Observation(
                obs_id    = r.event_id,
                timestamp = r.timestamp,
                position  = pos,
                modality  = "SIGINT",
                confidence= conf,
                source_id = r.imsi,
            ))
        return obs, ts


class GPSTrackerIngestor(SensorIngestor):
    """Converts GPS tracker records to Observations."""
    def ingest(self, gps_batch) -> Tuple[List[Observation], float]:
        obs = []
        ts = max(r.timestamp for r in gps_batch)
        for r in gps_batch:
            obs.append(Observation(
                obs_id    = r.device_id + "_" + str(r.timestamp),
                timestamp = r.timestamp,
                position  = np.array([r.x_metres, r.y_metres]),
                modality  = "GEOINT",
                confidence= 0.95,
                source_id = r.device_id,
            ))
        return obs, ts


class AISIngestor(SensorIngestor):
    """Converts AIS vessel reports to Observations."""
    def ingest(self, ais_batch) -> Tuple[List[Observation], float]:
        obs = []
        ts = max(r.timestamp for r in ais_batch)
        for r in ais_batch:
            pos = latlon_to_metres(r.lat, r.lon)
            obs.append(Observation(
                obs_id    = r.mmsi + "_" + str(r.timestamp),
                timestamp = r.timestamp,
                position  = pos,
                modality  = "AIS",
                confidence= 0.90,
                source_id = r.mmsi,
            ))
        return obs, ts
```

---

## 5.4 Deployment Architecture: Three Models

### 5.4.1 Single-Engine Tactical Deployment

The simplest deployment. One ARIA-INTEL engine instance, one sensor set, one operator.

```
Sensor front-end → Ingestor → ARIAIntelEngineV6 → Report → Operator terminal
```

Hardware: any x86 laptop or single-board computer (Intel NUC, Raspberry Pi 5). No GPU. No network beyond the sensor feed.

Use cases: mobile surveillance operations, single-investigation tracking, analyst workstation for reviewing historical data.

Latency: 28ms median per scan. Interactive.

### 5.4.2 Multi-Engine Zone Architecture

For city-scale or regional deployments, a zone architecture distributes load across multiple engine instances, each covering a geographic sub-area. A supervisor process routes observations to the correct zone engine based on position, and handles cross-zone track handoff.

```
Camera feeds ─┐
Phone CDR ────┤→ Zone Router → Zone Engine 0 (Sector A) ─┐
Informants ───┘               Zone Engine 1 (Sector B) ──┤→ City Aggregator → Analyst UI
                               Zone Engine N (Sector N) ─┘
```

**Cross-zone track handoff:** When a track in Zone Engine 0 is near the boundary between sector A and sector B, Zone Engine 0 serialises the track's state (particle filter particles and weights, PoL model, threat history, betweenness history) and sends it to Zone Engine 1. Zone Engine 1 creates a new track with the serialised state and continues tracking. The analyst UI sees a seamless track with no identity break.

**City Aggregator:** Receives confirmed track summaries (not raw particles) from all zone engines every scan. Deduplicates tracks that span zone boundaries (detected by matching track IDs or PoL model overlap). Produces the unified city-level common operating picture. Runs the IMMEDIATE/HIGH escalation alerting and the network-level analysis across all zones.

### 5.4.3 National/Multi-Jurisdictional Federation

For intelligence operations spanning multiple cities or countries, the zone architecture extends to a federation of city aggregators:

```
City A Aggregator ──┐
City B Aggregator ──┤→ National Fusion Node → Intelligence product
City N Aggregator ──┘
```

The National Fusion Node does not receive raw observations — only track-level summaries from city aggregators. This respects data sovereignty: each city's raw camera and communications data stays within its jurisdiction. Only the analytical product (confirmed tracks, events, network structures) crosses jurisdictional boundaries.

Track identity is maintained across cities via PoL model similarity matching: if City A's aggregator reports a track with a PoL model consistent with City B's track (similar spatio-temporal patterns after coordinate transformation), the national node fuses them under a single persistent identity.

---

## 5.5 Operational Control Interface

```python
class ARIAIntelOperatorAPI:
    """
    Wraps ARIAIntelEngineV6 with operational control functions for LE/Intel deployments.
    """
    def __init__(self, engine: ARIAIntelEngineV6, audit_log: AuditLogger,
                 auth: AuthorisationManager):
        self.engine = engine
        self.audit  = audit_log
        self.auth   = auth

    # ── Watchlist management ────────────────────────────────────────
    def add_watchlist_subject(self, subject_id: str, gallery_embeddings: List,
                               authority: str, expiry_date: float):
        """Add a person to the active watchlist. Requires legal authority record."""
        self.auth.validate(authority)
        self.audit.log("ADD_WATCHLIST", subject_id, authority)
        self.re_id_gallery.add(subject_id, gallery_embeddings, expiry=expiry_date)

    def remove_watchlist_subject(self, subject_id: str, reason: str):
        self.audit.log("REMOVE_WATCHLIST", subject_id, reason)
        self.re_id_gallery.remove(subject_id)

    # ── HVL management ─────────────────────────────────────────────
    def add_hvl(self, location: np.ndarray, label: str, authority: str):
        self.auth.validate(authority)
        self.audit.log("ADD_HVL", label, authority)
        self.engine.hvls.append(location)

    # ── Track actions ───────────────────────────────────────────────
    def annotate_track(self, track_id: str, annotation: str, analyst_id: str):
        self.audit.log("ANNOTATE_TRACK", track_id, analyst_id, annotation)

    def escalate_track(self, track_id: str, new_priority: str,
                        justification: str, analyst_id: str):
        self.audit.log("ESCALATE_TRACK", track_id, analyst_id,
                       f"{new_priority}: {justification}")

    def request_surveillance(self, track_id: str, modality: str,
                              authority: str, analyst_id: str):
        """Request active surveillance asset deployment. Requires authority."""
        self.auth.validate(authority)
        self.audit.log("SURVEILLANCE_REQUEST", track_id, analyst_id,
                       f"modality={modality} authority={authority}")

    # ── Query interface ─────────────────────────────────────────────
    def query_track(self, track_id: str) -> Dict:
        """Returns full track detail including PoL, events, network roles."""
        confirmed = self.engine.pmbm.confirmed()
        track = next((t for t in confirmed if t.tid == track_id), None)
        if not track: return {}
        return {
            'track_id':        track.tid,
            'position':        track.pos.tolist(),
            'velocity':        track.vel.tolist(),
            'pol_fitted':      track.pol._fitted,
            'pol_windows':     track.pol.active_windows(),
            'threat_history':  track._threat_history,
            'pos_history':     [p.tolist() for p in track._pos_history],
            'existence_p':     track.r,
            'poss_mismatch':   track.poss_mismatch,
            'dominant_model':  track.pf.dominant_model,
            'age_scans':       track.age,
            'parent_id':       track.parent_id,
        }

    def export_dossier(self, track_id: str, analyst_id: str) -> Dict:
        """Produces full analytical dossier for the track."""
        self.audit.log("EXPORT_DOSSIER", track_id, analyst_id)
        detail = self.query_track(track_id)
        # Add: all events involving this track from session history
        events = [e for r in self.engine.all_reports
                  for events_list in [r.get('tradecraft', []),
                                       r.get('rendezvous', []),
                                       r.get('network_roles', [])]
                  for e in events_list
                  if track_id in str(e)]
        detail['events'] = events
        detail['export_timestamp'] = time.time()
        detail['analyst_id'] = analyst_id
        return detail
```

---

## 5.6 Why ARIA-INTEL Outperforms Alternatives

This section is important for procurement, capability assessment, and technical defence of the system choice.

### The Association Problem

Every multi-target tracker must solve the data association problem: which observation belongs to which track? This is where the performance gap opens.

**Graph-based trackers (nearest-neighbour, Hungarian algorithm, JPDA):**

These approaches model data association as an assignment problem on a bipartite graph — observations on one side, tracks on the other, edges weighted by distance. Solving this graph exactly (via the Hungarian algorithm) takes O(N³) time where N is max(tracks, observations). Under high clutter (many false observations) or high density (many nearby tracks), the graph becomes dense and the algorithm becomes slow.

More critically, the graph approach commits to a single best assignment at each scan. If the best assignment is wrong — which happens during crossings, occlusions, and manoeuvres — the track identities are scrambled, and there is no principled mechanism to recover. The track picks up the wrong observations for the next several scans, during which its particle filter (if it has one) is being updated with corrupted position data. The error propagates forward in time.

**ARIA-INTEL's PMBM/Gibbs approach:**

The Gibbs sampler does not commit to a single assignment. It maintains a distribution over plausible assignments and samples from the high-probability region. This means that even if there is genuine ambiguity about which observation belongs to which track, the system does not make a definitive wrong choice. The association uncertainty is propagated forward: both tracks' particles are updated with appropriate weight for both possible assignments.

The Gibbs sampler runs in O(N_sweeps × N_tracks × N_observations) time — linear in tracks and observations, not cubic. At 50 tracks and 50 observations, the Gibbs sampler requires 50 × 50 × 14 = 35,000 operations per scan. The Hungarian algorithm requires O(50³) = 125,000 operations for the same input. As track count grows, the advantage compounds: at 200 tracks, Hungarian requires 8,000,000 operations versus 2,800,000 for Gibbs. And Gibbs provides better answers.

### The Prediction Problem

A track that goes off-sensor (missed observations) needs to be projected forward in time so that when observations resume, the track can reacquire rather than being started fresh as a new unknown. The quality of this projection directly affects reacquisition performance.

**Constant-velocity (CV) models:** Project the last known velocity forward. This is accurate for the first 5–10 seconds of a gap. For a 60-second scan interval, a pedestrian who was walking north may now be stopped, turned around, or on a bus. The CV prediction is wrong by any operationally useful standard.

**ARIA-INTEL's MOU model:** The Ornstein-Uhlenbeck process has a finite steady-state variance — it does not produce unbounded position uncertainty over time. For a 60-second gap, the OU process for the pedestrian model correctly produces a position distribution centred near the last known position (because pedestrians often remain near their last known location) but with variance consistent with how far a pedestrian can realistically travel in 60 seconds. This is physically correct and operationally useful.

**ARIA-INTEL's PoL prediction:** After 15 observations, the track also has a PoL model. During a dormancy period of hours to days, the PoL prediction takes over from the particle filter — projecting the target's likely location based on their historical routine rather than their last known velocity. No alternative operational tracking system has this capability.

### The Intelligence Problem

The gap between a position estimate and an intelligence product is enormous. Knowing that track T0023 is at coordinates [1200, 800] at 14:32 is not intelligence. Knowing that T0023 is at a location anomalous against their routine, approaching a known associate 28 minutes before a scheduled meeting, at a location where they have been detected 3 times in the last two weeks near the same HVL, with a rising threat score and a HANDLER role in a network whose other ASSET role member just executed an SDR — that is intelligence.

Every system that preceded ARIA-INTEL stops at the first sentence. The tracker gives you a position. What you do with it is left to the analyst. ARIA-INTEL produces the second sentence automatically, from the same data, at 28ms per scan, on a laptop.

---

## 5.7 Implementation Roadmap

### Phase 1 — Core LE Deployment (Weeks 1–4)

- Implement `CDRIngestor`, `GPSTrackerIngestor`, `ReIDIngestor` front-ends
- Implement `CityCamera`, `CTSurveillance`, `OrgCrime`, `FugitiveTracking` domain profiles
- Deploy `UrbanHUMINT` profile as default for LE operations
- Test against synthetic LE scenario generator (extend `generate_scenario()` with CT, drug trafficking, and fugitive scenario types)
- Implement `AuditLogger` and `AuthorisationManager` for legal compliance

### Phase 2 — New Detectors (Weeks 5–8)

- Implement `PreAttackPatternDetector`
- Implement `SafeHouseDetector`
- Implement `CrossBorderMovementDetector`
- Implement `AISDarkDetector`
- Implement `DeadDropMapBuilder` (accumulation layer over existing DEAD_DROP events)
- Integration test: end-to-end scenario for each detector type

### Phase 3 — Camera Front-End (Weeks 9–14)

- Integrate YOLOv8n object detection
- Integrate OSNet Re-ID model
- Implement homographic calibration per camera
- Implement gallery management system
- Implement `ReIDIngestor` converting Re-ID output to Observations
- Deploy against test camera network (minimum 10 cameras)
- Calibrate p_detection and pos_noise_m from empirical Re-ID performance

### Phase 4 — Zone Architecture (Weeks 15–20)

- Implement zone routing logic
- Implement track state serialisation/deserialisation for cross-zone handoff
- Implement city aggregator deduplication
- Deploy zone architecture against city camera test network
- Performance test at 50-track, 200-track, 500-track concurrent load

### Phase 5 — Operator Interface (Weeks 21–24)

- Implement `ARIAIntelOperatorAPI` full interface
- Implement watchlist management (add/remove subjects with authority records)
- Implement dossier export
- Implement session query interface for historical data
- User acceptance testing with operational team

### Phase 6 — Integration and Hardening (Weeks 25–30)

- Classification handling layer
- Data retention policies (configurable per track type)
- GDPR/legal compliance audit
- Penetration testing of network interfaces
- Full operational deployment

---

## 5.8 Summary: What the Complete System Delivers

An operational deployment of ARIA-INTEL for law enforcement and intelligence, at the completion of Phase 6, provides:

**Automatic tracking** of all persons in the sensor coverage area, with Bayesian existence probability (not just a "track exists" flag), full uncertainty quantification, and physically correct motion modelling that handles pedestrians, vehicles, cyclists, and public transport at the appropriate scan rates.

**Automatic pattern-of-life modelling** for every tracked person with sufficient observation history (15 observations, attainable in 15 minutes at 60-second scan rate). From this model: anomaly scoring against their own baseline, location prediction at any future time, and detection of breaks from routine that may indicate operational activity.

**30-minute advance rendezvous warning** for any pair of tracked persons, using three independent prediction methods that together achieve 100% detection. This is the foundational operational capability: 30 minutes is enough lead time to deploy surveillance or seek a warrant.

**Eight automatic tradecraft detectors** covering brush pass, surveillance detection routes, dead drops, mobile surveillance (tails), vehicle handoffs, loiter anomalies, cover stops, and chokepoint surveillance — all running simultaneously on every scan, on every confirmed track.

**Automatic network mapping** identifying the command structure of criminal or intelligence networks via betweenness centrality, with automatic HANDLER/COURIER/ASSET role assignment and cluster structure identification. The investigation's organisational chart is produced automatically.

**Multi-source evidence fusion** using Dempster-Shafer theory across any combination of GEOINT, SIGINT, COMMS, HUMINT, and OSINT observations, with automatic source credibility tracking that identifies unreliable or potentially compromised sources.

**Bayesian threat scoring** producing a calibrated [0, 1] threat score and priority tier for every confirmed track, based on eight evidence dimensions, with full uncertainty quantification (mean, std, P90, P95). Not a rule-based alert system — a probabilistic inference system.

**Edge-deployable performance**: 28ms median scan latency, single CPU core, no GPU, runs on a laptop. Scales linearly to 50+ tracks; with parallelisation, handles 200+ concurrent tracks. The competing graph-based trackers require GPU acceleration and distributed compute to approach this performance.

**Domain adaptability without code changes**: the same codebase handles urban HUMINT, maritime vessel tracking, airspace surveillance, vehicle convoys, city cameras, border patrol, and fugitive tracking by swapping a single configuration object.

**Composable, hot-swappable detectors**: new operational requirements are implemented as new BaseDetector plugins and registered at runtime. The core engine is never modified; capabilities are added incrementally without disrupting running operations.

**Full audit trail and legal compliance architecture**: every action, every alert, every data access is logged with timestamps and authority records, providing the evidentiary foundation for prosecution and the accountability record for oversight.

This is not a surveillance platform. It is an intelligence engine: a system that converts raw, uncertain, multi-source location data into structured, actionable, legally defensible intelligence products, automatically, in near-real-time, at a computational cost that makes deployment on modest hardware genuinely feasible.

---

*Part 5 complete. Merging all parts into the master technical brief.*
