# Porting notes: defects found and fixed

The C++23 port is not a transliteration. Twenty-one substantive defects were
found — eleven inherited from `reference/aria_intel.py`, ten introduced or
exposed by the port itself — while getting the simulations, then real
MOTChallenge data, and finally the engine's own cost profile to behave. Each is
recorded here with how it was found, why it was invisible before, and what
changed — partly as a changelog, partly because several are easy traps to fall
back into.

Two recurring themes, both about measurement:

**Choose metrics that can fail.** The reference reported peak track counts but
never identity continuity or false-track rates. Four of the seven inherited
defects are invisible in peak-track-count and position-error metrics, and
glaring the moment you count identity switches.

**Measure the input's own ceiling.** Three scenarios were documented as
tracking weaknesses until the sensors were asked what they had actually
produced. `anpr-corridor` recovers 111% of the detections its readers emit; it
was never failing. See [VALIDATION.md](VALIDATION.md).

---

## 1. Association was not one-to-one

**Severity: critical.** The Gibbs sampler discouraged two tracks from claiming
the same detection with a constant penalty (`0.3` per conflict) but never
forbade it. A one-to-one matching constraint expressed as a soft nudge is not a
constraint.

Consequence: when two tracks sat on one entity, *both* were fed that entity's
single detection every scan. Neither ever missed, so neither ever decayed, and
duplicates accumulated without bound. In the maze simulation this produced 11
tracks for 3 travellers and 53 identity switches in 60 scans.

**Fix:** the Gibbs sweep now samples each track's assignment conditioned on the
other tracks' current claims, excluding detections already taken — the correct
conditional for a matching. The greedy seed is likewise one-to-one.

Effect: identity switches 53 → 0; duplicate-track scans 53/60 → 0/60; latency
2.2 ms → 0.49 ms, because the engine was no longer carrying phantom tracks.

## 2. No track merging

**Severity: high.** Nothing ever merged two tracks that had converged onto the
same entity. The existence update alone cannot fix this: a duplicate that wins a
detection every few scans keeps resetting its own existence.

**Fix:** `PmbmManager::merge_duplicates`, gated on three tests — a distance
gate, a Mahalanobis test under the summed covariances, and a discriminator that
two genuinely distinct entities each generate their own detection in the same
scan, whereas two tracks on one entity can only take turns. Without that third
test a couple walking together would be collapsed into one track, which is a
worse error than the one being fixed.

## 3. Existence was updated twice per detection

**Severity: high.** `PMBMManager.update` performed the clutter-aware existence
update, then called `update_hit`, which performed *another* existence update
using a hardcoded clutter constant of `1e-4`. Every association was counted
twice.

**Fix:** existence is now updated in exactly one place — the manager, which is
the only component that knows the current clutter density. `Track::update_hit`
folds in the measurement and leaves `r` alone.

## 4. A newborn track was confirmed by its own birth detection

**Severity: high.** A new track was created with `r = r_birth`, then immediately
given `update_hit`, whose `c = 1e-4` drove `r` to 0.9998 — using the very
detection that created it. The birth threshold was therefore dead code, and
every false alarm clearing the birth weight gate became a *confirmed* track for
at least one scan. The ghost rate tracked the sensor's false-alarm rate exactly.

The reference got away with this because its synthetic clutter was low-weight
OSINT that never cleared the birth gate. A camera's false detections arrive at
full GEOINT weight and sail straight through.

**Fix:** no existence update on birth; `r_birth < r_confirm` in most profiles,
so a track needs corroboration from a *subsequent* scan before it is reported.
Profiles where a lone sighting is precious (`FugitiveTracking`,
`WildlifeTelemetry`) invert this deliberately and say so.

## 5. Motion models were in "scan units" while every profile was written in SI

**Severity: critical, and the most insidious.** `MouConstants` were built with a
nominal `dt = 1.0` "scan", making sigma metres-per-scan. But every profile — in
the reference and in the brief's proposed profiles — was written as if sigma
were metres-per-second.

At a 60-second scan the urban `vehicle` regime meant 0.30 m/s rather than 18.
For `Maritime` at an hourly revisit the mismatch was a factor of ~2000: a vessel
covering 21.6 km between scans was being modelled by a filter that expected 9.5
metres. The filter simply could not follow, and the failure surfaced much later
as unexplained identity switches rather than as anything obviously unit-shaped.

**Fix:** SI throughout. `theta` is per second, velocity is m/s, and one
`predict()` advances the profile's real scan period. Profiles now build motion
models through a helper that takes quantities a person can check —

```cpp
motion("transiting", /*holds heading*/ 14400.0, /*typical speed*/ 6.0)  // ~12 knots
```

— because raw theta/sigma pairs are effectively unverifiable by inspection.

Effect on the dark-vessel scenario: detection rate 15% → 82%.

## 6. The IMM never interacted with the measurements

**Severity: medium.** The motion-regime mixture `mu` was only ever advanced
through the transition matrix. No measurement ever updated it, so it relaxed to
the transition matrix's fixed point and stayed there. `dominant_model` — which
`ModeTransitionDetector` depends on — reported a constant.

The first fix attempt (summing particle weight by each particle's regime) also
failed, for an interesting reason worth recording: over one scan a regime only
controls how fast velocity *decays*. At a 1-second scan with heading-hold times
of 4–15 seconds, every `alpha` is 0.78–0.94, so a particle labelled "standing"
that inherited 9 m/s still travels about 7 m. Every regime explains a single
step almost equally well; per-step particle mass carries no regime information
at all.

**Fix:** regimes are separated by *sustained speed*, which is what their
steady-state distributions actually describe. Under an OU velocity process each
axis is zero-mean Gaussian, so speed is Rayleigh with scale `sqrt(ss_vvar)`.
Scoring the track's speed against each regime's Rayleigh discriminates properly.

Effect: a stationary entity now reports `standing` at p = 0.85 (previously
`walking` at 0.57, indistinguishable from a 9 m/s mover).

## 7. Tracks could never time out

**Severity: medium.** Retirement depended solely on existence decay. Where
`p_detection` is low, a miss is almost uninformative — at `p_d = 0.15`,
existence falls from 0.9990 to 0.9988 per missed scan — so nothing was ever
retired and stale tracks accumulated indefinitely. This is correct Bayesian
behaviour and a useless operational outcome.

**Fix:** `max_coast_s`, a hard wall-clock limit on unseen coasting, independent
of existence. Tracks that time out with a fitted pattern of life become dormant
identities rather than being discarded, so they can still be reacquired.

---

## 8. Scoring used per-entity nearest-neighbour, not an assignment

**Severity: high — and it was in the measuring instrument.** `score_scan`
matched each truth entity to its own nearest track independently. That lets one
track "cover" several entities at once and reports identity switches whenever
the arbitrary winner changes, so the numbers judging every other fix were
themselves unreliable.

**Fix:** `sim/assignment.*` — a gated minimum-cost matching, Hungarian below 64
rows and globally-sorted greedy above it, where an exact O(n^3) solve per frame
across thousands of frames is not affordable.

Worth recording that this *disproved* a hypothesis rather than confirming one:
the `warehouse` scenario's high switch count had been attributed to metric
ambiguity from eight entities converging inside the match radius. Under optimal
assignment it moved from 871 to 868. Those switches are real.

## 9. Dormancy was, in two profiles, impossible

**Severity: high.** A track went dormant only while its existence sat between
`r_dormant` and `r_prune` — a band typically 0.01 wide, which a decaying
existence falls straight through in a single scan. Two shipped profiles had the
two values inverted, making the band empty and dormancy unreachable. Dormancy
additionally required a fitted pattern of life, which a short-lived track can
never have.

The consequence is quiet: reacquisition never fires, every reappearance becomes
a new track, and the cost shows up as identity switches far from the cause.

**Fix:** dormancy now depends on whether the track was *ever confirmed* — a
question about its history, not about where a decaying number happened to stop.

## 10. Reacquisition had only one cue, on the wrong timescale

**Severity: medium.** The only reacquisition mechanism was pattern-of-life
prediction, a `[hour, x, y]` model built for multi-day routine. In any short
sequence hour-of-day is constant and carries no information, so the mechanism
that exists to preserve identity across a gap could not operate at all on
short-lived tracks.

**Fix:** a second, complementary cue. Pattern of life answers "days later, at
his usual place"; kinematic extrapolation answers "moments later, where he was
heading", with the gate widening as the gap grows. Which one dominates is a
per-domain choice, and the MOT profile deliberately keeps the kinematic window
very short — see [VALIDATION.md](VALIDATION.md).

## 11. The merge gate did not scale with motion

**Severity: medium.** The duplicate-track merge gate was derived from sensor
noise alone. Measured against how far an entity travels between scans — which
is roughly how far from the original a duplicate is born — it was 75% of one
scan of motion in the maze and about **2%** in the sparse domains (hourly AIS,
four-hourly collar fixes). Merging appeared to work, because it worked in the
one scenario anyone was watching.

**Fix:** the gate scales with per-scan motion. Widening it is safe because
distance is only a prefilter here; the test that actually decides is whether the
two tracks were ever fed their own detection in the same scan.

## 12. The motion constraint leaked, one particle at a time

**Severity: medium — introduced by this port, not inherited.** The new
graph-constrained motion model decided *per particle* whether a position was
close enough to the network to be projected onto it. Any particle that wandered
past the tolerance was thereafter exempt and free to keep going, so the cloud
leaked sideways one particle at a time — exactly what the constraint exists to
prevent.

**Fix:** the on-network decision is made once, for the whole cloud, from its
weighted mean. Whether an entity is on a road is a fact about the entity, not
about each Monte-Carlo sample. Measured over 30 coasting scans: unconstrained
lateral spread 2,481 m, constrained 0 m.

## 13. Association exclusivity was global, not per sensor

**Severity: high.** A detection already claimed by another track was excluded
from every other track's candidate list. That is right within one sensor — a
camera reports a given entity once per scan — and wrong across sensors, where
two overlapping cameras both report the same entity and the second report is
*corroboration*, not a second entity.

The corroborating report was left unassigned, where it promptly founded a
duplicate track. Multi-source fusion is the entire purpose of the observation
model, so this was the case that most needed to work.

**Fix:** association runs once per source. A track may hold at most one
detection from each sensor, and existence is updated once per scan however many
sensors reported it — counting each separately would make an entity watched by
four cameras four times as certain as the same entity watched by one.

Effect: transit-hub (overlapping ceiling cameras) ghost tracks fell from 2.08
to **0.12** per scan; the behaviour-space scenario from 10.95 to 4.10; evader
identity switches from 8 to **0**.

## 14. The possibility/probability mismatch could only ever fire

**Severity: medium, and it had been reported as a feature.** The possibilistic
existence `pi_r` was a running product of factors that are always ≤ 1, so it
could only fall, while the Bayesian `r` rose to ~1. Every long-lived track
therefore converged to a mismatch of 1.0 regardless of the evidence behind it.

Measured in the spoofing scenario built for it: the diagnostic flagged a
fabricated track on 119 of 119 scans — and flagged real tracks just as hard,
both peaking at 1.000. It was noise being read as a signal.

**Fix:** a possibility measure has to be able to rise — good evidence makes a
hypothesis *more* permissible, not less. `pi_r` now tracks the normalised
quality of a track's evidence, so it converges to that quality while `r`
converges to 1 on sheer count, and the gap means "weak evidence has been
laundered into certainty".

| | flagged | peak mismatch |
|---|---|---|
| real entities | 0/484 | 0.27 |
| high-confidence phantom | 0/119 | 0.21 |
| marginal-quality rumour | **119/119** | 0.71 |

Note the middle row. The fixed diagnostic does **not** catch a convincing lie,
and cannot: a high-confidence fabrication looks exactly like high-confidence
truth on evidence quality alone. The reference's claim that it "flags sensor
deception" holds only for *low-quality* deception, and the scenario now says so.

## 15. The hub test used an absolute threshold in a relative classifier

**Severity: low.** Role inference compares speed against the population's own
median — deliberately, so it travels between domains — but tested betweenness
against a fixed 0.2. A normalised betweenness above 0.2 requires a near-perfect
star topology, which real contact graphs are not, so the HANDLER branch
effectively never fired and hubs fell through to the catch-all role.

**Fix:** betweenness is compared against the population's upper quartile, like
speed.

## 16. The merge discriminator mistook two sensors for two entities

**Severity: medium — a direct consequence of fixing #13.** Duplicate-track
merging is blocked when two tracks were each fed a detection in the same scan,
on the reasoning that one entity cannot produce two simultaneous detections.
Once association became per-sensor, that reasoning broke: one entity under two
overlapping cameras produces exactly that pattern every scan, so the two tracks
founded at birth could never be merged. Measured across nine seeds, only five
collapsed to a single track.

**Fix:** the discriminator became per (scan, sensor) rather than per scan. A
sensor reports a given entity once per scan, so one sensor feeding both tracks
in one scan settles it; the same scan via two *different* sensors proves
nothing.

That alone over-corrected, merging genuinely distinct entities whenever source
identifiers carried no spatial meaning, so a second guard was added: two tracks
both fed steadily by the *same set* of sensors, just never in the same scan, are
two entities whose detections alternate. Nine of nine seeds now collapse to one
track, and two entities under one sensor still stay separate.

A test-scenario defect surfaced alongside it: the synthetic generator assigned
each detection a source id at random from a pool, which no real sensor estate
does — a camera covers a region. Fixed to assign by region.

## 17. Source credibility was circular, and therefore inert

**Severity: high.** The per-source trust score asked one question: does this
source's report fit the track it was assigned to? That is circular. A sensor
whose reports have been steering a track all along fits it perfectly however
wrong it is.

Measured with a camera whose mount slipped 23 m over a run — sixteen times its
own noise — it ended scoring **0.950, higher than its sound neighbours**. The
mechanism had never been tested, and it did nothing.

**Fix:** three non-circular signals, answering different questions.

- *Detection* — the residual **direction**. A sound sensor is wrong in every
  direction equally; a biased one is wrong the same way every time, and the
  standard error of that mean falls as 1/sqrt(n).
- *Attribution* — a track is a weighted mean of the sources feeding it, so
  their residuals nearly cancel. A biased sensor drags the track towards
  itself, leaving its residual pointing one way and every sound sensor's
  pointing the other. **The minority direction is the culprit.**
- *Calibration* — not available. The absolute offset cannot be recovered from
  tracks the biased sensor helped build, because it drags them towards itself.
  That needs an independent reference: a surveyed landmark, or GPS truth.

The drifting camera is now uniquely flagged, credibility 0.434 against 0.724
for its neighbours, with no false accusations on a sound estate.

## 18. Peer attribution punished the innocent

**Severity: medium — introduced while fixing 17.** The first attempt compared
each source against the mean of the others. With only two sources reporting an
entity, that mean *is* the other source, so the disagreement is identical from
both sides and blaming either is a coin flip. It duly punished the sound camera
harder than the drifting one: 0.221 against 0.914.

**Fix:** peer attribution requires three or more sources, where the majority
pulls the mean towards the truth. With exactly two, the conflict is recorded and
surfaced — "these two disagree by 10.8 m on 92% of shared sightings" is
actionable even when "this one is wrong" is not knowable — and neither
credibility is touched.

## 19. A hardcoded ceiling of 80 tracks

**Severity: high for any crowded deployment.** `kMaxTracks = 80`, a file-scope
constant, silently discarded the weakest tracks beyond that. Offered 120, 180 or
270 entities, the engine tracked exactly 80 and said nothing. Every claim about
city-scale camera estates or stadium egress was bounded by a number nobody had
written down.

**Fix:** a profile field. A convoy needs a dozen, a city camera estate needs
hundreds; it is a domain decision like every other in a `DomainProfile`.

## 20. The convergence detector recomputed its forecasts once per pair

**Severity: high — this was 95% of the engine's runtime.** `pol_cross_predict`
built each track's pattern-of-life forecast *inside* the pair loop, so every
track's forecast was rebuilt once for every other track. At 270 tracks that is
270 times over: 33,000 pairs, each running up to 20 horizon steps of Monte-Carlo
GMM sampling for both parties.

Nothing about it was incorrect. It simply made the engine unusable at scale, and
no correctness test could have caught it.

**Fix:** compute each track's forecast once per scan and share it across every
pair. Measured at 270 tracks: the detector fell from **751 ms to 34 ms**, total
scan latency from **875 ms to 73 ms**, and overall cost from **n^1.82 to
n^1.14** — from approaching quadratic to effectively linear.

`tests/test_scaling.cpp` now guards the exponent, because this is precisely the
class of defect that passes every correctness test.

## 21. Betweenness centrality was O(V^3)

**Severity: medium.** Brandes' algorithm is O(V·E), but the implementation took
a dense adjacency matrix and scanned all V columns for every dequeued vertex,
making it cubic — and it built two dense n-by-n matrices every scan to do it.

**Fix:** adjacency lists throughout. This turned out *not* to be the bottleneck
(0.5 ms of a 135 ms scan at 400 tracks), which is itself the lesson: the
profiler found the real cost in one measurement after two wrong guesses.

## A note on measuring before optimising

Two hypotheses about where the time went were wrong before the third was right.
The spatial index built to fix the "obviously quadratic" pairwise detector loops
made almost no difference; the betweenness rewrite made none. Adding per-stage
timing to `ScanReport` found the true cost in a single run.

The per-stage breakdown is now part of every report, because the engine's cost
profile is not obvious from reading it: tracking is linear in track count, and
one detector was two orders of magnitude more expensive than the rest combined.

## A note on test thresholds

Three tests failed on the scalar backend while passing under AVX-512, which
looked like a backend defect and was not. The two paths consume the random
stream at different rates, and the outcome of this kind of tracking is strongly
seed-dependent: across 40 seeds the mean position error in the urban scenario
spans 13–209 m on both backends, with near-identical medians (63 m vectorised,
60 m scalar). The thresholds had been tuned to whichever seed was in front of
them.

Those tests now assert on a median across seeds, or on how many of N seeds
satisfy a structural property. A threshold fitted to one draw of a
high-variance process tests the draw.

## Smaller corrections

- **Trajectory update was dead code.** `update_hit` decided whether two
  detections were consecutive by comparing their gap against a fixed `_DT = 1.0`
  second. At the default 60-second scan the test never passed, so the
  velocity-from-two-detections update never ran. It now compares against the
  profile's own scan period.
- **`speed_mps` was not m/s.** It reported the filter's per-scan velocity under
  an SI name. Now genuinely SI.
- **Motion scoring used a fixed 30 m/s scale** for every domain, so a vessel and
  a sprinter were measured against the same yardstick. Now scaled per profile.
- **The SDR winding window was a fixed 12 samples.** For any loop taking longer
  than 12 scans the winding test could not reach threshold no matter what the
  entity did — the evader scenario proved it mathematically incapable of firing.
  Now `sdr_window`, a profile parameter.
- **The birth gate was derived from the fastest motion model.** A low-theta
  regime has an enormous steady-state velocity variance that says nothing about
  how far an entity travels in one scan; the camera profile's vehicle regime
  produced a 79 m gate across a 120 m maze, so everything corroborated
  everything. Now derived from the domain's own speed scale.

## Known limitations carried forward

Stated plainly, because the simulations make them measurable:

- **Appearance is implemented but does not help on MOT.** The mechanism works
  where descriptors are discriminative — six entities huddling then dispersing
  lose 6/6 identities without it and 3/6 with it. On MOT it delivers nothing,
  and a perfect *oracle* descriptor delivers nothing either, because 89% of the
  MOTA penalty there is missed detections. The earlier claim in this file that
  an appearance cue was "the obvious next step" was wrong, and the measurement
  that disproved it is in [VALIDATION.md](VALIDATION.md).
- **Role inference does not transfer to a new domain without calibration.** The
  behaviour-space scenario tracks well (104% of available detections) and its
  behavioural detectors fire, but the classifier does not recover ground-truth
  roles. Its thresholds are calibrated against a physical contact network.
- **Maritime sampling at 12 knots hourly.** A vessel moves 21.6 km between
  scans against a one-scan prediction uncertainty of ~13 km. `dark-vessel`
  recovers 70% of available detections, the only genuine shortfall among the
  scenarios, and it is an information limit rather than a tuning problem.
- **Regime identification needs the per-scan motion difference to exceed the
  measurement noise.** Where it does not, the regime posterior correctly falls
  back on the transition prior — correct, but not informative.
- **Nothing here is validated against real sensor data.** Every number in this
  repository comes from its own simulations.
