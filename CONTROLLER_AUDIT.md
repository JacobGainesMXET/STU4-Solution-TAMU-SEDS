# STU4 controller and score audit

## Current score optimization study

The active team checkout contains a score-driven controller ready for Space Teams trials. Its predecessor is preserved byte-for-byte in [comparison/pre_score_optimization](comparison/pre_score_optimization). The source hashes in that snapshot match the team's latest time-target recordings. The earlier under-50-kg variant is in [comparison/pre_transit_target](comparison/pre_transit_target). The active behavior and results below supersede the historical phases further down this file.

### Flight evidence and comparability

| Space Teams evidence | Arrivals or outcomes observed | Time | Fuel | Score |
|---|---|---:|---:|---:|
| Owner's older five zero-damage arrivals | 5 arrivals | 203.83 s | 53.39 kg | 37.92 |
| Source-matched under-50-kg controller | 2 arrivals | 219.43 s | 48.18 kg | 40.46 |
| Source-matched 150-second controller, successful debris flights | 4 arrivals | 188.55 s | 59.13 kg | 34.94 |
| Same 150-second controller, completed no-debris flight | 1 arrival | 150.67 s | 70.00 kg | 36.31 |

The last three rows use the first recorded entry into the 50 m success sphere. Those scores can differ slightly from the official scoring tick. The four successful debris flights had zero damage. That controller also logged two runs above the 209,200 J loss threshold and three further runs without a recorded terminal result, including two with damage. Among six identifiable outcomes it recorded four arrivals. Applying the loss formula to the first sampled threshold crossings gives an approximate six-outcome mean of −86.47 points and 87,912 J. The three incomplete records preclude a definitive all-attempt average. A short, incomplete no-debris recording is also excluded. Damage logged after confirmed arrival is outside the competition score.

I compared configs by entity name and system nametag. Shuttle mass, inertia, initial pose and velocity, target state, field effects, jet configuration and scoring settings match. The no-debris variant removes 500 debris entities, disables radar and spawning, and explicitly permits guidance without radar. The two spectator references differ. All three local competition config files now select the team checkout. The completed clear flight stopped accelerating at **54.2% progress**, with **14.05 m/s** closing speed. The old 70% shutdown occurred later. Debris interrupted burns on the four successful competition flights, whose peak closing speeds averaged 11.40 m/s. Their lower fuel bills explain how some scored better than the faster clear run. One clear run gives no estimate of simulator variance.

### Propulsion and attitude

The final sim uses the no-mains CSV, the SpaceBuggy mass of 3388.7 kg and the installed jet allocator. It solves for minimum total jet activity for the requested linear and angular acceleration, then rescales on-times to the actuator tick. Pure-axis tests with zero torque and a 0.12 m/s² command gave these delivered accelerations. All six axes used approximately **4.32 kg per m/s** of useful velocity change.

| Body direction | +X, nose | −X | +Y | −Y | +Z | −Z |
|---|---:|---:|---:|---:|---:|---:|
| Delivered m/s² | 0.175 | 0.097 | 0.143 | 0.096 | 0.096 | 0.117 |

The actual launch target direction in body coordinates is approximately (−0.062, −0.041, +0.997). Holding that attitude and requesting translation along the target line gave 0.110 m/s² and 4.75 kg per m/s, owing to the mixture of axes. Pure +X delivered essentially the same acceleration for 0.04, 0.12 and 0.5 m/s² requests in the saturated allocator. Increasing command size alone gives little benefit there. In the clear-flight check, allocating more authority to turning once outward lift was established reduced the time-target fuel from 62.96 to 58.61 kg at roughly the same arrival time.

| Clear rigid-body experiment with score-based guidance | Arrival | Fuel | Score |
|---|---:|---:|---:|
| Hold the broadside departure attitude and allocate translation directly | 247.00 s | 30.80 kg | 57.52 |
| Align the nearby +Z axis for propulsion | 239.85 s | 30.51 kg | 59.35 |
| Turn to use +X, as implemented | 209.10 s | 36.60 kg | 58.58 |
| Start already +X aligned, a counterfactual | 181.65 s | 37.16 kg | 66.51 |

The +Z alternative gained 0.77 points on this clear benchmark but lost about 31 seconds and offered less acceleration authority. Its advantage across debris fields is unverified. I retained the proven +X handoff, idle coast attitude and damped turn controller. The new emergency maneuver can use an oblique body-jet allocation while holding attitude. All directions tested for that maneuver add forward velocity.

At 20 s the rotating bench had used 13.44 kg and reached 2.32 m/s, compared with 9.14 kg and 1.88 m/s when holding attitude. The 4.30 kg difference includes additional outward impulse and its interaction with torque. The real no-debris log first measured nose error below 10° at 24.38 s. About 39.4 s of its sampled flight had angular command on, including about 3.0 s with translation off. Intervals bounded by angular-only samples accounted for about 0.07 kg. The shared allocator and one-second telemetry prevent an exact split between rotation and translation fuel.

### Burn and route selection

For arrival at time T, fuel F and damage D, the installed scorer gives **S = 50 + 12000/T − 100F/75 − 200D/209200**. Arrival is position-only. An extra kilogram costs 1.333 points. Saving one second near 150 s earns about 0.533 points, while saving one near 200 s earns 0.300. An improvement from T₁ to T₂ pays for at most **9000(1/T₂ − 1/T₁) kg** at equal damage. Saving 200 → 150 s breaks even at 15 kg. One 10,000 J impact costs roughly 9.56 points in addition to its trajectory consequences. The failure formula replaces the time reward with a penalty proportional to remaining distance. [Competition scoring](https://github.com/SimDynamX/STU4-Solution/wiki/2.-mission-success,-failure,-and-scoring)

`AccelerationGuidance._score_cutoff` compares coasting through target entry with adding two seconds of useful forward impulse and then coasting. It integrates gravity, prices the impulse using measured kg per delivered m/s, and uses the full mission clock. If the current 3-D flyby misses, radial timing prices extra speed while the 3-D guidance still orders the necessary course correction. A 0.05-point restart margin limits cycling. Discretionary burns reserve roughly one m/s of correction fuel and a fuel-telemetry interval under the 75 kg reference. Collision response and recovery retain priority. There is no terminal velocity requirement or compulsory braking.

In the same clear allocator plant, fixed cutoffs of 7, 8, 9, 11 and 13 m/s scored **58.10, 58.87, 58.73, 55.68 and 49.77**. The adaptive result was within 0.29 points of the best tested fixed cutoff. The former 70% latch was hard-coded. The active policy coasts whenever the intercept and score support it, at any progress, and keeps radar, course recovery and avoidance active through arrival. The two-second comparison is a local estimate of whole-flight score. Global optimality across unknown future encounters is unproven.

`RoutePlanner.forecast` rolls the same policy through burns, coast and recovery to arrival. `Search.advance` ranks qualified alternatives by predicted mission score, with a straight maneuver as a tie-breaker. A partially improved but still threatened route competes on clearance first. The launch controller starts on a fresh sweep while subsequent observations refine transverse motion. Its existing direct, tilted and wait options remain available. I kept its clearance-first launch ranking because the recorded estimates did not support a universal waiting time or departure angle. In one 277-track replay at 6.7 s, direct departure and a three-second wait both qualified, scoring 40.4 and 37.8 respectively. At 10.9 s in another flight, the straight forecast had two unresolved tracks while a shallow alternative cleared the tested gates. These are snapshot predictions. Attitude cost is approximate in the strategic forecast and measured directly only in the allocator experiments.

### Prediction, tracking and collisions

Four pre-arrival impacts on the recorded time-target controller had continuous negative predicted hull clearance for approximately **9.6, 33.2, 19.4 and 36.4 seconds** before first recorded damage. The affected flights illustrate two different failures.

- At 162.2 s the 70% latch disabled maneuvering. The closest modeled hull gap turned negative at 182.4 s, and an impact at 192.0 s registered 110,934 J. A new closing contact had only one association during much of the approach. Its previous eight-second horizon reduced early warning. The revised terminal predictor carries a closing first-hit contact through a 16-second horizon until association or expiry, with growing covariance. Replaying the recorded radar from the old path gave a negative predicted hull gap at 177.0 s, about five seconds earlier. Some subsequent states still had no clear candidate.
- Another flight showed negative hull clearance continuously from 117.1 s until a loss-level impact at 150.2 s, well before the 70% mark. Waiting and rear-only maneuvers could exhaust the available steering time. The new `VECTOR_ESCAPE` compares forward-oblique allocations through the existing body-jet interface only when coasting and the available rear-burn fallback both threaten the physical hull margin. It checks fresh evidence, spin, fuel-independent safety, predicted delivered acceleration from 0.05 to 0.25 m/s² and the worst pass across tracks. It requires at least one metre of predicted hull-gap improvement. An uncertainty overlap can persist. The recorded-state alternative at 140.7 s improved the estimated hull gap from −12.6 to −9.6 m and remained hazardous.

Radar supplies center, surface range, radial speed, diameter and measurement time. Radial speed from one observation cannot identify transverse velocity. Timestamped association and gravity propagation remain essential. The public spawner applies gravity and a small startup velocity perturbation to saved debris. Its optional targeting adjustment is disabled in the inspected code. The flight controller uses permitted radar and vehicle/target state. Track number changes, sparse revisits and inconsistent thrust delivery can degrade a forecast. The logs cannot identify the true impacting rock or separate all measurement error from model error. Long-range planning covers up to 120 s, excludes very diffuse predictions from exact ranking, and can be interrupted by immediate avoidance. [Competitor API](https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api) · [Competition rules](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218)

`RadarField.assess` retains all eligible tracks and determines closest approach on each candidate path segment. It bounds clearance using radius, covariance, relative motion and trajectory curvature before projecting covariance along the separation direction. It skips an exact projection only if the segment cannot be threatened and cannot set the least clearance. Unresolved and horizon-approaching sets are retained. Across 27 paths from recorded fields with 277–499 reliable tracks, closest-track IDs, threat sets, unresolved sets and numerical clearances agreed with the previous calculation. Median time for the assessment alone was **2.375 → 1.072 ms** on this machine. This measures one part of the planner.

### Paired validation and release decision

| Matched offline tests | Saved time-target controller | Score candidate |
|---|---:|---:|
| Clear flight, installed allocator | 149.95 s, 58.61 kg, 51.88 points | 209.10 s, 36.60 kg, 58.58 points |
| Four allocator debris flights, mean | 183.16 s, 55.88 kg, 41.69 points | 198.15 s, 45.85 kg, 50.60 points |
| Four allocator debris flights, clean arrivals and worst hull gap | 4/4, 0.97 m | 4/4, 7.79 m |
| Seven point-mass debris layouts × three matched radar seeds | 17/21 contact-free arrivals | 21/21 contact-free arrivals |
| The 17 point-mass flights clean in both, mean | 154.53 s, 59.69 kg, 48.18 points | 191.54 s, 39.85 kg, 60.50 points |

All point-mass scenarios reached the target sphere. The candidate's smallest modeled gap was 2.63 m. The three centered head-on failures and one crossing failure on the earlier policy all cleared in these tests. The point plant starts nose-aligned, approximates slew and charges ideal translation fuel. The allocator plant starts broadside, integrates rigid-body torque and charges the installed jet on-times. Its debris trials use two seeds per scene, asynchronous noisy radar, spherical collision geometry and a 0.05 s integration step. Neither plant models contact damage or launchpad physics. Score averages exclude contacts when damage would be unknown. Recorded-radar checks use the old flown trajectory and a fresh planner at each snapshot. They cannot establish that an alternative would have prevented a recorded impact.

**Release status.** The candidate is configured for Space Teams and passed 166 automated tests. Live validation of the exact candidate in the full debris field remains necessary before calling it submission-proven. In the next trials record official score and terminal outcome for every attempt, first-damage time and energy, fuel, arrival time, and the last `avoidance_strategy`, `baseline_hull_gap` and `hull_gap` before damage. For a hard deadline, the preserved source allows a direct comparison. A faster finish alone was an unreliable selection criterion in these tests.

| File | Current change |
|---|---|
| `Controller.py` | Enable score-based cruise, hand off actionable oblique impulses through the existing allocator, suppress a redundant turn during the impulse, expose score decisions. |
| `Guidance.py` | Add score-based burn/coast choice, dynamic terminal control, terminal first-hit lookahead and conservative emergency vector assessment. The previous deadline policy remains available for testing. |
| `RoutePlanner.py` | Use the active cutoff throughout rollouts, rank clear routes by score and prune exact covariance calculations with conservative bounds. |
| Local `FINAL_SUBMISSION.simconf` | Restore the controller source entry to the team checkout. The competition and no-debris test configurations already select that checkout. |
| `tests/check_launch_fuel.py`, `tests/compare_routes.py`, `tests/offline_scenarios.py`, `tests/audit_departure.py` | Reproduce saved-source comparisons, orientation alternatives, allocator and point-plant flights and recorded-state candidate assessments. |
| Guidance, controller-loop, forward-avoidance and route tests | Cover score timing, post-cutoff course/avoidance authority, new-contact warning, vector execution, the centered encounter and consistency of the risk filter. |

Run the checks from the team checkout with the Space Teams Python environment. Use a fresh process for each allocator configuration so it loads the chosen sources.

```powershell
python -B -m unittest discover -s tests -q
python -B tests/check_launch_fuel.py --through-arrival --before-score
python -B tests/check_launch_fuel.py --through-arrival
python -B tests/check_launch_fuel.py --through-arrival --attitude hold
python -B tests/check_launch_fuel.py --through-arrival --attitude top
python -B tests/check_launch_fuel.py --through-arrival --scene crossings --seed 23 --before-score
python -B tests/check_launch_fuel.py --through-arrival --scene crossings --seed 23
python -B tests/compare_routes.py --before-score --seed 7
python -B tests/compare_routes.py --before-score --seed 23
python -B tests/compare_routes.py --before-score --seed 41
python -B tests/audit_departure.py diagnostics/20261120T011731_373688Z --candidate --times 177 180 185
```

## Phase 1 — Baseline and source audit

The flight baseline is commit `b9dc3d2` on `feature/trajectory-techniques`. The SHA-256 hashes of Controller.py, Guidance.py, and RoutePlanner.py match all five latest flight recordings. The owner’s score-screen results average **37.92 points**, **203.83 seconds**, and **53.39 kg**, with five arrivals and zero recorded damage. The first recorded `ARRIVED` samples agree with the reported fuel and damage. Their timestamps are slightly later than the scorer’s, so the table uses the reported times and scores.

| Trial | Recording ID (UTC, shortened) | Score | Time (s) | Fuel (kg) | Closing speed near 20 s (m/s) | First normal rear burn after handoff (s) | Avoidance coast, 20–70 s (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 | 110522 | 43.9 | 181.58 | 54.11 | 1.98 | 28.4 | 7.2 |
| 2 | 111103 | 34.2 | 219.14 | 52.93 | 1.98 | 63.9 | 44.9 |
| 3 | 111745 | 34.4 | 210.52 | 54.44 | 1.99 | 71.5 | 39.9 |
| 4 | 112153 | 37.3 | 206.04 | 53.24 | 1.97 | 49.7 | 26.8 |
| 5 | 112855 | 39.8 | 201.87 | 52.23 | 1.96 | 58.3 | 24.8 |

Avoidance coast sums sampled intervals labeled `AVOID` with translation off. Angular jets sometimes fired during those intervals. Samples are about one second apart. Trials 3–5 had brief forward avoidance burns before normal acceleration resumed. Trial 1 spent about six seconds aligning under an acceleration request before rear thrust began. Across flights the last nose error sampled before 20 s was about 44–48°. Early waiting therefore combines hazard responses and the time needed to point the rear burn. There were zero emergency-mode samples and zero additional measured fuel after the 70% cutoff in all five flights. No proposed strategic route was logged as pending before 20 s.

### Scoring and simulation configuration

`FINAL_SUBMISSION.simconf` points to the TAMU SEDS controller, the official jet allocator and radar, and `STU4-Sim/Systems/Scoring.py`. Its scoring inputs are 120 s nominal time, 75 kg nominal fuel, 209,200 J damage threshold, and 50 m success radius. The scorer accepts a flyby and calculates the winning score as `50 + 100 × (120/t − f/75 − 2D/209200)`. Its failure formula uses remaining distance fraction in place of the time reward. The guidance model mirrors the winning formula. The strategic forecast assumes successful arrival with zero damage and uses estimated fuel cost per unit of delivered impulse.

| Time saved from a 204 s arrival | Points gained with equal fuel | Break-even extra fuel |
|---|---:|---:|
| 5 s | 1.48 | 1.11 kg |
| 10 s | 3.03 | 2.27 kg |
| 20 s | 6.39 | 4.80 kg |
| 30 s | 10.14 | 7.61 kg |

These are savings in **arrival time**. Cutting a 20 s wait can save a different amount at arrival if the departure also changes velocity, the cruise schedule, or the recovery burns. A few kilograms of turning and correction fuel can erase the benefit of a shorter wait. The official rules limit observation to permitted sensor interfaces. [Competition rules](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) · [Competitor API](https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api)

### Sensor processing and collision prediction

| Capability | Status and code evidence |
|---|---|
| Position and velocity | **Implemented.** `Controller.py:119` reads asynchronous radar records in Phobos axes. `Guidance.py:95` reconstructs the shuttle state at the measurement time. The six-state position/velocity Kalman filter at `Guidance.py:121` and `Guidance.py:161` estimates transverse motion from repeated positions. The first return initializes only the radial component of rock-relative velocity and assigns broad transverse uncertainty. Updates fuse center position, independently noisy surface range plus estimated radius, and radial velocity. |
| Association | **Implemented.** The tracker rejects invalid, stale, repeated, and inconsistent returns. It gates predicted position, size, and radial velocity, then greedily assigns one sample per track and one track per sample within the batch. Ambiguous assignments inflate velocity uncertainty, skip the range and radial updates, and reset the trusted-hit count. Strategic planning requires at least two hits and two reliable associations. Local track numbers are separate from radar sort order. |
| Depth and radar fusion | **Partially implemented.** The tracker combines the radar’s own measurement channels. Camera handles and helpers exist. The qualification loop contains no camera capture or depth update, so the five runs used radar alone. |
| Closest approach under thrust and coast | **Implemented.** `Guidance.py:656` propagates finite burns and then unpowered motion. `Guidance.py:686` compares separation at 0.5 s samples through up to 30 s, including possible paths during the turn. `RoutePlanner.py:181` compares target-relative trajectories through up to 120 s and interpolates closest approach within 3 s segments. A nearby receding rock outside the clearance envelope can be ignored. |
| Safety margin | **Implemented, calibration unverified.** Both planners count 1.15 times the tracked debris radius, a 2.5 m estimated shuttle radius, a 3 m fixed margin, and 2.5 times propagated position uncertainty projected onto separation. Tactical checks add a relative-speed allowance for inter-sample motion. Strategic checks add a curvature allowance and flag wide forecasts as unresolved. Thus a negative margin can coexist with a positive predicted hull gap. `physical_time` is also a conservative sampled estimate, never a measured collision. |
| Updates and confidence | **Implemented with limits.** The controller requests a tactical plan every 0.25 simulated seconds and incorporates fresh timestamped observations when available. The strategy layer reviews routes roughly every two seconds when eligible, evaluates two alternatives per update on a frozen field, and rechecks the winning plan with the latest state and measurements. Stale radar inhibits thrust. The review pauses during active tactical avoidance, when an alternate departure time could be most valuable. |
| Calibration | **Unverified.** Five successful flights support conservative operation. They do not establish the optimal radius or uncertainty multiplier. The 25 m long-range confidence gate, estimated hull dimensions, local gravity approximation, and unmodeled attitude/allocator response affect projected clearance. Flight logs contain estimated misses, so actual minimum separation cannot be inferred from them. |

### What the controller optimizes now

- **Nominal guidance** flies along the departure line during launch, uses gravity-affected ballistic arrival geometry, builds speed before coasting, and schedules limited forward corrections and speed top-ups. The 9 versus 10 m/s cruise choice and optional top-ups already use the winning score formula. The 70% progress latch independently shuts off translation and rotation. [Guidance.py](Guidance.py#L434) · [Controller.py](Controller.py#L480)
- **Strategic routing** forecasts full arrival time, translation fuel including recovery, and clearance for 3 or 6 s straight waits, earlier burns from cruise, an arrival offset, and shallow turns. It takes the direct corridor when clear and ranks a fully clear alternative above a partial improvement. Among fully clear alternatives it ranks a straight timing change first, then score. If the planner can only improve a predicted close pass, it ranks margin improvement ahead of score and remains subject to tactical checks. Path length and angular jet fuel are absent from the objective. [RoutePlanner.py](RoutePlanner.py#L73) · [candidate ranking](RoutePlanner.py#L275) · [candidate generation](RoutePlanner.py#L412)
- **Tactical avoidance** preserves a clear nominal trajectory, then prefers a clear coast at sufficient closing speed. It retains a feasible short burn when possible. Otherwise it searches straight and shallow rear burns, favors straight flight, then minimizes impulse and penalizes lateral cleanup. It does not compare expected mission score for coast versus burn. If all options overlap the uncertainty envelope, the fallback favors later predicted contact then clearance. `COAST_LIMITED_CLEARANCE` means coasting was the fallback and still carried forecast risk. It is distinct from a certified `COAST`. [Guidance.py](Guidance.py#L792)

### Opportunities, risks, and baseline checks

1. **Evaluate departure timing during a wait.** Existing strategic waits are 3 or 6 s and are planned before handoff or while tactical avoidance is inactive. Log or replay longer straight waits alongside small forward burns and shallow turns using the same measured state. Score only arrival forecasts that clear all eligible tracks. Initial surveys should distinguish negative margin, unresolved forecasts, and positive evidence of clearance. The risk is that a shorter apparent wait can just move the encounter beyond credible prediction. Revalidation must remain part of execution.
2. **Investigate alignment latency before allowing a launch bias.** The ~45° nose error at handoff makes the 35° tactical alignment gate relevant. A small preplanned tilt is potentially cheap if a repeatable track shows it can be built during the launch attitude maneuver. All five flights exercised the straight launch. Evidence for early steering is currently insufficient. Changing attitude during broadside lift would change allocator demand and may delay useful forward thrust or make clear launches worse.
3. **Compare score inside the existing safety envelope.** The established formula and forecast already provide the ingredients. The first question is whether a tilted option can beat a clear straight wait by a margin large enough to cover angular fuel and model error. A ranking change has little value if the best choice is infeasible at the nose angle or repeatedly canceled by tactical priority. Keep verified coast, rear-only avoidance, and current margins during experiments.
4. **Measure before reducing safety margins or adding a second sensor.** Dense fields can yield duplicate tracks under ambiguous association, and tactical and strategic rollouts differ in horizon and conservatism. Recorded planner samples peaked at roughly 142–181 ms. More search in the live control loop may add latency. Replay raw radar with logged ownship state before increasing runtime search or changing the filter.

The original isolated test suite ran 146 tests and had three failures and one error, all in the fake controller loop. Those tests still expected emergency lateral commands, direct lateral course trims, or the removed metered-launch flag. The actual allocator harness is different from the point-mass scenario suite, which starts aligned, models ideal translation fuel and simplified attitude, and supplies repeatable synthetic radar noise. Its seed-7 centered head-on scene reproduces a collision in the current simplified model even though all five live trials were clear. This is a useful stress test and does not establish a corresponding live collision. The local debris spawner has a fixed seed. Radar noise, simulator scheduling, and the variation visible in these five flights still require repeated full runs for a score comparison.

## Phase 2 — Recorded-radar comparison

[audit_departure.py](tests/audit_departure.py) uses the current guidance, tracker, score, and clearance routines offline. It validates the recording's flight-source hashes, reconstructs asteroid tracks from accepted radar samples in receipt order, and starts each alternative from the same logged shuttle and target states. It compares the direct burn, 3, 6, 12, and 20 s delays, and 6° turns in four directions. It reports projected arrival time, fuel still to spend, total fuel, score, tactical and strategic clearance, unresolved tracks, and whether the immediate command is executable. The comparison requires a finite arrival prediction, fresh radar, positive clearance in both forecasts, and no unresolved tracks before marking an option eligible. This last check is stricter than the live planner's treatment of unchanged unresolved tracks.

Across 20 snapshots near 18, 21, 30, and 40 seconds, **0 of 180 forecasts qualified**. In the 219.14 s flight at 21.2 s, a direct burn projected arrival at 179.2 s with a 33.9 point score, but its tactical clearance was −19.9 m and the nose could not deliver the burn. A 20 s wait improved the immediate margin to −12.7 m. Its projected arrival was 199.0 s and its modeled score was 27.0. Both options carried predicted risk, so the score comparison alone could not justify releasing the wait. In the fastest flight at 21.1 s, four tilt forecasts had positive margins for the tracks the long-range model could resolve. Each had three unresolved tracks, an immediate burn outside the alignment gate, and a lower score estimate than the direct schedule. The tested angles begin at or after 20 s. The results provide no evidence for changing attitude during the preceding launch burn.

This comparison uses interpolated 1 s ownship samples, the accepted radar observations recorded in flight, local gravity, and estimated burn and turn authority. It does not restore every internal latch, use future measurements to change plans, or represent a closed-loop trajectory. An ineligible snapshot indicates insufficient evidence to execute that forecast. It does not prove the flown coast was the only feasible option.

To reproduce the comparison from the team solution directory with Space Teams Python:

```powershell
$recordings = Get-ChildItem diagnostics -Directory | Sort-Object Name |
    Select-Object -Last 5 -ExpandProperty FullName
python -B tests/audit_departure.py @recordings
```

## Phase 3 — Verification and submission decision

| Check | Result |
|---|---|
| Source and scenario | All three guidance source hashes still match the five live recordings. `FINAL_SUBMISSION.simconf` still selects that controller and the official scoring system. |
| Automated tests | Baseline suite 142 of 146 passed. With test fixtures aligned to the flown version and a cutoff regression check, 147 of 147 passed. The fixtures now verify launch direction and turn bounds, risk reporting during launch, forward avoidance canceling a trim, and both actuator commands going to zero at the cutoff. [test_controller_loop.py](tests/test_controller_loop.py) |
| Recorded radar comparison | 180 forecasts. No earlier departure qualified for both clearance checks and command feasibility. [audit_departure.py](tests/audit_departure.py) |
| Jet allocator bench | The current controller arrived in 183.60 s at 43.25 kg in an ideal rigid-body model with installed allocator math and no debris or launchpad contact. At 20 s it reached 2.38 m/s of closing speed and had used 13.23 kg. |
| Synthetic debris stress | Used identical initial geometry and measurement-noise seeds 7 and 23 per pair. Both versions reached the arrival sphere in all ten scenarios. The older `pre_forward_only` snapshot avoided modeled contact in all ten. The flown version had modeled contact in both centered head-on cases (minimum modeled gaps of −2.64 and −1.96 m) and the seed-23 crossing case (−3.89 m). Its no-damage score estimates in those cases are invalid as performance comparisons. |
| Space Teams evidence | The five existing runs all completed with zero damage. New flight behavior and a live score increase were not established. The measured minimum hull clearance is unavailable. |

**Submission recommendation:** use the five-run baseline `b9dc3d2`. The three changes in this audit are the report, the offline comparison tool, and updated integration tests. A flight change would require repeatable evidence that it can improve score while retaining clearance, particularly in centered encounters. The most relevant measurements for any future departure experiment are the time and closing speed at the first sustained rear burn, duration of verified versus limited-clearance coasting, forecast margin and uncertainty at the decision, and official time, fuel, damage, and score.
