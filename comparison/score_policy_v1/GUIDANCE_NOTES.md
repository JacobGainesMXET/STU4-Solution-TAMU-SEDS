# Predictive guidance and control

The active entry point remains `Controller.py`. Its protected connection code, helpers,
and existing radar utilities are preserved. `Guidance.py` contains simulator-independent
control mathematics; `FlightRecorder.py` writes local diagnostics. No simulation,
scoring, physics, thruster data, timescale, or sim configuration is changed.

## Findings that motivated these changes

- The old translation loop tracked an 8 m/s velocity toward the target. It had no
  persistent debris identities or transverse-motion estimate, and ignored radar age.
- `Radar.py` updates one of 500 contacts per 120 Hz iteration and republishes after
  every 50 contacts. A contact can therefore be roughly four seconds old even when
  the radar is working. Old shuttle-relative positions cannot be compared directly
  to current ship state or differenced into velocities.
- `JetSelectionActuator.py` solves for jet activity, then rescales activity to a
  time step. Small nonzero acceleration requests can still produce substantial
  physical firings. Smoothing command amplitude alone does not solve this.
- An offline calculation using the actual jet geometry and inertia gave approximately
  0.096–0.175 m/s² of pure-axis acceleration for minimum-total-activity solutions.
  This is different from the 0.5 m/s² **command clamp**, and mixed torque/translation
  changes the available acceleration. These calculations used the installed SciPy
  `highs` solver; the simulator uses `revised simplex`, so the selected jet combination
  and achievable acceleration can differ. Online response measurement is essential.
- The local scoring system accepts entry into a 50 m sphere, without velocity matching:

  `score = 50 + 100 * (120 / time - fuel / 75 - 2 * damage / 209200)`

  The implementation also preserves the scorer's zero-time and nonnegative-fuel
  conventions. There is no reason to perform a full stop before entering the region.

Relevant upstream references: [competition rules](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218),
[vehicle/sensor model](https://github.com/SimDynamX/STU4-Solution/wiki/1.-vehicle-specs-and-model-details),
[permitted API](https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api),
[scoring](https://github.com/SimDynamX/STU4-Solution/wiki/2.-mission-success,-failure,-and-scoring).
The installed source and current `COMPETITION_SIM` were inspected as well.

## How the controller works

1. **Track measurements at their capture times.** Own-ship state is saved in a
   20 s history. Each radar sample is converted to an estimated Phobos-frame obstacle
   position using the ship's position at that sample's timestamp. Duplicate samples
   do not increase confidence. Finite-value, age, diameter, and centre/surface checks
   reject invalid measurements. The documented radar epoch is assumed; implausible
   future timestamps are rejected rather than silently realigned.
2. **Associate and estimate motion.** Gated one-to-one matching uses predicted
   position, diameter, and radial velocity, never the distance-sorted array index.
   A six-state Kalman filter estimates position and velocity. Radar range rate is a
   scalar velocity constraint; transverse velocity must be learned from successive
   positions. Ambiguous associations increase uncertainty. Repeated/missing data
   cannot falsely create confirmation. Tracks expire after 12 s without an update.
3. **Choose a useful burn, then coast.** A small search compares finite burns followed
   by ballistic coasting using the actual score. It includes target-relative velocity
   and estimated external acceleration. It uses measured propellant per delivered
   impulse and estimates burn authority from low-rotation firing windows. The burn
   authority retains 5% reserve, with an upper bound of 0.16 m/s². This is a tunable,
   local prediction model, not a proof of globally optimal score. A 10 m/s planning
   ceiling limits aggressive speed choices; it is not a competition rule.
4. **Estimate gravity without cancelling it continuously.** Phobos central gravity
   initializes the estimate. Velocity changes during coast windows estimate the
   remaining external acceleration. Large acceleration innovations are rejected as
   potential impacts. Guidance predicts ballistic motion with this estimate and
   corrects a projected arrival miss only when necessary. Gravity estimation does
   not filter the simulator's own-ship truth state.
5. **Predict encounters before choosing an avoidance burn.** For each candidate,
   predict an 8 s burn followed by coasting over up to 30 s. Propagate position and
   velocity covariance; add 2.5 sigma directional uncertainty, 15% debris-radius
   allowance, the existing 2.5 m shuttle-radius estimate, and 3 m clearance. A speed
   margin covers the 0.5 s sampling intervals. Unconfirmed contacts start with broad
   transverse uncertainty and an 8 s horizon; their horizon grows with observations.
   This is a constant/common-gravity approximation with process noise for departures,
   not exact asteroid dynamics or a guaranteed collision-free trajectory.
6. **Give safety priority.** All candidate paths are checked against all potentially
   relevant tracks. Full lateral burns and braking are available independently of
   goal thrust. Safe candidates minimize correction, fuel, and command changes. If
   none is safe, prioritize delaying the earliest predicted overlap, then clearance.
   The previous maneuver gets a continuity preference. Nominal guidance must remain
   comfortably safe for 1.5 s before avoidance releases. An imminent threat bypasses
   the ordinary pulse start delay. The broad-phase distance bound covers every
   candidate, so distant safe rocks can be excluded from repeated evaluations.
7. **Measure actual impulse.** Both linear and angular commands use accumulated
   requested-minus-delivered velocity changes to produce short firing intervals.
   Coast phases clear thruster requests. Small residual impulse debts decay during
   zero demand. This addresses the allocator's scaling without replacing it or
   accessing individual jet controls. New velocity/rate feedback closes the loop.
8. **Preserve the damped attitude controller.** The previous tested perpendicular-axis
   construction, rotation-vector error, angular-rate feedback, and 3°/s desired-rate
   cap remain. The reference slews at 2°/s toward the *required acceleration*, rather
   than the destination. It holds during coast. Sideways emergency thrust does not
   wait for nose alignment. Recovery from unusually large spin may use 0.03 rad/s²
   within the simulator's angular command limit.
9. **Fly through the success region.** A fixed 30 m offset aims away from the target
   centre while preserving a terminal aim region inside the 50 m success sphere.
   It reduces centre-line interception risk; target hull clearance is not certified.
   Entry latches completion and clears commands, preventing a reversal after flying
   past the target. There is no terminal zero-speed requirement in this controller.

Until sufficient contacts are confirmed, the controller prioritizes acquiring motion
estimates. A stale/disconnected radar inhibits mission acceleration and requests
gentle braking. State non-finiteness clears commands. A recording failure disables
recording without disabling guidance. No debris entity is queried at runtime.

## Evidence and its limits

Run the offline checks from this repository using the simulator's Python:

```powershell
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B -m unittest discover -s tests -v
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/offline_scenarios.py --output comparison/offline_results.json
```

Tests cover attitude geometry/direction/large angles, pulsed attitude dynamics,
delayed/reordered/repeated/noisy or missing observations, accelerating own-ship motion,
invalid/outlier samples, covariance growth, crossing encounters, safe near passes,
avoidance priority and release, score/fuel calibration, pulse delivery, and the
controller's API wiring using a fake simulator. No test imports the live controller
and connects to Space Teams.

The scenario report compares the previous and revised **translation** controllers
in a simplified point-mass plant, then exercises noisy radar with approaching and
crossing debris. The plant assumes constant gravity, 0.14 m/s² firing authority,
instantaneous attitude and ideal impulse fuel. It excludes real thruster allocation,
rotational fuel, impacts, camera measurements, and simulator scheduling. Its scores
and clearances are mathematical evidence only, not predicted competition results.
Actual mission reliability and score must be measured over multiple Space Teams runs.

The completed validation run passed **47 offline checks**. In the simplified clear
path experiment, the previous translation controller used 37.49 kg in 188.05 s
(modeled score 63.83); the revised controller used 32.92 kg in 207.45 s (63.95).
That is about 12% less ideal fuel and 10% more time, with effectively unchanged
modeled score. Rotational fuel is absent from both results. This comparison is a
check against optimizing fuel at the expense of score, not evidence of an actual
competition improvement. See `comparison/offline_results.json`.

Six approaching/crossing runs across three radar-noise seeds completed without
synthetic collisions. Their minimum true surface gaps were 11.15–14.92 m in this
particular point-mass geometry. The additional results are preserved in
`comparison/offline_noise_checks.json`; this small case set is not a reliability
guarantee or a substitute for repeated simulator runs.

A 501-track synthetic hazardous-scene timing check evaluated 36 candidate commands
in about 47 ms on this machine after broad-phase filtering. A deliberately extreme
500-*new*-sample update took about 225 ms; the live radar normally publishes smaller
batches. Check the recorded `planner_ms` on the actual run, especially during startup
and dense encounters. Avoidance candidates describe a finite future action, so the
planner must continue receiving fresh observations and replanning.

The main remaining uncertainties are real combined translation/rotation authority,
sensor clock alignment, association through dense close passes, the chosen process
noise and uncertainty margins, hull radii, impacts, and the accuracy of the gravity
and burn/coast model. These parameters are starting values, not new sensor guarantees.

## Compare versions and inspect the next run

The previous working controller is preserved in
`comparison/Controller_before_predictive.py`; its original tests are saved as text.
The current configuration still loads the root `Controller.py`. To A/B manually,
stop the simulator, save the current controller, temporarily copy the comparison
snapshot over the root controller, run the same competition scenario, and restore
the saved current controller. Use several runs per version; one clear run is not a
reliability estimate. Do not run two controllers against the same shuttle.

Each revised run creates `diagnostics/<UTC run stamp>/` containing:

- `telemetry.csv`: one row per simulated second, including state, mode, target motion,
  estimated gravity, fuel, damage, ETA, burn benefit, calibrated authority/fuel cost,
  pulse states/switch counts, coast fraction, worst predicted encounter, uncertainty,
  observation age and planner time. `score_if_arrived` is hypothetical until arrival.
- `radar.csv`: accepted unique new radar samples with receive and measurement times,
  sufficient for offline tracking analysis alongside the own-ship telemetry. Rejected
  measurements appear in counters, not in this file. This is not debris truth.
- `model.json`: starting model parameters and source hashes for reproducibility.

These diagnostic files stay on disk and are ignored by Git. They are not uploaded.
The on-screen `Guidance` log gives a short summary each simulated second.

Prioritize **zero-damage completion rate and official final score**, then time and
fuel together. Inspect minimum predicted clearance versus damage events, observation
age, confirmation/ambiguity/rejection counts, time in `EMERGENCY`/`SENSOR_BRAKE`, planner
latency, spin, firing/coasting fraction, and fuel per delivered velocity change.
Persistent emergency mode, depleted confirmation, or repeated side changes indicate
that prediction/association/model errors need attention before pursuing higher speed.
Send the new diagnostics folder with the matching Space Teams run logs for calibration;
the older logs contain useful status but lack the radar history needed for replay.
