# Predictive burn-coast-burn guidance

## Current controller — straight launch and timing-based avoidance

The five November 19 flight logs from 09:33–09:46 UTC match the reported arrival
fuel and the source hashes in `comparison/pre_straight_launch/`. Mean official time
was 221.70 s, mean fuel was 80.09 kg and mean score was −2.62, with zero damage.
Interpolating the fuel gauge at 20 s gives 11.62, 11.39, 9.33, 6.36 and 9.53 kg.
Four flights logged a direct strategic route throughout. The 64.17 kg flight logged
a strategic turn. Summing fuel differences against the preceding telemetry mode
attributes roughly 19.6–33.3 kg per flight to emergency maneuvers. This attribution
has the one-second sampling error of the recorder.

- The default aim is the target centre. Ballistic guidance still accepts entry
  anywhere inside the arrival corridor. During a broadside launch the controller
  commands useful outward acceleration and a damped turn. It converts both demands
  to impulse for the installed jet allocator's tick while the nose error exceeds
  35 degrees in the first 20 s. Calibrated launch windows do not update the estimate
  of maximum thrust authority.
- Radar and tactical protection operate from liftoff. Strategic candidates include
  the direct launch prefix in their predictions. Their proposed burns, turns and
  aim-point changes begin at or after 20 s. A pending choice must clear the newest
  radar and measured state at handoff. Tactical intervention discards that choice.
- When the direct path is threatened and outward speed exceeds 0.5 m/s, tactical
  avoidance first checks whether the shuttle can coast through the encounter.
  Straight forward burns take priority over shallow turns. A stronger transverse
  escape is available when those choices fail. A course trim can redirect velocity
  while preserving its magnitude. Collision clearance retains priority throughout.
- Intentional translational braking is permitted only from 20 s up to 28 s, one
  existing tactical burn interval after launch planning. Every sampled nonbraking
  option must predict hull contact within eight seconds and the short reverse burn
  must clear the safety margin. The actuator checks the time window on every tick.
  Radar loss or a planner fault requests an unpowered coast. Jet selection for
  attitude control remains with the simulator.
- A 0.3 m/s closing-speed deficit triggers evaluation of a finite top-up. Arrival
  geometry, nose alignment, time since the last burn, and estimated score gain
  determine whether it fires. A clear final approach remains unpowered.

### Checks for this revision

Run these commands using the installed Space Teams Python from the solution directory.

```text
python -B -m unittest discover -s tests -q
python -B tests/compare_routes.py --as-flown --seed 7
python -B tests/compare_routes.py --as-flown --seed 23
python -B tests/check_launch_fuel.py --previous --through-arrival
python -B tests/check_launch_fuel.py --through-arrival
```

The clear-flight allocator check starts with a 93.5-degree nose error and uses the
installed jet layout and allocation math with an ideal rigid body. Previous and
revised fuel at 20 s were 13.21 and 12.25 kg, with nose errors of 37 and 13 degrees.
At 3 m/s the totals were 17.11 and 16.73 kg, reached at 26.55 and 29.75 s.
At arrival they were 47.17 and 46.61 kg in 187.6 and 188.7 s. This bench excludes
launchpad contact and asteroids. The separate point-mass obstacle comparisons
exercise noisy radar and avoidance with simplified attitude and propulsion.

All 145 automated tests passed. Both versions arrived collision-free in the
five point-mass geometries on seeds 7 and 23. Revised minus as-flown results
are shown below. Positive score change and negative fuel change are favorable.

| Scene | Score change, seed 7 | Fuel change (kg), seed 7 | Score change, seed 23 | Fuel change (kg), seed 23 |
|---|---:|---:|---:|---:|
| Clear | +0.49 | 0.00 | +0.49 | 0.00 |
| Head-on | −2.69 | +1.24 | −3.65 | +1.61 |
| Single crossing | +8.55 | −5.84 | +2.82 | −3.05 |
| Two crossings | +3.04 | −4.23 | +3.91 | −3.30 |
| Two slow drifters | +0.24 | +0.04 | +6.13 | −6.51 |

For the next live series, compare fuel and nose error at 20 s, time and fuel at
3 m/s, repeated emergency updates, total fuel, time, damage and official score.
`pending_route`, `metered_launch`, `avoidance_strategy` and `brake_authorized`
identify the new behaviors in `telemetry.csv`. A near head-on encounter is the
most important tradeoff to measure alongside crossing debris.

## Previous controller — early radar route planning

Launch, attitude damping and radar acquisition run together. After unambiguous repeat
observations, the controller can compare flight paths over the next 120 seconds.
It checks for immediate threats every 0.25 simulated seconds. The long-range
review normally runs every two seconds during acceleration and clear coasting.
Each update evaluates at most two queued alternatives on a common radar snapshot.
The winner must still clear the latest observations before the controller flies it.

- **Flight prediction.** `RoutePlanner.py` rolls a private copy of the existing burn,
  coast and trim guidance forward from the measured target-relative state. It uses
  local gravity, measured burn authority and the learned fuel cost per unit of
  velocity change. Each candidate is priced through predicted entry into the
  50 m arrival sphere, including recovery burns. Estimates assume zero damage
  on a successful path. Angular fuel, future radar discoveries and thruster
  allocation introduce differences from the official flight score.
- **Route selection.** The direct route keeps priority while clear. When threatened,
  the planner tries a shorter or longer launch burn, a different aim point inside
  the arrival sphere, or a shallow turn sharing the main burn. It compares all
  reliable tracks on the same timeline. A retained route has a clearance and score
  margin against switching sides. Deliberate steering uses rear translation with
  damped attitude control and a forecast allowance for turn delay. The existing
  tactical planner can interrupt any route, including the final coast.
- **Cruise selection.** Launch starts with a 10 m/s closing-speed target. Early
  guidance may choose 9 m/s when its projected path clears the observed threats
  and improves estimated score by over one point or clears a blocked 10 m/s path.
  Before the lower cutoff, a new credible crossing can restore 10 m/s when the
  faster schedule clears it. A safe ballistic arrival needs no braking burn.
- **Tracking and uncertainty.** The existing six-state Kalman filter associates
  timestamped position and range-rate observations across scans using shuttle
  motion at measurement time. The filter now also uses the independently noisy
  surface range plus half the estimated diameter. Ambiguous associations need two
  further unambiguous updates to regain strategic eligibility. Each path is checked
  between samples against rock and hull radii, a clearance allowance, and a
  2.5-sigma margin along the direction of closest separation. Diffuse predictions
  shorten the useful horizon. A route cannot clear a known conflict by leaving
  that rock still approaching at the end of the forecast. These margins have
  yet to be calibrated as a flight-wide collision probability.
- **Camera evaluation.** The installed SDK exposes timestamped float32 depth
  frames in centimetres. The available flight recordings contain radar and
  telemetry. Camera fusion needs paired depth frames and radar observations to
  validate camera pose, depth along a ray versus the optical axis, invalid pixels,
  association error and capture latency. The flight loop currently uses radar
  ranging. RGB alone supplies no measured range. The official example also
  documents the cost of blocking camera capture. [Competitor Python API](https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api)

### Paired offline flights

Each row uses the same initial conditions and noise seed across all three controllers.
All 15 flights reached the target without hull contact. Scores use the competition
formula with point-mass dynamics and ideal impulse fuel. The minimum gap is the
simulated hull-to-rock gap, available only to the offline evaluator.

| Scene | Previous score at 10 m/s | Previous score at 9 m/s | Revised score | Revised time (s) | Revised fuel (kg) | Revised minimum gap (m) |
|---|---:|---:|---:|---:|---:|---:|
| Clear | 63.69 | 64.85 | 64.85 | 176.20 | 39.94 | — |
| Head-on | 58.55 | 59.13 | 59.13 | 178.50 | 43.57 | 15.10 |
| Crossing | 50.00 | 51.49 | 50.21 | 182.80 | 49.08 | 11.75 |
| Two crossings | 56.07 | 54.74 | 54.73 | 164.95 | 51.01 | 17.39 |
| Two slow drifters | 53.95 | 55.48 | 63.45 | 177.60 | 40.59 | 23.40 |

For the two drifters, a route adopted at 13.75 s reduced tactical-override updates
from 105 to 39 versus the previous controller at the same speed. In a separate
controlled geometry, one early steering leg cleared two crossings at roughly 93 s
and 99 s into the forecast. A maneuver delayed until ten seconds before the first
crossing could no longer achieve the same uncertainty margin at the available thrust.

In the two-crossing regression, the lower cruise cutoff accounted for nearly the
entire 1.34-point loss. The new controller kept 3.63 m more minimum hull clearance
than the previous 10 m/s run. With the updated tracker and route selection disabled,
the 10 versus 9 m/s score differences were +1.25, −0.39 and 0.00 across noise seeds
7, 23 and 82. Longer early coasts also reversed their advantage across seeds. These
results favor keeping the measured-clearance test for speed changes. Neither a
universal best speed nor a live competition score follows from these flights.

### Running and measuring the controller

The offline suite passed 138 tests. A fixed-state replay of the first 45 s of
`diagnostics/20261119T055550_287409Z` exercised the revised planner on recorded
radar packets using interpolated shuttle telemetry and the logged control modes.
It first produced a forecast at 2.20 s, considered up to 441 repeat tracks, and
selected no detour on this recorded segment. The longest strategic call among
12 eligible calls was 34 ms on this machine. The replay holds shuttle motion
fixed, so it cannot predict a revised mission score or closed-loop latency.

The two local competition configurations reference this checkout's `Controller.py`.
Keep `Controller.py`, `Guidance.py`, `RoutePlanner.py` and `FlightRecorder.py`
together. The previous files for side-by-side comparison are in
`comparison/pre_early_routes/`. Run the following checks from this solution directory
with the bundled Space Teams Python interpreter.

```text
python -B -m unittest discover -s tests -q
python -B tests/compare_routes.py
python -B tests/compare_routes.py --scene "two crossings" --seed 23 --fixed-speeds
```

In Space Teams, compare official score, damage, time and fuel across repeated
flights with the same asteroid field where available. In `telemetry.csv`, inspect
`cruise_reason`, `route_reason`, `route_review_t`, `route_tracks`, `route_cpa_s`,
`route_direct_clearance_m`, `route_clearance_m` and `route_delta_fuel_kg` at each
route change. Check subsequent tactical overrides, measured fuel and spin, and
`planner_ms` for the cost of the full planning cycle. Compare projected fuel and
ETA with the first `ARRIVED` row and the official result. Keep the accompanying
`radar.csv` and `model.json` so the observations and controller version can be matched.

## Previous controller — build to 10 m/s, then use finite correction burns

The previous working version is preserved in `comparison/pre_burn_coast_burn/`.
`Controller.py` now enables `cruise_burns` and sets `COAST_SPEED_M_S = 10.0`.
The launch criterion uses closing speed toward the actual target, with a 0.05 m/s
tolerance. High sideways speed cannot prematurely end acceleration. Main launch
thrust and attitude control retain their existing behavior and acceleration limits.
The target distance is read each update rather than hardcoded to 1300 m.

The phases are:

- **ACCELERATE:** build roughly 10 m/s of useful forward velocity. If gravity would
  stop a later coast short, forward acceleration can resume even above 10 m/s.
- **CLEAR_COAST:** all routine commands are zero while relative velocity and gravity
  predict entry into the arrival corridor, and the radar-based avoidance check is
  safe. Some gravitational speed loss is accepted.
- **TRIM_BURN:** when a coast would miss, estimate the lateral velocity change needed
  to enter the corridor. Test quarter/half/three-quarter/full corrections and choose
  the shortest predicted successful impulse. Burns last 0.05-8 s, limited to the
  calibrated planning authority. If a larger correction is required, apply a bounded
  segment and re-evaluate. A 1.5 s observation gap separates completed course trims
  unless arrival is close. A newly valid coast cancels the remaining trim immediately.
- **REBOOST:** consider restoring speed only after it falls more than 0.8 m/s below
  the initial target, after at least 8 s coasting, and with more than 20 s to arrival.
  Test finite 2/4/8 s burns, shortened where needed to avoid overshooting the speed
  target. Forecast arrival during the burn and after it under gravity. Accept a burn
  only if the estimated time improvement exceeds its fuel penalty by 0.25 score
  points. The nose must already be within 8 degrees of the velocity direction, so an
  optional top-up does not pay for a large turn. This is not continuous speed control.
- **FINAL_COAST / ARRIVED:** preserve unpowered arrival. A safe intercept within 8 s
  can stop launch below 10 m/s. There is no terminal speed-matching requirement.

Each scheduled burn ends on either its time limit or its measured delivered velocity
change, compensating for estimated gravity. The cutoff is checked every control tick,
not just every 0.25 s guidance update. This avoids stretching a tiny requested trim
through the allocator or overburning when actual authority is higher than predicted.
`TRIM_WAIT` is an unpowered interval awaiting the next guidance decision. Radar faults,
excessive spin or an avoidance override cancel the scheduled nominal burn.

The existing forward-first avoidance and authorized emergency escape remain active.
Its baseline forecast now uses the nominal burn's actual remaining duration, rather
than assuming every trim lasts 8 s. The protected connection/helpers, simulation,
scoring, and both simulator configurations are unchanged.

All **122 offline tests passed**. Added checks cover useful closing speed, 10 m/s
cutoff, finite-burn arrival mathematics, measured-impulse cutoff between guidance
updates, mandatory gravity/course corrections, optional-burn score rejection,
coast intervals, final approach, attitude eligibility and avoidance preemption.

Four synthetic point-mass comparisons arrived without collision:

| Synthetic case | Previous 8 m/s: time / fuel | New 10 m/s: time / fuel | Ideal score before → after |
| --- | --- | --- | --- |
| Clear | 191.10 s / 35.70 kg | 164.85 s / 44.33 kg | 65.19 → 63.69 |
| Head-on | 193.55 s / 40.30 kg | 167.55 s / 47.20 kg | 58.26 → 58.69 |
| Crossing | 193.75 s / 44.20 kg | 171.20 s / 50.53 kg | 53.00 → 52.72 |
| Lower authority, stronger gravity | 212.10 s / 38.38 kg | 186.30 s / 47.82 kg | 55.40 → 50.65 |

These are not Space Teams results or mission replays. The first three use constant
0.14 m/s^2 authority; the last uses 0.096 m/s^2 and constant axial gravity of
-0.0095 m/s^2. They omit attitude fuel, real jet allocation and scheduler behavior.
See `comparison/burn_coast_burn_checks.json`. Transit improved about 23-26 s, while
fuel increased 6.9-9.4 kg; a better competition score is not established. Optional
speed top-ups were rejected in these cases; the head-on case needed one course trim.

A 10 m/s coast is not a 10 m/s mission average. Starting from rest, a simple axial
example with 0.09 m/s^2 net acceleration takes about 111 s to reach 10 m/s and covers
556 m during that burn. Coasting the remaining 694 m to a 50 m arrival radius takes
another 69 s: approximately 181 s before allowance for avoidance or varying gravity.
The actual simulation must establish the time and score tradeoff.

Restart Space Teams to reload. Compare official score, arrival time, fuel, damage,
first time reaching 10 m/s closing speed, and coast fraction on repeated runs.
Telemetry adds `guidance_burn_s`, `guidance_burn_plans` and `reburn_advantage` alongside
`nominal_mode`, velocity, gravity and existing avoidance diagnostics. A plan count
includes burns canceled before firing. Positive reburn advantage is a forecast,
not a measured score gain. Check that routine coasts stay unpowered and trim burn
durations count down rather than repeatedly resetting.

## Previous change: preserve forward progress during asteroid avoidance

The previous controller is saved under `comparison/pre_forward_avoidance/`. This
earlier change preserved the 8 m/s coast policy, radar tracker, protected integration code,
scoring, physics and both simulator configurations.

The latest recorded debris run (`diagnostics/20261119T043917_959360Z`) arrived at
257.78 s with 77.76 kg used and zero damage. Across 49 avoidance samples, the mean
cosine between desired acceleration and the target direction was only 0.147; some
commands opposed progress. These are recorded-state observations, not proof that
another controller would have achieved a particular score on that flight.

Routine avoidance now tries forward burns and shallow course changes before an
emergency escape:

- If nominal guidance is already predicted safe, it receives no extra boost.
- Candidate burns last 2, 4 or 8 seconds. Directions include the existing forward
  course, the target, and 5/12/20-degree offsets constrained to remain within 20
  degrees of the target. Prefer low impulse and small lateral cleanup, retaining
  a useful existing flyby course instead of unnecessarily steering at the centre.
- Every candidate is checked against all relevant radar tracks, including the
  existing uncertainty and between-sample margins. Keep the accepted direction
  stable while its remaining burn counts down; a new hazard can interrupt it.
  A nominal path with more than 3 m predicted clearance cancels an unnecessary
  burn immediately, while direction history remains for release hysteresis.
- Forward avoidance commands translation only along the current body +X nose.
  Damped attitude control turns toward the selected course. A 2-degree attitude
  acceptance cone, with 1-degree reengagement hysteresis, allows a deliberate small
  dodge that the usual broad tolerance would ignore. The nose must already be
  within 35 degrees of the target; initial large misalignment can require an escape.
- Prediction includes a delayed heading blend (3 s response allowance, conservative
  0.25 degrees/s blend), analytically integrated through the finite burn and coast.
  Collision checks cover the swept corridor between delayed and immediate steering,
  so a candidate cannot rely on instant nose alignment. This remains approximate;
  real flight evidence is needed for different initial spin states.
- If no forward maneuver is safe, check unpowered coasting. Strong sideways burns
  or braking are available only when neither has a safe solution, as requested for
  emergency fallback. If no candidate clears every hazard, select the escape
  predicted to postpone impact best. Sensor faults still inhibit blind forward
  acceleration. Spin above 3 degrees/s also inhibits these nose-led burns until
  damping restores stability. Collision-free flight cannot be guaranteed.

Changing attitude uses control jets and changing course requires some lateral
impulse. Ordinary avoidance does not request pure sideways translation or backward
braking; the simulator still chooses individual attitude-control jets.

The installed allocator rescales oversized requests. A calculation with a
0.005 rad/s^2 yaw request gave these delivered values:

| Forward request | Forward acceleration | Yaw acceleration | Fuel per second |
| --- | --- | --- | --- |
| 0.50 m/s^2 | 0.09615 m/s^2 | 0.000961 rad/s^2 | 0.41614 kg/s |
| 0.09 m/s^2 | 0.09481 m/s^2 | 0.005267 rad/s^2 | 0.41285 kg/s |

Forward avoidance therefore requests at most the calibrated 0.09 m/s^2 rear
authority. This retains almost all forward acceleration and gives much better
turning response without raising attitude limits. It does not imply a weak rear
burn. Nominal launch acceleration and emergency translation retain their previous
actuation. These are calculations using the installed allocator and jet data,
not measured Space Teams fuel rates.

All 102 offline tests passed. Six allocator/rigid-body checks for 5/12/20-degree
turns in two planes reached the 2-degree cone in 10.3-18.7 s, with peak spin
0.45-2.01 degrees/s. The steering forecast did not lead the nose rotation over the
first 8 s in these checks. See `comparison/forward_steering_checks.json`. These
checks prescribe rear thrust and omit translation and debris.

The simple point-mass mission comparisons all arrived without collision. Transit
changed from 194.10 to 193.55 s for the head-on case and 196.00 to 193.75 s for the
crossing case, but fuel increased by 0.82 and 1.48 kg respectively; calculated ideal
score decreased by about 0.9 and 1.3 points. Clear-flight behavior was unchanged.
This is not established score improvement. That model assumes instantaneous
attitude, identical 0.14 m/s^2 authority in every direction and ideal impulse fuel;
it cannot evaluate the benefit of replacing real side-jet allocation and turning
with rear thrust. See `comparison/forward_avoidance_checks.json`. A synthetic
500-track input retained 133 tracks after broadphase and evaluated 120 candidates
in about 161 ms on this machine; dense real scenes can cost more.

Restart Space Teams and compare repeated debris runs. Watch arrival time, fuel,
damage and official score together. New telemetry includes `avoidance_strategy`
(`GUIDANCE`, `FORWARD`, `COAST`, `ESCAPE`), `forward_avoidance`, remaining
`avoidance_burn_s`, `body_lateral_command` and the world nose vector. During FORWARD,
the body lateral translation command should be near zero; this does not count
attitude jet firings. Check ESCAPE duration, spin, and `planner_ms` too. Frequent
emergency escapes or planning times approaching 250 ms identify useful follow-up
tuning; eliminating visible jet firings alone does not prove a better score.

Run the added allocator/steering check with:

```powershell
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/check_forward_steering.py
```

## Previous change: coast at 8 m/s when the trajectory is safe

`Controller.py` now sets `COAST_SPEED_M_S = 8.0`, measured relative to the target.
Reaching that speed ends the main forward burn when a coast can reach the target.
If the predicted path misses sideways, mode CORRECT requests only a bounded lateral
correction toward the existing flyby aim point, rather than continuing full forward
acceleration. All thrusters can switch off once the arrival and obstacle checks pass.
There is no braking to maintain 8 m/s, and a safe coast slowing below 8 m/s does not
restart thrust. Guidance can resume forward acceleration if gravity would stop the
shuttle short, arrival exceeds the bounded forecast, or motion turns away from the target.

The no-debris recording `diagnostics/20261119T041329_610001Z/telemetry.csv` reached
8.04 m/s at 90.425 s, 931.7 m from the target. Its velocity was about 2.71 degrees
from the target direction, just outside the straight-line 42 m corridor's 2.58-degree
cone. Gravity also affected the predicted miss. The old controller kept accelerating
past 12 m/s. This recorded state now selects a small course correction. A nose-angle
threshold alone would not identify that trajectory error.

The previous working controller, guidance, recorder, notes and affected tests are
preserved under `comparison/pre_adaptive_eight_coast/`. That change left attitude
gains, obstacle avoidance, sensor-failure behavior and simulator configuration intact.

## Separate no-debris test simulation

`COMPETITION_SIM_NO_DEBRIS.simconf` sets the Controller parameter
`NoDebrisTestMode: true`. This explicitly authorizes guidance without radar input
in this known-empty testing environment. Its Radar and SpawnDebris systems stay
disabled; the controller skips reading the disabled radar and permits normal
acceleration, attitude control, coasting, and arrival. A startup log identifies the
mode and telemetry includes `no_debris_test`. No-debris scores are test results,
not competition results.

The normal COMPETITION_SIM does not set this parameter. Empty, stale, or failed
radar there still inhibits blind acceleration. Do not enable this flag in a debris
scenario. `make_no_debris_sim.py` includes the flag automatically when creating the
separate copy, resolves paths relative to the checkout, and refuses to overwrite
its source. Reopen the no-debris simulation after regenerating it so its new
parameters load. Gravity, thruster allocation, and scoring are retained unchanged.

The active simulator entry remains the team Controller.py. Protected connection
code and radar helpers remain unchanged. Earlier controller/modules/notes are preserved in
`comparison/pre_ballistic_coast/`; the -21 version is separately preserved under
`comparison/pre_adaptive_alignment/`.

## What changed and why

The latest recording, `diagnostics/20261119T024046_818494Z/`, showed the nose still
about 67 degrees off target after 37 seconds. Most of those early seconds were
in avoidance, which previously disabled target alignment entirely. A bigger gain
only in clear flight could not fix that. The run reached the arrival region at
about 204.4 seconds with 102.7 kg consumed and zero damage; official scoring alerts
remain authoritative.

The controller now seeks a broad acceptable cone instead of precision pointing:

- Normal distant flight accepts 8 degrees. The cone widens smoothly to 18 degrees
  by 100 m from the target. During avoidance it is at least 12 degrees.
- Alignment disengages on entry into the cone and only restarts after error grows
  another 4 degrees. The internal tracking deadband is half the acceptance cone,
  so the controller enters the cone rather than endlessly approaching its edge.
  Existing spin still gets damped after positional alignment disengages.
- Clear-flight reference slew can reach 10 degrees/s and desired body rate can
  reach 5 degrees/s for large pointing errors. Angular acceleration requests and
  braking stay inside .030 rad/s? and the simulator's actual limit. These are
  request limits, not guaranteed delivered acceleration.
- Nonurgent avoidance with no predicted overlap within 8 seconds permits a modest
  coarse turn toward the target, at quarter reference priority and up to .008
  rad/s? commanded angular acceleration. It never points at the transient dodge
  vector. Urgent avoidance still gets translation priority and spin damping only.
- When translation is off but a coarse turn is useful, the previously tested
  actuator-tick gain calibration applies to alignment as well as damping. This
  avoids full-strength firings from tiny idle angular requests.

## When all thrusters switch off

`ballistic_entry` predicts the first entry into a target sphere using current
relative position and velocity plus estimated external acceleration. It solves
`|r - v*t - 0.5*g*t^2|^2 = radius^2`, including lateral motion. The target is treated
as constant velocity over the prediction. A correctly aimed nose alone cannot
qualify a coast if the actual velocity or gravity would cause a miss.

A new coast requires predicted entry into a 42 m sphere inside the actual 50 m
success region. Once coasting, a 46 m sphere provides hysteresis. Below the speed
cutoff, the existing 90 s entry / 120 s retention windows still permit early or final
coasting. After reaching 8 m/s, a 600 s upper bound permits long coasts; the old 90 s
limit must not force another burn on an otherwise safe distant path. Prediction is
recomputed every guidance update, including current gravity and target-relative
velocity. If the intercept is lost, guidance can correct it; cutoff is reversible.

The radar-based planner checks the coast against all potentially relevant tracks
within its existing 30 s collision horizon (or up to arrival). With fresh radar,
zero required translation, and at least 3 m predicted clearance, mode CLEAR_COAST
shuts off both linear and angular commands. FINAL_COAST does the same when the
predicted entry is within 8 seconds. Positional alignment does not restart firing
just because the nose drifts within this coast. Arrival remains latched and unpowered.
A new threat or stale radar can cancel quiet coasting. Exceptional spin above the
existing recovery threshold can still request damping for stability.

The acceptable velocity direction depends on distance, target motion and gravity,
rather than a fixed nose-angle tolerance. Setting `COAST_SPEED_M_S = None` disables
the 8 m/s burn cutoff while retaining the earlier short-horizon ballistic coast.
Near-target pointing tolerance never weakens arrival or collision-clearance tests.

## Validation

Before the forward-avoidance update, all 86 offline checks passed. Cutoff cases cover distant 8 m/s coasting,
continued acceleration below the cutoff, lateral corrections instead of forward
burns, gravity-induced misses and stopping short, drift after coast entry, arrival
margin hysteresis and the recorded 8.04 m/s state. The real loop is exercised with
a fake API to verify zero linear AND angular commands during distant coast despite
nose misalignment. Existing hazard, stale-radar, attitude, allocator and no-debris
configuration tests also pass. Tests do not connect to the live simulator.

Three synthetic point-mass comparisons reached the target without collision with
the 8 m/s cutoff. The clear case used 35.70 kg versus 45.08 kg before this change,
but took 191.10 s versus 163.25 s; ideal calculated score was 65.19 versus 63.40.
The model omits attitude fuel, full jet dynamics and real scheduler behavior. These
are separate synthetic trajectories, not replayed missions or actual competition
scores. See `comparison/adaptive_eight_coast_checks.json` for all three comparisons.

An interrupted-turn check executes the actual attitude statements extracted from
both controller versions with the installed jet allocator and ideal rigid-body
rotation. With clear thrust for 0-12 seconds and nonurgent avoidance for 12-40
seconds, entry into a 12-degree cone improved from 57.8 s to 32.0 s, peaking near
4.1 degrees/s in the revised calculation. This is not a Space Teams flight or a
mission replay: translation commands are prescribed and debris/position are not
integrated. See `comparison/interrupted_alignment_check.json`.

Synthetic point-mass coast scenarios also completed without collision. On a clear
path this model used about 45 kg rather than the previous continuous-acceleration
experiment's 81 kg, but it omits rotational fuel, real scheduler behavior and full
jet dynamics. Actual score and reliability still require repeated Space Teams runs.
See `comparison/ballistic_coast_experiment.json`; do not interpret its ideal scores
as competition results.

Run offline checks from this solution directory:

```powershell
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B -m unittest discover -s tests -v
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/check_interrupted_alignment.py
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/offline_scenarios.py
```

## Next simulator run

Stop and reopen COMPETITION_SIM so Python reloads. Keep Controller.py, Guidance.py,
RoutePlanner.py and FlightRecorder.py together. Compare official score, damage, fuel, early time to
enter the 12-degree cone, and peak spin. In diagnostics inspect `nose_error_deg`,
`nose_tolerance_deg`, `alignment_priority`, `speed_cutoff_reached`, `velocity_error_deg`,
`quiet_coast`, `coast_entry_s`, linear/angular firing, and any return from coast to
correction, avoidance or acceleration. `coast_entry_s = inf` in CORRECT means an
unpowered intercept has not yet qualified. Check fuel at the first cutoff and on
arrival, time, damage, coast fraction and official score together: saving fuel can
also lengthen the flight. The no-debris and normal configurations use the same code.

Limits still needing flight evidence include dense-contact association, the
constant-acceleration arrival forecast, actual combined translation/rotation
response, unknown changes in debris motion, and hull geometry estimates. Turning
faster can cost torque fuel; longer coasting can cost time. The real score must
settle those tradeoffs. Diagnostic files stay local under `diagnostics/` and are
ignored by Git; model parameters and source hashes identify each run.
