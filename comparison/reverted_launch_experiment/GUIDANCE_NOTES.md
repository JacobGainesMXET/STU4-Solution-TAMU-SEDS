# Launch-window and gentle-turn experiment

The active entry remains STU4-Solution-TAMU-SEDS/Controller.py. Protected connection
code, radar helpers, simulation physics, scoring, timescale, and system activation
are unchanged. The preceding positive-scoring policy and modules are saved in
`comparison/pre_launch_window/`. Earlier comparisons remain in their own folders.

## First 20 seconds: no evasive commands

`AVOIDANCE_DELAY_S = 20.0` in Controller.py bypasses the avoidance planner before
20 simulated seconds from the existing simulation start timestamp. Nominal position/
velocity guidance controls acceleration and attitude instead. Radar acquisition,
association and tracking continue, and shadow predictions are logged. No actual
radar or simulator system is deactivated. At the first control tick at/after 20 s,
a planning update is forced even if the regular planning period has not elapsed.
The warmed tracker is then available to normal avoidance.

This deliberately disables automatic dodges even for a predicted imminent collision
in that launch window. It is an experiment, not an established improvement in
reliability. Invalid own-ship state still clears commands; arrival and trajectory-
qualified coasting remain in force. Launch does not wait for missing radar, but
normal post-launch missing-radar handling returns at 20 s. Set the delay to 0 to
restore avoidance from the beginning without changing the simulator configuration.

The existing recordings do not establish that early avoidance wastes propellant.
For example, the best-scoring recorded run used avoidance during part of its first
20 s and consumed about 7.5 kg by then; another run with no early avoidance consumed
about 12.7 kg. These are different recorded trajectories, not a controlled A/B test.
Compare first-20-second fuel AND damage, then final official score over multiple runs.

## Gentler turning

Large pointing errors now have a normal command cap of .010 rad/s? instead of .030,
a maximum desired turn rate of 2.5 degrees/s instead of 5, and reference slew up to
4 degrees/s instead of 10. Nonurgent avoidance uses still gentler coarse alignment.
The broad acceptance cones (8 degrees far away, at least 12 during avoidance, up
to 18 near arrival) and disengagement hysteresis remain.

Angular braking retains up to .020 rad/s? in clear powered flight, within the actual
vehicle limit, so existing spin can still be stopped. Exceptional-spin recovery
retains its previous limit. Idle commands retain the allocator-tick calibration.
This reduces angular momentum being built, rather than weakening the damping needed
to stop it. Counter-thrust cancels angular momentum; it is not a continuous holding
force once angular velocity is negligible.

An ideal rigid-body calculation using the installed allocator and identical
prescribed translation phases reduced peak spin from about 4.56 to 2.29 degrees/s
and peak angular momentum from 473 to 238 N m s. It took longer to enter the
12-degree cone (about 53 versus 25 seconds). These are mathematical control-response
results, not a mission replay, competition score, or proof of fuel savings.
See `comparison/gentle_turn_check.json`. This explicitly trades fast pointing for
less spin and a smaller counter-impulse; the actual run must establish its value.

## Velocity-vector guidance, including the first 60 seconds

Trajectory guidance already uses target-relative position, target-relative velocity,
and estimated gravity in Phobos axes. It corrects lateral velocity and predicts
ballistic arrival; it never waits for nose alignment before accelerating. The nose
is used for attitude control and converting world commands into body-frame commands.
This separation is preserved throughout the flight, not switched off at 60 s.

New `velocity_target_angle_deg` telemetry measures the angle between target-relative
velocity and the target line. Below .5 m/s it is undefined (NaN), avoiding a noisy
normalized vector at liftoff. Position-based launch guidance remains available then.
Compare this field with `nose_error_deg`: a poorly aligned nose can accompany a good
flight path, and the opposite is also possible. `early_flight` marks the first 60 s;
`avoidance_enabled` exposes the 20-second bypass and reactivation. Mode LAUNCH means
nominal acceleration during the bypass. These labels do not change simulator state.

## Existing coast and safety behavior

With fresh radar and no predicted conflict, a ballistic path into the target region
can still enter CLEAR_COAST or FINAL_COAST, clearing both linear and angular thrust.
Entry uses a 42 m sphere inside the 50 m success region and a 90 s forecast; an
ongoing coast uses 46 m and 120 s for hysteresis. FINAL_COAST begins within 8 s of
predicted entry. A new threat after the launch window or a lost intercept can restore
control. Near-zero pointing error is not required to coast. Exceptional spin may
still request recovery damping. Arrival remains latched and unpowered.

## Validation and next run

All 74 offline checks passed, including the launch bypass with a mocked imminent
threat, no planner calls before 20 s, restoration at the boundary, continued tracking,
launch with missing radar and post-launch fallback, and identical world-frame thrust
commands with different nose orientations. Velocity-angle tests cover stationary,
aligned, transverse and receding motion, and lateral correction during the first
60 s. Previous geometry, allocation, coast and override checks remain.

Run from the solution directory:

```powershell
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B -m unittest discover -s tests -v
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/check_interrupted_alignment.py --save
```

No new Space Teams run was performed by these checks. Restart COMPETITION_SIM to
reload the code. Keep Controller.py, Guidance.py, and FlightRecorder.py together.
Inspect first-20-second fuel, damage, velocity angle versus nose angle, peak spin,
angular firing and counter-corrections, plus the first maneuver after avoidance
reactivates. Then compare official score, time, total fuel and zero-damage completion
rate against the saved positive-scoring controller. The comparison changes both
launch avoidance and turning, so use a 0-second delay to isolate the turn change.

Diagnostics remain local under `diagnostics/` and are ignored by Git. Source hashes
and starting model parameters identify each run; no data is uploaded automatically.
