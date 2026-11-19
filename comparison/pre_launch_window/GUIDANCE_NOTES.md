# Ballistic coasting and coarse nose alignment

The active simulator entry remains the team Controller.py. Protected connection
code, radar helpers, physics, scoring, system activation, and sim configuration are
unchanged. The preceding controller/modules/notes are preserved in
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

A new coast requires predicted entry within 90 seconds into a 42 m sphere inside
the actual 50 m success region. Once coasting, a 46 m sphere and 120 s window provide
hysteresis. Prediction is recomputed every guidance update. If the intercept is
lost, guidance can resume; there is no irreversible engine cutoff.

The radar-based planner checks the coast against all potentially relevant tracks
within its existing 30 s collision horizon (or up to arrival). With fresh radar,
zero required translation, and at least 3 m predicted clearance, mode CLEAR_COAST
shuts off both linear and angular commands. FINAL_COAST does the same when the
predicted entry is within 8 seconds. Positional alignment does not restart firing
just because the nose drifts within this coast. Arrival remains latched and unpowered.
A new threat or stale radar can cancel quiet coasting. Exceptional spin above the
existing recovery threshold can still request damping for stability.

The coast condition is based on geometric arrival and observed hazards, not an
arbitrary distance or a fixed 5 m/s speed servo. `COAST_SPEED_M_S = None` still
disables the optional fixed-speed experiment; it no longer disables automatic
trajectory-qualified coasting. Near-target pointing tolerance never weakens the
position-based success or collision-clearance criteria.

## Validation

All 69 offline checks passed. New cases cover ballistic intercepts, lateral misses,
gravity that stops the shuttle short or curves it away, zero linear AND angular
commands during safe coast despite nose misalignment, threat/stale-radar override,
coarse alignment during nonurgent avoidance, urgent-avoidance priority, and relaxed
attitude tolerance. Tests do not connect to the live simulator.

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
and FlightRecorder.py together. Compare official score, damage, fuel, early time to
enter the 12-degree cone, and peak spin. In diagnostics inspect `nose_error_deg`,
`nose_tolerance_deg`, `alignment_priority`, `quiet_coast`, `coast_entry_s`, linear/
angular firing, and any return from coast to avoidance or acceleration.

Limits still needing flight evidence include dense-contact association, the
constant-acceleration arrival forecast, actual combined translation/rotation
response, unknown changes in debris motion, and hull geometry estimates. Turning
faster can cost torque fuel; longer coasting can cost time. The real score must
settle those tradeoffs. Diagnostic files stay local under `diagnostics/` and are
ignored by Git; model parameters and source hashes identify each run.
