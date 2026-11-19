# Clear-path acceleration experiment

The active controller is `STU4-Solution-TAMU-SEDS/Controller.py`, selected by
`STU4-Sim/Sims/COMPETITION_SIM.simconf`. Its protected connection/helper section and
radar tools are unchanged. The preceding score/coast controller and its modules/notes
are preserved in `comparison/score_policy_v1/`. The earlier stable controller is
still in `comparison/Controller_before_predictive.py`.

## Adaptive nose alignment after the -21 run

The best recorded run, `20261119T022628_760581Z`, arrived at about 187.9 s,
101.25 kg, zero damage, and a telemetry score of -21.13. Its Controller.py and
Guidance.py hashes match the saved files in `comparison/pre_adaptive_alignment/`.
Those files preserve the controller before this attitude-only revision.

On accepted clear-path thrust, actual nose error now schedules alignment strength
smoothly from zero extra boost at 5 degrees to full boost at 60 degrees. The
reference slew grows from 2 to 6 degrees/s, desired rotation-rate cap grows from
3 to 4 degrees/s, proportional gain increases up to twice its prior value, and
angular acceleration command authority grows from .005 to .020 rad/s² within
the vehicle's actual limit. These are controller tuning values, not new physics.
Small errors retain gentle control. Up to .020 rad/s² remains available to brake
existing rotation even when position error shrinks, reducing overshoot. The
separate high-spin recovery limit remains .030 rad/s² within the simulator limit.

During avoidance, nose-alignment priority drops to zero even when lateral jets
are firing. The reference holds and spin damping remains active. Sideways collision
avoidance does not wait for the nose to turn. This implements lower target-pointing
priority during safety maneuvers without a singular inverse-angle gain. Coasting
keeps the previously validated idle-torque calibration. Translational guidance,
full clear-path thrust, collision prediction, and the disabled coast-speed cutoff
are unchanged.

The original `attitude_error_deg` compares attitude to a slewed reference and can
therefore be small while the actual nose is still far from the ship. New columns
`nose_error_deg`, `alignment_priority`, `angular_accel_limit`, and
`angular_brake_limit` expose the actual pointing error and scheduled authority.
During clear thrust the pointing direction is the required acceleration toward the
target aim region; during avoidance the nose diagnostic uses the direct target line.

All 56 offline checks passed after the revision. A 90-degree ideal rigid-body turn
using the installed jet allocator reached within 5 degrees in 31.2 s versus 50.0 s
with the previous settings, peaking at 3.77 versus 2.90 degrees/s. It also had less
residual pointing error after 120 s. This calculation assumes fixed world thrust
and excludes debris and translation integration: it is not a mission score or a
flight result. Results are in `comparison/adaptive_alignment_allocator_check.json`;
reproduce with `python -B tests/check_alignment_allocator.py`.

On the next Space Teams run, compare time to `nose_error_deg < 5`, peak spin,
overshoot, fuel, official score, and damage against the -21 baseline. Stronger
turning can consume more torque fuel; the real run must establish the tradeoff.

## What the actual flight data showed

The recorded flight `diagnostics/20261119T011253_713806Z/` reached roughly 5 m/s,
then spent a long time coasting while gravity reduced its velocity. There was no
5 m/s speed servo: the old score search stopped requesting forward burns early.
First recorded arrival was around 399 s with 171.7 kg consumed and zero damage.
These are controller telemetry values; the official scoring alert is authoritative.

Adjacent telemetry intervals labelled COAST totalled about 190 s and 75.4 kg.
Linear firing was off in almost every COAST snapshot, while angular firing was
active in about 68% of those snapshots. This strongly implicates rotational
corrections in the fuel loss. One-second snapshots do not provide exact fuel
attribution between translation and rotation. The analysis is saved in
`comparison/score_policy_v1/recorded_flight_analysis.json`.

The installed allocator rescales solved jet activity as on-time. Small angular
commands can produce strong firing, which the earlier idealized angular plant
failed to represent. New tests extract the allocator's real pure functions and
use its jet geometry, inertia, and revised-simplex allocation without connecting
to Space Teams.

## Current control policy

- On a clear predicted path, send a sustained linear command at the vehicle's
  permitted command limit. This bypasses the translation pulse/debt limiter.
  The 0.5 m/s? command limit is not a promise of 0.5 m/s? delivered acceleration.
- Start when valid fresh radar observations exist. Do not wait for 60% of all
  contacts to receive a second scan. Unconfirmed tracks retain broad transverse
  uncertainty and a shorter prediction horizon while tracking matures.
- Keep timestamp-aware radar association, motion estimation, covariance propagation,
  and finite-burn encounter checks. Forecasts use estimated delivered thrust,
  not the full command magnitude. Online burn-authority measurements remain active.
- If full forward thrust threatens clearance, consider coasting first, then lateral
  corrections or braking as needed. Safe paths retain priority. Braking receives
  a trajectory-progress penalty, but remains available when required for safety.
- An uncertain conflict far in the forecast uses CAUTION rather than forcing an
  immediate maximum-duty emergency burn. Predicted overlap within six seconds
  can still bypass the pulse start delay. Release hysteresis remains in place.
- Keep the attitude reference through brief avoidance commands. While clear-path
  thrust is active, slew toward its required direction with the existing damped
  attitude feedback. Do not chase every sideways correction with the nose.
- When linear thrust is off, damp angular velocity rather than recovering an exact
  orientation. Inside 0.15 degrees/s, request zero idle torque. Otherwise, attenuate
  the damping command by the installed actuator's 1/60 s tick duration. Allocation
  tests verify the intended low-acceleration response and reduced propellant in
  this unsaturated idle regime. This changes controller gains only, not the actuator.
- Preserve position-only arrival: coast through the final safe approach and latch
  entry into the 50 m success region. There is no zero-speed rendezvous requirement.
- Missing radar inhibits acceleration and permits gentle braking. Nonfinite own-ship
  state clears commands. Diagnostic write failures do not stop guidance.

There is no cruise-speed target and no automatic score-based en-route coast decision
in this experiment. `COAST_SPEED_M_S = None` in Controller.py disables the optional
cutoff. If a positive value is chosen after flight analysis, reaching it latches
coasting; the controller does not brake down to that speed or keep reaccelerating
whenever gravity reduces it. Collision corrections can still override coasting.

## Evidence and remaining tradeoff

All 51 offline checks passed, including the actual allocator calculations, initial
launch before track confirmation, continuous clear-path commands, absence of a speed
servo, optional coast latching, radar failure handling, geometry, and collision cases.
These are offline checks, not new Space Teams flight results.

The point-mass experiment in `comparison/clear_path_experiment.json` intentionally
excludes angular dynamics/fuel and real simulator scheduling. Continuous clear-path
acceleration arrived in about 136 s using 81 kg of ideal propellant. Its modeled
score was about 30. An illustrative 8 m/s coast cutoff used about 36 kg and arrived
in 191 s, scoring about 65 in that artificial plant. Thus continuously accelerating
is a useful faster-ascent baseline, not evidence of the maximum competition score.
The coast cutoff remains disabled as requested so actual acceleration and fuel data
can guide the next choice. Synthetic approaching/crossing scenarios also completed
without collision; they do not establish real mission reliability.

Run checks from the solution directory:

```powershell
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B -m unittest discover -s tests -v
& 'C:/Users/jacob/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe' -B tests/offline_scenarios.py --sweep
```

## Next flight

Stop and reopen the competition simulation so the Python code reloads. Keep
Controller.py, Guidance.py, and FlightRecorder.py together. No scoring, physics,
thruster configuration, system activation or timescale is changed by this revision.
The installed JetSelectionActuator is configured for 60 Hz; idle torque gain needs
revalidation if a different competition backend changes that rate or allocation math.

Flight diagnostics remain under `diagnostics/<UTC stamp>/`, ignored by Git:
`telemetry.csv`, accepted unique sensor samples in `radar.csv`, and starting parameters
plus source hashes in `model.json`. New telemetry fields distinguish `full_thrust`,
`nominal_mode`, `forward_command`, `delivered_forward_accel`, `thrust_power`, `braking`,
`angular_command_norm`, and the optional `coast_speed`.

First check official score and zero-damage completion. Then examine time to reach
5/8/10 m/s, progress in the first 20/40/60 s, fuel growth with linear firing off,
spin during coast, time in CAUTION/EMERGENCY/SENSOR_BRAKE, and requested versus
measured forward acceleration. Use several runs to distinguish a better policy
from a fortunate debris encounter. The real coast cutoff should balance arrival
time, propellant, and maneuvering margin using those new recordings.
