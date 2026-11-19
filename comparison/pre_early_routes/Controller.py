#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

#############################################################################################
# RADAR TOOLS                                                                               #
# Only needs numpy (np / npt are already imported by the boilerplate).                      #
#############################################################################################

from dataclasses import dataclass

# From the wiki: radar 1-sigma noise is 2 m on position and closest-range, 0.02 m/s on range
# rate, and 5 % on diameter. The cone uses the position noise as part of its safety margin.
RADAR_POS_SIGMA_M = 2.0

# ROUGH guess at the shuttle's bounding radius (camera mounts sit ~1-2 m from the CoM).
# Check it against the wireframe images on the wiki and tune it.
SHUTTLE_RADIUS_M = 2.5


@dataclass
class RadarContact:
    """One radar observation of one piece of debris, with the 3x3 matrix unpacked."""
    rel_pos: npt.NDArray     # m    debris center relative to the shuttle, in Phobos-frame axes
    surface_range: float     # m    shuttle -> closest point on the debris (radar r_c)
    range_rate: float        # m/s  positive = moving apart, negative = closing
    diameter: float          # m    average diameter (5 % noise)
    t_meas: float            # s    sim time of the measurement

    @property
    def center_range(self) -> float:
        return float(np.linalg.norm(self.rel_pos))

    @property
    def radius(self) -> float:
        return 0.5 * self.diameter

    @property
    def closing(self) -> bool:
        return self.range_rate < 0.0

    def time_to_contact(self, clearance_m: float = 0.0) -> float:
        """
        Rough time until the gap to the debris surface shrinks to `clearance_m`, assuming
        the current range rate holds. Radar only gives the radial part of the relative
        velocity, so treat this as a priority hint, not a prediction. inf if not closing.
        """
        if self.range_rate >= 0.0:
            return float("inf")
        return max(self.surface_range - clearance_m, 0.0) / -self.range_rate


@dataclass
class TravelCone:
    """The cone that was used, plus the debris that intersects it (nearest first)."""
    axis: npt.NDArray        # unit vector, Phobos-frame axes, from the shuttle outward
    length_m: float
    half_angle_deg: float
    contacts: list[RadarContact]


def GetRadarContacts(max_count: int = 500) -> list[RadarContact]:
    """Read the radar once and unpack it. Call this once per loop and reuse the result."""
    contacts: list[RadarContact] = []
    for item in GetNearestDebrisLocations(max_count):   # boilerplate helper, max 500
        contacts.append(RadarContact(
            rel_pos=np.array(item[0, :], dtype=float),
            surface_range=float(item[1, 0]),
            range_rate=float(item[1, 1]),
            diameter=float(item[1, 2]),
            t_meas=float(item[2, 0]),
        ))
    return contacts


def GetDebrisWithinRange(contacts: list[RadarContact],
                         max_range_m: float,
                         reference=None,
                         measure: str = "surface") -> list[RadarContact]:
    """
    All debris within `max_range_m` of a reference, nearest first.

    reference:
        None             -> measure from the shuttle (default)
        a RadarContact   -> measure from ANOTHER piece of debris (that piece is excluded)
        a 3-vector       -> measure from any point, given relative to the shuttle in
                            Phobos-frame axes
    measure:
        "surface" -> gap to the debris surface (uses the radar's own r_c for the shuttle case)
        "center"  -> distance to the debris center
    """
    if not contacts:
        return []

    if reference is None:
        if measure == "surface":
            dist = np.array([c.surface_range for c in contacts])
        else:
            dist = np.array([c.center_range for c in contacts])
        mask = dist <= max_range_m
    else:
        if isinstance(reference, RadarContact):
            ref_pos, ref_radius = reference.rel_pos, reference.radius
        else:
            ref_pos, ref_radius = np.asarray(reference, dtype=float), 0.0

        positions = np.array([c.rel_pos for c in contacts])
        dist = np.linalg.norm(positions - ref_pos, axis=1)
        if measure == "surface":
            radii = np.array([c.radius for c in contacts])
            dist = np.maximum(dist - radii - ref_radius, 0.0)
        mask = (dist <= max_range_m) & np.array([c is not reference for c in contacts])

    idx = np.flatnonzero(mask)
    idx = idx[np.argsort(dist[idx])]
    return [contacts[i] for i in idx]


def GetDebrisInTravelCone(contacts: list[RadarContact],
                          ship_vel_phobos: npt.NDArray,
                          half_angle_deg: float = 20.0,
                          lookahead_s: float = 10.0,
                          min_length_m: float = 30.0,
                          fallback_axis: npt.NDArray | None = None,
                          min_speed_m_s: float = 0.5,
                          ship_radius_m: float = SHUTTLE_RADIUS_M,
                          safety_margin_m: float = 2.0 * RADAR_POS_SIGMA_M,
                          only_closing: bool = False) -> TravelCone:
    """
    Debris that could matter as the shuttle travels: everything that touches a cone whose
    apex is the shuttle and whose axis points along the direction of travel.

    ship_vel_phobos   shuttle velocity in Phobos-frame axes (the same `vel` the starter code
                      reads). Radar positions are already in these axes, so no conversion.
    half_angle_deg    cone half-angle. Widen it if you expect to steer / swerve a lot.
    lookahead_s       cone length = max(min_length_m, speed * lookahead_s)
    fallback_axis     direction to use when the shuttle is nearly stationary (e.g. the
                      direction to the target). With no usable direction at all, the cone
                      becomes a full sphere of radius `length`, which is the safe choice.
    ship_radius_m,    each piece of debris is inflated by its own radius + the shuttle's
    safety_margin_m   radius + this margin (default = 2 sigma of radar position noise), so
                      big debris just off-axis is still caught and noise doesn't hide a miss.
    only_closing      if True, drop debris whose range rate says it is moving away.
    """
    v = np.asarray(ship_vel_phobos, dtype=float)
    speed = float(np.linalg.norm(v))

    if speed >= min_speed_m_s:
        axis = v / speed
        half_deg = half_angle_deg
    elif fallback_axis is not None and np.linalg.norm(fallback_axis) > 1e-9:
        fa = np.asarray(fallback_axis, dtype=float)
        axis = fa / np.linalg.norm(fa)
        half_deg = half_angle_deg
    else:
        axis = np.array([1.0, 0.0, 0.0])   # arbitrary; irrelevant for a full sphere
        half_deg = 180.0

    length = max(min_length_m, speed * lookahead_s)
    cone = TravelCone(axis=axis, length_m=length, half_angle_deg=half_deg, contacts=[])
    if not contacts:
        return cone

    positions = np.array([c.rel_pos for c in contacts])               # (N, 3)
    dist = np.linalg.norm(positions, axis=1)                          # center distances
    r_eff = np.array([c.radius for c in contacts]) + ship_radius_m + safety_margin_m

    safe_dist = np.maximum(dist, 1e-9)
    along = positions @ axis                                          # distance along the axis
    theta = np.arccos(np.clip(along / safe_dist, -1.0, 1.0))          # angle off the axis
    alpha = np.arcsin(np.clip(r_eff / safe_dist, 0.0, 1.0))           # angular size of inflated debris

    inside = (theta <= np.deg2rad(half_deg) + alpha) & (along <= length + r_eff)
    inside |= dist <= r_eff                                           # already touching: always keep

    if only_closing:
        inside &= np.array([c.closing for c in contacts])

    idx = np.flatnonzero(inside)
    idx = idx[np.argsort(dist[idx])]
    cone.contacts = [contacts[i] for i in idx]
    return cone

#############################################################################################
# END RADAR TOOLS                                                                           #
#############################################################################################
# ---------------------------------------------------------------------------------------
# Example use inside the main loop (uses `vel` and `goalDirection` from the starter code):
#
#   contacts = GetRadarContacts(500)                       # read the radar ONCE per loop
#
#   nearby = GetDebrisWithinRange(contacts, 50.0)          # gap to surface <= 50 m
#   nearby_center = GetDebrisWithinRange(contacts, 50.0, measure="center")
#   neighbors = GetDebrisWithinRange(contacts, 15.0, reference=nearby[0])  # near ANOTHER debris
#
#   cone = GetDebrisInTravelCone(contacts, vel, half_angle_deg=20.0,
#                                lookahead_s=10.0, fallback_axis=goalDirection)
#   for c in cone.contacts[:5]:
#       print(c.surface_range, c.range_rate, c.time_to_contact(SHUTTLE_RADIUS_M))
#
#   st.OnScreenLogMessage(f"{len(nearby)} within 50 m, {len(cone.contacts)} in travel cone",
#                         "RadarTools", st.Severity.Info)
# ---------------------------------------------------------------------------------------


#############################################################################################
# QUALIFICATION CONTROLLER: fly toward the target, steer around radar-detected debris       #
#############################################################################################

import math

controller_directory = os.path.dirname(os.path.abspath(__file__))
if controller_directory not in sys.path:
    sys.path.insert(0, controller_directory)

from Guidance import (MissionModel, StateHistory, RadarTracker, AccelerationGuidance,
                      PredictionScene, AvoidancePlanner, PulseController,
                      AccelerationAttitude, limited, learn_burn_model, angular_command,
                      alignment_strength, pointing_tolerance)
from FlightRecorder import FlightRecorder

# Main forward burns on clear paths and shallow avoidance; pulsed emergency escape.
ATTITUDE_GAIN = 0.15
MAX_TURN_RAD_S = math.radians(3.0)
MAX_ANG_ACCEL = 0.01
ANG_RATE_TIME_S = 1.5
ATTITUDE_TOL_RAD = math.radians(1.5)
ANG_RATE_TOL_RAD_S = math.radians(0.15)
RADAR_COUNT = 500
PLANNING_INTERVAL_S = 0.25
STATUS_INTERVAL_S = 1.0
ACTUATOR_PERIOD_S = 1.0 / 60.0  # installed JetSelectionActuator LoopFreqHz; idle torque calibration
COAST_SPEED_M_S = 10.0  # initial closing-speed target; timed trims after the main burn


def LimitVector(vector: npt.NDArray, limit: float) -> npt.NDArray:
    magnitude = float(np.linalg.norm(vector))
    return vector * min(1.0, limit / max(magnitude, 1e-12))


def MakeTargetAttitude(forward: npt.NDArray, roll_reference: npt.NDArray) -> npt.NDArray:
    """Rows are perpendicular target body axes expressed in Phobos coordinates.

    Carry the previous desired y-axis forward to avoid unnecessary roll during a dodge.
    """
    x_axis = forward / np.linalg.norm(forward)
    y_axis = roll_reference - float(roll_reference @ x_axis) * x_axis
    if np.linalg.norm(y_axis) < 1e-6:
        reference = np.eye(3)[np.argmin(np.abs(x_axis))]
        y_axis = reference - float(reference @ x_axis) * x_axis
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    return np.vstack((x_axis, y_axis, z_axis))


def RotationVector(rotation: npt.NDArray) -> npt.NDArray:
    """Shortest axis-angle vector of an active rotation matrix, including 180 degrees.

    Working with a matrix makes this independent of quaternion sign conventions.
    """
    sine_axis = 0.5 * np.array([rotation[2, 1] - rotation[1, 2],
                                rotation[0, 2] - rotation[2, 0],
                                rotation[1, 0] - rotation[0, 1]])
    sine = float(np.linalg.norm(sine_axis))
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.atan2(sine, cosine)
    if cosine > 0.0 and sine < 1e-8:
        return sine_axis
    if cosine < 0.0 and sine < 1e-5:
        # The skew part vanishes at 180 degrees. Its eigenvector with eigenvalue 1
        # still gives the rotation axis; choose a deterministic sign at exactly pi.
        values, axes = np.linalg.eigh(0.5 * (rotation + rotation.T))
        axis = axes[:, np.argmax(values)]
        if sine > 1e-10:
            if float(axis @ sine_axis) < 0.0:
                axis = -axis
        elif axis[np.argmax(np.abs(axis))] < 0.0:
            axis = -axis
        return angle * axis
    return (angle / sine) * sine_axis


def ComputeAttitudeAcceleration(current_axes, target_axes, angular_velocity, accel_limit,
                                *, gain_scale=1.0, rate_limit=None, brake_limit=None, tolerance=None):
    """Return body angular acceleration and attitude error in radians.

    Each input matrix has body axes in its ROWS, expressed in Phobos coordinates.
    current_axes @ target_axes.T is the desired ACTIVE rotation in current body axes.
    """
    error = RotationVector(current_axes @ target_axes.T)
    angle = float(np.linalg.norm(error))
    tolerance = ATTITUDE_TOL_RAD if tolerance is None else tolerance
    if angle <= tolerance and np.linalg.norm(angular_velocity) <= ANG_RATE_TOL_RAD_S:
        return np.zeros(3), angle
    desired_rate = ATTITUDE_GAIN * gain_scale * error * max(0.0, 1.0 - tolerance / max(angle, 1e-12))
    desired_rate = LimitVector(desired_rate, MAX_TURN_RAD_S if rate_limit is None else rate_limit)
    acceleration = (desired_rate - angular_velocity) / ANG_RATE_TIME_S
    # Preserve braking authority as pointing error shrinks while the shuttle is
    # still rotating. Weakening damping here would cause overshoot and fuel reversal.
    if brake_limit is not None and float(acceleration @ angular_velocity) < 0.0:
        accel_limit = max(accel_limit, brake_limit)
    return LimitVector(acceleration, accel_limit), angle


# Explicitly enabled only by the separate no-debris test configuration. An empty
# or failed radar in the normal competition must still inhibit blind acceleration.
no_debris_test_mode = (this.GetParam(st.VarType.bool, "NoDebrisTestMode")
                       if this.HasParam("NoDebrisTestMode") else False)
if no_debris_test_mode:
    st.OnScreenLogMessage("No-debris TEST mode: guidance enabled without radar observations",
                          "Guidance", st.Severity.Info)

# Read only own-ship properties and permitted target/planet state. Never read debris truth.
linear_limit = shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAccel_m_s2"])
angular_limit = shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAngAccel_rad_s2"])
if not (np.isfinite(linear_limit) and linear_limit > 0.0
        and np.isfinite(angular_limit) and angular_limit > 0.0):
    SetCommand(np.zeros(3), np.zeros(3))
    raise ValueError("Invalid shuttle acceleration limits")
model = MissionModel(planning_accel=min(.12, .5 * linear_limit),
                     avoidance_accel=min(.09, linear_limit),
                     coast_speed=COAST_SPEED_M_S,
                     cruise_burns=True,
                     fuel_per_dv=1.3 * shuttle.getMass() / (80.0 * 9.8))
try:
    phobos_mu = phobos.GetParam(st.VarType.double, ["Dynamics", "GravitationalParameter_km3_s2"]) * 1e9
except Exception:
    phobos_mu = 0.0                    # coasting observations still estimate total external acceleration
history = StateHistory()
tracker = RadarTracker(model)
guidance = AccelerationGuidance(model)
avoidance = AvoidancePlanner(model)
linear_pulses = PulseController(.04, .008, .25, .08)
was_full_thrust = False
linear_switches = 0
angular_switches = 0
attitude_reference = None
roll_reference = None
previous_time = None
previous_velocity = None
previous_omega_world = None
previous_linear_command = np.zeros(3)
previous_angular_command = np.zeros(3)
external_residual = np.zeros(3)
next_plan = -math.inf
next_status = -math.inf
last_fault_log = -math.inf
command = np.zeros(3)
mode = "ACQUIRE"
plan = None
avoidance_burn_until = -math.inf
nominal = None
mission_complete = False
quiet_coast = False
alignment_engaged = True
coast_seconds = 0.0
total_seconds = 0.0
fuel_used = 0.0
damage = 0.0
fuel_observer_fuel = 0.0
fuel_observer_dv = 0.0
fuel_observer_on_seconds = 0.0
fuel_observer_rotation = 0.0
planner_ms = 0.0
recorder = None
try:
    recorder = FlightRecorder(controller_directory, model)
    st.OnScreenLogMessage(f"Flight diagnostics: {recorder.directory}", "Guidance", st.Severity.Info)
except Exception as error:
    st.OnScreenLogMessage(f"Flight recording unavailable: {error}", "Guidance", st.Severity.Warning)

t0_sim = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0 = t0_sim.as_datetime()
exit_flag = False
while not exit_flag:
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))
    now = (st.SimGlobals.SimClock.GetTimeNow().as_datetime() - t0).total_seconds()
    dt = 1.0 / this.GetParam(st.VarType.double, "LoopFreqHz") if previous_time is None else now - previous_time
    if dt <= 0.0:
        continue
    previous_time = now
    state = shuttle.getLocVelAcc()
    loc = np.asarray(state.loc_WRT_ExprIn(phobosFrame), dtype=float)
    vel = np.asarray(state.vel_WRT_ExprIn(phobosFrame), dtype=float)
    target_state = target.getLocVelAcc()
    target_loc = np.asarray(target_state.loc_WRT_ExprIn(phobosFrame), dtype=float)
    target_vel = np.asarray(target_state.vel_WRT_ExprIn(phobosFrame), dtype=float)
    current_axes = np.vstack([
        st.frames.FramedVector(axis, shuttleFrame).ExprIn(phobosFrame) for axis in np.eye(3)
    ])
    omega_body = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), dtype=float)
    if not np.all(np.isfinite(np.r_[loc, vel, target_loc, target_vel, current_axes.ravel(), omega_body])):
        SetCommand(np.zeros(3), np.zeros(3))
        previous_velocity = previous_omega_world = None
        previous_linear_command = previous_angular_command = np.zeros(3)
        if now - last_fault_log > 2.0:
            st.OnScreenLogMessage("Invalid vehicle state: commands cleared", "Guidance", st.Severity.Warning)
            last_fault_log = now
        continue
    omega_world = current_axes.T @ omega_body
    history.add(now, loc, vel)
    central_gravity = -phobos_mu * loc / max(float(np.linalg.norm(loc)), 1.0)**3
    observed_dv = np.zeros(3) if previous_velocity is None else vel - previous_velocity
    observed_dw = np.zeros(3) if previous_omega_world is None else omega_world - previous_omega_world
    # Learn Mars/other external acceleration only during coast intervals. Reject impact
    # impulses; do not mistake commanded thrust for gravity or filter the truth states.
    if previous_velocity is not None and np.linalg.norm(previous_linear_command) < 1e-9 and dt < .5:
        residual = observed_dv / dt - central_gravity
        if np.linalg.norm(residual - external_residual) < .04:
            external_residual += -math.expm1(-dt / 2.0) * (residual - external_residual)
    gravity = central_gravity + external_residual
    measured_thrust_dv = observed_dv - gravity * dt if previous_velocity is not None else np.zeros(3)
    if np.linalg.norm(previous_linear_command) > 0.0:
        fuel_observer_dv += float(np.linalg.norm(measured_thrust_dv))
        fuel_observer_on_seconds += dt
    fuel_observer_rotation += float(np.linalg.norm(observed_dw))
    previous_velocity, previous_omega_world = vel.copy(), omega_world.copy()
    total_seconds += dt
    if np.linalg.norm(previous_linear_command) == 0.0 and np.linalg.norm(previous_angular_command) == 0.0:
        coast_seconds += dt
    relative_position = target_loc - loc
    relative_velocity = vel - target_vel
    distance = float(np.linalg.norm(relative_position))
    goal = relative_position / max(distance, 1e-9)
    if attitude_reference is None:
        attitude_reference = AccelerationAttitude(current_axes[0])
        roll_reference = current_axes[1].copy()
    # Success is position-only. Latch entry so passing the target does not trigger a
    # costly reversal/flip after the scorer has already accepted the run.
    mission_complete = mission_complete or distance <= model.success_radius
    if now >= next_plan and not mission_complete:
        started = time.perf_counter()
        quiet_coast = False
        try:
            observations = []
            if not no_debris_test_mode:
                contacts = GetRadarContacts(RADAR_COUNT)
                observations = tracker.update(contacts, now, history, gravity)
            if recorder is not None:
                try:
                    recorder.observations(now, observations)
                except Exception as error:
                    st.OnScreenLogMessage(f"Flight recording stopped: {error}", "Guidance", st.Severity.Warning)
                    recorder.close()
                    recorder = None
            nominal = guidance.decide(relative_position, relative_velocity, gravity, now,
                                       fuel_used, roll_reference, nose=current_axes[0])
            if not no_debris_test_mode and not tracker.fresh(now):
                guidance.finish_burn(now)
                stale = now - tracker.last_fresh > model.radar_timeout
                nominal.acceleration = -limited(relative_velocity / 8.0, model.planning_accel) if stale else np.zeros(3)
                nominal.mode = "SENSOR_BRAKE" if stale else "ACQUIRE"
                nominal.burn_seconds, nominal.advantage = 0., 0.
            scene = PredictionScene(tracker.tracks.values(), now, loc, vel, gravity, model,
                                    horizon=min(model.horizon, nominal.eta + 1.0))
            plan = avoidance.choose(scene, nominal.acceleration, relative_velocity, goal, now,
                                    nose=current_axes[0], allow_forward=(nominal.mode not in ("ACQUIRE", "SENSOR_BRAKE")
                                    and np.linalg.norm(omega_body) <= MAX_TURN_RAD_S),
                                    nominal_burn_seconds=nominal.burn_seconds or 8.)
            if plan.mode != "GUIDANCE":
                guidance.finish_burn(now)  # avoidance may preempt any scheduled nominal burn
            avoidance_burn_until = now + plan.burn_seconds if plan.forward_burn else -math.inf
            command = plan.acceleration.copy()
            mode = nominal.mode if plan.mode == "GUIDANCE" else plan.mode
            quiet_coast = (nominal.mode in ("CLEAR_COAST", "FINAL_COAST")
                           and (no_debris_test_mode or tracker.fresh(now)) and np.linalg.norm(command) < 1e-9
                           and plan.encounter.clearance >= 3.)
            if quiet_coast:
                mode = nominal.mode
        except Exception as error:
            guidance.finish_burn(now)
            # Retain recent radar tracks, but discard an unvalidated command. Gentle
            # braking buys observation time; there is no blind beeline fallback.
            command = -limited(relative_velocity / 8.0, model.planning_accel)
            mode = "SENSOR_BRAKE"
            if now - last_fault_log > 2.0:
                st.OnScreenLogMessage(f"Guidance degraded: {error}", "Guidance", st.Severity.Warning)
                last_fault_log = now
        planner_ms = 1000.0 * (time.perf_counter() - started)
        next_plan = now + PLANNING_INTERVAL_S
    if mission_complete:
        guidance.finish_burn(now)
        command = np.zeros(3)
        mode = "ARRIVED"
    forward_ready = ((no_debris_test_mode or tracker.fresh(now))
                     and np.linalg.norm(omega_body) <= MAX_TURN_RAD_S)
    timed_burn = mode in ("TRIM_BURN", "REBOOST") and not mission_complete
    if timed_burn and (not forward_ready or guidance.remaining_burn(now, relative_velocity, gravity) <= 1e-6):
        # Check both elapsed time and observed delivered impulse EVERY control tick,
        # not just at the slower guidance-update rate. Never leave a stale burn on.
        guidance.finish_burn(now)
        command = np.zeros(3)
        mode, timed_burn = "TRIM_WAIT", False
    forward_avoidance = (plan is not None and plan.forward_burn and mode == "AVOID"
                         and now < avoidance_burn_until and not mission_complete
                         and forward_ready)
    expired_avoidance = (plan is not None and plan.forward_burn and mode == "AVOID"
                         and (now >= avoidance_burn_until
                              or not forward_ready))
    full_thrust = (mode in ("ACCELERATE", "REBOOST") or forward_avoidance) and not mission_complete
    pointing_axis = command / max(float(np.linalg.norm(command)), 1e-12) if full_thrust else goal
    nose_error = math.acos(float(np.clip(current_axes[0] @ pointing_axis, -1.0, 1.0)))
    avoiding = mode in ("AVOID", "CAUTION", "EMERGENCY")
    nose_tolerance = pointing_tolerance(distance, avoiding)
    if forward_avoidance:
        # A purposeful shallow turn must not disappear inside the usual broad
        # target-pointing tolerance. This tolerance applies only during the dodge.
        nose_tolerance = math.radians(2.)
    if nose_error <= nose_tolerance:
        alignment_engaged = False
    elif nose_error > nose_tolerance + math.radians(1. if forward_avoidance else 4.):
        alignment_engaged = True
    # Moderate-risk avoidance can keep turning coarsely toward the ship. Urgent
    # evasions still reserve authority for translation and damping existing spin.
    moderate_risk = (avoiding and plan is not None and not plan.urgent
                     and min(plan.baseline.time, plan.encounter.time) >= 8.)
    align_nose = (alignment_engaged and not quiet_coast and not mission_complete
                  and (full_thrust or moderate_risk))
    phase_priority = (1. if full_thrust else .25) if align_nose else 0.
    alignment_priority = alignment_strength(nose_error) * phase_priority
    reference_accel = (command if full_thrust else goal * model.planning_accel) if align_nose else np.zeros(3)
    desired_forward = attitude_reference.update(reference_accel, dt, pointing_error=nose_error,
                                                 priority=phase_priority)
    target_axes = MakeTargetAttitude(desired_forward, roll_reference)
    roll_reference = target_axes[1].copy()
    recovery = np.linalg.norm(omega_body) > 2.0 * MAX_TURN_RAD_S
    alignment_limit = min(angular_limit, .03 if recovery else
                          (.005 + .025 * alignment_priority if full_thrust else (.008 if align_nose else .005)))
    braking_limit = min(angular_limit, .03 if recovery or full_thrust else (.008 if align_nose else .005))
    alpha_body, attitude_error = ComputeAttitudeAcceleration(
        current_axes, target_axes, omega_body, alignment_limit,
        gain_scale=1.0 + 2.0 * alignment_priority if full_thrust else .75,
        rate_limit=math.radians(3.0 + 2.0 * alignment_priority), brake_limit=braking_limit,
        # Drive into the acceptance cone, then disengage above. Using the same
        # tolerance for both would approach its boundary asymptotically forever.
        tolerance=.5 * nose_tolerance)
    if mission_complete:
        alpha_body = np.zeros(3)
    urgent = bool(plan is not None and plan.urgent and not mission_complete)
    transition = full_thrust != was_full_thrust
    if transition:
        linear_pulses.reset()  # don't carry launch impulse debt into a new avoidance direction
    if forward_avoidance:
        # Turn with damped attitude control and request net translation only along
        # the CURRENT nose. The jet allocator still chooses attitude-control jets.
        # The planner checks the corridor between delayed and immediate steering.
        # A .5 request saturates this allocator and suppresses relative turning
        # authority. A request near measured rear authority still gives a full
        # rear burn while retaining the intended angular feedback response.
        rear_request = min(model.planning_accel, model.avoidance_accel, linear_limit)
        linear_world = (current_axes[0] * rear_request
                        if current_axes[0] @ goal >= math.cos(math.radians(35.)) else np.zeros(3))
    elif expired_avoidance:
        linear_world = linear_pulses.command(np.zeros(3), np.zeros(3), dt, now)
    elif timed_burn:
        # Commands are calibrated near delivered RCS authority; the burn's measured
        # impulse cutoff prevents the allocator from stretching a small trim.
        linear_world = command.copy()
    elif full_thrust:
        linear_world = command / max(float(np.linalg.norm(command)), 1e-12) * linear_limit
    else:
        linear_world = linear_pulses.command(command, np.zeros(3) if transition else measured_thrust_dv,
                                             dt, now, urgent=urgent)
    was_full_thrust = full_thrust
    linear_world = limited(linear_world, linear_limit)
    alpha_command = angular_command(alpha_body, omega_body, np.linalg.norm(linear_world) > 0.,
                                     ACTUATOR_PERIOD_S, max(alignment_limit, braking_limit), align=align_nose,
                                     idle_alignment=align_nose)
    if mission_complete or (quiet_coast and not recovery):
        alpha_command = np.zeros(3)
    angular_world = current_axes.T @ alpha_command
    linear_body = st.frames.FramedVector(linear_world, phobosFrame).ExprIn(shuttleFrame)
    SetCommand(linear_body, current_axes @ angular_world)
    linear_switches += int((np.linalg.norm(linear_world) > 0.) != (np.linalg.norm(previous_linear_command) > 0.))
    angular_switches += int((np.linalg.norm(angular_world) > 0.) != (np.linalg.norm(previous_angular_command) > 0.))
    previous_linear_command = linear_world.copy()
    previous_angular_command = angular_world.copy()
    if now >= next_status:
        fuel_used = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        damage = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
        ideal_fuel_per_dv = shuttle.getMass() / (80.0 * 9.8)
        learn_burn_model(model, fuel_used - fuel_observer_fuel, fuel_observer_dv,
                        fuel_observer_rotation, ideal_fuel_per_dv, fuel_observer_on_seconds)
        model.planning_accel = min(model.planning_accel, .5 * linear_limit)
        fuel_observer_fuel, fuel_observer_dv, fuel_observer_rotation = fuel_used, 0.0, 0.0
        fuel_observer_on_seconds = 0.0
        encounter = plan.encounter if plan is not None else None
        st.OnScreenLogMessage(
            f"t={now:6.1f}s {mode:12s} dist={distance:7.1f}m speed={np.linalg.norm(vel):4.1f}m/s "
            f"fuel={fuel_used:6.1f}kg damage={damage:.0f}J tracks={len(tracker.tracks)} "
            f"clearance={encounter.clearance if encounter else math.inf:6.1f}m "
            f"ttc={encounter.time if encounter else math.inf:4.1f}s spin={math.degrees(np.linalg.norm(omega_body)):4.2f}deg/s "
            f"nose={math.degrees(nose_error):4.1f}deg "
            f"coast={coast_seconds / max(total_seconds, .01):.0%} plan={planner_ms:.0f}ms",
            "Guidance", st.Severity.Info)
        if recorder is not None:
            values = dict(t=now, mode=mode, distance=distance, speed=np.linalg.norm(vel), closing_speed=relative_velocity @ goal,
                          no_debris_test=int(no_debris_test_mode),
                          fuel=fuel_used, damage=damage, score_if_arrived=model.score(now, fuel_used, damage),
                          eta=nominal.eta if nominal else math.inf,
                          full_thrust=int(full_thrust), coast_speed=model.coast_speed,
                          guidance_burn_s=guidance.remaining_burn(now, relative_velocity, gravity),
                          guidance_burn_plans=guidance.burn_count,
                          reburn_advantage=nominal.advantage if nominal else 0.,
                          avoidance_strategy=plan.strategy if plan else "GUIDANCE",
                          avoidance_burn_s=max(0., avoidance_burn_until - now) if forward_avoidance else 0.,
                          forward_avoidance=int(forward_avoidance),
                          body_lateral_command=float(np.linalg.norm(np.asarray(linear_body)[1:])),
                          speed_cutoff_reached=int(guidance.coast_latched),
                          velocity_error_deg=(math.degrees(math.acos(float(np.clip(
                              relative_velocity @ goal / np.linalg.norm(relative_velocity), -1., 1.))))
                              if np.linalg.norm(relative_velocity) > 1e-6 else math.nan),
                          nominal_mode=nominal.mode if nominal else "ACQUIRE", forward_command=float(linear_world @ goal),
                          delivered_forward_accel=float(measured_thrust_dv @ goal / dt),
                          thrust_power=float(linear_world @ relative_velocity),
                          braking=int(float(linear_world @ relative_velocity) < -.001),
                          angular_command_norm=float(np.linalg.norm(alpha_command)),
                          nose_error_deg=math.degrees(nose_error), alignment_priority=alignment_priority,
                          nose_tolerance_deg=math.degrees(nose_tolerance), quiet_coast=int(quiet_coast),
                          coast_entry_s=nominal.coast_entry if nominal else math.inf,
                          angular_accel_limit=alignment_limit, angular_brake_limit=braking_limit,
                          tracks=len(tracker.tracks), confirmed=sum(track.hits >= 2 for track in tracker.tracks.values()),
                          radar_age=now - tracker.last_fresh, rejected=tracker.stats.get('rejected', 0),
                          ambiguous=tracker.stats.get('ambiguous', 0), threat_id=encounter.track if encounter else -1,
                          ttc=encounter.time if encounter else math.inf, cpa_time=encounter.closest_time if encounter else math.inf,
                          predicted_miss=encounter.miss_distance if encounter else math.inf,
                          clearance=encounter.clearance if encounter else math.inf, uncertainty=encounter.uncertainty if encounter else 0.,
                          threat_age=encounter.age if encounter else 0., baseline_clearance=plan.baseline.clearance if plan else math.inf,
                          candidates=plan.candidates if plan else 0, attitude_error_deg=math.degrees(attitude_error),
                          spin_deg_s=math.degrees(np.linalg.norm(omega_body)), linear_on=int(np.linalg.norm(linear_world) > 0.),
                          angular_on=int(np.linalg.norm(angular_world) > 0.), linear_switches=linear_switches,
                          angular_switches=angular_switches, coast_fraction=coast_seconds / max(total_seconds, .01),
                          fuel_per_dv=model.fuel_per_dv, planning_accel=model.planning_accel, planner_ms=planner_ms)
            for names, vector in ((('x','y','z'), loc), (('vx','vy','vz'), vel), (('gx','gy','gz'), gravity),
                                  (('nose_x','nose_y','nose_z'), current_axes[0]),
                                  (('target_x','target_y','target_z'), target_loc),
                                  (('target_vx','target_vy','target_vz'), target_vel),
                                  (('desired_ax','desired_ay','desired_az'), command),
                                  (('command_ax','command_ay','command_az'), linear_world)):
                values.update(zip(names, vector))
            try:
                recorder.record(values)
            except Exception as error:
                st.OnScreenLogMessage(f"Flight recording stopped: {error}", "Guidance", st.Severity.Warning)
                recorder.close()
                recorder = None
        next_status = now + STATUS_INTERVAL_S

if recorder is not None:
    recorder.close()
st.leave_sim()
