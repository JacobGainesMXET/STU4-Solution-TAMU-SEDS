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

# ----------------------------------- TUNING CONSTANTS ------------------------------------
CRUISE_SPEED_M_S   = 8.0    # speed toward the target when the path is clear
MAX_SPEED_M_S      = 12.0   # hard cap on commanded velocity
MAX_ACCEL_M_S2     = 0.5    # also limited to the shuttle's actual actuator setting below
ACCEL_GAIN         = 0.5    # 1/s : gentler velocity tracking
SAFETY_BUBBLE_M    = 6.0    # extra clearance on top of shuttle radius + debris radius
INFLUENCE_M        = 45.0   # minimum reaction distance; extended for braking below
AVOID_SPEED_M_S    = 7.0    # velocity added to dodge a rock touching the bubble
BRAKE_DECEL_M_S2   = 0.25   # planning value; reserve acceleration for sideways avoidance
VELOCITY_TOL_M_S   = 0.15   # coast inside this velocity error instead of chasing tiny errors
VELOCITY_FILTER_S = 0.75   # smooth ordinary avoidance changes using simulation time
URGENT_CONTACT_S  = 6.0    # radial time-to-contact hint: bypass smoothing for nearby threats
ATTITUDE_GAIN     = 0.15   # 1/s : attitude error -> desired body angular velocity
MAX_TURN_RAD_S    = math.radians(3.0)
MAX_ANG_ACCEL     = 0.01   # rad/s^2; also limited to the actual actuator setting
ANG_RATE_TIME_S   = 1.5    # time constant for damping angular velocity
ATTITUDE_TOL_RAD  = math.radians(1.5)
ANG_RATE_TOL_RAD_S = math.radians(0.15)
RADAR_COUNT        = 100    # how many nearest radar contacts to process each loop
STATUS_EVERY_N     = 20     # print a status line every N loop iterations
# ------------------------------------------------------------------------------------------


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


def ComputeAttitudeAcceleration(current_axes, target_axes, angular_velocity, accel_limit):
    """Return body angular acceleration and attitude error in radians.

    Each input matrix has body axes in its ROWS, expressed in Phobos coordinates.
    current_axes @ target_axes.T is the desired ACTIVE rotation in current body axes.
    """
    error = RotationVector(current_axes @ target_axes.T)
    angle = float(np.linalg.norm(error))
    if angle <= ATTITUDE_TOL_RAD and np.linalg.norm(angular_velocity) <= ANG_RATE_TOL_RAD_S:
        return np.zeros(3), angle
    desired_rate = ATTITUDE_GAIN * error * max(0.0, 1.0 - ATTITUDE_TOL_RAD / max(angle, 1e-12))
    desired_rate = LimitVector(desired_rate, MAX_TURN_RAD_S)
    acceleration = (desired_rate - angular_velocity) / ANG_RATE_TIME_S
    return LimitVector(acceleration, accel_limit), angle


def ComputeLinearAcceleration(desired_velocity, velocity, previous_desired, dt,
                              accel_limit, urgent=False):
    """Smooth ordinary guidance, but respond immediately to an urgent radar hint."""
    if previous_desired is None or urgent:
        filtered = desired_velocity.copy()
    else:
        alpha = -math.expm1(-max(dt, 0.0) / VELOCITY_FILTER_S)
        filtered = previous_desired + alpha * (desired_velocity - previous_desired)
    error = filtered - velocity
    magnitude = float(np.linalg.norm(error))
    # Subtract the tolerance continuously: no jump in thrust at the coasting boundary.
    error *= max(0.0, 1.0 - VELOCITY_TOL_M_S / max(magnitude, 1e-12))
    return LimitVector(ACCEL_GAIN * error, accel_limit), filtered


def ComputeDesiredVelocity(contacts, vel: npt.NDArray, goal_dir: npt.NDArray,
                           brake_decel: float = BRAKE_DECEL_M_S2,
                           dodge_axis: npt.NDArray | None = None):
    """
    Reactive avoidance. Returns (desired_velocity_in_Phobos_axes, nearest_bubble_gap_m).
    Every rock gets a safety bubble (shuttle radius + rock radius + margin). Rocks whose
    bubble is close push the desired velocity away from them and sideways around them
    (toward the goal side), and rocks blocking the path also slow the shuttle down.
    """
    speed = float(np.linalg.norm(vel))
    travel_dir = vel / speed if speed > 0.5 else goal_dir
    # Plan early enough to brake using achievable acceleration, with room for filtering.
    influence = max(INFLUENCE_M, speed * speed / (2.0 * brake_decel)
                    + speed * VELOCITY_FILTER_S + SAFETY_BUBBLE_M)

    steer = np.zeros(3)
    path_gap = float("inf")        # along-path distance to the first rock blocking the path
    min_gap = float("inf")         # smallest bubble gap of any rock (for status prints)

    for c in contacts:
        d = c.center_range
        if d < 1e-3:
            continue
        u = c.rel_pos / d                                   # unit vector shuttle -> rock center
        r_block = c.radius + SHUTTLE_RADIUS_M + SAFETY_BUBBLE_M
        gap = d - r_block                                   # <0 means inside the bubble
        min_gap = min(min_gap, gap)
        if gap > influence:
            continue

        ahead = float(u @ travel_dir)                       # 1 = dead ahead, -1 = behind
        closing = c.range_rate < -0.1
        if ahead < 0.0 and not closing:
            continue                                        # behind us and not catching up

        # Does this rock sit on our path? If so, remember how far away it is.
        lateral = d * math.sqrt(max(0.0, 1.0 - ahead * ahead))
        blocks_path = ahead > 0.0 and lateral < r_block
        if blocks_path:
            entry = ahead * d - math.sqrt(max(r_block * r_block - lateral * lateral, 0.0))
            path_gap = min(path_gap, max(entry, 0.0))
        elif gap > INFLUENCE_M:
            continue                                        # distant, clear-of-path rocks need no dodge

        # Weight: 0 at the edge of the influence zone, 1 at the bubble, more if inside it.
        reaction_distance = influence if blocks_path else INFLUENCE_M
        prox = 1.0 - max(gap, 0.0) / reaction_distance
        w = prox * prox
        if gap < 0.0:
            w += min(-gap / r_block, 1.0)
        if closing:
            w *= 1.0 + min(-c.range_rate / 5.0, 2.0)
        w *= 0.3 + 0.7 * max(ahead, 0.0)                    # rocks ahead matter most

        # Direction: away from the rock, plus sliding around it toward the goal side.
        t = goal_dir - float(goal_dir @ u) * u
        tn = float(np.linalg.norm(t))
        if tn < 0.25:                                       # rock is dead on the goal line
            # Keep the same preferred side as noisy observations cross the goal line.
            ref = dodge_axis if dodge_axis is not None else np.eye(3)[np.argmin(np.abs(u))]
            t = ref - float(ref @ u) * u
            if np.linalg.norm(t) < 1e-6:
                ref = np.eye(3)[np.argmin(np.abs(u))]
                t = ref - float(ref @ u) * u
            tn = float(np.linalg.norm(t))
        t = t / max(tn, 1e-9)
        steer += w * (0.5 * (-u) + 1.0 * t)

    sn = float(np.linalg.norm(steer))
    if sn > 1.5:
        steer *= 1.5 / sn

    cruise = CRUISE_SPEED_M_S
    if math.isfinite(path_gap):
        allowed = math.sqrt(2.0 * brake_decel * path_gap)
        cruise = min(CRUISE_SPEED_M_S, allowed)

    v_des = cruise * goal_dir + AVOID_SPEED_M_S * steer
    vn = float(np.linalg.norm(v_des))
    if vn > MAX_SPEED_M_S:
        v_des *= MAX_SPEED_M_S / vn
    return v_des, min_gap


###############################
# Get initial simulation time #
###############################

t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0: datetime.datetime = t0_sim.as_datetime()

##################
# Start the loop #
##################

iteration = 0
previous_sim_time = None
previous_desired_velocity = None
roll_reference = None
dodge_axis = None
# Read limits without modifying simulator physics or thruster configuration.
linear_accel_limit = min(MAX_ACCEL_M_S2,
                         shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAccel_m_s2"]))
angular_accel_limit = min(MAX_ANG_ACCEL,
                          shuttle.GetParam(st.VarType.double, ["Thrusters", "MaxAngAccel_rad_s2"]))
brake_decel = min(BRAKE_DECEL_M_S2, 0.5 * linear_accel_limit)
exit_flag: bool = False
while not exit_flag:

    # Wait between loop iterations
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    t_now: datetime.datetime = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    t_from_start: float = (t_now - t0).total_seconds()
    dt = (1.0 / this.GetParam(st.VarType.double, "LoopFreqHz") if previous_sim_time is None
          else t_from_start - previous_sim_time)
    previous_sim_time = t_from_start
    if dt <= 0.0:                                      # paused simulation: don't advance filters
        continue

    # ---------- Vehicle + target state (Phobos-fixed frame) ----------
    locVelAcc = shuttle.getLocVelAcc()
    loc = locVelAcc.loc_WRT_ExprIn(phobosFrame)
    vel = locVelAcc.vel_WRT_ExprIn(phobosFrame)
    # Obtain actual body axes through explicit frame conversions. This avoids guessing
    # whether a quaternion/DCM API represents an active or a passive rotation.
    current_axes = np.vstack([
        st.frames.FramedVector(axis, shuttleFrame).ExprIn(phobosFrame) for axis in np.eye(3)
    ])

    targetState: st.frames.FramedLocVelAcc = target.getLocVelAcc()
    targetLoc: npt.NDArray = targetState.loc_WRT_ExprIn(phobosFrame)

    toTarget = targetLoc - loc
    distToTarget = float(np.linalg.norm(toTarget))
    if distToTarget < 1e-6:
        SetCommand(np.zeros(3), np.zeros(3))
        iteration += 1
        continue
    goalDirection: npt.NDArray = toTarget / distToTarget
    if roll_reference is None:
        roll_reference = current_axes[1].copy()
        dodge_axis = MakeTargetAttitude(goalDirection, roll_reference)[1].copy()

    # ---------- Translational control: goal-seek + radar avoidance ----------
    minGap = float("inf")
    contacts = []
    radar_ok = True
    v_des = vel.copy()                                 # unreadable radar: coast, don't accelerate blind
    try:
        contacts = GetRadarContacts(RADAR_COUNT)         # read the radar ONCE per loop
        v_des, minGap = ComputeDesiredVelocity(contacts, vel, goalDirection, brake_decel, dodge_axis)
        if not np.all(np.isfinite(v_des)):
            raise ValueError("non-finite avoidance command")
    except Exception as e:                               # never let avoidance crash the run
        radar_ok = False
        v_des = vel.copy()
        st.OnScreenLogMessage(f"Avoidance error, coasting this step: {e}", "AvoidErr", st.Severity.Info)

    urgent = (not radar_ok or minGap < 0.0 or any(
        c.time_to_contact(SHUTTLE_RADIUS_M + SAFETY_BUBBLE_M) < URGENT_CONTACT_S
        for c in contacts
    ))
    accel_phobos, previous_desired_velocity = ComputeLinearAcceleration(
        v_des, vel, previous_desired_velocity, dt, linear_accel_limit, urgent
    )
    linearAcceleration_m_s2 = st.frames.FramedVector(accel_phobos, phobosFrame).ExprIn(shuttleFrame)

    # ---------- Stable attitude: face the target, translate sideways to dodge ----------
    target_axes = MakeTargetAttitude(goalDirection, roll_reference)
    roll_reference = target_axes[1].copy()
    current_AngVel = shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame)
    angularAcceleration_rad_s2, attitude_error = ComputeAttitudeAcceleration(
        current_axes, target_axes, current_AngVel, angular_accel_limit
    )

    # ---------- Send commands ----------
    SetCommand(linearAcceleration_m_s2, angularAcceleration_rad_s2)

    # ---------- Status line ----------
    if iteration % STATUS_EVERY_N == 0:
        try:
            damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
            fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
            st.OnScreenLogMessage(
                f"t={t_from_start:6.1f}s  dist={distToTarget:7.1f} m  speed={np.linalg.norm(vel):5.1f} m/s  "
                f"nearest-rock gap={minGap:6.1f} m  damage={damage:.2f}  fuel={fuelUsed:.1f}  "
                f"attitude-error={math.degrees(attitude_error):5.1f} deg  "
                f"spin={math.degrees(float(np.linalg.norm(current_AngVel))):5.2f} deg/s  "
                f"accel={np.linalg.norm(accel_phobos):.3f} m/s^2",
                "Status", st.Severity.Info)
        except Exception:
            pass

    iteration += 1

st.leave_sim()
