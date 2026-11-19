"""Simulator-independent tracking, burn/coast guidance, and collision prediction.

All obstacle inputs must come from radar. Positions use Phobos axes, metres and seconds.
The planner uses a constant-acceleration approximation, not hidden debris state.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def limited(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    return vector * min(1., maximum / max(float(np.linalg.norm(vector)), 1e-12))


def unit(vector, fallback=(1., 0., 0.)):
    length = float(np.linalg.norm(vector))
    return np.asarray(vector, dtype=float) / length if length > 1e-9 else np.array(fallback, dtype=float)


@dataclass
class MissionModel:
    # Verified against the local scoring system; not written to the simulator.
    nominal_time: float = 120.
    nominal_fuel: float = 75.
    max_damage: float = 209200.
    success_radius: float = 50.
    flyby_offset: float = 30.           # aim off the ship centre, still inside its success sphere
    planning_accel: float = .09        # below the measured .096 minimum pure-axis authority
    avoidance_accel: float = .09
    fuel_per_dv: float = 5.6           # kg/(m/s); ideal is mass/(80*9.8), updated from flight
    coast_speed: float | None = None  # initial burn cutoff, never an exact speed-holding target
    cruise_burns: bool = False        # timed course trims and score-qualified speed restoration
    cruise_speed_band: float = .8     # tolerate deceleration before considering another burn
    cruise_coast_seconds: float = 8.  # minimum coast between optional speed-restoration burns
    reburn_score_margin: float = .25
    position_sigma: float = 2.
    process_accel_sigma: float = .015   # tuning allowance for unmodelled relative acceleration
    uncertainty_sigma: float = 2.5
    ship_radius: float = 2.5           # existing geometry estimate; still needs hull validation
    clearance: float = 3.
    horizon: float = 30.
    track_timeout: float = 12.
    radar_timeout: float = 3.
    avoidance_turn_rate: float = math.radians(.25)  # conservative allocated-attitude forecast
    avoidance_turn_delay: float = 3.              # allow time for damped attitude response

    def score(self, elapsed, fuel, damage=0.):
        time_score = self.nominal_time / elapsed if elapsed > 0. else 0.
        return 50. + 100. * (time_score - max(fuel / self.nominal_fuel, 0.) - 2. * damage / self.max_damage)


def learn_burn_model(model, fuel_delta, delivered_dv, angular_delta, ideal_cost, on_seconds=0.):
    """Calibrate the score estimate on low-rotation burn windows, never on coasting.

    Include actual allocated propellant, not the commanded acceleration magnitude.
    """
    if delivered_dv > .03 and angular_delta < .01 and fuel_delta > 0.:
        if on_seconds > 0. and not .03 < delivered_dv / on_seconds < .25:
            return
        measured = float(np.clip(fuel_delta / delivered_dv, ideal_cost, 3. * ideal_cost))
        model.fuel_per_dv = .9 * model.fuel_per_dv + .1 * measured
        # Use most of observed burn authority instead of unnecessarily throttling a
        # good burn. Keep 5% reserve and reject impact-sized acceleration estimates.
        if on_seconds > .2 and .03 < delivered_dv / on_seconds < .25:
            available = min(.16, .95 * delivered_dv / on_seconds)
            model.planning_accel = .8 * model.planning_accel + .2 * available


class StateHistory:
    """Own-ship truth history, used to put each OLD radar sample into world coordinates."""
    def __init__(self):
        self.samples = deque()

    def add(self, time, position, velocity):
        if self.samples and time <= self.samples[-1][0]:
            return
        self.samples.append((time, np.array(position), np.array(velocity)))
        while len(self.samples) > 2 and self.samples[1][0] < time - 20.:
            self.samples.popleft()

    def at(self, time):
        if not self.samples or time < self.samples[0][0] - .5 or time > self.samples[-1][0] + .2:
            return None
        times = [sample[0] for sample in self.samples]
        index = int(np.searchsorted(times, time))
        if index == 0 or index == len(times):
            stamp, position, velocity = self.samples[min(index, len(times) - 1)]
            return position + velocity * (time - stamp), velocity.copy()
        t0, p0, v0 = self.samples[index - 1]
        t1, p1, v1 = self.samples[index]
        weight = (time - t0) / (t1 - t0)
        return p0 + weight * (p1 - p0), v0 + weight * (v1 - v0)


def predict_state(state, covariance, dt, gravity, process_sigma):
    transition = np.eye(6)
    transition[:3, 3:] = np.eye(3) * dt
    predicted = transition @ state
    predicted[:3] += .5 * gravity * dt * dt
    predicted[3:] += gravity * dt
    noise_map = np.vstack((np.eye(3) * .5 * dt * dt, np.eye(3) * dt))
    uncertainty = transition @ covariance @ transition.T + process_sigma**2 * noise_map @ noise_map.T
    return predicted, uncertainty


def measurement_update(state, covariance, matrix, value, noise):
    innovation = matrix @ covariance @ matrix.T + noise
    gain = np.linalg.solve(innovation, matrix @ covariance).T
    updated = state + gain @ (value - matrix @ state)
    identity = np.eye(6) - gain @ matrix
    # Joseph form preserves positive covariance despite very accurate radial velocities.
    uncertainty = identity @ covariance @ identity.T + gain @ noise @ gain.T
    return updated, .5 * (uncertainty + uncertainty.T)


@dataclass
class DebrisTrack:
    number: int
    time: float
    state: np.ndarray
    covariance: np.ndarray
    radius: float
    hits: int = 1
    reliable_hits: int | None = None

    def __post_init__(self):
        if self.reliable_hits is None:
            self.reliable_hits = self.hits

    def predict(self, time, gravity, model):
        return predict_state(self.state, self.covariance, max(0., time - self.time),
                             gravity, model.process_accel_sigma)


class RadarTracker:
    """Gated one-to-one association: radar array indices are never object identities.

    A position/velocity Kalman filter uses 3-D position history plus the independent
    radial-velocity measurement. A single range rate is NOT treated as a 3-D velocity.
    """
    def __init__(self, model):
        self.model = model
        self.tracks = {}
        self.seen = {}
        self.next_number = 1
        self.last_fresh = -math.inf
        self.stats = {}

    def update(self, contacts, now, history, gravity):
        self.tracks = {key: track for key, track in self.tracks.items()
                       if now - track.time <= self.model.track_timeout}
        self.seen = {key: stamp for key, stamp in self.seen.items() if now - stamp <= 16.}
        self.stats = dict(new=0, matched=0, created=0, rejected=0, ambiguous=0)
        observations = []
        for contact in contacts:
            values = np.r_[contact.rel_pos, contact.surface_range, contact.range_rate,
                           contact.diameter, contact.t_meas]
            if (not np.all(np.isfinite(values)) or contact.diameter <= 0.
                    or contact.t_meas < 0. or contact.t_meas > now + .2
                    or now - contact.t_meas > self.model.track_timeout):
                self.stats['rejected'] += 1
                continue
            key = tuple(values)
            if key in self.seen:
                continue
            self.seen[key] = contact.t_meas
            ship_state = history.at(contact.t_meas)
            distance = float(np.linalg.norm(contact.rel_pos))
            if ship_state is None or distance < 1e-3:
                self.stats['rejected'] += 1
                continue
            # Independent centre/surface observations should agree within their noise.
            tolerance = 10. + .15 * contact.diameter
            if abs(distance - .5 * contact.diameter - contact.surface_range) > tolerance:
                self.stats['rejected'] += 1
                continue
            ship_position, ship_velocity = ship_state
            observations.append((contact, ship_position + contact.rel_pos, ship_velocity,
                                 contact.rel_pos / distance))
            self.last_fresh = max(self.last_fresh, contact.t_meas)
        self.stats['new'] = len(observations)
        # Associate against the tracks that existed BEFORE this batch. Otherwise the
        # 500 simultaneous initial measurements could merge unrelated nearby rocks.
        tracks = list(self.tracks.values())
        pairs = []
        ambiguous = set()
        if tracks:
            states = np.array([track.state for track in tracks])
            covariances = np.array([track.covariance for track in tracks])
            timestamps = np.array([track.time for track in tracks])
            diameters = np.array([2. * track.radius for track in tracks])
        for index, (contact, position, ship_velocity, ray) in enumerate(observations):
            candidates = []
            if tracks:
                dt = np.maximum(0., contact.t_meas - timestamps)
                predicted_position = states[:, :3] + states[:, 3:] * dt[:, None] + .5 * gravity * dt[:, None]**2
                position_variance = (np.diagonal(covariances[:, :3, :3], axis1=1, axis2=2)
                                     + 2. * dt[:, None] * np.diagonal(covariances[:, :3, 3:], axis1=1, axis2=2)
                                     + dt[:, None]**2 * np.diagonal(covariances[:, 3:, 3:], axis1=1, axis2=2)
                                     + .25 * self.model.process_accel_sigma**2 * dt[:, None]**4)
                scores = np.sum((position - predicted_position)**2 /
                                np.maximum(position_variance + self.model.position_sigma**2, 1e-6), axis=1)
                radial_errors = contact.range_rate - (states[:, 3:] + gravity * dt[:, None] - ship_velocity) @ ray
                radial_variances = (np.einsum('i,nij,j->n', ray, covariances[:, 3:, 3:], ray)
                                    + self.model.process_accel_sigma**2 * dt**2 + .25)
                valid = ((contact.t_meas > timestamps + 1e-6) & (scores <= 16.)
                         & (np.abs(contact.diameter - diameters) <= np.maximum(1., .25 * np.maximum(diameters, contact.diameter)))
                         & (radial_errors**2 <= 16. * radial_variances))
                for track_index in np.flatnonzero(valid):
                    score = scores[track_index] + radial_errors[track_index]**2 / radial_variances[track_index]
                    candidates.append((float(score), index, tracks[track_index].number))
            candidates.sort()
            if len(candidates) > 1 and candidates[1][0] < candidates[0][0] + 2.:
                ambiguous.add(index)
            pairs.extend(candidates)
        assigned = {}
        used_tracks = set()
        for score, index, number in sorted(pairs):
            if index not in assigned and number not in used_tracks:
                assigned[index] = number
                used_tracks.add(number)
        for index, (contact, position, ship_velocity, ray) in enumerate(observations):
            if index in assigned:
                track = self.tracks[assigned[index]]
                state, covariance = track.predict(contact.t_meas, gravity, self.model)
                track.hits += 1
                self.stats['matched'] += 1
                if index in ambiguous:
                    covariance[3:, 3:] += np.eye(3) * .5**2
                    self.stats['ambiguous'] += 1
                    track.reliable_hits = 0
                else:
                    track.reliable_hits += 1
            else:
                # Unknown transverse velocity gets a broad prior, not a fictitious
                # radar-derived vector. Subsequent position observations resolve it.
                state = np.r_[position, ship_velocity + contact.range_rate * ray]
                covariance = np.diag([1e6] * 3 + [16.] * 3)
                track = DebrisTrack(self.next_number, contact.t_meas, state, covariance, .5 * contact.diameter)
                self.tracks[track.number] = track
                self.next_number += 1
                self.stats['created'] += 1
            position_matrix = np.column_stack((np.eye(3), np.zeros((3, 3))))
            state, covariance = measurement_update(state, covariance, position_matrix, position,
                                                   np.eye(3) * self.model.position_sigma**2)
            velocity_matrix = np.r_[np.zeros(3), ray][None, :]
            # Bearing noise couples transverse motion into the radial observation.
            bearing_noise = 2. / max(contact.center_range, 2.) * np.linalg.norm(state[3:] - ship_velocity)
            radial_noise = .08**2 + bearing_noise**2
            if index not in ambiguous:
                # The simulator independently perturbs surface range and 3D position.
                # Linearize the range at the measured ray. Diameter uncertainty is
                # part of this measurement's noise, especially for the larger rocks.
                range_matrix = np.r_[ray, np.zeros(3)][None, :]
                centre_range = contact.surface_range + .5 * contact.diameter
                range_var = self.model.position_sigma**2 + (.025 * contact.diameter)**2
                ship_origin = position - contact.rel_pos
                state, covariance = measurement_update(
                    state, covariance, range_matrix, np.array([ray @ ship_origin + centre_range]),
                    np.array([[range_var]]))
                state, covariance = measurement_update(
                    state, covariance, velocity_matrix,
                    np.array([contact.range_rate + ship_velocity @ ray]), np.array([[radial_noise]]))
            track.state, track.covariance, track.time = state, covariance, contact.t_meas
            track.radius = .8 * track.radius + .2 * (.5 * contact.diameter)
        return [item[0] for item in observations]

    def ready(self, now):
        confirmed = sum(track.hits >= 2 for track in self.tracks.values())
        return bool(self.tracks) and confirmed >= .6 * len(self.tracks) and now - self.last_fresh < self.model.radar_timeout

    def fresh(self, now):
        """Initial observations can protect launch while transverse tracks mature."""
        return bool(self.tracks) and now - self.last_fresh < self.model.radar_timeout


def travel_time(distance, speed, acceleration):
    if distance <= 0.:
        return 0.
    if abs(acceleration) < 1e-9:
        return distance / speed if speed > 1e-6 else math.inf
    discriminant = speed * speed + 2. * acceleration * distance
    if discriminant < 0.:
        return math.inf
    roots = [(-speed + sign * math.sqrt(discriminant)) / acceleration for sign in (-1., 1.)]
    return min((root for root in roots if root >= 0.), default=math.inf)


@dataclass
class GuidanceDecision:
    acceleration: np.ndarray
    mode: str
    eta: float
    advantage: float = 0.
    burn_seconds: float = 0.
    coast_entry: float = math.inf


def ballistic_entry(relative_position, relative_velocity, gravity, radius, horizon):
    """First predicted entry into a target sphere under constant external acceleration.

    Uses trajectory direction, not nose direction. Positive relative_velocity is
    ship minus target velocity; target acceleration is assumed negligible.
    """
    r, v, g = map(lambda value: np.asarray(value, dtype=float),
                  (relative_position, relative_velocity, gravity))
    if not np.all(np.isfinite(np.r_[r, v, g])) or radius <= 0. or horizon <= 0.:
        return math.inf
    if r @ r <= radius * radius:
        return 0.
    coefficients = np.array([.25 * (g @ g), v @ g, v @ v - r @ g,
                             -2. * (r @ v), r @ r - radius * radius])
    # Drop only exact leading zeros; a small gravity term may matter over a long coast.
    coefficients = np.trim_zeros(coefficients, 'f')
    if len(coefficients) < 2:
        return math.inf
    roots = np.roots(coefficients)
    candidates = [float(root.real) for root in roots
                  if abs(root.imag) < 1e-6 and 0. <= root.real <= horizon]
    return min(candidates, default=math.inf)


def finite_burn_entry(relative_position, relative_velocity, gravity, acceleration, duration,
                      radius, horizon=600.):
    """Arrival during a finite burn or its subsequent gravity-affected coast."""
    r, v, g, a = map(lambda value: np.asarray(value, dtype=float),
                     (relative_position, relative_velocity, gravity, acceleration))
    if not math.isfinite(duration) or not math.isfinite(horizon) or horizon <= 0.:
        return math.inf
    if np.linalg.norm(r) <= radius:
        return 0.
    duration = min(max(float(duration), 0.), horizon)
    during = ballistic_entry(r, v, g + a, radius, duration)
    if math.isfinite(during):
        return during
    after_r = r - v * duration - .5 * (g + a) * duration**2
    after_v = v + (g + a) * duration
    return duration + ballistic_entry(after_r, after_v, g, radius, horizon - duration)


@dataclass
class ScheduledBurn:
    acceleration: np.ndarray
    start: float
    initial_velocity: np.ndarray
    gravity: np.ndarray
    duration: float
    mode: str
    advantage: float = 0.

    def remaining(self, now, velocity, gravity):
        elapsed = max(0., now - self.start)
        magnitude = float(np.linalg.norm(self.acceleration))
        delivered = np.asarray(velocity) - self.initial_velocity - .5 * (self.gravity + gravity) * elapsed
        delivered = float(delivered @ unit(self.acceleration))
        impulse_left = max(0., magnitude * self.duration - delivered)
        return max(0., min(self.duration - elapsed, impulse_left / max(magnitude, 1e-9)))


class AccelerationGuidance:
    """Launch to cruise speed, coast, and schedule finite corrections when needed.

    Acceleration here predicts delivered motion. Controller.py sends a full linear
    command when this nominal path is accepted. This is not a speed-holding servo.
    """
    def __init__(self, model):
        self.model = model
        self.offset = None
        self.coast_latched = False
        self.ballistic_coasting = False
        self.burn = None
        self.last_burn_end = -math.inf
        self.coast_started = None
        self.burn_count = 0
        self.reburn_advantage = 0.

    def finish_burn(self, now):
        if self.burn is not None:
            self.last_burn_end = now
            self.coast_started = now
        self.burn = None

    def remaining_burn(self, now, velocity, gravity):
        return self.burn.remaining(now, velocity, gravity) if self.burn is not None else 0.

    def _start_burn(self, acceleration, duration, mode, now, velocity, gravity, advantage=0.):
        self.burn = ScheduledBurn(np.array(acceleration), now, np.array(velocity), np.array(gravity),
                                  duration, mode, advantage)
        self.burn_count += 1
        self.coast_started = None

    def _burn_decision(self, r, v, g, now, eta):
        remaining = self.remaining_burn(now, v, g)
        if remaining <= 1e-6:
            self.finish_burn(now)
            return None
        burn = self.burn
        return GuidanceDecision(burn.acceleration.copy(), burn.mode, eta,
                                advantage=burn.advantage, burn_seconds=remaining)

    def _speed_burn(self, r, v, g, now, fuel, coast_entry, radius, nose):
        model = self.model
        if nose is not None and unit(nose) @ unit(v) < math.cos(math.radians(8.)):
            # An optional top-up must not incur a large turn or become a sidestep.
            self.finish_burn(now)
            return None
        if self.burn is not None:
            if self.burn.mode == 'REBOOST' and float(v @ unit(r)) < model.coast_speed:
                remaining = self.remaining_burn(now, v, g)
                eta = finite_burn_entry(r, v, g, self.burn.acceleration, remaining, radius)
                if math.isfinite(eta):
                    decision = self._burn_decision(r, v, g, now, eta)
                    if decision is not None:
                        return decision
            self.finish_burn(now)
        if self.coast_started is None:
            self.coast_started = now
        closing = float(v @ unit(r))
        if (coast_entry <= 20. or closing >= model.coast_speed - model.cruise_speed_band
                or now - self.coast_started < model.cruise_coast_seconds
                or now - self.last_burn_end < model.cruise_coast_seconds):
            return None
        direction = unit(v)
        authority = min(model.planning_accel, model.avoidance_accel)
        net_forward = float((authority * direction + g) @ unit(r))
        if net_forward <= .001:
            return None
        to_target_speed = (model.coast_speed - closing) / net_forward
        durations = sorted({min(duration, to_target_speed, 8.) for duration in (2., 4., 8.)})
        best = None
        self.reburn_advantage = -math.inf
        for duration in durations:
            if duration < .05:
                continue
            acceleration = authority * direction
            eta = finite_burn_entry(r, v, g, acceleration, duration, radius)
            if not math.isfinite(eta):
                continue
            burn_fuel = model.fuel_per_dv * authority * min(duration, eta)
            advantage = model.score(now + eta, fuel + burn_fuel) - model.score(now + coast_entry, fuel)
            self.reburn_advantage = max(self.reburn_advantage, advantage)
            if advantage > model.reburn_score_margin and (best is None or advantage > best[0]):
                best = (advantage, acceleration, duration, eta)
        if best is None:
            return None
        advantage, acceleration, duration, eta = best
        self._start_burn(acceleration, duration, 'REBOOST', now, v, g, advantage)
        return self._burn_decision(r, v, g, now, eta)

    def _course_burn(self, r, v, g, now, delta_v, eta, radius):
        if self.burn is not None:
            if self.burn.mode == 'TRIM_BURN' and self.burn.acceleration @ delta_v > 0.:
                decision = self._burn_decision(r, v, g, now, eta)
                if decision is not None:
                    return decision
            self.finish_burn(now)
        # Allow a short observation gap after a completed trim, unless arrival is
        # close. Geometric entry hysteresis supplies the normal correction deadband.
        if now - self.last_burn_end < 1.5 and eta > 15.:
            return GuidanceDecision(np.zeros(3), 'TRIM_WAIT', eta)
        authority = min(self.model.planning_accel, self.model.avoidance_accel)
        if authority <= 0. or np.linalg.norm(delta_v) < 1e-9:
            return GuidanceDecision(np.zeros(3), 'TRIM_WAIT', eta)
        acceleration = authority * unit(delta_v)
        candidates = []
        for fraction in (.25, .5, .75, 1.):
            duration = float(np.clip(fraction * np.linalg.norm(delta_v) / authority, .05, 8.))
            entry = finite_burn_entry(r, v, g, acceleration, duration, radius)
            candidates.append((duration, entry))
        safe = [(duration, entry) for duration, entry in candidates if math.isfinite(entry)]
        duration, entry = min(safe) if safe else candidates[-1]
        self._start_burn(acceleration, duration, 'TRIM_BURN', now, v, g)
        return self._burn_decision(r, v, g, now, entry if math.isfinite(entry) else eta)

    def decide(self, relative_position, relative_velocity, gravity, elapsed, fuel, reference, *, nose=None):
        model = self.model
        cruise = model.cruise_burns and model.coast_speed is not None
        if np.linalg.norm(relative_position) <= model.success_radius:
            self.finish_burn(elapsed)
            return GuidanceDecision(np.zeros(3), 'ARRIVED', 0.)
        approaching = float(relative_position @ relative_velocity) > 0.
        # Remember reaching the cutoff so gravity slowing a safe coast below it
        # does not repeatedly restart the main burn. A changed trajectory still
        # has to pass the arrival test below, or receive a course correction.
        if not approaching or model.coast_speed is None:
            self.coast_latched = False
        elif (float(relative_velocity @ unit(relative_position)) if cruise
              else np.linalg.norm(relative_velocity)) >= model.coast_speed - (.05 if cruise else 0.):
            self.coast_latched = True
        # Enter with 8 m radial margin; retain coast with 4 m margin and a larger
        # prediction window to avoid thrust chatter at the edge of the capture test.
        # At the speed cutoff, allow a long coast: the old 90 s limit alone could
        # otherwise force continued acceleration even on an aligned 8 m/s path.
        capture_radius = model.success_radius - (4. if self.ballistic_coasting else 8.)
        horizon = 600. if self.coast_latched else (120. if self.ballistic_coasting else 90.)
        coast_entry = ballistic_entry(relative_position, relative_velocity, gravity,
                                       capture_radius, horizon)
        self.ballistic_coasting = (math.isfinite(coast_entry) and approaching
                                  and (not cruise or self.coast_latched or coast_entry <= 8.))
        if self.ballistic_coasting:
            if cruise and coast_entry > 8.:
                decision = self._speed_burn(relative_position, relative_velocity, gravity,
                                             elapsed, fuel, coast_entry, capture_radius, nose)
                if decision is not None:
                    return decision
            else:
                self.finish_burn(elapsed)
            return GuidanceDecision(np.zeros(3), 'FINAL_COAST' if coast_entry <= 8. else 'CLEAR_COAST',
                                    coast_entry, advantage=self.reburn_advantage, coast_entry=coast_entry)
        if self.offset is None:
            axis = unit(relative_position)
            side = reference - float(reference @ axis) * axis
            if np.linalg.norm(side) < 1e-6:
                side = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
            self.offset = unit(side) * min(model.flyby_offset, .6 * model.success_radius)
        aim = relative_position + self.offset
        direction = unit(aim)
        aim_radius = max(2., model.success_radius - np.linalg.norm(self.offset) - 2.)
        distance = max(0., float(np.linalg.norm(aim)) - aim_radius)
        speed = float(relative_velocity @ direction)
        g_along = float(gravity @ direction)
        coast_eta = travel_time(distance, speed, g_along)
        powered_eta = travel_time(distance, speed, g_along + model.planning_accel)
        lateral_velocity = relative_velocity - speed * direction
        lateral_gravity = gravity - g_along * direction
        # If gravity would stop forward progress short of the target, or arrival
        # is beyond the bounded forecast, more forward impulse is still needed.
        if not math.isfinite(coast_eta) or coast_eta > 600.:
            self.coast_latched = False
        eta = coast_eta if self.coast_latched else powered_eta
        eta = float(np.clip(eta, 1., 600.))
        acceleration = np.zeros(3) if self.coast_latched else model.planning_accel * direction
        # Correct a substantial lateral arrival miss without subtracting forward speed.
        # After the cutoff, use the full coast ETA and steer toward the existing
        # flyby aim point. Do not keep adding forward thrust to fix a lateral miss.
        correction_eta = eta if self.coast_latched else min(eta, 120.)
        lateral_miss = lateral_velocity * correction_eta + .5 * lateral_gravity * correction_eta**2
        if cruise and self.coast_latched:
            delta_v = -(lateral_velocity + .5 * lateral_gravity * correction_eta)
            return self._course_burn(relative_position, relative_velocity, gravity, elapsed,
                                      delta_v, eta, capture_radius)
        if cruise:
            self.finish_burn(elapsed)
            self.coast_started = None
        if self.coast_latched or np.linalg.norm(lateral_miss) > aim_radius:
            correction = -(lateral_velocity + .5 * lateral_gravity * correction_eta) / max(3., .25 * correction_eta)
            acceleration += limited(correction, .4 * model.planning_accel)
        # All unpowered coasts are qualified by the 3D entry test above.
        mode = 'CORRECT' if self.coast_latched else 'ACCELERATE'
        return GuidanceDecision(limited(acceleration, model.planning_accel), mode, eta)


def alignment_strength(angle):
    """Smoothly increase alignment priority between 5 and 60 degrees of nose error.

    Bounded at both ends: there is no singular 1/angle gain near correct alignment.
    """
    fraction = float(np.clip((angle - math.radians(5.)) / math.radians(55.), 0., 1.))
    return fraction * fraction * (3. - 2. * fraction)


def pointing_tolerance(distance, avoiding=False):
    """Attitude tolerance only: never relax the geometric arrival/collision tests."""
    near_fraction = float(np.clip((500. - distance) / 400., 0., 1.))
    degrees = 8. + 10. * near_fraction
    return math.radians(max(degrees, 12. if avoiding else 8.))


def angular_command(desired_alpha, omega_body, linear_on, actuator_period, limit, *, align=True,
                    idle_alignment=False):
    """Use gentle spin damping when translation is off, without a minimum torque pulse.

    The installed allocator treats solved activity as on-time. In its unsaturated
    regime an activity is divided by its tick duration to obtain effective acceleration.
    Attenuating idle torque by that period is a feedback-gain calibration, not a
    change to the allocator. It prevents tiny errors from demanding full-tick firings.
    During translation, shared allocation already limits the response.
    """
    if linear_on and align:
        return limited(desired_alpha, limit)
    if idle_alignment and align:
        return limited(desired_alpha, limit) * actuator_period
    if np.linalg.norm(omega_body) <= math.radians(.15):
        return np.zeros(3)
    damping = limited(-np.asarray(omega_body) / 2., limit)
    return damping if linear_on else damping * actuator_period


@dataclass
class Encounter:
    clearance: float = math.inf
    time: float = math.inf
    track: int = -1
    miss_distance: float = math.inf
    uncertainty: float = 0.
    age: float = 0.
    closest_time: float = math.inf


def burn_displacement(times, acceleration, burn_seconds=8., initial_direction=None,
                      turn_rate=math.radians(.25), turn_delay=3.):
    """Finite burn followed by coast, optionally with a delayed change of heading.

    The heading blend approximates a damped turn, rather than instantaneous sideways
    thrust. Integrate its linear acceleration ramp analytically, including burns
    that finish before the turn does. This is a forecast, not a thruster model.
    """
    times = np.asarray(times, dtype=float)
    acceleration = np.asarray(acceleration, dtype=float)
    powered = np.minimum(times, burn_seconds)
    impulse_time = .5 * powered**2 + powered * (times - powered)
    instant = impulse_time[:, None] * acceleration
    magnitude = float(np.linalg.norm(acceleration))
    if initial_direction is None or magnitude < 1e-9:
        return instant, instant
    initial = magnitude * unit(initial_direction)
    angle = math.acos(float(np.clip(initial @ acceleration / magnitude**2, -1., 1.)))
    if angle < 1e-6:
        return instant, instant
    turn_time = angle / max(turn_rate, 1e-6)
    after_delay = np.maximum(powered - turn_delay, 0.)
    after_turn = np.maximum(after_delay - turn_time, 0.)
    ramp_position = (after_delay**3 - after_turn**3) / (6. * turn_time)
    ramp_velocity = (after_delay**2 - after_turn**2) / (2. * turn_time)
    ramp = ramp_position + (times - powered) * ramp_velocity
    delayed = impulse_time[:, None] * initial + ramp[:, None] * (acceleration - initial)
    return delayed, instant


class PredictionScene:
    """Vectorized finite-burn/then-coast paths, including propagated uncertainty.

    Half-second sampling gets a relative-speed margin to cover between-sample misses.
    Unconfirmed tracks use an 8 s horizon while their transverse velocity is unknown.
    """
    def __init__(self, tracks, now, ship_position, ship_velocity, gravity, model, horizon=None):
        self.model = model
        self.tracks = list(tracks)
        self.times = np.arange(0., min(model.horizon, horizon or model.horizon) + .25, .5)
        states, covariances = [], []
        for track in self.tracks:
            state, covariance = track.predict(now, gravity, model)
            states.append(state)
            covariances.append(covariance)
        if not states:
            self.position = np.empty((0, 3))
            return
        states, self.covariances = np.array(states), np.array(covariances)
        self.position = states[:, :3] - ship_position
        self.velocity = states[:, 3:] - ship_velocity
        self.radii = np.array([track.radius * 1.15 + model.ship_radius + model.clearance for track in self.tracks])
        self.valid = self.times[None, :] <= np.array([
            min(model.horizon, 8. + 8. * (track.hits - 1)) for track in self.tracks])[:, None]
        self.ages = np.array([now - track.time for track in self.tracks])
        t = self.times[None, :, None, None]
        self.future_covariance = (self.covariances[:, None, :3, :3]
                                  + t * (self.covariances[:, None, :3, 3:] + self.covariances[:, None, 3:, :3])
                                  + t**2 * self.covariances[:, None, 3:, 3:])
        # Broad-phase bound covers EVERY candidate, including its largest possible
        # displacement and the largest covariance eigenvalue (Gershgorin bound).
        # Far safe rocks need not be reevaluated for each lateral direction.
        time = self.times[None, :]
        powered = np.minimum(time, 8.)
        maximum_accel = max(model.planning_accel, model.avoidance_accel)
        maximum_displacement = maximum_accel * (.5 * powered**2 + powered * np.maximum(time - 8., 0.))
        maximum_variance = np.max(np.sum(np.abs(self.future_covariance), axis=3), axis=2)
        maximum_variance += .25 * model.process_accel_sigma**2 * time**4
        maximum_margin = model.uncertainty_sigma * np.sqrt(np.maximum(maximum_variance, 0.))
        maximum_margin += .25 * (np.linalg.norm(self.velocity, axis=1)[:, None] + maximum_accel * powered)
        unforced_distance = np.linalg.norm(self.position[:, None, :] + self.velocity[:, None, :] * time[:, :, None], axis=2)
        relevant = np.any((unforced_distance <= self.radii[:, None] + maximum_margin + maximum_displacement) & self.valid, axis=1)
        self.tracks = [track for track, keep in zip(self.tracks, relevant) if keep]
        for name in ('position', 'velocity', 'radii', 'valid', 'ages', 'future_covariance'):
            setattr(self, name, getattr(self, name)[relevant])

    def assess(self, acceleration, burn_seconds=8., initial_direction=None):
        if not len(self.position):
            return Encounter()
        if not 0. <= burn_seconds <= 8.:
            raise ValueError('Avoidance burn must fit the 8 second prediction envelope')
        time = self.times
        powered = np.minimum(time, burn_seconds)
        delayed, instant = burn_displacement(time, acceleration, burn_seconds, initial_direction,
                                             self.model.avoidance_turn_rate, self.model.avoidance_turn_delay)
        separation = self.position[:, None, :] + self.velocity[:, None, :] * time[None, :, None]
        separation -= delayed[None, :, :]
        # Check the swept corridor between an immediate turn and the delayed turn.
        # Either timing can be unsafe; do not select a dodge that relies on instant
        # attitude response or misses a different rock halfway through the turn.
        sweep = instant - delayed
        fraction = np.clip(np.einsum('nti,ti->nt', separation, sweep)
                           / np.maximum(np.sum(sweep**2, axis=1), 1e-12)[None, :], 0., 1.)
        separation -= fraction[:, :, None] * sweep[None, :, :]
        distance = np.linalg.norm(separation, axis=2)
        normal = separation / np.maximum(distance[:, :, None], 1e-9)
        variance = np.einsum('nti,ntij,ntj->nt', normal, self.future_covariance, normal)
        variance += .25 * self.model.process_accel_sigma**2 * time[None, :]**4
        uncertainty = self.model.uncertainty_sigma * np.sqrt(np.maximum(variance, 0.))
        # Cover the distance to the nearest sample even at the largest segment speed.
        sample_margin = .25 * (np.linalg.norm(self.velocity, axis=1)[:, None]
                               + np.linalg.norm(acceleration) * powered[None, :])
        clearance = distance - self.radii[:, None] - uncertainty - sample_margin
        clearance[~self.valid] = math.inf
        row, column = np.unravel_index(np.argmin(clearance), clearance.shape)
        dangerous = clearance < 0.
        first_time = float(time[np.flatnonzero(np.any(dangerous, axis=0))[0]]) if np.any(dangerous) else math.inf
        return Encounter(float(clearance[row, column]), first_time, self.tracks[row].number,
                         float(distance[row, column]), float(uncertainty[row, column]), float(self.ages[row]), float(time[column]))


@dataclass
class AvoidanceDecision:
    acceleration: np.ndarray
    mode: str
    encounter: Encounter
    baseline: Encounter
    urgent: bool = False
    candidates: int = 1
    forward_burn: bool = False
    burn_seconds: float = 8.
    strategy: str = 'GUIDANCE'


class AvoidancePlanner:
    def __init__(self, model):
        self.model = model
        self.active = False
        self.previous = np.zeros(3)
        self.safe_since = None
        self.forward_burn = False
        self.burn_until = -math.inf

    def choose(self, scene, nominal, velocity, goal, now, *, nose=None, allow_forward=True,
               nominal_burn_seconds=8., rear_guidance=False):
        baseline = (scene.assess(nominal, nominal_burn_seconds, initial_direction=nose)
                    if rear_guidance else scene.assess(nominal, nominal_burn_seconds))
        goal_axis = unit(goal)
        if baseline.clearance >= 0.:
            if not self.active:
                return AvoidanceDecision(nominal, 'GUIDANCE', baseline, baseline)
            if baseline.clearance > 3.:
                self.safe_since = now if self.safe_since is None else self.safe_since
                if self.forward_burn:
                    # Retain the maneuver's direction history for hysteresis,
                    # without burning merely to finish an already unnecessary turn.
                    if now - self.safe_since >= 1.5:
                        self.active = False
                    return AvoidanceDecision(nominal, 'GUIDANCE', baseline, baseline)
            else:
                self.safe_since = None
            if self.safe_since is not None and now - self.safe_since >= 1.5:
                self.active = False
                return AvoidanceDecision(nominal, 'GUIDANCE', baseline, baseline)
        else:
            self.safe_since = None
        # Keep an accepted short forward burn stable, but never carry it past its
        # forecast duration without checking the new state and radar observations.
        nose_forward = allow_forward and (nose is None or unit(nose) @ goal_axis >= math.cos(math.radians(35.)))
        if self.active and self.forward_burn and now < self.burn_until and nose_forward:
            remaining = min(8., self.burn_until - now)
            encounter = scene.assess(self.previous, remaining, initial_direction=nose)
            if encounter.clearance >= 0.:
                return AvoidanceDecision(self.previous.copy(), 'AVOID', encounter, baseline,
                                         baseline.time < 6., 2, True, remaining, 'FORWARD')

        side = unit(np.cross(goal_axis, np.eye(3)[np.argmin(np.abs(goal_axis))]))
        other_side = np.cross(goal_axis, side)
        # Preserve an existing forward course, including the guidance flyby offset.
        # Pointing every boost at the target centre can create a second correction
        # merely to restore the already acceptable arrival trajectory.
        course = unit(velocity) if np.linalg.norm(velocity) > .5 else goal_axis
        if np.linalg.norm(nominal) > .01 and unit(nominal) @ goal_axis > math.cos(math.radians(20.)):
            course = unit(nominal)
        if course @ goal_axis < math.cos(math.radians(20.)):
            course = goal_axis
        course_side = unit(np.cross(course, np.eye(3)[np.argmin(np.abs(course))]))
        course_other = np.cross(course, course_side)
        directions = [course] if np.linalg.norm(course - goal_axis) < 1e-9 else [course, goal_axis]
        for degrees in (5., 12., 20.):
            angle = math.radians(degrees)
            for azimuth in np.arange(8) * math.pi / 4.:
                lateral = math.cos(azimuth) * course_side + math.sin(azimuth) * course_other
                direction = math.cos(angle) * course + math.sin(angle) * lateral
                if direction @ goal_axis >= math.cos(math.radians(20.)) - 1e-12:
                    directions.append(direction)
        evaluated = []
        count = 1
        # A rear-thrust maneuver is meaningful only while the nose already faces
        # generally toward the target. Otherwise retain the emergency translation
        # options while the existing coarse alignment turns the shuttle.
        if nose_forward:
            for direction in directions:
                acceleration = min(self.model.planning_accel, self.model.avoidance_accel) * direction
                for duration in (2., 4., 8.):
                    encounter = scene.assess(acceleration, duration, initial_direction=nose)
                    count += 1
                    if encounter.clearance < 0.:
                        continue
                    # Minimize impulse and subsequent lateral cleanup. All these
                    # planned directions retain cos(20 degrees), about 94%, or more
                    # of their acceleration toward the target.
                    lateral = acceleration - float(acceleration @ course) * course
                    cost = duration * (np.linalg.norm(acceleration) + 2. * np.linalg.norm(lateral))
                    if self.active:
                        cost += .5 * min(duration, 2.) * np.linalg.norm(acceleration - self.previous)
                    evaluated.append((cost, acceleration, duration, encounter))
        if evaluated:
            _, acceleration, duration, encounter = min(evaluated, key=lambda item: item[0])
            self.active, self.forward_burn = True, True
            self.previous = acceleration.copy()
            self.burn_until = now + duration
            return AvoidanceDecision(acceleration, 'AVOID', encounter, baseline,
                                     baseline.time < 6., count, True, duration, 'FORWARD')

        coast = scene.assess(np.zeros(3))
        count += 1
        if coast.clearance >= 0.:
            self.active, self.forward_burn = True, False
            self.previous = np.zeros(3)
            return AvoidanceDecision(self.previous.copy(), 'AVOID', coast, baseline,
                                     baseline.time < 6., count, strategy='COAST')

        # Only when forward burns, shallow turns AND coasting have no safe option
        # may an emergency escape request a strong sidestep or braking impulse.
        forward = unit(velocity if np.linalg.norm(velocity) > .5 else goal_axis)
        candidates = [nominal, self.previous, -self.model.avoidance_accel * forward]
        for angle in np.arange(8) * math.pi / 4.:
            direction = math.cos(angle) * side + math.sin(angle) * other_side
            for strength in (.5, 1.):
                candidates.append(strength * self.model.avoidance_accel * direction)
                candidates.append(limited(nominal + strength * self.model.avoidance_accel * direction,
                                          self.model.avoidance_accel))
            candidates.append(self.model.avoidance_accel * unit(goal_axis + direction))
        evaluated = []
        for acceleration in candidates:
            encounter = baseline if acceleration is nominal else scene.assess(acceleration, 8.)
            # Safety first. Among safe paths, minimize correction/fuel and prefer the
            # previous maneuver rather than alternating sides with measurement noise.
            cost = (np.linalg.norm(acceleration - nominal) + .3 * np.linalg.norm(acceleration)
                    + (.4 * np.linalg.norm(acceleration - self.previous) if self.active else 0.))
            cost += 1.5 * max(0., -float(acceleration @ unit(goal)))
            if encounter.clearance >= 0.:
                rank = (1, -cost)
            else:
                rank = (0, encounter.time + .1 * encounter.clearance - cost)
            evaluated.append((rank, acceleration, encounter))
        _, acceleration, encounter = max(evaluated, key=lambda item: item[0])
        self.active, self.forward_burn = True, False
        self.previous = acceleration.copy()
        urgent = min(baseline.time, encounter.time) < 6.
        return AvoidanceDecision(acceleration, 'EMERGENCY', encounter, baseline,
                                 urgent, count + len(candidates), strategy='ESCAPE')


class PulseController:
    """Feedback impulse debt prevents the allocator from turning tiny requests into
    continuous firing. Desired and measured velocity changes use one common frame.
    """
    def __init__(self, start, stop, min_on, command_floor):
        self.start, self.stop, self.min_on, self.command_floor = start, stop, min_on, command_floor
        self.debt = np.zeros(3)
        self.previous_desired = np.zeros(3)
        self.active = False
        self.changed_at = -math.inf
        self.switches = 0

    def reset(self):
        self.debt[:] = 0.
        self.previous_desired[:] = 0.
        self.active = False

    def command(self, desired, measured_delta, dt, now, urgent=False):
        self.debt = limited(self.debt + self.previous_desired * dt - measured_delta, 4. * self.start)
        self.previous_desired = np.array(desired)
        magnitude = float(np.linalg.norm(desired))
        direction = unit(desired)
        aligned_debt = float(self.debt @ direction)
        before = self.active
        if magnitude < 1e-6:
            self.active = False
            self.debt *= math.exp(-dt / .5)
        elif urgent or (not self.active and aligned_debt >= self.start):
            self.active = True
        elif self.active and aligned_debt <= self.stop and now - self.changed_at >= self.min_on:
            self.active = False
        if before != self.active:
            self.changed_at = now
            self.switches += 1
        if not self.active:
            return np.zeros(3)
        return direction * max(magnitude, self.command_floor)


class AccelerationAttitude:
    """Slew the reference toward sustained thrust; hold the reference during coasting."""
    def __init__(self, forward):
        self.forward = unit(forward)

    def update(self, acceleration, dt, pointing_error=None, priority=1.):
        if np.linalg.norm(acceleration) < .005:
            return self.forward.copy()
        desired = unit(acceleration)
        cosine = float(np.clip(self.forward @ desired, -1., 1.))
        angle = math.acos(cosine)
        strength = alignment_strength(pointing_error) * float(np.clip(priority, 0., 1.)) if pointing_error is not None else 0.
        step = min(angle, math.radians(2. + 8. * strength) * dt)
        if angle > 1e-8:
            tangent = desired - cosine * self.forward
            if np.linalg.norm(tangent) < 1e-8:
                tangent = np.cross(self.forward, np.eye(3)[np.argmin(np.abs(self.forward))])
            self.forward = unit(math.cos(step) * self.forward + math.sin(step) * unit(tangent))
        return self.forward.copy()
