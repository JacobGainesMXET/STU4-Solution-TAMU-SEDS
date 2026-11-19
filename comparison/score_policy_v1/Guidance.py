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
    max_speed: float = 10.
    position_sigma: float = 2.
    process_accel_sigma: float = .015   # tuning allowance for unmodelled relative acceleration
    uncertainty_sigma: float = 2.5
    ship_radius: float = 2.5           # existing geometry estimate; still needs hull validation
    clearance: float = 3.
    horizon: float = 30.
    track_timeout: float = 12.
    radar_timeout: float = 3.

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
                state, covariance = measurement_update(
                    state, covariance, velocity_matrix,
                    np.array([contact.range_rate + ship_velocity @ ray]), np.array([[radial_noise]]))
            track.state, track.covariance, track.time = state, covariance, contact.t_meas
            track.radius = .8 * track.radius + .2 * (.5 * contact.diameter)
        return [item[0] for item in observations]

    def ready(self, now):
        confirmed = sum(track.hits >= 2 for track in self.tracks.values())
        return bool(self.tracks) and confirmed >= .6 * len(self.tracks) and now - self.last_fresh < self.model.radar_timeout


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


class ScoreGuidance:
    """Compare an extra burn followed by ballistic coasting using the actual score.

    This is a small receding-horizon estimate, not a claim of globally optimal guidance.
    Arrival speed is deliberately unconstrained: the scorer requires position only.
    """
    def __init__(self, model):
        self.model = model
        self.offset = None
        self.burning = False

    def decide(self, relative_position, relative_velocity, gravity, elapsed, fuel, reference):
        model = self.model
        if np.linalg.norm(relative_position) <= model.success_radius:
            return GuidanceDecision(np.zeros(3), 'ARRIVED', 0.)
        if self.offset is None:
            axis = unit(relative_position)
            side = reference - float(reference @ axis) * axis
            if np.linalg.norm(side) < 1e-6:
                side = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
            self.offset = unit(side) * min(model.flyby_offset, .6 * model.success_radius)
        aim = relative_position + self.offset
        direction = unit(aim)
        # An 18 m ball about a 30 m offset point is wholly inside the 50 m success area.
        aim_radius = max(2., model.success_radius - np.linalg.norm(self.offset) - 2.)
        distance = max(0., float(np.linalg.norm(aim)) - aim_radius)
        speed = float(relative_velocity @ direction)
        g_along = float(gravity @ direction)
        coast_eta = travel_time(distance, speed, g_along)
        coast_score = model.score(elapsed + coast_eta, fuel) if math.isfinite(coast_eta) else -math.inf
        choices = []
        for duration in np.linspace(0., 140., 71):
            if duration > 0. and speed + model.planning_accel * duration > model.max_speed:
                continue
            powered_eta = travel_time(distance, speed, g_along + model.planning_accel)
            burn = min(duration, powered_eta)
            covered = speed * burn + .5 * (g_along + model.planning_accel) * burn * burn
            remaining = travel_time(max(0., distance - covered), speed + (g_along + model.planning_accel) * burn, g_along)
            eta = burn + remaining
            if math.isfinite(eta) and eta < 1200.:
                score = model.score(elapsed + eta, fuel + model.fuel_per_dv * model.planning_accel * burn)
                choices.append((score, -burn, eta))
        if not choices:
            return GuidanceDecision(model.planning_accel * direction, 'RECOVER', 300.)
        score, negative_burn, eta = max(choices)
        burn = -negative_burn
        advantage = score - coast_score
        threshold = .05 if self.burning else .3
        self.burning = burn > .5 and advantage > threshold and speed < model.max_speed
        acceleration = model.planning_accel * direction if self.burning else np.zeros(3)
        eta = eta if self.burning else coast_eta
        eta = float(np.clip(eta, 1., 600.))
        lateral_velocity = relative_velocity - speed * direction
        lateral_gravity = gravity - g_along * direction
        lateral_miss = (lateral_velocity + .5 * lateral_gravity * eta) * eta
        if np.linalg.norm(lateral_miss) > .7 * aim_radius:
            correction_dv = -lateral_miss / eta * (1. - .7 * aim_radius / np.linalg.norm(lateral_miss))
            acceleration += correction_dv / min(6., max(1., .2 * eta))
        mode = 'BURN' if self.burning else ('CORRECT' if np.linalg.norm(acceleration) > .003 else 'COAST')
        return GuidanceDecision(limited(acceleration, model.planning_accel), mode, eta, advantage, burn)


@dataclass
class Encounter:
    clearance: float = math.inf
    time: float = math.inf
    track: int = -1
    miss_distance: float = math.inf
    uncertainty: float = 0.
    age: float = 0.
    closest_time: float = math.inf


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

    def assess(self, acceleration):
        if not len(self.position):
            return Encounter()
        burn_seconds = 8.
        time = self.times
        powered = np.minimum(time, burn_seconds)
        displacement = .5 * powered**2 + powered * np.maximum(time - burn_seconds, 0.)
        separation = self.position[:, None, :] + self.velocity[:, None, :] * time[None, :, None]
        separation -= displacement[None, :, None] * acceleration[None, None, :]
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


class AvoidancePlanner:
    def __init__(self, model):
        self.model = model
        self.active = False
        self.previous = np.zeros(3)
        self.safe_since = None

    def choose(self, scene, nominal, velocity, goal, now):
        baseline = scene.assess(nominal)
        if baseline.clearance >= 0.:
            if not self.active:
                return AvoidanceDecision(nominal, 'GUIDANCE', baseline, baseline)
            if baseline.clearance > 3.:
                self.safe_since = now if self.safe_since is None else self.safe_since
            else:
                self.safe_since = None
            if self.safe_since is not None and now - self.safe_since >= 1.5:
                self.active = False
                return AvoidanceDecision(nominal, 'GUIDANCE', baseline, baseline)
        else:
            self.safe_since = None
        forward = unit(velocity if np.linalg.norm(velocity) > .5 else goal)
        side = unit(np.cross(forward, np.eye(3)[np.argmin(np.abs(forward))]))
        other_side = np.cross(forward, side)
        candidates = [nominal, np.zeros(3), self.previous, -self.model.avoidance_accel * forward]
        for angle in np.arange(8) * math.pi / 4.:
            direction = math.cos(angle) * side + math.sin(angle) * other_side
            for strength in (.5, 1.):
                # A full lateral burn must be available independently of goal thrust.
                # Merely adding a small dodge to the forward command can remain unsafe.
                candidates.append(strength * self.model.avoidance_accel * direction)
                candidates.append(limited(nominal + strength * self.model.avoidance_accel * direction,
                                          self.model.avoidance_accel))
        evaluated = []
        for acceleration in candidates:
            encounter = scene.assess(acceleration)
            # Safety first. Among safe paths, minimize correction/fuel and prefer the
            # previous maneuver rather than alternating sides with measurement noise.
            cost = (np.linalg.norm(acceleration - nominal) + .3 * np.linalg.norm(acceleration)
                    + (.4 * np.linalg.norm(acceleration - self.previous) if self.active else 0.))
            if encounter.clearance >= 0.:
                rank = (1, -cost)
            else:
                rank = (0, encounter.time + .1 * encounter.clearance - cost)
            evaluated.append((rank, acceleration, encounter))
        _, acceleration, encounter = max(evaluated, key=lambda item: item[0])
        self.active = True
        self.previous = acceleration.copy()
        urgent = baseline.time < 6. or encounter.clearance < 0.
        return AvoidanceDecision(acceleration, 'EMERGENCY' if urgent else 'AVOID',
                                 encounter, baseline, urgent, len(candidates))


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

    def update(self, acceleration, dt):
        if np.linalg.norm(acceleration) < .005:
            return self.forward.copy()
        desired = unit(acceleration)
        cosine = float(np.clip(self.forward @ desired, -1., 1.))
        angle = math.acos(cosine)
        step = min(angle, math.radians(2.) * dt)
        if angle > 1e-8:
            tangent = desired - cosine * self.forward
            if np.linalg.norm(tangent) < 1e-8:
                tangent = np.cross(self.forward, np.eye(3)[np.argmin(np.abs(self.forward))])
            self.forward = unit(math.cos(step) * self.forward + math.sin(step) * unit(tangent))
        return self.forward.copy()
