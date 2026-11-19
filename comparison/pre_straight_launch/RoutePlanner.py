"""Compare early maneuvers by rolling the flight guidance forward from measured state."""
import copy
from dataclasses import dataclass
import math

import numpy as np

from Guidance import GuidanceDecision, unit


STEP = 3.0
HORIZON = 120.
TIMES = np.arange(0., HORIZON + STEP, STEP)


@dataclass
class Maneuver:
    name: str
    start: float
    duration: float
    side: np.ndarray
    angle: float = 0.

    @property
    def rear_burn(self):
        return self.name in ('steer', 'advance')

    def set_aim(self, guidance):
        if self.name in ('steer', 'flyby'):
            # Recover toward the same side of the arrival sphere as the early
            # deflection, avoiding a fuel-consuming correction across the route.
            guidance.offset = self.side * min(guidance.model.flyby_offset,
                                               .6 * guidance.model.success_radius)

    def apply(self, nominal, now, goal, model):
        age = now - self.start
        if not 0. <= age < self.duration or nominal.mode in ('FINAL_COAST', 'ARRIVED'):
            return nominal
        if self.name == 'delay' and nominal.mode == 'ACCELERATE':
            return GuidanceDecision(np.zeros(3), 'ROUTE_COAST', nominal.eta)
        if self.name == 'advance' and nominal.mode == 'CLEAR_COAST':
            nominal = GuidanceDecision(min(model.planning_accel, model.avoidance_accel) * goal, 'ACCELERATE', nominal.eta,
                                       burn_seconds=min(8., self.duration - age))
        if self.angle and nominal.mode == 'ACCELERATE':
            # Follow the selected offset heading during this leg. The ordinary
            # flyby law takes over after release and its cleanup is in the score.
            axis = goal
            side = self.side - (self.side @ axis) * axis
            # Share the existing forward burn. Ease into and out of the heading
            # bias at a rate consistent with the allocated shallow-turn response.
            desired = unit(axis + math.tan(math.radians(self.angle)) * side)
            base = unit(nominal.acceleration)
            difference = math.degrees(math.acos(float(np.clip(base @ desired, -1., 1.))))
            slew = min(.5 * (age + .25), .5 * (self.duration - age), 16.)
            weight = min(1., slew / max(1e-8, difference))
            axis = unit((1. - weight) * base + weight * desired)
            return GuidanceDecision(min(np.linalg.norm(nominal.acceleration), model.avoidance_accel) * axis, nominal.mode, nominal.eta,
                                    burn_seconds=nominal.burn_seconds)
        return nominal


@dataclass
class Forecast:
    positions: np.ndarray            # target-relative displacement since plan start
    eta: float
    fuel: float                      # projected total at arrival
    score: float
    burns: int
    seconds_on: float
    delta_v: float


def forecast(guidance, r, v, gravity, reference, now, fuel, action, nose=None):
    # The rollout owns its latches, trim impulse budget and learned model snapshot.
    guide = copy.deepcopy(guidance)
    model = guide.model
    if action is not None:
        action.set_aim(guide)
    positions = [np.zeros(3)]
    remaining, speed = r.copy(), v.copy()
    heading = unit(r) if nose is None else nose.copy()
    elapsed = dv = on_time = 0.
    burns, was_on = 0, False
    # Sample risk on the first 120 s, but price the flight through arrival.
    while elapsed < 360.:
        end = elapsed + STEP
        while end - elapsed > 1e-7:
            clock = now + elapsed
            decision = guide.decide(remaining, speed, gravity, clock, fuel + model.fuel_per_dv * dv, reference)
            if action is not None:
                decision = action.apply(decision, clock, unit(remaining), model)
            dt = end - elapsed
            if decision.burn_seconds > 1e-6:
                dt = min(dt, decision.burn_seconds)
            if action is not None and clock < action.start + action.duration - 1e-7:
                dt = min(dt, action.start + action.duration - clock)
            accel = decision.acceleration
            if (action is not None and action.rear_burn and decision.mode == 'ACCELERATE'
                    and clock < action.start + action.duration):
                desired = unit(accel)
                error = math.acos(float(np.clip(heading @ desired, -1., 1.)))
                turn_time = max(0., clock + dt - max(clock, action.start + model.avoidance_turn_delay))
                fraction = min(1., turn_time * model.avoidance_turn_rate / max(error, 1e-8))
                end_heading = unit(heading + fraction * (desired - heading))
                accel = min(model.planning_accel, model.avoidance_accel) * unit(heading + end_heading)
                heading = end_heading
            step = speed * dt + .5 * (accel + gravity) * dt**2
            # Segment/sphere entry interpolates arrival even on a fast flyby.
            square = float(step @ step)
            along = float(remaining @ step)
            disc = along**2 - square * (remaining @ remaining - model.success_radius**2)
            fraction = ((along - math.sqrt(disc)) / square if disc >= 0. and square > 1e-12 else math.inf)
            arrived = np.linalg.norm(remaining) <= model.success_radius or 0. <= fraction <= 1.
            if arrived:
                dt *= float(np.clip(fraction, 0., 1.)) if np.linalg.norm(remaining) > model.success_radius else 0.
                step = speed * dt + .5 * (accel + gravity) * dt**2
            power = float(np.linalg.norm(accel))
            on = power > 1e-8
            burns += int(on and not was_on)
            was_on = on
            dv += power * dt
            on_time += dt if on else 0.
            elapsed += dt
            remaining -= step
            speed += (gravity + accel) * dt
            if arrived:
                if len(positions) < len(TIMES):
                    positions.append(r - remaining)
                total = fuel + dv * model.fuel_per_dv
                return Forecast(np.array(positions), elapsed, total, model.score(now + elapsed, total),
                                burns, on_time, dv)
        if len(positions) < len(TIMES):
            positions.append(r - remaining)
    return Forecast(np.array(positions), math.inf, math.inf, -math.inf, burns, on_time, dv)


@dataclass
class Risk:
    clearance: float = math.inf
    miss: float = math.inf
    t: float = math.inf
    sigma: float = math.inf
    track: int = -1
    threats: int = 0
    threat_ids: frozenset = frozenset()
    unresolved: frozenset = frozenset()
    approaching_at_horizon: frozenset = frozenset()

    def clears(self, direct):
        # Moving a predicted encounter just beyond its credible time horizon
        # cannot by itself demonstrate an improvement.
        return (self.clearance >= 0. and not self.approaching_at_horizon.intersection(direct.threat_ids)
                and not self.unresolved.intersection(direct.threat_ids)
                and not self.unresolved.difference(direct.unresolved))


class RadarField:
    def __init__(self, tracks, now, origin, target_velocity, gravity, model):
        self.model = model
        tracks = [k for k in tracks if k.hits >= 2 and 0. <= now - k.time <= model.track_timeout
                  and k.reliable_hits >= 2]
        self.ids = np.array([k.number for k in tracks])
        if not tracks:
            return
        state = np.array([k.state for k in tracks])
        age = np.array([now - k.time for k in tracks])[:, None]
        dt = age + TIMES
        vel = state[:, 3:] - target_velocity
        self.xyz = ((state[:, None, :3] - origin) + state[:, None, 3:] * age[:, :, None]
                    + vel[:, None, :] * TIMES[:, None] + .5 * gravity * dt[:, :, None]**2)
        self.radius = np.array([k.radius * 1.15 + model.ship_radius + model.clearance for k in tracks])
        p = np.array([k.covariance for k in tracks])
        # Upper bound on the largest eigenvalue of each propagated 3x3 block.
        # Reuse the existing process acceleration with independent 4 s intervals
        # at long range (constant unknown acceleration within the first interval).
        t = dt[:, :, None, None]
        cov = p[:, None, :3, :3] + t * (p[:, None, 3:, :3] + p[:, None, :3, 3:]) + t*t*p[:, None, 3:, 3:]
        process_var = model.process_accel_sigma**2 * np.minimum(.25 * dt**4, 4./3. * dt**3)
        self.cov = cov
        self.process_var = process_var
        self.sigma = np.sqrt(np.maximum(0., np.max(np.sum(np.abs(cov), axis=-1), axis=-1) + process_var))

    def assess(self, path):
        if not len(self.ids) or len(path.positions) < 2:
            return Risk()
        n = len(path.positions)
        xyz = self.xyz[:, :n]
        # The last sample can be a fraction of a step when the shuttle arrives.
        if path.eta < TIMES[n-1]:
            xyz = xyz.copy()
            f = np.clip((path.eta - TIMES[n-2]) / STEP, 0., 1.)
            xyz[:, -1] = xyz[:, -2] + f * (xyz[:, -1] - xyz[:, -2])
        sep = xyz - path.positions
        delta = np.diff(sep, axis=1)
        approaching = frozenset()
        if path.eta > HORIZON:
            closing = np.sum(sep[:, -1] * delta[:, -1], axis=1) < 0.
            approaching = frozenset(self.ids[closing])
        frac = np.clip(-np.sum(sep[:, :-1]*delta, axis=2) /
                       np.maximum(1e-12, np.sum(delta*delta, axis=2)), 0., 1.)
        closest = sep[:, :-1] + frac[:, :, None]*delta
        miss = np.linalg.norm(closest, axis=2)
        # For a spherical obstacle the plane perpendicular to closest separation
        # bounds the whole segment. Project the track covariance onto that normal.
        # Check both time endpoints and retain the broad-axis confidence gate.
        normal = closest / np.maximum(miss[:, :, None], 1e-8)
        bound = np.maximum(self.sigma[:, 1:n], self.sigma[:, :n-1])
        def projected(lo, hi):
            return (np.einsum('nti,ntij,ntj->nt', normal, self.cov[:, lo:hi], normal)
                    + self.process_var[:, lo:hi])
        sigma = np.sqrt(np.maximum(0., np.maximum(projected(0, n-1), projected(1, n))))
        sigma = np.where(miss < 1e-8, bound, sigma)
        clearance = miss - self.radius[:, None] - self.model.uncertainty_sigma * sigma
        clearance -= .125 * max(self.model.planning_accel, self.model.avoidance_accel) * STEP**2
        # A very diffuse forecast cannot justify fuel expenditure on a specific
        # side of a rock. The tactical layer still sees these observations.
        diffuse = bound > np.maximum(25., self.radius[:, None])
        unresolved = frozenset(self.ids[np.any(diffuse & (miss <= self.radius[:, None] + 25.), axis=1)])
        clearance[diffuse] = math.inf
        threats = frozenset(self.ids[np.any(clearance < 0., axis=1)])
        i, j = np.unravel_index(np.argmin(clearance), clearance.shape)
        if not math.isfinite(clearance[i, j]):
            return Risk(unresolved=unresolved, approaching_at_horizon=approaching)
        time = TIMES[j] + frac[i, j] * min(STEP, path.eta - TIMES[j])
        return Risk(float(clearance[i,j]), float(miss[i,j]), float(time), float(sigma[i,j]),
                    int(self.ids[i]), len(threats), threats, unresolved, approaching)


class Search:
    """Evaluate a few alternatives per control update on one frozen radar field."""
    def __init__(self, choices, field, direct_risk, guidance, r, v, g, reference, now, fuel, nose):
        self.choices = list(choices)
        self.field = field
        self.direct_risk = direct_risk
        self.args = (copy.deepcopy(guidance), r.copy(), v.copy(), g.copy(), reference.copy(), now, fuel)
        self.nose = nose.copy()
        self.start = now
        self.best = None
        self.score = -math.inf

    def advance(self):
        count = min(2, len(self.choices))
        for _ in range(count):
            action = self.choices.pop(0)
            path = forecast(*self.args, action, self.nose)
            if path.score > self.score and self.field.assess(path).clears(self.direct_risk):
                self.best, self.score = action, path.score
        return count


class RoutePlanner:
    def __init__(self):
        self.action = None
        self.next_review = 0.
        self.reason = 'acquiring'
        self.risk = self.direct_risk = Risk()
        self.path = None
        self.extra_fuel = 0.
        self.track_count = self.candidates = 0
        self.search = None
        self.cruise_reviewed = False
        self.cruise_reason = 'launch default'
        self.review_time = None

    def interrupt(self, now, reason):
        self.action = self.path = None
        self.search = None
        self.reason = reason
        self.risk = self.direct_risk = Risk()
        self.extra_fuel = 0.
        self.track_count = self.candidates = 0
        self.next_review = now + 1.
        self.review_time = None

    def command(self, nominal, guidance, tracks, now, origin, r, v, target_velocity, gravity,
                reference, nose, fuel, known_empty=False):
        goal = unit(r)
        if self.action is not None and now >= self.action.start + self.action.duration:
            self.action = None
            self.next_review = now
        winner = None
        if self.search is not None:
            if now - self.search.start <= 4.:
                self.candidates += self.search.advance()
                if self.search.choices:
                    return nominal
                winner = self.search.best
            self.search = None
            self.next_review = now if winner is not None else now + 1.
            self.reason = 'no safe detour' if winner is None else 'rechecking route'
        if now >= self.next_review:
            self.next_review = now + 2.
            self.review_time = now
            field = RadarField(tracks, now, origin, target_velocity, gravity, guidance.model)
            self.track_count = len(field.ids)
            if not self.track_count and not known_empty:
                self.interrupt(now, 'acquiring tracks')
                return nominal
            def evaluate(action):
                path = forecast(guidance, r, v, gravity, reference, now, fuel, action, nose)
                return path, field.assess(path)
            direct, risk = evaluate(None)
            # Price the cruise speed once during early ascent. Use the slower
            # option only if it has a clear observed corridor and a score margin.
            if (not self.cruise_reviewed and now >= 4. and nominal.mode == 'ACCELERATE'
                    and guidance.model.coast_speed == 10. and v @ goal < 4.):
                self.cruise_reviewed = True
                economy = copy.deepcopy(guidance)
                economy.model.coast_speed = 9.
                trial = forecast(economy, r, v, gravity, reference, now, fuel, None)
                trial_risk = field.assess(trial)
                if (math.isfinite(trial.eta) and trial_risk.clears(risk)
                        and (risk.clearance < 0. or trial.score > direct.score + 1.)):
                    guidance.model.coast_speed = 9.
                    direct, risk = trial, trial_risk
                    self.cruise_reason = '9 m/s clear and preferred'
                else:
                    self.cruise_reason = 'retain 10 m/s'
            # The two launch burns are physically identical before the lower
            # cutoff. A newly resolved crossing can change which coast is safe.
            # Recover the faster schedule only when its whole observed path clears.
            if (self.cruise_reviewed and guidance.model.coast_speed == 9. and not guidance.coast_latched
                    and self.action is None and nominal.mode == 'ACCELERATE' and 8. <= v @ goal < 8.95
                    and risk.clearance < 0.):
                faster = copy.deepcopy(guidance)
                faster.model.coast_speed = 10.
                trial = forecast(faster, r, v, gravity, reference, now, fuel, None, nose)
                trial_risk = field.assess(trial)
                if math.isfinite(trial.eta) and trial_risk.clears(risk):
                    guidance.model.coast_speed = 10.
                    direct, risk = trial, trial_risk
                    self.cruise_reason = '10 m/s clears updated crossing'
            self.direct_risk = risk
            self.candidates = 1
            old = self.action
            self.action, self.path, self.risk, self.extra_fuel = None, direct, risk, 0.
            self.reason = 'direct clear'
            if risk.clearance < 0.:
                self.reason = 'no safe detour'
            elif risk.unresolved:
                self.reason = 'monitoring uncertain tracks'
            if winner is not None and risk.clearance < 0.:
                # Several updates elapsed during the search. Re-anchor the winning
                # leg and validate it on the current state and radar before use.
                winner.start = now
                path, current = evaluate(winner)
                if current.clears(risk) and math.isfinite(path.eta):
                    self.action, self.path, self.risk = winner, path, current
                    self.reason = 'predicted intersection'
                    winner.set_aim(guidance)
            if old is not None and now < old.start + old.duration:
                path, current = evaluate(old)
                self.candidates += 1
                if current.clears(risk) and math.isfinite(path.eta) and (
                        risk.clearance < 3. or direct.score < path.score + .5):
                    self.action, self.path, self.risk = old, path, current
                    self.reason = 'retain safe route'
            if (self.action is None and winner is None and risk.clearance < 0. and risk.t > 8.
                    and nominal.mode in ('ACCELERATE', 'CLEAR_COAST') and math.isfinite(direct.eta)):
                side = unit(np.cross(goal, np.eye(3)[np.argmin(np.abs(goal))]))
                other = np.cross(goal, side)
                coast = nominal.mode == 'CLEAR_COAST'
                choices = []
                for duration in (3., 6.):
                    if coast and nose @ goal >= math.cos(math.radians(20.)):
                        choices.append(Maneuver('advance', now, duration, side))
                    elif not coast and min(v @ goal, (v + gravity*duration) @ goal) > .5:
                        choices.append(Maneuver('delay', now, duration, side))
                if not coast:
                    choices.extend(Maneuver('flyby', now, direct.eta, lateral)
                                   for lateral in (side, -side, other, -other))
                if not coast and nose @ goal >= math.cos(math.radians(35.)):
                    choices.extend(Maneuver('steer', now, duration, lateral, angle)
                                   for lateral in (side, -side, other, -other)
                                   for duration, angle in ((30., 6.), (80., 14.)))
                if coast and nose @ goal >= math.cos(math.radians(20.)):
                    choices.extend(Maneuver('advance', now, 16., lateral, 6.)
                                   for lateral in (side, -side, other, -other))
                self.search = Search(choices, field, risk, guidance, r, v, gravity, reference, now, fuel, nose)
                self.reason = 'checking alternatives'
            self.extra_fuel = self.path.fuel - direct.fuel if math.isfinite(direct.fuel) else 0.
        if self.action is None or now >= self.action.start + self.action.duration:
            return nominal
        return self.action.apply(nominal, now, goal, guidance.model)
