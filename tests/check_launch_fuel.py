"""Fly the controller with the installed jet allocation math and an ideal rigid body.

Run separately for each controller so Python loads the matching guidance and recorder.
Optional synthetic radar scenes include spherical clearance measurements. The
plant has no contact response, damage model or launchpad constraint.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import sys
from unittest.mock import patch
import warnings

import numpy as np
from scipy.optimize import linprog

ROOT = Path(__file__).resolve().parents[1]


def compare(previous, through_arrival, before_transit=False, *, before_score=False,
            scene=None, seed=7, attitude='flight'):
    directory = ROOT
    if previous:
        directory = ROOT / 'comparison/pre_forward_only'
    elif before_transit:
        directory = ROOT / 'comparison/pre_transit_target'
    elif before_score:
        directory = ROOT / 'comparison/pre_score_optimization'
    sys.path.insert(0, str(directory))
    from test_controller_loop import FakeSimulation, StopLoop
    from test_allocator_response import AllocatorResponseTests
    from test_controller_math import active_rotation
    from test_guidance import Sample
    import Guidance
    import RoutePlanner

    AllocatorResponseTests.setUpClass()
    jets = AllocatorResponseTests()
    gravity = np.array([-.006, 0., 0.])
    origin = np.array([0., 0., -10000.])
    # Synthetic objects and sensor observations belong only to this test plant.
    cases = {
        'head-on': [([500., 0., 0.], [-1., 0., 0.], 2.)],
        'crossings': [([420., 100., 0.], [-.2, -1.13, 0.], 2.),
                      ([655., 80., -15.], [0., -.7, .1], 4.),
                      ([755., -85., 4.], [0., .8, 0.], 3.)],
    }
    rocks = cases[scene] if scene else []
    rng = np.random.default_rng(seed)
    samples = [None] * len(rocks)
    stamps = [-1.] * len(rocks)

    class Launch(FakeSimulation):
        fuel = 0.
        at_20 = None
        at_speed = None
        min_gap = float('inf')

        def radar(self, count):
            for i, (offset, drift, radius) in enumerate(rocks):
                # Each object updates once per 500/120 s sweep, at its own phase.
                period = 500. / 120.
                phase = period * i / len(rocks)
                stamp = max(0., phase + period * np.floor((self.now - phase) / period))
                if stamp <= stamps[i]:
                    continue
                age = self.now - stamp
                velocity = np.asarray(drift) + gravity * stamp
                rel = origin + offset + np.asarray(drift)*stamp + .5*gravity*stamp**2
                rel -= self.position - age * self.velocity
                distance = np.linalg.norm(rel)
                rate = (velocity - self.velocity) @ rel / distance
                samples[i] = Sample(rel + rng.normal(0., 2., 3), distance - radius + rng.normal(0., 2.),
                                    rate + rng.normal(0., .02), 2.*radius*(1. + rng.normal(0., .05)), stamp)
                stamps[i] = stamp
            return sorted(samples, key=lambda sample: sample.center_range)[:count]

        def get_param(self, kind, key):
            return self.fuel if key == ['Stats', 'FuelUsed'] else super().get_param(kind, key)

        def sleep(self, dt):
            gap = np.linalg.norm(self.target_state().loc_WRT_ExprIn(None) - self.position)
            if self.now >= (350. if through_arrival else 60.) or gap <= 50.:
                raise StopLoop()
            rhs = np.r_[self.command, self.angular]
            times = np.zeros(len(jets.activity))
            if np.linalg.norm(rhs) > 1e-9:
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', DeprecationWarning)
                    solution = linprog(np.ones(len(times)), A_eq=jets.activity.T, b_eq=rhs,
                                       method='revised simplex')
                if not solution.success:
                    raise ValueError(solution.message)
                times = solution.x
            impulse, kg = jets.helpers['GetEffectiveImpulseAndFuelUse'](
                jets.activity, jets.mass, jets.inertia, times, jets.period)
            accel = self.rotation @ impulse[:3] / (jets.mass * jets.period) + gravity
            torque = impulse[3:] / jets.period - np.cross(self.spin, jets.inertia @ self.spin)
            self.spin += np.linalg.solve(jets.inertia, torque) * dt
            self.rotation = self.rotation @ active_rotation(self.spin * dt)
            self.position += self.velocity*dt + .5*accel*dt**2
            self.velocity += accel*dt
            self.fuel += kg * dt / jets.period
            self.now += dt
            for offset, drift, radius in rocks:
                rock = origin + offset + np.asarray(drift)*self.now + .5*gravity*self.now**2
                self.min_gap = min(self.min_gap, np.linalg.norm(rock - self.position) - radius - 2.5)
            sample = (round(self.now, 2), round(self.fuel, 2), round(self.velocity[0], 2),
                      round(np.degrees(np.arccos(np.clip(self.rotation[0, 0], -1., 1.))), 1))
            if self.at_20 is None and self.now >= 20.:
                self.at_20 = sample
            if self.at_speed is None and self.velocity[0] >= 3.:
                self.at_speed = sample

    initial = active_rotation([0., np.radians(93.5), 0.])
    if attitude == 'aligned':
        initial = np.eye(3)
    if attitude == 'top':
        basis = active_rotation([0., -np.pi/2., 0.])
        initial = initial @ basis
        jets.activity = np.column_stack((jets.activity[:, :3] @ basis, jets.activity[:, 3:] @ basis))
        jets.inertia = basis.T @ jets.inertia @ basis
    launch = Launch(no_debris_test=not rocks, rotation=initial)
    with ExitStack() as context:
        if attitude == 'hold':
            def direct(request, nose, goal, velocity):
                return request
            context.enter_context(patch.object(Guidance, 'angular_command', return_value=np.zeros(3)))
            context.enter_context(patch.object(Guidance, 'rear_thrust', direct))
            context.enter_context(patch.object(RoutePlanner, 'rear_thrust', direct))
        launch.run(source_path=directory / 'Controller.py')
    faults = [msg for msg in launch.messages if 'Guidance degraded:' in msg or 'forecast unavailable:' in msg]
    if faults:
        raise RuntimeError(faults[0])
    print('time_s, fuel_kg, closing_m_s, nose_deg')
    print('at 20 s', launch.at_20, 'at 3 m/s', launch.at_speed)
    if through_arrival:
        gap = np.linalg.norm(launch.target_state().loc_WRT_ExprIn(None) - launch.position)
        print('arrival', gap <= 50., 'time', round(launch.now, 2), 'fuel', round(launch.fuel, 2))
        if rocks:
            print('minimum modeled hull gap', round(launch.min_gap, 2))
        if gap <= 50. and launch.min_gap > 0.:
            print('zero-damage score', round(50. + 12000./launch.now - 100.*launch.fuel/75., 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    baseline = parser.add_mutually_exclusive_group()
    baseline.add_argument('--previous', action='store_true')
    baseline.add_argument('--before-transit', action='store_true', help='Use the saved under-50 kg flight controller.')
    baseline.add_argument('--before-score', action='store_true', help='Use the saved 150-second controller.')
    parser.add_argument('--scene', choices=('head-on', 'crossings'))
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--attitude', choices=('flight', 'hold', 'top', 'aligned'), default='flight')
    parser.add_argument('--through-arrival', action='store_true')
    args = parser.parse_args()
    compare(args.previous, args.through_arrival, args.before_transit, before_score=args.before_score,
            scene=args.scene, seed=args.seed, attitude=args.attitude)
