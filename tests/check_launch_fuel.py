"""Fly the first 60 seconds with the installed jet allocation math and an ideal rigid body.

Run separately for each controller so Python loads the matching guidance and recorder.
There is no launchpad contact or debris in this comparison.
"""
import argparse
from pathlib import Path
import sys
import warnings

import numpy as np
from scipy.optimize import linprog

ROOT = Path(__file__).resolve().parents[1]


def compare(previous, through_arrival):
    directory = ROOT / 'comparison/pre_forward_only' if previous else ROOT
    sys.path.insert(0, str(directory))
    from test_controller_loop import FakeSimulation, StopLoop
    from test_allocator_response import AllocatorResponseTests
    from test_controller_math import active_rotation

    AllocatorResponseTests.setUpClass()
    jets = AllocatorResponseTests()

    class Launch(FakeSimulation):
        fuel = 0.
        at_20 = None
        at_speed = None

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
            accel = self.rotation @ impulse[:3] / (jets.mass * jets.period) + [-.006, 0., 0.]
            torque = impulse[3:] / jets.period - np.cross(self.spin, jets.inertia @ self.spin)
            self.spin += np.linalg.solve(jets.inertia, torque) * dt
            self.rotation = self.rotation @ active_rotation(self.spin * dt)
            self.position += self.velocity*dt + .5*accel*dt**2
            self.velocity += accel*dt
            self.fuel += kg * dt / jets.period
            self.now += dt
            sample = (round(self.now, 2), round(self.fuel, 2), round(self.velocity[0], 2),
                      round(np.degrees(np.arccos(np.clip(self.rotation[0, 0], -1., 1.))), 1))
            if self.at_20 is None and self.now >= 20.:
                self.at_20 = sample
            if self.at_speed is None and self.velocity[0] >= 3.:
                self.at_speed = sample

    launch = Launch(no_debris_test=True, rotation=active_rotation([0., np.radians(93.5), 0.]))
    launch.run(source_path=directory / 'Controller.py')
    print('time_s, fuel_kg, closing_m_s, nose_deg')
    print('at 20 s', launch.at_20, 'at 3 m/s', launch.at_speed)
    if through_arrival:
        gap = np.linalg.norm(launch.target_state().loc_WRT_ExprIn(None) - launch.position)
        print('arrival', gap <= 50., 'time', round(launch.now, 2), 'fuel', round(launch.fuel, 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', action='store_true')
    parser.add_argument('--through-arrival', action='store_true')
    args = parser.parse_args()
    compare(args.previous, args.through_arrival)
