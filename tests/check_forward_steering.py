"""Damped shallow-turn check using extracted controller code and actual allocator math.

Ideal rigid-body attitude with prescribed rear thrust, fixed target and requested
course. No translational/debris integration; not a Space Teams run or mission replay.
"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from check_interrupted_alignment import attitude_code
from test_allocator_response import AllocatorResponseTests
from test_controller_math import active_rotation
from Guidance import MissionModel


def run():
    AllocatorResponseTests.setUpClass()
    allocator = AllocatorResponseTests()
    model = MissionModel()
    results = []
    for lateral_axis in (1, 2):
        for degrees in (5., 12., 20.):
            scope, code = attitude_code(ROOT)
            rotation, omega = np.eye(3), np.zeros(3)
            direction = np.zeros(3)
            direction[0], direction[lateral_axis] = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
            scope.update(attitude_reference=scope['AccelerationAttitude'](rotation[:, 0]),
                         roll_reference=rotation[:, 1].copy(), mission_complete=False,
                         model=SimpleNamespace(planning_accel=.12), quiet_coast=False,
                         alignment_engaged=True, angular_limit=.05, distance=1300., dt=.1,
                         goal=np.array([1., 0., 0.]), forward_avoidance=True, mode='AVOID',
                         command=.12 * direction,
                         plan=SimpleNamespace(urgent=False, baseline=SimpleNamespace(time=20.),
                                              encounter=SimpleNamespace(time=math.inf)))
            first_within, peak = None, 0.
            forecast_lead = 0.
            timeline = []
            for step in range(301):
                now = step * .1
                scope.update(current_axes=rotation.T, omega_body=omega,
                             linear_world=.09 * rotation[:, 0])
                exec(code, scope)
                delivered, _ = allocator.allocate(np.array([.09, 0., 0.]), scope['alpha_command'])
                gyro = np.linalg.solve(allocator.inertia, np.cross(omega, allocator.inertia @ omega))
                omega += (delivered - gyro) * .1
                rotation = rotation @ active_rotation(omega * .1)
                error = math.degrees(math.acos(float(np.clip(rotation[:, 0] @ direction, -1., 1.))))
                peak = max(peak, math.degrees(float(np.linalg.norm(omega))))
                if error <= 2. and first_within is None:
                    first_within = now
                if now <= 8.:
                    fraction = np.clip((now - model.avoidance_turn_delay)
                                       * model.avoidance_turn_rate / math.radians(degrees), 0., 1.)
                    forecast_angle = math.degrees(math.atan2(fraction * math.sin(math.radians(degrees)),
                                                             1. - fraction + fraction * math.cos(math.radians(degrees))))
                    actual_angle = math.degrees(math.acos(float(np.clip(rotation[0, 0], -1., 1.))))
                    forecast_lead = max(forecast_lead, forecast_angle - actual_angle)
                if step % 20 == 0:
                    timeline.append(dict(t=round(now, 1), error_deg=round(error, 3)))
            results.append(dict(lateral_axis=lateral_axis, requested_deg=degrees,
                                within_two_deg_s=first_within, peak_spin_deg_s=round(peak, 3),
                                max_forecast_lead_deg=round(forecast_lead, 6),
                                final_error_deg=round(error, 3), timeline=timeline))
            assert first_within is not None and peak < 3., results[-1]
            assert forecast_lead < .1, results[-1]
    return dict(evidence=__doc__, results=results)


if __name__ == '__main__':
    report = run()
    destination = ROOT / 'comparison/forward_steering_checks.json'
    destination.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    for result in report['results']:
        print({key: value for key, value in result.items() if key != 'timeline'})
