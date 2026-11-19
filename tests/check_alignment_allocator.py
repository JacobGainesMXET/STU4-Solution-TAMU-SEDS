"""Offline 90-degree turn comparison: installed allocator plus ideal rigid-body rotation.

Run from the solution directory: python -B tests/check_alignment_allocator.py
Fixed world thrust; no debris or translation integration. Not a Space Teams run.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from Guidance import AccelerationAttitude, alignment_strength, angular_command
from test_allocator_response import AllocatorResponseTests
from test_controller_math import CONTROL, active_rotation


def compare():
    AllocatorResponseTests.setUpClass()
    allocator = AllocatorResponseTests()
    results = {}
    for adaptive in (False, True):
        rotation = active_rotation([0., math.pi / 2., 0.])
        omega = np.zeros(3)
        reference = AccelerationAttitude(rotation[:, 0])
        roll = rotation[:, 1].copy()
        aligned_at, peak, fuel = None, 0., 0.
        dt = .2
        for index in range(600):
            nose = math.acos(float(np.clip(rotation[0, 0], -1., 1.)))
            strength = alignment_strength(nose) if adaptive else 0.
            forward = reference.update(np.array([.12, 0., 0.]), dt,
                                       pointing_error=nose if adaptive else None)
            target_axes = CONTROL['MakeTargetAttitude'](forward, roll)
            roll = target_axes[1].copy()
            alpha, _ = CONTROL['ComputeAttitudeAcceleration'](
                rotation.T, target_axes, omega, .005 + .015 * strength,
                gain_scale=1. + strength, rate_limit=math.radians(3. + strength),
                brake_limit=.02 if adaptive else None)
            request = angular_command(alpha, omega, True, 1. / 60., .02 if adaptive else .005)
            actual, spent = allocator.allocate(rotation.T @ np.array([.5, 0., 0.]), request)
            gyro = np.linalg.solve(allocator.inertia, np.cross(omega, allocator.inertia @ omega))
            omega += (actual - gyro) * dt
            rotation = rotation @ active_rotation(omega * dt)
            peak = max(peak, float(np.linalg.norm(omega)))
            fuel += spent * dt / allocator.period
            if aligned_at is None and nose < math.radians(5.):
                aligned_at = index * dt
        results['adaptive' if adaptive else 'previous'] = dict(
            first_within_5_deg_s=aligned_at, peak_spin_deg_s=math.degrees(peak),
            final_nose_deg=math.degrees(math.acos(float(np.clip(rotation[0, 0], -1., 1.)))),
            fuel_over_120_s=fuel)
    return results


if __name__ == '__main__':
    print(json.dumps(dict(evidence=__doc__, results=compare()), indent=2))
