"""Offline checks: extract pure helpers without importing the live simulator controller.

Run with the simulator's Python: python -m unittest discover -s tests -v
These verify control mathematics, not the simulator's thruster allocation or mission score.
"""

import ast
from dataclasses import dataclass
import math
from pathlib import Path
import unittest

import numpy as np
import numpy.typing as npt
from Guidance import PulseController, low_tilt_target


def load_helpers():
    source = Path(__file__).resolve().parents[1] / "Controller.py"
    tree = ast.parse(source.read_text(encoding="utf-8-sig"))
    names = {"RadarContact", "LimitVector", "MakeTargetAttitude", "RotationVector",
             "ComputeAttitudeAcceleration"}
    nodes = [node for node in tree.body
             if (isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names)
             or (isinstance(node, ast.Assign)
                 and all(isinstance(target, ast.Name) and target.id.isupper()
                         for target in node.targets))]
    namespace = {"np": np, "npt": npt, "math": math, "dataclass": dataclass,
                 "__name__": __name__}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    return namespace


CONTROL = load_helpers()


def active_rotation(vector):
    """Independent Rodrigues construction for synthetic vehicle attitudes."""
    vector = np.asarray(vector, dtype=float)
    angle = np.linalg.norm(vector)
    if angle < 1e-14:
        return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * (skew @ skew)


def contact(position, range_rate=-8., diameter=2.):
    position = np.asarray(position, dtype=float)
    return CONTROL["RadarContact"](position, np.linalg.norm(position) - diameter / 2.,
                                   range_rate, diameter, 0.)


class AttitudeChecks(unittest.TestCase):
    def test_low_tilt_reference_limits_full_rotation_in_launch_coordinates(self):
        pad = active_rotation([.6, .2, -.4]).T
        for local_goal in ([1., 0., 0.], [0., 0., -1.], [0., 0., 1.], [-.062, -.041, .997]):
            goal = np.array(local_goal) / np.linalg.norm(local_goal)
            ref = low_tilt_target(pad, pad.T @ goal)
            angle = np.linalg.norm(CONTROL["RotationVector"](pad @ ref.T))
            self.assertLessEqual(math.degrees(angle), 7.5 + 1e-9)
            np.testing.assert_allclose(ref @ ref.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(ref), 1.)
        # At the final-scene launch pose the nose rises slightly and the
        # thrust axis stays nearly parallel to the departure corridor.
        self.assertGreater(ref[0] @ pad[2], 0.)
        self.assertGreater(ref[2] @ (pad.T @ goal), math.cos(math.radians(2.)))

    def test_perpendicular_axes_even_when_roll_reference_is_parallel(self):
        rng = np.random.default_rng(42)
        cases = [(np.array([0., 1., 0.]), np.array([0., -1., 0.]))]
        cases += [(rng.normal(size=3), rng.normal(size=3)) for _ in range(100)]
        for forward, reference in cases:
            axes = CONTROL["MakeTargetAttitude"](forward, reference)
            np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(axes), 1.)
            np.testing.assert_allclose(axes[0], forward / np.linalg.norm(forward))

    def test_preserves_roll_when_forward_direction_is_unchanged(self):
        axes = active_rotation([.2, .4, -.5]).T
        np.testing.assert_allclose(CONTROL["MakeTargetAttitude"](axes[0], axes[1]), axes, atol=1e-12)

    def test_shortest_rotation_including_zero_and_half_turns(self):
        rng = np.random.default_rng(17)
        for degrees in [0., 1e-9, 25., 179.9999, 180., 180.0001, 350.]:
            for _ in range(10):
                axis = rng.normal(size=3)
                axis /= np.linalg.norm(axis)
                rotation = active_rotation(axis * math.radians(degrees))
                vector = CONTROL["RotationVector"](rotation)
                self.assertLessEqual(np.linalg.norm(vector), math.pi + 1e-12)
                np.testing.assert_allclose(active_rotation(vector), rotation, atol=1e-9)

    def test_body_corrections_have_the_right_sign_at_arbitrary_attitudes(self):
        rng = np.random.default_rng(11)
        for _ in range(50):
            current = active_rotation(rng.normal(size=3))
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            target = current @ active_rotation(.2 * axis)
            acceleration, angle = CONTROL["ComputeAttitudeAcceleration"](
                current.T, target.T, np.zeros(3), .01)
            self.assertAlmostEqual(angle, .2)
            self.assertGreater(acceleration @ axis, 0.)
            np.testing.assert_allclose(np.cross(acceleration, axis), np.zeros(3), atol=1e-12)

    def test_body_correction_is_invariant_to_world_reference_rotation(self):
        current = active_rotation([.1, -.2, .8]).T
        target = active_rotation([.3, .6, -.2]).T
        world_rotation = active_rotation([.4, -.7, .2])
        first, _ = CONTROL["ComputeAttitudeAcceleration"](current, target, np.zeros(3), .01)
        second, _ = CONTROL["ComputeAttitudeAcceleration"](
            current @ world_rotation.T, target @ world_rotation.T, np.zeros(3), .01)
        np.testing.assert_allclose(first, second, atol=1e-12)

    def test_damps_spin_and_respects_actual_acceleration_limit(self):
        omega = np.array([.1, -.2, .3])
        acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](np.eye(3), np.eye(3), omega, .003)
        self.assertLess(acceleration @ omega, 0.)
        self.assertAlmostEqual(np.linalg.norm(acceleration), .003)

    def test_coasts_inside_both_attitude_and_spin_tolerances(self):
        target = active_rotation([0., 0., math.radians(1.)]).T
        acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](
            np.eye(3), target, np.array([0., 0., math.radians(.1)]), .01)
        np.testing.assert_array_equal(acceleration, np.zeros(3))

    def test_wide_pointing_tolerance_does_not_chase_small_attitude_error(self):
        target = active_rotation([0., 0., math.radians(6.)]).T
        acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](
            np.eye(3), target, np.zeros(3), .03, tolerance=math.radians(8.))
        np.testing.assert_array_equal(acceleration, np.zeros(3))

    def test_desired_turn_rate_is_bounded(self):
        target = active_rotation([0., 0., math.pi]).T
        acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](np.eye(3), target, np.zeros(3), 10.)
        self.assertLessEqual(np.linalg.norm(acceleration) * CONTROL["ANG_RATE_TIME_S"],
                             CONTROL["MAX_TURN_RAD_S"] + 1e-12)

    def test_adaptive_alignment_can_request_stronger_large_error_correction(self):
        target = active_rotation([0., 0., math.radians(60.)]).T
        slow, _ = CONTROL["ComputeAttitudeAcceleration"](np.eye(3), target, np.zeros(3), .005)
        fast, _ = CONTROL["ComputeAttitudeAcceleration"](
            np.eye(3), target, np.zeros(3), .02, gain_scale=2., rate_limit=math.radians(4.))
        self.assertGreater(np.linalg.norm(fast), np.linalg.norm(slow))
        self.assertLessEqual(np.linalg.norm(fast), .02 + 1e-12)

    def test_small_position_error_does_not_weaken_braking_for_existing_spin(self):
        target = active_rotation([0., 0., math.radians(2.)]).T
        omega = np.array([0., 0., .1])
        acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](
            np.eye(3), target, omega, .005, brake_limit=.02)
        self.assertLess(acceleration @ omega, 0.)
        self.assertAlmostEqual(np.linalg.norm(acceleration), .02)

    def test_ideal_closed_loop_settles_large_angles_and_initial_spin(self):
        # Integrate ideal requested angular accelerations. This is not a Space Teams run.
        axis = np.array([1., 2., -3.]) / math.sqrt(14.)
        for degrees, initial_spin in [(90., 0.), (180., 0.), (179.999, .12)]:
            with self.subTest(degrees=degrees, initial_spin=initial_spin):
                attitude = active_rotation(axis * math.radians(degrees))
                omega = initial_spin * axis
                peak_spin = np.linalg.norm(omega)
                for _ in range(3600):
                    acceleration, _ = CONTROL["ComputeAttitudeAcceleration"](
                        attitude.T, np.eye(3), omega, .01)
                    self.assertLessEqual(np.linalg.norm(acceleration), .01 + 1e-12)
                    omega += acceleration * .05
                    attitude = attitude @ active_rotation(omega * .05)
                    peak_spin = max(peak_spin, np.linalg.norm(omega))
                final_error = np.linalg.norm(CONTROL["RotationVector"](attitude.T))
                self.assertLess(final_error, math.radians(1.6))
                self.assertLessEqual(np.linalg.norm(omega), CONTROL["ANG_RATE_TOL_RAD_S"])
                self.assertLessEqual(peak_spin, max(initial_spin, CONTROL["MAX_TURN_RAD_S"]) + 1e-8)

    def test_pulsed_attitude_loop_settles_with_amplified_actuator_response(self):
        axis = np.array([1., 2., -3.]) / math.sqrt(14.)
        attitude = active_rotation(axis * math.radians(90.))
        omega_world = np.zeros(3)
        observed_delta = np.zeros(3)
        pulses = PulseController(math.radians(.12), math.radians(.025), .08, .006)
        peak_spin = 0.
        for index in range(7000):
            omega_body = attitude.T @ omega_world
            desired, _ = CONTROL["ComputeAttitudeAcceleration"](
                attitude.T, np.eye(3), omega_body, .01)
            command = pulses.command(attitude @ desired, observed_delta, .02, index * .02)
            actual = command / np.linalg.norm(command) * .03 if np.linalg.norm(command) else np.zeros(3)
            observed_delta = actual * .02
            omega_world += observed_delta
            attitude = active_rotation(omega_world * .02) @ attitude
            peak_spin = max(peak_spin, np.linalg.norm(omega_world))
        self.assertLess(np.linalg.norm(CONTROL["RotationVector"](attitude.T)), math.radians(2.))
        self.assertLess(np.linalg.norm(omega_world), math.radians(.2))
        self.assertLess(peak_spin, math.radians(3.5))


if __name__ == "__main__":
    unittest.main()
