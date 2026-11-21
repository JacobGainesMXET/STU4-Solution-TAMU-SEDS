"""Check idle torque gain against the installed allocator's actual math and jet CSV.

Extract only pure functions: importing the actuator script would connect to the sim.
These are allocation calculations, not Space Teams flight tests.
"""
import ast
from pathlib import Path
import unittest
import warnings

import numpy as np
from scipy.optimize import linprog

from Guidance import angular_command


class AllocatorResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[2] / 'STU4-Sim'
        source = root / 'Systems/JetSelectionActuator.py'
        names = {'ComputeActivityVectors', 'GetEffectiveImpulseAndFuelUse', 'GetThrustersToFire', 'remap'}
        nodes = [node for node in ast.parse(source.read_text()).body
                 if isinstance(node, ast.FunctionDef) and node.name in names]
        cls.helpers = {'np': np}
        exec(compile(ast.Module(nodes, []), str(source), 'exec'), cls.helpers)
        jets = np.loadtxt(root / 'Data/SpaceBuggyJetConfigNoMains.csv', delimiter=',', skiprows=1)
        cls.mass = 3388.7
        cls.inertia = np.array([[3025.8, -171.7, 461.7], [-171.7, 5927.5, 313.6], [461.7, 313.6, 6756.]])
        cls.activity = np.array(cls.helpers['ComputeActivityVectors'](
            *jets.T, len(jets), cls.mass, np.linalg.inv(cls.inertia)))
        cls.period = 1. / 60.

    def allocate(self, acceleration, angular):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', DeprecationWarning)
            result = linprog(np.ones(len(self.activity)), A_eq=self.activity.T,
                             b_eq=np.r_[acceleration, angular], method='revised simplex')
        self.assertTrue(result.success, result.message)
        impulse, fuel = self.helpers['GetEffectiveImpulseAndFuelUse'](
            self.activity, self.mass, self.inertia, result.x, self.period)
        delivered_alpha = np.linalg.solve(self.inertia, impulse[3:]) / self.period
        return delivered_alpha, fuel

    def test_idle_torque_delivers_damping_instead_of_saturated_firing(self):
        omega = np.array([0., 0., .01])
        command = angular_command(np.zeros(3), omega, False, self.period, .005)
        delivered, fuel = self.allocate(np.zeros(3), command)
        np.testing.assert_allclose(delivered, -omega / 2., atol=1e-8)
        _, unscaled_fuel = self.allocate(np.zeros(3), -omega / 2.)
        self.assertLess(fuel, .25 * unscaled_fuel)

    def test_stationary_coast_does_not_fire_to_chase_orientation(self):
        command = angular_command(np.array([.005, .005, .005]), np.zeros(3), False, self.period, .005)
        np.testing.assert_array_equal(command, np.zeros(3))

    def test_linear_burn_keeps_bounded_attitude_feedback(self):
        command = angular_command(np.array([0., 0., -.005]), np.zeros(3), True, self.period, .005)
        delivered, fuel = self.allocate(np.array([.5, 0., 0.]), command)
        self.assertTrue(np.all(np.isfinite(delivered)))
        self.assertLess(delivered[2], 0.)
        self.assertLessEqual(np.linalg.norm(command), .005)

    def test_rear_request_near_authority_retains_more_turning_response(self):
        angular = np.array([0., 0., .005])
        saturated, saturated_fuel = self.allocate(np.array([.5, 0., 0.]), angular)
        calibrated, calibrated_fuel = self.allocate(np.array([.09, 0., 0.]), angular)
        self.assertGreater(calibrated[2], 4. * saturated[2])
        self.assertLessEqual(calibrated_fuel, saturated_fuel)


if __name__ == '__main__':
    unittest.main()
