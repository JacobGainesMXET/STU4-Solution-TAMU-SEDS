"""Finite guidance burns and cruise decisions, without importing the simulator."""
import math
import unittest

import numpy as np

from Guidance import (AccelerationGuidance, MissionModel, ScheduledBurn,
                      ballistic_entry, finite_burn_entry, PredictionScene, AvoidancePlanner)
from test_guidance import track


class BurnCoastBurnTests(unittest.TestCase):
    def setUp(self):
        self.zero = np.zeros(3)
        self.reference = np.array([0., 1., 0.])
        self.model = MissionModel(coast_speed=10., cruise_burns=True,
                                  flyby_offset=0., fuel_per_dv=4.4)
        self.guidance = AccelerationGuidance(self.model)

    def decide(self, distance, velocity, time=100., gravity=None, fuel=40.):
        return self.guidance.decide(np.array([distance, 0., 0.]), np.array(velocity, dtype=float),
                                    self.zero if gravity is None else np.array(gravity),
                                    time, fuel, self.reference)

    def start_coast(self):
        return self.decide(1300., [10., 0., 0.], time=0.)

    def test_launch_does_not_coast_early_just_because_eight_meter_path_can_arrive(self):
        decision = self.decide(500., [8., 0., 0.])
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertGreater(decision.acceleration[0], 0.)

    def test_initial_cutoff_uses_progress_toward_target_not_sideways_speed(self):
        decision = self.decide(1000., [.1, 20., 0.])
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertFalse(self.guidance.coast_latched)

    def test_ten_meter_closing_speed_starts_a_safe_unpowered_coast(self):
        decision = self.start_coast()
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        np.testing.assert_array_equal(decision.acceleration, self.zero)
        self.assertAlmostEqual(decision.coast_entry, (1300. - 42.) / 10.)

    def test_disabling_cutoff_preserves_the_previous_short_horizon_coast(self):
        self.model.coast_speed = None
        decision = self.decide(500., [8., 0., 0.])
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertIsNone(self.guidance.burn)

    def test_slight_gravity_deceleration_does_not_create_a_speed_servo(self):
        self.start_coast()
        decision = self.decide(900., [9.8, 0., 0.], time=40., gravity=[-.005, 0., 0.])
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        np.testing.assert_array_equal(decision.acceleration, self.zero)

    def test_speed_restoration_burn_requires_positive_score_advantage(self):
        self.start_coast()
        decision = self.decide(900., [9., 0., 0.], time=10., gravity=[-.001, 0., 0.])
        self.assertEqual(decision.mode, 'REBOOST')
        self.assertGreater(decision.advantage, self.model.reburn_score_margin)
        self.assertGreater(decision.burn_seconds, 0.)
        self.assertLessEqual(decision.burn_seconds, 8.)
        self.assertGreater(decision.acceleration[0], 0.)

    def test_late_fuel_expensive_topup_is_rejected_even_below_speed_band(self):
        self.start_coast()
        decision = self.decide(900., [9., 0., 0.], time=100.)
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertLess(decision.advantage, 0.)
        np.testing.assert_array_equal(decision.acceleration, self.zero)

    def test_three_tenths_speed_loss_prices_a_short_topup(self):
        self.start_coast()
        self.assertEqual(self.decide(900., [9.71, 0., 0.], time=10.).mode, 'CLEAR_COAST')
        burn = self.decide(900., [9.7, 0., 0.], time=10., gravity=[-.001, 0., 0.])
        self.assertEqual(burn.mode, 'REBOOST')
        self.assertLessEqual(burn.burn_seconds, .3 / (.09 - .001) + 1e-8)

    def test_optional_topup_does_not_pay_for_a_large_attitude_change(self):
        self.start_coast()
        decision = self.guidance.decide(np.array([900., 0., 0.]), np.array([9., 0., 0.]),
                                        self.zero, 10., 40., self.reference, nose=np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertIsNone(self.guidance.burn)

    def test_topup_has_minimum_coast_time_and_final_approach_exclusion(self):
        self.start_coast()
        early = self.decide(900., [9., 0., 0.], time=2.)
        near = self.decide(150., [9., 0., 0.], time=10.)
        self.assertEqual(early.mode, 'CLEAR_COAST')
        self.assertEqual(near.mode, 'DISTANCE_COAST')
        self.assertIsNone(self.guidance.burn)

    def test_completed_burn_cannot_immediately_restart_for_a_small_speed_loss(self):
        self.start_coast()
        self.assertEqual(self.decide(900., [9., 0., 0.], time=10.).mode, 'REBOOST')
        self.guidance.finish_burn(12.)
        self.assertEqual(self.decide(850., [9., 0., 0.], time=13.).mode, 'CLEAR_COAST')
        self.assertIsNone(self.guidance.burn)

    def test_lateral_miss_gets_a_forward_banked_trim(self):
        decision = self.decide(1000., [10., 1., 0.])
        self.assertEqual(decision.mode, 'TRIM_BURN')
        self.assertGreater(decision.acceleration @ np.array([10., 1., 0.]), 0.)
        self.assertAlmostEqual(-decision.acceleration[1] / decision.acceleration[0], math.tan(math.radians(30.)))
        self.assertLess(decision.acceleration[1], 0.)
        self.assertGreater(decision.burn_seconds, 0.)
        self.assertLessEqual(decision.burn_seconds, 8.)
        # A single banked burn may only reduce the miss. Propagate to the target plane.
        t = decision.burn_seconds
        v = np.array([10., 1., 0.]) + t * decision.acceleration
        p = t * np.array([10., 1., 0.]) + .5 * decision.acceleration * t**2
        self.assertLess(abs(p[1] + (1000. - p[0]) * v[1] / v[0]), 100.)

    def test_gravity_curving_the_path_away_gets_a_trim_regardless_of_fuel_cost(self):
        self.model.fuel_per_dv = 1e6
        decision = self.decide(1000., [10., 0., 0.], gravity=[0., .012, 0.])
        self.assertEqual(decision.mode, 'TRIM_BURN')
        self.assertLess(decision.acceleration[1], 0.)

    def test_coast_reacquisition_cancels_the_rest_of_a_trim(self):
        self.decide(1000., [10., 1., 0.])
        decision = self.decide(990., [10., 0., 0.], time=101.)
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertIsNone(self.guidance.burn)
        np.testing.assert_array_equal(decision.acceleration, self.zero)

    def test_final_coast_can_begin_before_launch_speed_and_never_brakes(self):
        decision = self.decide(70., [8., 0., 0.])
        self.assertEqual(decision.mode, 'FINAL_COAST')
        np.testing.assert_array_equal(decision.acceleration, self.zero)
        self.assertEqual(self.decide(40., [10., 0., 0.]).mode, 'ARRIVED')

    def test_forward_gravity_stopping_short_resumes_the_main_burn(self):
        self.start_coast()
        decision = self.decide(2000., [10., 0., 0.], gravity=[-.04, 0., 0.])
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertGreater(decision.acceleration[0], 0.)

    def test_delivered_impulse_can_finish_a_burn_before_the_wall_clock_deadline(self):
        burn = ScheduledBurn(np.array([.09, 0., 0.]), 10., np.array([10., 0., 0.]),
                              np.array([-.01, 0., 0.]), 2., 'REBOOST')
        self.assertAlmostEqual(burn.remaining(10.5, np.array([10.04, 0., 0.]), burn.gravity), 1.5)
        self.assertAlmostEqual(burn.remaining(11., np.array([10.17, 0., 0.]), burn.gravity), 0.)
        self.assertEqual(burn.remaining(12., np.array([10., 0., 0.]), burn.gravity), 0.)

    def test_finite_burn_forecast_includes_gravity_and_subsequent_coast(self):
        entry = finite_burn_entry([100., 0., 0.], [8., 0., 0.], self.zero, [.1, 0., 0.], 2., 42.)
        self.assertAlmostEqual(entry, 2. + (100. - 16.2 - 42.) / 8.2)
        no_burn = finite_burn_entry([100., 0., 0.], [8., 0., 0.], [-.01, 0., 0.], self.zero, 2., 42.)
        self.assertAlmostEqual(no_burn, ballistic_entry([100., 0., 0.], [8., 0., 0.], [-.01, 0., 0.], 42., 600.))
        during = finite_burn_entry([60., 0., 0.], [10., 0., 0.], self.zero, [.1, 0., 0.], 8., 42.)
        self.assertAlmostEqual(during, (-10. + math.sqrt(100. + 3.6)) / .1)

    def test_avoidance_checks_the_actual_finite_nominal_burn_duration(self):
        model = MissionModel(planning_accel=.12, process_accel_sigma=0.)
        scene = PredictionScene([track([120., 200., 0.], [-4., -200. / 30., 0.])],
                                0., self.zero, self.zero, self.zero, model)
        nominal = np.array([.09, 0., 0.])
        goal = np.array([1., 0., 0.])
        short = AvoidancePlanner(model).choose(scene, nominal, 10. * goal, goal, 0., nominal_burn_seconds=2.)
        long = AvoidancePlanner(model).choose(scene, nominal, 10. * goal, goal, 0., nominal_burn_seconds=8.)
        self.assertLess(short.baseline.clearance, 0.)
        self.assertGreaterEqual(long.baseline.clearance, 0.)
        self.assertEqual(long.mode, 'GUIDANCE')


if __name__ == '__main__':
    unittest.main()
