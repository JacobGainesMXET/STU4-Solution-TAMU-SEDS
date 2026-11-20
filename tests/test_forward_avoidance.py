"""Forward-avoidance geometry and integration checks; no simulator connection."""
import math
import unittest

import numpy as np

from Guidance import (MissionModel, PredictionScene, AvoidancePlanner, Encounter, burn_displacement, nonbraking)
from test_guidance import track


class ForwardAvoidanceTests(unittest.TestCase):
    def setUp(self):
        self.model = MissionModel(planning_accel=.12, process_accel_sigma=0.)
        self.zero = np.zeros(3)
        self.goal = np.array([1., 0., 0.])

    def scene(self, position, velocity):
        return PredictionScene([track(position, velocity)], 0., self.zero, self.zero,
                                self.zero, self.model)

    def choose(self, scene, **kwargs):
        return AvoidancePlanner(self.model).choose(scene, self.zero, 8. * self.goal,
                                                  self.goal, 0., **kwargs)

    def test_finite_burn_stops_accelerating_and_continues_coasting(self):
        times = np.array([0., 2., 8., 10.])
        delayed, instant = burn_displacement(times, [.12, 0., 0.], 2.)
        np.testing.assert_allclose(instant[:, 0], [0., .24, 1.68, 2.16])
        np.testing.assert_array_equal(instant, delayed)

    def test_delayed_turn_does_not_assume_instantaneous_sideways_thrust(self):
        angle = math.radians(20.)
        acceleration = .12 * np.array([math.cos(angle), math.sin(angle), 0.])
        times = np.array([0., 2., 8., 9., 10.])
        delayed, instant = burn_displacement(times, acceleration, 8., self.goal)
        self.assertEqual(delayed[1, 1], 0.)
        self.assertGreater(instant[1, 1], 0.)
        self.assertLess(delayed[2, 1], instant[2, 1])
        np.testing.assert_allclose(delayed[-1] - delayed[-2], delayed[-2] - delayed[-3])

    def test_straight_boost_can_clear_a_crossing_without_a_sidestep(self):
        scene = self.scene([120., 200., 0.], [-4., -200. / 30., 0.])
        decision = self.choose(scene, nose=self.goal)
        self.assertLess(decision.baseline.clearance, 0.)
        self.assertTrue(decision.forward_burn)
        self.assertEqual(decision.strategy, 'FORWARD')
        self.assertEqual(decision.burn_seconds, 8.)
        self.assertGreaterEqual(decision.encounter.clearance, 0.)
        self.assertGreater(decision.acceleration[0], 0.)
        np.testing.assert_allclose(decision.acceleration[1:], self.zero[1:])
        self.assertLess(scene.assess(decision.acceleration, 2., self.goal).clearance, 0.)
        self.assertLess(scene.assess(decision.acceleration, 4., self.goal).clearance, 0.)

    def test_shallow_turn_is_preferred_when_it_predicts_safe_clearance(self):
        scene = self.scene([100., 8.5, 0.], [-4., 0., 0.])
        # The nose already has a shallow useful offset from the velocity vector;
        # don't assume an unachievable instantaneous turn from a centred nose.
        angle = math.radians(5.)
        nose = np.array([math.cos(angle), -math.sin(angle), 0.])
        decision = self.choose(scene, nose=nose)
        self.assertEqual(decision.strategy, 'FORWARD')
        cosine = decision.acceleration @ self.goal / np.linalg.norm(decision.acceleration)
        self.assertGreaterEqual(cosine, math.cos(math.radians(20.)) - 1e-12)
        self.assertGreaterEqual(decision.encounter.clearance, 0.)
        self.assertGreater(np.linalg.norm(decision.acceleration[1:]), 0.)

    def test_safe_original_path_does_not_receive_an_extra_boost(self):
        decision = self.choose(self.scene([100., 40., 0.], [-4., 0., 0.]), nose=self.goal)
        self.assertEqual(decision.strategy, 'GUIDANCE')
        self.assertFalse(decision.forward_burn)
        np.testing.assert_array_equal(decision.acceleration, self.zero)

    def test_blocked_head_on_reports_remaining_risk_while_coasting(self):
        decision = self.choose(self.scene([30., 0., 0.], [-4., 0., 0.]), nose=self.goal)
        self.assertFalse(decision.forward_burn)
        self.assertEqual(decision.strategy, 'COAST_LIMITED_CLEARANCE')
        self.assertEqual(decision.mode, 'AVOID')
        self.assertTrue(math.isfinite(decision.encounter.physical_time))

    def test_stale_sensor_inhibition_cannot_be_bypassed_by_a_forward_boost(self):
        scene = self.scene([120., 200., 0.], [-4., -200. / 30., 0.])
        self.assertTrue(self.choose(scene, nose=self.goal).forward_burn)
        inhibited = self.choose(scene, nose=self.goal, allow_forward=False)
        self.assertFalse(inhibited.forward_burn)

    def test_nose_facing_away_cannot_request_a_rear_thrust_dodge(self):
        scene = self.scene([120., 200., 0.], [-4., -200. / 30., 0.])
        decision = self.choose(scene, nose=-self.goal)
        self.assertFalse(decision.forward_burn)

    def test_clear_path_stops_the_burn_without_finishing_an_unnecessary_turn(self):
        planner = AvoidancePlanner(self.model)
        scene = self.scene([120., 200., 0.], [-4., -200. / 30., 0.])
        first = planner.choose(scene, self.zero, 8. * self.goal, self.goal, 0., nose=self.goal)
        clear = PredictionScene([], .25, self.zero, self.zero, self.zero, self.model)
        released = planner.choose(clear, self.zero, 8. * self.goal, self.goal, .25, nose=self.goal)
        self.assertTrue(first.forward_burn)
        self.assertFalse(released.forward_burn)
        np.testing.assert_array_equal(released.acceleration, self.zero)
        self.assertTrue(planner.active)  # retain direction history without firing

    def test_committed_burn_counts_down_and_new_hazard_can_interrupt_it(self):
        planner = AvoidancePlanner(self.model)
        scene = self.scene([120., 200., 0.], [-4., -200. / 30., 0.])
        first = planner.choose(scene, self.zero, 8. * self.goal, self.goal, 0., nose=self.goal)
        retained = planner.choose(scene, self.zero, 8. * self.goal, self.goal, .25, nose=self.goal)
        self.assertTrue(retained.forward_burn)
        self.assertAlmostEqual(retained.burn_seconds, first.burn_seconds - .25)
        np.testing.assert_array_equal(retained.acceleration, first.acceleration)
        blocked = planner.choose(self.scene([15., 0., 0.], [-4., 0., 0.]), self.zero,
                                 8. * self.goal, self.goal, .5, nose=self.goal)
        self.assertEqual(blocked.strategy, 'COAST_LIMITED_CLEARANCE')
        np.testing.assert_array_equal(blocked.acceleration, self.zero)

    def test_prediction_rejects_a_burn_longer_than_its_broadphase_envelope(self):
        scene = self.scene([100., 0., 0.], [-4., 0., 0.])
        with self.assertRaises(ValueError):
            scene.assess([.12, 0., 0.], 16.01)

    def test_final_new_closing_track_remains_visible_beyond_eight_seconds(self):
        rock = track([100., 0., 0.], [-8., 0., 0.], hits=1)
        early = PredictionScene([rock], 0., self.zero, self.zero, self.zero, self.model)
        terminal = PredictionScene([rock], 0., self.zero, self.zero, self.zero, self.model, terminal=True)
        self.assertGreater(early.assess(self.zero).clearance, 0.)
        self.assertLess(terminal.assess(self.zero).hull_gap, 0.)

    def test_oblique_escape_clears_head_on_without_reversing(self):
        from offline_scenarios import run
        result = run(obstacle=([500., 0., 0.], [-1., 0., 0.]), seed=23, score_cruise=True)
        self.assertTrue(result['arrived'])
        self.assertFalse(result['collision'])
        self.assertGreater(result['avoidance_strategies'].get('VECTOR_ESCAPE', 0), 0)

    def test_safe_coast_precedes_a_safe_boost_when_nominal_burn_would_hit(self):
        class Crossing:
            def assess(_, acceleration, burn_seconds=8., initial_direction=None):
                gap = -1. if burn_seconds == 8. and np.linalg.norm(acceleration) else 2.
                return Encounter(clearance=gap, time=8.)
        choice = AvoidancePlanner(self.model).choose(Crossing(), .09*self.goal, 8.*self.goal,
                                                     self.goal, 42., nose=self.goal)
        self.assertEqual(choice.strategy, 'COAST')
        np.testing.assert_array_equal(choice.acceleration, self.zero)

    def test_short_transit_burn_expires_until_a_new_threat_observation(self):
        class Crossing:
            departure = False
            age = 0.
            coast_gap = 30.

            def assess(scene, acceleration, burn_seconds=8., initial_direction=None):
                gap = scene.coast_gap if not np.any(acceleration) else (5. if burn_seconds <= 2. else -1.)
                return Encounter(clearance=gap, age=scene.age)

        scene, planner = Crossing(), AvoidancePlanner(self.model)
        def select(now):
            return planner.choose(scene, .12*self.goal, 8.*self.goal, self.goal, now,
                                  nose=self.goal, transit_burn=True)
        for now, age, strategy in [(42., 0., 'SHORT_FORWARD'), (43., 1., 'SHORT_FORWARD'),
                                   (44., 2., 'COAST'), (45., 0., 'SHORT_FORWARD')]:
            scene.age = age
            choice = select(now)
            self.assertEqual(choice.strategy, strategy)
            if now == 43.:
                self.assertEqual(choice.burn_seconds, 1.)
        scene.coast_gap = 4.
        self.assertEqual(select(45.1).strategy, 'COAST')
        scene.coast_gap, scene.departure = 30., True
        self.assertEqual(select(45.2).strategy, 'COAST')

    def test_no_reverse_even_when_it_would_be_the_only_clear_action(self):
        class Blocked:
            def __init__(self, near_miss=False):
                self.near_miss = near_miss

            def assess(scene, acceleration, burn_seconds=8., initial_direction=None):
                if acceleration[0] < 0.:
                    return Encounter(clearance=2.)
                contact = math.inf if scene.near_miss and acceleration[1] > .02 else 4.
                return Encounter(clearance=-2., time=2., physical_time=contact)
        for now, near_miss in ((19.99, False), (20., False), (25., True), (90., False), (100., False)):
            plan = AvoidancePlanner(self.model).choose(Blocked(near_miss), .09*self.goal,
                                                       self.goal*4., self.goal, now, nose=self.goal,
                                                       launch=now < 20.)
            self.assertLess(plan.encounter.clearance, 0.)
            self.assertGreaterEqual(plan.acceleration @ self.goal, 0.)
            if now < 20.:
                self.assertEqual(plan.strategy, 'LAUNCH_STRAIGHT_LIMITED_CLEARANCE')
                np.testing.assert_array_equal(plan.acceleration, .09 * self.goal)
            else:
                self.assertIn(plan.strategy, ('COAST_LIMITED_CLEARANCE', 'FORWARD_LIMITED_CLEARANCE'))

    def test_handoff_coasts_only_with_a_marginal_burn_and_a_clear_moving_coast(self):
        clear = self.scene([80., 12., 0.], [-4., 0., 0.])
        planner = AvoidancePlanner(self.model)
        choice = planner.choose(clear, .09*self.goal, self.goal, self.goal, 20., plan_on_coast=True)
        self.assertEqual(choice.mode, 'PLAN_COAST')
        stopped = planner.choose(clear, .09*self.goal, self.zero, self.goal, 20., plan_on_coast=True)
        self.assertEqual(stopped.mode, 'GUIDANCE')
        open_corridor = self.scene([80., 60., 0.], [-4., 0., 0.])
        self.assertEqual(planner.choose(open_corridor, .09*self.goal, self.goal, self.goal,
                                        20., plan_on_coast=True).mode, 'GUIDANCE')
        self.assertEqual(planner.choose(open_corridor, .09*self.goal, self.goal, self.goal,
                                        20., plan_on_coast=True, route_uncertain=True).mode, 'PLAN_COAST')
        blocked = self.scene([30., 0., 0.], [-4., 0., 0.])
        self.assertEqual(planner.choose(blocked, .09*self.goal, self.goal, self.goal, 20.,
                                        plan_on_coast=True).mode, 'AVOID')

    def test_launch_offset_clears_a_crossing_without_waiting_for_nose_alignment(self):
        scene = self.scene([30., 7., 0.], [-1., 0., 0.])
        plan = AvoidancePlanner(self.model).choose(scene, .12*self.goal, 2.*self.goal,
                                                  self.goal, 6., nose=-self.goal, launch=True)
        self.assertLess(plan.baseline.clearance, 0.)
        self.assertGreaterEqual(plan.encounter.clearance, 0.)
        self.assertTrue(plan.launch_vector)
        self.assertEqual(plan.strategy, 'LAUNCH_TILT')
        self.assertGreaterEqual(plan.acceleration[0], .12*math.cos(math.radians(20.)) - 1e-10)
        self.assertLess(plan.acceleration[1], 0.)

    def test_launch_can_wait_on_pad_if_only_coast_is_clear(self):
        class BlockedLift:
            def assess(_, acceleration, burn_seconds=8., initial_direction=None):
                return Encounter(clearance=-2. if np.any(acceleration) else 2.)
        plan = AvoidancePlanner(self.model).choose(BlockedLift(), .12*self.goal, self.zero,
                                                  self.goal, 1., launch=True)
        self.assertEqual(plan.strategy, 'LAUNCH_COAST')
        np.testing.assert_array_equal(plan.acceleration, self.zero)

    def test_launch_timing_checks_the_allocator_acceleration_range(self):
        rock = track([27., 32., 0.], [0., -1.6, 0.])
        def gap(departure):
            scene = PredictionScene([rock], 0., self.zero, self.zero, self.zero, self.model,
                                    departure=departure, horizon=30.)
            return scene.assess(.12*self.goal, 8.).clearance
        self.assertGreater(gap(False), 0.)
        self.assertLess(gap(True), 0.)

    def test_closed_loop_directly_blocked_departure(self):
        from offline_scenarios import run
        result = run(obstacle=([65., 0., 0.], [-.55, 0., 0.]), seed=7)
        self.assertTrue(result['arrived'])
        self.assertFalse(result['collision'])
        self.assertGreater(result['min_true_gap'], 0.)
        self.assertTrue(any(name.startswith('LAUNCH_TILT') for name in result['avoidance_strategies']))

    def test_extend_launch_prediction_only_for_well_observed_tracks(self):
        rock = track([80., 0., 0.], [-2., 0., 0.], hits=7)
        def coast():
            return PredictionScene([rock], 0., self.zero, self.zero, self.zero, self.model,
                                   departure=True).assess(self.zero)
        self.assertAlmostEqual(coast().closest_time, 40.)
        self.assertLess(coast().hull_gap, 0.)
        rock.reliable_hits = 1
        self.assertGreater(coast().clearance, 0.)

    def test_course_correction_preserves_speed_and_target_progress(self):
        v = np.array([5., 2., 1.])
        for cmd in (np.array([-1., .1, 0.]), np.array([0., -.1, 0.]), -v):
            adjusted = nonbraking(cmd, v, self.goal)
            self.assertGreaterEqual(adjusted @ v, -1e-12)
            self.assertGreaterEqual(adjusted @ self.goal, -1e-12)

    def test_swept_turn_corridor_catches_a_rock_between_steering_timings(self):
        acceleration = np.array([0., .09, 0.])
        delayed, instant = burn_displacement(np.array([30.]), acceleration, 8., self.goal)
        midpoint = .5 * (delayed[0] + instant[0])
        velocity = np.array([-10., 0., 0.])
        scene = self.scene(midpoint - velocity * 30., velocity)
        self.assertGreater(scene.assess(acceleration).clearance, 0.)
        self.assertLess(scene.assess(acceleration, initial_direction=self.goal).clearance, 0.)


if __name__ == '__main__':
    unittest.main()
