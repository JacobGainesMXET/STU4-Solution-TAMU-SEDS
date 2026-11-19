from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from FlightRecorder import FlightRecorder
from Guidance import (MissionModel, StateHistory, RadarTracker, DebrisTrack,
                      AccelerationGuidance, PredictionScene, AvoidancePlanner,
                      PulseController, AccelerationAttitude, travel_time, learn_burn_model,
                      alignment_strength, angular_command, ballistic_entry, pointing_tolerance)


@dataclass
class Sample:
    rel_pos: np.ndarray
    surface_range: float
    range_rate: float
    diameter: float
    t_meas: float

    @property
    def center_range(self):
        return np.linalg.norm(self.rel_pos)


def sample(world_position, world_velocity, ship_position, ship_velocity, stamp, diameter=4.):
    relative = np.asarray(world_position, dtype=float) - ship_position
    distance = np.linalg.norm(relative)
    radial = float((np.asarray(world_velocity) - ship_velocity) @ relative / distance)
    return Sample(relative, distance - .5 * diameter, radial, diameter, stamp)


def track(position, velocity, number=1, covariance=1e-5, hits=5, radius=2.):
    return DebrisTrack(number, 0., np.r_[position, velocity].astype(float), np.eye(6) * covariance, radius, hits)


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.model = MissionModel()
        self.history = StateHistory()
        for stamp in np.arange(0., 17., .25):
            self.history.add(stamp, np.array([5. * stamp, 0., 0.]), np.array([5., 0., 0.]))
        self.tracker = RadarTracker(self.model)

    def observation(self, stamp, origin=(100., 10., 0.), velocity=(-1., .4, 0.), diameter=4.):
        return sample(np.array(origin) + np.array(velocity) * stamp, velocity,
                      np.array([5. * stamp, 0., 0.]), np.array([5., 0., 0.]), stamp, diameter)

    def test_own_state_history_interpolates_measurement_time(self):
        position, velocity = self.history.at(3.1)
        np.testing.assert_allclose(position, [15.5, 0., 0.])
        np.testing.assert_allclose(velocity, [5., 0., 0.])
        self.assertIsNone(self.history.at(-2.))

    def test_old_positions_are_corrected_for_own_ship_motion(self):
        for stamp in (0., 4., 8.):
            self.tracker.update([self.observation(stamp)], stamp + 2., self.history, np.zeros(3))
        self.assertEqual(len(self.tracker.tracks), 1)
        result = next(iter(self.tracker.tracks.values()))
        np.testing.assert_allclose(result.state[:3], [92., 13.2, 0.], atol=.3)
        np.testing.assert_allclose(result.state[3:], [-1., .4, 0.], atol=.15)

    def test_repeated_snapshot_does_not_create_confidence(self):
        observation = self.observation(0.)
        self.tracker.update([observation], .1, self.history, np.zeros(3))
        before = next(iter(self.tracker.tracks.values())).covariance.copy()
        for now in (.2, .4, .6):
            self.tracker.update([observation], now, self.history, np.zeros(3))
        result = next(iter(self.tracker.tracks.values()))
        self.assertEqual(result.hits, 1)
        np.testing.assert_array_equal(result.covariance, before)
        # Independent range also constrains the radial component of one fix.
        self.assertGreater(result.covariance[0, 0], 1.9)
        self.assertGreater(result.covariance[2, 2], 3.9)

    def test_range_rate_does_not_invent_transverse_velocity(self):
        self.tracker.update([self.observation(0., origin=(100., 0., 0.))], .1, self.history, np.zeros(3))
        result = next(iter(self.tracker.tracks.values()))
        self.assertGreater(result.covariance[4, 4], 15.)
        self.assertGreater(result.covariance[5, 5], 15.)
        self.assertLess(result.covariance[3, 3], 1.)

    def test_reordering_contacts_does_not_exchange_track_identities(self):
        origins = [(80., 15., 0.), (130., -15., 0.)]
        for stamp in (0., 4., 8.):
            observations = [self.observation(stamp, origin=origin, diameter=4. + index * 8.)
                            for index, origin in enumerate(origins)]
            if stamp == 4.:
                observations.reverse()
            self.tracker.update(observations, stamp + .2, self.history, np.zeros(3))
        self.assertEqual(len(self.tracker.tracks), 2)
        self.assertLess(self.tracker.tracks[1].radius, self.tracker.tracks[2].radius)
        self.assertEqual(self.tracker.tracks[1].hits, 3)

    def test_invalid_future_and_inconsistent_samples_are_rejected(self):
        nan = self.observation(0.)
        nan.rel_pos[0] = math.nan
        inconsistent = self.observation(0.)
        inconsistent.surface_range = -1000.
        future = self.observation(8.)
        self.tracker.update([nan, inconsistent, future], 1., self.history, np.zeros(3))
        self.assertEqual(len(self.tracker.tracks), 0)
        self.assertEqual(self.tracker.stats['rejected'], 3)

    def test_missing_measurements_age_tracks_and_disable_ready_state(self):
        for stamp in (0., 4.):
            self.tracker.update([self.observation(stamp)], stamp + .1, self.history, np.zeros(3))
        self.assertTrue(self.tracker.ready(4.2))
        self.assertFalse(self.tracker.ready(8.))
        self.tracker.update([], 17., self.history, np.zeros(3))
        self.assertEqual(len(self.tracker.tracks), 0)

    def test_covariances_remain_positive_and_grow_with_measurement_age(self):
        for stamp in (0., 4., 8., 12.):
            self.tracker.update([self.observation(stamp)], stamp + .1, self.history, np.zeros(3))
        result = next(iter(self.tracker.tracks.values()))
        _, predicted = result.predict(16., np.zeros(3), self.model)
        self.assertGreater(np.trace(predicted[:3, :3]), np.trace(result.covariance[:3, :3]))
        self.assertGreaterEqual(np.min(np.linalg.eigvalsh(predicted)), -1e-10)

    def test_accelerating_ship_does_not_look_like_accelerating_debris(self):
        history = StateHistory()
        for stamp in np.arange(0., 13., .1):
            history.add(stamp, [.04 * stamp**2, 0., 0.], [.08 * stamp, 0., 0.])
        for stamp in (0., 4., 8., 12.):
            observation = sample([100., stamp, 0.], [0., 1., 0.],
                                 [.04 * stamp**2, 0., 0.], [.08 * stamp, 0., 0.], stamp)
            self.tracker.update([observation], stamp + .1, history, np.zeros(3))
        self.assertEqual(len(self.tracker.tracks), 1)
        np.testing.assert_allclose(self.tracker.tracks[1].state[3:], [0., 1., 0.], atol=.12)


class PredictionTests(unittest.TestCase):
    def setUp(self):
        self.model = MissionModel(process_accel_sigma=0.)

    def scene(self, tracks):
        return PredictionScene(tracks, 0., np.zeros(3), np.zeros(3), np.zeros(3), self.model)

    def test_detects_crossing_motion_that_is_not_on_current_nose_line(self):
        scene = self.scene([track([30., 60., 0.], [-2., -4., 0.])])
        encounter = scene.assess(np.zeros(3))
        self.assertLess(encounter.clearance, 0.)
        self.assertLess(encounter.time, 15.)
        self.assertAlmostEqual(encounter.closest_time, 15., delta=.5)

    def test_safe_near_pass_does_not_force_a_maneuver(self):
        scene = self.scene([track([60., 25., 0.], [-4., 0., 0.])])
        decision = AvoidancePlanner(self.model).choose(scene, np.zeros(3), np.zeros(3), np.array([1., 0., 0.]), 0.)
        self.assertEqual(decision.mode, 'GUIDANCE')
        np.testing.assert_array_equal(decision.acceleration, np.zeros(3))

    def test_receding_rock_does_not_force_a_maneuver(self):
        scene = self.scene([track([25., 0., 0.], [3., 0., 0.])])
        self.assertGreater(scene.assess(np.zeros(3)).clearance, 0.)

    def test_old_or_uncertain_tracks_require_more_clearance(self):
        confident = self.scene([track([60., 25., 0.], [-4., 0., 0.], covariance=.0001)])
        uncertain = self.scene([track([60., 25., 0.], [-4., 0., 0.], covariance=.25)])
        self.assertLess(uncertain.assess(np.zeros(3)).clearance, confident.assess(np.zeros(3)).clearance)

    def test_checks_all_obstacles_when_choosing_a_dodge(self):
        scene = self.scene([track([100., 0., 0.], [-4., 0., 0.]),
                            track([100., 12., 0.], [-4., 0., 0.], number=2)])
        decision = AvoidancePlanner(self.model).choose(scene, np.zeros(3), np.zeros(3), np.array([1., 0., 0.]), 0.)
        self.assertGreater(decision.candidates, 10)
        self.assertGreater(decision.encounter.clearance, decision.baseline.clearance)
        individual = [self.scene([rock]).assess(decision.acceleration, decision.burn_seconds)
                      for rock in scene.tracks]
        self.assertAlmostEqual(decision.encounter.clearance, min(hit.clearance for hit in individual))
        self.assertEqual(decision.strategy, 'FORWARD_LIMITED_CLEARANCE')
        self.assertLessEqual(np.linalg.norm(decision.acceleration), self.model.avoidance_accel + 1e-12)

    def test_avoidance_takes_priority_over_forward_burn(self):
        scene = self.scene([track([100., 0., 0.], [-4., 0., 0.])])
        nominal = np.array([self.model.planning_accel, 0., 0.])
        decision = AvoidancePlanner(self.model).choose(scene, nominal, np.zeros(3), nominal, 0.)
        self.assertLess(decision.baseline.clearance, 0.)
        self.assertFalse(np.allclose(decision.acceleration, nominal))
        self.assertGreater(decision.encounter.clearance, decision.baseline.clearance)

    def test_avoidance_release_has_hysteresis(self):
        planner = AvoidancePlanner(self.model)
        danger = self.scene([track([100., 0., 0.], [-4., 0., 0.])])
        clear = self.scene([])
        planner.choose(danger, np.zeros(3), np.zeros(3), np.array([1., 0., 0.]), 0.)
        planner.choose(clear, np.zeros(3), np.zeros(3), np.array([1., 0., 0.]), .2)
        self.assertTrue(planner.active)
        planner.choose(clear, np.zeros(3), np.zeros(3), np.array([1., 0., 0.]), 2.)
        self.assertFalse(planner.active)

    def test_between_sample_collision_gets_a_speed_margin(self):
        scene = self.scene([track([21., 0., 0.], [-20., 0., 0.], radius=.1)])
        self.assertLess(scene.assess(np.zeros(3)).clearance, 0.)

    def test_broadphase_covers_high_calibrated_forward_authority(self):
        model = MissionModel(planning_accel=.16, process_accel_sigma=0.)
        scene = PredictionScene([track([102., 0., 0.], [-2., 0., 0.])], 0.,
                                np.zeros(3), np.zeros(3), np.zeros(3), model)
        self.assertEqual(len(scene.tracks), 1)
        self.assertLess(scene.assess(np.array([.16, 0., 0.])).clearance, 0.)


class GuidanceTests(unittest.TestCase):


    def test_ballistic_entry_handles_straight_and_offset_intercepts(self):
        straight = ballistic_entry([100., 0., 0.], [10., 0., 0.], [0., 0., 0.], 42., 90.)
        self.assertAlmostEqual(straight, 5.8)
        offset = ballistic_entry([100., 30., 0.], [10., 0., 0.], [0., 0., 0.], 42., 90.)
        self.assertAlmostEqual(offset, (100. - math.sqrt(42.**2 - 30.**2)) / 10.)

    def test_aimed_nose_is_not_enough_when_velocity_or_gravity_will_miss(self):
        self.assertEqual(ballistic_entry([100., 60., 0.], [10., 0., 0.], [0., 0., 0.], 42., 90.), math.inf)
        self.assertEqual(ballistic_entry([300., 0., 0.], [10., 0., 0.], [0., .2, 0.], 42., 90.), math.inf)
        self.assertEqual(ballistic_entry([100., 0., 0.], [1., 0., 0.], [-.1, 0., 0.], 42., 90.), math.inf)

    def test_clear_ballistic_intercept_requests_zero_thrust_before_final_approach(self):
        guidance = AccelerationGuidance(MissionModel())
        decision = guidance.decide(np.array([500., 0., 0.]), np.array([10., 0., 0.]),
                                   np.zeros(3), 100., 40., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertAlmostEqual(decision.coast_entry, 45.8)
        np.testing.assert_array_equal(decision.acceleration, np.zeros(3))

    def test_ballistic_coast_resumes_guidance_when_intercept_is_lost(self):
        guidance = AccelerationGuidance(MissionModel())
        guidance.decide(np.array([300., 0., 0.]), np.array([10., 0., 0.]),
                        np.zeros(3), 100., 40., np.array([0., 1., 0.]))
        decision = guidance.decide(np.array([300., 0., 0.]), np.array([0., 10., 0.]),
                                   np.zeros(3), 101., 40., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertFalse(guidance.ballistic_coasting)

    def test_calibrates_burn_authority_and_fuel_from_delivered_impulse(self):
        model = MissionModel()
        for _ in range(40):
            learn_burn_model(model, 4.32 * .14, .14, 0., 4.32, 1.)
        self.assertAlmostEqual(model.fuel_per_dv, 4.32, delta=.03)
        self.assertAlmostEqual(model.planning_accel, .95 * .14, delta=.001)

    def test_does_not_calibrate_from_spin_or_impact_impulses(self):
        model = MissionModel()
        before = model.planning_accel
        learn_burn_model(model, 1., .14, .1, 4.32, 1.)
        self.assertEqual(model.planning_accel, before)
        learn_burn_model(model, 1., 5., 0., 4.32, .1)
        self.assertEqual(model.planning_accel, before)

    def test_score_matches_reported_mission(self):
        self.assertAlmostEqual(MissionModel().score(202.33, 129.14), -62.88, places=1)
        self.assertEqual(MissionModel().score(0., -1.), 50.)

    def test_travel_time_handles_stopping_short_and_moving_away(self):
        self.assertEqual(travel_time(100., 0., 0.), math.inf)
        self.assertEqual(travel_time(100., 1., -.1), math.inf)
        self.assertGreater(travel_time(100., -2., .1), 20.)

    def test_clear_path_accelerates_without_a_five_meter_per_second_servo(self):
        for speed in (0., 5., 8., 12., 20.):
            guidance = AccelerationGuidance(MissionModel(flyby_offset=0.))
            decision = guidance.decide(np.array([5000., 0., 0.]), np.array([speed, 0., 0.]),
                                       np.zeros(3), 100., 50., np.array([0., 1., 0.]))
            self.assertEqual(decision.mode, 'ACCELERATE')
            self.assertAlmostEqual(decision.acceleration[0], guidance.model.planning_accel)

    def test_fuel_model_does_not_trigger_early_coasting(self):
        decisions = [AccelerationGuidance(MissionModel(fuel_per_dv=cost)).decide(
            np.array([1300., 0., 0.]), np.zeros(3), np.zeros(3), 0., 0., np.array([0., 1., 0.]))
            for cost in (4.4, 9.)]
        np.testing.assert_array_equal(decisions[0].acceleration, decisions[1].acceleration)

    def test_optional_coast_threshold_never_brakes_to_hold_speed(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
        for speed in (10., 7.):
            decision = guidance.decide(np.array([1300., 0., 0.]), np.array([speed, 0., 0.]),
                                       np.zeros(3), 100., 40., np.array([0., 1., 0.]))
            self.assertEqual(decision.mode, 'CLEAR_COAST')
            np.testing.assert_array_equal(decision.acceleration, np.zeros(3))

    def test_eight_meter_cutoff_allows_arrival_more_than_ninety_seconds_away(self):
        guidance = AccelerationGuidance(MissionModel(coast_speed=8.))
        decision = guidance.decide(np.array([1000., 0., 0.]), np.array([8., 0., 0.]),
                                   np.zeros(3), 90., 45., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CLEAR_COAST')
        self.assertAlmostEqual(decision.coast_entry, (1000. - 42.) / 8.)
        np.testing.assert_array_equal(decision.acceleration, np.zeros(3))

    def test_below_cutoff_distant_path_still_accelerates(self):
        guidance = AccelerationGuidance(MissionModel(coast_speed=8.))
        decision = guidance.decide(np.array([1000., 0., 0.]), np.array([7.9, 0., 0.]),
                                   np.zeros(3), 90., 45., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertGreater(decision.acceleration[0], 0.)

    def test_velocity_outside_arrival_cone_gets_lateral_correction_at_cutoff(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
        decision = guidance.decide(np.array([1000., 0., 0.]), np.array([8., 1., 0.]),
                                   np.zeros(3), 90., 45., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CORRECT')
        self.assertAlmostEqual(decision.acceleration[0], 0.)
        self.assertLess(decision.acceleration[1], 0.)
        self.assertLessEqual(np.linalg.norm(decision.acceleration), .4 * guidance.model.planning_accel)

    def test_cutoff_tolerance_depends_on_arrival_geometry(self):
        for lateral_offset, expected in ((40., 'CLEAR_COAST'), (60., 'CORRECT')):
            guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
            decision = guidance.decide(np.array([1000., lateral_offset, 0.]), np.array([8., 0., 0.]),
                                       np.zeros(3), 90., 45., np.array([0., 1., 0.]))
            self.assertEqual(decision.mode, expected)

    def test_gravity_curving_aimed_velocity_away_requires_correction(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
        decision = guidance.decide(np.array([1000., 0., 0.]), np.array([8., 0., 0.]),
                                   np.array([0., .01, 0.]), 90., 45., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CORRECT')
        self.assertAlmostEqual(decision.acceleration[0], 0.)
        self.assertLess(decision.acceleration[1], 0.)

    def test_gravity_stopping_short_resumes_forward_burn(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
        r, v, up = np.array([1000., 0., 0.]), np.array([8., 0., 0.]), np.array([0., 1., 0.])
        self.assertEqual(guidance.decide(r, v, np.zeros(3), 90., 45., up).mode, 'CLEAR_COAST')
        decision = guidance.decide(r, v, np.array([-.04, 0., 0.]), 91., 45., up)
        self.assertEqual(decision.mode, 'ACCELERATE')
        self.assertGreater(decision.acceleration[0], 0.)
        self.assertFalse(guidance.coast_latched)

    def test_cutoff_rechecks_trajectory_after_drift_and_after_turning_away(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0., coast_speed=8.))
        r, up = np.array([1000., 0., 0.]), np.array([0., 1., 0.])
        guidance.decide(r, np.array([8., 0., 0.]), np.zeros(3), 90., 45., up)
        drift = guidance.decide(r, np.array([7., 1., 0.]), np.zeros(3), 91., 45., up)
        self.assertEqual(drift.mode, 'CORRECT')
        away = guidance.decide(r, np.array([-8., 0., 0.]), np.zeros(3), 92., 45., up)
        self.assertEqual(away.mode, 'ACCELERATE')
        self.assertFalse(guidance.coast_latched)

    def test_cutoff_preserves_arrival_margin_hysteresis(self):
        guidance = AccelerationGuidance(MissionModel(coast_speed=8.))
        v, up = np.array([8., 0., 0.]), np.array([0., 1., 0.])
        first = guidance.decide(np.array([1000., 40., 0.]), v, np.zeros(3), 90., 45., up)
        noisy = guidance.decide(np.array([998., 44., 0.]), v, np.zeros(3), 91., 45., up)
        lost = guidance.decide(np.array([990., 48., 0.]), v, np.zeros(3), 92., 45., up)
        self.assertEqual(first.mode, 'CLEAR_COAST')
        self.assertEqual(noisy.mode, 'CLEAR_COAST')
        self.assertEqual(lost.mode, 'CORRECT')

    def test_recorded_no_debris_eight_meter_state_corrects_instead_of_full_burn(self):
        # Rounded own-ship/target state at t=90.425 s from the recorded test.
        r = np.array([-144.7849723, 29.6146114, -919.9500102])
        v = np.array([-1.12486778, -.103234663, -7.96050049])
        gravity = np.array([.000773756, -.0000991701, .009446874])
        self.assertEqual(ballistic_entry(r, v, gravity, 42., 600.), math.inf)
        guidance = AccelerationGuidance(MissionModel(coast_speed=8.))
        decision = guidance.decide(r, v, gravity, 90.425, 45.603, np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'CORRECT')
        self.assertLess(np.linalg.norm(decision.acceleration), .4 * guidance.model.planning_accel)
        self.assertAlmostEqual(float(decision.acceleration @ (r + guidance.offset)), 0., places=9)

    def test_ballistic_safe_final_approach_does_not_brake(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0.))
        decision = guidance.decide(np.array([70., 0., 0.]), np.array([8., 0., 0.]),
                                   np.zeros(3), 180., 40., np.array([0., 1., 0.]))
        self.assertGreaterEqual(decision.acceleration[0], 0.)
        self.assertEqual(decision.mode, 'FINAL_COAST')

    def test_moving_target_and_gravity_create_lateral_correction(self):
        guidance = AccelerationGuidance(MissionModel(flyby_offset=0.))
        decision = guidance.decide(np.array([500., 0., 0.]), np.array([5., -1., 0.]),
                                   np.array([0., -.003, 0.]), 100., 30., np.array([0., 1., 0.]))
        self.assertGreater(decision.acceleration[1], 0.)

    def test_success_requires_no_velocity_matching(self):
        decision = AccelerationGuidance(MissionModel()).decide(np.array([49., 0., 0.]), np.array([10., 2., 0.]),
                                                       np.zeros(3), 180., 30., np.array([0., 1., 0.]))
        self.assertEqual(decision.mode, 'ARRIVED')
        np.testing.assert_array_equal(decision.acceleration, np.zeros(3))


class PulseAndAttitudeTests(unittest.TestCase):
    def test_pointing_tolerance_widens_near_target_and_during_avoidance(self):
        self.assertAlmostEqual(math.degrees(pointing_tolerance(1300.)), 8.)
        self.assertAlmostEqual(math.degrees(pointing_tolerance(1300., True)), 12.)
        self.assertAlmostEqual(math.degrees(pointing_tolerance(100.)), 18.)

    def test_coarse_turn_can_use_calibrated_idle_torque_outside_quiet_coast(self):
        desired = np.array([0., 0., .005])
        command = angular_command(desired, np.zeros(3), False, 1. / 60., .008,
                                  align=True, idle_alignment=True)
        np.testing.assert_allclose(command, desired / 60.)

    def test_alignment_strength_is_smooth_bounded_and_increases_with_error(self):
        values = [alignment_strength(math.radians(angle)) for angle in (0., 5., 10., 30., 60., 180.)]
        self.assertEqual(values[:2], [0., 0.])
        self.assertEqual(values[-2:], [1., 1.])
        self.assertEqual(values, sorted(values))

    def test_large_nose_error_moves_the_reference_faster(self):
        baseline = AccelerationAttitude([1., 0., 0.])
        adaptive = AccelerationAttitude([1., 0., 0.])
        slow = baseline.update(np.array([0., .1, 0.]), 1.)
        fast = adaptive.update(np.array([0., .1, 0.]), 1., pointing_error=math.pi / 2.)
        self.assertAlmostEqual(math.degrees(math.acos(slow[0])), 2.)
        self.assertAlmostEqual(math.degrees(math.acos(fast[0])), 10.)

    def test_avoidance_disables_target_alignment_even_during_side_thrust(self):
        command = angular_command(np.array([.02, .02, .02]), np.zeros(3), True,
                                  1. / 60., .005, align=False)
        np.testing.assert_array_equal(command, np.zeros(3))
        omega = np.array([.01, -.02, .03])
        damping = angular_command(np.array([.02, .02, .02]), omega, True,
                                  1. / 60., .005, align=False)
        self.assertLess(damping @ omega, 0.)
        self.assertLessEqual(np.linalg.norm(damping), .005 + 1e-12)

    def test_feedback_pulses_regulate_an_allocator_that_amplifies_small_commands(self):
        pulses = PulseController(.04, .008, .25, .08)
        previous = np.zeros(3)
        delivered = np.zeros(3)
        coast_steps = 0
        for index in range(6000):
            measured = (np.array([.15, 0., 0.]) if np.linalg.norm(previous) else np.zeros(3)) * .02
            delivered += measured
            previous = pulses.command(np.array([.03, 0., 0.]), measured, .02, index * .02)
            coast_steps += int(not pulses.active)
        self.assertAlmostEqual(delivered[0] / 120., .03, delta=.001)
        self.assertGreater(coast_steps / 6000., .7)
        self.assertLess(pulses.switches, 600)

    def test_urgent_maneuver_does_not_wait_for_pulse_debt(self):
        pulses = PulseController(.04, .008, .25, .08)
        command = pulses.command(np.array([0., .05, 0.]), np.zeros(3), .02, 0., urgent=True)
        self.assertGreater(command[1], 0.)

    def test_zero_demand_stops_firing_even_during_minimum_pulse(self):
        pulses = PulseController(.04, .008, .25, .08)
        pulses.command(np.array([.05, 0., 0.]), np.zeros(3), .02, 0., urgent=True)
        command = pulses.command(np.zeros(3), np.zeros(3), .02, .02)
        np.testing.assert_array_equal(command, np.zeros(3))

    def test_attitude_reference_slews_and_holds_during_coast(self):
        attitude = AccelerationAttitude([1., 0., 0.])
        forward = attitude.update(np.array([0., .1, 0.]), 1.)
        self.assertAlmostEqual(math.degrees(math.acos(forward[0])), 2.)
        np.testing.assert_array_equal(attitude.update(np.zeros(3), 10.), forward)

    def test_opposite_thrust_direction_does_not_flip_attitude_reference(self):
        attitude = AccelerationAttitude([1., 0., 0.])
        forward = attitude.update(np.array([-.1, 0., 0.]), .1)
        self.assertLess(math.acos(forward[0]), math.radians(.21))
        self.assertAlmostEqual(np.linalg.norm(forward), 1.)

    def test_flight_recorder_writes_replayable_sensor_data(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = FlightRecorder(directory, MissionModel())
            observation = sample([100., 0., 0.], [0., 1., 0.], np.zeros(3), np.zeros(3), 1.)
            recorder.observations(1.2, [observation])
            recorder.record(dict(t=1.2, mode='COAST', x=0., y=0., z=0.))
            recorder.close()
            self.assertIn('measurement_t', (recorder.directory / 'radar.csv').read_text())
            self.assertIn('COAST', (recorder.directory / 'telemetry.csv').read_text())
            self.assertTrue((recorder.directory / 'model.json').exists())


if __name__ == '__main__':
    unittest.main()
