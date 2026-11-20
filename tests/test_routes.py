"""Strategic guidance geometry, forecast uncertainty and command handoffs."""
import unittest
import numpy as np

from Guidance import (MissionModel, AccelerationGuidance, AvoidancePlanner, DebrisTrack,
                      GuidanceDecision, PredictionScene)
from RoutePlanner import Maneuver, RadarField, Risk, RoutePlanner, forecast


class RouteTests(unittest.TestCase):
    def test_covariance_shortlist_preserves_clearance_and_unresolved_tracks(self):
        from pathlib import Path
        from compare_routes import load_module
        source = Path(__file__).resolve().parents[1] / 'comparison/pre_score_optimization/RoutePlanner.py'
        reference = load_module(source, 'before_score_routes_check')
        rng = np.random.default_rng(37)
        tracks = [self.encounter(3*(1+i%32), rng.normal(0., 25., 3), i) for i in range(75)]
        for t in tracks[::6]:
            t.covariance *= 100.
        field = RadarField(tracks, self.start, self.zero, self.zero, self.g, self.model)
        for duration in (24., self.direct.eta):
            path = forecast(self.guide, self.r, self.v, self.g, self.ref, self.start, 4., None)
            path.eta = duration
            path.positions = path.positions[:int(duration//3)+2]
            expected = reference.RadarField.assess(field, path)
            actual = field.assess(path)
            for key in ('threat_ids', 'unresolved', 'track', 'approaching_at_horizon'):
                self.assertEqual(getattr(actual, key), getattr(expected, key))
            np.testing.assert_allclose([actual.clearance, actual.miss, actual.sigma, actual.t],
                                       [expected.clearance, expected.miss, expected.sigma, expected.t])

    def setUp(self):
        self.model = MissionModel(planning_accel=.1, cruise_burns=True, coast_speed=10.)
        self.guide = AccelerationGuidance(self.model)
        self.router = RoutePlanner()
        self.zero = np.zeros(3)
        self.goal = np.array([1., 0., 0.])
        self.ref = np.array([0., 1., 0.])
        self.r = self.goal * 1300.
        self.v = self.goal * .8
        self.g = -.006 * self.goal
        self.start = 10.
        self.nominal = self.guide.decide(self.r, self.v, self.g, self.start, 4., self.ref)
        self.direct = forecast(self.guide, self.r, self.v, self.g, self.ref, self.start, 4., None)

    def encounter(self, seconds, offset, number=1):
        velocity = np.array([.3, .5, -.4])
        centre = self.direct.positions[seconds // 3] - velocity*seconds - .5*self.g*seconds**2 + offset
        return DebrisTrack(number, self.start, np.r_[centre, velocity],
                           np.diag([1.]*3 + [.0002]*3), 2., hits=7)

    def update(self, tracks, elapsed=0.):
        # Advance own measured state while the incremental search is running.
        a = self.nominal.acceleration + self.g
        origin = self.v * elapsed + .5 * a * elapsed**2
        return self.router.command(self.nominal, self.guide, tracks, self.start + elapsed,
                                   origin, self.r-origin, self.v+a*elapsed, self.zero, self.g,
                                   self.ref, self.goal, 4.)

    def test_one_early_leg_clears_two_distant_moving_tracks(self):
        self.start = 16.
        tracks = [self.encounter(93, self.ref*18), self.encounter(99, self.ref*19, 2)]
        self.update(tracks)
        self.assertEqual(self.router.direct_risk.threats, 2)
        self.assertGreater(self.router.direct_risk.t, 85.)
        for t in np.arange(.25, 4., .25):
            self.assertIs(self.update(tracks, t), self.nominal)
            self.assertIsNone(self.router.action)
        self.assertEqual(self.router.pending.name, 'steer')
        np.testing.assert_array_equal(self.guide.offset, self.zero)
        self.update(tracks, 4.)
        self.assertIsNotNone(self.router.action)
        self.assertEqual(self.router.action.name, 'steer')
        self.assertGreaterEqual(self.router.risk.clearance, 0.)
        self.assertTrue(np.isfinite(self.router.path.score))
        self.assertEqual(self.router.action.start, 20.)
        # At the first encounter, even changing from full thrust to full thrust
        # in the opposite direction for the last 10 s cannot restore this margin.
        field = RadarField(tracks, self.start, self.zero, self.zero, self.g, self.model)
        sample = 93 // 3
        direct_miss = np.linalg.norm(field.xyz[0, sample] - self.direct.positions[sample])
        required = field.radius[0] + self.model.uncertainty_sigma * field.sigma[0, sample]
        max_late_displacement = .5 * (2. * self.model.planning_accel) * 10.**2
        self.assertLess(direct_miss + max_late_displacement, required)
        first = self.router.action
        self.update(tracks, 4.25)
        self.assertIs(self.router.action, first)

    def test_unreachable_deferred_turn_is_not_accepted(self):
        tracks = [self.encounter(93, self.ref*8), self.encounter(99, self.ref*9, 2)]
        for t in np.arange(0., 2., .25):
            self.assertIs(self.update(tracks, t), self.nominal)
        self.assertLess(self.router.direct_risk.clearance, 0.)
        self.assertIsNone(self.router.pending)

    def test_pending_turn_is_cancelled_on_new_radar_at_launch_handoff(self):
        self.start = 16.
        tracks = [self.encounter(93, self.ref*18), self.encounter(99, self.ref*19, 2)]
        for t in np.arange(0., 4., .25):
            self.update(tracks, t)
        self.assertIsNotNone(self.router.pending)
        self.assertIs(self.update([], 4.), self.nominal)
        self.assertIsNone(self.router.pending)
        np.testing.assert_array_equal(self.guide.offset, self.zero)

    def test_a_safe_near_pass_costs_no_extra_maneuver(self):
        command = self.update([self.encounter(99, self.ref*90)])
        self.assertIs(command, self.nominal)
        self.assertIsNone(self.router.search)
        self.assertEqual(self.router.direct_risk.threats, 0)

    def test_newly_observed_threat_triggers_review_after_direct_flight(self):
        safe = self.encounter(90, self.ref*150)
        self.update([safe])
        self.update([safe, self.encounter(99, self.ref*8, 2)], 2.1)
        self.assertIsNotNone(self.router.search)
        self.router.interrupt(12.2, 'tactical override')
        self.assertIsNone(self.router.search)
        self.assertIsNone(self.router.action)

    def test_worker_result_is_rechecked_if_rock_has_disappeared(self):
        tracks = [self.encounter(93, self.ref*8)]
        self.update(tracks)
        self.assertIsNotNone(self.router.search)
        for t in np.arange(.25, 2., .25):
            self.update([], t)
        self.assertIsNone(self.router.action)

    def test_single_hit_stale_and_ambiguous_tracks_do_not_steer_far_ahead(self):
        tracks = [self.encounter(99, self.zero, i) for i in range(3)]
        tracks[0].hits = 1
        tracks[1].time -= 15.
        tracks[2].reliable_hits = 0
        self.update(tracks)
        self.assertEqual(self.router.track_count, 0)
        self.assertIsNone(self.router.action)

    def test_uncertain_transverse_velocity_shortens_useful_horizon(self):
        rock = self.encounter(99, self.zero)
        def assess():
            return RadarField([rock], self.start, self.zero, self.zero, self.g, self.model).assess(self.direct)
        self.assertLess(assess().clearance, 0.)
        rock.covariance[4, 4] = 4.
        self.assertEqual(assess().threats, 0)

    def test_clearance_uses_uncertainty_in_the_direction_of_separation(self):
        self.model.process_accel_sigma = 0.
        track = self.encounter(99, self.ref * 20.)
        # The asteroid follows a parallel z=0 track. Uncertainty normal to the
        # encounter plane cannot reduce its measured in-plane separation.
        track.state[2] = track.state[5] = 0.
        track.covariance = np.diag([.04, .04, 64., .0001, .0001, .0001])
        field = RadarField([track], self.start, self.zero, self.zero, self.g, self.model)
        risk = field.assess(self.direct)
        self.assertLess(risk.sigma, 2.)
        self.assertGreater(risk.clearance, 0.)

    def test_moving_a_conflict_into_an_uncertain_window_cannot_qualify_a_detour(self):
        direct = Risk(clearance=-2., threat_ids=frozenset([7]))
        self.assertFalse(Risk(clearance=3., unresolved=frozenset([7])).clears(direct))
        self.assertTrue(Risk(clearance=3.).clears(direct))

    def test_still_approaching_a_known_conflict_at_forecast_end_is_unresolved(self):
        slow = forecast(self.guide, self.r, self.v, self.g, self.ref, self.start, 4., None)
        track = self.encounter(120, self.goal * 100., number=4)
        risk = RadarField([track], self.start, self.zero, self.zero, self.g, self.model).assess(slow)
        self.assertGreater(risk.clearance, 0.)
        self.assertIn(4, risk.approaching_at_horizon)
        self.assertFalse(risk.clears(Risk(clearance=-1., threat_ids=frozenset([4]))))

    def test_late_speed_review_requires_a_clear_observed_alternative(self):
        self.model.coast_speed = 9.
        self.model.process_accel_sigma = .001
        self.router.cruise_reviewed = True
        r, v = self.goal * 800., self.goal * 8.4
        direct = forecast(self.guide, r, v, self.zero, self.ref, self.start, 4., None)
        t = 60
        crossing = self.ref * 4.
        state = np.r_[direct.positions[t//3] - crossing*t, crossing]
        track = DebrisTrack(7, self.start, state, np.diag([.04]*3 + [.0001]*3), 2., hits=12)
        nominal = self.guide.decide(r, v, self.zero, self.start, 4., self.ref)
        self.router.command(nominal, self.guide, [track], self.start, self.zero, r, v, self.zero,
                            self.zero, self.ref, self.goal, 4.)
        self.assertEqual(self.guide.model.coast_speed, 10.)
        self.assertGreater(self.router.risk.clearance, 0.)
        self.assertIsNone(self.router.action)

    def test_moving_reference_frame_does_not_change_predicted_encounter(self):
        rock = self.encounter(99, self.zero)
        rock.time -= 2.
        def assess(target_v):
            return RadarField([rock], self.start, self.zero, target_v, self.g, self.model).assess(self.direct)
        before = assess(self.zero)
        boost = np.array([0., 7., 0.])
        rock.state[:3] -= 2.*boost
        rock.state[3:] += boost
        self.assertAlmostEqual(assess(boost).clearance, before.clearance)

    def test_forecast_fuel_and_time_for_constant_velocity_arrival(self):
        v = self.goal * 10.
        path = forecast(self.guide, self.r, v, self.zero, self.ref, self.start, 40., None)
        self.assertAlmostEqual(path.eta, 125.)
        self.assertAlmostEqual(path.fuel, 40.)
        self.assertEqual(path.burns, 0)
        self.assertAlmostEqual(path.score, self.model.score(135., 40.))
        self.assertFalse(self.guide.coast_latched)

    def test_final_approach_does_not_accept_an_extra_burn(self):
        advance = Maneuver('advance', 10., 6., self.ref)
        final = GuidanceDecision(self.zero, 'FINAL_COAST', 5.)
        self.assertIs(advance.apply(final, 11., self.goal, self.model), final)

    def test_crossing_after_distance_cutoff_is_reported_without_a_burn(self):
        velocity = self.goal * 10.
        final = self.guide.decide(self.goal * 100., velocity, self.zero, self.start, 4., self.ref)
        self.assertEqual(final.mode, 'DISTANCE_COAST')
        rock = DebrisTrack(1, self.start, np.array([40., 8., 0., 0., -2., 0.]),
                           np.eye(6) * .001, 2., hits=5)
        scene = PredictionScene([rock], self.start, self.zero, velocity, self.zero,
                                self.model, horizon=final.eta + 1.)
        plan = AvoidancePlanner(self.model).choose(scene, final.acceleration, velocity,
                                                  self.goal, self.start, nose=self.goal, coast_only=True)
        self.assertLess(plan.baseline.time, final.eta)
        self.assertEqual(plan.strategy, 'DISTANCE_COAST')
        self.assertLess(plan.encounter.clearance, 0.)
        np.testing.assert_array_equal(plan.acceleration, self.zero)
