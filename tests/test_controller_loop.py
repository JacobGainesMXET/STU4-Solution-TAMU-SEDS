"""Exercise the real controller loop against a small fake API, without connecting.

This verifies API wiring and failure paths. It is not a dynamics/Space Teams test.
"""
import datetime
from contextlib import nullcontext
import os
import sys
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import numpy.typing as npt

from FlightRecorder import FlightRecorder
from Guidance import (AvoidancePlanner, AvoidanceDecision, Encounter, AccelerationGuidance,
                      GuidanceDecision, PredictionScene)
from RoutePlanner import Maneuver, RoutePlanner


class StopLoop(Exception):
    pass


class FakeSimulation:
    phobos_mu_km3_s2 = 0.

    def __init__(self, failure=None, arrived=False, distance=1300., velocity=None, rotation=None,
                 radar_period=4., no_debris_test=False, spin=None, stop_at=14.):
        self.now = 0.
        self.stop_at = stop_at
        self.position = np.array([0., 0., -10000.])
        self.velocity = np.zeros(3) if velocity is None else np.array(velocity, dtype=float)
        self.distance = distance
        self.rotation = np.eye(3) if rotation is None else np.array(rotation, dtype=float)
        self.spin = np.zeros(3) if spin is None else np.array(spin, dtype=float)
        self.radar_period = radar_period
        self.no_debris_test = no_debris_test
        self.radar_reads = 0
        self.command = np.zeros(3)
        self.angular = np.zeros(3)
        self.failure = failure
        self.arrived = arrived
        self.commands = []
        self.messages = []
        self.zero_time = datetime.datetime(2026, 1, 1)

    def sleep(self, dt):
        if self.now > self.stop_at:
            raise StopLoop()
        thrust = self.rotation @ (self.command / np.linalg.norm(self.command) * .14) if np.linalg.norm(self.command) else np.zeros(3)
        self.position += self.velocity * dt + .5 * thrust * dt**2
        self.velocity += thrust * dt
        self.now += dt

    def timestamp(self):
        return SimpleNamespace(as_datetime=lambda: self.zero_time + datetime.timedelta(seconds=self.now))

    def get_param(self, kind, key):
        if key == 'NoDebrisTestMode':
            return self.no_debris_test
        if key == ['Time', 'StartDateTAI']:
            return SimpleNamespace(as_datetime=lambda: self.zero_time)
        if key == 'LoopFreqHz':
            return 20.
        if key == ['Thrusters', 'MaxAccel_m_s2']:
            return .5
        if key == ['Thrusters', 'MaxAngAccel_rad_s2']:
            return .05
        return 0.

    def set_param(self, kind, key, value):
        if key[-1] == 'CommandedAccel':
            self.command = np.array(value)
        else:
            self.angular = np.array(value)
            self.commands.append((self.now, self.command.copy(), self.angular.copy()))

    def state(self):
        return SimpleNamespace(loc_WRT_ExprIn=lambda frame: self.position.copy(),
                               vel_WRT_ExprIn=lambda frame: self.velocity.copy())

    def target_state(self):
        offset = 49. if self.arrived else self.distance
        return SimpleNamespace(loc_WRT_ExprIn=lambda frame: np.array([offset, 0., -10000.]),
                               vel_WRT_ExprIn=lambda frame: np.zeros(3))

    def radar(self, count):
        self.radar_reads += 1
        if self.failure == 'radar':
            raise RuntimeError('synthetic disconnected radar')
        if self.failure == 'empty':
            return []
        stamp = max(0., self.radar_period * int(self.now // self.radar_period))
        relative = np.array([800., 500., 0.])  # far, synthetic observation
        matrix = np.array([relative, [np.linalg.norm(relative) - 2., 0., 4.], [stamp, 0., 0.]])
        from test_guidance import Sample
        return [Sample(matrix[0], matrix[1, 0], matrix[1, 1], matrix[1, 2], stamp)]

    def run(self, recording_failure=False, avoidance_decision=None, source_path=None):
        source_path = source_path or Path(__file__).resolve().parents[1] / 'Controller.py'
        source = source_path.read_text(encoding='utf-8-sig')
        source = source[source.index('import math', source.index('# QUALIFICATION CONTROLLER:')):]
        def vector(value, frame):
            def express(other):
                if frame == other:
                    return np.asarray(value)
                return (self.rotation if frame == 1 else self.rotation.T) @ value
            return SimpleNamespace(ExprIn=express)
        shuttle = SimpleNamespace(GetParam=self.get_param, SetParam=self.set_param, getMass=lambda: 3388.7,
                                  getLocVelAcc=self.state,
                                  getAngVelocity=lambda: SimpleNamespace(WRT=lambda frame: SimpleNamespace(ExprIn=lambda other: self.spin.copy())))
        st = SimpleNamespace(VarType=SimpleNamespace(double=0, timestamp=1, bool=2),
                             frames=SimpleNamespace(FramedVector=vector),
                             SimGlobals=SimpleNamespace(GetSimEntity=lambda: SimpleNamespace(GetParam=self.get_param),
                                                        SimClock=SimpleNamespace(GetTimeNow=self.timestamp)),
                             OnScreenLogMessage=lambda message, *args: self.messages.append(message),
                             Severity=SimpleNamespace(Info=0, Warning=1), leave_sim=lambda: None)
        with tempfile.TemporaryDirectory() as directory:
            namespace = dict(np=np, npt=npt, os=os, datetime=datetime, st=st,
                             time=SimpleNamespace(sleep=self.sleep, perf_counter=lambda: self.now),
                             this=SimpleNamespace(GetParam=self.get_param,
                                                  HasParam=lambda key: key == 'NoDebrisTestMode' and self.no_debris_test),
                             shuttle=shuttle, phobos=SimpleNamespace(GetParam=lambda *args: self.phobos_mu_km3_s2),
                             target=SimpleNamespace(getLocVelAcc=self.target_state),
                             phobosFrame=0, shuttleFrame=1, GetRadarContacts=self.radar,
                             SetCommand=lambda linear, angular: (self.set_param(0, ['CommandedAccel'], linear),
                                                                 self.set_param(0, ['CommandedAngAccel'], angular)),
                             sys=sys, __file__=str(Path(directory) / 'Controller.py'))
            mock = patch.object(FlightRecorder, 'observations', side_effect=OSError('synthetic full disk')) if recording_failure else patch.object(FlightRecorder, 'observations', FlightRecorder.observations)
            avoidance_mock = (patch.object(AvoidancePlanner, 'choose', return_value=avoidance_decision)
                              if avoidance_decision is not None else nullcontext())
            with mock, avoidance_mock, patch.object(sys, 'path', list(sys.path)):
                try:
                    exec(compile(source, str(source_path), 'exec'), namespace)
                except StopLoop:
                    pass
                finally:
                    if namespace.get('recorder') is not None:
                        namespace['recorder'].close()
            return namespace


class ControllerLoopTests(unittest.TestCase):
    def test_confirmed_vector_escape_is_allocated_in_world_direction(self):
        sim = FakeSimulation(velocity=[5., 0., 0.], radar_period=.5, stop_at=51.)
        sim.now = 50.
        demand = np.array([.07, .05, 0.])
        escape = AvoidanceDecision(demand, 'AVOID', Encounter(clearance=3.), Encounter(hull_gap=-2.),
                                   forward_burn=True, burn_seconds=2., vector_escape=True)
        sim.run(avoidance_decision=escape)
        self.assertTrue(any(np.allclose(linear, demand) for _, linear, _ in sim.commands))

    def test_lift_allocation_changes_only_after_liftoff_on_a_clear_corridor(self):
        for gap, expected in [(3., .5), (20., .12)]:
            simulation = FakeSimulation(velocity=[1., 0., 0.], no_debris_test=True, stop_at=1.)
            clear = Encounter(clearance=gap)
            plan = AvoidanceDecision(np.array([.12, 0., 0.]), 'GUIDANCE', clear, clear)
            simulation.run(avoidance_decision=plan)
            self.assertAlmostEqual(np.linalg.norm(simulation.commands[-1][1]), expected)

    def test_failed_strategic_forecast_keeps_immediate_collision_checks_running(self):
        simulation = FakeSimulation(radar_period=.5)
        with patch.object(RoutePlanner, 'command', side_effect=ArithmeticError('synthetic route fault')):
            result = simulation.run()
        self.assertTrue(any('Route forecast unavailable' in m for m in simulation.messages))
        self.assertEqual(result['mode'], 'ACCELERATE')
        self.assertEqual(result['plan'].mode, 'GUIDANCE')
        self.assertIsNone(result['routes'].action)

    def test_strategic_steering_uses_rear_thrust_while_nose_turns(self):
        def steer(router, nominal, guidance, tracks, now, *args, **kwargs):
            router.action = Maneuver('steer', now, 30., np.array([0., 1., 0.]), 12.)
            return GuidanceDecision(np.array([.085, .018, 0.]), 'ACCELERATE', 150.)
        simulation = FakeSimulation(no_debris_test=True)
        with patch.object(RoutePlanner, 'command', steer):
            result = simulation.run()
        self.assertTrue(result['route_rear'])
        self.assertTrue(any(np.linalg.norm(angular) > 0. for _, _, angular in simulation.commands))
        for _, linear, _ in simulation.commands:
            self.assertGreater(linear[0], 0.)
            np.testing.assert_allclose(linear[1:], 0., atol=1e-12)

    def test_trim_stops_on_observed_impulse_between_guidance_updates(self):
        def scheduled_once(guidance, r, v, g, now, fuel, reference, *, nose=None):
            if guidance.burn_count == 0:
                acceleration = np.array([.09, 0., 0.])
                guidance._start_burn(acceleration, .075, 'TRIM_BURN', now, v, g)
                return GuidanceDecision(acceleration, 'TRIM_BURN', 100., burn_seconds=.075)
            return GuidanceDecision(np.zeros(3), 'CLEAR_COAST', 100., coast_entry=100.)
        simulation = FakeSimulation(velocity=[10., 0., 0.], no_debris_test=True)
        with patch.object(AccelerationGuidance, 'decide', scheduled_once):
            simulation.run()
        firing_times = [t for t, linear, _ in simulation.commands if np.linalg.norm(linear) > 0.]
        self.assertTrue(firing_times)
        self.assertLess(max(firing_times), .15)  # next guidance plan is not until .30 s
        self.assertEqual(simulation.commands[1][1][0], 0.)

    def test_forward_avoidance_cancels_a_scheduled_course_burn(self):
        hazard = AvoidanceDecision(np.array([.09, 0., 0.]), 'AVOID',
                                   Encounter(clearance=1., time=5.), Encounter(clearance=-5., time=5.),
                                   forward_burn=True, burn_seconds=2., strategy='FORWARD')
        simulation = FakeSimulation(velocity=[10., 1., 0.], no_debris_test=True)
        result = simulation.run(avoidance_decision=hazard)
        self.assertEqual(result['mode'], 'AVOID')
        self.assertIsNone(result['guidance'].burn)
        self.assertTrue(any(linear[0] > 0. for _, linear, _ in simulation.commands))
        self.assertTrue(all(linear[0] >= 0. and np.linalg.norm(linear[1:]) < 1e-12
                            for _, linear, _ in simulation.commands))

    def test_forward_avoidance_steers_nose_and_commands_no_sideways_translation(self):
        angle = np.deg2rad(12.)
        maneuver = AvoidanceDecision(.12 * np.array([np.cos(angle), np.sin(angle), 0.]), 'AVOID',
                                     Encounter(clearance=1.), Encounter(clearance=-1., time=15.),
                                     forward_burn=True, burn_seconds=2., strategy='FORWARD')
        simulation = FakeSimulation(no_debris_test=True)
        result = simulation.run(avoidance_decision=maneuver)
        self.assertTrue(result['forward_avoidance'])
        self.assertTrue(result['full_thrust'])
        self.assertTrue(any(np.linalg.norm(angular) > 0. for _, _, angular in simulation.commands))
        for _, linear, _ in simulation.commands:
            self.assertGreaterEqual(linear[0], 0.)
            np.testing.assert_allclose(linear[1:], np.zeros(2), atol=1e-12)

    def test_expired_forward_burn_does_not_reappear_as_a_sideways_pulse(self):
        maneuver = AvoidanceDecision(np.array([.12, .01, 0.]), 'AVOID',
                                     Encounter(clearance=1.), Encounter(clearance=-1., time=15.),
                                     forward_burn=True, burn_seconds=.01, strategy='FORWARD')
        simulation = FakeSimulation(no_debris_test=True)
        simulation.run(avoidance_decision=maneuver)
        self.assertTrue(any(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))
        self.assertTrue(any(linear[0] > 0. for _, linear, _ in simulation.commands))
        self.assertTrue(all(np.linalg.norm(linear[1:]) == 0. for _, linear, _ in simulation.commands))

    def test_high_spin_cancels_forward_avoidance_until_stable(self):
        maneuver = AvoidanceDecision(np.array([.09, .01, 0.]), 'AVOID',
                                     Encounter(clearance=1.), Encounter(clearance=-1., time=15.),
                                     forward_burn=True, burn_seconds=2., strategy='FORWARD')
        simulation = FakeSimulation(no_debris_test=True, spin=[0., 0., .08])
        result = simulation.run(avoidance_decision=maneuver)
        self.assertFalse(result['forward_avoidance'])
        self.assertTrue(all(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))
        self.assertTrue(any(np.linalg.norm(angular) > 0. for _, _, angular in simulation.commands))

    def test_ten_meter_distant_coast_shuts_off_linear_and_angular_thrusters(self):
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        simulation = FakeSimulation(distance=1000., velocity=[10., 0., 0.], rotation=rotation,
                                    no_debris_test=True)
        simulation.now, simulation.stop_at = 130., 144.
        result = simulation.run()
        self.assertTrue(result['model'].score_cruise)
        self.assertIsNone(result['model'].coast_speed)
        self.assertEqual(result['mode'], 'CLEAR_COAST')
        self.assertTrue(result['quiet_coast'])
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0.
                            for _, linear, angular in simulation.commands))

    def test_no_debris_test_mode_flies_without_reading_disabled_radar(self):
        simulation = FakeSimulation(failure='radar', no_debris_test=True)
        result = simulation.run()
        self.assertEqual(result['mode'], 'ACCELERATE')
        self.assertEqual(simulation.radar_reads, 0)
        self.assertGreater(simulation.position[0], 10.)
        self.assertTrue(any(np.linalg.norm(linear) > .49 for _, linear, _ in simulation.commands))

    def test_ten_meter_course_miss_turns_and_trims_with_rear_thrust(self):
        simulation = FakeSimulation(distance=1000., velocity=[10., 1., 0.], no_debris_test=True)
        simulation.now, simulation.stop_at = 130., 144.
        result = simulation.run()
        self.assertIn(result['mode'], ('TRIM_BURN', 'TRIM_WAIT', 'CLEAR_COAST'))
        # Fixed attitude in this fake API verifies wiring, independent of turn dynamics.
        self.assertTrue(any(angular[2] < 0. for _, _, angular in simulation.commands))
        self.assertTrue(all(linear[0] >= 0. and np.linalg.norm(linear[1:]) < 1e-12
                            for _, linear, _ in simulation.commands))
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))
        self.assertTrue(all(np.linalg.norm(linear) < .49 for _, linear, _ in simulation.commands))

    def test_no_debris_test_mode_still_allows_unpowered_arrival_path(self):
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], failure='radar', no_debris_test=True)
        simulation.now, simulation.stop_at = 130., 144.
        result = simulation.run()
        self.assertTrue(result['quiet_coast'])
        self.assertEqual(result['mode'], 'CLEAR_COAST')
        self.assertEqual(simulation.radar_reads, 0)
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0.
                            for _, linear, angular in simulation.commands))

    def test_empty_radar_in_normal_competition_still_inhibits_blind_acceleration(self):
        simulation = FakeSimulation(failure='empty', velocity=[3., 1., 0.])
        result = simulation.run()
        self.assertEqual(result['mode'], 'SENSOR_COAST')
        self.assertGreater(simulation.radar_reads, 0)
        self.assertTrue(all(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))




    def test_safe_coast_turns_all_thrusters_off_even_with_a_misaligned_nose(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], rotation=rotation, radar_period=.5)
        simulation.now, simulation.stop_at = 130., 144.
        result = simulation.run()
        self.assertTrue(result['quiet_coast'])
        self.assertEqual(result['mode'], 'CLEAR_COAST')
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0.
                            for _, linear, angular in simulation.commands))

    def test_final_safe_approach_stays_unpowered_through_arrival(self):
        simulation = FakeSimulation(distance=100., velocity=[10., 0., 0.], radar_period=.5)
        result = simulation.run()
        self.assertTrue(result['mission_complete'])
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0.
                            for _, linear, angular in simulation.commands))

    def test_threat_overrides_an_otherwise_valid_coast(self):
        hazard = AvoidanceDecision(np.array([.09, .01, 0.]), 'AVOID',
                                   Encounter(clearance=1., time=5.), Encounter(clearance=-5., time=5.),
                                   forward_burn=True, burn_seconds=2., strategy='FORWARD')
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.])
        result = simulation.run(avoidance_decision=hazard)
        self.assertFalse(result['quiet_coast'])
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))

    def test_stale_radar_cancels_quiet_coast(self):
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], radar_period=4.)
        simulation.run()
        self.assertTrue(any('SENSOR_COAST' in message for message in simulation.messages))

    def test_broadside_launch_builds_speed_along_the_departure_line_with_bounded_turning(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        sim = FakeSimulation(rotation=rotation, no_debris_test=True)
        result = sim.run()
        self.assertTrue(result['launch_burn'])
        self.assertTrue(any(np.linalg.norm(a) > 0. for _, _, a in sim.commands))
        self.assertTrue(result['clear_departure'])
        self.assertTrue(all(np.linalg.norm(a) <= .05 + 1e-12 for _, _, a in sim.commands))
        self.assertLessEqual(np.linalg.norm(sim.commands[0][2]), .03 + 1e-12)
        self.assertGreater(sim.velocity[0], 0.)
        lift = np.array([rotation @ linear for _, linear, _ in sim.commands])
        np.testing.assert_allclose(lift[:, 1:], 0., atol=1e-12)
        self.assertEqual(lift[0, 0], .5)
        self.assertEqual(lift[-1, 0], .12)
        self.assertTrue(np.all(np.isin(lift[:, 0], [.5, .12])))

    def test_benign_radar_uses_same_launch_and_handoff_as_empty_field(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        empty = FakeSimulation(rotation=rotation, no_debris_test=True, stop_at=24.)
        clear = FakeSimulation(rotation=rotation, radar_period=.5, stop_at=24.)
        empty.run()
        clear.run()
        self.assertGreater(clear.radar_reads, 0)
        np.testing.assert_allclose([np.r_[t, a, b] for t, a, b in clear.commands],
                                   [np.r_[t, a, b] for t, a, b in empty.commands], atol=1e-12)

    def test_handoff_can_review_the_route_while_spin_still_inhibits_rear_thrust(self):
        sim = FakeSimulation(velocity=[2., 0., 0.], spin=[0., .13, 0.],
                             no_debris_test=True, stop_at=20.5)
        sim.now = 20.
        result = sim.run()
        self.assertIsNotNone(result['routes'].review_time)
        self.assertFalse(result['forward_ready'])
        self.assertTrue(sim.commands)
        self.assertTrue(all(np.linalg.norm(a) == 0. for _, a, _ in sim.commands))

    def test_nonurgent_avoidance_can_continue_coarse_alignment(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        moderate = AvoidanceDecision(np.zeros(3), 'AVOID', Encounter(clearance=1.),
                                     Encounter(clearance=-1., time=15.))
        simulation = FakeSimulation(rotation=rotation)
        simulation.run(avoidance_decision=moderate)
        self.assertTrue(any(np.linalg.norm(angular) > 0. for _, _, angular in simulation.commands))

    def test_new_launch_hazard_damps_a_fast_turn_before_resuming_aim(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        sim = FakeSimulation(rotation=rotation, radar_period=.5, spin=[0., 0., .16])
        hazard = AvoidanceDecision(np.zeros(3), 'AVOID', Encounter(clearance=1.),
                                   Encounter(clearance=-2., time=3.))
        result = sim.run(avoidance_decision=hazard)
        self.assertTrue(result['arrest_launch_turn'])
        for _, translation, angular in sim.commands:
            np.testing.assert_array_equal(translation, np.zeros(3))
            np.testing.assert_array_equal(angular[:2], np.zeros(2))
            self.assertLess(angular @ sim.spin, 0.)

    def test_launch_uses_a_clear_angled_vector_even_from_a_broadside_pad(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        simulation = FakeSimulation(radar_period=.5, rotation=rotation)
        def assess(scene, accel, seconds=8., initial_direction=None):
            return Encounter(clearance=2. if accel[1] > .015 else -5., time=3.)
        with patch.object(PredictionScene, 'assess', assess):
            result = simulation.run()
        self.assertEqual(result['plan'].strategy, 'LAUNCH_TILT')
        self.assertFalse(result['clear_departure'])
        self.assertTrue(all(np.linalg.norm(a) <= .03 + 1e-12 for _, _, a in simulation.commands))
        self.assertLess(result['plan'].baseline.clearance, 0.)
        self.assertGreater(result['plan'].encounter.clearance, 0.)
        powered = [rotation @ linear for t, linear, _ in simulation.commands if np.linalg.norm(linear)]
        self.assertTrue(powered)
        for world in powered:
            self.assertGreater(world[1], 0.)
            self.assertGreater(world[0], .46)
            self.assertAlmostEqual(np.linalg.norm(world), .5)

    def test_detected_hazard_can_still_command_an_escape_after_seventy_percent(self):
        simulation = FakeSimulation(distance=500., velocity=[10., 1., 0.],
                                    radar_period=.5, stop_at=37.)
        hazard = AvoidanceDecision(np.array([.09, 0., 0.]), 'AVOID',
                                   Encounter(clearance=1.), Encounter(clearance=-5., time=5.),
                                   forward_burn=True, burn_seconds=2., strategy='FORWARD')
        with patch.object(AvoidancePlanner, 'choose', return_value=hazard) as chooser:
            result = simulation.run()
        self.assertEqual(result['mode'], 'AVOID')
        self.assertFalse(chooser.call_args.kwargs['coast_only'])
        self.assertTrue(any(linear[0] > 0. for t, linear, _ in simulation.commands if 21. < t < 28.))
        final_commands = [np.r_[linear, angular] for t, linear, angular in simulation.commands if t >= 36.]
        self.assertTrue(final_commands)
        self.assertTrue(any(linear[0] > 0. for linear in final_commands))

    def test_acquisition_then_guidance_sends_finite_bounded_commands(self):
        simulation = FakeSimulation()
        result = simulation.run()
        self.assertTrue(result['tracker'].ready(simulation.now))
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))
        # Launch no longer waits for 60% of contacts to have a second observation.
        first_burn = next(t for t, linear, _ in simulation.commands if np.linalg.norm(linear) > 0.)
        self.assertLess(first_burn, 1.)
        self.assertTrue(any(abs(np.linalg.norm(linear) - .5) < 1e-9 for _, linear, _ in simulation.commands))
        for _, linear, angular in simulation.commands:
            self.assertTrue(np.all(np.isfinite(np.r_[linear, angular])))
            self.assertLessEqual(np.linalg.norm(linear), .5 + 1e-12)
            self.assertLessEqual(np.linalg.norm(angular), .05 + 1e-12)

    def test_radar_failure_clears_forward_guidance(self):
        simulation = FakeSimulation(failure='radar')
        result = simulation.run()
        self.assertEqual(result['mode'], 'SENSOR_COAST')
        self.assertTrue(all(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))

    def test_recording_failure_does_not_disable_guidance(self):
        simulation = FakeSimulation()
        result = simulation.run(recording_failure=True)
        self.assertIsNone(result['recorder'])
        self.assertNotEqual(result['mode'], 'SENSOR_COAST')
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))

    def test_entering_success_area_latches_zero_thrust(self):
        simulation = FakeSimulation(arrived=True)
        result = simulation.run()
        self.assertTrue(result['mission_complete'])
        self.assertEqual(result['mode'], 'ARRIVED')
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0. for _, linear, angular in simulation.commands))


if __name__ == '__main__':
    unittest.main()
