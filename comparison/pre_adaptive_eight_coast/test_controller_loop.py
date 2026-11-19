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
from Guidance import AvoidancePlanner, AvoidanceDecision, Encounter


class StopLoop(Exception):
    pass


class FakeSimulation:
    def __init__(self, failure=None, arrived=False, distance=1300., velocity=None, rotation=None, radar_period=4., no_debris_test=False):
        self.now = 0.
        self.position = np.array([0., 0., -10000.])
        self.velocity = np.zeros(3) if velocity is None else np.array(velocity, dtype=float)
        self.distance = distance
        self.rotation = np.eye(3) if rotation is None else np.array(rotation, dtype=float)
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
        if self.now > 14.:
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

    def run(self, recording_failure=False, avoidance_decision=None):
        source_path = Path(__file__).resolve().parents[1] / 'Controller.py'
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
                                  getAngVelocity=lambda: SimpleNamespace(WRT=lambda frame: SimpleNamespace(ExprIn=lambda other: np.zeros(3))))
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
                             shuttle=shuttle, phobos=SimpleNamespace(GetParam=lambda *args: 0.),
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
    def test_no_debris_test_mode_flies_without_reading_disabled_radar(self):
        simulation = FakeSimulation(failure='radar', no_debris_test=True)
        result = simulation.run()
        self.assertEqual(result['mode'], 'ACCELERATE')
        self.assertEqual(simulation.radar_reads, 0)
        self.assertGreater(simulation.position[0], 10.)
        self.assertTrue(any(np.linalg.norm(linear) > .49 for _, linear, _ in simulation.commands))

    def test_no_debris_test_mode_still_allows_unpowered_arrival_path(self):
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], failure='radar', no_debris_test=True)
        result = simulation.run()
        self.assertTrue(result['quiet_coast'])
        self.assertEqual(result['mode'], 'CLEAR_COAST')
        self.assertEqual(simulation.radar_reads, 0)
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0.
                            for _, linear, angular in simulation.commands))

    def test_empty_radar_in_normal_competition_still_inhibits_blind_acceleration(self):
        simulation = FakeSimulation(failure='empty')
        result = simulation.run()
        self.assertEqual(result['mode'], 'SENSOR_BRAKE')
        self.assertGreater(simulation.radar_reads, 0)
        self.assertTrue(all(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))




    def test_safe_coast_turns_all_thrusters_off_even_with_a_misaligned_nose(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], rotation=rotation, radar_period=.5)
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
        hazard = AvoidanceDecision(np.array([0., .05, 0.]), 'EMERGENCY',
                                   Encounter(clearance=1., time=5.), Encounter(clearance=-5., time=5.), True)
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.])
        result = simulation.run(avoidance_decision=hazard)
        self.assertFalse(result['quiet_coast'])
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))

    def test_stale_radar_cancels_quiet_coast(self):
        simulation = FakeSimulation(distance=300., velocity=[10., 0., 0.], radar_period=4.)
        simulation.run()
        self.assertTrue(any('SENSOR_BRAKE' in message for message in simulation.messages))

    def test_nonurgent_avoidance_can_continue_coarse_alignment(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        moderate = AvoidanceDecision(np.zeros(3), 'AVOID', Encounter(clearance=1.),
                                     Encounter(clearance=-1., time=15.))
        simulation = FakeSimulation(rotation=rotation)
        simulation.run(avoidance_decision=moderate)
        self.assertTrue(any(np.linalg.norm(angular) > 0. for _, _, angular in simulation.commands))

    def test_urgent_avoidance_does_not_chase_target_alignment(self):
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        hazard = AvoidanceDecision(np.array([0., .05, 0.]), 'EMERGENCY',
                                   Encounter(clearance=1.), Encounter(clearance=-5., time=3.), True)
        simulation = FakeSimulation(rotation=rotation)
        simulation.run(avoidance_decision=hazard)
        self.assertTrue(all(np.linalg.norm(angular) == 0. for _, _, angular in simulation.commands))

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
        self.assertEqual(result['mode'], 'SENSOR_BRAKE')
        self.assertTrue(all(np.linalg.norm(linear) == 0. for _, linear, _ in simulation.commands))

    def test_recording_failure_does_not_disable_guidance(self):
        simulation = FakeSimulation()
        result = simulation.run(recording_failure=True)
        self.assertIsNone(result['recorder'])
        self.assertNotEqual(result['mode'], 'SENSOR_BRAKE')
        self.assertTrue(any(np.linalg.norm(linear) > 0. for _, linear, _ in simulation.commands))

    def test_entering_success_area_latches_zero_thrust(self):
        simulation = FakeSimulation(arrived=True)
        result = simulation.run()
        self.assertTrue(result['mission_complete'])
        self.assertEqual(result['mode'], 'ARRIVED')
        self.assertTrue(all(np.linalg.norm(np.r_[linear, angular]) == 0. for _, linear, angular in simulation.commands))


if __name__ == '__main__':
    unittest.main()
