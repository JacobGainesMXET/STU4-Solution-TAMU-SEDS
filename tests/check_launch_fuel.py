"""Fly the controller with the installed jet allocation math and an ideal rigid body.

Run separately for each controller so Python loads the matching guidance and recorder.
Optional synthetic radar scenes include spherical clearance measurements. The
plant has no contact response, damage model or launchpad constraint.
"""
import argparse
from contextlib import ExitStack
import csv
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import warnings

import numpy as np
from scipy.optimize import linprog

ROOT = Path(__file__).resolve().parents[1]


def compare(previous, through_arrival, before_transit=False, *, before_score=False,
            scene=None, seed=7, attitude='flight', packages=None, main_burn_until=None, pause=None,
            delayed_radar=False, controller_dir=None, controller_hz=None, recorded_launch=None):
    directory = controller_dir.resolve() if controller_dir is not None else ROOT
    if previous:
        directory = ROOT / 'comparison/pre_forward_only'
    elif before_transit:
        directory = ROOT / 'comparison/pre_transit_target'
    elif before_score:
        directory = ROOT / 'comparison/pre_score_optimization'
    sys.path.insert(0, str(directory))
    from test_controller_loop import FakeSimulation, StopLoop
    from test_allocator_response import AllocatorResponseTests
    from test_controller_math import active_rotation
    from test_guidance import Sample
    import Guidance
    import RoutePlanner

    AllocatorResponseTests.setUpClass()
    jets = AllocatorResponseTests()
    # Classify by the physical force axes before any test-only change of basis.
    jet_force = jets.mass * np.linalg.norm(jets.activity[:, :3], axis=1)
    bottom = jets.mass * jets.activity[:, 2] > .99 * jet_force
    gravity = np.array([-.006, 0., 0.])
    origin = np.array([0., 0., -10000.])
    departure_axis = np.array([1., 0., 0.])
    environment = None
    display_threshold = None
    if packages is not None:
        from audit_coasts import Environment, FINAL_CONFIG, MU
        from scipy.spatial.transform import Rotation
        config = json.loads(FINAL_CONFIG.read_bytes())
        if controller_hz is None:
            controller_system = next(s for s in config['SystemManager']['Systems'] if s['Nametag'] == 'Controller')
            controller_hz = float(controller_system['Inst_Parameters']['LoopFreqHz'])
        entities = {e['#Required']['Name']: e for e in config['DataManager']['entities']}
        shuttle, cycler = entities['SpaceBuggy'], entities['AsteroidCycler']
        display_threshold = shuttle['JetSelection']['RCSMinOnTime']
        origin = np.array(shuttle['Location'][1:])
        target_position = np.array(cycler['Location'][1:])
        departure_axis = Guidance.unit(target_position - origin)
        environment = Environment(packages, config['DataManager']['SimEntity']['Time']['StartDateTAI'][1])
    # Synthetic objects and sensor observations belong only to this test plant.
    cases = {
        'launch-crossing': [([29., 9., 1.], [-.25, -.45, 0.], 3.)],
        'head-on': [([500., 0., 0.], [-1., 0., 0.], 2.)],
        'crossings': [([420., 100., 0.], [-.2, -1.13, 0.], 2.),
                      ([655., 80., -15.], [0., -.7, .1], 4.),
                      ([755., -85., 4.], [0., .8, 0.], 3.)],
        'offset-crossings': [([320., 60., 12.], [.5, -.85, -.1], 5.),
                             ([830., -100., 0.], [.2, .7, 0.], 8.)],
    }
    geometry = 'launch-crossing' if scene == 'late-launch-crossing' else scene
    rocks = cases[geometry] if geometry and geometry != 'clear-corridor' else []
    if delayed_radar or scene in ('clear-corridor', 'late-launch-crossing'):
        # Ten distant benign returns keep receipt of other radar batches fresh
        # while an individual near-field contact awaits its next observation.
        rocks = rocks + [([100. * i, 1800. + 80.*i, 100.], [0., 0., 0.], 2.) for i in range(10)]
    own_history = Guidance.StateHistory()
    rock_states = []
    if environment and rocks:
        # Rotate the synthetic corridor about the final departure line. The
        # debris and shuttle then experience the same position-dependent field.
        side = Guidance.unit(np.cross(departure_axis, np.eye(3)[np.argmin(np.abs(departure_axis))]))
        corridor = np.column_stack((departure_axis, side, np.cross(departure_axis, side)))
        rock_states = [(origin + corridor @ offset, corridor @ drift) for offset, drift, _ in rocks]
    rng = np.random.default_rng(seed)
    samples = [None] * len(rocks)
    stamps = [-1.] * len(rocks)

    class Launch(FakeSimulation):
        fuel = 0.
        at_20 = None
        at_speed = None
        peak_closing = 0.
        min_gap = float('inf')
        peak_launch_spin = 0.
        peak_hull_rotation = 0.
        first_translation = None
        bottom_fuel = 0.
        other_plume_seconds = 0.
        current_other_plume = 0.
        longest_other_plume = 0.

        def target_state(self):
            if environment is None:
                return super().target_state()
            return SimpleNamespace(loc_WRT_ExprIn=lambda frame: target_position,
                                   vel_WRT_ExprIn=lambda frame: np.zeros(3))

        def radar(self, count):
            for i, (offset, drift, radius) in enumerate(rocks):
                # Each object updates once per sweep, at its assigned phase.
                period = 500. / 120.
                phase = period * i / len(rocks)
                available = self.now
                if delayed_radar:
                    period = 8.6
                    phase = period * ((3*i) % 10) / 10.
                    available -= 1.
                    if available < 0.:
                        return []
                stamp = max(0., phase + period * np.floor((available - phase) / period))
                if scene == 'late-launch-crossing' and i == 0 and stamp < 6.:
                    continue
                if stamp <= stamps[i]:
                    continue
                age = self.now - stamp
                if environment:
                    pos, vel = rock_states[i]
                    accel = environment.body_acceleration(self.now, pos, vel)
                    velocity = vel - age * accel
                    rel = pos - age * vel + .5 * age**2 * accel
                else:
                    velocity = np.asarray(drift) + gravity * stamp
                    rel = origin + offset + np.asarray(drift)*stamp + .5*gravity*stamp**2
                if delayed_radar:
                    ship_pos, ship_vel = own_history.at(stamp)
                else:
                    ship_pos, ship_vel = self.position - age * self.velocity, self.velocity
                rel -= ship_pos
                distance = np.linalg.norm(rel)
                rate = (velocity - ship_vel) @ rel / distance
                samples[i] = Sample(rel + rng.normal(0., 2., 3), distance - radius + rng.normal(0., 2.),
                                    rate + rng.normal(0., .02), 2.*radius*(1. + rng.normal(0., .05)), stamp)
                stamps[i] = stamp
            return sorted((s for s in samples if s is not None), key=lambda s: s.center_range)[:count]

        def get_param(self, kind, key):
            if key == 'LoopFreqHz' and controller_hz is not None:
                return controller_hz
            return self.fuel if key == ['Stats', 'FuelUsed'] else super().get_param(kind, key)

        def sleep(self, dt):
            gap = np.linalg.norm(self.target_state().loc_WRT_ExprIn(None) - self.position)
            if self.now >= (350. if through_arrival else 60.) or gap <= 50.:
                raise StopLoop()
            if self.first_translation is None and np.any(self.command):
                self.first_translation = self.now
            rhs = np.r_[self.command, self.angular]
            times = np.zeros(len(jets.activity))
            if np.linalg.norm(rhs) > 1e-9:
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', DeprecationWarning)
                    solution = linprog(np.ones(len(times)), A_eq=jets.activity.T, b_eq=rhs,
                                       method='revised simplex')
                if not solution.success:
                    raise ValueError(solution.message)
                times = solution.x
            impulse, kg = jets.helpers['GetEffectiveImpulseAndFuelUse'](
                jets.activity, jets.mass, jets.inertia, times, jets.period)
            # The installed allocator scales every selected jet's on-time by
            # one common factor. Its share of fuel equals its share of impulse.
            spent = np.maximum(times, 0.) * jet_force
            if kg > 0.:
                self.bottom_fuel += kg * dt / jets.period * spent[bottom].sum() / spent.sum()
            if display_threshold is not None:
                lit = jets.helpers['GetThrustersToFire'](times, display_threshold, jets.period)
                if any(not bottom[i] for i in lit):
                    self.other_plume_seconds += dt
                    self.current_other_plume += dt
                    self.longest_other_plume = max(self.longest_other_plume, self.current_other_plume)
                else:
                    self.current_other_plume = 0.
            external = environment.body_acceleration(self.now, self.position, self.velocity) if environment else gravity
            accel = self.rotation @ impulse[:3] / (jets.mass * jets.period) + external
            torque = impulse[3:] / jets.period - np.cross(self.spin, jets.inertia @ self.spin)
            self.spin += np.linalg.solve(jets.inertia, torque) * dt
            self.rotation = self.rotation @ active_rotation(self.spin * dt)
            hull_angle = np.degrees(np.arccos(np.clip((np.trace(initial.T @ self.rotation) - 1.) / 2., -1., 1.)))
            self.peak_hull_rotation = max(self.peak_hull_rotation, hull_angle)
            self.position += self.velocity*dt + .5*accel*dt**2
            self.velocity += accel*dt
            self.fuel += kg * dt / jets.period
            self.now += dt
            if self.now <= Guidance.LAUNCH_PLAN_TIME:
                self.peak_launch_spin = max(self.peak_launch_spin, np.degrees(np.linalg.norm(self.spin)))
            if delayed_radar:
                own_history.add(self.now, self.position, self.velocity)
            for i, (offset, drift, radius) in enumerate(rocks):
                if environment:
                    pos, vel = rock_states[i]
                    accel = environment.body_acceleration(self.now - dt, pos, vel)
                    rock = pos + dt * vel + .5 * dt**2 * accel
                    rock_states[i] = (rock, vel + dt * accel)
                else:
                    rock = origin + offset + np.asarray(drift)*self.now + .5*gravity*self.now**2
                self.min_gap = min(self.min_gap, np.linalg.norm(rock - self.position) - radius - 2.5)
            sample = (round(self.now, 2), round(self.fuel, 2), round(self.velocity @ departure_axis, 2),
                      round(np.degrees(np.arccos(np.clip(self.rotation[:, 0] @ departure_axis, -1., 1.))), 1))
            to_target = self.target_state().loc_WRT_ExprIn(None) - self.position
            self.peak_closing = max(self.peak_closing, self.velocity @ Guidance.unit(to_target))
            if self.at_20 is None and self.now >= 20.:
                self.at_20 = sample
            if self.at_speed is None and self.velocity @ departure_axis >= 3.:
                self.at_speed = sample

    initial = active_rotation([0., np.radians(93.5), 0.])
    if environment:
        initial = Rotation.from_quat(shuttle['Rotation'][1:]).as_matrix().T
    if attitude == 'aligned':
        initial = Rotation.align_vectors([departure_axis], [[1., 0., 0.]])[0].as_matrix() if environment else np.eye(3)
    if attitude == 'top':
        basis = active_rotation([0., -np.pi/2., 0.])
        initial = initial @ basis
        jets.activity = np.column_stack((jets.activity[:, :3] @ basis, jets.activity[:, 3:] @ basis))
        jets.inertia = basis.T @ jets.inertia @ basis
    launch = Launch(no_debris_test=not rocks, rotation=initial, low_tilt=attitude == 'low-tilt',
                    bottom_lift=attitude == 'bottom-lift')
    if environment:
        launch.position = origin.copy()
        launch.phobos_mu_km3_s2 = MU / 1e9
    if recorded_launch is not None:
        if not environment or rocks:
            raise ValueError('Recorded starts require final gravity and an empty field')
        with recorded_launch.open(newline='') as stream:
            pad = None
            for row in csv.DictReader(stream):
                if float(row['fuel']) > 0.:
                    break
                pad = row
            else:
                raise ValueError('Recording contains no powered departure')
        if pad is None or float(pad['hull_rotation_deg']) > .2 or float(pad['damage']) > 0.:
            raise ValueError('Expected an undamaged pre-burn sample at the launch attitude')
        launch.position = np.array([float(pad[k]) for k in ('x', 'y', 'z')])
        launch.velocity = np.array([float(pad[k]) for k in ('vx', 'vy', 'vz')])
        launch.now = float(pad['t'])
    if delayed_radar:
        own_history.add(launch.now, launch.position, launch.velocity)
    with ExitStack() as context:
        if main_burn_until is not None:
            def cutoff(guide, r, v, g, now, fuel, entry, radius):
                guide.coast_arrival = now + entry
                return now >= main_burn_until
            context.enter_context(patch.object(Guidance.AccelerationGuidance, '_score_cutoff', cutoff))
        if pause is not None:
            normal_decision = Guidance.AccelerationGuidance.decide
            def split_burn(guide, r, v, g, now, fuel, reference, **kwargs):
                if pause[0] <= now < pause[1]:
                    return Guidance.GuidanceDecision(np.zeros(3), 'CLEAR_COAST', 300.)
                return normal_decision(guide, r, v, g, now, fuel, reference, **kwargs)
            context.enter_context(patch.object(Guidance.AccelerationGuidance, 'decide', split_burn))
        if attitude == 'hold':
            def direct(request, nose, goal, velocity):
                return request
            context.enter_context(patch.object(Guidance, 'angular_command', return_value=np.zeros(3)))
            context.enter_context(patch.object(Guidance, 'rear_thrust', direct))
            context.enter_context(patch.object(RoutePlanner, 'rear_thrust', direct))
        launch.run(source_path=directory / 'Controller.py')
    faults = [msg for msg in launch.messages if 'Guidance degraded:' in msg or 'forecast unavailable:' in msg]
    if faults:
        raise RuntimeError(faults[0])
    print('time_s, fuel_kg, closing_m_s, nose_deg')
    print('controller Hz', launch.get_param(None, 'LoopFreqHz'))
    print('at 20 s', launch.at_20, 'at 3 m/s', launch.at_speed)
    print('first translation (s)', launch.first_translation, 'peak hull rotation (deg)', round(launch.peak_hull_rotation, 2))
    print('peak launch spin (deg/s)', round(launch.peak_launch_spin, 2))
    print('bottom jet fuel (kg)', round(launch.bottom_fuel, 3),
          'other jet fuel (kg)', round(launch.fuel - launch.bottom_fuel, 3))
    if display_threshold is not None:
        print('other plume indicated (s)', round(launch.other_plume_seconds, 2),
              'longest continuous indication (s)', round(launch.longest_other_plume, 2))
    if through_arrival:
        gap = np.linalg.norm(launch.target_state().loc_WRT_ExprIn(None) - launch.position)
        print('arrival', gap <= 50., 'time', round(launch.now, 2), 'fuel', round(launch.fuel, 2))
        print('peak closing speed', round(launch.peak_closing, 2))
        if rocks:
            print('minimum modeled hull gap', round(launch.min_gap, 2))
        if gap <= 50. and launch.min_gap > 0.:
            print('zero-damage score', round(50. + 12000./launch.now - 100.*launch.fuel/75., 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    baseline = parser.add_mutually_exclusive_group()
    baseline.add_argument('--controller-dir', type=Path, help='Run a saved controller and its matching flight modules.')
    baseline.add_argument('--previous', action='store_true')
    baseline.add_argument('--before-transit', action='store_true', help='Use the saved under-50 kg flight controller.')
    baseline.add_argument('--before-score', action='store_true', help='Use the saved 150-second controller.')
    parser.add_argument('--scene', choices=('clear-corridor', 'launch-crossing', 'late-launch-crossing',
                                            'head-on', 'crossings', 'offset-crossings'))
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--attitude', choices=('flight', 'hold', 'top', 'aligned', 'low-tilt', 'bottom-lift'), default='flight')
    parser.add_argument('--through-arrival', action='store_true')
    parser.add_argument('--final-gravity', type=Path, metavar='PACKAGES',
                        help='Use the final departure pose and SPICE gravity from Compute_Server/packages.')
    parser.add_argument('--controller-hz', type=float, help='Override the controller rate for timing sensitivity tests.')
    parser.add_argument('--recorded-launch', type=Path, metavar='TELEMETRY_CSV',
                        help='Start the empty-field plant at the last recorded zero-fuel pose and velocity.')
    parser.add_argument('--main-burn-until', type=float, metavar='SECONDS',
                        help='Experiment with a fixed main-burn cutoff. Flyby correction stays active.')
    parser.add_argument('--pause', type=float, nargs=2, metavar=('START', 'END'),
                        help='Insert an unpowered interval into a prescribed clear-field main burn.')
    parser.add_argument('--delayed-radar', action='store_true',
                        help='Use an 8.6 s synthetic sweep, 1 s latency and distant background returns.')
    args = parser.parse_args()
    if args.controller_hz is not None and not 0. < args.controller_hz < float('inf'):
        parser.error('controller rate must be finite and positive')
    if args.pause and (args.scene or args.main_burn_until is None or
                       not 0. < args.pause[0] < args.pause[1] < args.main_burn_until):
        parser.error('the pause must fall inside a prescribed clear-field burn')
    if args.delayed_radar and not args.scene:
        parser.error('delayed radar requires a synthetic debris scene')
    compare(args.previous, args.through_arrival, args.before_transit, before_score=args.before_score,
            scene=args.scene, seed=args.seed, attitude=args.attitude, packages=args.final_gravity,
            main_burn_until=args.main_burn_until, pause=args.pause, delayed_radar=args.delayed_radar,
            recorded_launch=args.recorded_launch,
            controller_dir=args.controller_dir, controller_hz=args.controller_hz)
