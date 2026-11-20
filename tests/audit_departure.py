"""Compare departure schedules from recorded ownship state and permitted radar.

Run with the installed simulator Python, passing one or more diagnostics folders.
Each row is a snapshot forecast. This does not replay a flown trajectory or
estimate the measured clearance of a maneuver that was never executed.
"""
import argparse
import csv
from dataclasses import fields
from hashlib import sha256
from itertools import groupby
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Guidance import (AccelerationGuidance, AvoidancePlanner, FORWARD_LOOKAHEAD, LAUNCH_PLAN_TIME, MissionModel, PredictionScene,
                      RadarTracker, StateHistory, rear_thrust, unit)
from RoutePlanner import Maneuver, RadarField, forecast


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def vector(row, names):
    return np.array([float(row[key]) for key in names.split()])


def radar_contact(row):
    rel = vector(row, 'rel_x rel_y rel_z')
    return SimpleNamespace(rel_pos=rel, t_meas=float(row['measurement_t']),
                           surface_range=float(row['surface_range']), range_rate=float(row['range_rate']),
                           diameter=float(row['diameter']), center_range=np.linalg.norm(rel))


def rank_departures(directory, times, candidate=False):
    model_info = json.loads((directory / 'model.json').read_text(encoding='utf-8'))
    for name, checksum in model_info['source_sha256'].items():
        if not candidate and sha256((ROOT / name).read_bytes()).hexdigest() != checksum:
            raise ValueError(f'{directory.name} was recorded with different {name} source')
    telemetry = read_rows(directory / 'telemetry.csv')
    allowed = {field.name for field in fields(MissionModel)}
    model = MissionModel(**{key: value for key, value in model_info['model'].items() if key in allowed})
    if candidate:
        model.score_cruise, model.arrival_target = True, None
    state = StateHistory()
    # Interpolate logged ownship samples at the radar measurement timestamps.
    # Use the full list here so that early radar samples remain available.
    state.samples.extend((float(row['t']), vector(row, 'x y z'), vector(row, 'vx vy vz')) for row in telemetry)
    tracker = RadarTracker(model)
    radar_times = groupby(read_rows(directory / 'radar.csv'), key=lambda row: float(row['received_t']))
    batches = ((t, list(rows)) for t, rows in radar_times)
    received, samples = next(batches, (float('inf'), []))
    for requested in sorted(times):
        row = next(row for row in telemetry if float(row['t']) >= requested)
        now, fuel = float(row['t']), float(row['fuel'])
        gravity = vector(row, 'gx gy gz')
        while received <= now:
            tracker.update([radar_contact(sample) for sample in samples], received, state, gravity)
            received, samples = next(batches, (float('inf'), []))
        model.coast_speed = float(row['coast_speed']) if row['coast_speed'] else None
        model.planning_accel = float(row['planning_accel'])
        model.fuel_per_dv = float(row['fuel_per_dv'])
        guide = AccelerationGuidance(model)
        guide.observe_distance(float(telemetry[0]['distance']))
        guide.coast_latched = row['speed_cutoff_reached'] == '1'
        position, velocity = vector(row, 'x y z'), vector(row, 'vx vy vz')
        target_velocity = vector(row, 'target_vx target_vy target_vz')
        r, v = vector(row, 'target_x target_y target_z') - position, velocity - target_velocity
        nose = vector(row, 'nose_x nose_y nose_z')
        goal = unit(r)
        side = unit(np.cross(goal, np.eye(3)[np.argmin(np.abs(goal))]))
        other = np.cross(goal, side)
        start = now if candidate else max(now, LAUNCH_PLAN_TIME)
        choices = [('direct', None)]
        choices.extend((f'wait-{wait}', Maneuver('delay', start, wait, side)) for wait in (3, 6, 12, 20))
        choices.extend((f'tilt-{index}', Maneuver('steer', start, 30., direction, 6.))
                       for index, direction in enumerate((side, -side, other, -other), 1))
        nominal = guide.decide(r, v, gravity, now, fuel, side, nose=nose)
        field = RadarField(tracker.tracks.values(), now, position, target_velocity, gravity, model)
        departure = now < LAUNCH_PLAN_TIME or np.linalg.norm(r) > .90 * guide.launch_distance
        horizon = guide.departure_horizon(np.linalg.norm(r), v @ goal) if departure else model.horizon
        scene = PredictionScene(tracker.tracks.values(), now, position, velocity, gravity, model,
                                horizon=min(horizon, nominal.eta + 1.), departure=departure,
                                terminal=candidate and np.linalg.norm(r) <= .30 * guide.launch_distance)
        direct = None
        yield f'{directory.name} +{now:.1f}s {row["avoidance_strategy"]} ({len(field.ids)} reliable tracks)'
        if candidate:
            launch = now < LAUNCH_PLAN_TIME and nominal.mode == 'ACCELERATE'
            plan = AvoidancePlanner(model).choose(scene, nominal.acceleration, v, goal, now, nose=nose,
                    launch=launch, allow_forward=tracker.fresh(now) and float(row['spin_deg_s']) <= (6. if launch else 3.),
                    rear_guidance=not launch and np.any(nominal.acceleration),
                    transit_burn=guide.transit_reason == 'score favors burn')
            yield (f'  candidate {nominal.mode}/{plan.strategy} hull={plan.encounter.hull_gap:.1f}m '
                   f'base_hull={plan.baseline.hull_gap:.1f}m margin={plan.encounter.clearance:.1f}m')
        for name, action in choices:
            path = forecast(guide, r, v, gravity, side, now, fuel, action, nose)
            risk = field.assess(path)
            direct = risk if direct is None else direct
            command = nominal if action is None else action.apply(nominal, now, goal, model)
            burn = command.burn_seconds or 8.
            if now < LAUNCH_PLAN_TIME:
                burn = min(FORWARD_LOOKAHEAD, LAUNCH_PLAN_TIME - now)
            initial = None if now < LAUNCH_PLAN_TIME else nose
            near = scene.assess(command.acceleration, burn, initial_direction=initial)
            qualified = (bool(len(field.ids)) and tracker.fresh(now) and not risk.unresolved
                         and risk.clears(direct) and near.clearance >= 0. and np.isfinite(path.score))
            executable = (now < LAUNCH_PLAN_TIME or not np.any(command.acceleration)
                          or bool(np.any(rear_thrust(command.acceleration, nose, goal, v))))
            yield (f'  {name:9} arrival={now + path.eta:6.1f}s fuel={path.fuel:5.1f}kg '
                   f'extra_fuel={path.fuel - fuel:5.1f}kg score={path.score:5.1f} '
                   f'tactical={near.clearance:6.1f}m strategic={risk.clearance:6.1f}m '
                   f'unresolved={len(risk.unresolved):2} can_execute={executable} eligible={qualified and executable}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', type=Path, nargs='+')
    parser.add_argument('--times', nargs='+', type=float, default=[18., 21., 30., 40.])
    parser.add_argument('--candidate', action='store_true',
                        help='Evaluate the current score policy on a recording from another controller version.')
    args = parser.parse_args()
    for directory in args.recordings:
        for line in rank_departures(directory, args.times, args.candidate):
            print(line)
