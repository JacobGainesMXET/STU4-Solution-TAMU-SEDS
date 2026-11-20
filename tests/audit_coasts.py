"""Offline comparison of gravity forecasts on fuel-flat Space Teams telemetry.

Run from the team checkout with the installed simulator Python. SPICE inputs
belong to the offline reference model. This script never connects to a sim.
"""
import argparse
from collections import deque
import csv
from datetime import datetime
from functools import lru_cache
import hashlib
from itertools import groupby
import json
from pathlib import Path
import re

import numpy as np
import spiceypy as spice

ROOT = Path(__file__).resolve().parents[1]
FINAL_CONFIG = ROOT.parent / 'STU4-Sim/Sims/FINAL_SUBMISSION.simconf'
MU = 707500.0
MARS_MU = 42828.374e9


def point_mass(position, mu=MU):
    return -mu * position / np.linalg.norm(position)**3


class Environment:
    def __init__(self, packages, epoch):
        for kernel in ('naif0012.tls', 'pck00010.tpc', 'de440.bsp', 'mar097.bsp'):
            spice.furnsh(str(packages / 'Spicy' / 'kernels' / kernel))
        tai = datetime.fromisoformat(epoch.rstrip('Z')) - datetime(2000, 1, 1, 12)
        self.epoch = spice.unitim(tai.total_seconds(), 'TAI', 'ET')

    @lru_cache(maxsize=2048)
    def primary_states(self, sec):
        et = self.epoch + sec
        mars = 1000. * spice.spkpos('MARS', et, 'J2000', 'NONE', 'PHOBOS')[0]

        def velocity(offset):
            return spice.spkezr('PHOBOS', et + offset, 'J2000', 'NONE', 'SSB')[0][3:]

        origin_accel = 500. * (velocity(1.) - velocity(-1.))
        return mars, origin_accel

    def relative_acceleration(self, sec, position):
        mars, origin_accel = self.primary_states(sec)
        return point_mass(position) + point_mass(position - mars, MARS_MU) - origin_accel

    def transform(self, sec):
        return spice.sxform('IAU_PHOBOS', 'J2000', self.epoch + sec)

    @lru_cache(maxsize=512)
    def body_terms(self, sec):
        state = self.transform(sec)
        rotation = state[:3, :3]
        rate = rotation.T @ state[3:, :3]
        rotation_ddot = .5 * (self.transform(sec + 1)[3:, :3] - self.transform(sec - 1)[3:, :3])
        mars, origin_accel = self.primary_states(sec)
        return rotation.T @ mars, rotation.T @ origin_accel, rate, rotation.T @ rotation_ddot

    def body_acceleration(self, sec, position, velocity):
        # Freeze the celestial coefficients for at most half a second. Evaluate the
        # spatial gravity and velocity-dependent frame terms at each plant tick.
        mars, origin_accel, rate, curvature = self.body_terms(round(sec))
        return (point_mass(position) + point_mass(position - mars, MARS_MU)
                - origin_accel - 2. * rate @ velocity - curvature @ position)

    def forecasts(self, sec, position, velocity, gravity, duration, step):
        """Velocity Verlet in local axes for the gradient model and inertial axes for SPICE."""
        residual = gravity - point_mass(position)
        local, local_v = position.copy(), velocity.copy()
        xyz = self.transform(sec) @ np.r_[position, velocity]
        pos, vel = xyz[:3], xyz[3:]
        count = max(1, int(np.ceil(duration / step)))
        dt = duration / count
        for i in range(count):
            t = sec + i * dt
            old = self.relative_acceleration(t, pos)
            pos = pos + dt * vel + .5 * dt**2 * old
            vel = vel + .5 * dt * (old + self.relative_acceleration(t + dt, pos))
            old = point_mass(local) + residual
            local += dt * local_v + .5 * dt**2 * old
            local_v += .5 * dt * (old + point_mass(local) + residual)
            elapsed = (i + 1) * dt
            linear = position + elapsed * velocity
            yield elapsed, (linear, linear + .5 * gravity * elapsed**2, local.copy(),
                            self.transform(sec + elapsed)[:3, :3].T @ pos)


@lru_cache(maxsize=1)
def scenario_logs():
    logs = ROOT.parents[2] / 'Logs'
    by_recording = {}
    scenario = FINAL_CONFIG.read_bytes()
    for log in logs.glob('*FINAL_SUBMISSION*/FINAL_SUBMISSION_PySys_Controller.log'):
        archived = log.parent / 'FINAL_SUBMISSION.simconf'
        if not archived.is_file() or archived.read_bytes() != scenario:
            continue
        match = re.search(r'Flight diagnostics: .*?diagnostics[\\/](\d{8}T\d{6}_\d{6}Z)',
                          log.read_text(encoding='utf-8', errors='replace'))
        if match:
            by_recording[match[1]] = log.parent
    return by_recording


def matched_flights():
    hashes = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
              for name in ('Controller.py', 'Guidance.py', 'RoutePlanner.py')}
    for file in sorted((ROOT / 'diagnostics').glob('*/model.json')):
        if file.parent.name not in scenario_logs() or json.loads(file.read_text())['source_sha256'] != hashes:
            continue
        with (file.parent / 'telemetry.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        if rows and rows[0]['no_debris_test'] == '0':
            end = next((i+1 for i, r in enumerate(rows) if r['mode'] == 'ARRIVED'), len(rows))
            yield file.parent.name, rows[:end]


def arrays(rows, keys):
    return np.array([[float(row[k]) for k in keys.split()] for row in rows])


def coast_segments(rows):
    t, fuel, damage, linear, angular = arrays(rows, 't fuel damage linear_on angular_on').T
    start = None
    for i in range(len(t)):
        quiet = not (linear[i] or angular[i]) and t[i] > 20.
        if start is not None and (not quiet or fuel[i] != fuel[i-1] or damage[i] != damage[i-1]
                                  or t[i] - t[i-1] > 2.5):
            yield start, i-1
            start = None
        if start is None and quiet:
            start = i
    if start is not None:
        yield start, len(t)-1


def compare(env, verbose):
    for horizon in (30., 90., 150.):
        errors = []
        for name, rows in matched_flights():
            t = arrays(rows, 't').ravel()
            pos, vel, gravity = (arrays(rows, keys) for keys in ('x y z', 'vx vy vz', 'gx gy gz'))
            for start, end in coast_segments(rows):
                i = np.searchsorted(t, t[start] + 3.)
                if i > end or t[end] - t[i] < horizon:
                    continue
                j = np.searchsorted(t, t[i] + horizon)
                path = env.forecasts(t[i], pos[i], vel[i], gravity[i], t[j] - t[i], 2.)
                _, predictions = deque(path, maxlen=1).pop()
                err = np.linalg.norm(np.array(predictions) - pos[j], axis=1)
                errors.append(err)
                if verbose:
                    print(name, round(t[i], 1), round(t[j]-t[i], 1), np.round(err, 3))
                break
        if errors:
            print(horizon, 's, flights', len(errors), 'median/max metres (linear, frozen, gradient, SPICE)')
            print(np.round(np.median(errors, axis=0), 3), np.round(np.max(errors, axis=0), 3))


def passage(env, row):
    """Closest range and first entry under zero further thrust, for a stationary target."""
    pos, vel, g, target, target_vel = (arrays([row], keys)[0] for keys in
                                       ('x y z', 'vx vy vz', 'gx gy gz', 'target_x target_y target_z',
                                        'target_vx target_vy target_vz'))
    if np.linalg.norm(target_vel) > 1e-6:
        raise ValueError('This replay requires the stationary final-scenario target.')
    start = float(row['t'])
    old = np.array([pos - target, pos - target])
    minimum = np.linalg.norm(old, axis=1)
    entry = np.where(minimum <= 50., 0., np.inf)
    last = 0.
    for elapsed, estimates in env.forecasts(start, pos, vel, g, 360., 1.):
        new = np.array([estimates[1], estimates[3]]) - target
        step = new - old
        square = np.maximum(np.sum(step**2, axis=1), 1e-20)
        projection = -np.sum(old * step, axis=1) / square
        closest = old + np.clip(projection, 0., 1.)[:, None] * step
        minimum = np.minimum(minimum, np.linalg.norm(closest, axis=1))
        disc = projection**2 - (np.sum(old**2, axis=1) - 50.**2) / square
        fraction = projection - np.sqrt(np.maximum(0., disc))
        crossed = (disc >= 0.) & (fraction >= 0.) & (fraction <= 1.) & np.isinf(entry)
        entry[crossed] = last + fraction[crossed] * (elapsed - last)
        last, old = elapsed, new
    return minimum, entry


def decisions(env):
    print('First trim in each arriving flight. Frozen/full-field closest range (m) and entry (s from state).')
    terminal_errors = []
    for name, rows in matched_flights():
        if rows[-1]['mode'] != 'ARRIVED':
            continue
        trim = next((i for i, row in enumerate(rows) if row['mode'] == 'TRIM_BURN'), None)
        if trim is not None:
            for i in sorted({max(0, trim - 1), trim}):
                minimum, entry = passage(env, rows[i])
                print(name, rows[i]['mode'], round(float(rows[i]['t']), 1),
                      np.round(minimum, 2), np.round(entry, 2))
        t = arrays(rows, 't').ravel()
        for a, b in coast_segments(rows):
            # Only compare to arrival when the uninterrupted coast reaches the last
            # pre-entry sample and the last step is also fuel/damage flat.
            if (b < len(rows)-2 or t[-1] - t[b] > 2.5 or rows[-1]['fuel'] != rows[b]['fuel']
                    or rows[-1]['damage'] != rows[b]['damage']):
                continue
            i = np.searchsorted(t, t[a] + 3.)
            if i > b or t[-1] - t[i] < 30.:
                continue
            _, entry = passage(env, rows[i])
            actual = t[-1] - t[i]
            terminal_errors.append(entry - actual)
            print(name, 'unbroken coast to first recorded entry', round(actual, 2),
                  'predicted', np.round(entry, 2))
    if terminal_errors:
        errors = np.array(terminal_errors)
        print('Terminal forecasts with finite entry', np.isfinite(errors).sum(axis=0), '/', len(errors))
        for col in errors.T:
            finite = col[np.isfinite(col)]
            if finite.size:
                print('Absolute ETA error for finite entries, median/max s', np.round(
                    [np.median(np.abs(finite)), np.max(np.abs(finite))], 2))


def official_outcomes():
    arrived = []
    for name, _ in matched_flights():
        log = scenario_logs()[name] / 'FINAL_SUBMISSION_PySys_Scoring.log'
        result = log.read_text(encoding='utf-8', errors='replace')
        finish = re.search(r'(escaped Phobos with|finished with) a score of (-?[\d.]+);', result)
        if not finish:
            print(name, 'no official result')
            continue
        values = [float(re.search(key + r'\s*=\s*([\d.]+)', result)[1])
                  for key in ('Sim elapsed time', 'Total fuel used', 'Total damage taken')]
        record = [float(finish[2]), *values]
        print(name, finish[1], record)
        if finish[1] == 'escaped Phobos with':
            arrived.append(record)
    if arrived:
        print('Official arrivals, count and mean score/time/fuel/damage', len(arrived),
              np.round(np.mean(arrived, axis=0), 3))


def encounter_replay():
    from audit_departure import (MissionModel, PredictionScene, RadarTracker, StateHistory,
                                 radar_contact, read_rows)
    for name, rows in matched_flights():
        times, damage = arrays(rows, 't damage').T
        hits = np.flatnonzero(np.diff(damage, prepend=0.) > 0.)
        if not hits.size:
            continue
        impact = times[hits[0]]
        directory = ROOT / 'diagnostics' / name
        model = MissionModel(**json.loads((directory / 'model.json').read_text())['model'])
        tracker, history = RadarTracker(model), StateHistory()
        positions, velocities, gravity = (arrays(rows, keys) for keys in ('x y z', 'vx vy vz', 'gx gy gz'))
        history.samples.extend(zip(times, positions, velocities))
        groups = groupby(read_rows(directory / 'radar.csv'), key=lambda r: float(r['received_t']))
        print(name, 'first recorded damage at', round(impact, 2),
              's. Incoming samples within 60 m in previous 15 s, assuming continued coast.')
        for received, batch in groups:
            if received > impact:
                break
            i = min(len(times)-1, np.searchsorted(times, received))
            contacts = [radar_contact(sample) for sample in batch]
            tracker.update(contacts, received, history, gravity[i])
            if received < impact - 15.:
                continue
            pos, vel = history.at(received)
            for sample in contacts:
                if sample.surface_range >= 60. or sample.range_rate >= -2.:
                    continue
                # No truth IDs. Find the contemporaneous updated state closest
                # to this observation for a retrospective association hypothesis.
                world = history.at(sample.t_meas)[0] + sample.rel_pos
                matches = [tr for tr in tracker.tracks.values() if tr.time == sample.t_meas]
                if not matches:
                    continue
                track = min(matches, key=lambda tr: np.linalg.norm(tr.state[:3] - world))
                state, _ = track.predict(received, gravity[i], model)
                rel, dv = state[:3] - pos, state[3:] - vel
                tca = max(0., -(rel @ dv) / max(dv @ dv, 1e-12))
                scene = PredictionScene(tracker.tracks.values(), received, pos, vel, gravity[i], model,
                                        terminal=float(rows[i]['distance']) <= .3 * float(rows[0]['distance']))
                print('receipt, range, rate, hits, radius, time to CPA, center miss, coast hull gap',
                      np.round([received, sample.surface_range, sample.range_rate, track.hits, track.radius,
                                tca, np.linalg.norm(rel + tca * dv), scene.assess(np.zeros(3)).hull_gap], 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packages', type=Path, help='installed Compute_Server/packages directory')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--decisions', action='store_true', help='Also replay pre-trim and terminal intercepts.')
    parser.add_argument('--official', action='store_true', help='Also print the matched official scorer outcomes.')
    parser.add_argument('--encounters', action='store_true', help='Replay radar approaching the first damage event.')
    args = parser.parse_args()
    sim = json.loads(FINAL_CONFIG.read_bytes())
    epoch = sim['DataManager']['SimEntity']['Time']['StartDateTAI'][1]
    environment = Environment(args.packages, epoch)
    if not any(matched_flights()):
        parser.error('no matching controller diagnostics with corresponding STPro FINAL_SUBMISSION logs')
    if args.official:
        official_outcomes()
    compare(environment, args.verbose)
    if args.decisions:
        decisions(environment)
    if args.encounters:
        encounter_replay()
