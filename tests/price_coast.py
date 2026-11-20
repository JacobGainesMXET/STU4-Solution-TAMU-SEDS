"""Price a forward burn followed by coast at a recorded Space Teams state."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from audit_departure import MissionModel, read_rows, vector
from Guidance import finite_burn_entry


def compare(recording, requested_times):
    model = MissionModel(**json.loads((recording / 'model.json').read_text())['model'])
    rows = read_rows(recording / 'telemetry.csv')
    for requested in requested_times or [None]:
        if requested is None:
            row = next((r for r in rows if r['mode'] == 'CLEAR_COAST'
                        and r.get('transit_reason') == 'score favors coast'), None)
        else:
            row = next((r for r in rows if float(r['t']) >= requested), None)
        if row is None:
            print(f'{recording.name} has no matching sample for {requested}')
            continue
        now, fuel, damage = (float(row[key]) for key in ('t', 'fuel', 'damage'))
        r = vector(row, 'target_x target_y target_z') - vector(row, 'x y z')
        v = vector(row, 'vx vy vz') - vector(row, 'target_vx target_vy target_vz')
        gravity = vector(row, 'gx gy gz')
        nose = vector(row, 'nose_x nose_y nose_z')
        acceleration = nose * float(row['planning_accel'])
        kg_per_second = np.linalg.norm(acceleration) * float(row['fuel_per_dv'])
        print(f"\n{recording.name} at {now:.2f} s, {row['mode']}, "
              f"closing {float(row['closing_speed']):.2f} m/s, {fuel:.2f} kg spent")
        print('burn_s  arrival_s  extra_kg  predicted_score  gain_over_coast')
        coast_score = None
        for duration in (0., 2., 4., 8., 12., 16., 24.):
            eta = finite_burn_entry(r, v, gravity, acceleration, duration, model.success_radius)
            if not math.isfinite(eta):
                print(f'{duration:6.1f}  no predicted entry')
                continue
            extra = kg_per_second * min(duration, eta)
            score = model.score(now + eta, fuel + extra, damage)
            if duration == 0.:
                coast_score = score
            gain = f'{score - coast_score:+.2f}' if coast_score is not None else 'n/a'
            print(f'{duration:6.1f}  {now + eta:9.2f}  {extra:8.2f}  {score:15.2f}  {gain:>15}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        'Uses the measured nose direction, current gravity and learned burn cost. '
        'Assumes no further maneuvers or damage. This does not check radar clearance.'))
    parser.add_argument('recordings', type=Path, nargs='+')
    parser.add_argument('--at', type=float, nargs='+',
                        help='mission seconds, defaults to the first score-driven clear coast')
    args = parser.parse_args()
    for recording in args.recordings:
        compare(recording, args.at)
