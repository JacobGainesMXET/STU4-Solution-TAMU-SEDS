"""Paired point-mass flights using identical radar noise seeds and initial conditions."""
import importlib.util
import argparse
import sys
from pathlib import Path
from unittest.mock import patch

import offline_scenarios as sim


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def compare(seed=7, scene=None, fixed_speeds=False, as_flown=False):
    snapshot = 'pre_forward_only' if as_flown else 'pre_early_routes'
    saved = Path(__file__).resolve().parents[1] / 'comparison' / snapshot
    previous = load_module(saved / 'Guidance.py', snapshot + '_guidance')
    names = ('MissionModel', 'RadarTracker', 'AccelerationGuidance', 'AvoidancePlanner',
             'PredictionScene', 'StateHistory', 'PulseController', 'learn_burn_model')
    source = sim if fixed_speeds else previous
    controls = {key: getattr(source, key) for key in names}
    if as_flown:
        with patch.dict(sys.modules, Guidance=previous):
            controls['RoutePlanner'] = load_module(saved / 'RoutePlanner.py', snapshot + '_routes').RoutePlanner
    scenes = {
        'clear': [],
        'head-on': [([500., 0., 0.], [-1., 0., 0.])],
        'crossing': [([420., 100., 0.], [-.2, -1.13, 0.])],
        'two crossings': [([655., 80., -15.], [0., -.7, .1]), ([755., -85., 4.], [0., .8, 0.])],
        'two slow drifters': [([540., -10., 5.], [0., .3, -.05]), ([612., -13., 6.], [0., .3, -.05])],
    }
    for name, rocks in scenes.items():
        if scene is not None and scene != name:
            continue
        with patch.multiple(sim, **controls):
            before = sim.run(obstacles=rocks, seed=seed, plan_routes=as_flown)
            lower_speed = None if as_flown else sim.run(obstacles=rocks, seed=seed, plan_routes=False, coast_speed=9.)
        after = sim.run(obstacles=rocks, seed=seed, plan_routes=True)
        yield name, before, lower_speed, after


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--scene', choices=('clear', 'head-on', 'crossing', 'two crossings', 'two slow drifters'))
    controls = parser.add_mutually_exclusive_group()
    controls.add_argument('--fixed-speeds', action='store_true', help='Use the current tracker for both fixed-speed controls.')
    controls.add_argument('--as-flown', action='store_true', help='Compare with the controller saved before forward-only avoidance.')
    args = parser.parse_args()
    control = 'current fixed' if args.fixed_speeds else 'previous'
    for name, before, lower_speed, after in compare(args.seed, args.scene, args.fixed_speeds, args.as_flown):
        label_before = 'as flown' if args.as_flown else control + ' at 10'
        for label, row in ((label_before, before), (control + ' at 9', lower_speed), ('revised', after)):
            if row is None:
                continue
            print(name, label, {key: row[key] for key in
                  ('arrived', 'collision', 'time', 'fuel', 'ideal_score', 'min_true_gap', 'cruise', 'route_legs',
                   'first_route_s', 'avoidance_plans')})
