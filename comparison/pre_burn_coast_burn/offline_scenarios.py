"""Synthetic point-mass experiments, NOT Space Teams mission results.

Run from the solution directory: python -B tests/offline_scenarios.py
No real simulator is imported. The plant has constant .14 m/s^2 authority, instant
attitude and simplified allocator saturation; fuel uses ideal impulse/Isp only.
"""
import ast
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import numpy.typing as npt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Guidance import (MissionModel, StateHistory, RadarTracker, AccelerationGuidance,
                      AvoidancePlanner, PredictionScene, PulseController, learn_burn_model)


def previous_helpers():
    source = (ROOT / 'comparison/Controller_before_predictive.py').read_text(encoding='utf-8-sig')
    nodes = [node for node in ast.parse(source).body
             if (isinstance(node, ast.FunctionDef) and node.name in ('LimitVector', 'ComputeDesiredVelocity', 'ComputeLinearAcceleration'))
             or (isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets))]
    namespace = dict(np=np, npt=npt, math=math, dataclass=dataclass)
    exec(compile(ast.Module(nodes, []), '<previous-translation-helpers>', 'exec'), namespace)
    return namespace


def run(previous=False, obstacle=None, seed=7, coast_speed=8.):
    model = MissionModel(coast_speed=coast_speed)
    history, tracker = StateHistory(), RadarTracker(model)
    guidance, avoidance = AccelerationGuidance(model), AvoidancePlanner(model)
    pulses = PulseController(.04, .008, .25, .08)
    baseline = previous_helpers() if previous else None
    position, velocity = np.zeros(3), np.zeros(3)
    gravity = np.array([-.004, .0005, 0.])
    target_origin, target_velocity = np.array([1300., 0., 0.]), np.array([0., .05, 0.])
    command, observed_dv = np.zeros(3), np.zeros(3)
    old_desired = None
    desired, urgent = np.zeros(3), False
    fuel, coasting, min_gap = 0., 0., math.inf
    window_fuel, window_dv, on_seconds = 0., 0., 0.
    next_plan, next_radar, avoidance_plans = 0., 0., 0
    full_thrust = was_full_thrust = False
    strategies = {}
    rng = np.random.default_rng(seed)
    for step in range(16000):
        now, dt = step * .05, .05
        target = target_origin + target_velocity * now
        relative = target - position
        if np.linalg.norm(relative) <= model.success_radius:
            return dict(arrived=True, time=round(now, 2), fuel=round(fuel, 2),
                        ideal_score=round(model.score(now, fuel), 2),
                        min_true_gap=round(min_gap, 2) if math.isfinite(min_gap) else None,
                        coast_fraction=round(coasting / max(now, dt), 3), collision=bool(min_gap <= 0.),
                        avoidance_plans=avoidance_plans, avoidance_strategies=strategies)
        history.add(now, position, velocity)
        contacts = []
        if obstacle is not None:
            rock_position = np.array(obstacle[0]) + np.array(obstacle[1]) * now + .5 * gravity * now**2
            rock_velocity = np.array(obstacle[1]) + gravity * now
            true_relative = rock_position - position
            distance = np.linalg.norm(true_relative)
            min_gap = min(min_gap, distance - 4.5)
            if now >= next_radar:
                measured = true_relative + rng.normal(0., 2., 3)
                radial = (rock_velocity - velocity) @ true_relative / distance + rng.normal(0., .02)
                contacts = [SimpleNamespace(rel_pos=measured, surface_range=distance - 2., range_rate=radial,
                                           diameter=4., t_meas=now, center_range=np.linalg.norm(measured))]
                tracker.update(contacts, now, history, gravity)
                next_radar = now + 4.
        if previous:
            # Isolate the old velocity servo on a clear path; the old system had no
            # motion predictor to compare here. Do not label obstacle runs as A/B tests.
            v_des, _ = baseline['ComputeDesiredVelocity']([], velocity, relative / np.linalg.norm(relative))
            command, old_desired = baseline['ComputeLinearAcceleration'](v_des, velocity, old_desired, dt, .5)
        else:
            if now >= next_plan:
                decision = guidance.decide(relative, velocity - target_velocity, gravity, now, fuel, np.array([0., 1., 0.]))
                scene = PredictionScene(tracker.tracks.values(), now, position, velocity, gravity, model)
                selected = avoidance.choose(scene, decision.acceleration, velocity - target_velocity, relative, now)
                avoidance_plans += int(selected.mode != 'GUIDANCE')
                strategy = getattr(selected, 'strategy', selected.mode)
                strategies[strategy] = strategies.get(strategy, 0) + 1
                desired, urgent = selected.acceleration, selected.urgent
                full_thrust = ((selected.mode == 'GUIDANCE' and decision.mode == 'ACCELERATE')
                               or getattr(selected, 'forward_burn', False))
                next_plan = now + .25
            transition = full_thrust != was_full_thrust
            if transition:
                pulses.reset()
            command = desired / np.linalg.norm(desired) * .5 if full_thrust else pulses.command(
                desired, np.zeros(3) if transition else observed_dv, dt, now, urgent)
            was_full_thrust = full_thrust
        thrust = command / np.linalg.norm(command) * .14 if np.linalg.norm(command) > 1e-9 else np.zeros(3)
        observed_dv = thrust * dt
        position += velocity * dt + .5 * (thrust + gravity) * dt**2
        velocity += (thrust + gravity) * dt
        fuel += 3388.7 / (80. * 9.8) * np.linalg.norm(thrust) * dt
        window_dv += float(np.linalg.norm(observed_dv))
        on_seconds += dt if np.linalg.norm(thrust) else 0.
        if not previous and step % 20 == 19:
            learn_burn_model(model, fuel - window_fuel, window_dv, 0., 3388.7 / (80. * 9.8), on_seconds)
            window_fuel, window_dv, on_seconds = fuel, 0., 0.
        coasting += dt if np.linalg.norm(thrust) == 0. else 0.
    return dict(arrived=False, time=800., fuel=round(fuel, 2), collision=bool(min_gap <= 0.))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Optional JSON report path')
    parser.add_argument('--sweep', action='store_true', help='Compare illustrative coast thresholds on the synthetic plant')
    arguments = parser.parse_args()
    report = dict(evidence='Synthetic point-mass, constant gravity, ideal fuel, instantaneous attitude; not Space Teams',
                  previous_clear=run(previous=True), revised_clear=run(),
                  revised_head_on=run(obstacle=([500., 0., 0.], [-1., 0., 0.])),
                  revised_crossing=run(obstacle=([420., 100., 0.], [-.2, -1.13, 0.])))
    if arguments.sweep:
        report['synthetic_coast_thresholds'] = {str(speed): run(coast_speed=speed)
                                                for speed in (6., 8., 10., 12.)}
    output = json.dumps(report, indent=2)
    print(output)
    if arguments.output:
        arguments.output.write_text(output + '\n', encoding='utf-8')
