"""Synthetic point-mass experiments, NOT Space Teams mission results.

Run from the solution directory: python -B tests/offline_scenarios.py
No real simulator is imported. Default authority is constant .14 m/s^2; run() also
accepts a different authority and gravity. The current plant turns toward a burn
at a bounded rate, omits allocator dynamics, and charges ideal translation impulse only.
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
from RoutePlanner import RoutePlanner


def previous_helpers():
    source = (ROOT / 'comparison/Controller_before_predictive.py').read_text(encoding='utf-8-sig')
    nodes = [node for node in ast.parse(source).body
             if (isinstance(node, ast.FunctionDef) and node.name in ('LimitVector', 'ComputeDesiredVelocity', 'ComputeLinearAcceleration'))
             or (isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets))]
    namespace = dict(np=np, npt=npt, math=math, dataclass=dataclass)
    exec(compile(ast.Module(nodes, []), '<previous-translation-helpers>', 'exec'), namespace)
    return namespace


def run(previous=False, obstacle=None, seed=7, coast_speed=10., cruise_burns=True,
        plant_accel=.14, gravity=None, plan_routes=True, obstacles=None):
    rocks = list(obstacles) if obstacles is not None else ([] if obstacle is None else [obstacle])
    model = MissionModel(coast_speed=coast_speed, cruise_burns=cruise_burns and coast_speed is not None)
    history, tracker = StateHistory(), RadarTracker(model)
    guidance, avoidance = AccelerationGuidance(model), AvoidancePlanner(model)
    forward_only = not previous and hasattr(guidance, 'observe_distance')
    if forward_only:
        from Guidance import rear_thrust, LAUNCH_PLAN_TIME
    routes = RoutePlanner()
    early_legs = set()
    pulses = PulseController(.04, .008, .25, .08)
    baseline = previous_helpers() if previous else None
    position, velocity = np.zeros(3), np.zeros(3)
    gravity = np.array([-.004, .0005, 0.] if gravity is None else gravity, dtype=float)
    target_origin, target_velocity = np.array([1300., 0., 0.]), np.array([0., .05, 0.])
    command, observed_dv = np.zeros(3), np.zeros(3)
    old_desired = None
    desired, urgent = np.zeros(3), False
    fuel, coasting, min_gap = 0., 0., math.inf
    window_fuel, window_dv, on_seconds = 0., 0., 0.
    next_plan, next_radar, avoidance_plans = 0., 0., 0
    full_thrust = was_full_thrust = route_rear = False
    nose = np.array([1., 0., 0.])
    strategies = {}
    timed_burn = False
    mode_counts = {}
    rng = np.random.default_rng(seed)
    for step in range(16000):
        now, dt = step * .05, .05
        target = target_origin + target_velocity * now
        relative = target - position
        if forward_only:
            guidance.observe_distance(np.linalg.norm(relative))
        if np.linalg.norm(relative) <= model.success_radius:
            return dict(arrived=True, time=round(now, 2), fuel=round(fuel, 2),
                        ideal_score=round(model.score(now, fuel), 2),
                        min_true_gap=round(min_gap, 2) if math.isfinite(min_gap) else None,
                        coast_fraction=round(coasting / max(now, dt), 3), collision=bool(min_gap <= 0.),
                        avoidance_plans=avoidance_plans, avoidance_strategies=strategies,
                        nominal_modes=mode_counts, guidance_burn_plans=getattr(guidance, 'burn_count', 0),
                        cruise=model.coast_speed, route_legs=len(early_legs),
                        first_route_s=min(early_legs, default=None))
        history.add(now, position, velocity)
        contacts = []
        for obstacle in rocks:
            rock_position = np.array(obstacle[0]) + np.array(obstacle[1]) * now + .5 * gravity * now**2
            rock_velocity = np.array(obstacle[1]) + gravity * now
            true_relative = rock_position - position
            distance = np.linalg.norm(true_relative)
            min_gap = min(min_gap, distance - 4.5)
            if now >= next_radar:
                measured = true_relative + rng.normal(0., 2., 3)
                radial = (rock_velocity - velocity) @ true_relative / distance + rng.normal(0., .02)
                contacts.append(SimpleNamespace(rel_pos=measured, surface_range=distance - 2. + rng.normal(0., 2.),
                                                range_rate=radial, diameter=4. + rng.normal(0., .2),
                                                t_meas=now, center_range=np.linalg.norm(measured)))
        if contacts:
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
                if plan_routes and not avoidance.active and decision.mode in ('ACCELERATE', 'CLEAR_COAST'):
                    decision = routes.command(decision, guidance, tracker.tracks.values(), now, position, relative,
                                              velocity - target_velocity, target_velocity, gravity, np.array([0., 1., 0.]),
                                              nose, fuel, known_empty=not rocks)
                else:
                    routes.interrupt(now, 'tactical or nominal priority')
                scene = PredictionScene(tracker.tracks.values(), now, position, velocity, gravity, model)
                options = dict(nominal_burn_seconds=decision.burn_seconds or 8.) if hasattr(guidance, 'burn') else {}
                if routes.action is not None and routes.action.rear_burn and decision.mode == 'ACCELERATE':
                    options.update(rear_guidance=True, nose=nose)
                if forward_only:
                    options.update(launch=now < LAUNCH_PLAN_TIME, coast_only=guidance.distance_coast,
                                   plan_on_coast=(LAUNCH_PLAN_TIME <= now < LAUNCH_PLAN_TIME + 2.
                                                  and bool(rocks) and routes.action is None and decision.mode == 'ACCELERATE'),
                                   rear_guidance=now >= LAUNCH_PLAN_TIME and bool(np.any(decision.acceleration)),
                                   nose=nose)
                selected = avoidance.choose(scene, decision.acceleration, velocity - target_velocity, relative, now, **options)
                if selected.mode in ('AVOID', 'EMERGENCY') and hasattr(guidance, 'finish_burn'):
                    guidance.finish_burn(now)
                    routes.interrupt(now, 'tactical override')
                if routes.action is not None:
                    early_legs.add(routes.action.start)
                mode_counts[decision.mode] = mode_counts.get(decision.mode, 0) + 1
                avoidance_plans += int(selected.mode in ('AVOID', 'EMERGENCY'))
                strategy = getattr(selected, 'strategy', selected.mode)
                strategies[strategy] = strategies.get(strategy, 0) + 1
                desired, urgent = selected.acceleration, selected.urgent
                full_thrust = ((selected.mode == 'GUIDANCE' and decision.mode == 'ACCELERATE')
                               or getattr(selected, 'forward_burn', False))
                timed_burn = selected.mode == 'GUIDANCE' and decision.mode in ('TRIM_BURN', 'REBOOST')
                route_rear = (selected.mode == 'GUIDANCE' and routes.action is not None and routes.action.rear_burn
                              and bool(options.get('rear_guidance')))
                burn_end = now + selected.burn_seconds if getattr(selected, 'forward_burn', False) else math.inf
                next_plan = now + .25
            transition = full_thrust != was_full_thrust
            if transition:
                pulses.reset()
            if timed_burn and guidance.remaining_burn(now, velocity - target_velocity, gravity) <= 1e-6:
                guidance.finish_burn(now)
                timed_burn, desired = False, np.zeros(3)
            if forward_only:
                goal = relative / np.linalg.norm(relative)
                command = np.zeros(3)
                if not guidance.distance_coast and now < burn_end:
                    if now < LAUNCH_PLAN_TIME and full_thrust:
                        command = plant_accel * goal
                    elif np.any(desired) or selected.mode in ('PLAN_COAST', 'AVOID'):
                        axis = desired / np.linalg.norm(desired) if np.any(desired) else goal
                        angle = math.acos(float(np.clip(nose @ axis, -1., 1.)))
                        rate = model.avoidance_turn_rate if route_rear or selected.forward_burn else math.radians(3.)
                        nose += min(1., rate * dt / max(angle, 1e-8)) * (axis - nose)
                        nose /= np.linalg.norm(nose)
                        if np.any(desired):
                            command = rear_thrust(plant_accel * axis, nose, goal, velocity - target_velocity)
            elif route_rear:
                axis = desired / np.linalg.norm(desired)
                angle = math.acos(float(np.clip(nose @ axis, -1., 1.)))
                if now >= routes.action.start + model.avoidance_turn_delay:
                    nose += min(1., model.avoidance_turn_rate * dt / max(angle, 1e-8)) * (axis - nose)
                    nose /= np.linalg.norm(nose)
                command = min(plant_accel, model.avoidance_accel) * nose
            elif timed_burn:
                command = desired
            else:
                command = desired / np.linalg.norm(desired) * .5 if full_thrust else pulses.command(
                    desired, np.zeros(3) if transition else observed_dv, dt, now, urgent)
            was_full_thrust = full_thrust
            if not forward_only and full_thrust and not route_rear and np.linalg.norm(command) > 1e-9:
                nose = command / np.linalg.norm(command)
        thrust = command.copy()
        if not route_rear and np.linalg.norm(command) > 1e-9:
            thrust = command / np.linalg.norm(command) * plant_accel
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
    return dict(arrived=False, time=800., fuel=round(fuel, 2), collision=bool(min_gap <= 0.),
                ideal_score=None, min_true_gap=round(min_gap, 2) if rocks else None,
                cruise=model.coast_speed, route_legs=len(early_legs), first_route_s=min(early_legs, default=None),
                avoidance_plans=avoidance_plans)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Optional JSON report path')
    parser.add_argument('--sweep', action='store_true', help='Compare illustrative coast thresholds on the synthetic plant')
    arguments = parser.parse_args()
    report = dict(evidence='Synthetic point-mass, constant gravity, ideal fuel, approximate attitude; not Space Teams',
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
