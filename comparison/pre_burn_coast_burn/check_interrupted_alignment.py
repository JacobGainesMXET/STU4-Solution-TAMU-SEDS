"""Allocator/rigid-body check with early avoidance interrupting clear thrust.

Executes only the attitude statements extracted from each controller, never its
simulator integration. Prescribed translation: clear 0-12 s, nonurgent avoidance
12-40 s, then clear. Avoidance alternates lateral firing and translation-off.
No debris/position integration: this is not a Space Teams run or a mission replay.
"""
import ast
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from test_allocator_response import AllocatorResponseTests
from test_controller_math import active_rotation


def attitude_code(directory):
    namespace = dict(np=np, math=math, npt=__import__('numpy.typing', fromlist=['NDArray']))
    names = {'LimitVector', 'MakeTargetAttitude', 'RotationVector', 'ComputeAttitudeAcceleration',
             'limited', 'unit', 'alignment_strength', 'pointing_tolerance', 'angular_command', 'AccelerationAttitude'}
    for name in ('Guidance.py', 'Controller.py'):
        tree = ast.parse((directory / name).read_text(encoding='utf-8-sig'))
        nodes = [node for node in tree.body
                 if (isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names)
                 or (name == 'Controller.py' and isinstance(node, ast.Assign)
                     and all(isinstance(target, ast.Name) and target.id.isupper() for target in node.targets)
                     and not any(isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute)
                                 and isinstance(child.func.value, ast.Name) and child.func.value.id == 'st'
                                 for child in ast.walk(node)))]
        exec(compile(ast.Module(nodes, []), str(directory / name), 'exec'), namespace)
    loop = next(node for node in tree.body if isinstance(node, ast.While)
                and any(isinstance(child, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'full_thrust'
                                                            for t in child.targets) for child in node.body))
    def assigns(node, name):
        return isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)
    start = next(i for i, node in enumerate(loop.body) if assigns(node, 'full_thrust'))
    end = next(i for i, node in enumerate(loop.body[start:], start) if assigns(node, 'urgent'))
    dispatch = next(i for i, node in enumerate(loop.body) if assigns(node, 'alpha_command'))
    block = loop.body[start:end] + loop.body[dispatch:dispatch + 2]
    return namespace, compile(ast.Module(block, []), '<extracted-attitude-path>', 'exec')


def compare():
    AllocatorResponseTests.setUpClass()
    allocator = AllocatorResponseTests()
    results = {}
    for label, directory in [('previous', ROOT / 'comparison/pre_ballistic_coast'), ('revised', ROOT)]:
        scope, code = attitude_code(directory)
        rotation = active_rotation([0., math.pi / 2., 0.])
        omega = np.zeros(3)
        scope.update(attitude_reference=scope['AccelerationAttitude'](rotation[:, 0]),
                     roll_reference=rotation[:, 1].copy(), mission_complete=False,
                     model=SimpleNamespace(planning_accel=.12), quiet_coast=False,
                     alignment_engaged=True, angular_limit=.05, distance=1300., dt=.2,
                     forward_avoidance=False,
                     goal=np.array([1., 0., 0.]), plan=SimpleNamespace(
                         urgent=False, baseline=SimpleNamespace(time=15.), encounter=SimpleNamespace(time=math.inf)))
        first_coarse, peak = None, 0.
        for index in range(450):
            time = index * .2
            clear = not 12. <= time < 40.
            linear_world = np.array([.5, 0., 0.]) if clear else np.array([0., .09 if int(time) % 2 == 0 else 0., 0.])
            scope.update(mode='ACCELERATE' if clear else 'AVOID', command=np.array([.12, 0., 0.]) if clear else linear_world,
                         current_axes=rotation.T, omega_body=omega, linear_world=linear_world)
            exec(code, scope)
            actual, _ = allocator.allocate(rotation.T @ linear_world, scope['alpha_command'])
            gyro = np.linalg.solve(allocator.inertia, np.cross(omega, allocator.inertia @ omega))
            omega += (actual - gyro) * .2
            rotation = rotation @ active_rotation(omega * .2)
            error = math.degrees(math.acos(float(np.clip(rotation[0, 0], -1., 1.))))
            peak = max(peak, math.degrees(float(np.linalg.norm(omega))))
            if error < 12. and first_coarse is None:
                first_coarse = time
        results[label] = dict(first_within_12_deg_s=first_coarse, peak_spin_deg_s=peak, final_nose_deg=error)
    return dict(evidence=__doc__, results=results)


if __name__ == '__main__':
    output = json.dumps(compare(), indent=2)
    print(output)
    if '--save' in sys.argv:
        (ROOT / 'comparison/interrupted_alignment_check.json').write_text(output + '\n', encoding='utf-8')
