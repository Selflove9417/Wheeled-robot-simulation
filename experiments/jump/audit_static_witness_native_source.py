#!/usr/bin/env python3
"""Cross-check an offline witness against direct same-step engine/ForceCmd data."""
import argparse
import csv
import json
import math
from pathlib import Path


def rows_at(path, stamp):
    result = []
    with path.open() as stream:
        for row in csv.DictReader(stream):
            value = int(row['sim_time_ns'])
            if value > stamp:
                break
            if value == stamp:
                result.append(row)
    if not result:
        raise ValueError(f'missing source step {stamp}: {path}')
    return result


def audit(record, witness):
    with witness.open() as stream:
        witness_rows = list(csv.DictReader(stream))
    stamps = {int(row['sim_ns']) for row in witness_rows}
    if len(stamps) != 1:
        raise ValueError('witness must contain one explicitly paired source step')
    stamp = stamps.pop()
    rows = [r for r in rows_at(record/'engine_state.csv', stamp) if r['phase'] == 'after_step']
    source = {r['entity_name']: r for r in rows}
    if len(rows) != 9 or len(source) != 9:
        raise ValueError('missing or duplicated direct after-step entities')
    iterations = {int(r['iteration']) for r in rows}
    if len(iterations) != 1 or any(int(r['dt_ns']) != 1_000_000 or
            int(r['physical_state_time_ns']) != stamp or
            any(r[k] != '1' for k in ('time_valid', 'position_valid', 'velocity_valid'))
            for r in rows):
        raise ValueError('invalid direct physical clock/state')
    iteration = iterations.pop()
    base = source['flat_jump_world::bbot::base_link']
    q = [float(base['position_y']), float(base['position_z']),
         2*math.atan2(float(base['quat_x']), float(base['quat_w']))] + [None]*6
    v = [float(base['linear_vy']), float(base['linear_vz']), float(base['angular_vx'])] + [None]*6
    for part, dof in zip((2, 3, 4, 5, 6, 7), (3, 4, 7, 5, 6, 8)):
        row = source[f'flat_jump_world::bbot::link_{part:03d}_joint']
        if row['joint_dof'] != '1':
            raise ValueError('unexpected native joint DOF')
        q[dof], v[dof] = float(row['joint_position_0']), float(row['joint_velocity_0'])
    commands = {}
    for row in rows_at(record/'native_wrench.csv', stamp):
        idx = int(row['joint_index'])
        if idx in commands or int(row['physics_iteration']) != iteration or \
                int(row['before_physics_sim_time_ns']) != stamp or \
                int(row['before_physics_iteration']) != iteration or \
                int(row['before_physics_dt_ns']) != 1_000_000 or \
                row['before_physics_phase'] != 'before_physics_update' or \
                row['before_physics_joint_force_cmd_component_present'] != '1' or \
                row['before_physics_joint_force_cmd_valid'] != '1':
            raise ValueError('invalid or mismatched actual ForceCmd input')
        commands[idx] = float(row['before_physics_joint_force_cmd_sim_input'])
    if set(commands) != set(range(6)):
        raise ValueError('missing six native inputs; never fill with zero')
    u = [commands[i] for i in (0, 1, 3, 4, 2, 5)]
    if not all(math.isfinite(x) for x in q+v+u):
        raise ValueError('nonfinite native state/input')
    differences = []
    for row in witness_rows:
        qw = [float(row[f'q{i}']) for i in range(9)]
        vw = [float(row[f'v{i}']) for i in range(9)]
        uw = [float(row[f'baseline_u{i}']) for i in range(4)] + [float(row[f'wheel_u{i}']) for i in range(2)]
        if not all(math.isfinite(x) for x in qw+vw+uw):
            raise ValueError('invalid witness state/input')
        differences.append([max(abs(a-b) for a, b in zip(actual, used))
                            for actual, used in ((q, qw), (v, vw), (u, uw))])
    maximum = [max(d[i] for d in differences) for i in range(3)]
    return dict(gate='PASS' if max(maximum) <= 1e-12 else 'FAIL',
                sim_ns=stamp, iteration=iteration, witness_rows=len(witness_rows),
                direct_engine_after_q=q, direct_engine_after_v=v,
                direct_before_physics_actual_input=u,
                max_q_diff=maximum[0], max_v_diff=maximum[1], max_u_diff=maximum[2],
                scope='same end-step key: direct post-state and actual before-step input; hypothetical next-step holding calculation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--witness', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = audit(args.record, args.witness)
    except (KeyError, TypeError, ValueError, OSError) as exc:
        result = dict(gate='FAIL', reason=str(exc))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return int(result['gate'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main())
