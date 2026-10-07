#!/usr/bin/env python3
"""Offline same-step response check for a fixed ground-standing window.

The bridge is the existing audit_native_command_response.cpp protocol. Model
parameters and controller gains are never fit or changed by this script.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

STEP_NS = 1_000_000
# Native joint indices are HL, KL, wheelL, HR, KR, wheelR. The model vector
# order is base forward/world-Y, base-Z, roll-X, HL, KL, HR, KR, wheelL, wheelR.
MODEL_ORDER = (0, 1, 3, 4, 2, 5)
DOF_NAMES = ('base_forward', 'base_z', 'roll_x', 'hip_left', 'knee_left',
             'hip_right', 'knee_right', 'wheel_left', 'wheel_right')
ACTUATOR_NAMES = {0: 'hip_left', 1: 'knee_left', 2: 'wheel_left',
                  3: 'hip_right', 4: 'knee_right', 5: 'wheel_right'}
GROUND_TOKEN = 'ground_plane::link::collision'
WHEEL_TOKENS = ('link_004::link_004_collision_collision',
                'link_007::link_007_collision_collision')


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f'{path}: CSV header missing')
        return list(reader)


def first(row: dict[str, Any], *keys: str, default: Any = '') -> Any:
    for key in keys:
        if key in row and row[key] not in (None, ''):
            return row[key]
    return default


def integer(value: Any) -> int | None:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        try:
            x = float(value)
            return int(x) if math.isfinite(x) and x.is_integer() else None
        except (TypeError, ValueError):
            return None


def finite(value: Any) -> float | None:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def truth(row: dict[str, Any], *keys: str) -> bool:
    return str(first(row, *keys, default='')).lower() in ('1', 'true', 'yes', 'valid', 'ok')


def key_of(row: dict[str, Any]) -> tuple[int, int] | None:
    it = integer(first(row, 'physics_iteration', 'iteration', 'physics_iter'))
    ns = integer(first(row, 'sim_time_ns', 'simulation_time_ns', 'stamp_ns'))
    return (it, ns) if it is not None and ns is not None else None


def group_native(rows: list[dict[str, str]]) -> tuple[dict[tuple[int, int], dict[int, dict[str, str]]], Counter]:
    frames: dict[tuple[int, int], dict[int, dict[str, str]]] = defaultdict(dict)
    problems: Counter = Counter()
    for row in rows:
        key = key_of(row)
        idx = integer(first(row, 'joint_index', 'actuator_index', 'index'))
        if key is None or idx is None:
            problems['unkeyed_native_rows'] += 1
            continue
        if idx in frames[key]:
            problems['duplicate_joint_rows'] += 1
            continue
        frames[key][idx] = row
    return dict(frames), problems


def index_frames(rows: list[dict[str, str]], label: str) -> tuple[dict[tuple[int, int], dict[str, str]], Counter]:
    indexed: dict[tuple[int, int], dict[str, str]] = {}
    problems: Counter = Counter()
    for row in rows:
        key = key_of(row)
        if key is None:
            problems[f'unkeyed_{label}_rows'] += 1
        elif key in indexed:
            problems[f'duplicate_{label}_frames'] += 1
        else:
            indexed[key] = row
    return indexed, problems


def clock_report(rows: list[dict[str, str]], grouped_native: bool = False) -> dict[str, int]:
    seq = []
    seen = set()
    for row in rows:
        key = key_of(row)
        if key is None:
            continue
        if grouped_native and key in seen:
            continue
        if grouped_native:
            seen.add(key)
        seq.append((key[0], key[1], integer(first(row, 'dt_ns', 'physics_dt_ns'))))
    result = {'time_reversals': 0, 'iteration_reversals': 0, 'time_step_errors': 0,
              'iteration_step_errors': 0, 'dt_errors': 0}
    for i, (it, ns, dt) in enumerate(seq):
        if dt != STEP_NS:
            result['dt_errors'] += 1
        if i:
            pit, pns, _ = seq[i-1]
            result['time_reversals'] += int(ns < pns)
            result['iteration_reversals'] += int(it < pit)
            result['time_step_errors'] += int(ns - pns != STEP_NS)
            result['iteration_step_errors'] += int(it - pit != 1)
    return result


def legal_contact(row: dict[str, str] | None) -> tuple[bool, int | None, str]:
    if row is None:
        return False, None, 'missing_contact_frame'
    count = integer(first(row, 'num_contacts', 'contact_count'))
    if count is None or count < 0 or not truth(row, 'frame_valid', 'valid') or first(row, 'error', default=''):
        return False, count, 'invalid_contact_frame'
    text = first(row, 'collision_pairs_json', 'pairs_json', 'contacts_json')
    if text == '':
        return False, count, 'missing_collision_pairs'
    try:
        pairs = json.loads(text)
    except (TypeError, ValueError):
        return False, count, 'malformed_collision_pairs'
    if not isinstance(pairs, list) or len(pairs) != count:
        return False, count, 'contact_count_payload_mismatch'
    wheels = set()
    for pair in pairs:
        if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(x, str) for x in pair):
            return False, count, 'malformed_contact_pair'
        if not all('flat_jump_world::' in name for name in pair):
            return False, count, 'unexpected_world_scope'
        if not any(GROUND_TOKEN in name for name in pair):
            return False, count, 'non_ground_contact'
        robot_names = [name for name in pair if GROUND_TOKEN not in name]
        if len(robot_names) != 1:
            return False, count, 'unknown_collision_pair'
        matches = [token for token in WHEEL_TOKENS if token in robot_names[0]]
        if len(matches) != 1:
            return False, count, 'non_wheel_ground_contact'
        wheels.add(matches[0])
    if count == 2 and len(wheels) != 2:
        return False, count, 'bilateral_wheels_not_both_contacting'
    if count > 0 and count not in (1, 2):
        return False, count, 'unexpected_contact_count'
    return True, count, 'ok'


def model_state(joints: dict[int, dict[str, str]]) -> tuple[list[float] | None, list[float] | None, list[float] | None, list[float] | None, list[float | None] | None, str | None]:
    base = joints.get(0)
    if base is None:
        return None, None, None, None, None, 'missing_native_base_record_index0'
    if not truth(base, 'before_physics_base_pose_valid') or not truth(base, 'before_physics_base_velocity_valid') or not truth(base, 'post_base_velocity_valid'):
        return None, None, None, None, None, 'native_base_pose_or_velocity_invalid'
    if str(first(base, 'before_physics_phase', default='')).lower() != 'before_physics_update':
        return None, None, None, None, None, 'wrong_before_physics_phase'
    qx = finite(base.get('before_physics_base_qx'))
    qw = finite(base.get('before_physics_base_qw'))
    base_y = finite(base.get('before_physics_base_y'))
    base_z = finite(base.get('before_physics_base_z'))
    bv_y = finite(base.get('before_physics_base_world_vy'))
    bv_z = finite(base.get('before_physics_base_world_vz'))
    bw_x = finite(base.get('before_physics_base_world_wx'))
    post_by = finite(base.get('post_base_world_vy'))
    post_bz = finite(base.get('post_base_world_vz'))
    post_bw = finite(base.get('post_base_world_wx'))
    if any(x is None for x in (qx, qw, base_y, base_z, bv_y, bv_z, bw_x, post_by, post_bz, post_bw)):
        return None, None, None, None, None, 'nonfinite_native_base_state'
    q = [base_y, base_z, 2.0 * math.atan2(qx, qw)]
    v = [bv_y, bv_z, bw_x]
    post = [post_by, post_bz, post_bw]
    cmd_model, net_model = [], []
    qj, vj, postj = [], [], []
    for idx in MODEL_ORDER:
        row = joints.get(idx)
        if row is None:
            return None, None, None, None, None, f'missing_joint_index_{idx}'
        if not (truth(row, 'before_physics_joint_state_valid') and truth(row, 'state_valid') and
                truth(row, 'before_physics_joint_force_cmd_component_present') and truth(row, 'before_physics_joint_force_cmd_valid')):
            return None, None, None, None, None, f'invalid_state_or_input_index_{idx}'
        before_ns = integer(first(row, 'before_physics_sim_time_ns'))
        before_it = integer(first(row, 'before_physics_iteration'))
        before_dt = integer(first(row, 'before_physics_dt_ns'))
        main_ns = integer(first(row, 'sim_time_ns'))
        main_it = integer(first(row, 'physics_iteration'))
        if (before_ns, before_it, before_dt) != (main_ns, main_it, STEP_NS):
            return None, None, None, None, None, f'before_physics_timing_mismatch_index_{idx}'
        q0 = finite(row.get('before_physics_joint_position'))
        v0 = finite(row.get('before_physics_joint_velocity'))
        q1 = finite(row.get('joint_position'))
        v1 = finite(row.get('joint_velocity'))
        u = finite(row.get('before_physics_joint_force_cmd_sim_input'))
        net = finite(row.get('transmitted_axis_torque')) if truth(row, 'wrench_valid') else None
        if any(x is None for x in (q0, v0, q1, v1, u)):
            return None, None, None, None, None, f'nonfinite_native_state_or_input_index_{idx}'
        if net is None:
            return None, None, None, None, None, f'invalid_or_nonfinite_transmitted_wrench_diagnostic_index_{idx}'
        qj.append(q0); vj.append(v0); postj.append(v1); cmd_model.append(u); net_model.append(net)
    q.extend(qj); v.extend(vj); post.extend(postj)
    if len(q) != 9 or len(v) != 9 or len(post) != 9 or len(cmd_model) != 6:
        return None, None, None, None, None, 'internal_model_vector_size_error'
    # The net wrench stays in its independent bridge diagnostic vector; the
    # six before-Physics command values alone form the model input RHS.
    return q, v, post, cmd_model, net_model, None


def stats(values: list[float]) -> dict[str, Any]:
    vals = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    if not vals:
        return {'count': 0, 'RMS': None, 'max_abs': None}
    return {'count': len(vals), 'RMS': math.sqrt(sum(x*x for x in vals) / len(vals)),
            'max_abs': max(abs(x) for x in vals)}


def split_gate(steps: list[dict[str, Any]], gate: dict[str, Any], label: str) -> dict[str, Any]:
    invalid_steps = [s for s in steps if not s.get('prediction_valid', True)]
    limits = gate.get('thrust_acceleration_RMS_limits')
    rel_limit = float(gate.get('thrust_acceleration_relative_RMS_max', 0.2))
    minimum = max(30, int(gate.get('minimum_thrust_steps', 30)))
    result: dict[str, Any] = {'label': label, 'steps': len(steps), 'minimum_steps': minimum,
                              'invalid_prediction_steps': len(invalid_steps), 'degrees_of_freedom': {}, 'equation_residual': stats([s['equation_residual'] for s in steps]),
                              'contact_constraint_residuals': {}, 'contact_forces': {},
                              'aggregate_joint_load_difference': {}, 'gate': 'FAIL', 'failures': []}
    if not isinstance(limits, list) or len(limits) != 9:
        result['failures'].append('response_gate_must_supply_nine_absolute_RMS_limits')
        return result
    if invalid_steps:
        result['failures'].append('prediction_rejected_in_original_support_steps')
    if len(steps) < minimum:
        result['failures'].append('insufficient_steps_for_independent_split_gate')
    for j, dof in enumerate(DOF_NAMES):
        residual = [s['residual'][j] for s in steps]
        observed = [s['observed'][j] for s in steps]
        err_rms = stats(residual)['RMS']
        obs_rms = stats(observed)['RMS']
        relative = err_rms / max(obs_rms or 0.0, 1.0) if err_rms is not None else None
        abs_limit = float(limits[j])
        passed = (not invalid_steps and len(steps) >= minimum and err_rms is not None and err_rms <= abs_limit and
                  relative is not None and relative <= rel_limit)
        result['degrees_of_freedom'][dof] = {
            'absolute_error_RMS': err_rms, 'absolute_RMS_limit': abs_limit,
            'observed_acceleration_RMS': obs_rms, 'relative_RMS': relative,
            'relative_RMS_limit': rel_limit, 'relative_denominator': 'max(observed_RMS, 1)',
            'gate': 'PASS' if passed else 'FAIL'}
        if not passed:
            result['failures'].append(f'{dof}_absolute_or_relative_RMS_failed')
    for field, result_key in (('contact_force', 'contact_forces'),
                              ('contact_residual', 'contact_constraint_residuals'),
                              ('aggregate_difference', 'aggregate_joint_load_difference')):
        width = 4 if field != 'aggregate_difference' else 6
        result[result_key] = {str(i): stats([s[field][i] for s in steps if s[field][i] is not None])
                              for i in range(width)}
    if not invalid_steps and len(steps) >= minimum and all(x['gate'] == 'PASS' for x in result['degrees_of_freedom'].values()):
        result['gate'] = 'PASS'
    return result


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    output: Path = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    if args.hold_start_ns <= 0 or args.hold_end_ns <= args.hold_start_ns:
        window_established = False
    else:
        window_established = True
    gate = json.loads(args.response_gate_json.read_text())
    native_rows = read_rows(args.native_wrench_csv)
    contact_rows = read_rows(args.contact_frames_csv)
    geometry_rows = read_rows(args.geometry_csv)
    native, native_problems = group_native(native_rows)
    contacts, contact_problems = index_frames(contact_rows, 'contact')
    geometry, geometry_problems = index_frames(geometry_rows, 'geometry')
    clock = {'native': clock_report(native_rows, grouped_native=True),
             'contacts': clock_report(contact_rows), 'geometry': clock_report(geometry_rows)}
    expected_ns = list(range(args.hold_start_ns, args.hold_end_ns, STEP_NS)) if window_established else []
    midpoint_ns = args.hold_start_ns + (args.hold_end_ns - args.hold_start_ns) // 2 if window_established else None
    time_to_key: dict[int, tuple[int, int]] = {}
    for key in sorted(native):
        if key[1] in time_to_key:
            native_problems['duplicate_native_timestamps'] += 1
        time_to_key.setdefault(key[1], key)

    frame_audit = []
    candidates = []
    reasons = Counter()
    contact_transitions = []
    contact_count_by_ns: dict[int, tuple[int | None, bool]] = {}
    all_window_integrity_ok = window_established and len(expected_ns) > 0
    for ns in expected_ns:
        key = time_to_key.get(ns)
        joints = native.get(key, {}) if key else {}
        cframe = contacts.get(key) if key else None
        gframe = geometry.get(key) if key else None
        row_reasons = []
        dt = integer(first(joints.get(0, {}), 'dt_ns')) if 0 in joints else None
        if key is None:
            row_reasons.append('missing_native_frame')
        if key and len(joints) != 6:
            row_reasons.append('six_joint_rows_missing_or_extra')
        if dt != STEP_NS:
            row_reasons.append('native_dt_not_1ms')
        if key is None or key not in contacts:
            row_reasons.append('missing_contact_same_physics_key')
        if key is None or key not in geometry:
            row_reasons.append('missing_geometry_same_physics_key')
        contact_valid, ncontact, contact_reason = legal_contact(cframe)
        if not contact_valid:
            row_reasons.append(contact_reason)
        if not gframe or not truth(gframe, 'frame_valid') or first(gframe, 'error', default=''):
            row_reasons.append('invalid_geometry_frame')
        q = v = post = commands = None
        state_reason = 'native_frame_missing_or_incomplete'
        net_values = None
        if key and len(joints) == 6:
            q, v, post, commands, net_values, state_reason = model_state(joints)
            if state_reason:
                row_reasons.append(state_reason)
            # All native force inputs must be finite and marked valid. Model
            # aggregate wrench values remain separate diagnostics only.
        if row_reasons:
            all_window_integrity_ok = False
        for reason in row_reasons:
            reasons[reason] += 1
        audit_record = {'physics_iteration': key[0] if key else None, 'sim_time_ns': ns,
                            'dt_ns': dt, 'native_frame_present': int(key is not None),
                            'native_joint_rows': len(joints), 'contact_frame_present': int(key in contacts if key else 0),
                            'geometry_frame_present': int(key in geometry if key else 0),
                            'contact_valid': int(contact_valid), 'contact_count': ncontact,
                            'contact_reason': contact_reason, 'frame_integrity_ok': int(not row_reasons),
                            'integrity_failures': '|'.join(row_reasons), 'model_state_valid': int(state_reason is None),
                            'contact_transition': 0, 'support_eligible': 0, 'split': 'diagnostic_record' if midpoint_ns is not None and ns < midpoint_ns else 'independent_validation' if midpoint_ns is not None else 'unset'}
        if q is not None and v is not None and post is not None:
            for j, dof in enumerate(DOF_NAMES):
                audit_record[f'{dof}_q_before'] = q[j]
                audit_record[f'{dof}_v_before'] = v[j]
                audit_record[f'{dof}_v_post'] = post[j]
        for idx, name in ACTUATOR_NAMES.items():
            jr = joints.get(idx, {})
            audit_record[f'{name}_before_q_raw'] = first(jr, 'before_physics_joint_position', default='')
            audit_record[f'{name}_before_v_raw'] = first(jr, 'before_physics_joint_velocity', default='')
            audit_record[f'{name}_post_q_raw'] = first(jr, 'joint_position', default='')
            audit_record[f'{name}_post_v_raw'] = first(jr, 'joint_velocity', default='')
            audit_record[f'{name}_before_physics_input_raw'] = first(jr, 'before_physics_joint_force_cmd_sim_input', default='')
            audit_record[f'{name}_transmitted_wrench_raw'] = first(jr, 'transmitted_axis_torque', default='')
        if commands is not None:
            for j, idx in enumerate(MODEL_ORDER):
                audit_record[f'{ACTUATOR_NAMES[idx]}_before_physics_input'] = commands[j]
        frame_audit.append(audit_record)
        contact_count_by_ns[ns] = (ncontact, contact_valid)
        if not row_reasons and key is not None and q is not None and v is not None and post is not None and commands is not None:
            candidates.append({'key': key, 'ns': ns, 'q': q, 'v': v, 'post': post,
                               'commands': commands, 'net': net_values,
                               'contact_count': ncontact, 'split': 'diagnostic_record' if ns < midpoint_ns else 'independent_validation'})

    # Record contact transitions independently. Include only adjacent physical
    # frame changes; missing frames are already a hard integrity failure.
    for i, ns in enumerate(expected_ns):
        current = contact_count_by_ns.get(ns)
        prev = contact_count_by_ns.get(ns - STEP_NS)
        if i > 0 and current and prev and current[0] != prev[0]:
            event = {'sim_time_ns': ns, 'from_contact_count': prev[0], 'to_contact_count': current[0]}
            contact_transitions.append(event)
            frame_audit[i]['contact_transition'] = 1
    # Only use rows surrounded by complete legal bilateral contact on the same
    # 1 ms clock. This context rule is fixed before seeing prediction errors.
    by_ns = {r['ns']: r for r in candidates}
    support = []
    def contact_at_physics_time(ns: int) -> tuple[int | None, bool] | None:
        key = time_to_key.get(ns)
        if key is None or key not in contacts:
            return None
        ok, count, _ = legal_contact(contacts[key])
        return (count, ok)
    for item in candidates:
        ns = item['ns']
        prev = contact_at_physics_time(ns - STEP_NS)
        cur = contact_at_physics_time(ns)
        nxt = contact_at_physics_time(ns + STEP_NS)
        if (cur == (2, True) and prev == (2, True) and nxt == (2, True)):
            support.append(item)
            frame_idx = (ns - args.hold_start_ns) // STEP_NS
            frame_audit[frame_idx]['support_eligible'] = 1

    # Assign contiguous support-segment IDs; any gap/contact conversion starts
    # a new segment, and segment IDs are retained in every response row.
    segment_id = -1
    last_ns = None
    for item in support:
        if last_ns is None or item['ns'] != last_ns + STEP_NS:
            segment_id += 1
        item['support_segment'] = segment_id
        last_ns = item['ns']

    bridge_inputs = []
    for item in support:
        observed = [(item['post'][i] - item['v'][i]) / STEP_NS * 1e9 for i in range(9)]
        item['observed'] = observed
        # Bridge protocol: t bilateral q9 v9 observed-qdd9 cmd6 aggregate-wrench6.
        # The final aggregate vector is used only for its side diagnostic output.
        line = [item['ns'] * 1e-9, 1, *item['q'], *item['v'], *observed, *item['commands']]
        if any(x is None for x in item['net']):
            raise ValueError('missing aggregate diagnostic input must remain invalid, never zero-filled')
        line.extend(item['net'])
        bridge_inputs.append(' '.join(f'{x:.17g}' for x in line))
    (output / 'bridge_input.txt').write_text('\n'.join(bridge_inputs) + ('\n' if bridge_inputs else ''), encoding='utf-8')
    bridge_error = None
    output_rows = []
    if bridge_inputs:
        try:
            proc = subprocess.run([str(args.cpp_bridge.resolve())], input='\n'.join(bridge_inputs) + '\n',
                                  capture_output=True, text=True, check=False)
            (output / 'bridge_output.txt').write_text(proc.stdout, encoding='utf-8')
            (output / 'bridge_stderr.txt').write_text(proc.stderr, encoding='utf-8')
            if proc.returncode:
                bridge_error = f'bridge exit {proc.returncode}: {proc.stderr}'
            lines = [line for line in proc.stdout.splitlines() if line.strip()]
            if len(lines) != len(support):
                raise ValueError(f'bridge output rows {len(lines)} != input rows {len(support)}')
            for item, line in zip(support, lines):
                vals = [float(x) for x in line.split()]
                if len(vals) != 35:
                    raise ValueError(f'bridge output must have 35 columns, got {len(vals)}')
                if not all(math.isfinite(x) for x in vals[:2] + vals[12:21]):
                    raise ValueError('bridge identity or observed echo is nonfinite')
                if abs(vals[0] - item['ns'] * 1e-9) > 1e-12 or vals[1] != 1.0:
                    raise ValueError('bridge output time/contact identity does not match submitted bilateral frame')
                pred, observed = vals[3:12], vals[12:21]
                if any(abs(a-b) > 1e-12 for a, b in zip(observed, item['observed'])):
                    raise ValueError('bridge observed-acceleration echo does not match submitted same-step delta-v')
                prediction_valid = vals[2] >= 0 and all(math.isfinite(x) for x in vals)
                if not prediction_valid:
                    vals = [x if math.isfinite(x) else None for x in vals]
                    pred = vals[3:12]
                residual = [p - o if p is not None else None for p, o in zip(pred, observed)]
                output_rows.append({**item, 'prediction_valid': prediction_valid, 'equation_residual': vals[2] if prediction_valid else None, 'predicted': pred,
                                    'observed_bridge': observed, 'residual': residual,
                                    'contact_force': vals[21:25], 'contact_residual': vals[25:29],
                                    'aggregate_difference': [vals[29+i] if item['net'][i] is not None else None for i in range(6)]})
        except (OSError, subprocess.CalledProcessError, ValueError) as exc:
            bridge_error = str(exc)
            if isinstance(exc, subprocess.CalledProcessError):
                bridge_error += '\n' + exc.stderr
            (output / 'bridge_error.txt').write_text(bridge_error, encoding='utf-8')

    continuity_diffs = []
    complete_by_ns = {r['ns']: r for r in candidates}
    for item in candidates:
        nxt_item = complete_by_ns.get(item['ns'] + STEP_NS)
        if nxt_item is not None:
            continuity_diffs.extend(abs(a-b) for a,b in zip(nxt_item['v'], item['post']))
    continuity_limit = float(gate.get('same_step_velocity_continuity_max', 1e-9))
    continuity_ok = bool(continuity_diffs) and max(continuity_diffs) <= continuity_limit
    record_steps = [r for r in output_rows if r['split'] == 'diagnostic_record']
    validation_steps = [r for r in output_rows if r['split'] == 'independent_validation']
    record_gate = split_gate(record_steps, gate, 'first_half_diagnostic_record')
    validation_gate = split_gate(validation_steps, gate, 'second_half_independent_validation')
    clock_ok = all(v == 0 for group in clock.values() for v in group.values())
    structural_ok = window_established and all_window_integrity_ok and not native_problems and not contact_problems and not geometry_problems and clock_ok
    failures = []
    if not window_established: failures.append('window_not_established')
    if not all_window_integrity_ok: failures.append('original_window_frame_integrity_failed')
    if native_problems: failures.append('native_duplicate_or_unkeyed_rows')
    if contact_problems: failures.append('contact_duplicate_or_unkeyed_frames')
    if geometry_problems: failures.append('geometry_duplicate_or_unkeyed_frames')
    if not clock_ok: failures.append('source_clock_reversal_or_step_error')
    if len(record_steps) < 30: failures.append('fewer_than_30_supported_first_half_steps')
    if len(validation_steps) < 30: failures.append('fewer_than_30_supported_second_half_steps')
    if bridge_error: failures.append('cpp_bridge_failed')
    if not continuity_ok: failures.append('before_to_next_before_velocity_continuity_failed_or_missing')
    if record_gate['gate'] != 'PASS': failures.extend('diagnostic:' + x for x in record_gate['failures'])
    if validation_gate['gate'] != 'PASS': failures.extend('validation:' + x for x in validation_gate['failures'])

    frame_by_ns = {r['sim_time_ns']: r for r in frame_audit}
    response_csv = []
    for step in output_rows:
        rec = {'physics_iteration': step['key'][0], 'sim_time_ns': step['ns'],
               'dt_ns': STEP_NS, 'contact_count': step['contact_count'],
               'split': step['split'], 'support_segment': step['support_segment'],
               'equation_residual': step['equation_residual'], 'prediction_valid': int(step['prediction_valid'])}
        for j, name in enumerate(DOF_NAMES):
            rec[f'{name}_q_before'] = step['q'][j]
            rec[f'{name}_v_before'] = step['v'][j]
            rec[f'{name}_qdd_predicted'] = step['predicted'][j]
            rec[f'{name}_qdd_observed'] = step['observed'][j]
            rec[f'{name}_qdd_residual'] = step['residual'][j]
        for j, idx in enumerate(MODEL_ORDER):
            rec[f'{ACTUATOR_NAMES[idx]}_before_physics_input'] = step['commands'][j]
            rec[f'{ACTUATOR_NAMES[idx]}_transmitted_wrench_axis_diagnostic'] = step['net'][j]
        for j in range(4):
            rec[f'contact_force_{j}'] = step['contact_force'][j]
            rec[f'contact_constraint_residual_{j}'] = step['contact_residual'][j]
        for j in range(6):
            rec[f'command_minus_damping_minus_transmitted_{j}'] = step['aggregate_difference'][j]
        response_csv.append(rec)
    write_dict_csv(output / 'window_frames.csv', frame_audit)
    write_dict_csv(output / 'response_steps.csv', response_csv)
    support_segments = Counter(s['support_segment'] for s in support)
    result = {'audit': 'ground_model_response_stage2',
              'gate': 'PASS' if not failures else 'FAIL',
              'scope': 'fixed-model, one stationary ground bilateral-support operating condition only',
              'window': {'established': window_established,
                         'start_ns': args.hold_start_ns if window_established else None,
                         'end_ns_exclusive': args.hold_end_ns if window_established else None,
                         'duration_s': (args.hold_end_ns-args.hold_start_ns)*1e-9 if window_established else 0,
                         'midpoint_ns': midpoint_ns, 'all_expected_original_frames': len(expected_ns),
                         'window_full_frame_integrity_gate': 'PASS' if all_window_integrity_ok else 'FAIL'},
              'input_files': {'native_wrench_csv': str(args.native_wrench_csv),
                              'contact_frames_csv': str(args.contact_frames_csv),
                              'geometry_csv': str(args.geometry_csv),
                              'cpp_bridge': str(args.cpp_bridge),
                              'response_gate_json': str(args.response_gate_json)},
              'clock': clock, 'row_problems': {'native': dict(native_problems),
                                                'contacts': dict(contact_problems), 'geometry': dict(geometry_problems)},
              'frame_integrity_failures': dict(reasons),
              'contact': {'transition_count': len(contact_transitions), 'transitions': contact_transitions,
                          'supported_steps_after_neighbor_contact_rule': len(support),
                          'support_segments': {str(k): v for k, v in support_segments.items()},
                          'support_rule': 'current and immediate previous/next 1 ms frames must each have valid legal bilateral wheel-ground contact'},
              'split_gates': {'first_half_diagnostic_record': record_gate,
                              'second_half_independent_validation': validation_gate},
              'model_semantics': {'model_is_immutable': True,
                                  'first_half_is_diagnostic_only_no_model_parameters_fitted': True,
                                  'second_half_is_independent_of_first_half_errors': True,
                                  'q_order': list(DOF_NAMES), 'native_joint_index_model_order': list(MODEL_ORDER),
                                  'base_coordinates': 'world-Y forward, world-Z vertical, positive roll about world-X',
                                  'command_input': 'six before-Physics JointForceCmd values only; net wrench and observed wheel acceleration never enter acceleration prediction',
                                  'net_wrench': 'required finite CPP protocol side-diagnostic operand for command-minus-damping-minus-wrench output; it is not used in the RHS/mass acceleration solve and never replaces JointForceCmd',
                                  'observed_acceleration': 'same physical step (post velocity - before-Physics velocity) / 1 ms',
                                  'relative_RMS': 'absolute error RMS / max(observed acceleration RMS, 1)',
                                  'gate_applies_separately_to_each_split': True},
              'continuity': {'same_step_post_to_next_before_pairs': len(continuity_diffs),
                            'max_abs_velocity_difference': max(continuity_diffs) if continuity_diffs else None,
                            'limit': continuity_limit, 'gate': 'PASS' if continuity_ok else 'FAIL'},
              'diagnostics': {'bridge_error': bridge_error,
                              'all_supported_step_rows_saved': len(output_rows),
                              'invalid_prediction_steps': sum(not s['prediction_valid'] for s in output_rows),
                              'response_steps_csv': 'response_steps.csv',
                              'window_frames_csv': 'window_frames.csv',
                              'bridge_input_txt': 'bridge_input.txt',
                              'equation_residual_all': stats([s['equation_residual'] for s in output_rows]),
                              'contact_constraint_residual_all': {str(i): stats([s['contact_residual'][i] for s in output_rows]) for i in range(4)},
                              'contact_force_all': {str(i): stats([s['contact_force'][i] for s in output_rows]) for i in range(4)},
                              'aggregate_joint_load_difference_all': {str(i): stats([s['aggregate_difference'][i] for s in output_rows]) for i in range(6)}},
              'not_certified': ['all-speed response envelope', 'other configurations or leg postures',
                                'braking/deceleration envelope', 'flight or landing response', 'controller gain acceptance'],
              'failures': failures}
    (output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return result


def write_dict_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = list(dict.fromkeys(key for row in rows for key in row)) if rows else ['physics_iteration', 'sim_time_ns']
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--native-wrench-csv', type=Path, required=True)
    p.add_argument('--contact-frames-csv', type=Path, required=True)
    p.add_argument('--geometry-csv', type=Path, required=True)
    p.add_argument('--hold-start-ns', type=int, required=True)
    p.add_argument('--hold-end-ns', type=int, required=True)
    p.add_argument('--cpp-bridge', type=Path, required=True)
    p.add_argument('--response-gate-json', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    args = p.parse_args()
    try:
        result = run_audit(args)
    except (OSError, ValueError, csv.Error, json.JSONDecodeError) as exc:
        p.error(str(exc))
    print(json.dumps({'gate': result['gate'], 'failures': result['failures'],
                      'split_gates': {k: v['gate'] for k, v in result['split_gates'].items()},
                      'supported_steps': result['contact']['supported_steps_after_neighbor_contact_rule']}, indent=2))
    return 0 if result['gate'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
