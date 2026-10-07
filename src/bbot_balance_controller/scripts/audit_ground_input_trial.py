#!/usr/bin/env python3
"""Offline audit for the allocator-off, bounded-P-wheel ground input trial.

This checks only the stage-one physical input and standing window. It does not
certify a model response, braking envelope, or jump.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

STEP_NS = 1_000_000
CONTROLLER_MAX_AGE_NS = 10_000_000
ACTUATORS = {0: 'hip_left', 1: 'knee_left', 2: 'wheel_left',
             3: 'hip_right', 4: 'knee_right', 5: 'wheel_right'}
LEGS = {0: 'hip_left', 1: 'knee_left', 3: 'hip_right', 4: 'knee_right'}
WHEELS = {2: 'wheel_left', 5: 'wheel_right'}
POSITION_LIMITS = {0: 1.52, 1: 1.5708, 3: 1.52, 4: 1.5708}
EFFORT_LIMITS = {'hip_left': 75.0, 'knee_left': 60.0, 'hip_right': 75.0, 'knee_right': 60.0,
                 'wheel_left': 10.0, 'wheel_right': 10.0}
WHEEL_RATE_LIMIT = 30.0


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f'{path}: CSV header missing')
        return list(reader)


def first(row: dict[str, Any], *names: str, default: Any = '') -> Any:
    for name in names:
        if name in row and row[name] not in (None, ''):
            return row[name]
    return default


def number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def integer(value: Any) -> int | None:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        try:
            f = float(value)
            return int(f) if math.isfinite(f) and f.is_integer() else None
        except (TypeError, ValueError):
            return None


def flag(row: dict[str, Any], *names: str) -> bool:
    value = str(first(row, *names, default='')).strip().lower()
    return value in ('1', 'true', 'yes', 'valid', 'ok')


def frame_key(row: dict[str, Any]) -> tuple[int, int] | None:
    it = integer(first(row, 'physics_iteration', 'iteration', 'physics_iter'))
    ns = integer(first(row, 'sim_time_ns', 'simulation_time_ns', 'stamp_ns'))
    return (it, ns) if it is not None and ns is not None else None


def unique_index(rows: list[dict[str, str]], label: str) -> tuple[dict[tuple[int, int], dict[str, str]], Counter]:
    result: dict[tuple[int, int], dict[str, str]] = {}
    duplicates: Counter = Counter()
    for row in rows:
        key = frame_key(row)
        if key is None:
            duplicates['unkeyed'] += 1
            continue
        if key in result:
            duplicates['duplicate'] += 1
            continue
        result[key] = row
    if duplicates.get('duplicate'):
        raise ValueError(f'{label}: duplicate physical frame keys')
    return result, duplicates


def native_frames(path: Path) -> tuple[dict[tuple[int, int], dict[int, dict[str, str]]], Counter]:
    result: dict[tuple[int, int], dict[int, dict[str, str]]] = defaultdict(dict)
    diagnostics: Counter = Counter()
    for row in read_csv(path):
        key = frame_key(row)
        idx = integer(first(row, 'joint_index', 'actuator_index', 'index'))
        if key is None or idx is None:
            diagnostics['unkeyed_native_rows'] += 1
            continue
        if idx in result[key]:
            diagnostics['duplicate_joint_rows'] += 1
            continue
        result[key][idx] = row
    return dict(result), diagnostics


def pitch_from_quaternion(row: dict[str, str]) -> float | None:
    qx = number(first(row, 'before_physics_base_qx', 'base_qx', 'qx'))
    qw = number(first(row, 'before_physics_base_qw', 'base_qw', 'qw'))
    if qx is None or qw is None:
        return None
    # This model's pitch axis is world X; retain this raw model pose separately
    # from the controller IMU pitch, which includes its startup offset.
    return 2.0 * math.atan2(qx, qw)


def check_contact(row: dict[str, str]) -> tuple[bool, int | None, str]:
    valid = flag(row, 'frame_valid', 'valid', 'contact_frame_valid')
    err = first(row, 'error', 'error_message', default='')
    count = integer(first(row, 'num_contacts', 'contact_count', 'contacts_count'))
    if not valid or err or count is None or count < 0:
        return False, count, 'invalid_frame'
    pairs_text = first(row, 'collision_pairs_json', 'pairs_json', 'contacts_json')
    if pairs_text:
        try:
            pairs = json.loads(pairs_text)
        except (ValueError, TypeError):
            return False, count, 'malformed_pairs'
        if not isinstance(pairs, list) or len(pairs) != count:
            return False, count, 'count_mismatch'
        # Stage one permits only ground contact by either wheel, and no other
        # collision. Names vary by world prefix, so match exact link/collision IDs.
        allowed = ('link_004::link_004_collision_collision',
                   'link_007::link_007_collision_collision')
        contacted_wheels = set()
        for pair in pairs:
            if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(x, str) for x in pair):
                return False, count, 'malformed_pair'
            if not any('ground_plane::link::collision' in x for x in pair):
                return False, count, 'non_ground_contact'
            robot = [x for x in pair if 'ground_plane::link::collision' not in x]
            if len(robot) != 1 or not any(tag in robot[0] for tag in allowed):
                return False, count, 'illegal_contact'
            contacted_wheels.add(next(tag for tag in allowed if tag in robot[0]))
        if count == 2 and len(contacted_wheels) != 2:
            return False, count, 'bilateral_wheels_not_both_contacting'
    elif first(row, 'contacts_legal', 'legal_contacts'):
        if not flag(row, 'contacts_legal', 'legal_contacts'):
            return False, count, 'illegal_contact'
    return True, count, 'ok'


def load_controller(path: Path) -> list[dict[str, Any]]:
    parsed = []
    for row in read_csv(path):
        t = number(first(row, 'timestamp', 'sim_time', 'time_s'))
        ns = integer(first(row, 'control_sim_time_ns', 'controller_sim_time_ns', 'sim_time_ns'))
        if ns is None and t is not None:
            ns = int(round(t * 1e9))
        if ns is None:
            continue
        parsed.append({'ns': ns, 'row': row})
    return parsed


def controller_at(rows: list[dict[str, Any]], ns: int) -> dict[str, str] | None:
    # Controller diagnostics run slower than physics. Use the latest preceding
    # controller sample, while all physical streams remain exact-key joined.
    lo, hi = 0, len(rows)
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid]['ns'] <= ns:
            lo = mid + 1
        else:
            hi = mid
    return rows[lo - 1]['row'] if lo else None


def is_effort_hold(row: dict[str, str] | None) -> bool:
    if row is None:
        return False
    stage = str(first(row, 'ground_input_stage', 'input_stage', 'stage', default='')).lower()
    state = str(first(row, 'state_name', 'state', default='')).lower()
    if stage:
        return stage in ('effort', 'effort_hold', 'ground_effort_hold', 'hold')
    if flag(row, 'effort_mode_active'):
        return True
    return 'effort' in state and ('hold' in state or 'ground' in state)


def source_actuator(row: dict[str, str]) -> tuple[str, float | None]:
    name = str(first(row, 'actuator', 'joint_name', 'name', 'device', default='')).lower()
    idx = integer(first(row, 'joint_index', 'actuator_index', 'index'))
    if idx in ACTUATORS:
        name = ACTUATORS[idx]
    aliases = {'hl': 'hip_left', 'hipl': 'hip_left', 'hip_left': 'hip_left',
               'kl': 'knee_left', 'kneel': 'knee_left', 'knee_left': 'knee_left',
               'hr': 'hip_right', 'hipr': 'hip_right', 'hip_right': 'hip_right',
               'kr': 'knee_right', 'kneer': 'knee_right', 'knee_right': 'knee_right',
               'left_wheel': 'wheel_left', 'wheel_left': 'wheel_left', 'wheell': 'wheel_left',
               'right_wheel': 'wheel_right', 'wheel_right': 'wheel_right', 'wheelr': 'wheel_right'}
    actuator = aliases.get(name.replace('-', '_').replace(' ', '_'), name)
    value = number(first(row, 'torque', 'effort', 'command_value', 'value', 'target_torque', 'published_torque'))
    return actuator, value


def read_publications(path: Path) -> list[dict[str, Any]]:
    out = []
    for row in read_csv(path):
        kind = str(first(row, 'kind', 'command_kind', default='')).lower()
        publish_ns = integer(first(row, 'publish_ns', 'published_ns', 'publication_ns', 'publish_time_ns'))
        control_ns = integer(first(row, 'control_ns', 'source_ns', 'source_time_ns', 'control_time_ns'))
        if publish_ns is None:
            continue
        if kind in ('leg', 'legs'):
            for col, actuator in zip(('c0', 'c1', 'c2', 'c3'), ('hip_left', 'knee_left', 'hip_right', 'knee_right')):
                value = number(row.get(col))
                if value is not None:
                    out.append({'actuator': actuator, 'value': value, 'publish_ns': publish_ns,
                                'source_ns': control_ns, 'command_id': first(row, 'command_id'), 'valid': value is not None, 'row': row})
        elif kind in ('wheel', 'wheels', 'wheel'):
            # wheel_linear/angular are targets in velocity space, never effort.
            continue
        else:
            actuator, value = source_actuator(row)
            if actuator in ACTUATORS.values() and value is not None:
                out.append({'actuator': actuator, 'value': value, 'publish_ns': publish_ns,
                            'source_ns': control_ns, 'command_id': first(row, 'command_id'), 'valid': value is not None, 'row': row})
    return sorted(out, key=lambda p: p['publish_ns'])


def read_servo(path: Path) -> list[dict[str, Any]]:
    out = []
    for row in read_csv(path):
        ns = integer(first(row, 'ros_publish_ns', 'publish_ns', 'published_ns', 'torque_publish_ns', 'sim_time_ns', 'timestamp_ns'))
        source_ns = integer(first(row, 'cmd_source_ns', 'source_ns', 'control_ns', 'source_sim_time_ns', 'target_source_ns'))
        joint_source_ns = integer(first(row, 'joint_source_ns', 'wheel_joint_source_ns'))
        # Current servo contract logs both wheel torques in each publication row.
        paired_columns = (('wheel_left', 'left_torque_cmd'), ('wheel_right', 'right_torque_cmd'))
        found_paired = False
        common_valid = (flag(row, 'cmd_valid') and flag(row, 'joint_valid') and
                        str(first(row, 'valid_reason', default='valid')).lower() == 'valid')
        for actuator, column, rate_column in (('wheel_left', 'left_torque_cmd', 'left_target_rate'),
                                                 ('wheel_right', 'right_torque_cmd', 'right_target_rate')):
            if column in row:
                found_paired = True
                value = number(row.get(column))
                rate = number(row.get(rate_column))
                gain = number(first(row, 'gain', 'p_gain', 'proportional_gain'))
                out.append({'actuator': actuator, 'value': value, 'valid': common_valid and value is not None,
                            'target_rate': rate, 'gain': gain,
                            'publish_ns': ns, 'source_ns': source_ns, 'joint_source_ns': joint_source_ns,
                            'reason': first(row, 'valid_reason', 'cmd_reason', 'joint_reason', default=''), 'row': row})
        if found_paired:
            continue
        name = str(first(row, 'actuator', 'wheel', 'wheel_name', 'joint_name', 'name', default='')).lower()
        idx = integer(first(row, 'joint_index', 'wheel_index', 'index'))
        if idx == 2 or 'left' in name or name in ('l', 'wl', 'wheell'):
            actuator = 'wheel_left'
        elif idx == 5 or 'right' in name or name in ('r', 'wr', 'wheelr'):
            actuator = 'wheel_right'
        else:
            continue
        value = number(first(row, 'published_torque', 'torque_command', 'torque_cmd', 'effort_command',
                             'wheel_effort', 'effort', 'command_value', 'value', 'output_torque'))
        raw_valid = first(row, 'valid', 'command_valid', 'torque_valid')
        valid = flag(row, 'valid', 'command_valid', 'torque_valid') if raw_valid != '' else value is not None
        out.append({'actuator': actuator, 'value': value, 'valid': valid and value is not None,
                    'target_rate': number(first(row, 'target_rate', 'target_rate_rad_s')),
                    'gain': number(first(row, 'gain', 'p_gain', 'proportional_gain')),
                    'publish_ns': ns, 'source_ns': source_ns, 'joint_source_ns': joint_source_ns,
                    'reason': first(row, 'valid_reason', 'reason', 'invalid_reason', default=''), 'row': row})
    return out


def native_value(row: dict[str, str], stage: str) -> tuple[float | None, bool]:
    if stage == 'before':
        value = number(first(row, 'before_physics_joint_force_cmd_sim_input', 'before_physics_joint_force_cmd', 'joint_force_cmd_before'))
        valid = flag(row, 'before_physics_joint_force_cmd_valid', 'before_physics_input_valid', 'joint_force_cmd_valid_before')
        present = flag(row, 'before_physics_joint_force_cmd_component_present', 'before_physics_input_present')
    else:
        value = number(first(row, 'joint_force_cmd_sim_input', 'joint_force_cmd', 'force_command'))
        valid = flag(row, 'joint_force_cmd_valid', 'input_valid')
        present = flag(row, 'joint_force_cmd_component_present', 'input_present')
    return value, valid and present and value is not None


def vector3(row: dict[str, str], *prefixes: str) -> tuple[float, float, float] | None:
    for prefix in prefixes:
        vals = tuple(number(row.get(prefix + axis)) for axis in ('x', 'y', 'z'))
        if all(x is not None for x in vals):
            return vals  # type: ignore[return-value]
    return None


def write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def latency_audit(publications: list[dict[str, Any]], servo: list[dict[str, Any]], frames: dict[tuple[int, int], dict[int, dict[str, str]]]) -> list[dict[str, Any]]:
    events = publications + [e for e in servo if e.get('value') is not None]
    events.sort(key=lambda x: (x.get('publish_ns') or -1, x['actuator']))
    observed: dict[str, list[tuple[int, float]]] = defaultdict(list)
    for (_, ns), joints in sorted(frames.items(), key=lambda x: x[0][1]):
        for idx, name in ACTUATORS.items():
            r = joints.get(idx)
            if r is None:
                continue
            value, valid = native_value(r, 'before')
            if valid and value is not None:
                observed[name].append((ns, value))
    reports = []
    for event in events:
        name, target, pub_ns = event['actuator'], event.get('value'), event.get('publish_ns')
        if target is None or pub_ns is None:
            continue
        candidates = [(ns, value) for ns, value in observed[name] if event.get('valid', True) and ns >= pub_ns and math.isclose(value, target, rel_tol=0.0, abs_tol=1e-9)]
        first_match = candidates[0] if candidates else None
        # An identical value may match a later/earlier publication too. In that
        # case value-only attribution is ambiguous and is not called a latency.
        competing = [other for other in events if other['actuator'] == name and other is not event and
                     other.get('value') is not None and math.isclose(other['value'], target, rel_tol=0.0, abs_tol=1e-9) and
                     (other.get('publish_ns') or 0) <= (first_match[0] if first_match else pub_ns)]
        ambiguous = bool(competing)
        raw_match_offset = first_match[0] - pub_ns if first_match else None
        pub_end_ns = integer(first(event.get('row', {}), 'publish_end_ns'))
        lower_offset = first_match[0] - pub_end_ns if first_match and pub_end_ns is not None else raw_match_offset
        interval_valid = pub_end_ns is None or pub_end_ns >= pub_ns
        latency_reportable = (first_match is not None and not ambiguous and interval_valid and
                              lower_offset is not None and lower_offset > 0)
        reports.append({'actuator': name, 'command_id': event.get('command_id', ''), 'publication_valid': event.get('valid', True),
                        'publish_ns': pub_ns, 'publish_end_ns': pub_end_ns,
                        'latency_lower_bound_ns': lower_offset if latency_reportable else None,
                        'latency_upper_bound_ns': raw_match_offset if latency_reportable else None,
                        'source_ns': event.get('source_ns'), 'joint_source_ns': event.get('joint_source_ns'), 'published_value': target,
                        'first_matching_physics_ns': first_match[0] if first_match else None,
                        'observed_value': first_match[1] if first_match else None,
                        'matching_offset_ns': raw_match_offset,
                        'latency_ns': raw_match_offset if latency_reportable else None,
                        'match_count_after_publication': len(candidates),
                        'ambiguous_same_value_attribution': ambiguous,
                        'latency_interpretable': latency_reportable,
                        'valid_reason': event.get('reason', ''),
                        'note': 'Exact value match is observational; repeated or same-tick values do not prove causal application.'})
    return reports


def audit(args: argparse.Namespace) -> dict[str, Any]:
    outdir: Path = args.output_dir
    outdir.mkdir(parents=True, exist_ok=True)
    controller = load_controller(args.controller_csv)
    controller_clock = {'time_reversals': 0, 'duplicate_timestamps': 0}
    for i in range(1, len(controller)):
        if controller[i]['ns'] < controller[i-1]['ns']:
            controller_clock['time_reversals'] += 1
        elif controller[i]['ns'] == controller[i-1]['ns']:
            controller_clock['duplicate_timestamps'] += 1
    controller.sort(key=lambda x: x['ns'])
    pubs = read_publications(args.command_publication_csv)
    servo = read_servo(args.wheel_servo_csv)
    servo_gain_values = sorted({e['gain'] for e in servo if e.get('gain') is not None})
    native, ndiag = native_frames(args.native_wrench_csv)
    contacts, cdiag = unique_index(read_csv(args.contact_frames_csv), 'contact_frames')
    geometry, gdiag = unique_index(read_csv(args.geometry_csv), 'geometry')
    sorted_keys = sorted(native)
    clock = {'native_rows': sum(len(j) for j in native.values()), 'native_frames': len(native),
             'duplicate_joint_rows': ndiag.get('duplicate_joint_rows', 0),
             'controller_time_reversals': controller_clock['time_reversals'],
             'controller_duplicate_timestamps': controller_clock['duplicate_timestamps'],
             'contact_unkeyed_rows': cdiag.get('unkeyed', 0), 'geometry_unkeyed_rows': gdiag.get('unkeyed', 0),
             'time_reversals': 0, 'iteration_reversals': 0, 'time_step_errors': 0,
             'iteration_step_errors': 0, 'dt_errors': 0}
    source_order = []
    # Test clock in CSV source order, not sorted dictionary order.
    seen_clock_keys = set()
    for row in read_csv(args.native_wrench_csv):
        key = frame_key(row)
        dt = integer(first(row, 'dt_ns', 'physics_dt_ns'))
        if key and key not in seen_clock_keys:
            source_order.append((key[0], key[1], dt))
            seen_clock_keys.add(key)
    prev = None
    for it, ns, dt in source_order:
        if dt != STEP_NS:
            clock['dt_errors'] += 1
        if prev:
            if ns < prev[1]: clock['time_reversals'] += 1
            elif ns - prev[1] != STEP_NS: clock['time_step_errors'] += 1
            if it < prev[0]: clock['iteration_reversals'] += 1
            elif it - prev[0] != 1: clock['iteration_step_errors'] += 1
        prev = (it, ns)

    # Contact clock and geometry clock are independently checked in file order.
    for label, path in (('contact', args.contact_frames_csv), ('geometry', args.geometry_csv)):
        seq = []
        for row in read_csv(path):
            k = frame_key(row); dt = integer(first(row, 'dt_ns', 'physics_dt_ns'))
            if k: seq.append((k[0], k[1], dt))
        reversals = sum(seq[i][1] < seq[i-1][1] for i in range(1, len(seq)))
        gaps = sum(seq[i][1] - seq[i-1][1] != STEP_NS for i in range(1, len(seq)))
        iteration_reversals = sum(seq[i][0] < seq[i-1][0] for i in range(1, len(seq)))
        iteration_gaps = sum(seq[i][0] - seq[i-1][0] != 1 for i in range(1, len(seq)))
        dt_errors = sum(row_dt != STEP_NS for _, _, row_dt in seq)
        if label == 'contact':
            clock['contact_time_reversals'] = reversals; clock['contact_step_errors'] = gaps; clock['contact_dt_errors'] = dt_errors
            clock['contact_iteration_reversals'] = iteration_reversals; clock['contact_iteration_step_errors'] = iteration_gaps
        else:
            clock['geometry_time_reversals'] = reversals; clock['geometry_step_errors'] = gaps; clock['geometry_dt_errors'] = dt_errors
            clock['geometry_iteration_reversals'] = iteration_reversals; clock['geometry_iteration_step_errors'] = iteration_gaps

    if args.hold_start_ns < 0 or args.hold_end_ns < 0:
        raise ValueError('hold bounds must be non-negative')
    window_established = args.hold_start_ns > 0 and args.hold_end_ns > args.hold_start_ns
    keyed = []
    coverage_counts = Counter()
    contact_sequence: list[tuple[int, int | None, bool]] = []
    signal_rows: list[dict[str, Any]] = []
    hold_rows: list[dict[str, Any]] = []
    pair_rows: list[dict[str, Any]] = []
    last_post: dict[int, float | None] | None = None
    continuity = []
    window_continuity = []
    invalid_finite_input_values = 0
    limit_violations: list[dict[str, Any]] = []
    window_servo_valid_wheels = Counter()
    window_target_rate_violations: list[dict[str, Any]] = []
    window_servo_effort_violations: list[dict[str, Any]] = []
    for key in sorted_keys:
        it, ns = key
        joints = native[key]
        frame = contacts.get(key)
        geo = geometry.get(key)
        c = controller_at(controller, ns)
        controller_ns = integer(first(c or {}, 'control_sim_time_ns', 'controller_sim_time_ns', 'sim_time_ns'))
        if controller_ns is None and c is not None:
            ct = number(first(c, 'timestamp', 'sim_time', 'time_s'))
            controller_ns = int(round(ct * 1e9)) if ct is not None else None
        controller_age = ns - controller_ns if controller_ns is not None else None
        controller_fresh = controller_age is not None and 0 <= controller_age <= CONTROLLER_MAX_AGE_NS
        in_window = args.hold_start_ns <= ns < args.hold_end_ns
        effort_hold = is_effort_hold(c)
        startup_position = bool(c and not flag(c, 'effort_mode_active') and not effort_hold)
        present_six = all(i in joints for i in ACTUATORS)
        valid_flags = []
        values: dict[int, float | None] = {}
        identity_ok = True
        states_ok = True
        for idx in ACTUATORS:
            row = joints.get(idx)
            v, ok = native_value(row, 'before') if row else (None, False)
            raw_input = first(row or {}, 'before_physics_joint_force_cmd_sim_input', 'before_physics_joint_force_cmd', 'joint_force_cmd_before')
            if row and not ok and number(raw_input) is not None:
                invalid_finite_input_values += 1
            values[idx] = v
            valid_flags.append(ok)
            if in_window and v is not None and abs(v) > EFFORT_LIMITS[ACTUATORS[idx]] + 1e-9:
                limit_violations.append({'physics_iteration': it, 'sim_time_ns': ns, 'actuator': ACTUATORS[idx],
                                         'value_nm': v, 'limit_nm': EFFORT_LIMITS[ACTUATORS[idx]]})
            if row:
                before_it = integer(first(row, 'before_physics_iteration', 'before_iteration'))
                before_ns = integer(first(row, 'before_physics_sim_time_ns', 'before_sim_time_ns'))
                before_dt = integer(first(row, 'before_physics_dt_ns', 'before_dt_ns', 'dt_ns'))
                before_phase = str(first(row, 'before_physics_phase', 'native_before_phase', default='')).lower()
                phase_ok = before_phase in ('before_physics_update', 'physicsstep')
                identity_ok &= before_it == it and before_ns == ns and before_dt == STEP_NS and phase_ok
                if not phase_ok: coverage_counts['native_wrong_before_physics_phase_rows'] += 1
                before_q = number(first(row, 'before_physics_joint_position', 'before_joint_position'))
                before_v = number(first(row, 'before_physics_joint_velocity', 'before_joint_velocity'))
                post_q = number(first(row, 'joint_position', 'post_joint_position'))
                post_v = number(first(row, 'joint_velocity', 'post_joint_velocity'))
                states_ok &= (flag(row, 'before_physics_joint_state_valid', 'before_state_valid') and
                              flag(row, 'state_valid', 'post_joint_state_valid') and
                              all(x is not None for x in (before_q, before_v, post_q, post_v)))
                signal_rows.append({'physics_iteration': it, 'sim_time_ns': ns, 'actuator': ACTUATORS[idx],
                                    'signal': 'JointForceCmd_before_Physics', 'value': v,
                                    'valid': int(ok), 'reason': '' if ok else 'missing_or_invalid_native_input',
                                    'semantics': 'physics-step input; simulation command, not measured motor torque'})
                transmitted = number(first(row, 'transmitted_axis_torque', 'joint_transmitted_wrench_axis_torque'))
                wvalid = flag(row, 'wrench_valid', 'transmitted_wrench_valid') and transmitted is not None
                signal_rows.append({'physics_iteration': it, 'sim_time_ns': ns, 'actuator': ACTUATORS[idx],
                                    'signal': 'JointTransmittedWrench_axis', 'value': transmitted,
                                    'valid': int(wvalid), 'reason': '' if wvalid else 'missing_or_invalid_wrench',
                                    'semantics': 'joint transmitted aggregate load; not measured motor torque'})
        if present_six: coverage_counts['frames_with_all_six_rows'] += 1
        coverage_counts['trace_ground_effort_hold_frames'] += int(effort_hold)
        coverage_counts['trace_startup_position_frames'] += int(startup_position)
        if present_six and identity_ok: coverage_counts['frames_with_exact_native_identity'] += 1
        if present_six and states_ok: coverage_counts['frames_with_all_six_before_post_joint_states'] += 1
        if in_window and present_six and identity_ok: coverage_counts['window_frames_with_exact_native_identity'] += 1
        if in_window and present_six and states_ok: coverage_counts['window_frames_with_all_six_before_post_joint_states'] += 1
        coverage_counts['expected_actuator_steps'] += 6
        coverage_counts['valid_actuator_steps'] += sum(valid_flags)
        if in_window:
            coverage_counts['window_physics_frames'] += 1
            coverage_counts['window_effort_frames'] += int(effort_hold)
            coverage_counts['window_fresh_controller_frames'] += int(controller_fresh)
            coverage_counts['window_startup_position_frames'] += int(startup_position)
            coverage_counts['window_complete_six_valid_frames'] += int(present_six and all(valid_flags))
        # geometry and contact are paired only by the physical identity tuple
        geo_valid = bool(geo and flag(geo, 'frame_valid', 'valid') and not first(geo, 'error', default=''))
        contact_ok, count, contact_reason = check_contact(frame) if frame else (False, None, 'missing_frame')
        if frame:
            contact_sequence.append((ns, count, contact_ok))
        if in_window and frame:
            coverage_counts['window_contact_frames'] += 1
            coverage_counts['window_legal_contact_frames'] += int(contact_ok)
        if in_window and geo_valid:
            coverage_counts['window_geometry_frames'] += 1
        # same physical step post velocity to next before velocity continuity
        if last_post is not None and (ns - last_post['ns'] == STEP_NS and it == last_post['iteration'] + 1):
            for idx in ACTUATORS:
                before = number(first(joints.get(idx, {}), 'before_physics_joint_velocity', 'before_joint_velocity'))
                post = last_post.get(ACTUATORS[idx])
                if before is not None and post is not None:
                    diff = abs(before - post)
                    continuity.append(diff)
                    if args.hold_start_ns <= last_post['ns'] < args.hold_end_ns:
                        window_continuity.append(diff)
        post_state: dict[str, Any] = {'ns': ns, 'iteration': it}
        for idx, name in ACTUATORS.items():
            row = joints.get(idx, {})
            v = number(first(row, 'joint_velocity', 'post_joint_velocity'))
            if not flag(row, 'state_valid', 'post_joint_state_valid'):
                v = None
            post_state[name] = v
        last_post = post_state
        zero_contact = count == 0
        invalid_single = count == 1
        limited = []
        for idx, limit in POSITION_LIMITS.items():
            q = number(first(joints.get(idx, {}), 'before_physics_joint_position', 'before_joint_position'))
            if q is not None and abs(q) >= limit - 1e-4:
                limited.append(ACTUATORS[idx])
        if in_window:
            pitch = number(first(c or {}, 'pitch'))
            rate = number(first(c or {}, 'pitch_rate'))
            com = number(first(c or {}, 'capture_com_velocity', 'com_velocity'))
            hold = bool(effort_hold and c is not None)
            stable = bool(hold and controller_fresh and pitch is not None and abs(pitch - 0.034) < 0.05 and
                          rate is not None and abs(rate) < 0.15 and com is not None and abs(com) < 0.10)
            if hold:
                coverage_counts['window_controller_stability_samples'] += 1
                coverage_counts['window_stable_samples'] += int(stable)
            if limited: coverage_counts['window_limit_frames'] += 1
            if invalid_single: coverage_counts['window_single_contact_frames'] += 1
            if zero_contact: coverage_counts['window_zero_contact_frames'] += 1
            if count == 2 and contact_ok:
                coverage_counts['window_double_support_frames'] += 1
            qpitch = pitch_from_quaternion(joints.get(0, {}))
            body_v = vector3(joints.get(0, {}), 'before_physics_base_world_v')
            body_v_post = vector3(joints.get(0, {}), 'post_base_world_v')
            wheel_poses = {}
            wheel_pose_valid = True
            for idx, side in ((2, 'left'), (5, 'right')):
                wr = joints.get(idx, {})
                pose_ok = flag(wr, 'child_pose_valid', 'wheel_pose_valid')
                pos = tuple(number(first(wr, f'child_{axis}', f'wheel_{side}_{axis}')) for axis in 'xyz')
                wheel_pose_valid &= pose_ok and all(x is not None for x in pos)
                wheel_poses[f'native_wheel_{side}_x'] = pos[0]
                wheel_poses[f'native_wheel_{side}_y'] = pos[1]
                wheel_poses[f'native_wheel_{side}_z'] = pos[2]
            if wheel_pose_valid: coverage_counts['window_native_wheel_pose_frames'] += 1
            if body_v is not None and body_v_post is not None and flag(joints.get(0, {}), 'before_physics_base_velocity_valid') and flag(joints.get(0, {}), 'post_base_velocity_valid'):
                coverage_counts['window_native_base_velocity_frames'] += 1
            state_fields = {}
            for idx, name in ACTUATORS.items():
                jr = joints.get(idx, {})
                state_fields[f'{name}_before_q'] = number(first(jr, 'before_physics_joint_position', 'before_joint_position'))
                state_fields[f'{name}_before_v'] = number(first(jr, 'before_physics_joint_velocity', 'before_joint_velocity'))
                state_fields[f'{name}_post_q'] = number(first(jr, 'joint_position', 'post_joint_position'))
                state_fields[f'{name}_post_v'] = number(first(jr, 'joint_velocity', 'post_joint_velocity'))
            geometry_pitch = qpitch
            pair_rows.append({'physics_iteration': it, 'sim_time_ns': ns, 'dt_ns': integer(first(joints.get(0, {}), 'before_physics_dt_ns', 'dt_ns')),
                              'phase': ('ground_effort_hold' if effort_hold else 'startup_position' if startup_position else 'other'),
                              'in_explicit_hold_window': 1, 'all_six_joint_rows': int(present_six),
                              'all_six_before_inputs_valid_finite': int(present_six and all(valid_flags)),
                              'native_contact_valid': int(contact_ok), 'native_contact_count': count,
                              'contact_reason': contact_reason, 'geometry_valid': int(geo_valid),
                              'controller_sample_age_ns': controller_age, 'controller_sample_fresh': int(controller_fresh),
                              'controller_pitch': pitch, 'controller_pitch_rate': rate, 'controller_com_velocity': com,
                              'native_model_pitch_from_quaternion': geometry_pitch,
                              'native_base_linear_velocity_x': body_v[0] if body_v else None,
                              'native_base_linear_velocity_y': body_v[1] if body_v else None,
                              'native_base_linear_velocity_z': body_v[2] if body_v else None,
                              'native_base_post_linear_velocity_x': body_v_post[0] if body_v_post else None,
                              'native_base_post_linear_velocity_y': body_v_post[1] if body_v_post else None,
                              'native_base_post_linear_velocity_z': body_v_post[2] if body_v_post else None,
                              'controller_stable': int(stable), 'at_joint_limit': int(bool(limited)),
                              'joint_limit_names': '|'.join(limited), 'zero_contact': int(zero_contact),
                              'single_contact': int(invalid_single), 'sustained_double_support': int(count == 2 and contact_ok)})
            pair_rows[-1].update(state_fields)
            pair_rows[-1].update(wheel_poses)

    for event in servo:
        pub_ns = event.get('publish_ns')
        if pub_ns is None or not (args.hold_start_ns <= pub_ns < args.hold_end_ns):
            continue
        if event.get('valid'):
            window_servo_valid_wheels[event['actuator']] += 1
        val = event.get('value')
        if val is not None and abs(val) > EFFORT_LIMITS[event['actuator']] + 1e-9:
            window_servo_effort_violations.append({'sim_time_ns': pub_ns, 'actuator': event['actuator'],
                                                   'value_nm': val, 'limit_nm': EFFORT_LIMITS[event['actuator']]})
        rate = event.get('target_rate')
        if event.get('valid') and rate is not None and abs(rate) > WHEEL_RATE_LIMIT + 1e-9:
            window_target_rate_violations.append({'sim_time_ns': pub_ns, 'actuator': event['actuator'],
                                                  'target_rate_rad_s': rate, 'limit_rad_s': WHEEL_RATE_LIMIT})
    # Record exact commands. Leg publication c0..3 are effort commands. Wheel
    # linear/angular targets are deliberately excluded from effort curves.
    published_effort_violations = []
    for p in pubs:
        if args.hold_start_ns <= p['publish_ns'] < args.hold_end_ns and p.get('value') is not None and abs(p['value']) > EFFORT_LIMITS[p['actuator']] + 1e-9:
            published_effort_violations.append({'sim_time_ns': p['publish_ns'], 'actuator': p['actuator'],
                                                'value_nm': p['value'], 'limit_nm': EFFORT_LIMITS[p['actuator']]})
        signal_rows.append({'physics_iteration': '', 'sim_time_ns': p['publish_ns'], 'actuator': p['actuator'],
                            'signal': 'PublishedCommand_exact', 'value': p['value'], 'valid': 1,
                            'reason': '', 'semantics': 'exact publication value; wheel velocity targets excluded'})
    for e in servo:
        signal_rows.append({'physics_iteration': '', 'sim_time_ns': e.get('publish_ns'), 'actuator': e['actuator'],
                            'signal': 'PublishedCommand_exact', 'value': e['value'],
                            'valid': int(e['valid']), 'reason': e['reason'],
                            'semantics': 'servo torque publication; not wheel speed target and not measured torque'})
    latency = latency_audit(pubs, servo, native)
    latency_valid = [x['latency_ns'] for x in latency if x['latency_interpretable']]

    # A run of consecutive paired legal double-contact frames; report transitions
    # independently from sustained support length.
    transitions = []
    previous_count = None
    for ns, count, legal in contact_sequence:
        if previous_count is not None and count != previous_count:
            transitions.append({'sim_time_ns': ns, 'from_contact_count': previous_count, 'to_contact_count': count})
        previous_count = count
    ds_runs = []
    run = []
    prev_ns = None
    for ns, count, legal in contact_sequence:
        if count == 2 and legal and (prev_ns is None or ns - prev_ns == STEP_NS):
            run.append(ns)
        else:
            if run: ds_runs.append(run)
            run = [ns] if count == 2 and legal else []
        prev_ns = ns
    if run: ds_runs.append(run)
    longest_ds = max((len(x) for x in ds_runs), default=0)

    expected_ns = list(range(args.hold_start_ns, args.hold_end_ns, STEP_NS)) if window_established else []
    frame_keys = set(native)
    missing_window_keys = sum((next((it for it, t in frame_keys if t == ns), None), ns) not in frame_keys for ns in expected_ns)
    duration_ns = args.hold_end_ns - args.hold_start_ns if window_established else 0
    window_duration_ok = window_established and duration_ns >= 5_000_000_000
    frame_count_ok = window_established and coverage_counts['window_physics_frames'] == len(expected_ns) and missing_window_keys == 0
    six_inputs_ok = window_established and coverage_counts['window_complete_six_valid_frames'] == len(expected_ns)
    effort_ok = window_established and coverage_counts['window_effort_frames'] == len(expected_ns)
    controller_fresh_ok = window_established and coverage_counts['window_fresh_controller_frames'] == len(expected_ns)
    stable_ok = (window_established and coverage_counts['window_stable_samples'] == len(expected_ns) and
                 coverage_counts['window_controller_stability_samples'] == len(expected_ns))
    contact_ok = (window_established and coverage_counts['window_legal_contact_frames'] == len(expected_ns) and
                  coverage_counts['window_double_support_frames'] == len(expected_ns))
    geometry_ok = window_established and coverage_counts['window_geometry_frames'] == len(expected_ns)
    native_identity_ok = window_established and coverage_counts['window_frames_with_exact_native_identity'] == len(expected_ns)
    native_states_ok = window_established and coverage_counts['window_frames_with_all_six_before_post_joint_states'] == len(expected_ns)
    continuity_expected = len(expected_ns) * len(ACTUATORS)
    continuity_ok = (window_established and len(window_continuity) >= continuity_expected and
                     max(window_continuity, default=float('inf')) <= 1e-9)
    wheel_pose_ok = window_established and coverage_counts['window_native_wheel_pose_frames'] == len(expected_ns)
    body_velocity_ok = window_established and coverage_counts['window_native_base_velocity_frames'] == len(expected_ns)
    no_air_or_single = coverage_counts['window_zero_contact_frames'] == 0 and coverage_counts['window_single_contact_frames'] == 0
    no_limits = coverage_counts['window_limit_frames'] == 0
    hard_caps_ok = not limit_violations and not published_effort_violations and not window_servo_effort_violations and not window_target_rate_violations
    servo_coverage_ok = (window_established and window_servo_valid_wheels['wheel_left'] > 0 and
                         window_servo_valid_wheels['wheel_right'] > 0)
    gain_ok = not servo_gain_values or all(abs(value - 1.0) <= 1e-12 for value in servo_gain_values)
    clock_ok = all(clock.get(k, 0) == 0 for k in ('time_reversals','iteration_reversals','time_step_errors','iteration_step_errors','dt_errors',
                                                  'duplicate_joint_rows','controller_time_reversals','controller_duplicate_timestamps',
                                                  'contact_time_reversals','contact_step_errors','contact_dt_errors','contact_iteration_reversals','contact_iteration_step_errors',
                                                  'geometry_time_reversals','geometry_step_errors','geometry_dt_errors','geometry_iteration_reversals','geometry_iteration_step_errors'))
    failures = []
    if clock['duplicate_joint_rows']:
        failures.append('duplicate_native_joint_rows')
    if clock['controller_time_reversals'] or clock['controller_duplicate_timestamps']:
        failures.append('controller_trace_clock_invalid')
    for ok, reason in ((window_established, 'hold_window_not_established'), (window_duration_ok, 'hold_window_shorter_than_5s'), (frame_count_ok, 'physical_frames_missing_or_not_contiguous'),
                       (six_inputs_ok, 'six_physics_inputs_not_all_finite_valid'), (effort_ok, 'window_not_entirely_ground_effort_hold'),
                       (stable_ok, 'controller_pose_or_com_speed_outside_initial_balance_limits'),
                       (contact_ok, 'invalid_or_missing_complete_bilateral_contact_frame'), (geometry_ok, 'invalid_or_missing_geometry_frame'),
                       (native_identity_ok, 'native_before_physics_identity_or_phase_does_not_match_physics_key'),
                       (native_states_ok, 'before_or_post_joint_state_missing_or_invalid'),
                       (wheel_pose_ok, 'native_wheel_pose_missing_or_invalid'),
                       (body_velocity_ok, 'native_body_velocity_missing_or_invalid'),
                       (continuity_ok, 'post_to_next_before_velocity_continuity_incomplete_or_failed'),
                       (no_air_or_single, 'airborne_or_single_contact_in_hold_window'),
                       (window_established and coverage_counts['window_double_support_frames'] == len(expected_ns), 'not_continuous_bilateral_support'),
                       (no_limits, 'joint_limit_margin_crossed'), (hard_caps_ok, 'command_or_native_input_exceeded_effort_or_wheel_rate_limit'),
                       (servo_coverage_ok, 'wheel_servo_valid_publication_missing'), (gain_ok, 'wheel_servo_gain_not_one'),
                       (clock_ok, 'clock_reversal_or_step_error')):
        if not ok: failures.append(reason)
    result = {
        'audit': 'ground_input_trial_stage1',
        'gate': 'PASS' if not failures else 'FAIL',
        'scope': 'allocator-off bounded-P wheel ground standing six-input validation only',
        'hold_window': {'established': window_established, 'start_ns': args.hold_start_ns if window_established else None, 'end_ns_exclusive': args.hold_end_ns if window_established else None,
                        'duration_s': duration_ns * 1e-9, 'duration_at_least_5s': window_duration_ok,
                        'window_selected_only_by_explicit_bounds': True},
        'counts': dict(coverage_counts),
        'clock': clock,
        'native_input_coverage': {'valid_finite_actuator_steps': coverage_counts['valid_actuator_steps'],
                                  'expected_actuator_steps': coverage_counts['expected_actuator_steps'],
                                  'fraction': (coverage_counts['valid_actuator_steps'] / coverage_counts['expected_actuator_steps']
                                               if coverage_counts['expected_actuator_steps'] else 0.0),
                                  'all_invalid_values_remain_invalid': invalid_finite_input_values == 0,
                                  'invalid_input_fields_with_finite_value': invalid_finite_input_values},
        'continuity': {'post_to_next_before_velocity_pairs': len(continuity),
                       'window_pairs': len(window_continuity), 'window_expected_pairs_minimum': continuity_expected,
                       'window_max_abs_velocity_difference': max(window_continuity) if window_continuity else None,
                       'window_gate_max_abs_difference': 1e-9,
                       'window_gate': 'PASS' if continuity_ok else 'FAIL',
                       'max_abs_velocity_difference': max(continuity) if continuity else None,
                       'rms_abs_velocity_difference': math.sqrt(sum(x*x for x in continuity) / len(continuity)) if continuity else None,
                       'meaning': 'same-physics post velocity compared with next physical frame before velocity'},
        'contact': {'transition_count': len(transitions), 'transitions': transitions,
                    'longest_legal_double_support_frames': longest_ds,
                    'longest_legal_double_support_s': longest_ds * STEP_NS * 1e-9,
                    'double_support_is_reported_separately_from_contact_transitions': True},
        'latency': {'publication_events': len(pubs), 'servo_torque_publications': len(servo),
                    'interpretable_exact_match_count': len(latency_valid),
                    'latency_ns_median': sorted(latency_valid)[len(latency_valid)//2] if latency_valid else None,
                    'ambiguous_events': sum(bool(x['ambiguous_same_value_attribution']) for x in latency),
                    'same_value_is_not_zero_latency_guarantee': True,
                    'records': latency},
        'stability_thresholds': {'controller_max_sample_age_ns': CONTROLLER_MAX_AGE_NS,
                                 'abs_controller_pitch_minus_0.034_rad_lt': 0.05,
                                 'abs_pitch_rate_rad_s_lt': 0.15, 'abs_capture_com_velocity_m_s_lt': 0.10,
                                 'native_model_pose_pitch_reported_separately': True},
        'hard_input_limits': {'effort_limits_nm': EFFORT_LIMITS, 'wheel_target_rate_limit_rad_s': WHEEL_RATE_LIMIT,
                              'native_before_physics_violations': limit_violations,
                              'published_effort_violations': published_effort_violations,
                              'wheel_servo_effort_violations': window_servo_effort_violations,
                              'wheel_target_rate_violations': window_target_rate_violations,
                              'wheel_servo_valid_publication_counts': dict(window_servo_valid_wheels),
                              'p_gain_expected': 1.0, 'p_gain_logged_values': servo_gain_values,
                              'p_gain_status': ('verified_from_servo_csv' if servo_gain_values else 'not_logged_in_servo_csv; verify runtime configuration')},
        'joint_limits': {'hip_abs_rad': 1.52, 'knee_abs_rad': 1.5708,
                         'margin_rad': 1e-4, 'interpretation': 'instrumented position limit threshold; not a physical impact detector'},
        'signals': {'curve_data_csv': 'signals.csv',
                    'semantics': ['PublishedCommand_exact: exact leg effort publications; servo wheel torque publication; wheel velocity targets are excluded',
                                  'JointForceCmd_before_Physics: simulator input, not measured motor torque',
                                  'JointTransmittedWrench_axis: aggregate joint load, not measured motor torque'],
                    'plot_file': 'signals.png'},
        'jump_metrics': {'jump_height_m': None, 'flight_duration_s': None, 'airborne_frames': 0, 'accepted_jumps': 0},
        'not_certified': ['model-response gate', 'braking/deceleration capability', 'jump feasibility', 'landing performance'],
        'failures': failures,
    }
    (outdir / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    write_csv(outdir / 'paired_frames.csv', list(pair_rows[0]) if pair_rows else ['physics_iteration','sim_time_ns'], pair_rows)
    write_csv(outdir / 'signals.csv', ['physics_iteration','sim_time_ns','actuator','signal','value','valid','reason','semantics'], signal_rows)
    write_csv(outdir / 'input_latency.csv', ['actuator','command_id','publication_valid','publish_ns','publish_end_ns','latency_lower_bound_ns','latency_upper_bound_ns','source_ns','joint_source_ns','published_value','first_matching_physics_ns','observed_value','matching_offset_ns','latency_ns','match_count_after_publication','ambiguous_same_value_attribution','latency_interpretable','valid_reason','note'], latency)
    (outdir / 'contact_transitions.json').write_text(json.dumps({'transitions': transitions, 'longest_legal_double_support_frames': longest_ds}, indent=2)+'\n', encoding='utf-8')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        plot_order = ['hip_left','hip_right','knee_left','knee_right','wheel_left','wheel_right']
        colors = {name: f'C{i}' for i,name in enumerate(plot_order)}
        for signal, axis, title in [
            ('PublishedCommand_exact', axes[0], 'Exact published effort / wheel servo torque'),
            ('JointForceCmd_before_Physics', axes[1], 'Physics-before JointForceCmd'),
            ('JointTransmittedWrench_axis', axes[2], 'JointTransmittedWrench axial aggregate load')]:
            for name in plot_order:
                pts = [r for r in signal_rows if r['signal'] == signal and r['actuator'] == name and r['valid'] and r['value'] is not None]
                if pts:
                    axis.plot([(int(r['sim_time_ns'])-args.hold_start_ns)*1e-9 for r in pts], [float(r['value']) for r in pts], label=name, color=colors[name], linewidth=0.8)
            axis.set_ylabel('N·m')
            axis.set_title(title); axis.grid(True, alpha=.25); axis.legend(ncol=3, fontsize=8)
        axes[-1].set_xlabel('Time relative to hold start (s)')
        fig.tight_layout(); fig.savefig(outdir / 'signals.png', dpi=140); plt.close(fig)
    except Exception as exc:
        result['signals']['plot_file'] = None
        result['signals']['plot_error'] = str(exc)
        (outdir / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-csv', type=Path, required=True)
    parser.add_argument('--command-publication-csv', type=Path, required=True)
    parser.add_argument('--native-wrench-csv', type=Path, required=True)
    parser.add_argument('--contact-frames-csv', type=Path, required=True)
    parser.add_argument('--geometry-csv', type=Path, required=True)
    parser.add_argument('--wheel-servo-csv', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--hold-start-ns', type=int, default=0, help='explicit Effort-hold start in simulation ns; zero leaves window unestablished')
    parser.add_argument('--hold-end-ns', type=int, default=0, help='explicit Effort-hold end (exclusive) in simulation ns; zero leaves window unestablished')
    args = parser.parse_args()
    try:
        result = audit(args)
    except (OSError, ValueError, csv.Error) as exc:
        parser.error(str(exc))
    print(json.dumps({'gate': result['gate'], 'failures': result['failures'],
                      'hold_window': result['hold_window'], 'counts': result['counts']}, indent=2))
    return 0 if result['gate'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
