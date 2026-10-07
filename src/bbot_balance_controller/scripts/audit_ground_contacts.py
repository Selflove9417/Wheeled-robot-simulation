#!/usr/bin/env python3
"""Validate native ground-contact names and sensor/bridge presence per session."""
import argparse
import csv
from pathlib import Path

GROUND_COLLISION = 'ground_plane::link::collision'
WHEEL_COLLISIONS = {
    'bbot::link_004::link_004_collision_collision',
    'bbot::link_007::link_007_collision_collision',
}


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def audit(log_path, contacts_path):
    trace = read_csv(log_path)
    contacts = read_csv(contacts_path)
    out = {'ground_contact_pass': False, 'ground_contact_events': 0, 'reason': ''}
    reasons = []
    jumps = [row for row in trace if row.get('jump_id') not in ('', '0', None)]
    if not jumps:
        out['reason'] = 'no jump-id trace'
        return out
    start = float(jumps[0]['timestamp'])
    end = float(jumps[-1]['timestamp'])
    statuses = [row for row in contacts if row.get('side') == 'GROUND_STATUS' and
                start <= float(row['sim_time']) <= end]
    if not statuses:
        reasons.append('ground sensor status is missing during jump session')
    else:
        # Sensor teardown can publish a final zero-publisher status at the same
        # simulation timestamp as the controller's last trace row.  Validate
        # coverage from positive publisher observations; a teardown marker is
        # not evidence that the bridge was absent during the active jump.
        active_times = sorted(set(
            float(row['sim_time']) for row in statuses
            if int(float(row.get('ground_publisher_count') or 0)) > 0))
        if not active_times:
            reasons.append('ground contact bridge has no publisher')
        if active_times and (active_times[0] - start > .25 or end - active_times[-1] > .25):
            reasons.append('ground sensor status does not cover full jump trace')
        if any(b - a > .25 for a, b in zip(active_times, active_times[1:])):
            reasons.append('ground sensor status has an observation gap')

    events = []
    all_events = []
    invalid_pairs = {}
    malformed_events = set()
    for row in contacts:
        if row.get('side') != 'GROUND' or int(float(row.get('num_contacts') or 0)) <= 0:
            continue
        stamp = float(row['stamp_sec'])
        if not start <= stamp <= end:
            continue
        out['ground_contact_events'] += 1
        c1, c2 = row.get('collision_1', ''), row.get('collision_2', '')
        pair = {c1, c2}
        if GROUND_COLLISION not in pair:
            malformed_events.add('ground contact collision pair lacks expected ground collision name')
            continue
        robot_parts = pair - {GROUND_COLLISION}
        all_events.append((stamp, robot_parts))
        if not robot_parts or not robot_parts.issubset(WHEEL_COLLISIONS):
            invalid_pairs.setdefault(tuple(sorted(robot_parts)), stamp)
            continue
        events.append((stamp, robot_parts))
    if not events:
        reasons.append('no native ground contact events recorded')

    jump_ids = []
    previous_id = None
    for row in jumps:
        jump_id = row.get('jump_id', '')
        if jump_id != previous_id:
            jump_ids.append(jump_id)
            previous_id = jump_id
    if jump_ids != [str(index) for index in range(1, len(jump_ids) + 1)]:
        reasons.append(f'jump_id sequence is not strictly 1..N: {jump_ids}')
    for jump_id in jump_ids:
        hop = [row for row in trace if row.get('jump_id') == jump_id]
        flight = next((row for row in hop if row.get('state_name') == 'FLIGHT'), None)
        touchdown = next((row for row in hop if row.get('state_name') == 'TOUCHDOWN_BUFFER'), None)
        balance = next((row for row in hop if row.get('state_name') == 'BALANCE'), None)
        if flight is None or touchdown is None:
            reasons.append(f'jump_id {jump_id} has no complete FLIGHT/touchdown interval')
            continue
        ftime, tdtime = float(flight['timestamp']), float(touchdown['timestamp'])
        first_touch_events = [event for event in all_events if ftime <= event[0] <= tdtime + .15]
        if not first_touch_events:
            reasons.append(f'jump_id {jump_id} has no wheel-ground event at touchdown')
        else:
            first_touch_time = min(event[0] for event in first_touch_events)
            first_touch_at_stamp = [event for event in first_touch_events
                                    if abs(event[0] - first_touch_time) <= 1e-9]
            if any(not event[1] or not event[1].issubset(WHEEL_COLLISIONS)
                   for event in first_touch_at_stamp):
                reasons.append(f'jump_id {jump_id} first touchdown contact was not a wheel')
            touch_times = sorted(set(
                float(row['stamp_sec']) for row in contacts
                if row.get('side') == 'GROUND' and int(float(row.get('num_contacts') or 0)) > 0
                and first_touch_time <= float(row['stamp_sec']) <= first_touch_time + .020))
            if (len(touch_times) < 8 or touch_times[-1] - touch_times[0] < .010 or
                    touch_times[0] > first_touch_time + .005 or
                    touch_times[-1] < first_touch_time + .015 or
                    any(b - a > .005 for a, b in zip(touch_times, touch_times[1:]))):
                reasons.append(f'jump_id {jump_id} native touchdown contact cadence is insufficient')

        if balance is None:
            reasons.append(f'jump_id {jump_id} has no BALANCE hold')
            continue
        balance_start = float(balance['timestamp'])
        hold_end = balance_start + 12.0
        balance_rows = [row for row in hop if balance_start <= float(row['timestamp']) <= hold_end]
        if not balance_rows or float(balance_rows[-1]['timestamp']) < hold_end - .080:
            reasons.append(f'jump_id {jump_id} has less than 12s full-trace BALANCE coverage')
            continue
        if any(row.get('state_name') != 'BALANCE' for row in balance_rows):
            reasons.append(f'jump_id {jump_id} left BALANCE during 12s hold')
        balance_trace_times = [float(row['timestamp']) for row in balance_rows]
        if (any(b <= a for a, b in zip(balance_trace_times, balance_trace_times[1:])) or
                any(b - a > .080 for a, b in zip(balance_trace_times, balance_trace_times[1:]))):
            reasons.append(f'jump_id {jump_id} BALANCE trace timestamps are duplicate, out of order, or >80ms apart')
        native_hold_times = sorted(set(
            float(row['stamp_sec']) for row in contacts
            if row.get('side') == 'GROUND' and int(float(row.get('num_contacts') or 0)) > 0
            and balance_start <= float(row['stamp_sec']) <= hold_end))
        if (not native_hold_times or native_hold_times[0] > balance_start + .020 or
                native_hold_times[-1] < hold_end - .020 or
                any(b - a > .020 for a, b in zip(native_hold_times, native_hold_times[1:]))):
            reasons.append(f'jump_id {jump_id} native ground-contact coverage missing or has >20ms gap during BALANCE hold')
    reasons.extend(sorted(malformed_events))
    reasons.extend(
        f'non-wheel or unknown robot-ground collision at {stamp:.3f}: {list(pair)}'
        for pair, stamp in invalid_pairs.items())
    out['ground_contact_pass'] = not reasons
    out['reason'] = '; '.join(dict.fromkeys(reasons))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('contacts', type=Path)
    args = parser.parse_args()
    result = audit(args.log, args.contacts)
    print(result)
    raise SystemExit(0 if result['ground_contact_pass'] else 1)


if __name__ == '__main__':
    main()
