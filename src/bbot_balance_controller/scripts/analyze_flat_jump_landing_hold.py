#!/usr/bin/env python3
"""Audit sustained landing from full CSV traces, independent of early summaries.

This checks landing recovery only; takeoff-height/velocity and normal-flight
acceptance retain their separate verdicts. Contact effort is not available.
"""
import argparse
import csv
import math
from pathlib import Path

FIELDS = ('trial', 'jump_id', 'flight_pitch_rate', 'touchdown_pitch', 'touchdown_lean',
          'balance_delay', 'hold_duration', 'tail_pitch', 'tail_com_vx',
          'tail_height', 'max_joint_abs', 'landing_hold_pass', 'reason')

def read(path):
    with path.open(newline='') as f:
        return [r for r in csv.DictReader(f) if r.get('timestamp') and r.get('state_name')]

def audit(path, hold_seconds, jump_id=None):
    rows = read(path)
    if jump_id is not None:
        rows = [r for r in rows if str(r.get('jump_id', '')) == str(jump_id)]
    out = dict.fromkeys(FIELDS, '')
    out['trial'] = path.stem
    out['jump_id'] = '' if jump_id is None else str(jump_id)
    out['landing_hold_pass'] = False
    if not rows:
        out['reason'] = 'no data'
        return out
    flight = next((r for r in rows if r['state_name'] == 'FLIGHT'), None)
    td = next((r for r in rows if r['state_name'] == 'TOUCHDOWN_BUFFER'), None)
    if flight:
        out['flight_pitch_rate'] = float(flight['pitch_rate'])
    if not td:
        out['reason'] = 'no touchdown'
        return out
    td_time = float(td['timestamp'])
    out['touchdown_pitch'] = float(td['pitch'])
    out['touchdown_lean'] = float(td['com_lean'])
    after = [r for r in rows if float(r['timestamp']) >= td_time]
    first = next((r for r in after if r['state_name'] == 'BALANCE'), None)
    if not first:
        out['reason'] = 'EMERGENCY before recovery' if any(
            r['state_name'] == 'EMERGENCY' for r in after) else 'no BALANCE'
        return out
    b_time = float(first['timestamp'])
    balance = [r for r in after if float(r['timestamp']) >= b_time]
    end = float(balance[-1]['timestamp'])
    out['balance_delay'] = b_time-td_time
    out['hold_duration'] = end-b_time
    out['max_joint_abs'] = max(abs(float(r[key])) for r in balance for key in
        ('hip_pos_left', 'hip_pos_right', 'knee_pos_left', 'knee_pos_right'))
    tail = [r for r in balance if float(r['timestamp']) >= end-min(5., hold_seconds)]
    out['tail_pitch'] = float(tail[-1]['pitch'])
    out['tail_com_vx'] = float(tail[-1]['capture_com_velocity'])
    out['tail_height'] = float(tail[-1]['z'])
    reasons = []
    # A quiet final window cannot erase invalid measurements earlier in the
    # claimed standing interval. In particular, comparisons against NaN do
    # not reject a bad height or joint sample on their own.
    standing_channels = ('timestamp', 'pitch', 'pitch_rate', 'z', 'com_lean',
                         'capture_com_velocity', 'gazebo_world_z_dot',
                         'hip_pos_left', 'hip_pos_right',
                         'knee_pos_left', 'knee_pos_right')
    if any(not math.isfinite(float(row[key]))
           for row in balance for key in standing_channels):
        reasons.append('nonfinite standing measurement')
    if out['balance_delay'] > 6:
        reasons.append('recovery >6s')
    if out['hold_duration'] < hold_seconds:
        reasons.append('hold too short')
    if any(r['state_name'] != 'BALANCE' for r in balance):
        reasons.append('left BALANCE')
    balance_times = [float(r['timestamp']) for r in balance]
    if any(b <= a for a, b in zip(balance_times, balance_times[1:])):
        reasons.append('duplicate or out-of-order standing timestamps')
    elif any(b - a > 0.080 for a, b in zip(balance_times, balance_times[1:])):
        reasons.append('standing trace gap >80ms')
    if out['max_joint_abs'] >= 1.45:
        reasons.append('joint near hard stop')
    if any(abs(float(r['z'])-.5) >= .04 for r in balance):
        reasons.append('standing height drift')
    for row in tail:
        if (not all(math.isfinite(float(row[key])) for key in
                    ('pitch','pitch_rate','capture_com_velocity','gazebo_world_z_dot','com_lean','z'))
            or abs(float(row['pitch'])-.03) >= .04 or abs(float(row['pitch_rate'])) >= .15
            or abs(float(row['capture_com_velocity'])) >= .08
            or abs(float(row['gazebo_world_z_dot'])) >= .03
            or abs(float(row['com_lean'])) >= .08):
            reasons.append('tail outside quiet limits')
            break
    for row in balance:
        stamp = float(row['timestamp'])
        if row.get('capture_world_valid') != '1' or any(
            not 0 <= stamp-float(row[key]) <= .08
            for key in ('com_sample_stamp','imu_sample_stamp','joint_sample_stamp','odom_sample_stamp')):
            reasons.append('stale/invalid standing sensing')
            break
    out['landing_hold_pass'] = not reasons
    out['reason'] = '; '.join(reasons)
    return out

def audit_session(path, hold_seconds, expected_jumps=None):
    """Audit every recorded jump id independently; never let a later hop mask one."""
    rows = read(path)
    jump_ids = []
    previous_nonzero_id = None
    for row in rows:
        jump_id = str(row.get('jump_id', ''))
        if jump_id in ('', '0'):
            continue
        if jump_id != previous_nonzero_id:
            jump_ids.append(jump_id)
        previous_nonzero_id = jump_id
    if expected_jumps is None and jump_ids:
        expected_jumps = len(jump_ids)
    if expected_jumps is not None:
        expected = [str(i) for i in range(1, expected_jumps + 1)]
        results = [audit(path, hold_seconds, jid) for jid in expected]
        for jid in jump_ids:
            if jid not in expected:
                extra = dict.fromkeys(FIELDS, '')
                extra.update(trial=path.stem, jump_id=jid, landing_hold_pass=False,
                             reason='unexpected or reused jump_id in trace')
                results.append(extra)
        if jump_ids != expected:
            for result in results:
                result['landing_hold_pass'] = False
                result['reason'] = '; '.join(filter(
                    None, (result['reason'], f'jump_id sequence is not exactly 1..N: {jump_ids}')))
        return results
    return [audit(path, hold_seconds, jid) for jid in jump_ids] or [audit(path, hold_seconds)]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--hold-seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = [result for p in sorted(args.directory.glob('*_log.csv'))
               for result in audit_session(p, args.hold_seconds)]
    if args.output:
        with args.output.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(results)
    for row in results:
        print(row['trial'], 'PASS' if row['landing_hold_pass'] else 'FAIL',
              'delay=', row['balance_delay'], 'hold=', row['hold_duration'], row['reason'])
    print('Landing hold:', sum(r['landing_hold_pass'] for r in results), '/', len(results),
          '(not full jump acceptance)')

if __name__ == '__main__':
    main()
