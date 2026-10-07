#!/usr/bin/env python3
"""Compare flat-jump RECOVERY stable windows using reliable log channels.

This is a diagnostic, not a causal estimator. The nearest baseline is chosen
only by the listed entry coordinates; contact wrench is absent in Gazebo.
"""
import argparse
import csv
import math
from pathlib import Path

ENTRY_KEYS = ('com_lean', 'x_dot', 'pitch', 'pitch_rate',
              'hip_pos_left', 'knee_pos_left')
ENTRY_SCALES = (0.06, 0.15, 0.04, 0.10, 0.10, 0.10)


def load_trial(path):
    with path.open(newline='') as stream:
        rows = [r for r in csv.DictReader(stream)
                if r.get('timestamp') and r.get('pitch') and r.get('state_name')]
    stable = [r for r in rows if r.get('state_name') == 'RECOVERY'
              and r.get('recovery_subphase') == '1']
    if not stable:
        return {'name': path.stem.replace('_log', ''), 'stable': False}
    start = float(stable[0]['timestamp'])
    def nearest(offset):
        return min(stable, key=lambda r: abs(float(r['timestamp'])-start-offset))
    entry = nearest(0.02)
    def crossing(level):
        for r in stable:
            if abs(float(r['com_lean'])) >= level:
                return float(r['timestamp'])-start
        return math.nan
    return {
        'name': path.stem.replace('_log', ''), 'stable': True,
        'entry': {key: float(entry[key]) for key in ENTRY_KEYS},
        'cross15': crossing(0.15), 'cross30': crossing(0.30),
        'last_state': rows[-1]['state_name'],
        'last_pitch': float(rows[-1]['pitch']),
        'duration': float(stable[-1]['timestamp'])-start,
    }


def load_directory(directory):
    return [load_trial(path) for path in sorted(directory.glob('trial_*_log.csv'))]


def entry_distance(a, b):
    return sum(((a['entry'][key]-b['entry'][key])/scale)**2
               for key, scale in zip(ENTRY_KEYS, ENTRY_SCALES))


def show(trial):
    if not trial['stable']:
        return f"{trial['name']}: no RECOVERY stable window"
    values = ', '.join(f'{key}={trial["entry"][key]:.3f}' for key in ENTRY_KEYS)
    return (f"{trial['name']}: entry [{values}], "
            f"|lean|=.15/.30 at {trial['cross15']:.3f}/{trial['cross30']:.3f}s, "
            f"duration={trial['duration']:.2f}s, last={trial['last_state']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--baseline', type=Path)
    args = parser.parse_args()
    base = [t for t in load_directory(args.baseline) if t['stable']] if args.baseline else []
    for candidate in load_directory(args.candidate):
        print(show(candidate))
        if candidate['stable'] and base:
            for match in sorted(base, key=lambda b: entry_distance(candidate, b))[:3]:
                print(f'  nearest d2={entry_distance(candidate, match):.3f}: {show(match)}')


if __name__ == '__main__':
    main()
