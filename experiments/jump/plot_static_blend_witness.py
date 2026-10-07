#!/usr/bin/env python3
"""Plot the offline static-friction witness; never labels inferred force as measured."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--witness', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.witness.open() as stream:
        rows = list(csv.DictReader(stream))
    values = lambda key: np.array([float(row[key]) for row in rows])
    alpha = values('alpha')
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for ax, joint, indices in ((axes[0, 0], 'Hip', (0, 2)), (axes[0, 1], 'Knee', (1, 3))):
        for index, side in zip(indices, ('Left', 'Right')):
            ax.plot(alpha, values(f'blended_u{index}'), 'o-' if side == 'Left' else 'x--', label=side)
        ax.set_title(f'{joint} torque: hypothetical blended output')
        ax.set_ylabel('Torque (Nm)')
        ax.legend()
    ax = axes[1, 0]
    ax.plot(alpha, values('max_stick_effort'), 'o-', label='Full forward model: required static reaction')
    ax.axhline(.1, color='gray', linestyle='--', label='Existing static friction limit')
    ax.set_ylim(-.005, .115)
    ax.set_ylabel('Maximum absolute joint reaction (Nm)')
    ax.set_title('Inferred static friction; all four joints stick')
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    ax.plot(alpha, values('forward_valid'), 'o-', label='81-mode plant feasible')
    ax.plot(alpha, values('helper_valid'), 'x--', label='Proposed controller accepts')
    ax.set_yticks((0, 1), labels=('Reject', 'Accept'))
    ax.set_ylim(-.15, 1.2)
    ax.set_title('Qualification fails during handoff')
    ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.set_xlabel('New-law blend fraction')
        ax.set_xticks(alpha)
        ax.grid(alpha=.25)
    fig.suptitle('OFFLINE MODEL ONLY - fixed native anchor at 18.567 s', fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    main()
