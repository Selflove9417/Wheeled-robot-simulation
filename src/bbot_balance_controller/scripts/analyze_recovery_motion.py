#!/usr/bin/env python3
"""Kinematic path decomposition, not a force or causal estimator.

Integrate COM-forward Jacobian contributions along measured hip/knee/pitch
paths. Report wheel/axle forward velocity using a single +forward coordinate.
Only uses the validated geometry from planar_leg_pitch_model_3dof.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
import planar_leg_pitch_model_3dof as model

FIELDS = ("group", "trial", "offset", "pitch", "hip", "knee", "com_forward",
          "pitch_forward_delta", "hip_forward_delta", "knee_forward_delta",
          "closure_error", "com_velocity", "axle_velocity", "wheel_axle_velocity",
          "command", "capture_active")
POSITION_KEYS = ("hip_pos_left", "knee_pos_left", "pitch")
VELOCITY_KEYS = ("hip_vel_left", "knee_vel_left", "pitch_rate")

def forward_jacobian(q):
    return np.einsum("b,bi->i", model.MASSES,
                     model.position_jacobian(q)[:, 0, :]) / model.MASSES.sum()

def decompose(path):
    with path.open(newline="") as stream:
        rows = [r for r in csv.DictReader(stream)
                if r.get("state_name") == "RECOVERY" and r.get("recovery_subphase") == "1"]
    if not rows:
        return []
    start = float(rows[0]["timestamp"])
    base = np.array([float(rows[0][key]) for key in POSITION_KEYS])
    previous = base.copy()
    integral = np.zeros(3)
    samples = []
    for row in rows:
        q = np.array([float(row[key]) for key in POSITION_KEYS])
        integral += .5*(forward_jacobian(previous)+forward_jacobian(q))*(q-previous)
        previous = q
        samples.append((row, q, integral.copy()))
        if abs(float(row["com_lean"])) > .30:
            break
    output = []
    for offset in (0., .2, .4, .6, .8, 1., 1.2, 1.6, 1.8, 2.):
        if offset > float(samples[-1][0]["timestamp"])-start:
            continue
        row, q, contribution = min(samples, key=lambda sample:
                                   abs(float(sample[0]["timestamp"])-start-offset))
        qdot = np.array([float(row[key]) for key in VELOCITY_KEYS])
        relative_velocity = forward_jacobian(q) @ qdot
        axle_velocity = float(row["capture_com_velocity"]) - relative_velocity
        wheel_axle_velocity = -model.R_WHEEL*(float(row["left_wheel_vel"])
                                  +qdot[0]+qdot[1]-qdot[2])
        forward = model.centroidal(q)[0]
        values = (path.parent.name, path.stem, offset, q[2], q[0], q[1], forward,
                  contribution[2], contribution[0], contribution[1],
                  contribution.sum()-(forward-model.centroidal(base)[0]),
                  row["capture_com_velocity"], axle_velocity, wheel_axle_velocity,
                  row["wheel_cmd_target"], row["capture_world_active"])
        output.append(dict(zip(FIELDS, values)))
    return output

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", type=Path, nargs="+")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for directory in args.directories:
            for path in sorted(directory.glob("*_log.csv")):
                writer.writerows(decompose(path))
    print(args.output)

if __name__ == "__main__":
    main()
