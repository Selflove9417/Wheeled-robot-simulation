#!/usr/bin/env python3
"""Estimate planar pitch-axis angular momentum from a flat-jump controller log.

Diagnostic only: the seven-body URDF model omits the 10 g IMU and any payload.
Only rows whose IMU and joint samples share a simulation stamp are used.
Wheel effort is unavailable from the velocity joint-state interface.
"""

import argparse
import csv
import math
from pathlib import Path


MASS = (9.5, 1.2, 0.8, 2.0, 1.2, 0.8, 2.0)
IXX = (0.159013 * 9.5 / 14.0, 0.017921, 0.013130, 0.006481,
       0.017921, 0.013130, 0.006481)


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


def scale(s, a):
    return s * a[0], s * a[1]


def perp(a):
    return -a[1], a[0]


def rotate(angle, a):
    c, s = math.cos(angle), math.sin(angle)
    return c * a[0] - s * a[1], s * a[0] + c * a[1]


def momentum(row):
    """Return body, leg, wheel-orbital, wheel-axial and total Hx [kg m²/s]."""
    q = [float(row[name]) for name in (
        "hip_pos_left", "knee_pos_left", "hip_pos_right", "knee_pos_right")]
    dq = [float(row[name]) for name in (
        "hip_vel_left", "knee_vel_left", "hip_vel_right", "knee_vel_right")]
    wheel_rate = [float(row[name]) for name in ("left_wheel_vel", "right_wheel_vel")]
    base_rate = -float(row["pitch_rate_raw"])  # pitch = -URDF roll

    positions = [(0.13261282, 0.05396677)]
    relative_velocities = [(0.0, 0.0)]
    link_rates = [base_rate]
    for side in range(2):
        hip_angle, knee_angle = q[2 * side:2 * side + 2]
        hip_rate, knee_rate = dq[2 * side:2 * side + 2]
        hip = (0.125, -0.07)
        thigh = rotate(hip_angle, (-0.13690699, -0.02116697))
        knee = rotate(hip_angle, (-0.29348091, -0.06220095))
        shank = rotate(hip_angle + knee_angle, (0.11538205, -0.08532288))
        wheel = rotate(hip_angle + knee_angle, (0.28210870, -0.19553796))
        positions.extend((add(hip, thigh), add(add(hip, knee), shank),
                          add(add(hip, knee), wheel)))
        relative_velocities.extend((
            scale(hip_rate, perp(thigh)),
            add(scale(hip_rate, perp(knee)), scale(hip_rate + knee_rate, perp(shank))),
            add(scale(hip_rate, perp(knee)), scale(hip_rate + knee_rate, perp(wheel))),
        ))
        link_rates.extend((base_rate + hip_rate, base_rate + hip_rate + knee_rate,
                           base_rate + hip_rate + knee_rate + wheel_rate[side]))

    total_mass = sum(MASS)
    com = tuple(sum(m * p[axis] for m, p in zip(MASS, positions)) / total_mass
                for axis in (0, 1))
    orbital = []
    axial = []
    for m, inertia, pos, relative_v, rate in zip(
            MASS, IXX, positions, relative_velocities, link_rates):
        velocity = add(scale(base_rate, perp(pos)), relative_v)
        rel_pos = (pos[0] - com[0], pos[1] - com[1])
        orbital.append(m * (rel_pos[0] * velocity[1] - rel_pos[1] * velocity[0]))
        axial.append(inertia * rate)

    body = orbital[0] + axial[0]
    leg = sum(orbital[i] + axial[i] for i in (1, 2, 4, 5))
    wheel_orbital = orbital[3] + orbital[6]
    wheel_axial = axial[3] + axial[6]
    return body, leg, wheel_orbital, wheel_axial, body + leg + wheel_orbital + wheel_axial


def sample(rows, target, max_offset=0.006):
    candidates = [row for row in rows
                  if abs(float(row["timestamp"]) - target) <= max_offset
                  and abs(float(row["imu_sample_stamp"]) -
                          float(row["joint_sample_stamp"])) <= 0.0011
                  and 0.0 <= float(row["timestamp"]) -
                      float(row["imu_sample_stamp"]) <= 0.020
                  and 0.0 <= float(row["timestamp"]) -
                      float(row["joint_sample_stamp"]) <= 0.020]
    return min(candidates, key=lambda row: abs(float(row["timestamp"]) - target)) if candidates else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.logs:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        flight = next((row for row in rows if row["state_name"] == "FLIGHT"), None)
        if flight is None:
            print(f"{path.name}: UNAVAILABLE (no FLIGHT rows)")
            continue
        t0 = float(flight["timestamp"])
        print(f"{path.name}: FLIGHT={t0:.3f}s")
        for delta in (-0.050, -0.025, 0.0, 0.025, 0.050, 0.100, 0.150, 0.200):
            row = sample(rows, t0 + delta)
            if row is None:
                print(f"  {delta:+.3f}s UNAVAILABLE (no stamp-aligned sample)")
                continue
            body, leg, wheel_orbital, wheel_axial, total = momentum(row)
            actual_delta = float(row["timestamp"]) - t0
            print(f"  target={delta:+.3f}s actual={actual_delta:+.3f}s "
                  f"t={float(row['timestamp']):.3f} "
                  f"body={body:+.4f} leg={leg:+.4f} "
                  f"wheel_orbital={wheel_orbital:+.4f} "
                  f"wheel_axial={wheel_axial:+.4f} total={total:+.4f}")


if __name__ == "__main__":
    main()
