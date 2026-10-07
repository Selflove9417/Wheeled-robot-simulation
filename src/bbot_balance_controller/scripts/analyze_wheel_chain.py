#!/usr/bin/env python3
"""Align flat-jump controller logs with independent wheel joint observations.

This reports observable command tracking; JointState.effort is never assumed to
be measured motor torque merely because the field exists.
"""

import argparse
import bisect
import csv
import math
import statistics
from pathlib import Path


WHEEL_RADIUS = 0.07  # controller's wheel_radius_, not the diff-drive config value
MAX_SAMPLE_AGE = 0.015


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(row, field):
    try:
        value = float(row[field])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def wheel_samples(rows, side):
    samples = []
    for row in rows:
        if row.get("side") != side:
            continue
        stamp = number(row, "stamp")
        velocity = number(row, "velocity")
        effort = number(row, "effort")
        if stamp is not None and stamp > 0 and velocity is not None:
            samples.append((stamp, velocity, effort))
    return sorted(samples)


def nearest(samples, stamps, time):
    index = bisect.bisect_left(stamps, time)
    candidates = [samples[i] for i in (index - 1, index) if 0 <= i < len(samples)]
    if not candidates:
        return None
    sample = min(candidates, key=lambda item: abs(item[0] - time))
    return sample if abs(sample[0] - time) <= MAX_SAMPLE_AGE else None


def analyze(log_path, wheel_path):
    controller = read_csv(log_path)
    joints = read_csv(wheel_path)
    flight = [row for row in controller if row.get("state_name") == "FLIGHT"]
    if len(flight) < 20:
        return {"status": "INVALID", "reason": "less than 20 FLIGHT rows"}
    start = number(flight[0], "timestamp")
    if start is None:
        return {"status": "INVALID", "reason": "missing FLIGHT timestamp"}
    window = [row for row in flight
              if (time := number(row, "timestamp")) is not None
              and start + 0.05 <= time <= start + 0.30
              and (number(row, "landing_wheel_ground_blend") or 0.0) <= 0.05]
    if len(window) < 15:
        return {"status": "INVALID", "reason": "less than 15 pure-air control rows"}
    sides = {side: wheel_samples(joints, side) for side in ("left", "right")}
    stamps = {side: [item[0] for item in samples] for side, samples in sides.items()}
    aligned = []
    for row in window:
        time = number(row, "timestamp")
        left = nearest(sides["left"], stamps["left"], time)
        right = nearest(sides["right"], stamps["right"], time)
        command = number(row, "cmd_x")
        if left is None or right is None or command is None:
            continue
        actual = WHEEL_RADIUS * (left[1] + right[1]) / 2.0
        aligned.append((time, row, command, actual, left, right))
    coverage = len(aligned) / len(window)
    if coverage < 0.90:
        return {"status": "INVALID", "reason": f"joint alignment coverage {coverage:.1%} < 90%"}
    efforts = [sample[2] for _, _, _, _, left, right in aligned
               for sample in (left, right) if sample[2] is not None]
    effort_coverage = len(efforts) / (2 * len(aligned))
    effort_dynamic = (effort_coverage >= 0.90 and
                      max((abs(value) for value in efforts), default=0.0) >= 0.05 and
                      max(efforts) - min(efforts) >= 0.05)
    errors = [abs(command - actual) for _, _, command, actual, _, _ in aligned]
    ages = [abs(time - sample[0]) for time, _, _, _, left, right in aligned
            for sample in (left, right)]
    first, last = aligned[0], aligned[-1]
    return {
        "status": "VALID", "flight_start": start,
        "window": (first[0] - start, last[0] - start),
        "coverage": coverage, "max_age_ms": max(ages) * 1000,
        "median_abs_tracking_error": statistics.median(errors),
        "large_error_fraction": sum(error > 0.15 for error in errors) / len(errors),
        "pitch_first": number(first[1], "pitch"), "pitch_last": number(last[1], "pitch"),
        "rate_first": number(first[1], "pitch_rate"), "rate_last": number(last[1], "pitch_rate"),
        "cmd_first": first[2], "cmd_last": last[2],
        "wheel_first": first[3], "wheel_last": last[3],
        "effort_coverage": effort_coverage,
        "effort_range": (min(efforts), max(efforts)) if efforts else None,
        "effort_dynamic": effort_dynamic,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_dir", type=Path)
    args = parser.parse_args()
    for log_path in sorted(args.campaign_dir.glob("*_log.csv")):
        wheel_path = log_path.with_name(log_path.name.replace("_log.csv", "_wheel_joints.csv"))
        if not wheel_path.exists():
            print(f"{log_path.name}: INVALID (missing wheel joint sidecar)")
            continue
        result = analyze(log_path, wheel_path)
        print(f"{log_path.name}: {result}")


if __name__ == "__main__":
    main()
