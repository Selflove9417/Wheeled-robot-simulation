#!/usr/bin/env python3
"""Summarize simulation-only flight probe sidecars without claiming causal sign from noisy pairs."""

import argparse
import csv
import statistics
from pathlib import Path

MODES = ("baseline", "wheel_pos", "wheel_neg", "hip_pos", "hip_neg", "knee_pos", "knee_neg")


def interpolate(rows, time, key):
    samples = sorted((float(row["flight_elapsed"]), float(row[key])) for row in rows)
    before = max((p for p in samples if p[0] <= time), default=None)
    after = min((p for p in samples if p[0] >= time), default=None)
    if before is None or after is None:
        raise ValueError(f"no bracket for t={time:.3f}")
    if before[0] == after[0]:
        return before[1]
    return before[1] + (after[1] - before[1]) * (time - before[0]) / (after[0] - before[0])


def read_trial(path, mode):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 25 or {row["mode"] for row in rows} != {mode}:
        raise ValueError("missing or mismatched flight records")
    active = [row for row in rows if row["active"] == "1"]
    if mode == "baseline":
        if active or any(abs(float(row["applied_wheel_delta"])) > 1e-9 or
                         abs(float(row["applied_leg_delta"])) > 1e-9 for row in rows):
            raise ValueError("baseline was perturbed")
        applied = 0.0
    else:
        field = "applied_wheel_delta" if mode.startswith("wheel") else "applied_leg_delta"
        applied = statistics.mean(float(row[field]) for row in active) if active else 0.0
        threshold = 0.045 if mode.startswith("wheel") else 0.30
        if len(active) < 5 or abs(applied) < threshold:
            raise ValueError(f"pulse not applied (rows={len(active)}, mean={applied:.4f})")
        if any(abs(float(row["applied_leg_delta"])) > 1e-9 for row in rows) and mode.startswith("wheel"):
            raise ValueError("wheel probe also altered leg torque")
        if any(abs(float(row["applied_wheel_delta"])) > 1e-9 for row in rows) and not mode.startswith("wheel"):
            raise ValueError("leg probe also altered wheel command")
    rate_04 = interpolate(rows, 0.04, "pitch_rate")
    rate_08 = interpolate(rows, 0.08, "pitch_rate")
    rate_12 = interpolate(rows, 0.12, "pitch_rate")
    return {
        "corrected_rate_change": (rate_12 - rate_08) - (rate_08 - rate_04),
        "pitch_08": interpolate(rows, 0.08, "pitch"),
        "rate_08": rate_08,
        "applied": applied,
        "wheel_velocity_change": interpolate(rows, 0.12, "wheel_vel_left") - interpolate(rows, 0.08, "wheel_vel_left"),
        "hip_velocity_change": interpolate(rows, 0.12, "hip_vel_left") - interpolate(rows, 0.08, "hip_vel_left"),
        "knee_velocity_change": interpolate(rows, 0.12, "knee_vel_left") - interpolate(rows, 0.08, "knee_vel_left"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_dir", type=Path)
    args = parser.parse_args()
    trials = {}
    for rep in (1, 2, 3):
        for mode in MODES:
            files = sorted((args.campaign_dir / f"rep_{rep}" / mode).glob("*_flight_probe.csv"))
            if not files:
                print(f"rep {rep} {mode}: INVALID (no probe file)")
                continue
            trial = None
            errors = []
            for path in reversed(files):
                try:
                    trial = read_trial(path, mode)
                    break
                except (ValueError, KeyError) as exc:
                    errors.append(str(exc))
            if trial is None:
                print(f"rep {rep} {mode}: INVALID ({'; '.join(errors)})")
                continue
            trials[(rep, mode)] = trial
            print(f"rep {rep} {mode}: applied={trial['applied']:+.3f}, "
                  f"corrected Δrate={trial['corrected_rate_change']:+.3f} rad/s, "
                  f"Δwheel={trial['wheel_velocity_change']:+.2f}, "
                  f"Δhip={trial['hip_velocity_change']:+.2f}, Δknee={trial['knee_velocity_change']:+.2f}, "
                  f"entry rate={trial['rate_08']:+.3f}")
    for channel in ("wheel", "hip", "knee"):
        effects = []
        for rep in (1, 2, 3):
            positive = trials.get((rep, channel + "_pos"))
            negative = trials.get((rep, channel + "_neg"))
            baseline = trials.get((rep, "baseline"))
            if not positive or not negative or not baseline:
                continue
            if abs(positive["pitch_08"] - negative["pitch_08"]) > 0.06 or \
               abs(positive["rate_08"] - negative["rate_08"]) > 0.35:
                print(f"{channel} rep {rep}: unmatched initial state")
                continue
            effects.append(positive["corrected_rate_change"] -
                           negative["corrected_rate_change"])
        if len(effects) != 3:
            print(f"{channel}: INCONCLUSIVE ({len(effects)}/3 matched pairs)")
            continue
        mean = statistics.mean(effects)
        spread = statistics.stdev(effects)
        consistent = all(value > 0 for value in effects) or all(value < 0 for value in effects)
        strong = consistent and abs(mean) > 3.0 * spread / (len(effects) ** 0.5)
        print(f"{channel}: {'REPEATABLE' if strong else 'INCONCLUSIVE'} "
              f"paired Δrate={mean:+.3f} rad/s, SD={spread:.3f}, n={len(effects)}")


if __name__ == "__main__":
    main()
