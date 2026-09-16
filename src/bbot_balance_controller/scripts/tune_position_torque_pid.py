#!/usr/bin/env python3
"""Bounded, staged tuning for the independent position PID.

This script deliberately tunes only the new outer position loop.  The accepted
velocity/attitude/rate gains are passed through unchanged.  It runs at most
eight physical trials (under the requested twelve-trial limit): two static P
candidates, two +/-10 N D candidates, and two static I candidates.  A frozen
parameter file is written only when every stage has at least one usable
candidate.
"""

import argparse
import csv
import json
import os
import subprocess
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RUNNER = os.path.join(SCRIPT_DIR, "run_position_pid_exploration.py")


def run_candidate(args, name, job, kp, ki, kd, force=20.0):
    data_dir = os.path.join(args.data_root, name)
    cmd = [
        sys.executable, RUNNER,
        "--job", job,
        "--height", "0.30",
        "--force", str(force),
        "--repeats", "1",
        "--data-dir", data_dir,
        "--position-kp", str(kp),
        "--position-ki", str(ki),
        "--position-kd", str(kd),
        "--startup-timeout", str(args.startup_timeout),
        "--ws-root", args.ws_root,
    ]
    print(f"[Tune] {name}: {' '.join(cmd)}", flush=True)
    completed = subprocess.run(cmd, cwd=args.ws_root, check=False)
    summary_path = os.path.join(data_dir, "exploration_summary.csv")
    rows = []
    if os.path.exists(summary_path):
        with open(summary_path, newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    for row in rows:
        for key in ("protocol_valid", "target_constant", "velocity_direction_ok"):
            if key in row:
                row[key] = row[key].lower() in ("1", "true")
        for key, value in list(row.items()):
            if key not in ("trial_tag", "fail_reason", "trial", "protocol_valid",
                           "target_constant", "velocity_direction_ok"):
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    pass
    return {
        "name": name,
        "job": job,
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "returncode": completed.returncode,
        "rows": rows,
        "data_dir": data_dir,
    }


def usable(candidate):
    if candidate["returncode"] != 0 or not candidate["rows"]:
        return False
    for row in candidate["rows"]:
        if not row.get("protocol_valid", False):
            return False
        if row.get("fail_reason") not in (None, "", "None"):
            return False
    return True


def score(candidate, stage):
    rows = candidate["rows"]
    values = []
    for row in rows:
        if stage == "P":
            values.append(
                float(row.get("position_rms_mm", 1e6))
                + 0.25 * float(row.get("pitch_peak_deg", 1e6))
                + 25.0 * float(row.get("torque_saturation_ratio", 1e6)))
        elif stage == "D":
            values.append(
                float(row.get("position_10mm_recovery_s", 1e6))
                + 0.05 * float(row.get("position_peak_mm", 1e6))
                + 0.25 * float(row.get("pitch_peak_deg", 1e6))
                + 25.0 * float(row.get("torque_saturation_ratio", 1e6)))
        else:
            values.append(
                float(row.get("position_rms_mm", 1e6))
                + 0.25 * float(row.get("pitch_peak_deg", 1e6))
                + 25.0 * float(row.get("torque_saturation_ratio", 1e6))
                + 0.5 * float(row.get("position_integral_peak", 1e6)))
    return sum(values) / len(values) if values else 1e12


def choose(candidates, stage):
    valid = [candidate for candidate in candidates if usable(candidate)]
    if not valid:
        raise RuntimeError(f"stage {stage} produced no valid candidate; tuning stopped")
    return min(valid, key=lambda candidate: score(candidate, stage)), valid


def write_frozen_yaml(path, kp, ki, kd):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    content = f"""position_torque_cascade_pid_controller:
  ros__parameters:
    pid:
      position:
        kp: {kp:.9g}
        ki: {ki:.9g}
        kd: {kd:.9g}
        v_ref_limit: 0.40
        integral_limit: 2.0
      rate_limit_u: 0.0
      delta_theta_limit: 0.055
      total_torque_max: 20.0
      wheel_torque_max: 10.0
      low:
        kp_rate: 20.0
        kd_rate: 0.02
        kp_theta: 5.5
        kd_theta: 0.10
        kp_v: 0.08
        ki_v: 0.008
        kd_v: 0.001
      high:
        kp_rate: 22.0
        kd_rate: 0.025
        kp_theta: 6.0
        kd_theta: 0.12
        kp_v: 0.09
        ki_v: 0.008
        kd_v: 0.001
"""
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--data-root", default=None)
    parser.add_argument("--startup-timeout", type=float, default=75.0)
    args = parser.parse_args()
    if args.data_root is None:
        args.data_root = os.path.join(
            args.ws_root, "src/bbot_balance_controller/src/data_logs/position_torque_pid_tuning")
    os.makedirs(args.data_root, exist_ok=True)

    candidates = []
    # Stage P: static balance, integral disabled and derivative held at the
    # nominal damping value.
    stage_p = [
        run_candidate(args, "stage_P_kp_0p35", "constant", 0.35, 0.0, 0.30),
        run_candidate(args, "stage_P_kp_0p60", "constant", 0.60, 0.0, 0.30),
    ]
    candidates.extend(stage_p)
    best_p, valid_p = choose(stage_p, "P")
    kp = best_p["kp"]

    # Stage D: +/-10 N calibration push.  The runner validates both signs and
    # records protocol-invalid attempts separately.
    stage_d = [
        run_candidate(args, "stage_D_kd_0p15", "calibration_push", kp, 0.0, 0.15, 10.0),
        run_candidate(args, "stage_D_kd_0p45", "calibration_push", kp, 0.0, 0.45, 10.0),
    ]
    candidates.extend(stage_d)
    best_d, valid_d = choose(stage_d, "D")
    kd = best_d["kd"]

    # Stage I: static balance with the selected P and D values.
    stage_i = [
        run_candidate(args, "stage_I_ki_0p005", "constant", kp, 0.005, kd),
        run_candidate(args, "stage_I_ki_0p015", "constant", kp, 0.015, kd),
    ]
    candidates.extend(stage_i)
    best_i, valid_i = choose(stage_i, "I")
    ki = best_i["ki"]

    frozen_path = os.path.join(args.data_root, "position_torque_cascade_pid_gains_frozen.yaml")
    write_frozen_yaml(frozen_path, kp, ki, kd)
    result = {
        "status": "complete",
        "trial_count": len(candidates),
        "selected": {"kp": kp, "ki": ki, "kd": kd},
        "stage_best": {
            "P": {"name": best_p["name"], "score": score(best_p, "P")},
            "D": {"name": best_d["name"], "score": score(best_d, "D")},
            "I": {"name": best_i["name"], "score": score(best_i, "I")},
        },
        "frozen_yaml": frozen_path,
        "candidates": candidates,
    }
    with open(os.path.join(args.data_root, "tuning_result.json"), "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True, default=str)
    print(json.dumps({k: result[k] for k in ("status", "trial_count", "selected", "frozen_yaml")}, indent=2))


if __name__ == "__main__":
    main()
