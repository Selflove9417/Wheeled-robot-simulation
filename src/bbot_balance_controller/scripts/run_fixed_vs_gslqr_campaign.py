#!/usr/bin/env python3
"""Matched fixed-midpoint LQR versus scheduled GS-LQR formal campaign."""

import argparse
import json
import time
from pathlib import Path

from run_formal_pid_gslqr_campaign import (
    HEIGHTS, move_invalid, run_one, trial_tag, write_csv,
)

CONTROLLERS = ("fixed_lqr", "gs_lqr")


def trials(smoke=False):
    if smoke:
        return [
            {"job": "push", "height": 0.30, "force": 20.0, "rep": 1},
            {"job": "lift", "height": 0.30, "force": 0.0, "speed": 0.15, "rep": 1},
        ]
    return [
        {"job": "push", "height": height, "force": force, "rep": rep}
        for height in HEIGHTS for force in (20.0, -20.0)
        for rep in range(1, 4)
    ] + [
        {"job": "lift", "height": 0.30, "force": 0.0, "speed": 0.15, "rep": rep}
        for rep in range(1, 4)
    ]


def save_results(root, results):
    (root / "formal_results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    write_csv(results, root / "formal_results.csv")
    for controller in CONTROLLERS:
        subset = [row for row in results if row["controller"] == controller]
        if subset:
            write_csv(subset, root / controller / "formal_results.csv")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--ws-root", type=Path, default=Path("/home/admin/bbot_ws_new"))
    parser.add_argument("--startup-timeout", type=float, default=75.0)
    parser.add_argument("--force-duration", type=float, default=0.20)
    parser.add_argument("--force-axis", choices=("x", "y"), default="y")
    parser.add_argument("--world-name", default="balance_test_world")
    parser.add_argument("--link-name", default="base_link")
    parser.add_argument("--startup-height", type=float, default=0.36)
    parser.add_argument("--pid-gains-file", default="/home/admin/bbot_ws_new/src/bbot_balance_controller/config/position_torque_cascade_pid_gains.yaml")
    parser.add_argument("--gs-config-file", default="/home/admin/bbot_ws_new/src/bbot_balance_controller/config/gs_lqr_historical_experiment.yaml")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    args.lift_stable_gated = True
    if not 0.19 <= args.force_duration <= 0.21:
        parser.error("force duration must be 0.19-0.21 s")
    root = args.output_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    for controller in CONTROLLERS:
        (root / controller / "launch_logs").mkdir(parents=True, exist_ok=True)
    result_path = root / "formal_results.json"
    if result_path.exists() and not args.resume:
        parser.error("output already contains results; use --resume")
    results = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else []
    manifest_path = root / "comparison_manifest.json"
    if not manifest_path.exists():
        manifest_path.write_text(json.dumps({
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "controllers": {"fixed_lqr": "legacy_safe K(0.40 m), nominal equilibrium",
                            "gs_lqr": "legacy_safe five-node interpolated K(H), nominal equilibrium"},
            "pulse": {"forces_N": [20.0, -20.0], "duration_s": args.force_duration,
                      "axis": args.force_axis, "heights_m": list(HEIGHTS)},
            "lift": {"heights_m": [0.30, 0.50, 0.30], "speed_mps": 0.15,
                     "high_hold_s": 10.0, "post_return_observation_s": 10.0,
                     "static_gate_s": 5.0},
            "repetitions_per_condition": 3,
            "world": args.world_name, "link": args.link_name,
            "smoke": args.smoke,
        }, ensure_ascii=False, indent=2), encoding="utf-8")
    completed = {(row["controller"], row["trial_tag"]) for row in results}
    for trial in trials(args.smoke):
        for controller in CONTROLLERS:
            tag = trial_tag(controller, trial)
            if (controller, tag) in completed:
                continue
            print(f"\n[{len(results) + 1}] {tag}", flush=True)
            controller_root = root / controller
            for attempt in range(1, 4):
                result = run_one(args, controller, trial, controller_root, attempt)
                recording_error = result.get("fail_reason") in (
                    "no_csv_data", "controller_stopped_logging")
                if (result["protocol_valid"] and not recording_error) or not result["retryable_protocol_error"] or attempt == 3:
                    break
                move_invalid(controller_root, result)
                print(f"[Compare] protocol retry {tag}: {attempt + 1}/3", flush=True)
            results.append(result)
            completed.add((controller, tag))
            save_results(root, results)
            print(f"[Compare] {tag}: protocol={result['protocol_valid']} fail={result['fail_reason']}", flush=True)
    print(f"Saved {len(results)} results in {root}", flush=True)


if __name__ == "__main__":
    main()
