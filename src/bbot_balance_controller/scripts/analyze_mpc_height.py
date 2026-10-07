#!/usr/bin/env python3
"""核对高度调度 Linear MPC 日志里与高度有关的量。

这些检查回答三个问题：模型用的是不是 current_height、theta_eq 用的是不是 adaptive
的规则（先插 y_com/z_com 再取 -atan2）、腿部有没有真的走到指令高度。全部只读控制器
自己的 CSV，其中 `gt_pose_z` 是 gz 的 base_link 位姿真值（只记录，不进控制律），
髋-轴高度的真值换算为 gt_pose_z - (base_to_hip + wheel_radius) = gt_pose_z - 0.14。

用法:
    python3 analyze_mpc_height.py <csv> [<csv> ...] [--json out.json]
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

# adaptive_lqr_balance_controller.cpp:300-312 的 y_com / z_com 列。
GAIN_TABLE = [
    (0.3000, -0.0276264, 0.3592154),
    (0.3500, -0.0258066, 0.4016316),
    (0.4000, -0.0234056, 0.4443432),
    (0.4500, -0.0203401, 0.4872441),
    (0.5000, -0.0164269, 0.5302655)]

BASE_TO_HIP_PLUS_WHEEL = 0.14      # height.base_to_hip 0.07 + wheel_radius 0.07
TAIL_WINDOW_S = 4.0


def table_theta_eq(height):
    """与控制器/adaptive 相同：先按高度线性插值 y_com / z_com，再取 -atan2。"""
    if height <= GAIN_TABLE[0][0]:
        _, y_com, z_com = GAIN_TABLE[0]
    elif height >= GAIN_TABLE[-1][0]:
        _, y_com, z_com = GAIN_TABLE[-1]
    else:
        for index in range(len(GAIN_TABLE) - 1):
            low = GAIN_TABLE[index]
            high = GAIN_TABLE[index + 1]
            if low[0] <= height <= high[0]:
                ratio = (height - low[0]) / (high[0] - low[0])
                y_com = low[1] + (high[1] - low[1]) * ratio
                z_com = low[2] + (high[2] - low[2]) * ratio
                break
    return -np.arctan2(y_com, z_com)


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        return [row for row in reader
                if row.get("stage") not in (None, "")
                and all(row.get(key) not in (None, "") for key in columns)]


def column(rows, key):
    return np.array([float(row[key]) for row in rows if row.get(key) not in (None, "")])


def analyse(path):
    rows = read_rows(path)
    if not rows:
        return {"csv": str(path), "error": "no complete rows"}
    engaged = [row for row in rows if row["stage"] != "disabled"]
    if not engaged:
        return {"csv": str(path), "error": "no controlled rows"}

    time = column(engaged, "time")
    current = column(engaged, "height")
    target = column(engaged, "target_height")
    model = column(engaged, "model_height")
    theta_eq = column(engaged, "theta_eq")
    rebuild_us = column(engaged, "model_rebuild_us")
    rebuild_failed = column(engaged, "model_rebuild_failed")
    truth_z = column(engaged, "gt_pose_z")
    theta_error = column(engaged, "theta_error")
    x_error = column(engaged, "x_error")
    u_mpc = column(engaged, "u_mpc")
    tau_each = column(engaged, "tau_each")
    saturated = column(engaged, "total_torque_saturated")
    wheel_saturated = column(engaged, "wheel_torque_saturated")
    iterations = column(engaged, "solver_iterations")
    solver_us = column(engaged, "solver_time_us")
    stages = sorted({row["stage"] for row in engaged})

    # theta_eq 是否真的按 adaptive 的规则随 current_height 走。
    theta_rule_gap = float(np.max(np.abs(theta_eq - np.array(
        [table_theta_eq(value) for value in current]))))

    # 模型高度与指令高度的偏差：frozen scheduling 下应当逐拍为零（重建失败才非零）。
    model_gap = np.abs(model - current)
    tail = engaged[-int(round(TAIL_WINDOW_S / 0.005)):] if len(engaged) > 40 else engaged
    tail_current = column(tail, "height")
    tail_target = column(tail, "target_height")
    tail_truth = column(tail, "gt_pose_z")
    leg_truth_error = float(np.mean(tail_truth - BASE_TO_HIP_PLUS_WHEEL - tail_current)) \
        if tail_truth.size and np.any(tail_truth != 0.0) else None

    moving = rebuild_us > 0.0
    return {
        "csv": str(path),
        "rows": len(rows),
        "controlled_rows": len(engaged),
        "sim_span_s": [round(float(time[0]), 3), round(float(time[-1]), 3)],
        "stages": stages,
        "fallback_rows": int(sum(1 for row in engaged if row["stage"] != "solved")),
        "target_height_first_last": [round(float(target[0]), 4), round(float(target[-1]), 4)],
        "current_height_min_max": [round(float(current.min()), 4), round(float(current.max()), 4)],
        "height_reach_error_m": round(float(tail_current[-1] - tail_target[-1]), 6),
        "leg_truth_minus_commanded_m": None if leg_truth_error is None
        else round(leg_truth_error, 5),
        "model_vs_current_max_m": round(float(model_gap.max()), 7),
        "model_vs_current_nonzero_rows": int(np.count_nonzero(model_gap > 1.0e-9)),
        "theta_rule_max_gap_rad": theta_rule_gap,
        "theta_error_rms_deg": round(float(np.sqrt(np.mean(theta_error ** 2)) * 180.0 / np.pi), 4),
        "theta_error_absmax_deg": round(float(np.abs(theta_error).max() * 180.0 / np.pi), 4),
        "x_error_rms_m": round(float(np.sqrt(np.mean(x_error ** 2))), 5),
        "x_error_absmax_m": round(float(np.abs(x_error).max()), 5),
        "u_rms_Nm": round(float(np.sqrt(np.mean(u_mpc ** 2))), 4),
        "u_absmax_Nm": round(float(np.abs(u_mpc).max()), 4),
        "tau_each_absmax_Nm": round(float(np.abs(tau_each).max()), 4),
        "saturated_fraction": round(float(np.mean(saturated)), 5),
        "wheel_saturated_fraction": round(float(np.mean(wheel_saturated)), 5),
        "solver_mean_us": round(float(solver_us[moving].mean()), 1) if moving.any() else None,
        "solver_iterations_max": int(iterations.max()),
        "rebuild_us_mean": round(float(rebuild_us[moving].mean()), 1) if moving.any() else None,
        "rebuild_us_p95": round(float(np.percentile(rebuild_us[moving], 95)), 1)
        if moving.any() else None,
        "rebuild_us_max": round(float(rebuild_us[moving].max()), 1) if moving.any() else None,
        "rebuild_failed_rows": int(rebuild_failed.sum()),
        "dt_mean_s": round(float(np.mean(column(engaged, "dt"))), 6),
        "dt_max_s": round(float(np.max(column(engaged, "dt"))), 6),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_paths", nargs="+")
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    results = [analyse(path) for path in args.csv_paths]
    keys = ("target_height_first_last", "current_height_min_max", "height_reach_error_m",
            "leg_truth_minus_commanded_m", "model_vs_current_max_m",
            "model_vs_current_nonzero_rows", "theta_rule_max_gap_rad", "theta_error_rms_deg",
            "x_error_rms_m", "u_absmax_Nm", "saturated_fraction", "fallback_rows", "stages",
            "rebuild_us_mean", "rebuild_us_max", "rebuild_failed_rows", "dt_max_s")
    for item in results:
        print(f"\n### {Path(item['csv']).name}")
        if "error" in item:
            print(f"    {item['error']}")
            continue
        for key in keys:
            print(f"    {key:32s} {item[key]}")
    if args.json:
        Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"\njson -> {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
