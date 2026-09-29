#!/usr/bin/env python3
"""长时窗位置验证：编码器 x 对比 Gazebo 真值位姿。

只读。回答 22 s 试验无法区分的两个问题：

  1. 0.30 m 阶跃后约 50 s，位置误差是否真的收敛到零，还是趋于非零渐近线？
     -> 拟合 e_x(t) = c0 + c1 exp(-t/tau1) + c2 exp(-t/tau2)，
        报告 c0 及其标准误，并用最后窗口的均值/斜率作无模型校验。
  2. 轮式里程计 x 与仿真器位姿之间是否存在比例或固定偏移？
     -> 在受控窗口内最小二乘拟合 gt_displacement = a * x_encoder + b，
        并计算 0.30 m 阶跃实际走过的位移。

前进方向的世界轴从数据中识别（日志同时含 gt_pose_x 和 gt_pose_y）；
不假设 URDF 使用哪个轴。

用法：
    python3 analyze_mpc_long_horizon_position.py --dir /tmp/mpc_longhorizon_pos
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import curve_fit

ENCODER = "x"
CANDIDATE_AXES = ("gt_pose_y", "gt_pose_x")


def load(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        rows = [row for row in reader
                if row.get("stage") not in (None, "")
                and all(row.get(key) not in (None, "") for key in columns)]
    data = {}
    for key in columns:
        if key == "stage":
            continue
        try:
            data[key] = np.array([float(row[key]) for row in rows])
        except ValueError:
            continue
    return data, [row["stage"] for row in rows]


def pick_forward_axis(data, encoder):
    """选择行程与编码器估计最匹配的轴。"""
    span = encoder.max() - encoder.min()
    best = None
    for key in CANDIDATE_AXES:
        if key not in data:
            continue
        axis = data[key]
        ratio = (axis.max() - axis.min()) / span if span > 1e-3 else 0.0
        correlation = float(np.corrcoef(axis, encoder)[0, 1]) if span > 1e-3 else 0.0
        score = abs(correlation)
        if best is None or score > best["score"]:
            best = {"field": key, "score": score, "range_m": round(float(axis.max() - axis.min()), 4),
                    "travel_over_encoder_span": round(ratio, 4), "correlation_with_encoder": round(correlation, 4)}
    return best


def fit_asymptote(times, values, step):
    """e_x(t) = c0 + A exp(-(t - step)/tau)，三参数，渐近线自由。

    先试过双指数拟合，但由于响应由单一慢模态主导，
    时间常数不可辨识（tau2 为负）；三参数才是这里的合理模型。
    """
    t = times - step
    mask = t > 1.0
    tt, vv = t[mask], values[mask]
    if tt.size < 50:
        return {"fit_failed": "not enough samples after the step"}

    def model(time, c0, amplitude, tau):
        return c0 + amplitude * np.exp(-time / tau)

    try:
        popt, pcov = curve_fit(model, tt, vv, p0=[0.0, vv[0], 7.0], maxfev=200000)
        perr = np.sqrt(np.diag(pcov))
        prediction = model(tt, *popt)
        return {"c0_m": round(float(popt[0]), 6), "c0_stderr_m": round(float(perr[0]), 6),
                "amplitude_m": round(float(popt[1]), 5), "tau_s": round(float(popt[2]), 3),
                "tau_stderr_s": round(float(perr[2]), 3),
                "fit_rms_m": round(float(np.sqrt(np.mean((vv - prediction) ** 2))), 5),
                "samples": int(tt.size)}
    except (RuntimeError, ValueError) as error:
        return {"fit_failed": str(error)[:120]}


def analyse(path):
    data, stages = load(path)
    if "gt_valid" not in data:
        return {"file": path.name, "error": "log has no gt_valid column (old binary?)"}
    valid = data["gt_valid"] > 0.5
    controlled = data["time"][data["control_enabled"] == 1.0]
    reference = data["x_ref"]
    jump = np.flatnonzero(np.abs(np.diff(reference)) > 1.0e-3)
    step = float(data["time"][jump[0]]) if jump.size else None
    out = {
        "file": path.name,
        "rows": int(data["time"].size),
        "log_seconds": round(float(data["time"][-1]), 2),
        "first_controlled_s": round(float(controlled[0]), 3) if controlled.size else None,
        "tripped_safety_gate": any(stage == "safety_disabled" for stage in stages),
        "gt_valid_rows": int(valid.sum()),
        "gt_valid_fraction": round(float(valid.mean()), 4),
        "step_time_s": step,
        "post_step_observation_s": round(float(data["time"][-1] - step), 2) if step else None,
    }
    if valid.sum() < 100 or step is None:
        out["error"] = "insufficient ground-truth samples or no step found"
        return out

    encoder = data[ENCODER]
    out["forward_axis"] = pick_forward_axis(data, encoder)
    axis_field = out["forward_axis"]["field"]
    truth = data[axis_field]
    base = float(truth[valid][0])
    truth_disp = truth - base                      # 米，世界坐标系
    gt_ok = valid

    # 受控窗口内的里程计关系
    window = gt_ok & (data["control_enabled"] == 1.0)
    slope, intercept = np.polyfit(encoder[window], truth_disp[window], 1)
    residual = truth_disp[window] - (slope * encoder[window] + intercept)
    out["gt_vs_encoder"] = {
        "proportional_slope": round(float(slope), 5),
        "fixed_offset_m": round(float(intercept), 5),
        "residual_rms_m": round(float(np.sqrt(np.mean(residual ** 2))), 5),
        "encoder_travel_m": round(float(encoder[window].max() - encoder[window].min()), 4),
        "truth_travel_m": round(float(truth_disp[window].max() - truth_disp[window].min()), 4),
    }

    e_enc = data["x_error"]
    late = gt_ok & (data["time"] >= step + 25.0) & (data["time"] <= step + 35.0)
    tail = gt_ok & (data["time"] >= step + 45.0)
    for tag, mask in (("25-35 s", late), ("after 45 s", tail)):
        if mask.sum() < 20:
            continue
        # 参考以编码器米为单位，世界系下的误差为
        # truth_disp - (slope * x_ref + intercept)。
        truth_error = truth_disp[mask] - (slope * data["x_ref"][mask] + intercept)
        out[f"error_{tag}"] = {
            "encoder_mean_m": round(float(np.mean(e_enc[mask])), 5),
            "encoder_std_m": round(float(np.std(e_enc[mask])), 5),
            "encoder_slope_m_per_s": round(float(np.polyfit(data["time"][mask], e_enc[mask], 1)[0]), 6),
            "truth_mean_m": round(float(np.mean(truth_error)), 5),
            "truth_std_m": round(float(np.std(truth_error)), 5),
            "truth_absmax_m": round(float(np.max(np.abs(truth_error))), 5),
            "samples": int(mask.sum()),
        }
    out["asymptote_fit"] = fit_asymptote(data["time"], e_enc, step)
    hold = gt_ok & (data["time"] >= step + 45.0)
    if hold.sum() > 20:
        out["holding_state_after_45s"] = {
            "pitch_error_mean_deg": round(float(np.degrees(np.mean(data["theta_error"][hold]))), 4),
            "x_dot_mean_mps": round(float(np.mean(data["x_dot"][hold])), 5),
            "u_mean_Nm": round(float(np.mean(data["u_mpc"][hold])), 4),
            "u_absmax_Nm": round(float(np.max(np.abs(data["u_mpc"][hold]))), 4),
            "saturated_fraction": round(float(np.mean(data["total_torque_saturated"][hold])), 4),
            "band_binding_fraction": round(float(np.mean(data["theta_band_binding"][hold])), 4),
            "height_m": round(float(np.mean(data["height"][hold])), 4),
            "theta_eq_rad": round(float(np.mean(data["theta_eq"][hold])), 6),
        }
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="/tmp/mpc_longhorizon_pos")
    args = parser.parse_args()
    directory = Path(args.dir)
    results = []
    for path in sorted(directory.glob("position_rep*.csv")):
        entry = analyse(path)
        results.append(entry)
        print(json.dumps(entry, ensure_ascii=False, indent=2))
    (directory / "long_horizon_analysis.json").write_text(json.dumps(results, ensure_ascii=False, indent=2))
    print(f"\nwrote {directory / 'long_horizon_analysis.json'} ({len(results)} reps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
