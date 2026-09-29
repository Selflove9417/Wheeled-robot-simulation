#!/usr/bin/env python3
"""第一阶段 MPC 稳态位置残差（R = 8）的根因分析。

只用数据和代数回答一个问题，不改控制器：为什么 0.30 m 参考阶跃后
x_error 稳在约 -0.085 m（静态约 -0.011 m）而不收敛到零？

用可测量的特征区分两种假设。参考固定时状态误差严格满足 d(e_x)/dt = e_x_dot：

  慢模态衰减：e_x_dot ~ -e_x / tau，即误差仍在变化，
              tau 应匹配 Ad - Bd*K 的最慢闭环极点
  真实偏移：  e_x_dot -> 0 而 e_x 不变，说明记录状态的平衡点不是参考值，
              指向估计（轮式里程计）偏差或扰动/平衡点建模问题。

用法：
    python3 analyze_mpc_position_residual.py --base /tmp/mpc_r_sweep_full --r 8
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import eigvals

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import verify_lqr_model_and_sweep as model  # noqa: E402

COLUMNS = ("time", "x", "x_ref", "x_error", "x_dot", "v_ref", "pitch", "pitch_rate",
           "theta_eq", "theta_error", "u_mpc", "u_lqr_fallback", "tau_each",
           "total_torque_saturated", "height", "control_enabled")


def load(path):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle)
                if row.get("stage") not in (None, "")
                # 节点每个周期写一行，teardown 时可能被截断，
                # 因此任何缺少分析字段的行直接丢弃而不猜测补全。
                and all(row.get(key) not in (None, "") for key in COLUMNS)]
    array = {}
    for key in COLUMNS:
        array[key] = np.array([float(row[key]) for row in rows])
    array["stage_list"] = [row["stage"] for row in rows]
    return array


def step_time(array):
    reference = array["x_ref"]
    jumps = np.flatnonzero(np.abs(np.diff(reference)) > 1.0e-3)
    return float(array["time"][jumps[0]]) if jumps.size else None


def window(array, start, end):
    times = array["time"]
    inside = (times >= start) & (times <= end)
    return {key: array[key][inside] for key in COLUMNS}


def describe(label, sample):
    tail = slice(-400, None) if len(sample["time"]) > 400 else slice(None)
    print(f"    {label}: n={len(sample['time'])} "
          f"t=[{sample['time'][0]:.2f},{sample['time'][-1]:.2f}] "
          f"e_x={np.mean(sample['x_error']):+.5f} m "
          f"(first {sample['x_error'][0]:+.5f}, last {sample['x_error'][-1]:+.5f}) "
          f"e_xdot={np.mean(sample['x_dot']):+.5f} m/s "
          f"e_theta={np.degrees(np.mean(sample['theta_error'])):+.4f} deg "
          f"u={np.mean(sample['u_mpc']):+.4f} Nm "
          f"|u|_max={np.max(np.abs(sample['u_mpc'])):.3f} Nm "
          f"sat={np.mean(sample['total_torque_saturated']):.4f}")
    return sample


def slope_per_second(times, values):
    """最小二乘拟合 d(values)/d(time)（SI 单位），并返回线性拟合的 R^2。"""
    if times.size < 10:
        return float("nan"), float("nan")
    coefficients = np.polyfit(times, values, 1)
    predicted = np.polyval(coefficients, times)
    residual = np.sum((values - predicted) ** 2)
    total = np.sum((values - np.mean(values)) ** 2)
    return float(coefficients[0]), float(1.0 - residual / total) if total > 0 else float("nan")


def analyse_runs(base, r_label, trial):
    directory = Path(base) / f"R{r_label}"
    paths = [directory / f"{trial}_rep{index}.csv" for index in range(1, 4)]
    results = []
    for path in paths:
        if not path.exists():
            print(f"  MISSING {path}")
            continue
        array = load(path)
        controlled = array["time"][array["control_enabled"] == 1.0]
        print(f"\n  {path.name}: rows={array['time'].size} "
              f"first_controlled={controlled[0]:.3f} s end={array['time'][-1]:.2f} s")
        step = step_time(array)
        if trial == "position":
            jump = int(np.flatnonzero(np.abs(np.diff(array["x_ref"])) > 1.0e-3)[0]) + 1
            after_step = slice(jump, None)
            print(f"    reference step at t={step:.3f} s: x_ref {array['x_ref'][jump-1]:+.4f} -> "
                  f"{array['x_ref'][-1]:+.4f} m; held to end within "
                  f"{np.max(np.abs(array['x_ref'][after_step] - array['x_ref'][-1])):.2e} m; "
                  f"v_ref max after step = {np.max(np.abs(array['v_ref'][after_step])):.4f} m/s")
            # 直接拟合 e_x 的实测衰减：若残差只是未完成的暂态，
            # 其时间常数应等于最慢闭环极点（model_check() 用节点相同的 Ad/Bd/Q/R 计算）。
            times, values = array["time"], array["x_error"]
            mask = (times >= step + 1.0) & (np.abs(values) > 1.0e-4)
            slope, goodness = slope_per_second(times[mask], np.log(np.abs(values[mask])))
            if slope < 0:
                print(f"    measured e_x decay over [step+1, end]: tau = {-1.0/slope:.2f} s "
                      f"(R^2={goodness:.3f}, n={int(mask.sum())})")
            for start, end, tag in ((step, step + 1.0, "during step"),
                                    (step + 1.0, step + 3.0, "response 1-3 s"),
                                    (step + 3.0, step + 8.0, "settled 3-8 s"),
                                    (step + 8.0, array["time"][-1], "settled 8-end")):
                sample = describe(tag, window(array, start, end))
                gradient, goodness = slope_per_second(sample["time"], sample["x_error"])
                print(f"        d(e_x)/dt = {gradient:+.5f} m/s (R^2={goodness:.3f}), "
                      f"-e_x/tau expected {(-np.mean(sample['x_error'])/7.07):+.5f} m/s")
            print("    every 2 s after the step: e_x / e_x_dot / e_theta(deg) / u")
            for offset in range(0, int(array["time"][-1] - step), 2):
                sample = window(array, step + offset, step + offset + 0.05)
                if sample["time"].size == 0:
                    continue
                print(f"        t=step+{offset:>2d}s e_x={sample['x_error'][0]:+.4f} "
                      f"e_xdot={sample['x_dot'][0]:+.4f} "
                      f"e_theta={np.degrees(sample['theta_error'][0]):+.3f} "
                      f"u={sample['u_mpc'][0]:+.3f}")
        else:
            ramp_end = array["time"][np.flatnonzero(array["height"] < array["height"].max() - 1e-6)][-1]
            for start, end, tag in ((0.0, ramp_end + 0.5, "startup+ramp"),
                                    (ramp_end + 0.5, ramp_end + 6.0, "post-ramp 0-6 s"),
                                    (array["time"][-1] - 6.0, array["time"][-1], "last 6 s")):
                sample = describe(tag, window(array, start, end))
                gradient, goodness = slope_per_second(sample["time"], sample["x_error"])
                print(f"        d(e_x)/dt = {gradient:+.5f} m/s (R^2={goodness:.3f})")
        results.append({"file": path.name, "step": step})
    return results


def model_check(r_values):
    print("\n=== model: x(k+1) = Ad x(k) + Bd u(k) at H = 0.40 m, Ts = %.3f s ===" % model.TS)
    A, B = model.design_matrices(0.40)[:2]
    Ad, Bd = model.discretize(A, B)
    print("Ad =\n%s" % np.array2string(Ad, precision=9, suppress_small=True))
    print("Bd = %s" % np.array2string(Bd, precision=9, suppress_small=True))
    singular_values = np.linalg.svd(Ad - np.eye(4), compute_uv=False)
    rank = int(np.linalg.matrix_rank(Ad - np.eye(4)))
    print(f"\nrank(Ad - I) = {rank}/4, singular values = {singular_values}")
    null_space = np.linalg.svd(Ad - np.eye(4))[2][rank:]
    print(f"null(Ad - I) basis (rows):\n{np.array2string(null_space, precision=6)}")
    print("=> a state with e_x_dot = e_theta = e_theta_dot = 0 and any constant e_x "
          "is an EXACT equilibrium of the prediction model at u = 0."
          if null_space.size and abs(null_space[0][1]) < 1e-9 and abs(null_space[0][0]) > 0.9
          else "=> position is NOT a free coordinate of the model; check the derivation.")

    print("\n| lambda | of the slowest closed-loop modes and the implied time constant:")
    for r_value in r_values:
        Q, R = model.Q0, float(r_value)
        K = model.dare_gain(Ad, Bd, Q, R)
        poles = eigvals(Ad - Bd.reshape(4, 1) @ K.reshape(1, 4))
        magnitude = np.sort(np.abs(poles))[::-1]
        slowest = magnitude[0]
        tau = -model.TS / np.log(slowest) if slowest < 1.0 else float("inf")
        print(f"  R={r_value:>4}: |lambda| sorted = {np.array2string(magnitude, precision=6)} "
              f"-> slowest tau = {tau:.2f} s; K = {np.array2string(K, precision=3)}")
    return Ad, Bd


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="/tmp/mpc_r_sweep_full")
    parser.add_argument("--r", default="8")
    parser.add_argument("--r-values", default="2,4,6,8,12")
    args = parser.parse_args()

    print(f"=== R={args.r} static trials ===")
    analyse_runs(args.base, args.r, "static")
    print(f"\n=== R={args.r} position trials ===")
    analyse_runs(args.base, args.r, "position")
    model_check([float(item) for item in args.r_values.split(",")])
    return 0


if __name__ == "__main__":
    sys.exit(main())
