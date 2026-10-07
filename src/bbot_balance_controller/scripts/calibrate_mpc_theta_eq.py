#!/usr/bin/env python3
"""从无头 MPC 日志推导 MPC 名义平衡角 theta_eq。

控制器调节的是相对 theta_eq 的俯仰误差,错误的 theta_eq 相当于恒值扰动。对轮腿
平衡模型,闭环稳态为

    x_error_steady = (k_theta / k_x) * (theta_eq_used - p0)

其中 p0 是零力矩悬挂俯仰角。由此得到两种估计:

  direct   theta_eq := 运行安静段的 median(pitch)
           (仅当中位 |u| 较小时有效:机器人此时悬挂在自身固有角度,回路近乎空转)
  two-run  取两个不同 theta_eq 的日志,ratio = (x1 - x2) / (t1 - t2),
           p0 = t1 - x1 / ratio。ratio 是 k_theta / k_x 的独立测量,已部署的
           H = 0.40 行给出约 34。

用法:
  python3 calibrate_mpc_theta_eq.py run1.csv [run2.csv ...]
"""

import csv
import sys
from pathlib import Path

import numpy as np


def load(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle) if row["stage"] not in ("disabled",)]
    if len(rows) < 200:
        raise SystemExit(f"{path}: only {len(rows)} engaged rows, run the trial longer")
    numbers = lambda key: np.array([float(row[key]) for row in rows])  # noqa: E731
    return {
        "path": Path(path).name,
        "theta_eq": float(np.median(numbers("theta_eq"))),
        "pitch": numbers("pitch"),
        "x_error": numbers("x_error"),
        "u": numbers("u_mpc"),
        "time": numbers("time"),
    }


def quiet_tail(run, span_s=8.0):
    mask = run["time"] > run["time"][-1] - span_s
    return {key: float(np.median(run[key][mask])) for key in ("pitch", "x_error", "u")}


def main(paths):
    runs = [load(path) for path in paths]
    print(f"{'log':<26} {'theta_eq':>9} {'med pitch':>10} {'med x_err':>10} {'med |u|':>8}")
    for run in runs:
        tail = quiet_tail(run)
        print(f"{run['path']:<26} {run['theta_eq']:>9.4f} {tail['pitch']:>10.4f} "
              f"{tail['x_error']:>10.4f} {abs(tail['u']):>8.2f}")

    direct = [run for run in runs if abs(quiet_tail(run)["u"]) < 2.0]
    if direct:
        estimate = float(np.mean([quiet_tail(run)["pitch"] for run in direct]))
        print(f"\ndirect estimate  theta_eq = {estimate:.4f} rad "
              f"(from {len(direct)} quiet log(s): {', '.join(r['path'] for r in direct)})")

    if len(runs) >= 2:
        pairs = [(runs[0], other) for other in runs[1:]]
        for first, second in pairs:
            a, b = quiet_tail(first), quiet_tail(second)
            dtheta = first["theta_eq"] - second["theta_eq"]
            if abs(dtheta) < 1.0e-3:
                continue
            ratio = (a["x_error"] - b["x_error"]) / dtheta
            p0 = first["theta_eq"] - a["x_error"] / ratio
            print(f"two-run estimate   k_theta/k_x = {ratio:6.2f} (deployed row predicts ~34), "
                  f"p0 = {p0:.4f} rad   [{first['path']} vs {second['path']}]")
            print(f"  -> recommended mpc.theta_eq = {p0:.4f}")
    print("\nApply with: ros2 launch ... controller_type:=mpc mpc_theta_eq_source:=override "
          "mpc_theta_eq:=<value>")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(1)
    sys.exit(main(sys.argv[1:]))
