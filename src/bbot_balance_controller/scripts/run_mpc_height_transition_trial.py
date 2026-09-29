#!/usr/bin/env python3
"""第二阶段高度调度 Linear MPC 的动态升降试验。

用 /target_height（std_msgs/msg/Float64，与 adaptive_lqr_balance_controller 同一个
接口）依次下达 0.40 -> 0.30 -> 0.50 -> 0.40，腿部按 leg_transition_speed（默认
0.05 m/s）平滑逼近，平衡轮转矩全程不中断。

每一步的判据都来自控制器自己的 CSV：
  * target_height 列变成新目标 —— 证明 ros2 topic pub 真的到达了节点（从未 source
    的 shell 里发布是静默失败的，而日志其余部分看起来完好）。
  * height 列（current_height）到达新目标，误差为 0（限速逼近是精确落位）。
  * model_height 与 height 逐拍相等 —— 证明模型用的是当前高度而不是目标高度。
  * gt_pose_z - 0.14 跟着 height 走 —— 腿部在仿真里真的走到了该高度。

用法（在已 source 工作空间的 shell 中运行）:
    python3 run_mpc_height_transition_trial.py --out-dir /tmp/mpc_height_dynamic
    python3 run_mpc_height_transition_trial.py --sequence 0.40,0.30,0.50,0.40
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from run_mpc_balance_trials import (  # noqa: E402
    STABLE_HOLD_S, cleanup, engaged_window, launch_command, read_rows, stability_gate)
from analyze_mpc_height import analyse  # noqa: E402

TARGET_TOPIC = "/target_height"
TARGET_TYPE = "std_msgs/msg/Float64"
PUBLISH_RETRIES = 3


class LaunchArgs:
    """launch_command() 需要的字段，与 run_mpc_balance_trials.py 的默认值一致。"""

    def __init__(self, ws_root, height, theta_eq_source, r, horizon, theta_limit,
                 startup_hip_axle, startup_hold_time, spawn_z):
        self.ws_root = ws_root
        self.height = height
        self.theta_eq_source = theta_eq_source
        self.r = r
        self.horizon = horizon
        self.theta_limit = theta_limit
        self.theta_eq = None
        self.spawn_z = spawn_z
        self.startup_hip_axle = startup_hip_axle
        self.startup_hold_time = startup_hold_time


def latest(rows):
    return rows[-1] if rows else None


def wait_for_controlled(out_dir, csv_path, startup_timeout):
    """等到节点真正开始计算（而不是只看进程活着）。"""
    deadline = time.time() + startup_timeout
    while time.time() < deadline:
        rows = engaged_window(read_rows(csv_path))
        if rows:
            return rows
        time.sleep(0.5)
    return []


def wait_for_gate(csv_path, startup_timeout):
    deadline = time.time() + startup_timeout
    while time.time() < deadline:
        gate = stability_gate(engaged_window(read_rows(csv_path)), STABLE_HOLD_S)
        if gate is not None:
            return gate
        time.sleep(1.0)
    return None


def publish_target(env, value):
    payload = f"{{data: {value:.3f}}}"
    last_output = ""
    for attempt in range(PUBLISH_RETRIES):
        result = subprocess.run(
            ["ros2", "topic", "pub", "--times=1", "--qos-durability", "volatile",
             TARGET_TOPIC, TARGET_TYPE, payload],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=60, text=True)
        last_output = result.stdout.strip()
        if result.returncode == 0:
            return
        time.sleep(1.0)
    raise RuntimeError(f"publish {TARGET_TOPIC} {payload} failed "
                       f"(rc={result.returncode}): {last_output[-300:]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--sequence", default="0.40,0.30,0.50,0.40",
                        help="逗号分隔的髋-轴高度序列；第一个是出生目标，其余用 /target_height 下达")
    parser.add_argument("--leg-transition-speed", type=float, default=0.05)
    parser.add_argument("--startup-hip-axle", type=float, default=0.36)
    parser.add_argument("--startup-hold-time", type=float, default=2.0)
    parser.add_argument("--settle-s", type=float, default=4.0,
                        help="每次到达新高度后的额外观察时间（墙钟秒）")
    parser.add_argument("--r", type=float, default=8.0)
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--theta-limit", type=float, default=0.20)
    parser.add_argument("--theta-eq-source", default="table")
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--world-name", default="balance_test_world")
    parser.add_argument("--out-dir", default="/tmp/mpc_height_dynamic")
    args = parser.parse_args()

    sequence = [float(value) for value in args.sequence.split(",")]
    if not sequence:
        sys.exit("empty height sequence")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "height_transition.csv"
    if csv_path.exists():
        csv_path.unlink()

    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs(env["ROS_HOME"], exist_ok=True)
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    check = subprocess.run(["ros2", "node", "list"], env=env, text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    if check.returncode != 0:
        sys.exit("ros2 CLI is not usable in this environment. Source "
                 f"{args.ws_root}/install/setup.bash before running. "
                 f"Output: {check.stdout.strip()[-200:]}")

    launch_args = LaunchArgs(args.ws_root, sequence[0], args.theta_eq_source, args.r,
                             args.horizon, args.theta_limit, args.startup_hip_axle,
                             args.startup_hold_time, None)
    extra = f"mpc_leg_transition_speed:={args.leg_transition_speed:g} "

    cleanup(args.ws_root)
    log_file = open(out_dir / "launch.log", "w")
    proc = subprocess.Popen(launch_command(launch_args, csv_path, extra), shell=True,
                            executable="/bin/bash", stdout=log_file,
                            stderr=subprocess.STDOUT, preexec_fn=os.setsid, env=env)
    phases = []
    outcome = {}
    gate = None
    try:
        rows = wait_for_controlled(out_dir, csv_path, args.startup_timeout)
        if not rows:
            outcome = {"fail": "no MPC rows after unpause"}
        else:
            outcome["first_controlled_time_s"] = float(rows[0]["time"])
            gate = wait_for_gate(csv_path, args.startup_timeout)
            if gate is None:
                outcome["fail"] = "balance gate never satisfied before the first height command"
            outcome["stability_gate_sim_s"] = gate

        if gate is not None:
            for goal in sequence[1:]:
                before = latest(engaged_window(read_rows(csv_path)))
                command_sim = float(before["time"])
                command_height = float(before["height"])
                publish_target(env, goal)

                # 先确认节点收到了新目标，再等腿部到位。
                confirmed = None
                arrival = None
                deadline = time.time() + 30.0
                while time.time() < deadline and arrival is None:
                    live = engaged_window(read_rows(csv_path))
                    row = latest(live)
                    if row is not None:
                        if confirmed is None and abs(float(row["target_height"]) - goal) < 1.0e-6:
                            confirmed = float(row["time"])
                        if confirmed is not None and abs(float(row["height"]) - goal) < 1.0e-6:
                            arrival = float(row["time"])
                    time.sleep(0.5)
                if confirmed is None:
                    raise RuntimeError(f"/target_height {goal:.3f} 从未出现在日志的 "
                                       "target_height 列：发布链路失效，本条作废")
                if arrival is None:
                    raise RuntimeError(f"current_height 未在 30 s 内到达 {goal:.3f} m")

                time.sleep(max(0.0, args.settle_s))
                phases.append({
                    "from_m": round(command_height, 4),
                    "to_m": goal,
                    "command_sim_s": round(command_sim, 3),
                    "confirmed_sim_s": round(confirmed, 3),
                    "arrival_sim_s": round(arrival, 3),
                    "transition_s": round(arrival - command_sim, 3),
                    "expected_transition_s": round(abs(goal - command_height)
                                                   / args.leg_transition_speed, 3),
                })
                print(json.dumps(phases[-1], ensure_ascii=False), flush=True)

        time.sleep(2.0)
    finally:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_file.close()
        time.sleep(2.0)
        cleanup(args.ws_root)

    if "fail" in outcome and not phases:
        outcome["csv"] = str(csv_path)
        print(json.dumps(outcome, ensure_ascii=False, indent=2))
        return 1

    rows = engaged_window(read_rows(csv_path))
    sim_time = np.array([float(row["time"]) for row in rows])
    current = np.array([float(row["height"]) for row in rows])
    model = np.array([float(row["model_height"]) for row in rows])
    theta_error = np.array([float(row["theta_error"]) for row in rows])
    x_error = np.array([float(row["x_error"]) for row in rows])
    u_mpc = np.array([float(row["u_mpc"]) for row in rows])
    saturated = np.array([float(row["total_torque_saturated"]) for row in rows])
    rebuild_us = np.array([float(row["model_rebuild_us"]) for row in rows])
    rebuild_failed = np.array([float(row["model_rebuild_failed"]) for row in rows])
    stage = np.array([row["stage"] for row in rows])
    truth_z = np.array([float(row["gt_pose_z"]) for row in rows])

    for phase in phases:
        window = (sim_time >= phase["command_sim_s"]) & (sim_time <= phase["arrival_sim_s"] + 2.0)
        if not window.any():
            phase["error"] = "no rows in the transition window"
            continue
        inside = np.nonzero(window)[0]
        first, last = inside[0], inside[-1]
        moving = window & (rebuild_us > 0.0)
        phase.update({
            "height_range_m": [round(float(current[first:last + 1].min()), 4),
                               round(float(current[first:last + 1].max()), 4)],
            "model_minus_current_absmax_m": round(
                float(np.abs(model[first:last + 1] - current[first:last + 1]).max()), 7),
            "truth_leg_minus_commanded_absmax_m": round(
                float(np.abs(truth_z[first:last + 1] - 0.14 - current[first:last + 1]).max()), 5),
            "theta_error_absmax_deg": round(
                float(np.abs(theta_error[first:last + 1]).max() * 180.0 / np.pi), 4),
            "x_error_absmax_m": round(float(np.abs(x_error[first:last + 1]).max()), 5),
            "u_absmax_Nm": round(float(np.abs(u_mpc[first:last + 1]).max()), 4),
            "saturated_fraction": round(float(saturated[first:last + 1].mean()), 5),
            "non_solved_rows": int(np.count_nonzero(stage[first:last + 1] != "solved")),
            "rebuild_failed_rows": int(rebuild_failed[first:last + 1].sum()),
            "rebuild_us_mean": round(float(rebuild_us[moving].mean()), 1) if moving.any() else None,
            "rebuild_us_max": round(float(rebuild_us[moving].max()), 1) if moving.any() else None,
        })

    print("\n=== 分段结果 ===")
    print(json.dumps(phases, ensure_ascii=False, indent=2))
    print("\n=== 整条日志汇总 ===")
    summary = analyse(csv_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    report = {"config": {
        "sequence": sequence,
        "leg_transition_speed_mps": args.leg_transition_speed,
        "startup_hip_axle_m": args.startup_hip_axle,
        "startup_hold_time_s": args.startup_hold_time,
        "spawn_base_link_m": args.startup_hip_axle + 0.14,
        "r": args.r, "horizon": args.horizon, "theta_band_rad": args.theta_limit,
        "theta_eq_source": args.theta_eq_source,
        "control_period_s": 0.005, "q": [100.0, 5000.0, 3000.0, 1200.0],
        "world_name": args.world_name,
    }, "outcome": outcome, "phases": phases, "whole_run": summary}
    (out_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nreport -> {out_dir / 'report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
