#!/usr/bin/env python3
"""第一阶段定高 Linear MPC 控制器的验收试验。

五种工况,各自在全新的无头 Gazebo 会话中运行,模型高度、出生姿态和初始俯仰参考完全一致:

  static      在 x_ref = 0 处原地平衡
  position    位置阶跃(step_position:<d>)后保持
  speed       通过 /cmd_vel 指令前向速度,然后停止
  push        对 base_link 施加 20 N / 0.2 s 力脉冲,仅在满足平衡门限后才触发
              (与 4.2 节实验相同的门限)
  saturation  同样的推动,但轮子力矩上限降到每轮 2 Nm,此范围内回退链路和限幅
              逻辑必须保持良好定义

所有指标窗口均取自控制器自身的 CSV 日志(x_ref、v_ref 和求解器各列),因为本机上
`ros2 topic pub` 需要 1-12 s 才出现在话题图上,无法按仿真秒对齐计时。

用法:
  python3 run_mpc_balance_trials.py                      # 全部五项,各一次
  python3 run_mpc_balance_trials.py --trials static,push --repeats 3
  python3 run_mpc_balance_trials.py --out-dir /tmp/mpc_trials

需要已构建的工作空间(colcon build --packages-select bbot_balance_controller bbot_bringup)。
"""

import argparse
import csv
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
from run_height_campaign import ForcePulse, pulse_state_valid  # noqa: E402

ALL_TRIALS = ("static", "position", "speed", "push", "saturation")
STABLE_ANGLE_RAD = 0.0100      # 0.57 度:MPC 稳定在几个 mrad 量级,
                               # 但出生瞬态比 4.2 节门限更慢
STABLE_RATE_RAD_S = 0.02
STABLE_VELOCITY_M_S = 0.01
STABLE_HOLD_S = 2.0
RESPONSE_WINDOW_S = 1.5


def cleanup(ws_root):
    for pattern in ("gz sim", "ign gazebo", "ruby.*gazebo", "ros_gz_bridge", "spawner",
                    "bbot_force_pulse", "linear_mpc_balance_controller",
                    "lqr_gain_scheduled_controller", "bbot_velocity_jump_controller"):
        subprocess.run(["pkill", "-9", "-f", pattern], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def launch_command(args, csv_path, extra):
    # 默认值与 adaptive_lqr_balance_controller.cpp 保持一致:theta_eq 来自节点
    # 自带的表(-atan2(y_com, z_com),按指令高度跟踪),机器人在有反馈之前按
    # 节点指令的姿态出生,启动保持结束后腿部从 height.startup_hip_axle 伸展到目标高度。
    command = (
        f"source {args.ws_root}/install/setup.bash && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        "headless:=true gui:=false gazebo_start_paused:=true auto_unpause:=true "
        "controller_type:=mpc "
        "torque_pid_initial_roll:=0.0 "
        f"mpc_target_height:={args.height:.3f} "
        f"mpc_theta_eq_source:={args.theta_eq_source} "
        f"mpc_r:={args.r:g} "
        f"mpc_horizon:={args.horizon} "
        f"mpc_theta_error_limit:={args.theta_limit:g} "
        f"mpc_log_path:={csv_path} "
    )
    if args.theta_eq is not None:
        command += f"mpc_theta_eq:={args.theta_eq:.4f} "
    # 在有任何反馈之前,按节点将要指令的姿态出生。腿部位置接口是开环的,节点从
    # 第一个控制周期就发布启动姿态;若用仓库统一的 0.403 m(关节零位着地高度,
    # 髋轴 0.2577 m),身体会低于该姿态 0.10 m,开局就出现无对抗的腿部伸展。
    spawn_z = args.spawn_z
    if spawn_z is None:
        spawn_z = (args.startup_hip_axle if args.startup_hip_axle is not None
                   else 0.36) + 0.14
    command += f"mpc_spawn_z:={spawn_z:.3f} "
    if args.startup_hip_axle is not None:
        command += f"mpc_startup_height:={args.startup_hip_axle:.3f} "
    if args.startup_hold_time is not None:
        command += f"mpc_startup_hold_time:={args.startup_hold_time:g} "
    return command + f"{extra}"


def read_rows(path):
    if not Path(path).exists():
        return []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        # 节点先写文件头,再在仿真运行时追加行,因此必须丢弃最后一行可能写了一半的内容。
        complete = [row for row in reader if row.get("stage") not in (None, "")
                    and all(row.get(key) not in (None, "") for key in columns)]
        return complete


def column(rows, key):
    if not rows:
        return np.array([])
    return np.array([float(row[key]) for row in rows if row.get(key) not in (None, "")])


def publish(env, topic, msg_type, payload, times=1):
    result = subprocess.run(
        ["ros2", "topic", "pub", f"--times={times}", "--qos-durability", "volatile",
         topic, msg_type, payload],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=60, text=True)
    if result.returncode != 0:
        # 发布若静默失败,位置/速度/推动试验就退化成普通平衡试验,而 CSV 看起来仍然完好。
        raise RuntimeError(f"publish to {topic} failed (rc={result.returncode}): "
                           f"{result.stdout.strip()[-300:]}")


def engaged_window(rows):
    """控制器实际在计算的行(除等待传感器阶段外的所有阶段)。"""
    return [row for row in rows if row["stage"] != "disabled"]


def stability_gate(rows, hold_s):
    """平衡门限首次持续满足 hold_s 的仿真时刻。"""
    stable_since = None
    for row in rows:
        angle = abs(float(row["theta_error"]))
        calm = (angle <= STABLE_ANGLE_RAD and abs(float(row["pitch_rate"])) <= STABLE_RATE_RAD_S
                and abs(float(row["x_dot"])) <= STABLE_VELOCITY_M_S)
        stamp = float(row["time"])
        stable_since = stamp if calm and stable_since is None else stable_since
        if not calm:
            stable_since = None
        elif stamp - stable_since >= hold_s:
            return stamp
    return None


def find_reference_step(rows):
    """x_ref 发生跳变的仿真时刻(位置阶跃指令生效处)。"""
    reference = None
    for row in rows:
        value = float(row["x_ref"])
        if reference is None:
            reference = value
        elif abs(value - reference) > 1.0e-3:
            return float(row["time"]), value - reference
    return None, 0.0


def find_speed_window(rows, commanded):
    """|v_ref| 保持在指令值 10% 以内的 (start, end) 仿真时间段,
    即 1 s 斜坡之后、停止指令之前的巡航平台段。"""
    v_ref = column(rows, "v_ref")
    times = column(rows, "time")
    active = np.abs(v_ref) >= 0.9 * abs(commanded)
    if not active.any():
        return None
    return float(times[active.argmax()]), float(times[np.flatnonzero(active)[-1]])


def summarise(name, rows, notes):
    solved = engaged_window(rows)
    if not solved:
        return {"trial": name, "fail": "controller never engaged", "notes": notes}
    times = column(solved, "time")
    u = column(solved, "u_mpc")
    theta_error = column(solved, "theta_error")
    x_error = column(solved, "x_error")
    sat = column(solved, "total_torque_saturated")
    stage = {row["stage"] for row in rows}
    tail = times > times[-1] - 5.0
    return {
        "trial": name,
        "solved_s": round(float(times[-1] - times[0]), 2),
        "fell": "safety_disabled" in stage,
        "stages": sorted(stage),
        "theta_error_rms_deg": round(float(np.degrees(np.sqrt(np.mean(theta_error ** 2)))), 3),
        "theta_error_max_deg": round(float(np.degrees(np.max(np.abs(theta_error)))), 3),
        "x_error_absmax_m": round(float(np.max(np.abs(x_error))), 4),
        "x_error_rms_m": round(float(np.sqrt(np.mean(x_error ** 2))), 4),
        "x_error_tail_mean_m": round(float(np.mean(np.abs(x_error[tail]))) if tail.any() else float("nan"), 4),
        "x_error_final_mean_m": round(float(np.mean(x_error[tail])) if tail.any() else float("nan"), 4),
        "velocity_error_rms_mps": round(float(np.sqrt(np.mean(
            (column(solved, "x_dot") - column(solved, "v_ref")) ** 2))), 4),
        "u_rms_Nm": round(float(np.sqrt(np.mean(u ** 2))), 2),
        "u_absmax_Nm": round(float(np.max(np.abs(u))), 2),
        "saturated_fraction": round(float(np.mean(sat)), 3),
        "pinned_at_bound_fraction": round(float(np.mean(np.abs(u) >=
                                          np.max(np.abs(u)) - 1.0e-6)), 3),
        "solver_mean_us": round(float(np.mean(column(solved, "solver_time_us"))), 1),
        "solver_max_us": round(float(np.max(column(solved, "solver_time_us"))), 1),
        "solver_iterations_max": int(np.max(column(solved, "solver_iterations"))),
        "fallback_events": int(sum(1 for row in solved if row["stage"] != "solved")),
        "wheel_saturated_fraction": round(float(np.mean(column(solved, "wheel_torque_saturated"))), 3),
        "band_binding_fraction": round(float(np.mean(column(solved, "theta_band_binding"))), 3),
        "nonfinite_rows": int(sum(0 if all(np.isfinite(float(row[key])) for key in
                                           ("u_mpc", "tau_each", "x_error", "theta_error")) else 1
                                  for row in solved)),
        "notes": notes,
    }


def pulse_response_metrics(rows, pulse_start):
    """[pulse_start, pulse_start + RESPONSE_WINDOW_S] 区间内的指标。

    空窗口是合理结果,不是数据错误:较慢达到平衡门限的控制器触发脉冲太晚,
    窗口内几乎没有日志样本。返回"不可用"既避免对空数组做 np.max() 导致崩溃,
    也不会让 0 混入结果而被误读为完全无扰动。
    """
    response = [row for row in rows
                if pulse_start <= float(row["time"]) <= pulse_start + RESPONSE_WINDOW_S]
    if len(response) < 2:
        return {"available": False, "samples": len(response),
                "reason": f"empty response window: pulse_start={pulse_start:.3f} s, "
                          f"{len(response)} logged row(s) inside "
                          f"[{pulse_start:.3f}, {pulse_start + RESPONSE_WINDOW_S:.3f}]",
                "pitch_peak_deg": None, "x_error_peak_m": None, "torque_peak_Nm": None,
                "saturated_fraction": None}
    return {
        "available": True, "samples": len(response), "reason": "",
        "pitch_peak_deg": round(float(np.degrees(np.max(np.abs(
            column(response, "theta_error"))))), 3),
        "x_error_peak_m": round(float(np.max(np.abs(column(response, "x_error")))), 4),
        "torque_peak_Nm": round(float(np.max(np.abs(column(response, "u_mpc")))), 2),
        "saturated_fraction": round(float(np.mean(
            column(response, "total_torque_saturated"))), 3),
    }


def run_trial(name, args, env, out_dir, repetition):
    csv_path = out_dir / f"{name}_rep{repetition}.csv"
    if csv_path.exists():
        csv_path.unlink()
    extra = ""
    notes = {}
    if name == "saturation":
        # 只降低单轮上限;总上限保持 RobotParams 推导的 20 Nm,因此起作用的约束
        # 是 |tau| <= 2 Nm(即 |u| <= 4 Nm),轮子饱和标志可与总饱和标志区分开。
        extra = f"mpc_wheel_torque_max:={args.sat_wheel_torque_nm:g}"
        notes["wheel_torque_max_nm"] = args.sat_wheel_torque_nm

    cleanup(args.ws_root)
    log_file = open(out_dir / f"{name}_launch.log", "w")
    proc = subprocess.Popen(launch_command(args, csv_path, extra), shell=True,
                            executable="/bin/bash", stdout=log_file, stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid, env=env)
    pulse = None
    rows = []
    try:
        # launch 自带的就绪门控(gs_lqr_ready_unpause.py,已适配
        # controller_type:=mpc)会单步推进暂停的世界,直到
        # joint_state_broadcaster / leg_position_controller / wheel_effort_controller
        # 激活且 /imu、/joint_states 有数据流,然后才解除暂停。没有它时,9 次运行中
        # 有 4 次在第一个控制周期之前摆就已倒地。
        deadline = time.time() + args.startup_timeout
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            rows = read_rows(csv_path)
            if rows and any(row["stage"] != "disabled" for row in rows):
                break
            time.sleep(0.5)
        if not rows or not any(row["stage"] != "disabled" for row in rows):
            return {"trial": name, "fail": "no MPC rows after unpause", "notes": notes}
        # 接管延迟:协议有效的运行应在时钟推进后的一个控制周期内开始计算。
        # 更长的延迟意味着反馈桥接滞后,此时记录到的摔倒不能归咎于控制器。
        notes["first_controlled_time_s"] = float(
            next(row for row in rows if row["stage"] != "disabled")["time"])

        # 力桥接和脉冲节点只在满足平衡门限后才启动:开局几秒内启动它们会抢占
        # 200 Hz 墙钟定时器的 CPU,而此时摆仍在发散,实测会把一次站立运行变成摔倒。
        gate_time = None
        gate_deadline = time.time() + args.startup_timeout
        while gate_time is None and time.time() < gate_deadline and proc.poll() is None:
            gate_time = stability_gate(engaged_window(read_rows(csv_path)), STABLE_HOLD_S)
            if gate_time is None:
                time.sleep(1.0)
        if name == "position":
            if gate_time is None:
                return {"trial": name, "fail": "balance gate never satisfied",
                        "notes": notes}
            time.sleep(1.0)
            publish(env, "/linear_mpc/command", "std_msgs/msg/String",
                    f'data: "step_position:{args.position_step_m:g}"')
        elif name == "speed":
            if gate_time is None:
                return {"trial": name, "fail": "balance gate never satisfied",
                        "notes": notes}
            publish(env, "/cmd_vel", "geometry_msgs/msg/Twist",
                    f"{{linear: {{x: {args.commanded_speed_mps}}}}}", times=1)
            time.sleep(args.speed_hold_s)
            publish(env, "/cmd_vel", "geometry_msgs/msg/Twist", "{linear: {x: 0.0}}", times=1)
        elif name in ("push", "saturation"):
            if gate_time is None:
                rows = engaged_window(read_rows(csv_path))
                return summarise(name, rows, {**notes, "fail": "balance gate not reached"})
            pulse = ForcePulse(env, args.world_name, "base_link", "y", args.force_newtons,
                               args.force_duration)
            pulse.start_bridge(str(out_dir))
            pulse.arm(str(out_dir))
            time.sleep(2.0)
            rows = engaged_window(read_rows(csv_path))
            latest = float(rows[-1]["time"])
            pulse.fire(latest + 1.0)
            time.sleep(args.force_duration + 4.0)

        time.sleep(max(0.0, args.duration - 6.0))
        rows = read_rows(csv_path)
        solved = engaged_window(rows)
        summary = summarise(name, solved, notes)

        if name == "position":
            step_time, step_size = find_reference_step(solved)
            summary["notes"]["step_time_s"] = step_time
            summary["notes"]["step_size_m"] = round(step_size, 4)
            if step_time is not None:
                after = [row for row in solved if float(row["time"]) > step_time]
                settle = [row for row in after if float(row["time"]) > step_time + 3.0]
                summary["notes"]["error_after_step_max_m"] = round(
                    float(np.max(np.abs(column(after, "x_error")))), 4) if after else None
                summary["notes"]["error_settled_mean_m"] = round(
                    float(np.mean(np.abs(column(settle, "x_error")))), 4) if settle else None
        if name == "speed":
            window = find_speed_window(solved, args.commanded_speed_mps)
            summary["notes"]["cruise_window_s"] = [round(value, 2) for value in window] if window else None
            if window:
                inside = [row for row in solved
                          if window[0] <= float(row["time"]) <= window[1]]
                if inside:
                    summary["notes"]["cruise_seconds"] = round(window[1] - window[0], 2)
                    summary["notes"]["cruise_v_ref_mps"] = round(
                        float(np.mean(column(inside, "v_ref"))), 4)
                    summary["notes"]["cruise_measured_speed_mps"] = round(
                        float(np.mean(column(inside, "x_dot"))), 4)
                    summary["notes"]["cruise_speed_error_mps"] = round(
                        float(np.mean(np.abs(column(inside, "v_error")))), 4)
                    summary["notes"]["cruise_travel_m"] = round(
                        float(column(inside, "x")[-1] - column(inside, "x")[0]), 3)
                    summary["notes"]["cruise_pitch_mean_rad"] = round(
                        float(np.mean(column(inside, "theta_error"))), 4)
                    summary["notes"]["cruise_band_binding_fraction"] = round(
                        float(np.mean(column(inside, "theta_band_binding"))), 3)
                    summary["notes"]["cruise_saturated_fraction"] = round(
                        float(np.mean(column(inside, "total_torque_saturated"))), 3)
        if name in ("push", "saturation"):
            state = pulse.read_state() if pulse else None
            summary["notes"]["pulse_valid"] = bool(pulse_state_valid(state, args.force_duration))
            if state and state.get("pulse_start_sim_time"):
                metrics = pulse_response_metrics(solved, float(state["pulse_start_sim_time"]))
            else:
                metrics = {"available": False, "samples": 0,
                           "reason": "pulse reported no start sim time",
                           "pitch_peak_deg": None, "x_error_peak_m": None,
                           "torque_peak_Nm": None, "saturated_fraction": None}
            summary["notes"]["push_pitch_peak_deg"] = metrics["pitch_peak_deg"]
            summary["notes"]["push_x_error_peak_m"] = metrics["x_error_peak_m"]
            summary["notes"]["push_torque_peak_Nm"] = metrics["torque_peak_Nm"]
            summary["notes"]["push_saturated_fraction"] = metrics["saturated_fraction"]
            summary["notes"]["push_response_window"] = {
                key: metrics[key] for key in ("available", "samples", "reason")}
        return summary
    finally:
        if pulse is not None:
            pulse.shutdown()
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_file.close()
        time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--trials", default=",".join(ALL_TRIALS))
    parser.add_argument("--height", type=float, default=0.40)
    parser.add_argument("--theta-eq-source", default="table",
                        help="table (adaptive_lqr rule, default) | geometric | override")
    parser.add_argument("--theta-eq", type=float, default=None,
                        help="nominal balance pitch [rad]; only used with --theta-eq-source override")
    parser.add_argument("--spawn-z", type=float, default=None,
                        help="base_link spawn height [m]; default is the launch value (0.403)")
    parser.add_argument("--startup-hip-axle", type=float, default=None,
                        help="height.startup_hip_axle [m]; default is the launch value (0.36)")
    parser.add_argument("--startup-hold-time", type=float, default=None,
                        help="height.startup_hold_time [s]; default is the launch value (2.0)")
    parser.add_argument("--r", type=float, default=8.0, help="control weight R of the MPC cost")
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--theta-limit", type=float, default=0.20,
                        help="pitch-error band enforced over the horizon [rad]")
    parser.add_argument("--force-newtons", type=float, default=20.0)
    parser.add_argument("--force-duration", type=float, default=0.20)
    parser.add_argument("--position-step-m", type=float, default=0.30)
    parser.add_argument("--commanded-speed-mps", type=float, default=0.15)
    parser.add_argument("--speed-hold-s", type=float, default=6.0)
    parser.add_argument("--sat-wheel-torque-nm", type=float, default=0.5,
                        help="per-wheel authority for the saturation trial (|u| <= 2x this)")
    parser.add_argument("--duration", type=float, default=22.0, help="sim seconds per trial")
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--world-name", default="balance_test_world")
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()

    selected = [item for item in args.trials.split(",") if item in ALL_TRIALS]
    unknown = [item for item in args.trials.split(",") if item not in ALL_TRIALS]
    if unknown:
        parser.error(f"unknown trials: {unknown}; choose from {list(ALL_TRIALS)}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir or (Path(args.ws_root) / "src" / "bbot_balance_controller" /
                                    "src" / "data_logs" / "mpc_phase1" / stamp))
    out_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs(env["ROS_HOME"], exist_ok=True)
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    # 在第一个试验之前就失败,而不是之后:ros2 CLI 依赖 source 过的工作空间
    # (PYTHONPATH/AMENT_PREFIX_PATH),从未 source 的 shell 启动的批次不会发布
    # 任何内容,而每个 CSV 看起来仍然完好。
    check = subprocess.run(["ros2", "node", "list"], env=env, text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    if check.returncode != 0:
        sys.exit("ros2 CLI is not usable in this environment. Source "
                 f"{args.ws_root}/install/setup.bash before running. "
                 f"Output: {check.stdout.strip()[-200:]}")

    results = []
    for repetition in range(1, args.repeats + 1):
        for name in selected:
            label = name if args.repeats == 1 else f"{name}_rep{repetition}"
            print(f"\n### {label} (rep {repetition}/{args.repeats}) -> "
                  f"{out_dir / (name + '_rep' + str(repetition) + '.csv')}", flush=True)
            try:
                summary = run_trial(name, args, env, out_dir, repetition)
            except Exception as error:
                # 测试框架自身的失败不能毁掉整个批次(曾有一个指标 bug 以 rc=1
                # 杀死整个单元)。这里记录为显式异常,让 sweep 驱动将其视为
                # protocol_invalid 而非控制器结果。
                summary = {"trial": label, "repetition": repetition,
                           "fail": f"harness exception: {type(error).__name__}: "
                                   f"{str(error)[:200]}",
                           "harness_exception": True}
            # 权威的基础试验名。驱动按此字段统计名额;"trial" 只是显示标签
            # ("<name>_rep<k>"),不要解析它。
            summary["trial_name"] = name
            summary["trial"] = label
            summary["repetition"] = repetition
            print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
            results.append(summary)
        cleanup(args.ws_root)

    report = {
        "config": {
            "height_m": args.height,
            "spawn_base_link_m": (args.spawn_z if args.spawn_z is not None else
                                  (args.startup_hip_axle
                                   if args.startup_hip_axle is not None else 0.36) + 0.14),
            "startup_hip_axle_m": (args.startup_hip_axle if args.startup_hip_axle is not None
                                   else 0.36),
            "startup_hold_time_s": (args.startup_hold_time if args.startup_hold_time is not None
                                    else 2.0),
            "initial_roll_rad": 0.0,
            "theta_eq_source": args.theta_eq_source,
            "theta_eq_rad": args.theta_eq, "r": args.r, "horizon": args.horizon,
            "theta_band_rad": args.theta_limit, "control_period_s": 0.005,
            "total_torque_max_Nm": 20.0, "wheel_torque_max_Nm": 10.0,
            "q": [100.0, 5000.0, 3000.0, 1200.0],
            "force_N": args.force_newtons, "force_duration_s": args.force_duration,
            "position_step_m": args.position_step_m,
            "commanded_speed_mps": args.commanded_speed_mps,
            "saturation_wheel_torque_Nm": args.sat_wheel_torque_nm,
            "saturation_total_torque_Nm": 20.0,
            "trials": selected, "repeats": args.repeats,
        },
        "results": results,
    }
    (out_dir / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nsummary written to {out_dir / 'summary.json'}")
    failures = [item for item in results if item.get("fell") or item.get("fail")
                or item.get("nonfinite_rows")]
    print(f"trials with a fall / failure / non-finite row: {len(failures)} / {len(results)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
