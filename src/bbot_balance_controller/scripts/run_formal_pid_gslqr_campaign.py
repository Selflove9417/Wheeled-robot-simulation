#!/usr/bin/env python3
"""Formal Sec. 4.2 comparison: frozen four-loop PID vs historical GS-LQR.

The two controllers run in separate fresh directories.  Each of the ten
conditions is intended to have three protocol-valid repetitions.  Only a
startup/bridge/pulse protocol error is retryable; a controller fall, early
disable, missing recovery, or wrong physical response is retained as that
repetition's result and is never hidden by a replacement run.
"""

import argparse
import csv
import hashlib
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
from run_height_campaign import (  # noqa: E402
    ForcePulse,
    TrialHelper,
    PULSE_DURATION_TOLERANCE,
    pulse_state_valid,
)

HEIGHTS = (0.30, 0.40, 0.50)
RESET_SIM_TIME = 6.74
CONSTANT_END = 35.0
LIFT_END = 38.0
PUSH_END = 40.0
LIFT_LOW_HEIGHT_M = 0.30
LIFT_HIGH_HEIGHT_M = 0.50
LIFT_HEIGHT_TOLERANCE_M = 0.005
LIFT_POSITION_SPAN_TOLERANCE_M = 0.005
LIFT_STABLE_HOLD_S = 5.0
LIFT_HIGH_HOLD_S = 10.0
LIFT_LOW_HOLD_S = 10.0
LIFT_GATE_TIMEOUT_AFTER_RESET_S = 40.0
STABLE_ANGLE_DEG = 0.20
STABLE_RATE_RAD_S = 0.02
STABLE_VELOCITY_M_S = 0.01
STABLE_HOLD_S = 5.0
RESPONSE_WINDOW_S = 1.5
RECOVERY_HOLD_S = 2.0


def build_trials(repeats):
    trials = []
    for height in HEIGHTS:
        for rep in range(1, repeats + 1):
            trials.append({"job": "constant", "height": height,
                           "force": 0.0, "rep": rep})
    for rep in range(1, repeats + 1):
        trials.append({"job": "lift", "height": 0.30,
                       "force": 0.0, "rep": rep})
    for height in HEIGHTS:
        for force in (20.0, -20.0):
            for rep in range(1, repeats + 1):
                trials.append({"job": "push", "height": height,
                               "force": force, "rep": rep})
    return trials


def trial_tag(controller, trial):
    prefix = "pid" if controller == "position_pid" else "gs_lqr"
    if trial["job"] == "push":
        return f"{prefix}_push_h{trial['height']:.2f}_f{trial['force']:+.0f}_rep{trial['rep']}"
    if trial["job"] == "lift":
        return f"{prefix}_lift_rep{trial['rep']}"
    return f"{prefix}_constant_h{trial['height']:.2f}_rep{trial['rep']}"


def ros_cli(env, args, timeout=3.0):
    try:
        return subprocess.run(["ros2", *args], env=env, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def service_call(env, world_name, request):
    try:
        return subprocess.run(
            ["ros2", "service", "call", f"/world/{world_name}/control",
             "ros_gz_interfaces/srv/ControlWorld", request], env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5.0).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def service_ok(output):
    compact = output.lower().replace(" ", "")
    return "success:true" in compact or "success=true" in compact


def cleanup():
    patterns = (
        "gz sim", "ign gazebo", "ruby.*gazebo", "ros_gz_bridge", "spawner",
        "bbot_force_pulse", "bbot_trial_helper", "adaptive_lqr_balance_controller",
        "position_torque_cascade_pid_controller",
    )
    for pattern in patterns:
        subprocess.run(["pkill", "-9", "-f", pattern], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def wait_for_startup(env, world_name, controller, timeout):
    node_name = ("/position_torque_cascade_pid_controller"
                 if controller == "position_pid"
                 else "/adaptive_lqr_balance_controller")
    command_topic = ("/position_torque_cascade_pid/command"
                     if controller == "position_pid" else "/adaptive_lqr/command")
    service = f"/world/{world_name}/control"
    deadline = time.monotonic() + timeout
    paused = False
    detail = ""
    while time.monotonic() < deadline:
        services = ros_cli(env, ["service", "list"])
        if service in services and not paused:
            paused = service_ok(service_call(env, world_name, "{world_control: {pause: true}}"))
        if paused:
            service_call(env, world_name, "{world_control: {step: true, multi_step: 10}}")
        nodes = ros_cli(env, ["node", "list"])
        controllers = ros_cli(env, ["control", "list_controllers", "-c", "/controller_manager"])
        info = ros_cli(env, ["node", "info", node_name])
        wheel_active = any(
            "wheel_effort_controller" in line and "active" in line.lower()
            for line in controllers.splitlines())
        subscriptions = all(topic in info for topic in (
            "/imu", "/joint_states", "/target_height", command_topic))
        if paused and service in services and node_name in nodes and wheel_active and subscriptions:
            return True, "ready"
        detail = (f"paused={paused}, node={node_name in nodes}, "
                  f"wheel_active={wheel_active}, subscriptions={subscriptions}")
        time.sleep(0.5)
    return False, f"startup readiness timeout ({detail})"


def unpause(env, world_name):
    output = service_call(env, world_name, "{world_control: {pause: false}}")
    return service_ok(output), output[-240:]


def read_rows(path):
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def last_record(path, controller):
    rows = read_rows(path)
    if not rows:
        return None
    row = rows[-1]
    try:
        if controller == "position_pid":
            return {
                "time": float(row["time"]), "p": float(row["p"]),
                "height": float(row["height"]),
                "p_target": float(row["p_target"]),
                "target_latched": float(row["position_target_latched"]),
                "latch_count": float(row["position_latch_count"]),
                "pitch": float(row["pitch"]), "theta_eq": float(row["theta_eq"]),
                "pitch_rate": float(row["pitch_rate"]), "velocity": float(row["v"]),
            }
        return {
            "time": float(row["time"]), "p": float(row["x"]),
            "height": float(row["hip_axle_height"]),
            "p_target": float(row["x_ref"]), "target_latched": 1.0,
            "latch_count": 1.0, "pitch": float(row["pitch"]),
            "theta_eq": float(row["theta_eq_nominal"]),
            "pitch_rate": float(row["pitch_rate"]), "velocity": float(row["x_dot"]),
        }
    except (KeyError, TypeError, ValueError):
        return None


def arrays(path, controller):
    rows = read_rows(path)
    if not rows:
        raise ValueError("empty CSV")
    required = ("time", "p", "p_target", "pitch", "pitch_rate", "velocity",
                "theta_eq", "torque")
    out = {name: [] for name in required}
    if controller == "position_pid":
        mapping = {"p": "p", "p_target": "p_target", "pitch": "pitch",
                   "pitch_rate": "pitch_rate", "velocity": "v",
                   "theta_eq": "theta_eq", "torque": "u_clamped"}
    else:
        mapping = {"p": "x", "p_target": "x_ref", "pitch": "pitch",
                   "pitch_rate": "pitch_rate", "velocity": "x_dot",
                   "theta_eq": "theta_eq_nominal", "torque": "u_model"}
    for row in rows:
        try:
            out["time"].append(float(row["time"]))
            for key, name in mapping.items():
                out[key].append(float(row[name]))
        except (KeyError, TypeError, ValueError):
            continue
    if len(out["time"]) < 100:
        raise ValueError("CSV too short")
    return {key: np.asarray(value) for key, value in out.items()}


def continuous_time(t, ok, start, hold):
    for i in np.where(t >= start)[0]:
        j = i
        while j < len(t) and ok[j]:
            if t[j] - t[i] >= hold:
                return float(t[i] - start)
            j += 1
    return np.nan


def response_metrics(path, controller, trial, pulse_state):
    result = {"fall_or_early_disable": False, "position_target_constant": False}
    data = arrays(path, controller)
    t = data["time"]
    actual_pitch_deg = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    if controller == "position_pid":
        raw_rows = read_rows(path)
        target_post_mask = np.asarray([
            float(row.get("position_target_latched", 0.0)) > 0.5
            for row in raw_rows
        ], dtype=bool)
        if len(target_post_mask) != len(t):
            target_post_mask = t >= (RESET_SIM_TIME + 3.0)
    else:
        # Historical GS-LQR has no latch flag in its legacy CSV.  Detect the
        # first post-reset x_ref change, then evaluate only the fixed-target
        # segment rather than mixing the startup zero reference into it.
        target_post_mask = t >= (RESET_SIM_TIME + 3.0)
        changes = np.where((t >= RESET_SIM_TIME) &
                           (np.abs(data["p_target"] - data["p_target"][0]) > 1e-4))[0]
        if len(changes):
            target_post_mask = np.arange(len(t)) >= int(changes[0])
    target_values = data["p_target"][target_post_mask] if np.any(target_post_mask) else data["p_target"]
    target_constant = np.ptp(target_values) <= 1e-8
    result["position_target_constant"] = bool(target_constant)
    result["target_initial"] = float(target_values[0])
    result["target_final"] = float(target_values[-1])
    result["position_target_span_mm"] = float(np.ptp(target_values) * 1000.0)
    if controller == "position_pid":
        pid_rows = read_rows(path)
        result["position_latch_count"] = int(max(
            float(row.get("position_latch_count", 0.0)) for row in pid_rows))
    else:
        result["position_latch_count"] = 1 if target_constant else 0
    result["position_target_lock_once_ok"] = bool(
        result["position_latch_count"] == 1 and target_constant)
    target_ref = float(np.median(target_values))

    if trial["job"] == "push":
        if not pulse_state or pulse_state.get("pulse_start_sim_time") is None:
            result["fail_reason"] = "pulse_not_completed"
            return result
        t0 = float(pulse_state["pulse_start_sim_time"])
        t1 = float(pulse_state["pulse_end_sim_time"])
        pre = (t >= t0 - 2.0) & (t < t0)
        response = (t >= t0) & (t <= t0 + RESPONSE_WINDOW_S)
        if not np.any(pre) or not np.any(response):
            result["fail_reason"] = "insufficient_pulse_window"
            return result
        position_pre_mean = float(np.mean(data["p"][pre]))
        position_error_mm = (data["p"] - position_pre_mean) * 1000.0
        result.update({
            "pulse_start_sim_time": t0,
            "pulse_end_sim_time": t1,
            "pulse_duration_s": t1 - t0,
            "pre_pulse_position_mean_m": position_pre_mean,
            "position_peak_relative_mm": float(np.max(np.abs(position_error_mm[response]))),
            "pitch_peak_relative_deg": float(np.max(np.abs(actual_pitch_deg[response]))),
            "velocity_peak_mps": float(np.max(np.abs(data["velocity"][response]))),
            "torque_peak_postpulse_1p5s_nm": float(np.max(np.abs(data["torque"][response]))),
            "torque_rms_postpulse_1p5s_nm": float(np.sqrt(np.mean(data["torque"][response] ** 2))),
            "initial_velocity_change_mps": float(
                np.median(data["velocity"][response & (t >= t0 + 0.10)]) -
                np.median(data["velocity"][pre])) if np.any(response & (t >= t0 + 0.10)) else np.nan,
            "expected_velocity_sign": 1.0 if trial["force"] > 0 else -1.0,
            "force_direction_ok": False,
            "position_saturation_ratio_response": float(np.mean(
                np.abs(data["torque"][response]) >= 19.5)),
            "attitude_velocity_recovery_s": continuous_time(
                t, (np.abs(actual_pitch_deg) <= 0.5) &
                (np.abs(data["velocity"]) <= 0.02), t1, RECOVERY_HOLD_S),
            "position_10mm_recovery_s": continuous_time(
                t, np.abs(position_error_mm) <= 10.0, t1, RECOVERY_HOLD_S),
            "final_position_residual_mm": float(np.mean(position_error_mm[t >= t[-1] - 2.0])),
        })
        result["force_direction_ok"] = bool(
            np.isfinite(result["initial_velocity_change_mps"]) and
            abs(result["initial_velocity_change_mps"]) >= 0.03 and
            result["initial_velocity_change_mps"] * result["expected_velocity_sign"] > 0)
    elif trial["job"] == "constant":
        steady = t >= t[-1] - 10.0
        result.update({
            "steady_pitch_rms_deg": float(np.sqrt(np.mean(actual_pitch_deg[steady] ** 2))),
            "steady_velocity_rms_mps": float(np.sqrt(np.mean(data["velocity"][steady] ** 2))),
            "position_peak_relative_mm": float(np.max(np.abs((data["p"] - target_ref) * 1000.0))),
            "final_position_residual_mm": float((data["p"][-1] - target_ref) * 1000.0),
            "pitch_peak_relative_deg": float(np.max(np.abs(actual_pitch_deg))),
            "torque_peak_postpulse_1p5s_nm": float(np.max(np.abs(data["torque"]))),
        })
    else:
        result.update({
            "height_end_m": float(data["p_target"][-1]) if False else np.nan,
            "position_peak_relative_mm": float(np.max(np.abs((data["p"] - target_ref) * 1000.0))),
            "pitch_peak_relative_deg": float(np.max(np.abs(actual_pitch_deg))),
            "torque_peak_postpulse_1p5s_nm": float(np.max(np.abs(data["torque"]))),
            "final_position_residual_mm": float((data["p"][-1] - target_ref) * 1000.0),
        })
    return result


def capture_manifest(args, root):
    def sha(path):
        digest = hashlib.sha256()
        try:
            with open(path, "rb") as handle:
                for block in iter(lambda: handle.read(1 << 20), b""):
                    digest.update(block)
            return digest.hexdigest()
        except OSError:
            return None
    def command(cmd):
        try:
            return subprocess.run(cmd, cwd=args.ws_root, text=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  timeout=10).stdout.strip()
        except Exception as exc:
            return str(exc)
    files = [
        "src/bbot_balance_controller/src/position_torque_cascade_pid_controller.cpp",
        "src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp",
        "src/bbot_bringup/launch/bbot_gazebo.launch.py",
        "src/bbot_bringup/worlds/balance_test_world.sdf",
        args.pid_gains_file,
        args.gs_config_file,
    ]
    lift_recheck = bool(args.lift_only and args.lift_stable_gated)
    manifest = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "protocol": {
            "conditions": 1 if lift_recheck else 10,
            "repetitions_per_condition": args.repeats,
            "total_controller_runs": (2 if lift_recheck else 20) * args.repeats,
            "pulse_duration_s": args.force_duration,
            "response_window_s": RESPONSE_WINDOW_S,
            "recovery_hold_s": RECOVERY_HOLD_S,
            "stable_gate": {"angle_deg": STABLE_ANGLE_DEG,
                            "rate_rad_s": STABLE_RATE_RAD_S,
                            "velocity_m_s": STABLE_VELOCITY_M_S,
                            "hold_s": STABLE_HOLD_S},
        },
        "lift_recheck": ({
            "enabled": True,
            "low_height_m": LIFT_LOW_HEIGHT_M,
            "high_height_m": LIFT_HIGH_HEIGHT_M,
            "height_tolerance_m": LIFT_HEIGHT_TOLERANCE_M,
            "position_span_tolerance_m": LIFT_POSITION_SPAN_TOLERANCE_M,
            "static_hold_s": LIFT_STABLE_HOLD_S,
            "high_hold_s": LIFT_HIGH_HOLD_S,
            "low_observation_hold_s": LIFT_LOW_HOLD_S,
            "gate_timeout_after_reset_s": LIFT_GATE_TIMEOUT_AFTER_RESET_S,
        } if lift_recheck else {"enabled": False}),
        "pid": {"controller_type": "position_torque_cascade_pid",
                "gains_file": args.pid_gains_file,
                "kp": 0.60, "ki": 0.005, "kd": 0.45,
                "total_torque_limit_nm": 20.0,
                "wheel_torque_limit_nm": 10.0,
                "rate_limit_u_nm_s": 0.0},
        "gs_lqr": {"controller_type": "gs_lqr_historical",
                   "config_file": args.gs_config_file,
                   "experiment_mode": "nominal", "gain_mode": "scheduled",
                   "gain_profile": "legacy_safe",
                   "five_node_heights_m": [0.30, 0.35, 0.40, 0.45, 0.50]},
        "model": {"world": args.world_name, "link": args.link_name,
                  "force_axis": args.force_axis, "force_N": 20.0},
        "git_head": command(["git", "rev-parse", "HEAD"]),
        "git_status": command(["git", "status", "--short"]),
        "build_command": "colcon build --packages-select bbot_balance_controller bbot_bringup --symlink-install",
        "file_sha256": {path: sha(path if os.path.isabs(path) else os.path.join(args.ws_root, path))
                        for path in files},
    }
    (root / "formal_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))


def launch_command(args, controller, trial, csv_path):
    h = trial["height"] if trial["job"] != "lift" else 0.30
    common = (f"source {args.ws_root}/install/setup.bash && ros2 launch bbot_bringup "
              f"bbot_gazebo.launch.py gazebo_start_paused:=true headless:=true gui:=false "
              f"gazebo_record_path:=/tmp/bbot_gazebo_record_{os.getpid()} "
              f"gazebo_world_name:={args.world_name} adaptive_target_height:={h:.3f} "
              f"adaptive_startup_height:={args.startup_height:.3f}")
    if controller == "position_pid":
        return (common + " controller_type:=position_torque_cascade_pid "
                f"position_pid_gains_file:={args.pid_gains_file} "
                f"position_pid_log_path:={csv_path} position_pid_target_height:={h:.3f} "
                "position_pid_kp:=0.60 position_pid_ki:=0.005 position_pid_kd:=0.45 "
                "position_pid_v_ref_limit:=0.40 position_pid_rate_limit_u:=0.0 "
                "position_pid_total_torque_max:=20.0 position_pid_wheel_torque_max:=10.0")
    return (common + " controller_type:=gs_lqr_historical "
            f"historical_gs_lqr_config_file:={args.gs_config_file} "
            f"adaptive_log_path:={csv_path} adaptive_target_height:={h:.3f}")


def run_one(args, controller, trial, root, attempt):
    tag = trial_tag(controller, trial)
    csv_path = root / f"{tag}.csv"
    launch_log = root / "launch_logs" / f"{tag}_attempt{attempt}.log"
    root.joinpath("launch_logs").mkdir(exist_ok=True)
    for path in (csv_path, Path(str(csv_path) + ".pulse.json")):
        if path.exists():
            path.unlink()
    cleanup()
    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs(env["ROS_HOME"], exist_ok=True)
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    os.makedirs(f"/tmp/bbot_gazebo_record_{os.getpid()}", exist_ok=True)
    pulse = None
    if trial["job"] == "push":
        pulse = ForcePulse(env, args.world_name, args.link_name, args.force_axis,
                           trial["force"], args.force_duration)
        pulse.start_bridge(str(root / "launch_logs"))
        pulse.arm(str(root / "launch_logs"))
    command_topic = ("/position_torque_cascade_pid/command"
                     if controller == "position_pid" else "/adaptive_lqr/command")
    spec = [{"trigger": f"reset_{tag}.trg", "topic": command_topic,
             "type": "String", "payload": "reset_position"}]
    if trial["job"] == "lift":
        spec += [
            {"trigger": f"up_{tag}.trg", "topic": "/target_height",
             "type": "Float64", "payload": "0.50"},
            {"trigger": f"down_{tag}.trg", "topic": "/target_height",
             "type": "Float64", "payload": "0.30"},
        ]
    helper = TrialHelper(env, spec)
    helper.start()
    proc = None
    reset_sent = False
    force_on = False
    stable_since = None
    target_lock_time = None
    target_reference_seen = None
    position_history = []
    lift_gate_pass_time = None
    height_up = False
    height_down = False
    lift_rise_start_time = None
    lift_high_reached_time = None
    lift_low_return_time = None
    lift_observation_end_time = None
    previous_height = None
    startup_ready = False
    fail_reason = None
    event_state = {
        "target_lock_time_sim": None,
        "static_gate": {
            "height_target_m": LIFT_LOW_HEIGHT_M,
            "height_tolerance_m": LIFT_HEIGHT_TOLERANCE_M,
            "velocity_tolerance_m_s": STABLE_VELOCITY_M_S,
            "pitch_tolerance_deg": STABLE_ANGLE_DEG,
            "pitch_rate_tolerance_rad_s": STABLE_RATE_RAD_S,
            "position_span_tolerance_m": LIFT_POSITION_SPAN_TOLERANCE_M,
            "hold_s": LIFT_STABLE_HOLD_S,
            "passed": False,
        },
        "lift_command_up_sim_time": None,
        "actual_rise_start_sim_time": None,
        "high_reached_sim_time": None,
        "lift_command_down_sim_time": None,
        "actual_low_return_sim_time": None,
        "observation_end_sim_time": None,
        "completed": False,
        "failure_reason": None,
    }
    last_sim = -1.0
    no_data_wall = time.time()
    try:
        with launch_log.open("w", encoding="utf-8") as log_file:
            cmd = launch_command(args, controller, trial, csv_path)
            proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                    stdout=log_file, stderr=subprocess.STDOUT,
                                    env=env, preexec_fn=os.setsid, text=True)
            startup_ready, detail = wait_for_startup(
                env, args.world_name, controller, args.startup_timeout)
            if not startup_ready:
                fail_reason = detail
            else:
                ok, detail = unpause(env, args.world_name)
                if not ok:
                    fail_reason = f"unpause_failed:{detail}"
                else:
                    print(f"[Formal] ready {tag}", flush=True)
            end_time = CONSTANT_END if trial["job"] == "constant" else \
                LIFT_END if trial["job"] == "lift" else PUSH_END
            while fail_reason is None:
                if proc.poll() is not None:
                    fail_reason = f"launch_exited:{proc.poll()}"
                    break
                rec = last_record(csv_path, controller)
                if rec is None:
                    if time.time() - no_data_wall > 60.0:
                        fail_reason = "no_csv_data"
                        break
                    time.sleep(0.2)
                    continue
                if rec["time"] > last_sim + 0.05:
                    last_sim = rec["time"]
                    no_data_wall = time.time()
                elif time.time() - no_data_wall > 45.0:
                    fail_reason = "controller_stopped_logging"
                    break
                if controller == "gs_lqr" and target_reference_seen is None:
                    target_reference_seen = rec["p_target"]
                if not reset_sent and rec["time"] >= RESET_SIM_TIME:
                    helper.fire(spec[0]["trigger"])
                    reset_sent = True
                    print(f"[Formal] reset {tag} t={rec['time']:.3f}", flush=True)
                if reset_sent and target_lock_time is None:
                    if controller == "position_pid" and rec["target_latched"] > 0.5:
                        target_lock_time = rec["time"]
                    elif controller == "gs_lqr":
                        if target_reference_seen is None:
                            target_reference_seen = rec["p_target"]
                        elif abs(rec["p_target"] - target_reference_seen) > 1e-4:
                            target_lock_time = rec["time"]
                    if target_lock_time is not None:
                        event_state["target_lock_time_sim"] = target_lock_time
                if rec["time"] > (position_history[-1][0] if position_history else -1.0):
                    position_history.append((rec["time"], rec["p"]))
                    position_history = [item for item in position_history
                                        if item[0] >= rec["time"] - 2.0]
                if reset_sent and trial["job"] == "push" and not force_on:
                    actual_angle = abs((rec["pitch"] - rec["theta_eq"]) * 180.0 / np.pi)
                    stable = (actual_angle <= STABLE_ANGLE_DEG and
                              abs(rec["pitch_rate"]) <= STABLE_RATE_RAD_S and
                              abs(rec["velocity"]) <= STABLE_VELOCITY_M_S)
                    stable_since = rec["time"] if stable and stable_since is None else stable_since
                    if not stable:
                        stable_since = None
                    if stable_since is not None and rec["time"] - stable_since >= STABLE_HOLD_S:
                        pulse.fire(rec["time"])
                        force_on = True
                        print(f"[Formal] force {trial['force']:+.0f} N {tag} t={rec['time']:.3f}", flush=True)
                if reset_sent and trial["job"] == "lift" and args.lift_stable_gated:
                    if target_lock_time is not None and not height_up:
                        span = (max(item[1] for item in position_history) -
                                min(item[1] for item in position_history)) \
                            if position_history else float("inf")
                        stable = (
                            abs(rec["height"] - LIFT_LOW_HEIGHT_M) <= LIFT_HEIGHT_TOLERANCE_M and
                            abs(rec["velocity"]) <= STABLE_VELOCITY_M_S and
                            abs((rec["pitch"] - rec["theta_eq"]) * 180.0 / np.pi) <= STABLE_ANGLE_DEG and
                            abs(rec["pitch_rate"]) <= STABLE_RATE_RAD_S and
                            span <= LIFT_POSITION_SPAN_TOLERANCE_M and
                            rec["time"] >= target_lock_time + 2.0)
                        stable_since = rec["time"] if stable and stable_since is None else stable_since
                        if not stable:
                            stable_since = None
                        if stable_since is not None and rec["time"] - stable_since >= LIFT_STABLE_HOLD_S:
                            helper.fire(spec[1]["trigger"])
                            height_up = True
                            lift_gate_pass_time = rec["time"]
                            event_state["static_gate"].update({
                                "passed": True,
                                "passed_sim_time": rec["time"],
                                "position_span_m": span,
                            })
                            event_state["lift_command_up_sim_time"] = rec["time"]
                            print(f"[Formal] gated lift up {tag} t={rec['time']:.3f} span={span:.4f}", flush=True)
                    if reset_sent and not height_up and rec["time"] >= RESET_SIM_TIME + LIFT_GATE_TIMEOUT_AFTER_RESET_S:
                        fail_reason = "lift_static_gate_timeout"
                        event_state["failure_reason"] = fail_reason
                        break
                    if height_up and lift_rise_start_time is None and rec["height"] >= LIFT_LOW_HEIGHT_M + LIFT_HEIGHT_TOLERANCE_M:
                        lift_rise_start_time = rec["time"]
                        event_state["actual_rise_start_sim_time"] = rec["time"]
                    if height_up and lift_high_reached_time is None and rec["height"] >= LIFT_HIGH_HEIGHT_M - LIFT_HEIGHT_TOLERANCE_M:
                        lift_high_reached_time = rec["time"]
                        event_state["high_reached_sim_time"] = rec["time"]
                    if (lift_high_reached_time is not None and not height_down and
                            rec["time"] - lift_high_reached_time >= LIFT_HIGH_HOLD_S):
                        helper.fire(spec[2]["trigger"])
                        height_down = True
                        event_state["lift_command_down_sim_time"] = rec["time"]
                        print(f"[Formal] gated lift down {tag} t={rec['time']:.3f}", flush=True)
                    if (height_down and lift_low_return_time is None and
                            rec["height"] <= LIFT_LOW_HEIGHT_M + LIFT_HEIGHT_TOLERANCE_M):
                        lift_low_return_time = rec["time"]
                        lift_observation_end_time = rec["time"] + LIFT_LOW_HOLD_S
                        event_state["actual_low_return_sim_time"] = rec["time"]
                        event_state["observation_end_sim_time"] = lift_observation_end_time
                    if (lift_observation_end_time is not None and
                            rec["time"] >= lift_observation_end_time):
                        event_state["completed"] = True
                        break
                elif reset_sent and trial["job"] == "lift":
                    elapsed = rec["time"] - RESET_SIM_TIME
                    if elapsed >= 5.0 and not height_up:
                        helper.fire(spec[1]["trigger"]); height_up = True
                    if elapsed >= 19.0 and not height_down:
                        helper.fire(spec[2]["trigger"]); height_down = True
                if (trial["job"] != "lift" or not args.lift_stable_gated) and rec["time"] >= end_time:
                    break
                if trial["job"] == "lift" and args.lift_stable_gated and rec["time"] >= RESET_SIM_TIME + 80.0:
                    fail_reason = "lift_completion_timeout"
                    event_state["failure_reason"] = fail_reason
                    break
                previous_height = rec["height"]
                time.sleep(0.2)
    finally:
        if pulse is not None:
            pulse.shutdown()
        helper.shutdown()
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                for _ in range(20):
                    if proc.poll() is not None:
                        break
                    time.sleep(0.25)
            except Exception:
                pass
        time.sleep(0.5)
        pulse_state = pulse.read_state() if pulse is not None else None
        cleanup()

    if trial["job"] == "lift" and args.lift_stable_gated and not event_state["completed"] and fail_reason is None:
        fail_reason = "lift_incomplete"
        event_state["failure_reason"] = fail_reason

    protocol_pulse_ok = True
    if trial["job"] == "push":
        protocol_pulse_ok = pulse_state_valid(pulse_state, args.force_duration)
    recording_protocol_error = False
    if fail_reason in ("no_csv_data", "controller_stopped_logging"):
        try:
            launch_text = launch_log.read_text(encoding="utf-8", errors="ignore")
            recording_protocol_error = (
                "Failed to activate controller" in launch_text or
                ("[ERROR] [spawner-" in launch_text and "]: process has died" in launch_text))
        except OSError:
            recording_protocol_error = False
    result = {
        "controller": controller, "trial_tag": tag, "attempt": attempt,
        "protocol_startup_ready": bool(startup_ready),
        "protocol_reset_sent": bool(reset_sent),
        "protocol_pulse_valid": bool(protocol_pulse_ok),
        "protocol_valid": bool(startup_ready and reset_sent and protocol_pulse_ok),
        "retryable_protocol_error": bool((not startup_ready) or
                                          (trial["job"] == "push" and pulse_state is not None and not protocol_pulse_ok) or
                                          recording_protocol_error),
        "fail_reason": fail_reason,
        "lift_static_gate_valid": bool(event_state["static_gate"]["passed"])
            if trial["job"] == "lift" and args.lift_stable_gated else "",
        "lift_completed": bool(event_state["completed"])
            if trial["job"] == "lift" and args.lift_stable_gated else "",
        "csv": str(csv_path),
        "config_source": args.pid_gains_file if controller == "position_pid" else args.gs_config_file,
        "launch_log": str(launch_log),
        "recording_protocol_error": recording_protocol_error,
    }
    if csv_path.exists():
        try:
            result.update(response_metrics(csv_path, controller, trial, pulse_state))
        except (OSError, ValueError) as exc:
            result["analysis_error"] = str(exc)
    if pulse_state is not None:
        with (Path(str(csv_path) + ".pulse.json")).open("w", encoding="utf-8") as handle:
            json.dump(pulse_state, handle, indent=2, sort_keys=True)
        result.update({f"pulse_{key}": value for key, value in pulse_state.items()})
    if trial["job"] == "lift" and args.lift_stable_gated:
        with (root / f"{tag}.events.json").open("w", encoding="utf-8") as handle:
            json.dump(event_state, handle, indent=2, sort_keys=True)
        result.update({f"lift_{key}": value for key, value in event_state.items()
                       if not isinstance(value, dict)})
    return result


def move_invalid(root, result):
    invalid = root / "invalid_attempts"
    invalid.mkdir(exist_ok=True)
    csv_path = Path(result["csv"])
    if csv_path.exists():
        csv_path.rename(invalid / csv_path.name.replace(".csv", f"_attempt{result['attempt']}_invalid.csv"))
    pulse = Path(str(csv_path) + ".pulse.json")
    if pulse.exists():
        pulse.rename(invalid / pulse.name.replace(".csv.pulse.json", f"_attempt{result['attempt']}_invalid.csv.pulse.json"))


def write_csv(rows, path):
    if not rows:
        return
    fields = sorted({key for row in rows for key in row if not isinstance(row[key], (dict, list))})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in fields} for row in rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--startup-timeout", type=float, default=75.0)
    parser.add_argument("--force-duration", type=float, default=0.20)
    parser.add_argument("--force-axis", default="y", choices=("x", "y"))
    parser.add_argument("--world-name", default="balance_test_world")
    parser.add_argument("--link-name", default="base_link")
    parser.add_argument("--startup-height", type=float, default=0.36)
    parser.add_argument("--pid-gains-file", required=True)
    parser.add_argument("--gs-config-file", required=True)
    parser.add_argument("--smoke", action="store_true",
                        help="Run one H=0.30 constant trial per controller instead of the formal matrix")
    parser.add_argument("--controller", choices=("both", "position_pid", "gs_lqr"), default="both")
    parser.add_argument("--lift-only", action="store_true",
                        help="Run only the three-repetition lift recheck per controller")
    parser.add_argument("--lift-stable-gated", action="store_true",
                        help="Gate lift commands on the specified low-height static condition")
    parser.add_argument("--rep-filter", type=int, default=None,
                        help="Run only one repetition, used for an allowed protocol retry")
    args = parser.parse_args()
    if args.repeats != 3 and not args.smoke:
        parser.error("formal Sec. 4.2 requires exactly --repeats 3")
    if not (PULSE_DURATION_TOLERANCE[0] <= args.force_duration <= PULSE_DURATION_TOLERANCE[1]):
        parser.error("--force-duration must be within 0.19-0.21 s")
    if args.rep_filter is not None and args.rep_filter < 1:
        parser.error("--rep-filter must be positive")
    root = Path(args.output_root)
    root.mkdir(parents=True, exist_ok=True)
    controllers = ("position_pid", "gs_lqr") if args.controller == "both" else (args.controller,)
    for controller in controllers:
        (root / controller / "launch_logs").mkdir(parents=True, exist_ok=True)
    capture_manifest(args, root)
    all_results = []
    for controller in controllers:
        controller_root = root / controller
        if args.lift_only:
            trials = [{"job": "lift", "height": 0.30, "force": 0.0, "rep": rep}
                      for rep in ([args.rep_filter] if args.rep_filter is not None
                                  else range(1, args.repeats + 1))]
        else:
            trials = ([{"job": "constant", "height": 0.30, "force": 0.0, "rep": 1}]
                      if args.smoke else build_trials(args.repeats))
        for index, trial in enumerate(trials, 1):
            tag = trial_tag(controller, trial)
            print(f"\n[{controller} {index}/{len(trials)}] {tag}", flush=True)
            result = None
            for attempt in range(1, 4):
                result = run_one(args, controller, trial, controller_root, attempt)
                if result["protocol_valid"]:
                    break
                if not result["retryable_protocol_error"] or attempt >= 3:
                    break
                move_invalid(controller_root, result)
                print(f"[Formal] protocol invalid; retrying {tag} ({attempt + 1}/3)", flush=True)
            all_results.append(result)
            write_csv(all_results, root / "formal_results.csv")
            write_csv([r for r in all_results if r["controller"] == controller],
                      controller_root / "formal_results.csv")
    (root / "formal_results.json").write_text(json.dumps(all_results, ensure_ascii=False, indent=2, default=str))
    print(f"Formal results written to {root}")


if __name__ == "__main__":
    main()
