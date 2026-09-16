#!/usr/bin/env python3
"""Single-run exploration campaign for the independent four-loop PID.

This runner never changes the existing three-loop PID or the GS-LQR logs.  It
launches ``position_torque_cascade_pid`` and verifies that the first stable
``reset_position`` latches one immutable position target.  Push pulses use the
exact-once helper from ``run_height_campaign.py`` and are checked by sim time.
"""

import argparse
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from run_height_campaign import (  # noqa: E402
    ForcePulse,
    TrialHelper,
    cleanup_lingering_processes,
    pulse_state_valid,
    PULSE_DURATION_TOLERANCE,
)

HEIGHTS = (0.30, 0.40, 0.50)
GAIN_MODES = ("scheduled", "fixed_midpoint")
RESET_SIM_TIME = 6.74
CONSTANT_END = 35.0
LIFT_END = 38.0
PUSH_END = 40.0


def build_trials(job, repeats=1, force=20.0, height=None):
    if job == "constant":
        hs = (height,) if height is not None else HEIGHTS
        return [{"job": "constant", "height": h, "gain_mode": "scheduled",
                 "force": 0.0, "rep": r}
                for h in hs for r in range(1, repeats + 1)]
    if job == "lift":
        return [{"job": "lift", "height": 0.30, "gain_mode": "scheduled",
                 "force": 0.0, "rep": r} for r in range(1, repeats + 1)]
    if job in ("push", "calibration_push"):
        hs = (height,) if height is not None else HEIGHTS
        f = abs(force)
        return [{"job": "push", "height": h, "gain_mode": "scheduled",
                 "force": s * f, "rep": r}
                for h in hs for s in (1.0, -1.0) for r in range(1, repeats + 1)]
    if job == "exploration":
        trials = build_trials("constant", 1)
        trials += build_trials("lift", 1)
        trials += build_trials("push", 1, force)
        return trials
    raise ValueError(f"unknown job: {job}")


def trial_tag(trial):
    if trial["job"] == "push":
        return f"push_{trial['height']:g}_f{trial['force']:+g}_{trial['rep']}"
    if trial["job"] == "lift":
        return f"lift_{trial['rep']}"
    return f"constant_{trial['height']:g}_{trial['rep']}"


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("empty CSV")
    names = rows[0].keys()
    required = {
        "time", "height", "p", "p_target", "delta_p", "position_target_latched",
        "position_reset_event_count", "position_latch_count", "v", "v_ref", "v_ref_raw", "position_integral",
        "pitch", "pitch_rate", "theta_error", "u_clamped", "is_saturated",
        "force_active", "force_value",
    }
    missing = required - set(names)
    if missing:
        raise ValueError(f"missing columns: {sorted(missing)}")
    out = {name: np.asarray([float(row[name]) for row in rows]) for name in required}
    return out


def last_record(path):
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            return None
        row = rows[-1]
        return {k: float(row[k]) for k in (
            "time", "p_target", "delta_p", "position_target_latched",
            "position_reset_event_count", "position_latch_count", "v", "pitch", "pitch_rate", "theta_error")}
    except (OSError, ValueError, KeyError):
        return None


def service_ok(output):
    compact = output.lower().replace(" ", "")
    return "success:true" in compact or "success=true" in compact


def ros_cli(env, args, timeout=3.0):
    try:
        return subprocess.run(["ros2", *args], env=env, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def world_control(env, world_name, request):
    try:
        return subprocess.run(
            ["ros2", "service", "call", f"/world/{world_name}/control",
             "ros_gz_interfaces/srv/ControlWorld", request], env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5.0).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def wait_for_startup(env, world_name, timeout):
    deadline = time.monotonic() + timeout
    service = f"/world/{world_name}/control"
    paused = False
    last = ""
    while time.monotonic() < deadline:
        services = ros_cli(env, ["service", "list"])
        if service in services and not paused:
            paused = service_ok(world_control(env, world_name, "{world_control: {pause: true}}"))
        if service in services and paused:
            world_control(env, world_name, "{world_control: {step: true, multi_step: 10}}")
        nodes = ros_cli(env, ["node", "list"])
        controllers = ros_cli(env, ["control", "list_controllers", "-c", "/controller_manager"])
        info = ros_cli(env, ["node", "info", "/position_torque_cascade_pid_controller"])
        active = any("wheel_effort_controller" in line and "active" in line.lower()
                     for line in controllers.splitlines())
        subscriptions = all(topic in info for topic in (
            "/imu", "/joint_states", "/target_height",
            "/position_torque_cascade_pid/command"))
        if paused and service in services and active and \
                "/position_torque_cascade_pid_controller" in nodes and subscriptions:
            return True, "ready"
        last = f"pause={paused}, node={'yes' if '/position_torque_cascade_pid_controller' in nodes else 'no'}, active={active}, subscriptions={subscriptions}"
        time.sleep(0.5)
    return False, f"startup readiness timeout ({last})"


def unpause(env, world_name):
    output = world_control(env, world_name, "{world_control: {pause: false}}")
    return service_ok(output), output[-200:]


def cleanup_position_processes():
    cleanup_lingering_processes()
    # In this workspace Gazebo is launched through the ruby ign wrapper; the
    # generic cleanup pattern does not always match that wrapper process.
    subprocess.run(["pkill", "-9", "-f", "ruby.*gazebo"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "position_torque_cascade_pid_controller"],
                   stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def analyze(path, trial, pulse_state=None, force_duration=0.2):
    result = {"fail_reason": None, "protocol_valid": False}
    try:
        d = read_csv(path)
    except (OSError, ValueError) as exc:
        result["fail_reason"] = f"csv_invalid:{exc}"
        return result
    t = d["time"]
    latched = d["position_target_latched"] > 0.5
    if not np.any(latched):
        result["fail_reason"] = "position_target_not_latched"
        return result
    first = int(np.where(latched)[0][0])
    post = np.arange(len(t)) >= first
    target = d["p_target"][post]
    target_constant = np.ptp(target) <= 1e-8
    latch_count = int(np.max(d["position_latch_count"]))
    result.update({
        "target_lock_count": latch_count,
        "target_constant": bool(target_constant),
        "reset_event_count": int(np.max(d["position_reset_event_count"])),
        "position_peak_mm": float(np.max(np.abs(d["delta_p"][post])) * 1000.0),
        "position_rms_mm": float(np.sqrt(np.mean(d["delta_p"][post] ** 2)) * 1000.0),
        "final_position_error_mm": float(d["delta_p"][-1] * 1000.0),
        "pitch_peak_deg": float(np.max(np.abs(d["theta_error"][post])) * 180.0 / np.pi),
        "velocity_peak_mps": float(np.max(np.abs(d["v"][post]))),
        "torque_peak_nm": float(np.max(np.abs(d["u_clamped"][post]))),
        "torque_rms_nm": float(np.sqrt(np.mean(d["u_clamped"][post] ** 2))),
        "torque_saturation_ratio": float(np.mean(d["is_saturated"][post] > 0.5)),
        "position_ref_saturation_ratio": float(np.mean(np.abs(d["v_ref_raw"][post] - d["v_ref"][post]) > 1e-8)),
        "position_integral_peak": float(np.max(np.abs(d["position_integral"][post]))),
    })
    result["protocol_valid"] = bool(latch_count == 1 and target_constant)
    if not result["protocol_valid"]:
        result["fail_reason"] = "position_target_relock_or_moved"

    if trial["job"] == "push":
        state_ok = pulse_state_valid(pulse_state, force_duration)
        force_mask = (t >= float(pulse_state.get("pulse_start_sim_time", 0.0))) & \
            (t <= float(pulse_state.get("pulse_end_sim_time", 0.0)) + 0.5) if pulse_state else np.zeros_like(t, dtype=bool)
        marker_seen = bool(np.any(d["force_active"] > 0.5) and np.any(np.abs(d["force_value"]) > 1e-3))
        if not state_ok or not marker_seen:
            result["fail_reason"] = result["fail_reason"] or "pulse_protocol_invalid"
            result["protocol_valid"] = False
        if pulse_state and pulse_state.get("pulse_start_sim_time") is not None:
            t0 = float(pulse_state["pulse_start_sim_time"])
            t1 = float(pulse_state["pulse_end_sim_time"])
            before = (t >= t0 - 0.25) & (t < t0)
            response = (t >= t0 + 0.10) & (t <= t0 + 0.50)
            dv = float(np.median(d["v"][response]) - np.median(d["v"][before])) if np.any(before) and np.any(response) else np.nan
            result["initial_velocity_change_mps"] = dv
            result["expected_velocity_sign"] = 1.0 if trial["force"] > 0 else -1.0
            result["velocity_direction_ok"] = bool(np.isfinite(dv) and abs(dv) >= 0.03 and dv * result["expected_velocity_sign"] > 0)
            result["pulse_duration_s"] = t1 - t0
            result["pulse_start_sim_time"] = t0
            result["pulse_end_sim_time"] = t1
            if not result["velocity_direction_ok"]:
                result["fail_reason"] = result["fail_reason"] or "wrong_push_response_direction"
            after = t >= t1
            result["attitude_velocity_recovery_s"] = recovery_time(
                t, d["theta_error"] * 180.0 / np.pi, d["v"], t1, 0.5, 0.02, 2.0)
            result["position_10mm_recovery_s"] = position_recovery_time(
                t, d["delta_p"] * 1000.0, t1, 10.0, 2.0)
    elif trial["job"] == "constant":
        ss = t >= t[-1] - 10.0
        result["steady_pitch_rms_deg"] = float(np.sqrt(np.mean((d["theta_error"][ss] * 180.0 / np.pi) ** 2)))
        result["steady_velocity_rms_mps"] = float(np.sqrt(np.mean(d["v"][ss] ** 2)))
    elif trial["job"] == "lift":
        result["height_end_m"] = float(d["height"][-1]) if "height" in d else np.nan
    return result


def recovery_time(t, pitch_deg, velocity, start, pitch_limit, velocity_limit, hold):
    ok = (t >= start) & (np.abs(pitch_deg) <= pitch_limit) & (np.abs(velocity) <= velocity_limit)
    return continuous_time(t, ok, start, hold)


def position_recovery_time(t, position_mm, start, limit, hold):
    return continuous_time(t, (np.abs(position_mm) <= limit) & (t >= start), start, hold)


def continuous_time(t, ok, start, hold):
    idx = np.where(t >= start)[0]
    if len(idx) == 0:
        return np.nan
    for i in idx:
        j = i
        while j < len(t) and ok[j]:
            if t[j] - t[i] >= hold:
                return float(t[i] - start)
            j += 1
    return np.nan


def run_trial(trial, args, env, csv_path, attempt=1):
    cleanup_position_processes()
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    if os.path.exists(csv_path):
        os.remove(csv_path)
    if os.path.exists(csv_path + ".pulse.json"):
        os.remove(csv_path + ".pulse.json")
    tag = trial_tag(trial)
    log_path = os.path.join(args.log_dir, f"{tag}_attempt{attempt}.log")
    log_file = open(log_path, "w", encoding="utf-8")
    target_height = trial["height"] if trial["job"] in ("constant", "push") else 0.30
    cmd = (
        f"source {args.ws_root}/install/setup.bash && ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"controller_type:=position_torque_cascade_pid gazebo_start_paused:=true headless:=true gui:=false "
        f"gazebo_world_name:={args.world_name} position_pid_target_height:={target_height:.3f} "
        f"position_pid_gains_file:={args.gains_file} "
        f"position_pid_startup_height:={args.startup_height:.3f} "
        f"position_pid_log_path:={csv_path} position_pid_kp:={args.position_kp} "
        f"position_pid_ki:={args.position_ki} position_pid_kd:={args.position_kd} "
        f"position_pid_v_ref_limit:={args.v_ref_limit}"
    )
    pulse = None
    if trial["job"] == "push":
        pulse = ForcePulse(env, args.world_name, args.link_name, args.force_axis,
                           trial["force"], args.force_duration)
        pulse.start_bridge(args.log_dir)
        pulse.arm(args.log_dir)
    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=log_file,
                            stderr=subprocess.STDOUT, env=env, preexec_fn=os.setsid, text=True)
    tag_base = f"{tag}_a{attempt}"
    spec = [{"trigger": f"reset_{tag_base}.trg", "topic": "/position_torque_cascade_pid/command",
             "type": "String", "payload": "reset_position"}]
    if trial["job"] == "lift":
        spec += [{"trigger": f"up_{tag_base}.trg", "topic": "/target_height", "type": "Float64", "payload": "0.50"},
                 {"trigger": f"down_{tag_base}.trg", "topic": "/target_height", "type": "Float64", "payload": "0.30"}]
    helper = TrialHelper(env, spec)
    helper.start()
    pulse_state = None
    reset_sent = False
    steady_since = None
    force_on = False
    height_up = False
    height_down = False
    fail_reason = None
    last_data_wall = time.time()
    last_t = -1.0
    try:
        ready, detail = wait_for_startup(env, args.world_name, args.startup_timeout)
        if not ready:
            fail_reason = detail
        else:
            ok, detail = unpause(env, args.world_name)
            if not ok:
                fail_reason = f"unpause_failed:{detail}"
            else:
                print(f"[Runner] four-loop startup ready: {tag}", flush=True)
        end_time = CONSTANT_END if trial["job"] == "constant" else LIFT_END if trial["job"] == "lift" else PUSH_END
        while fail_reason is None:
            if proc.poll() is not None:
                fail_reason = f"launch_exited:{proc.poll()}"
                break
            rec = last_record(csv_path)
            if rec is None:
                if time.time() - last_data_wall > 60.0:
                    fail_reason = "no_csv_data"
                time.sleep(0.2)
                continue
            if rec["time"] > last_t + 0.05:
                last_t = rec["time"]
                last_data_wall = time.time()
            pitch_deg = abs(rec.get("pitch", 0.0)) * 180.0 / np.pi
            if pitch_deg > args.fall_pitch_deg:
                fail_reason = f"fall_pitch_deg:{pitch_deg:.3f}"
                break
            if not reset_sent and rec["time"] >= RESET_SIM_TIME:
                helper.fire(spec[0]["trigger"])
                print(f"[Runner] reset_position fired t={rec['time']:.3f}", flush=True)
                reset_sent = True
            if reset_sent and rec["position_latch_count"] >= 1 and not force_on and trial["job"] == "push":
                stable = abs(rec["theta_error"] if "theta_error" in rec else 0.0) <= 0.20 * np.pi / 180.0 and \
                    abs(rec["pitch_rate"]) <= 0.02 and abs(rec["v"]) <= 0.01
                steady_since = rec["time"] if stable and steady_since is None else steady_since
                if not stable:
                    steady_since = None
                if steady_since is not None and rec["time"] - steady_since >= 5.0:
                    pulse.fire(rec["time"])
                    force_on = True
                    print(f"[Runner] force {trial['force']:+.1f} N fired t={rec['time']:.3f}", flush=True)
            if reset_sent and trial["job"] == "lift":
                elapsed = rec["time"] - RESET_SIM_TIME
                if elapsed >= 5.0 and not height_up:
                    helper.fire(spec[1]["trigger"]); height_up = True
                if elapsed >= 19.0 and not height_down:
                    helper.fire(spec[2]["trigger"]); height_down = True
            if rec["time"] >= end_time:
                break
            time.sleep(0.2)
    finally:
        if pulse is not None:
            pulse.shutdown()
            pulse_state = pulse.read_state()
        helper.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(20):
                if proc.poll() is not None: break
                time.sleep(0.25)
        except Exception:
            pass
        cleanup_position_processes()
        log_file.close()
    metrics = analyze(csv_path, trial, pulse_state, args.force_duration)
    metrics.update({"trial": trial, "attempt": attempt, "fail_reason": fail_reason or metrics.get("fail_reason")})
    if pulse_state is not None:
        with open(csv_path + ".pulse.json", "w", encoding="utf-8") as handle:
            json.dump(pulse_state, handle, indent=2, sort_keys=True)
    return metrics


def write_summary(results, path):
    fields = sorted({k for result in results for k, value in result.items()
                     if k != "trial" and not isinstance(value, (dict, list))})
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({field: result.get(field, "") for field in fields})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", choices=["constant", "lift", "push", "calibration_push", "exploration"], default="exploration")
    parser.add_argument("--height", type=float, default=None)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--force", type=float, default=20.0)
    parser.add_argument("--force-duration", type=float, default=0.20)
    parser.add_argument("--force-axis", choices=["x", "y"], default="y")
    parser.add_argument("--world-name", default="balance_test_world")
    parser.add_argument("--link-name", default="base_link")
    parser.add_argument("--startup-height", type=float, default=0.36)
    parser.add_argument("--startup-timeout", type=float, default=75.0)
    parser.add_argument("--fall-pitch-deg", type=float, default=40.0)
    parser.add_argument("--ws-root", default="/home/admin/bbot_ws_new")
    parser.add_argument("--data-dir", default=None)
    parser.add_argument(
        "--gains-file",
        default="/home/admin/bbot_ws_new/src/bbot_balance_controller/config/position_torque_cascade_pid_gains.yaml",
    )
    parser.add_argument("--position-kp", type=float, default=0.50)
    parser.add_argument("--position-ki", type=float, default=0.010)
    parser.add_argument("--position-kd", type=float, default=0.30)
    parser.add_argument("--v-ref-limit", type=float, default=0.40)
    args = parser.parse_args()
    if not (PULSE_DURATION_TOLERANCE[0] <= args.force_duration <= PULSE_DURATION_TOLERANCE[1]):
        parser.error("--force-duration must be within 0.19-0.21 s")
    if args.data_dir is None:
        args.data_dir = os.path.join(args.ws_root, "src/bbot_balance_controller/src/data_logs/position_torque_pid_exploration")
    args.log_dir = os.path.join(args.data_dir, "launch_logs")
    os.makedirs(args.data_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs(env["ROS_HOME"], exist_ok=True)
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    trials = build_trials(args.job, args.repeats, args.force, args.height)
    results = []
    attempt_records = []
    for index, trial in enumerate(trials, 1):
        tag = trial_tag(trial)
        csv_path = os.path.join(args.data_dir, f"{tag}.csv")
        print(f"\n[{index}/{len(trials)}] {tag}", flush=True)
        result = None
        for attempt in range(1, 4):
            result = run_trial(trial, args, env, csv_path, attempt)
            attempt_records.append({"tag": tag, "attempt": attempt,
                                    "protocol_valid": result.get("protocol_valid", False),
                                    "fail_reason": result.get("fail_reason")})
            # Only protocol failures may be retried.  A valid protocol followed
            # by a fall or missed recovery is an actual exploratory result.
            if result.get("protocol_valid") or trial["job"] != "push" and result.get("protocol_valid"):
                break
            if attempt < 3:
                failed = csv_path.replace(".csv", f"_attempt{attempt}_invalid.csv")
                if os.path.exists(csv_path): os.replace(csv_path, failed)
                if os.path.exists(csv_path + ".pulse.json"):
                    os.replace(csv_path + ".pulse.json", failed + ".pulse.json")
                print(f"[Runner] protocol invalid; retry {attempt + 1}/3", flush=True)
        result["trial_tag"] = tag
        result["config_source"] = args.gains_file
        results.append(result)
        write_summary(results, os.path.join(args.data_dir, "exploration_summary.csv"))
    with open(os.path.join(args.data_dir, "attempts.json"), "w", encoding="utf-8") as handle:
        json.dump(attempt_records, handle, indent=2, sort_keys=True)
    print(f"\nSummary written to {os.path.join(args.data_dir, 'exploration_summary.csv')}")


if __name__ == "__main__":
    main()
