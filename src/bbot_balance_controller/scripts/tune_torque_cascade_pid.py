#!/usr/bin/env python3
"""Measured hierarchical tuning for the three-loop torque PID baseline.

The controller is tuned in the order rate -> attitude -> velocity.  Each
candidate runs in the corresponding controller stage and is accepted only
when it produces a finite, non-fallen response to the requested excitation.
The runtime YAML is written only after both endpoint campaigns and H=0.40
interpolation validation succeed.  Position is never a tuning objective.
"""

import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np


WS_ROOT = "/home/admin/bbot_ws_new"
DATA_LOG_DIR = os.path.join(WS_ROOT, "src/bbot_balance_controller/src/data_logs/torque_pid_tuning")
OUTPUT_GAINS_YAML = os.path.join(WS_ROOT, "src/bbot_balance_controller/config/torque_cascade_pid_gains.yaml")
SUMMARY_PATH = os.path.join(DATA_LOG_DIR, "tuning_summary.json")
DT = 0.005
DEG = 180.0 / math.pi


def cleanup():
    for proc_name in ("ign gazebo", "gz sim", "ruby", "ros_gz_bridge", "parameter_bridge",
                      "torque_cascade_pid_controller", "spawner", "robot_state_publisher", "ros2 launch"):
        subprocess.run(["pkill", "-9", "-f", proc_name], stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def band_energy(values, dt=DT):
    values = np.asarray(values, dtype=float)
    if values.size < 64:
        return float("nan")
    centered = values - np.mean(values)
    spectrum = np.abs(np.fft.rfft(centered)) ** 2
    frequencies = np.fft.rfftfreq(values.size, d=dt)
    band = (frequencies >= 6.0) & (frequencies <= 12.0)
    return float(np.sum(spectrum[band]) / (np.sum(spectrum) + 1e-12))


def read_log_rows(path):
    time.sleep(0.3)
    if not os.path.exists(path) or os.path.getsize(path) < 100:
        return []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        return [row for row in reader if len(row) == len(fields) and row.get("pitch")]


def farray(rows, key):
    try:
        return np.asarray([float(row[key]) for row in rows], dtype=float)
    except (KeyError, ValueError):
        return np.asarray([], dtype=float)


def evaluate_response(rows, eval_start, excitation_end, mode):
    """Compute all tuning metrics and reject unusable trials."""
    if len(rows) < 120:
        return {"valid": False, "reason": "insufficient_data", "samples": len(rows)}
    times = farray(rows, "time")
    pitch_error_deg = (farray(rows, "pitch") - farray(rows, "theta_eq")) * DEG
    rate = farray(rows, "pitch_rate")
    velocity = farray(rows, "v")
    torque = farray(rows, "u_clamped")
    saturated = farray(rows, "is_saturated")
    enabled = farray(rows, "control_enabled")
    arrays = (times, pitch_error_deg, rate, velocity, torque, saturated)
    if any(a.size != len(rows) or not np.all(np.isfinite(a)) for a in arrays):
        return {"valid": False, "reason": "non_finite_log", "samples": len(rows)}
    if enabled.size and np.min(enabled) < 0.5:
        return {"valid": False, "reason": "controller_disabled", "samples": len(rows)}

    settle = times >= eval_start
    if not np.any(settle):
        return {"valid": False, "reason": "missing_evaluation_window"}
    max_pitch = float(np.max(np.abs(pitch_error_deg)))
    sat_ratio = float(np.mean(saturated[settle]))
    if max_pitch > (15.0 if mode == "stage_a" else 25.0):
        return {"valid": False, "reason": "pitch_limit", "max_pitch_error_deg": max_pitch}
    if sat_ratio >= 0.95:
        return {"valid": False, "reason": "persistent_torque_saturation", "sat_ratio": sat_ratio}

    # Stage A is deliberately an inner-loop-only test.  It cannot hold the
    # inverted-pendulum angle without the attitude loop, so its recovery gate
    # is angular-rate recovery plus a safe pitch excursion.  Stages B/C use
    # the full attitude/velocity recovery gates.
    if mode == "stage_a":
        stable = np.abs(rate) <= 0.15
    else:
        stable = (np.abs(pitch_error_deg) <= 0.5) & (np.abs(rate) <= 0.15)
    if mode == "stage_c":
        stable &= np.abs(velocity) <= 0.02
    window = max(1, int(round(0.5 / DT)))
    recovery_time = None
    for idx in np.where(times >= (excitation_end or times[0]))[0]:
        if idx + window <= len(times) and np.all(stable[idx:idx + window]):
            recovery_time = float(times[idx] - (excitation_end or times[0]))
            break

    response = times >= eval_start
    metrics = {
        "valid": recovery_time is not None and recovery_time <= 8.0,
        "reason": "ok" if recovery_time is not None and recovery_time <= 8.0 else "no_recovery",
        "samples": len(rows),
        "steady_pitch_rms_deg": float(np.sqrt(np.mean(pitch_error_deg[settle] ** 2))),
        "steady_pitch_rate_rms_rad_s": float(np.sqrt(np.mean(rate[settle] ** 2))),
        "steady_velocity_rms_m_s": float(np.sqrt(np.mean(velocity[settle] ** 2))),
        "max_pitch_error_deg": max_pitch,
        "recovery_time_s": float(recovery_time) if recovery_time is not None else 999.0,
        "torque_peak_nm": float(np.max(np.abs(torque[response]))),
        "torque_rms_nm": float(np.sqrt(np.mean(torque[response] ** 2))),
        "saturation_ratio": sat_ratio,
        "pitch_6_12hz_energy": band_energy(pitch_error_deg[settle]),
        "torque_6_12hz_energy": band_energy(torque[settle]),
    }
    return metrics


def trial_command(log_path, stage_mode, height, duration, gains, disturbances, initial_roll):
    args = [
        f"source {WS_ROOT}/setup_env.sh &&", "ros2 launch bbot_bringup bbot_gazebo.launch.py",
        "controller_type:=torque_cascade_pid", "world:=balance_test_world.sdf",
        f"torque_pid_log_path:={log_path}", f"torque_pid_stage_mode:={stage_mode}",
        f"torque_pid_startup_height:={height:.2f}", f"torque_pid_target_height:={height:.2f}",
        f"torque_pid_initial_roll:={initial_roll:.6f}",
        "torque_pid_startup_hold_time:=1.0", "torque_pid_leg_transition_speed:=0.10",
        "torque_pid_k_x:=0.0", "torque_pid_rate_limit_u:=0.0",
        "torque_pid_total_torque_max:=20.0", "torque_pid_wheel_torque_max:=10.0",
    ]
    args.extend(f"{key}:={value}" for key, value in gains.items())
    args.extend(f"{key}:={value}" for key, value in disturbances.items())
    return " ".join(args)


def run_trial(stage_mode, height, duration, gains, disturbances, log_name, eval_start, excitation_end,
              initial_roll=0.0):
    cleanup()
    os.makedirs(DATA_LOG_DIR, exist_ok=True)
    log_path = os.path.join(DATA_LOG_DIR, log_name)
    if os.path.exists(log_path):
        os.remove(log_path)
    proc = subprocess.Popen(trial_command(log_path, stage_mode, height, duration, gains, disturbances, initial_roll),
                            shell=True, executable="/bin/bash", preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    started = False
    begin = time.time()
    try:
        while time.time() - begin < 45.0:
            if os.path.exists(log_path) and os.path.getsize(log_path) > 300:
                started = True
                break
            if proc.poll() is not None:
                break
            time.sleep(0.25)
        if started:
            time.sleep(duration)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(12):
                if proc.poll() is not None:
                    break
                time.sleep(0.25)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
        cleanup()
    rows = read_log_rows(log_path)
    if not started:
        return {"valid": False, "reason": "simulation_start_failed", "samples": len(rows)}
    return evaluate_response(rows, eval_start, excitation_end, stage_mode)


def endpoint_gains(prefix, values):
    return {f"torque_pid_{prefix}_{key}": values[key]
            for key in ("kp_rate", "kd_rate", "kp_theta", "kd_theta", "kp_v", "ki_v", "kd_v")}


def both_endpoint_gains(prefix, values):
    result = endpoint_gains(prefix, values)
    other = "high" if prefix == "low" else "low"
    result.update(endpoint_gains(other, values))
    return result


def run_stage(stage, height, prefix, candidates, fixed, records):
    best = None
    best_cost = float("inf")
    for index, candidate in enumerate(candidates, start=1):
        values = dict(fixed)
        values.update(candidate)
        if stage == "stage_a":
            disturbance = {"torque_pid_rate_disturbance_step": 0.30,
                           "torque_pid_rate_disturbance_start_time": 0.25}
            duration, eval_start, excitation_end = 1.5, 0.25, 0.75
            initial_roll = 0.0
        elif stage == "stage_b":
            disturbance = {"torque_pid_attitude_disturbance_step": 0.05236,
                           "torque_pid_disturbance_start_time": 2.0}
            duration, eval_start, excitation_end = 10.0, 2.0, 3.0
            # Apply the approximately 3-degree attitude deviation only after
            # the leg/effort interfaces are live.  A physical spawn tilt would
            # fall during the controller-manager startup gap and would measure
            # startup latency rather than the attitude loop.
            initial_roll = 0.0
        else:
            disturbance = {"torque_pid_velocity_disturbance_step": 0.15,
                           "torque_pid_velocity_disturbance_start_time": 2.0}
            duration, eval_start, excitation_end = 12.0, 2.0, 2.5
            initial_roll = 0.0
        log_name = f"{stage}_{prefix}_cand{index}.csv"
        print(f"  {stage} {prefix} candidate {index}/{len(candidates)}: {candidate}")
        metrics = run_trial(stage, height, duration, both_endpoint_gains(prefix, values),
                            disturbance, log_name, eval_start, excitation_end, initial_roll)
        records.append({"stage": stage, "height": height, "endpoint": prefix,
                        "candidate": candidate, "log": log_name, "metrics": metrics})
        if not metrics.get("valid", False):
            print(f"    INVALID: {metrics.get('reason', 'unknown')}")
            continue
        cost = (12.0 * metrics["steady_pitch_rms_deg"]
                + 3.0 * metrics["steady_pitch_rate_rms_rad_s"]
                + 25.0 * metrics["steady_velocity_rms_m_s"]
                + 0.4 * metrics["torque_peak_nm"] + 0.2 * metrics["torque_rms_nm"]
                + 15.0 * metrics["saturation_ratio"]
                + 6.0 * (metrics["pitch_6_12hz_energy"] + metrics["torque_6_12hz_energy"])
                + 0.5 * metrics["recovery_time_s"])
        records[-1]["cost"] = cost
        print(f"    valid cost={cost:.4f}, pitch_rms={metrics['steady_pitch_rms_deg']:.4f} deg, "
              f"rate_rms={metrics['steady_pitch_rate_rms_rad_s']:.4f}, "
              f"v_rms={metrics['steady_velocity_rms_m_s']:.4f}, "
              f"recovery={metrics['recovery_time_s']:.3f}s")
        if cost < best_cost:
            best_cost, best = cost, candidate
    if best is None:
        raise RuntimeError(f"[{stage}] no valid candidate at H={height:.2f} m; refusing to generate YAML")
    return best


def tune_endpoint(height, prefix, records):
    a = run_stage("stage_a", height, prefix,
                  [{"kp_rate": kp, "kd_rate": kd} for kp, kd in
                   ((14.0, 0.010), (18.0, 0.015), (22.0, 0.020), (26.0, 0.030))],
                  {"kp_rate": 20.0, "kd_rate": 0.020, "kp_theta": 5.5, "kd_theta": 0.10,
                   "kp_v": 0.08, "ki_v": 0.008, "kd_v": 0.001}, records)
    b = run_stage("stage_b", height, prefix,
                  [{"kp_theta": kp, "kd_theta": kd} for kp, kd in
                   ((4.0, 0.06), (5.0, 0.08), (6.0, 0.10), (7.0, 0.14))],
                  {"kp_rate": a["kp_rate"], "kd_rate": a["kd_rate"], "kp_theta": 5.5,
                   "kd_theta": 0.10, "kp_v": 0.08, "ki_v": 0.008, "kd_v": 0.001}, records)
    c = run_stage("stage_c", height, prefix,
                  [{"kp_v": kp, "ki_v": ki, "kd_v": kd} for kp, ki, kd in
                   ((0.06, 0.006, 0.0005), (0.08, 0.008, 0.0010),
                    (0.10, 0.012, 0.0010), (0.12, 0.016, 0.0015))],
                  {"kp_rate": a["kp_rate"], "kd_rate": a["kd_rate"], "kp_theta": b["kp_theta"],
                   "kd_theta": b["kd_theta"], "kp_v": 0.08, "ki_v": 0.008, "kd_v": 0.001}, records)
    return {"height": height, "kp_rate": a["kp_rate"], "kd_rate": a["kd_rate"],
            "kp_theta": b["kp_theta"], "kd_theta": b["kd_theta"],
            "kp_v": c["kp_v"], "ki_v": c["ki_v"], "kd_v": c["kd_v"]}


def validate_interpolation(low, high, records):
    print("  Stage D: validating endpoint interpolation at H=0.40 m")
    gains = endpoint_gains("low", low)
    gains.update(endpoint_gains("high", high))
    metrics = run_trial("normal", 0.40, 16.0, gains,
                        {"torque_pid_velocity_disturbance_step": 0.15,
                        "torque_pid_velocity_disturbance_start_time": 3.0},
                        "stage_d_validation_0.40m.csv", 7.0, 3.5, 0.0)
    if not metrics.get("valid", False):
        raise RuntimeError(f"[Stage D] H=0.40 m validation failed: {metrics.get('reason')}")
    records.append({"stage": "stage_d", "height": 0.40,
                    "metrics": metrics, "log": "stage_d_validation_0.40m.csv"})
    return metrics


def write_yaml(low, high):
    content = f"""# Measured output of tune_torque_cascade_pid.py
# Three-loop cascade: velocity -> attitude -> angular rate -> wheel torque
# Position is observation-only; k_x is fixed at zero.
torque_cascade_pid_controller:
  ros__parameters:
    pid:
      k_x: 0.0
      rate_limit_u: 0.0
      total_torque_max: 20.0
      wheel_torque_max: 10.0
      low:
        kp_rate: {low['kp_rate']:.12g}
        kd_rate: {low['kd_rate']:.12g}
        kp_theta: {low['kp_theta']:.12g}
        kd_theta: {low['kd_theta']:.12g}
        kp_v: {low['kp_v']:.12g}
        ki_v: {low['ki_v']:.12g}
        kd_v: {low['kd_v']:.12g}
      high:
        kp_rate: {high['kp_rate']:.12g}
        kd_rate: {high['kd_rate']:.12g}
        kp_theta: {high['kp_theta']:.12g}
        kd_theta: {high['kd_theta']:.12g}
        kp_v: {high['kp_v']:.12g}
        ki_v: {high['ki_v']:.12g}
        kd_v: {high['kd_v']:.12g}
"""
    with open(OUTPUT_GAINS_YAML, "w") as handle:
        handle.write(content)


def main():
    os.makedirs(DATA_LOG_DIR, exist_ok=True)
    records = []
    print("Three-loop torque PID tuning: A(rate) -> B(attitude) -> C(velocity), k_x=0")
    low = tune_endpoint(0.30, "low", records)
    high = tune_endpoint(0.50, "high", records)
    interpolation = validate_interpolation(low, high, records)
    write_yaml(low, high)  # only reachable after every stage has a valid result
    summary = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "architecture": "three_loop_velocity_attitude_rate_torque",
        "position_feedback": {"k_x": 0.0, "origin": "fixed_p0_observation_only"},
        "limits": {"rate_limit_u": 0.0, "total_torque_max_nm": 20.0, "wheel_torque_max_nm": 10.0},
        "low_height": low, "high_height": high, "interpolation_h040": interpolation,
        "records": records, "effective_yaml": OUTPUT_GAINS_YAML,
    }
    with open(SUMMARY_PATH, "w") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Tuning complete; measured YAML written to {OUTPUT_GAINS_YAML}")
    print(f"Candidate and validation summary written to {SUMMARY_PATH}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"TUNING FAILED: {exc}", file=sys.stderr)
        print("No new YAML was generated.", file=sys.stderr)
        sys.exit(1)
