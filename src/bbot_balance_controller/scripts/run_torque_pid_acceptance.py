#!/usr/bin/env python3
"""
run_torque_pid_acceptance.py
Comprehensive 6-Trial Acceptance Test Suite for BBOT Torque Cascade PID Controller.
Baseline: Classic Three-Loop Cascade (Velocity -> Attitude -> Pitch Rate), k_x = 0 strictly.
Position is strictly an observation variable relative to fixed origin p_0.

Trials:
  1. H = 0.30 m Static Balance (25s simulated time)
  2. H = 0.40 m Static Balance (25s simulated time)
  3. H = 0.50 m Static Balance (25s simulated time)
  4. Continuous Height Sweep 0.30m -> 0.50m -> 0.30m (38s simulated time)
  5. +20 N x 0.20s Push Disturbance at H = 0.30 m (stabilized >=12s in sim before push, 45s total)
  6. -20 N x 0.20s Push Disturbance at H = 0.30 m (stabilized >=12s in sim before push, 45s total)
"""

import os
import sys
import time
import signal
import subprocess
import csv
import json
import numpy as np

WS_ROOT = "/home/admin/bbot_ws_new"
TRIALS_DIR = os.path.join(WS_ROOT, "src/bbot_balance_controller/src/data_logs/torque_pid_trials")
SUMMARY_PATH = os.path.join(TRIALS_DIR, "torque_pid_acceptance_summary.json")
PUSH_STEADY_HOLD_TIME = 5.0
PUSH_STEADY_PITCH_LIMIT_DEG = 0.20
PUSH_STEADY_RATE_LIMIT_RAD_S = 0.02
PUSH_STEADY_VELOCITY_LIMIT_MPS = 0.01
PULSE_DURATION_TOLERANCE = (0.19, 0.21)
EXPECTED_FORCE_TO_VELOCITY_SIGN = 1.0
HEIGHT_SWEEP_INITIAL_HOLD_TIME = 12.0
PITCH_SAFETY_LIMIT_DEG = 15.0
VELOCITY_RMS_LIMIT_MPS = 0.015
SATURATION_RATIO_LIMIT = 0.05
CHATTER_ENERGY_LIMIT = 0.20

os.environ["ROS_HOME"] = os.path.join(WS_ROOT, ".ros")
os.environ["ROS_LOG_DIR"] = os.path.join(WS_ROOT, ".ros/log")
os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)


def cleanup():
    """Cleanly terminate any background simulator processes."""
    for proc_name in [
        "ign gazebo", "gz sim", "ruby", "ros_gz_bridge", "parameter_bridge",
        "torque_cascade_pid_controller", "spawner", "robot_state_publisher",
        "ros2 launch", "bbot_acceptance_force_pulse"
    ]:
        subprocess.run(["pkill", "-9", "-f", proc_name], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def compute_6_12hz_energy(signal, dt=0.005):
    """Compute normalized relative spectral energy in 6-12 Hz band."""
    if len(signal) < 64:
        return 0.0
    sig = signal - np.mean(signal)
    fft_vals = np.abs(np.fft.rfft(sig))
    freqs = np.fft.rfftfreq(len(sig), d=dt)
    band_mask = (freqs >= 6.0) & (freqs <= 12.0)
    if not np.any(band_mask):
        return 0.0
    band_energy = np.sum(fft_vals[band_mask] ** 2)
    total_energy = np.sum(fft_vals ** 2) + 1e-9
    return float(band_energy / total_energy)


def continuous_recovery_time(times, pitch_err_deg, velocity, start_time,
                             pitch_limit=0.5, velocity_limit=0.02, hold_time=2.0):
    mask = times >= start_time
    t = times[mask]
    in_band = ((np.abs(pitch_err_deg[mask]) <= pitch_limit) &
               (np.abs(velocity[mask]) <= velocity_limit))
    if len(t) < 2:
        return None
    dt = float(np.median(np.diff(t))) if len(t) > 2 else 0.005
    window = max(1, int(np.ceil(hold_time / max(dt, 1e-6))))
    for idx in range(0, len(in_band) - window + 1):
        if np.all(in_band[idx:idx + window]):
            return float(t[idx] - start_time)
    return None


def send_target_height(h):
    subprocess.run(
        [
            "bash", "-c",
            f"source {WS_ROOT}/setup_env.sh && ros2 topic pub --once /target_height std_msgs/msg/Float64 '{{data: {h:.3f}}}'"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def parse_last_torque_csv_line(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        # The controller log has gained columns over time.  Always resolve
        # fields by their header name so a new diagnostic column cannot shift
        # the push trigger or response analysis onto the wrong signal.
        last = None
        with open(path, "r", newline="") as f:
            for row in csv.DictReader(f):
                if row.get("time") not in (None, ""):
                    last = row
        if last is None:
            return None

        def number(name, default=0.0):
            value = last.get(name, "")
            return default if value in (None, "") else float(value)

        required = ("time", "p", "p_0", "delta_p", "v", "pitch",
                    "pitch_rate", "theta_eq", "u_clamped", "is_saturated",
                    "force_active", "force_value", "p_0_latched")
        if any(name not in last for name in required):
            return None
        return {
            "time": number("time"),
            "height": number("height"),
            "p": number("p"),
            "p_0": number("p_0"),
            "delta_p": number("delta_p"),
            "v": number("v"),
            "pitch": number("pitch"),
            "pitch_rate": number("pitch_rate"),
            "theta_eq": number("theta_eq"),
            "u_clamped": number("u_clamped"),
            "is_saturated": int(number("is_saturated")),
            "force_active": int(number("force_active")),
            "force_value": number("force_value"),
            "p_0_latched": int(number("p_0_latched")),
        }
    except Exception:
        return None


def read_log_rows(path):
    time.sleep(0.5)
    if not os.path.exists(path) or os.path.getsize(path) < 100:
        return []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        fl = len(reader.fieldnames)
        return [r for r in reader if len(r) == fl and r.get("pitch") is not None and r.get("pitch") != ""]


def load_pulse_state(path):
    try:
        with open(path, "r") as f:
            state = json.load(f)
        return state if isinstance(state, dict) else None
    except (OSError, ValueError, TypeError):
        return None


def pulse_state_valid(state):
    if not state or not state.get("bridge_ready"):
        return False
    if int(state.get("wrench_publish_count", 0)) != 1:
        return False
    actual = state.get("actual_duration")
    if actual is None or not (PULSE_DURATION_TOLERANCE[0] <= float(actual) <= PULSE_DURATION_TOLERANCE[1]):
        return False
    return bool(state.get("completed")) and bool(state.get("clear_complete"))


# ---------------------------------------------------------------------------
# Static Balance Trial Runner (Trials 1 - 3)
# ---------------------------------------------------------------------------
def run_static_test(trial_id, target_height, startup_height, log_filename, sim_duration=25.0):
    print("\n=======================================================")
    print(f"  [Trial {trial_id}] H = {target_height:.2f} m Static Balance ({sim_duration:.0f}s sim)")
    print("  Pass criteria: steady pitch RMS <= 0.05 deg, low velocity RMS, no fall, no sustained saturation or 6-12 Hz chatter")
    print("  Position: Observation only (relative to fixed p_0)")
    print("=======================================================")
    cleanup()
    log_path = os.path.join(TRIALS_DIR, log_filename)
    if os.path.exists(log_path):
        os.remove(log_path)

    cmd = (
        f"source {WS_ROOT}/setup_env.sh && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"headless:=true "
        f"controller_type:=torque_cascade_pid "
        f"world:=balance_test_world.sdf "
        f"torque_pid_log_path:={log_path} "
        f"torque_pid_k_x:=0.0 "
        f"torque_pid_leg_transition_speed:=0.05 "
        f"torque_pid_startup_height:={startup_height:.2f} "
        f"torque_pid_target_height:={target_height:.2f}"
    )

    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0_real = time.time()
    last_sim_time = -1.0
    last_progress_real = time.time()

    try:
        while True:
            if proc.poll() is not None:
                print("    [Warning] Launch process terminated early")
                break
            rec = parse_last_torque_csv_line(log_path)
            if rec is None:
                if time.time() - t0_real > 50.0:
                    print("    [Error] Startup timeout: No CSV log produced in 50s")
                    break
                time.sleep(0.3)
                continue

            t_sim = rec["time"]
            if t_sim > last_sim_time + 0.05:
                last_sim_time = t_sim
                last_progress_real = time.time()
            elif time.time() - last_progress_real > 30.0:
                print("    [Error] Simulation stalled for 30s")
                break

            if t_sim >= sim_duration:
                print(f"    --> Target simulated duration reached (t_sim = {t_sim:.2f} s >= {sim_duration:.1f} s)")
                break
            time.sleep(0.3)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(12):
                if proc.poll() is not None:
                    break
                time.sleep(0.3)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        cleanup()

    rows = read_log_rows(log_path)
    if len(rows) < 150:
        return False, {"reason": "insufficient_data", "samples": len(rows)}

    times = np.array([float(r["time"]) for r in rows])
    pitches = np.array([float(r["pitch"]) for r in rows])
    theta_eqs = np.array([float(r["theta_eq"]) for r in rows])
    vels = np.array([float(r["v"]) for r in rows])
    positions = np.array([float(r["p"]) for r in rows])
    p0s = np.array([float(r["p_0"]) for r in rows])
    torques = np.array([float(r["u_clamped"]) for r in rows])
    sats = np.array([int(r["is_saturated"]) for r in rows])

    # Check fall
    max_pitch_err_all = float(np.max(np.abs(pitches - theta_eqs) * 57.2957795))
    if max_pitch_err_all > 25.0 or len(rows) < 200:
        return False, {"reason": "fell_or_diverged", "max_pitch_err_deg": max_pitch_err_all}

    # Settled evaluation window (last 8.0s)
    mask = times >= (times[-1] - 8.0)
    pitch_err_deg = (pitches[mask] - theta_eqs[mask]) * 57.2957795
    pitch_rms = float(np.sqrt(np.mean(pitch_err_deg ** 2)))
    pitch_rates = np.array([float(r["pitch_rate"]) for r in rows])[mask]
    pitch_rate_rms = float(np.sqrt(np.mean(pitch_rates ** 2)))
    v_rms = float(np.sqrt(np.mean(vels[mask] ** 2)))
    sat_ratio = float(np.mean(sats[mask]))
    torque_peak = float(np.max(np.abs(torques[mask])))
    torque_rms = float(np.sqrt(np.mean(torques[mask] ** 2)))
    chatter_u = compute_6_12hz_energy(torques[mask])
    chatter_pitch = compute_6_12hz_energy(pitch_err_deg)

    # Position drift relative to fixed p_0
    deltas_p_mm = (positions - p0s) * 1000.0
    final_drift_mm = float(deltas_p_mm[-1])
    max_abs_drift_mm = float(np.max(np.abs(deltas_p_mm)))
    drift_velocity_mms = float((deltas_p_mm[-1] - deltas_p_mm[mask][0]) / (times[-1] - times[mask][0]))

    passed = (pitch_rms <= 0.05 and v_rms <= VELOCITY_RMS_LIMIT_MPS and
              max_pitch_err_all <= PITCH_SAFETY_LIMIT_DEG and
              sat_ratio <= SATURATION_RATIO_LIMIT and
              chatter_u <= CHATTER_ENERGY_LIMIT and chatter_pitch <= CHATTER_ENERGY_LIMIT)
    metrics = {
        "pitch_rms_deg": pitch_rms,
        "pitch_rate_rms_rad_s": pitch_rate_rms,
        "v_rms_mps": v_rms,
        "max_pitch_error_deg": max_pitch_err_all,
        "torque_peak_nm": torque_peak,
        "torque_rms_nm": torque_rms,
        "sat_ratio": sat_ratio,
        "chatter_6_12hz": chatter_u,
        "pitch_6_12hz": chatter_pitch,
        "final_drift_mm": final_drift_mm,
        "max_abs_drift_mm": max_abs_drift_mm,
        "drift_velocity_mms": drift_velocity_mms,
        "samples": len(rows),
    }

    print(f"  Result: Passed={passed} | Pitch RMS={pitch_rms:.4f} deg (<=0.05), v RMS={v_rms:.4f} m/s (<=0.015), sat={sat_ratio:.3f} (<=0.05)")
    print(f"    Observed Drift relative to p_0: final={final_drift_mm:+.2f} mm, max={max_abs_drift_mm:.2f} mm, drift_vel={drift_velocity_mms:+.2f} mm/s")
    return passed, metrics


# ---------------------------------------------------------------------------
# Trial 4: Continuous Height Sweep (0.30m -> 0.50m -> 0.30m)
# ---------------------------------------------------------------------------
def run_test_sweep(sim_duration=38.0, log_filename="trial4_sweep_h030_h050_h030.csv"):
    print("\n=======================================================")
    print(f"  [Trial 4] Continuous Height Sweep: 0.30m -> 0.50m -> 0.30m ({sim_duration:.0f}s sim)")
    print("  Pass criteria: initial low hold >=12s, safe peak pitch, post-sweep pitch/velocity recovery, no sustained chatter")
    print("=======================================================")
    cleanup()
    log_path = os.path.join(TRIALS_DIR, log_filename)
    if os.path.exists(log_path):
        os.remove(log_path)

    cmd = (
        f"source {WS_ROOT}/setup_env.sh && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"headless:=true "
        f"controller_type:=torque_cascade_pid "
        f"world:=balance_test_world.sdf "
        f"torque_pid_log_path:={log_path} "
        f"torque_pid_k_x:=0.0 "
        f"torque_pid_leg_transition_speed:=0.05 "
        f"torque_pid_startup_height:=0.36 "
        f"torque_pid_target_height:=0.30"
    )

    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    t0_real = time.time()
    last_sim_time = -1.0
    last_progress_real = time.time()
    raised = False
    lowered = False

    try:
        while True:
            if proc.poll() is not None:
                print("    [Warning] Launch process terminated early")
                break
            rec = parse_last_torque_csv_line(log_path)
            if rec is None:
                if time.time() - t0_real > 50.0:
                    print("    [Error] Startup timeout: No CSV log produced in 50s")
                    break
                time.sleep(0.3)
                continue

            t_sim = rec["time"]
            if t_sim > last_sim_time + 0.05:
                last_sim_time = t_sim
                last_progress_real = time.time()
            elif time.time() - last_progress_real > 30.0:
                print("    [Error] Simulation stalled for 30s")
                break

            # Settle at low height before commanding the first raise.
            if t_sim >= HEIGHT_SWEEP_INITIAL_HOLD_TIME and not raised:
                print(f"    --> Commanding target height to 0.50 m at t_sim = {t_sim:.2f} s...")
                send_target_height(0.50)
                raised = True
            # Hold at high height, then lower back to 0.30m at 24s sim time
            elif t_sim >= 24.0 and not lowered:
                print(f"    --> Commanding target height back to 0.30 m at t_sim = {t_sim:.2f} s...")
                send_target_height(0.30)
                lowered = True

            if t_sim >= sim_duration:
                print(f"    --> Target simulated duration reached (t_sim = {t_sim:.2f} s >= {sim_duration:.1f} s)")
                break
            time.sleep(0.3)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(12):
                if proc.poll() is not None:
                    break
                time.sleep(0.3)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        cleanup()

    rows = read_log_rows(log_path)
    if len(rows) < 250:
        return False, {"reason": "insufficient_data", "samples": len(rows)}

    times = np.array([float(r["time"]) for r in rows])
    pitches = np.array([float(r["pitch"]) for r in rows])
    theta_eqs = np.array([float(r["theta_eq"]) for r in rows])
    heights = np.array([float(r["height"]) for r in rows])
    vels = np.array([float(r["v"]) for r in rows])
    positions = np.array([float(r["p"]) for r in rows])
    p0s = np.array([float(r["p_0"]) for r in rows])

    pitch_errs_deg = (pitches - theta_eqs) * 57.2957795
    max_pitch_err = float(np.max(np.abs(pitch_errs_deg)))
    deltas_p_mm = (positions - p0s) * 1000.0

    mask_end = times >= (times[-1] - 4.0)
    end_pitch_err = float(np.max(np.abs(pitch_errs_deg[mask_end])))
    end_v_rms = float(np.sqrt(np.mean(vels[mask_end] ** 2)))
    end_pitch_rms = float(np.sqrt(np.mean(pitch_errs_deg[mask_end] ** 2)))
    pitch_rate = np.array([float(r["pitch_rate"]) for r in rows])
    torque = np.array([float(r["u_clamped"]) for r in rows])
    sat_ratio = float(np.mean(np.array([int(r["is_saturated"]) for r in rows])[mask_end]))
    chatter_pitch = compute_6_12hz_energy(pitch_errs_deg[mask_end])
    chatter_u = compute_6_12hz_energy(torque[mask_end])
    recovery_time = continuous_recovery_time(times, pitch_errs_deg, vels, times[-1] - 8.0)
    final_drift_mm = float(deltas_p_mm[-1])
    max_abs_drift_mm = float(np.max(np.abs(deltas_p_mm)))

    passed = (max_pitch_err <= PITCH_SAFETY_LIMIT_DEG and end_pitch_rms <= 0.50 and
              end_v_rms <= 0.02 and sat_ratio <= SATURATION_RATIO_LIMIT and
              chatter_pitch <= CHATTER_ENERGY_LIMIT and chatter_u <= CHATTER_ENERGY_LIMIT)
    metrics = {
        "max_pitch_err_deg": max_pitch_err,
        "end_pitch_err_deg": end_pitch_err,
        "end_pitch_rms_deg": end_pitch_rms,
        "end_v_rms_mps": end_v_rms,
        "end_pitch_rate_rms_rad_s": float(np.sqrt(np.mean(pitch_rate[mask_end] ** 2))),
        "torque_peak_nm": float(np.max(np.abs(torque))),
        "torque_rms_nm": float(np.sqrt(np.mean(torque ** 2))),
        "sat_ratio": sat_ratio,
        "pitch_6_12hz": chatter_pitch,
        "torque_6_12hz": chatter_u,
        "recovery_time_s": recovery_time,
        "height_min": float(np.min(heights)),
        "height_max": float(np.max(heights)),
        "final_drift_mm": final_drift_mm,
        "max_abs_drift_mm": max_abs_drift_mm,
        "samples": len(rows),
    }

    print(f"  Result: Passed={passed} | max_pitch={max_pitch_err:.2f} deg, end_pitch_rms={end_pitch_rms:.3f} deg, end_v={end_v_rms:.4f} m/s, sat={sat_ratio:.3f}")
    print(f"    Observed Drift relative to p_0: final={final_drift_mm:+.2f} mm, max={max_abs_drift_mm:.2f} mm")
    return passed, metrics


# ---------------------------------------------------------------------------
# Push Disturbance Trials (Trials 5 & 6)
# ---------------------------------------------------------------------------
PULSE_HELPER_CODE = r'''
import json, sys, time, os
import rclpy
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import EntityWrench, Entity
from std_msgs.msg import Float64

def main():
    topic_persistent, topic_instant, topic_clear, link_name, axis, force_str, duration_str, trigger_file, status_file = sys.argv[1:10]
    force_val = float(force_str)
    duration_val = float(duration_str)

    state = {
        "bridge_ready": False,
        "wrench_publish_count": 0,
        "clear_publish_count": 0,
        "pulse_start_sim_time": None,
        "pulse_end_sim_time": None,
        "actual_duration": None,
        "completed": False,
        "clear_complete": False,
        "invalid_reason": None,
    }

    def write_state():
        tmp = status_file + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(state, handle, sort_keys=True)
        os.replace(tmp, status_file)

    def fail(reason):
        state["invalid_reason"] = reason
        write_state()

    rclpy.init()
    node = rclpy.create_node("bbot_acceptance_force_pulse")
    pub_persistent = node.create_publisher(EntityWrench, topic_persistent, 10)
    pub_clear = node.create_publisher(Entity, topic_clear, 10)
    pub_dist_force = node.create_publisher(Float64, "/disturbance_force_y", 10)
    sim_time = [None]
    node.create_subscription(Clock, "/clock", lambda msg: sim_time.__setitem__(0, msg.clock.sec + msg.clock.nanosec * 1e-9), 10)

    # Only persistent and clear endpoints are part of this protocol.  The
    # instantaneous wrench topic is deliberately not used for a second wrench
    # command, because that would make the pulse un-auditable.
    waited = 0.0
    while (pub_persistent.get_subscription_count() == 0 or
           pub_clear.get_subscription_count() == 0) and waited < 60.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
        waited += 0.05
    if (pub_persistent.get_subscription_count() == 0 or
        pub_clear.get_subscription_count() == 0):
        fail("BRIDGE_NOT_CONNECTED")
        node.destroy_node()
        rclpy.shutdown()
        return

    state["bridge_ready"] = True
    clear_msg = Entity()
    clear_msg.name = link_name
    clear_msg.type = Entity.LINK
    # Clear once before arming the trigger, and repeat only the clear command
    # for transport reliability.  No wrench message is sent here.
    for _ in range(5):
        pub_clear.publish(clear_msg)
        state["clear_publish_count"] += 1
        rclpy.spin_once(node, timeout_sec=0.01)
    state["pre_clear_complete"] = True
    write_state()

    # The parent writes the trigger using simulation time.  Do not use wall
    # time for the pulse duration.
    while not os.path.exists(trigger_file):
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.005)
    with open(trigger_file) as handle:
        trigger_sim_time = float(handle.read().strip())
    while sim_time[0] is None or sim_time[0] < trigger_sim_time:
        rclpy.spin_once(node, timeout_sec=0.01)

    msg = EntityWrench()
    msg.entity.name = link_name
    msg.entity.type = Entity.LINK
    setattr(msg.wrench.force, axis, force_val)

    force_msg = Float64()
    force_msg.data = force_val

    force_start = sim_time[0]
    # Exactly one persistent wrench command per trial.  The marker is the
    # only periodic publication during the 0.20 s simulated pulse.
    pub_persistent.publish(msg)
    state["wrench_publish_count"] += 1
    state["pulse_start_sim_time"] = force_start
    write_state()
    while sim_time[0] is None or sim_time[0] < force_start + duration_val:
        pub_dist_force.publish(force_msg)
        rclpy.spin_once(node, timeout_sec=0.005)

    state["pulse_end_sim_time"] = sim_time[0]
    state["actual_duration"] = state["pulse_end_sim_time"] - force_start
    # Clear force after the simulation-clock duration.  Do not publish another
    # wrench; the clear topic is the only command that ends the persistent one.
    zero_force = Float64()
    zero_force.data = 0.0
    for _ in range(5):
        pub_clear.publish(clear_msg)
        state["clear_publish_count"] += 1
        rclpy.spin_once(node, timeout_sec=0.01)
    state["clear_complete"] = True
    pub_dist_force.publish(zero_force)
    state["completed"] = (state["wrench_publish_count"] == 1 and
                          0.19 <= state["actual_duration"] <= 0.21 and
                          state["clear_complete"])
    if not state["completed"]:
        state["invalid_reason"] = "invalid_pulse_protocol"
    write_state()
    time.sleep(0.10)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''


def run_push_test(force_value, trial_id, log_filename, sim_duration=50.0,
                  target_height=0.30, startup_height=0.30, late_bridge=False):
    print("\n=======================================================")
    print(f"  [Trial {trial_id}] H = {target_height:.2f} m, {force_value:+.1f} N Push Disturbance (0.20s, {sim_duration:.0f}s sim)")
    print("  Procedure: Hold the verified steady-state gate for 5s in sim -> fire force -> verify mapped response -> recover")
    print("  Pass criteria: Pitch error <= 0.5 deg for >=2s, v <= 0.02 m/s for >=2s within 35s post-push")
    print("=======================================================")
    cleanup()
    log_path = os.path.join(TRIALS_DIR, log_filename)
    if os.path.exists(log_path):
        os.remove(log_path)

    trigger_file = f"/tmp/bbot_push_trig_{trial_id}_{int(time.time())}"
    status_file = f"/tmp/bbot_push_status_{trial_id}_{int(time.time())}"
    if os.path.exists(trigger_file):
        os.remove(trigger_file)
    if os.path.exists(status_file):
        os.remove(status_file)

    env = os.environ.copy()

    # Publication runs may defer the force pipeline until balance is established:
    # on this machine, an early bridge can starve PID startup at non-nominal heights.
    bridge_log = open(f"/tmp/bridge_push_{trial_id}.log", "w")
    pulse_log = open(f"/tmp/pulse_push_{trial_id}.log", "w")
    bridge_proc = None
    pulse_helper = None

    def start_force_pipeline():
        bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "/world/balance_test_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
             "/world/balance_test_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity"],
            stdout=bridge_log, stderr=subprocess.STDOUT, env=env)
        helper = subprocess.Popen(
            [sys.executable, "-u", "-c", PULSE_HELPER_CODE,
             "/world/balance_test_world/wrench/persistent",
             "/world/balance_test_world/wrench",
             "/world/balance_test_world/wrench/clear",
             "base_link", "y", str(force_value), "0.20",
             trigger_file, status_file],
            stdout=pulse_log, stderr=subprocess.STDOUT, env=env)
        return bridge, helper

    if not late_bridge:
        bridge_proc, pulse_helper = start_force_pipeline()

    cmd = (
        f"source {WS_ROOT}/setup_env.sh && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"headless:=true "
        f"controller_type:=torque_cascade_pid "
        f"world:=balance_test_world.sdf "
        f"torque_pid_log_path:={log_path} "
        f"torque_pid_k_x:=0.0 "
        f"torque_pid_leg_transition_speed:=0.05 "
        f"torque_pid_startup_height:={startup_height:.2f} "
        f"torque_pid_target_height:={target_height:.2f}"
    )

    proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    t0_real = time.time()
    last_sim_time = -1.0
    last_progress_real = time.time()
    fired = False
    fire_sim_time = None
    bridge_ready = False
    steady_since = None
    pulse_state = None

    try:
        while True:
            if proc.poll() is not None:
                print("    [Warning] Launch process terminated early")
                break
            rec = parse_last_torque_csv_line(log_path)
            if rec is None:
                if time.time() - t0_real > 90.0:
                    print("    [Error] Startup timeout: No CSV log produced in 90s")
                    break
                time.sleep(0.3)
                continue

            t_sim = rec["time"]
            if t_sim > last_sim_time + 0.05:
                last_sim_time = t_sim
                last_progress_real = time.time()
            elif time.time() - last_progress_real > 30.0:
                print("    [Error] Simulation stalled for 30s")
                break

            if late_bridge and bridge_proc is None and t_sim >= 8.0:
                bridge_proc, pulse_helper = start_force_pipeline()
                last_progress_real = time.time()
                print("    --> Force pipeline launched after PID balance startup")

            if os.path.exists(status_file) and not bridge_ready:
                state = load_pulse_state(status_file)
                if state is None:
                    time.sleep(0.05)
                    continue
                if state.get("invalid_reason"):
                    print(f"    [Error] Force bridge validation failed: {state['invalid_reason']}")
                    break
                if state.get("bridge_ready") and state.get("pre_clear_complete"):
                    bridge_ready = True
                    print("    --> Force bridge verified; pre-trial clear completed")

            # A trigger is valid only after the controller has latched its
            # fixed p_0 origin and all balance conditions have held in
            # simulation time for five continuous seconds.
            stable = (rec["p_0_latched"] == 1 and
                      abs((rec["pitch"] - rec["theta_eq"]) * 57.2957795) <= PUSH_STEADY_PITCH_LIMIT_DEG and
                      abs(rec["pitch_rate"]) <= PUSH_STEADY_RATE_LIMIT_RAD_S and
                      abs(rec["v"]) <= PUSH_STEADY_VELOCITY_LIMIT_MPS)
            if stable:
                if steady_since is None:
                    steady_since = t_sim
            else:
                steady_since = None

            # Trigger is written in simulation time; the helper applies one
            # persistent wrench for 0.20 simulated seconds.
            if (bridge_ready and not fired and steady_since is not None and
                    t_sim - steady_since >= PUSH_STEADY_HOLD_TIME):
                print(f"    --> Firing {force_value:+.1f} N pulse after 5 s verified steady balance (t_sim = {t_sim:.2f} s)...")
                with open(trigger_file, "w") as f:
                    f.write(f"{t_sim:.9f}")
                fired = True
                fire_sim_time = t_sim

            if t_sim >= sim_duration:
                print(f"    --> Target simulated duration reached (t_sim = {t_sim:.2f} s >= {sim_duration:.1f} s)")
                break
            time.sleep(0.3)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(12):
                if proc.poll() is not None:
                    break
                time.sleep(0.3)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        if bridge_proc:
            try: bridge_proc.terminate()
            except Exception: pass
        if pulse_helper:
            try: pulse_helper.terminate()
            except Exception: pass
        pulse_state = load_pulse_state(status_file)
        if os.path.exists(trigger_file):
            os.remove(trigger_file)
        try: bridge_log.close()
        except Exception: pass
        try: pulse_log.close()
        except Exception: pass
        cleanup()

    rows = read_log_rows(log_path)
    if pulse_state is not None:
        with open(log_path + ".pulse.json", "w") as state_file:
            json.dump(pulse_state, state_file, indent=2, sort_keys=True)
    if not bridge_ready:
        return False, {"reason": "force_bridge_not_connected", "samples": len(rows)}
    if not fired:
        return False, {"reason": "force_trigger_not_fired", "samples": len(rows)}
    if not pulse_state_valid(pulse_state):
        return False, {"reason": "invalid_pulse_protocol", "pulse_state": pulse_state or {}}
    if len(rows) < 250:
        return False, {"reason": "insufficient_data", "samples": len(rows)}

    times = np.array([float(r["time"]) for r in rows])
    pitches = np.array([float(r["pitch"]) for r in rows])
    theta_eqs = np.array([float(r["theta_eq"]) for r in rows])
    vels = np.array([float(r["v"]) for r in rows])
    positions = np.array([float(r["p"]) for r in rows])
    p0s = np.array([float(r["p_0"]) for r in rows])
    forces_act = np.array([int(r["force_active"]) for r in rows])
    forces_val = np.array([float(r["force_value"]) for r in rows])

    # The log must contain the real bridge/controller force marker.  A missing
    # marker makes the trial invalid; falling back to the trigger time would
    # hide a disconnected bridge.
    force_idx = np.where(forces_act == 1)[0]
    if len(force_idx) == 0 or np.max(np.abs(forces_val[force_idx])) < 19.5:
        return False, {"reason": "no_actual_force_response", "force_active_samples": int(len(force_idx)), "force_max": float(np.max(np.abs(forces_val))) if len(forces_val) else 0.0}
    if np.any(np.sign(forces_val[force_idx]) != np.sign(force_value)):
        return False, {"reason": "force_marker_sign_mismatch"}
    t_pulse_start = float(pulse_state["pulse_start_sim_time"])
    t_pulse_end = float(pulse_state["pulse_end_sim_time"])
    if not (t_pulse_start <= t_pulse_end and
            abs((t_pulse_end - t_pulse_start) - float(pulse_state["actual_duration"])) < 1e-6):
        return False, {"reason": "invalid_pulse_timestamps", "pulse_state": pulse_state}

    # The marker must overlap the actual simulated pulse interval, and there
    # must be a post-clear log sample with the marker inactive.
    marker_start = times[force_idx[0]]
    marker_end = times[force_idx[-1]]
    if marker_end < t_pulse_start or marker_start > t_pulse_end:
        return False, {"reason": "force_marker_not_overlapping_pulse"}
    post_clear = (times >= t_pulse_end + 0.05) & (forces_act == 0) & (np.abs(forces_val) < 1e-6)
    if not np.any(post_clear):
        return False, {"reason": "force_marker_not_cleared"}

    # Verify physical response occurred: check excursion in window [t_pulse_start, t_pulse_start + 1.5]
    mask_resp = (times >= t_pulse_start) & (times <= t_pulse_start + 1.5)
    if not np.any(mask_resp):
        return False, {"reason": "response_window_missing"}

    peak_vel_resp = float(np.max(np.abs(vels[mask_resp])))
    pre_mask = (times >= t_pulse_start - 0.2) & (times < t_pulse_start)
    pre_v = float(np.mean(vels[pre_mask])) if np.any(pre_mask) else 0.0
    response_mask = (times >= t_pulse_start + 0.10) & (times <= t_pulse_start + 0.80)
    response_delta_v = float(np.mean(vels[response_mask]) - pre_v) if np.any(response_mask) else 0.0
    initial_pitch_resp = float(np.mean(pitches[mask_resp] - theta_eqs[mask_resp]) * 57.2957795)
    expected_direction = float(np.sign(force_value) * EXPECTED_FORCE_TO_VELOCITY_SIGN)
    observed_direction = float(np.sign(response_delta_v))

    # Physical check: disturbance must generate perceptible velocity reaction >= 0.05 m/s
    if (peak_vel_resp < 0.05 or abs(response_delta_v) < 0.02 or
            observed_direction != expected_direction):
        return False, {"reason": "no_physical_response_detected", "peak_vel": peak_vel_resp,
                       "response_delta_v_mps": response_delta_v,
                       "expected_response_direction": expected_direction,
                       "observed_response_direction": observed_direction}

    # Recovery check: find time where pitch error <= 0.5 deg and |v| <= 0.02 m/s continuously for >= 2.0s
    post_mask = times >= t_pulse_end
    t_post = times[post_mask]
    pitch_err_post = np.abs(pitches[post_mask] - theta_eqs[post_mask]) * 57.2957795
    v_post = np.abs(vels[post_mask])

    in_band = (pitch_err_post <= 0.50) & (v_post <= 0.02)
    recovery_time = None
    recovery_time = continuous_recovery_time(times, pitches * 57.2957795 - theta_eqs * 57.2957795,
                                              vels, t_pulse_end)

    recovered = (recovery_time is not None) and (recovery_time <= 35.0)

    # Pre-push and post-push drift relative to p_0
    drift_pre_mask = (times >= t_pulse_start - 2.0) & (times < t_pulse_start)
    drift_pre_mm = float((positions[drift_pre_mask][-1] - p0s[drift_pre_mask][-1]) * 1000.0) if np.any(drift_pre_mask) else 0.0
    drift_post_mm = float((positions[-1] - p0s[-1]) * 1000.0)
    push_net_displacement_mm = drift_post_mm - drift_pre_mm
    post_drift_velocity_mms = float((positions[-1] - p0s[-1] -
                                     (positions[drift_pre_mask][-1] - p0s[drift_pre_mask][-1])) * 1000.0 /
                                    max(times[-1] - t_pulse_start, 1e-6)) if np.any(drift_pre_mask) else 0.0
    torque = np.array([float(r["u_clamped"]) for r in rows])
    saturation = np.array([int(r["is_saturated"]) for r in rows])
    pitch_err_deg_all = (pitches - theta_eqs) * 57.2957795

    metrics = {
        "recovered": recovered,
        "recovery_time_s": recovery_time if recovery_time is not None else 999.0,
        "peak_vel_resp_mps": peak_vel_resp,
        "response_delta_v_mps": response_delta_v,
        "response_direction": observed_direction,
        "expected_response_direction": expected_direction,
        "response_mapping_valid": observed_direction == expected_direction,
        "initial_pitch_resp_deg": initial_pitch_resp,
        "peak_pitch_resp_deg": float(np.max(np.abs(pitch_err_deg_all[mask_resp]))),
        "torque_peak_nm": float(np.max(np.abs(torque[mask_resp]))),
        "torque_rms_nm": float(np.sqrt(np.mean(torque[mask_resp] ** 2))),
        "saturation_ratio": float(np.mean(saturation[mask_resp])),
        "drift_pre_push_mm": drift_pre_mm,
        "drift_post_push_mm": drift_post_mm,
        "push_net_displacement_mm": push_net_displacement_mm,
        "final_drift_mm": float((positions[-1] - p0s[-1]) * 1000.0),
        "max_abs_drift_mm": float(np.max(np.abs((positions - p0s) * 1000.0))),
        "post_push_drift_velocity_mms": post_drift_velocity_mms,
        "force_recorded_max": float(np.max(np.abs(forces_val))),
        "force_active_samples": int(len(force_idx)),
        "pulse_state": pulse_state,
        "samples": len(rows),
    }

    print(f"  Result: Recovered={recovered} | Recovery Time={metrics['recovery_time_s']:.2f} s (<=35s), Peak Vel={peak_vel_resp:.3f} m/s, Peak Pitch={metrics['peak_pitch_resp_deg']:.2f} deg")
    print(f"    Torque: peak={metrics['torque_peak_nm']:.2f} Nm, RMS={metrics['torque_rms_nm']:.2f} Nm, saturation={metrics['saturation_ratio']:.3f}")
    print(f"    Displacement: pre={drift_pre_mm:+.2f} mm, post={drift_post_mm:+.2f} mm, push_net_delta={push_net_displacement_mm:+.2f} mm")
    return recovered, metrics


def summarize_push_repeats(results, force_value):
    """Return mean/std for numeric push metrics without hiding failed runs."""
    entries = [entry for key, entry in results.items()
               if entry.get("metrics", {}).get("force_value") == force_value]
    selected = [entry["metrics"] for entry in entries]
    if not entries:
        return {"n": 0}
    summary = {"n": len(selected),
               "n_pass": sum(1 for entry in entries if entry.get("ok", False))}
    keys = set().union(*(m.keys() for m in selected))
    for key in sorted(keys):
        vals = [m[key] for m in selected
                if isinstance(m.get(key), (int, float)) and
                not isinstance(m.get(key), bool) and np.isfinite(m[key])]
        if vals:
            summary[key + "_mean"] = float(np.mean(vals))
            summary[key + "_std"] = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
    return summary


# ---------------------------------------------------------------------------
# Main Execution Suite
# ---------------------------------------------------------------------------
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6],
                        help="Trials to run, e.g. --trials 5 6")
    parser.add_argument("--repeats", type=int, default=1,
                        help="Repeats for push trials; use 3 for the formal matrix")
    parser.add_argument("--smoke", action="store_true",
                        help="Run only H=0.30 m +20/-20 N once each")
    args = parser.parse_args()

    if args.repeats < 1:
        parser.error("--repeats must be >= 1")
    if args.smoke:
        args.trials = [5, 6]
        args.repeats = 1

    print("=================================================================")
    print("  BBOT Torque Cascade PID Controller - Acceptance Suite")
    print("  Architecture: Three-Loop Baseline (Velocity -> Attitude -> Rate)")
    print("  Position: Observation only relative to fixed origin p_0         ")
    print("=================================================================")

    os.makedirs(TRIALS_DIR, exist_ok=True)
    results = {}

    # Trial 1: H = 0.30 m Static
    if 1 in args.trials:
        ok1, m1 = run_static_test(1, 0.30, 0.30, "trial1_static_h030.csv")
        results["trial1"] = {"ok": ok1, "metrics": m1}

    # Trial 2: H = 0.40 m Static
    if 2 in args.trials:
        ok2, m2 = run_static_test(2, 0.40, 0.36, "trial2_static_h040.csv")
        results["trial2"] = {"ok": ok2, "metrics": m2}

    # Trial 3: H = 0.50 m Static
    if 3 in args.trials:
        ok3, m3 = run_static_test(3, 0.50, 0.36, "trial3_static_h050.csv")
        results["trial3"] = {"ok": ok3, "metrics": m3}

    # Trial 4: Continuous Sweep
    if 4 in args.trials:
        ok4, m4 = run_test_sweep()
        results["trial4"] = {"ok": ok4, "metrics": m4}

    # Trials 5/6: push smoke or repeated formal runs.  The current PID
    # parameters are intentionally reused; this path never invokes tuning.
    for trial_id, force_value, stem in [
            (5, +20.0, "trial5_push_pos20n"),
            (6, -20.0, "trial6_push_neg20n")]:
        if trial_id not in args.trials:
            continue
        for rep in range(1, args.repeats + 1):
            suffix = "" if args.repeats == 1 else f"_rep{rep}"
            key = f"trial{trial_id}" if args.repeats == 1 else f"trial{trial_id}_rep{rep}"
            attempts = 0
            while True:
                attempts += 1
                ok, metrics = run_push_test(
                    force_value, trial_id, f"{stem}{suffix}.csv")
                transient = metrics.get("reason") in {
                    "force_bridge_not_connected", "force_trigger_not_fired",
                    "insufficient_data", "invalid_pulse_protocol",
                    "response_window_missing"}
                if ok or not transient or attempts >= 3:
                    break
                print(f"  [Retry] transient push invalidity on repeat {rep}: "
                      f"{metrics.get('reason')} (attempt {attempts}/3)")
            # Keep the force value at the top level for aggregation, while
            # retaining the full pulse state and response diagnostics.
            metrics["force_value"] = force_value
            metrics["repeat"] = rep
            results[key] = {"ok": ok, "metrics": metrics}

    # The two pulse trials are a paired directional check, not two unrelated
    # scalar tests.  A disconnected bridge or same-direction response makes
    # the pair invalid even if one individual recovery happened to pass.
    for rep in range(1, args.repeats + 1):
        pos_key = "trial5" if args.repeats == 1 else f"trial5_rep{rep}"
        neg_key = "trial6" if args.repeats == 1 else f"trial6_rep{rep}"
        if pos_key in results and neg_key in results:
            pos = results[pos_key]["metrics"]
            neg = results[neg_key]["metrics"]
            d_pos = pos.get("response_direction", 0.0)
            d_neg = neg.get("response_direction", 0.0)
            expected = (pos.get("expected_response_direction") == 1.0 and
                        neg.get("expected_response_direction") == -1.0)
            opposite = d_pos != 0.0 and d_neg != 0.0 and d_pos == -d_neg
            paired = bool(expected and opposite and
                          pos.get("response_mapping_valid", False) and
                          neg.get("response_mapping_valid", False))
            pos["paired_direction_valid"] = paired
            neg["paired_direction_valid"] = paired
            if not paired:
                results[pos_key]["ok"] = False
                results[neg_key]["ok"] = False
                print(f"  [Error] push pair repeat {rep}: response direction mapping invalid")

    push_summary = {
        "+20N": summarize_push_repeats(results, +20.0),
        "-20N": summarize_push_repeats(results, -20.0),
    }

    with open(SUMMARY_PATH, "w") as handle:
        json.dump({
            "architecture": "three_loop_velocity_attitude_rate_torque",
            "position_feedback": {"k_x": 0.0, "origin": "fixed_p0_observation_only"},
            "limits": {"total_torque_max_nm": 20.0, "wheel_torque_max_nm": 10.0,
                       "rate_limit_u_nm_s": 0.0},
            "push_repeats": push_summary,
            "trials": results,
        }, handle, indent=2)
    print(f"\nAcceptance summary written to {SUMMARY_PATH}")
    for label, aggregate in push_summary.items():
        if aggregate.get("n", 0):
            print(f"  {label}: n={aggregate['n']}, pass={aggregate.get('n_pass', 0)}, "
                  f"peak_v={aggregate.get('peak_vel_resp_mps_mean', float('nan')):.3f}"
                  f"±{aggregate.get('peak_vel_resp_mps_std', 0.0):.3f} m/s, "
                  f"recovery={aggregate.get('recovery_time_s_mean', float('nan')):.2f}"
                  f"±{aggregate.get('recovery_time_s_std', 0.0):.2f} s")

    # Print Summary Table
    print("\n" + "=" * 75)
    print("                BBOT TORQUE CASCADE PID ACCEPTANCE SUMMARY")
    print("=" * 75)
    print(f"{'Trial':<10} | {'Status':<8} | {'Attitude Metric':<24} | {'Velocity Metric':<18} | {'Position Drift (p - p0)':<22}")
    print("-" * 75)

    for tid, name, att_str, vel_str, pos_str in [
        ("trial1", "1 (H=0.30)",
         lambda m: f"Pitch RMS: {m.get('pitch_rms_deg', 0):.4f} deg",
         lambda m: f"v RMS: {m.get('v_rms_mps', 0):.4f} m/s",
         lambda m: f"final: {m.get('final_drift_mm', 0):+.2f} mm"),
        ("trial2", "2 (H=0.40)",
         lambda m: f"Pitch RMS: {m.get('pitch_rms_deg', 0):.4f} deg",
         lambda m: f"v RMS: {m.get('v_rms_mps', 0):.4f} m/s",
         lambda m: f"final: {m.get('final_drift_mm', 0):+.2f} mm"),
        ("trial3", "3 (H=0.50)",
         lambda m: f"Pitch RMS: {m.get('pitch_rms_deg', 0):.4f} deg",
         lambda m: f"v RMS: {m.get('v_rms_mps', 0):.4f} m/s",
         lambda m: f"final: {m.get('final_drift_mm', 0):+.2f} mm"),
        ("trial4", "4 (Sweep)",
         lambda m: f"Max Pitch: {m.get('max_pitch_err_deg', 0):.2f} deg",
         lambda m: f"End v: {m.get('end_v_rms_mps', 0):.4f} m/s",
         lambda m: f"final: {m.get('final_drift_mm', 0):+.2f} mm"),
        ("trial5", "5 (+20N)",
         lambda m: f"Recovered: {m.get('recovery_time_s', 0):.2f} s",
         lambda m: f"Peak v: {m.get('peak_vel_resp_mps', 0):.3f} m/s",
         lambda m: f"net delta: {m.get('push_net_displacement_mm', 0):+.2f} mm"),
        ("trial6", "6 (-20N)",
         lambda m: f"Recovered: {m.get('recovery_time_s', 0):.2f} s",
         lambda m: f"Peak v: {m.get('peak_vel_resp_mps', 0):.3f} m/s",
         lambda m: f"net delta: {m.get('push_net_displacement_mm', 0):+.2f} mm"),
    ]:
        if tid in results:
            r = results[tid]
            st = "PASS" if r["ok"] else "FAIL"
            m = r["metrics"]
            print(f"{name:<10} | {st:<8} | {att_str(m):<24} | {vel_str(m):<18} | {pos_str(m):<22}")
    print("=" * 75)

    all_passed = all(r["ok"] for r in results.values()) if results else False
    print(f"Overall Acceptance Result: {'ALL SELECTED TRIALS PASSED' if all_passed else 'SOME TRIALS FAILED'}\n")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
