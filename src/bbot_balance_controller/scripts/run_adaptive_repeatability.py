#!/usr/bin/env python3
"""
Automated experiment runner for Adaptive GS-LQR repeatability trials.
Executes trials under the identical initialization, reset, and timing protocol.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
import numpy as np


def parse_last_csv_line(path):
    """Read the last non-empty line of a CSV file and extract sim_time and fields."""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            buffer_size = min(size, 4096)
            f.seek(size - buffer_size)
            lines = f.read().decode("utf-8", errors="ignore").strip().splitlines()
            if not lines:
                return None
            last_line = lines[-1]
            parts = last_line.split(",")
            if len(parts) < 37:
                return None
            sim_time = float(parts[0])
            x_error = float(parts[6])
            delta_y_hat = float(parts[23])
            gate_open = int(parts[29]) if parts[29].isdigit() else 0
            u_model = float(parts[36])
            x_ref = float(parts[5])
            return {
                "time": sim_time,
                "x_error": x_error,
                "delta_y_hat": delta_y_hat,
                "gate_open": gate_open,
                "u_model": u_model,
                "x_ref": x_ref,
            }
    except Exception:
        return None


def cleanup_lingering_processes():
    """Ensure no stale Gazebo or ROS controller nodes remain."""
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "adaptive_lqr_balance_controller"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "spawner"], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def analyze_log(path):
    """Compute standard metrics for the trial."""
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        print(f"Error: log file {path} not found or empty.")
        return None

    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["time"]
    x_err = data["filtered_x_error"]
    x_ref = data["x_ref"]
    delta_y_hat = data["delta_y_hat"]
    u_model = data["u_model"]
    pitch = data["pitch"]
    theta_eq_true = data["theta_eq_true"]
    steps = data["delta_y_step"]

    # Detect reset time (change in x_ref)
    diff_xref = np.diff(x_ref)
    reset_idx = np.where(np.abs(diff_xref) > 0.01)[0]
    if len(reset_idx) > 0:
        t0 = t[reset_idx[0] + 1]
    else:
        t0 = t[0]

    # Detect permanent entry into +/- 5 mm deadband
    in_deadband = np.abs(x_err) <= 0.005
    last_outside = np.where(~in_deadband)[0]
    if len(last_outside) > 0 and last_outside[-1] < len(t) - 1:
        perm_enter_idx = last_outside[-1] + 1
        t_perm = t[perm_enter_idx]
        tc = t_perm - t0
        permanent_ok = True
    else:
        t_perm = None
        tc = None
        permanent_ok = False

    update_idx = np.where(np.abs(steps) > 1e-6)[0]
    n_updates = len(update_idx)

    # Last 10 seconds statistics
    mask_last10 = t >= (t[-1] - 10.0)
    final_x_err_mean = np.mean(x_err[mask_last10]) * 1000.0  # mm
    final_x_err_std = np.std(x_err[mask_last10]) * 1000.0  # mm
    final_delta_y_hat = delta_y_hat[-1] * 1000.0  # mm

    # Max metrics after reset
    post_reset_mask = t >= t0
    max_u = np.max(np.abs(u_model[post_reset_mask]))
    max_pitch_dev = np.rad2deg(np.max(np.abs(pitch[post_reset_mask] - theta_eq_true[post_reset_mask])))

    return {
        "t0": t0,
        "t_end": t[-1],
        "t_perm": t_perm,
        "tc": tc,
        "permanent_ok": permanent_ok,
        "n_updates": n_updates,
        "final_x_err_mean": final_x_err_mean,
        "final_x_err_std": final_x_err_std,
        "final_delta_y_hat": final_delta_y_hat,
        "max_u": max_u,
        "max_pitch_dev": max_pitch_dev,
        "total_points": len(t),
    }


def run_trial(trial_id, output_path, bias=0.001, max_sim_time=160.0):
    print(f"\n=======================================================")
    print(f"  Starting Trial {trial_id}")
    print(f"  Output CSV: {output_path}")
    print(f"  Injected COM-y Bias: {bias*1000:+.3f} mm")
    print(f"  Target max sim time: {max_sim_time} s")
    print(f"=======================================================")

    cleanup_lingering_processes()

    if os.path.exists(output_path):
        os.remove(output_path)

    cmd = [
        "ros2", "launch", "bbot_bringup", "bbot_gazebo.launch.py",
        "controller_type:=adaptive_lqr",
        "adaptive_experiment_mode:=adaptive",
        f"adaptive_com_y_bias:={bias}",
        "adaptive_target_height:=0.50",
        "adaptive_startup_height:=0.36",
        f"adaptive_log_path:={output_path}",
    ]

    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs("/tmp/ros_home", exist_ok=True)
    os.makedirs("/tmp/ros_log", exist_ok=True)

    log_file_handle = open(f"/tmp/trial_{trial_id}.log", "w")
    print(f"[Runner] Launching command: {' '.join(cmd)}")
    print(f"[Runner] Launch stdout/stderr redirected to /tmp/trial_{trial_id}.log")
    proc = subprocess.Popen(
        cmd,
        stdout=log_file_handle,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True,
    )

    reset_sent = False
    start_real_time = time.time()
    last_report_sim_time = 0.0

    try:
        while True:
            ret = proc.poll()
            if ret is not None:
                print(f"[Runner] Launch process terminated unexpectedly with code {ret}!")
                break

            state = parse_last_csv_line(output_path)
            if state is not None:
                sim_time = state["time"]

                # Send reset command at sim_time ~ 6.74 s (after standing up and settling at 0.50m)
                if not reset_sent and sim_time >= 6.74:
                    print(f"\n[Runner] Sim time is {sim_time:.3f} s. Triggering 'reset_position' (Space key equivalent)...")
                    pub_res = subprocess.run([
                        "ros2", "topic", "pub", "--once",
                        "--max-wait-time-secs", "5.0",
                        "/adaptive_lqr/command", "std_msgs/msg/String",
                        "{data: 'reset_position'}"
                    ], capture_output=True, text=True, env=env)
                    if pub_res.returncode == 0:
                        print(f"[Runner] 'reset_position' published successfully at sim_time = {sim_time:.3f} s.")
                        reset_sent = True
                    else:
                        print(f"[Runner] Warning: pub returned code {pub_res.returncode}: {pub_res.stderr}")

                # Report status every 10 sim seconds
                if sim_time - last_report_sim_time >= 10.0:
                    last_report_sim_time = sim_time
                    real_elapsed = time.time() - start_real_time
                    print(
                        f"[Sim {sim_time:6.1f}s | Real {real_elapsed:5.1f}s] "
                        f"x_err={state['x_error']*1000:+6.2f}mm, "
                        f"hat={state['delta_y_hat']*1000:+6.3f}mm, "
                        f"gate={state['gate_open']}, "
                        f"u={state['u_model']:+5.2f}Nm"
                    )

                # Termination check: when sim_time reaches target duration
                if sim_time >= max_sim_time:
                    print(f"\n[Runner] Target sim time {sim_time:.2f} s >= {max_sim_time} s reached.")
                    break

            time.sleep(0.5)

    finally:
        print("[Runner] Shutting down simulation process group...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(10):
                if proc.poll() is not None:
                    break
                time.sleep(0.5)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                time.sleep(1.0)
        except Exception as e:
            print(f"[Runner] Exception during shutdown: {e}")

        cleanup_lingering_processes()
        try:
            log_file_handle.close()
        except Exception:
            pass

    print(f"[Runner] Analyzing results for Trial {trial_id}...")
    metrics = analyze_log(output_path)
    if metrics is not None:
        print(f"-------------------------------------------------------")
        print(f"  Trial {trial_id} Results:")
        print(f"  Reset time t0:           {metrics['t0']:.3f} s")
        print(f"  End time:                {metrics['t_end']:.3f} s")
        if metrics['permanent_ok']:
            print(f"  Permanent entry time:    {metrics['t_perm']:.3f} s")
            print(f"  Convergence time Tc:     {metrics['tc']:.3f} s")
        else:
            print(f"  Permanent entry:         FAILED (not inside 5mm deadband at end)")
        print(f"  Updates count:           {metrics['n_updates']}")
        print(f"  Final x_error:           {metrics['final_x_err_mean']:.4f} +/- {metrics['final_x_err_std']:.6f} mm")
        print(f"  Final delta_y_hat:       {metrics['final_delta_y_hat']:.4f} mm")
        print(f"  Max |u_model|:           {metrics['max_u']:.4f} Nm")
        print(f"  Max pitch deviation:     {metrics['max_pitch_dev']:.4f} deg")
        print(f"-------------------------------------------------------")
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run repeatability trials")
    parser.add_argument("--trial-id", type=str, required=True, help="Trial ID (e.g. run2, run3)")
    parser.add_argument("--output", type=str, required=True, help="Output CSV path")
    parser.add_argument("--bias", type=float, default=0.001, help="COM-y bias in metres (default 0.001)")
    parser.add_argument("--max-sim-time", type=float, default=160.0, help="Max sim time in seconds")
    args = parser.parse_args()

    res = run_trial(args.trial_id, args.output, args.bias, args.max_sim_time)
    if res is None or not res.get("permanent_ok", False):
        sys.exit(1)
    sys.exit(0)
