#!/usr/bin/env python3
"""
Single-run runner, monitor, and analyzer for the 4-state Adaptive GS-LQR state machine.
Automates Gazebo launch, reset, live state monitoring, and post-experiment analysis & plotting.
Supports configurable adaptive_apply_rate_max.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

os.environ["MPLCONFIGDIR"] = "/tmp/mpl"
os.makedirs("/tmp/mpl", exist_ok=True)


def parse_last_csv_line(path):
    """Read and parse the latest CSV record."""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            buf_size = min(size, 4096)
            f.seek(size - buf_size)
            lines = f.read().decode("utf-8", errors="ignore").strip().splitlines()
            if not lines:
                return None
            parts = lines[-1].split(",")
            if len(parts) < 45:
                return None
            return {
                "time": float(parts[0]),
                "x_ref": float(parts[5]),
                "x_error": float(parts[6]),
                "x_dot": float(parts[7]),
                "pitch": float(parts[8]),
                "pitch_rate": float(parts[9]),
                "theta_eq_nominal": float(parts[10]),
                "theta_eq_adaptive": float(parts[11]),
                "theta_eq_true": float(parts[15]),
                "delta_y_hat": float(parts[23]),
                "filtered_x_error": float(parts[25]),
                "gate_open": int(parts[29]) if parts[29].isdigit() else 0,
                "adapt_state": int(parts[35]) if parts[35].isdigit() else 0,
                "delta_y_target": float(parts[36]),
                "delta_y_apply_rate": float(parts[37]),
                "filtered_x_accel": float(parts[38]),
                "observation_window_range": float(parts[39]),
                "u_model": float(parts[44]),
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


def run_state_machine_trial(output_path, bias=0.001, apply_rate=0.0010, two_stage=True, hold_record_sec=25.0, max_sim_time=120.0):
    print("=======================================================")
    print(f"  Adaptive GS-LQR State Machine Trial (bias={bias*1000:+.2f}mm, rate={apply_rate*1000:.2f}mm/s, two_stage={two_stage})")
    print(f"  Target CSV: {output_path}")
    print(f"  Injected Bias: {bias*1000:+.3f} mm")
    print(f"  Apply Rate Max: {apply_rate*1000:.3f} mm/s ({apply_rate:.5f} m/s)")
    print(f"  Two-Stage Enabled: {two_stage}")
    print(f"  Hold Recording Duration: >= {hold_record_sec:.1f} s")
    print("=======================================================")

    cleanup_lingering_processes()
    if os.path.exists(output_path):
        os.remove(output_path)

    cmd_str = (
        f"source /home/admin/bbot_ws_new/install/setup.bash && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"controller_type:=adaptive_lqr "
        f"adaptive_experiment_mode:=adaptive "
        f"adaptive_com_y_bias:={bias} "
        f"adaptive_apply_rate_max:={apply_rate} "
        f"adaptive_two_stage_enabled:={str(two_stage).lower()} "
        f"adaptive_target_height:=0.50 "
        f"adaptive_startup_height:=0.36 "
        f"adaptive_log_path:={output_path}"
    )

    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs("/tmp/ros_home", exist_ok=True)
    os.makedirs("/tmp/ros_log", exist_ok=True)

    log_file = open("/tmp/trial_state_machine.log", "w")
    print(f"[Runner] Launching command: {cmd_str}")
    proc = subprocess.Popen(
        cmd_str,
        shell=True,
        executable="/bin/bash",
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True,
    )

    reset_sent = False
    start_real_time = time.time()
    last_report_sim = 0.0
    t_hold_entered = None
    state_names = {
        0: "WAIT_COARSE",
        1: "APPLY_COARSE",
        2: "WAIT_FINE",
        3: "APPLY_FINE",
        4: "VERIFY",
        5: "HOLD",
    }
    last_state = None

    try:
        while True:
            ret = proc.poll()
            if ret is not None:
                print(f"[Runner] Process exited unexpectedly with code {ret}")
                break

            record = parse_last_csv_line(output_path)
            if record is not None:
                t_sim = record["time"]
                cur_state = record["adapt_state"]

                # Check state transition
                if cur_state != last_state:
                    state_str = state_names.get(cur_state, str(cur_state))
                    old_str = state_names.get(last_state, str(last_state)) if last_state is not None else "INIT"
                    print(f"\n>>> [State Transition @ {t_sim:6.3f}s] {old_str} -> {state_str} (target={record['delta_y_target']*1000:+.3f}mm, hat={record['delta_y_hat']*1000:+.3f}mm, err={record['filtered_x_error']*1000:+.2f}mm)")
                    last_state = cur_state
                    if cur_state == 5 and t_hold_entered is None:
                        t_hold_entered = t_sim
                        print(f">>> [HOLD Entered @ {t_sim:6.3f}s] Will continue recording for {hold_record_sec} s...")

                # Send reset at ~6.74s
                if not reset_sent and t_sim >= 6.74:
                    print(f"\n[Runner] Sim time {t_sim:.3f} s reached. Publishing reset_position command...")
                    res = subprocess.run([
                        "ros2", "topic", "pub", "--once",
                        "--max-wait-time-secs", "5.0",
                        "/adaptive_lqr/command", "std_msgs/msg/String",
                        "{data: 'reset_position'}"
                    ], capture_output=True, text=True, env=env)
                    if res.returncode == 0:
                        print(f"[Runner] reset_position command sent successfully at {t_sim:.3f} s.")
                        reset_sent = True
                    else:
                        print(f"[Runner] Warning: pub returned {res.returncode}: {res.stderr}")

                # Periodic progress output
                if t_sim - last_report_sim >= 2.0:
                    last_report_sim = t_sim
                    elapsed_real = time.time() - start_real_time
                    state_str = state_names.get(cur_state, str(cur_state))
                    print(
                        f"[Sim {t_sim:5.1f}s | Real {elapsed_real:4.0f}s] "
                        f"State: {state_str:12s} | "
                        f"x_err: {record['filtered_x_error']*1000:+6.2f}mm | "
                        f"tgt: {record['delta_y_target']*1000:+6.3f}mm | "
                        f"hat: {record['delta_y_hat']*1000:+6.3f}mm | "
                        f"gate: {record['gate_open']} | "
                        f"u: {record['u_model']:+5.2f}Nm"
                    )

                # Termination check: entered HOLD and recorded enough
                if t_hold_entered is not None and (t_sim >= t_hold_entered + hold_record_sec):
                    print(f"\n[Runner] Successfully recorded {hold_record_sec}s in HOLD (t_sim = {t_sim:.2f}s). Terminating.")
                    break

                if t_sim >= max_sim_time:
                    print(f"\n[Runner] Max sim time {max_sim_time}s reached. Terminating.")
                    break

            time.sleep(0.2)

    finally:
        print("[Runner] Stopping simulation...")
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
            print(f"[Runner] Exception while stopping: {e}")
        cleanup_lingering_processes()
        try:
            log_file.close()
        except Exception:
            pass


def analyze_and_plot(csv_path):
    """Analyze trial log and generate comprehensive diagnostic figures."""
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        print(f"Error: {csv_path} does not exist or is empty.")
        return None

    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = data["time"]
    x_err = data["filtered_x_error"] * 1000.0  # mm
    raw_x_err = data["x_error"] * 1000.0  # mm
    x_dot = data["x_dot"]
    x_ref = data["x_ref"]
    pitch = np.rad2deg(data["pitch"])
    pitch_true = np.rad2deg(data["theta_eq_true"])
    pitch_err = pitch - pitch_true
    u_model = data["u_model"]
    delta_y_hat = data["delta_y_hat"] * 1000.0  # mm
    delta_y_target = data["delta_y_target"] * 1000.0  # mm
    delta_y_true = data["delta_y_true"] * 1000.0  # mm
    adapt_state = data["adapt_state"].astype(int)
    gate_open = data["gate_open"].astype(int)

    # Detect reset time (change in x_ref)
    diff_xref = np.diff(x_ref)
    reset_idx = np.where(np.abs(diff_xref) > 0.01)[0]
    t0 = t[reset_idx[0] + 1] if len(reset_idx) > 0 else t[0]

    # Post-reset data mask
    mask_post = t >= t0
    t_post = t[mask_post]
    state_post = adapt_state[mask_post]

    # Detect state transitions post-reset
    transitions = []
    prev_s = state_post[0]
    t_first_capture = None
    t_first_apply_end = None
    t_fine_capture = None
    t_fine_apply_end = None
    n_reverify = 0

    for cur_t, s in zip(t_post, state_post):
        if s != prev_s:
            transitions.append((cur_t, prev_s, s))
            if prev_s == 0 and s == 1 and t_first_capture is None:
                t_first_capture = cur_t
            elif prev_s == 1 and (s == 2 or s == 4) and t_first_apply_end is None:
                t_first_apply_end = cur_t
            elif prev_s == 2 and s == 3 and t_fine_capture is None:
                t_fine_capture = cur_t
            elif prev_s == 3 and s == 4 and t_fine_apply_end is None:
                t_fine_apply_end = cur_t
            elif (prev_s == 4 and s == 2) or (prev_s == 2 and s == 0):
                n_reverify += 1
            prev_s = s

    # Key durations
    t_est = (t_first_capture - t0) if t_first_capture is not None else None
    t_apply = (t_first_apply_end - t_first_capture) if (t_first_capture is not None and t_first_apply_end is not None) else None
    t_fine_wait = (t_fine_capture - t_first_apply_end) if (t_fine_capture is not None and t_first_apply_end is not None) else None
    t_fine_apply = (t_fine_apply_end - t_fine_capture) if (t_fine_capture is not None and t_fine_apply_end is not None) else None

    # Detect entry into HOLD (state 5 or legacy state 3)
    is_two_stage = np.any(adapt_state == 5) or np.any(adapt_state == 4) or np.any(adapt_state == 2)
    hold_state_val = 5 if is_two_stage else 3
    hold_mask = (adapt_state == hold_state_val) & mask_post
    if np.any(hold_mask):
        t_hold = t[hold_mask][0]
        tc = t_hold - t0
        hold_success = True
    else:
        t_hold = None
        tc = None
        hold_success = False

    # Steady-state statistics in HOLD (last 15s)
    t_end = t[-1]
    mask_ss = (t >= (t_end - 15.0)) & mask_post
    ss_x_err_mean = np.mean(x_err[mask_ss])
    ss_x_err_std = np.std(x_err[mask_ss])
    final_hat = delta_y_hat[-1]
    final_target = delta_y_target[-1]

    # Extreme metrics during the active adaptation phase (between t0 and t_hold)
    mask_adapt = (t >= t0) & (t <= (t_hold if t_hold is not None else t_end))
    max_pitch_dev = np.max(np.abs(pitch_err[mask_adapt]))
    max_u = np.max(np.abs(u_model[mask_adapt]))
    max_x_err_mag = np.max(np.abs(x_err[mask_adapt]))

    # Approximate max apply rate from CSV
    apply_rate_est = np.max(np.abs(data["delta_y_apply_rate"][mask_adapt])) * 1000.0  # mm/s

    state_names = {
        0: "WAIT_COARSE",
        1: "APPLY_COARSE",
        2: "WAIT_FINE",
        3: "APPLY_FINE",
        4: "VERIFY",
        5: "HOLD",
    } if is_two_stage else {
        0: "WAIT_CAPTURE",
        1: "APPLY_TARGET",
        2: "VERIFY",
        3: "HOLD",
    }

    print("\n=======================================================")
    print(f"  {'Two-Stage' if is_two_stage else '4-State'} Adaptive Machine Trial Quantitative Analysis")
    print("=======================================================")
    print(f"  Injected bias:                   {data['injected_com_y_bias'][0]*1000:+.2f} mm")
    print(f"  Apply rate:                      {apply_rate_est:.2f} mm/s")
    print(f"  Reset time t0:                   {t0:.3f} s")
    print(f"  End time:                        {t_end:.3f} s")
    print(f"  Post-reset duration:             {t_end - t0:.3f} s")
    print("  --- User Key Metrics ---")
    if t_est is not None:
        print(f"  T_est,coarse (Wait to Coarse):   {t_est:.3f} s")
    if t_apply is not None:
        print(f"  T_apply,coarse (Ramping Coarse): {t_apply:.3f} s")
    if t_fine_wait is not None:
        print(f"  T_wait,fine (Wait to Fine):      {t_fine_wait:.3f} s")
    if t_fine_apply is not None:
        print(f"  T_apply,fine (Ramping Fine):     {t_fine_apply:.3f} s")
    if hold_success:
        print(f"  T_c (Total Convergence to HOLD): {tc:.3f} s")
    print(f"  Max |e_x| (Maximum Drift):       {max_x_err_mag:.4f} mm")
    print(f"  Max pitch deviation:             {max_pitch_dev:.4f} deg")
    print(f"  Max |u_model|:                   {max_u:.4f} Nm")
    print(f"  VERIFY Re-capture Count:         {n_reverify}")
    print("  --- Accuracy & Steady State ---")
    print(f"  Target Captured (delta_y_target): {final_target:+.4f} mm (True offset: {delta_y_true[-1]:+.4f} mm)")
    print(f"  Estimation Error at Capture:     {abs(final_target - delta_y_true[-1]):.4f} mm")
    print(f"  Final delta_y_hat:               {final_hat:+.4f} mm")
    print(f"  Final Position Error (last 15s): {ss_x_err_mean:+.4f} ± {ss_x_err_std:.4f} mm")
    print("  --- State Transitions ---")
    for trans_t, from_s, to_s in transitions:
        rel_t = trans_t - t0
        print(f"    t = {trans_t:6.3f}s (rel +{rel_t:5.2f}s): {state_names.get(from_s, str(from_s))} -> {state_names.get(to_s, str(to_s))}")
    print("=======================================================\n")

    # ----------------------------------------------------
    # Plotting 4-Panel Diagnostic Figure
    # ----------------------------------------------------
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True, dpi=300)
    t_rel = t - t0
    plot_mask = t_rel >= -2.0  # Show from 2s before reset

    injected_bias = data["injected_com_y_bias"][0] * 1000.0

    # Subplot 1: Position Error
    axes[0].plot(t_rel[plot_mask], raw_x_err[plot_mask], color="#90caf9", alpha=0.6, linewidth=1.0, label="Raw x_error")
    axes[0].plot(t_rel[plot_mask], x_err[plot_mask], color="#1565c0", linewidth=1.8, label="Filtered x_error")
    axes[0].axhline(5.0, color="#d32f2f", linestyle="--", linewidth=1.2, label="Deadband ±5 mm")
    axes[0].axhline(-5.0, color="#d32f2f", linestyle="--", linewidth=1.2)
    axes[0].axhline(0.0, color="black", linestyle=":", linewidth=0.8)
    axes[0].axvline(0.0, color="green", linestyle="-.", linewidth=1.2, label="Reset (Space key)")
    if hold_success:
        axes[0].axvline(tc, color="purple", linestyle="-.", linewidth=1.2, label=f"HOLD Entry (Tc = {tc:.2f} s)")
    axes[0].set_ylabel("Position Error (mm)", fontsize=10, fontweight="bold")
    mode_name = "Two-Stage Adaptive GS-LQR" if is_two_stage else "4-State Adaptive GS-LQR"
    axes[0].set_title(f"{mode_name} ({injected_bias:+.2f} mm Bias, Apply Rate = {apply_rate_est:.1f} mm/s)", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    # Subplot 2: COM-y Estimation & Target
    axes[1].plot(t_rel[plot_mask], delta_y_hat[plot_mask], color="#2e7d32", linewidth=2.0, label=r"Applied Estimate $\hat{\Delta y}$")
    axes[1].plot(t_rel[plot_mask], delta_y_target[plot_mask], color="#e65100", linestyle="--", linewidth=1.5, label=r"Captured Target $\Delta y_{\rm target}$")
    axes[1].plot(t_rel[plot_mask], delta_y_true[plot_mask], color="black", linestyle=":", linewidth=1.3, label=rf"True Offset $-b_y$ ({delta_y_true[-1]:+.2f} mm)")
    axes[1].axvline(0.0, color="green", linestyle="-.", linewidth=1.2)
    if hold_success:
        axes[1].axvline(tc, color="purple", linestyle="-.", linewidth=1.2)
    axes[1].set_ylabel("COM-y Offset (mm)", fontsize=10, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    # Subplot 3: State Machine Phase & Gate
    axes[2].step(t_rel[plot_mask], adapt_state[plot_mask], color="#6a1b9a", linewidth=2.0, where="post", label="Adaptive Phase")
    axes[2].plot(t_rel[plot_mask], gate_open[plot_mask] * (0.8 if not is_two_stage else 1.2), color="#00838f", linewidth=1.0, alpha=0.7, label="Gate Open (scaled)")
    if is_two_stage:
        axes[2].set_yticks([0, 1, 2, 3, 4, 5])
        axes[2].set_yticklabels(["0: WAIT_C", "1: APPLY_C", "2: WAIT_F", "3: APPLY_F", "4: VERIFY", "5: HOLD"], fontsize=8.5)
        axes[2].set_ylim(-0.5, 5.5)
    else:
        axes[2].set_yticks([0, 1, 2, 3])
        axes[2].set_yticklabels(["0: WAIT", "1: APPLY", "2: VERIFY", "3: HOLD"], fontsize=9)
        axes[2].set_ylim(-0.5, 3.5)
    axes[2].set_ylabel("Adaptive Phase", fontsize=10, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="upper left", fontsize=8.5, framealpha=0.9)

    # Subplot 4: Control Effort & Pitch Error
    ax4_pitch = axes[3].twinx()
    l1 = axes[3].plot(t_rel[plot_mask], u_model[plot_mask], color="#c2185b", linewidth=1.5, label=r"Model Torque $u_{\rm model}$ (Nm)")
    l2 = ax4_pitch.plot(t_rel[plot_mask], pitch_err[plot_mask], color="#f57c00", linestyle="--", linewidth=1.2, label=r"Pitch Deviation $\theta - \theta_{\rm eq,true}$ (deg)")
    axes[3].set_ylabel("Torque (Nm)", fontsize=10, fontweight="bold", color="#c2185b")
    ax4_pitch.set_ylabel("Pitch Dev (deg)", fontsize=10, fontweight="bold", color="#f57c00")
    axes[3].set_xlabel("Relative Simulation Time from Reset (s)", fontsize=11, fontweight="bold")
    axes[3].grid(True, linestyle=":", alpha=0.6)
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    axes[3].legend(lines, labels, loc="upper right", fontsize=8.5, framealpha=0.9)

    plt.tight_layout()
    out_plot = os.path.splitext(csv_path)[0] + "_analysis.png"
    plt.savefig(out_plot)
    plt.close()
    print(f"Saved diagnostic plot to {out_plot}")

    # Copy to artifacts directory
    artifact_dir = "/home/admin/.gemini/antigravity/brain/281a16b2-d35b-4459-b32e-8cb3701236e0"
    dest = os.path.join(artifact_dir, os.path.basename(out_plot))
    shutil.copy(out_plot, dest)
    print(f"Copied plot to artifact dir: {dest}")

    return {
        "t0": t0,
        "t_end": t_end,
        "t_est": t_est,
        "t_apply": t_apply,
        "tc": tc,
        "hold_success": hold_success,
        "final_target": final_target,
        "final_hat": final_hat,
        "ss_x_err_mean": ss_x_err_mean,
        "ss_x_err_std": ss_x_err_std,
        "max_pitch_dev": max_pitch_dev,
        "max_u": max_u,
        "max_x_err_mag": max_x_err_mag,
        "n_reverify": n_reverify,
        "transitions": transitions,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/adaptive_twostage_pos100.csv")
    parser.add_argument("--bias", type=float, default=0.010)
    parser.add_argument("--apply-rate", type=float, default=0.0010)
    parser.add_argument("--hold-sec", type=float, default=25.0)
    parser.add_argument("--max-time", type=float, default=160.0)
    parser.add_argument("--no-two-stage", action="store_true", help="Disable two-stage adaptation")
    args = parser.parse_args()

    run_state_machine_trial(args.output, args.bias, args.apply_rate, not args.no_two_stage, args.hold_sec, args.max_time)
    analyze_and_plot(args.output)
