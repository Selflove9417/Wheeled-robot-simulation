#!/usr/bin/env python3
"""
Comparative analysis script for Two-Stage Adaptive GS-LQR under different apply rates:
- 1.0 mm/s (baseline)
- 1.5 mm/s
- 2.0 mm/s
Evaluates peak drift (|e_x|_max), total convergence time (Tc), torque peaks, pitch deviations, and steady-state errors.
"""

import os
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

os.environ["MPLCONFIGDIR"] = "/tmp/mpl"
os.makedirs("/tmp/mpl", exist_ok=True)

DATA_DIR = "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs"
ARTIFACT_DIR = "/home/admin/.gemini/antigravity/brain/281a16b2-d35b-4459-b32e-8cb3701236e0"

FILES = [
    {
        "label": r"$v_y^{\max} = 1.0\ \mathrm{mm/s}$ (Baseline)",
        "rate": 1.0,
        "csv": os.path.join(DATA_DIR, "adaptive_twostage_pos100.csv"),
        "color": "#1565c0",
        "linestyle": "-",
    },
    {
        "label": r"$v_y^{\max} = 1.5\ \mathrm{mm/s}$",
        "rate": 1.5,
        "csv": os.path.join(DATA_DIR, "adaptive_twostage_pos100_rate15.csv"),
        "color": "#2e7d32",
        "linestyle": "--",
    },
    {
        "label": r"$v_y^{\max} = 2.0\ \mathrm{mm/s}$",
        "rate": 2.0,
        "csv": os.path.join(DATA_DIR, "adaptive_twostage_pos100_rate20.csv"),
        "color": "#d84315",
        "linestyle": "-.",
    },
]


def load_and_align_trial(csv_path):
    if not os.path.exists(csv_path):
        return None
    raw = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    t = raw["time"]
    x_err = raw["filtered_x_error"] * 1000.0  # mm
    raw_x_err = raw["x_error"] * 1000.0
    u_model = raw["u_model"]
    pitch_err = (raw["pitch"] - raw["theta_eq_true"]) * 180.0 / np.pi
    hat = raw["delta_y_hat"] * 1000.0
    tgt = raw["delta_y_target"] * 1000.0
    state = raw["adapt_state"]
    gate = raw["gate_open"]

    # Detect reset
    reset_idx = 0
    for i in range(1, len(t)):
        if abs(raw_x_err[i] - raw_x_err[i-1]) > 100.0 and abs(raw_x_err[i]) < 50.0:
            reset_idx = i
            break
    if reset_idx == 0:
        for i in range(1, len(t)):
            if state[i-1] == 0 and state[i] == 1:
                reset_idx = max(0, i - 700)
                break

    t0 = t[reset_idx]
    t_rel = t - t0
    mask = t_rel >= -1.0

    # Metrics post reset
    post_mask = t_rel >= 0.0
    post_t = t_rel[post_mask]
    post_x_err = x_err[post_mask]
    post_state = state[post_mask]
    post_hat = hat[post_mask]
    post_tgt = tgt[post_mask]
    post_u = u_model[post_mask]
    post_pitch_err = pitch_err[post_mask]

    # Find coarse capture
    t_est_coarse = None
    t_apply_coarse = None
    for i in range(1, len(post_state)):
        if post_state[i-1] == 0 and post_state[i] == 1:
            t_est_coarse = post_t[i]
        if post_state[i-1] == 1 and post_state[i] == 2:
            t_apply_coarse = post_t[i] - (t_est_coarse if t_est_coarse else 0.0)

    # Find HOLD entry
    tc = None
    for i in range(1, len(post_state)):
        if post_state[i] == 5:
            tc = post_t[i]
            break

    max_drift = np.max(np.abs(post_x_err))
    max_u = np.max(np.abs(post_u))
    max_pitch = np.max(np.abs(post_pitch_err))

    # Steady state error (last 15s)
    ss_mask = post_t >= (post_t[-1] - 15.0)
    ss_x_mean = np.mean(post_x_err[ss_mask]) if np.any(ss_mask) else 0.0
    ss_x_std = np.std(post_x_err[ss_mask]) if np.any(ss_mask) else 0.0

    return {
        "t_rel": t_rel[mask],
        "x_err": x_err[mask],
        "hat": hat[mask],
        "tgt": tgt[mask],
        "u": u_model[mask],
        "pitch_err": pitch_err[mask],
        "state": state[mask],
        "gate": gate[mask],
        "t_est_coarse": t_est_coarse,
        "t_apply_coarse": t_apply_coarse,
        "tc": tc,
        "max_drift": max_drift,
        "max_u": max_u,
        "max_pitch": max_pitch,
        "final_hat": post_hat[-1],
        "final_tgt": post_tgt[-1],
        "ss_x_mean": ss_x_mean,
        "ss_x_std": ss_x_std,
    }


def main():
    trials_data = []
    for item in FILES:
        d = load_and_align_trial(item["csv"])
        if d is not None:
            trials_data.append((item, d))
        else:
            print(f"Warning: {item['csv']} not found or empty.")

    if not trials_data:
        print("No valid trial data found.")
        return

    # Print summary table
    print("\n" + "="*80)
    print("  Two-Stage Adaptive GS-LQR: Apply Rate Sensitivity Comparison (b_y = +10.0 mm)")
    print("="*80)
    print(f"{'Metric':<32} | {'1.0 mm/s (Base)':<15} | {'1.5 mm/s':<15} | {'2.0 mm/s':<15}")
    print("-" * 80)

    res = {}
    for item, d in trials_data:
        res[item["rate"]] = d

    def get_val(rate, key, fmt="{:.2f}"):
        if rate in res and res[rate][key] is not None:
            return fmt.format(res[rate][key])
        return "N/A"

    print(f"{'T_est,coarse (s)':<32} | {get_val(1.0, 't_est_coarse', '{:.3f}'):<15} | {get_val(1.5, 't_est_coarse', '{:.3f}'):<15} | {get_val(2.0, 't_est_coarse', '{:.3f}'):<15}")
    print(f"{'T_apply,coarse (s)':<32} | {get_val(1.0, 't_apply_coarse', '{:.3f}'):<15} | {get_val(1.5, 't_apply_coarse', '{:.3f}'):<15} | {get_val(2.0, 't_apply_coarse', '{:.3f}'):<15}")
    print(f"{'Total Convergence Tc (s)':<32} | {get_val(1.0, 'tc', '{:.3f}'):<15} | {get_val(1.5, 'tc', '{:.3f}'):<15} | {get_val(2.0, 'tc', '{:.3f}'):<15}")
    print(f"{'Max Drift |e_x|_max (mm)':<32} | {get_val(1.0, 'max_drift', '{:.2f}'):<15} | {get_val(1.5, 'max_drift', '{:.2f}'):<15} | {get_val(2.0, 'max_drift', '{:.2f}'):<15}")
    print(f"{'Peak Model Torque |u| (Nm)':<32} | {get_val(1.0, 'max_u', '{:.2f}'):<15} | {get_val(1.5, 'max_u', '{:.2f}'):<15} | {get_val(2.0, 'max_u', '{:.2f}'):<15}")
    print(f"{'Max Pitch Deviation (deg)':<32} | {get_val(1.0, 'max_pitch', '{:.3f}'):<15} | {get_val(1.5, 'max_pitch', '{:.3f}'):<15} | {get_val(2.0, 'max_pitch', '{:.3f}'):<15}")
    print(f"{'Final Target delta_y (mm)':<32} | {get_val(1.0, 'final_tgt', '{:.4f}'):<15} | {get_val(1.5, 'final_tgt', '{:.4f}'):<15} | {get_val(2.0, 'final_tgt', '{:.4f}'):<15}")
    print(f"{'Steady State x_err (mm)':<32} | {get_val(1.0, 'ss_x_mean', '{:+.2f}'):<15} | {get_val(1.5, 'ss_x_mean', '{:+.2f}'):<15} | {get_val(2.0, 'ss_x_mean', '{:+.2f}'):<15}")
    print("="*80 + "\n")

    # Plot 4-panel comparison
    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True, dpi=300)

    # Subplot 1: Position Error
    for item, d in trials_data:
        axes[0].plot(d["t_rel"], d["x_err"], label=item["label"], color=item["color"], linestyle=item["linestyle"], linewidth=1.8)
    axes[0].axhline(5.0, color="#b71c1c", linestyle=":", linewidth=1.2, label="Deadband ±5 mm")
    axes[0].axhline(-5.0, color="#b71c1c", linestyle=":", linewidth=1.2)
    axes[0].axhline(0.0, color="black", linestyle="--", linewidth=0.7)
    axes[0].axvline(0.0, color="green", linestyle="-.", linewidth=1.0, label="Reset Point")
    axes[0].set_ylabel("Position Error (mm)", fontsize=10, fontweight="bold")
    axes[0].set_title(r"Two-Stage Adaptive GS-LQR: Effect of Apply Rate ($b_y = +10.0\ \mathrm{mm}$)", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    # Subplot 2: COM-y Estimation
    for item, d in trials_data:
        axes[1].plot(d["t_rel"], d["hat"], label=rf"$\hat{{\Delta y}}$ ({item['rate']:.1f} mm/s)", color=item["color"], linestyle=item["linestyle"], linewidth=1.8)
    axes[1].axhline(-10.0, color="black", linestyle=":", linewidth=1.3, label=r"True Offset $-b_y$ (-10.00 mm)")
    axes[1].axvline(0.0, color="green", linestyle="-.", linewidth=1.0)
    axes[1].set_ylabel("COM-y Offset (mm)", fontsize=10, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    # Subplot 3: Pitch Deviation
    for item, d in trials_data:
        axes[2].plot(d["t_rel"], d["pitch_err"], label=rf"Pitch Dev ({item['rate']:.1f} mm/s)", color=item["color"], linestyle=item["linestyle"], linewidth=1.5)
    axes[2].axhline(0.0, color="black", linestyle="--", linewidth=0.7)
    axes[2].axvline(0.0, color="green", linestyle="-.", linewidth=1.0)
    axes[2].set_ylabel("Pitch Dev (deg)", fontsize=10, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    # Subplot 4: Model Torque
    for item, d in trials_data:
        axes[3].plot(d["t_rel"], d["u"], label=rf"Torque $u$ ({item['rate']:.1f} mm/s)", color=item["color"], linestyle=item["linestyle"], linewidth=1.5)
    axes[3].axhline(0.0, color="black", linestyle="--", linewidth=0.7)
    axes[3].axvline(0.0, color="green", linestyle="-.", linewidth=1.0)
    axes[3].set_ylabel("Torque (Nm)", fontsize=10, fontweight="bold")
    axes[3].set_xlabel("Relative Simulation Time from Reset (s)", fontsize=11, fontweight="bold")
    axes[3].set_xlim(-1.0, 50.0)
    axes[3].grid(True, linestyle=":", alpha=0.6)
    axes[3].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    plt.tight_layout()
    out_plot = os.path.join(DATA_DIR, "twostage_apply_rate_comparison.png")
    plt.savefig(out_plot)
    plt.close()
    print(f"Saved comparison figure to: {out_plot}")

    dest_art = os.path.join(ARTIFACT_DIR, "twostage_apply_rate_comparison.png")
    shutil.copy(out_plot, dest_art)
    print(f"Copied to artifact dir: {dest_art}")


if __name__ == "__main__":
    main()
