#!/usr/bin/env python3
"""
Automated campaign runner and statistical analyzer for the Two-Stage Adaptive GS-LQR State Machine.
Fulfills user request:
1. b_y = +10 mm independent repeatability trials (Run 2, Run 3) + statistical Mean ± Std.
2. Two-stage multi-amplitude trials: +1 mm, +5 mm, +10 mm (analyze T_est, |e_x|_max, T_c scaling).
3. Negative direction large bias trial: b_y = -5 mm (verify directional symmetry).
"""

import os
import shutil
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

os.environ["MPLCONFIGDIR"] = "/tmp/mpl"
os.makedirs("/tmp/mpl", exist_ok=True)

# Add current scripts directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
from run_state_machine_trial import run_state_machine_trial, analyze_and_plot, cleanup_lingering_processes

DATA_DIR = "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs"
ARTIFACT_DIR = "/home/admin/.gemini/antigravity/brain/281a16b2-d35b-4459-b32e-8cb3701236e0"


def execute_campaign():
    trials = [
        {
            "name": "+10mm_run2",
            "bias": 0.010,
            "apply_rate": 0.0010,
            "output": os.path.join(DATA_DIR, "adaptive_twostage_pos100_run2.csv"),
            "hold_sec": 25.0,
            "max_time": 120.0,
        },
        {
            "name": "+10mm_run3",
            "bias": 0.010,
            "apply_rate": 0.0010,
            "output": os.path.join(DATA_DIR, "adaptive_twostage_pos100_run3.csv"),
            "hold_sec": 25.0,
            "max_time": 120.0,
        },
        {
            "name": "+1mm",
            "bias": 0.001,
            "apply_rate": 0.0010,
            "output": os.path.join(DATA_DIR, "adaptive_twostage_pos10.csv"),
            "hold_sec": 25.0,
            "max_time": 120.0,
        },
        {
            "name": "+5mm",
            "bias": 0.005,
            "apply_rate": 0.0010,
            "output": os.path.join(DATA_DIR, "adaptive_twostage_pos50.csv"),
            "hold_sec": 25.0,
            "max_time": 120.0,
        },
        {
            "name": "-5mm",
            "bias": -0.005,
            "apply_rate": 0.0010,
            "output": os.path.join(DATA_DIR, "adaptive_twostage_neg50.csv"),
            "hold_sec": 25.0,
            "max_time": 120.0,
        },
    ]

    results = {}
    
    # Also include existing Run 1 of +10mm
    run1_path = os.path.join(DATA_DIR, "adaptive_twostage_pos100.csv")
    if os.path.exists(run1_path) and os.path.getsize(run1_path) > 1000:
        print(f"\n[Campaign] Analyzing existing +10mm Run 1: {run1_path}")
        results["+10mm_run1"] = analyze_and_plot(run1_path)

    for idx, t_info in enumerate(trials, 1):
        print(f"\n=======================================================")
        print(f"  [Campaign] Starting Trial {idx}/{len(trials)}: {t_info['name']} (bias={t_info['bias']*1000:+.2f}mm)")
        print(f"=======================================================")
        
        run_state_machine_trial(
            output_path=t_info["output"],
            bias=t_info["bias"],
            apply_rate=t_info["apply_rate"],
            two_stage=True,
            hold_record_sec=t_info["hold_sec"],
            max_sim_time=t_info["max_time"],
        )
        
        metrics = analyze_and_plot(t_info["output"])
        results[t_info["name"]] = metrics

    # ----------------------------------------------------
    # 1. Repeatability Analysis (+10 mm: Run 1, Run 2, Run 3)
    # ----------------------------------------------------
    rep_runs = [results.get(k) for k in ["+10mm_run1", "+10mm_run2", "+10mm_run3"] if results.get(k) is not None]
    if len(rep_runs) >= 2:
        t_est_list = [r["t_est"] for r in rep_runs if r["t_est"] is not None]
        tc_list = [r["tc"] for r in rep_runs if r["tc"] is not None]
        drift_list = [r["max_x_err_mag"] for r in rep_runs]
        pitch_list = [r["max_pitch_dev"] for r in rep_runs]
        torque_list = [r["max_u"] for r in rep_runs]
        err_list = [abs(r["final_target"] - (-10.0)) for r in rep_runs]

        print("\n" + "#" * 60)
        print("  1. REPEATABILITY STATISTICAL SUMMARY (b_y = +10.0 mm, N=3)")
        print("#" * 60)
        print(f"  T_est,coarse:      {np.mean(t_est_list):.3f} ± {np.std(t_est_list):.3f} s  (Individual: {[round(x, 3) for x in t_est_list]})")
        print(f"  T_c (Total):       {np.mean(tc_list):.3f} ± {np.std(tc_list):.3f} s  (Individual: {[round(x, 3) for x in tc_list]})")
        print(f"  Max |e_x| (Drift): {np.mean(drift_list):.2f} ± {np.std(drift_list):.2f} mm  (Individual: {[round(x, 2) for x in drift_list]})")
        print(f"  Max Pitch Dev:     {np.mean(pitch_list):.3f} ± {np.std(pitch_list):.3f} deg  (Individual: {[round(x, 3) for x in pitch_list]})")
        print(f"  Max Model Torque:  {np.mean(torque_list):.3f} ± {np.std(torque_list):.3f} Nm   (Individual: {[round(x, 3) for x in torque_list]})")
        print(f"  Target Residual:   {np.mean(err_list):.4f} ± {np.std(err_list):.4f} mm (Individual: {[round(x, 4) for x in err_list]})")
        print("#" * 60)

    # ----------------------------------------------------
    # 2. Multi-Amplitude Scaling (+1 mm, +5 mm, +10 mm)
    # ----------------------------------------------------
    print("\n" + "#" * 60)
    print("  2. MULTI-AMPLITUDE SCALING (+1.0 mm, +5.0 mm, +10.0 mm)")
    print("#" * 60)
    amp_keys = [("+1mm", 1.0), ("+5mm", 5.0), ("+10mm_run1", 10.0)]
    print(f"  {'Bias':>8s} | {'T_est (s)':>10s} | {'Max Drift (mm)':>14s} | {'T_c (s)':>10s} | {'Target Error (mm)':>18s} | {'Hold e_x (mm)':>14s}")
    print("  " + "-" * 82)
    for name, b_val in amp_keys:
        r = results.get(name)
        if r:
            target_err = abs(r["final_target"] - (-b_val))
            print(f"  {b_val:+7.1f}mm | {r['t_est']:10.3f} | {r['max_x_err_mag']:14.2f} | {r['tc']:10.3f} | {target_err:18.4f} | {r['ss_x_err_mean']:+14.2f}")
    print("#" * 60)

    # ----------------------------------------------------
    # 3. Directional Symmetry (+5 mm vs. -5 mm)
    # ----------------------------------------------------
    r_pos5 = results.get("+5mm")
    r_neg5 = results.get("-5mm")
    if r_pos5 and r_neg5:
        print("\n" + "#" * 60)
        print("  3. DIRECTIONAL SYMMETRY EVALUATION (+5.0 mm vs. -5.0 mm)")
        print("#" * 60)
        print(f"  {'Metric':<25s} | {'+5.0 mm':>12s} | {'-5.0 mm':>12s} | {'Symmetry Asymmetry / Diff':>25s}")
        print("  " + "-" * 80)
        print(f"  {'T_est (Coarse Capture)':<25s} | {r_pos5['t_est']:11.3f}s | {r_neg5['t_est']:11.3f}s | {abs(r_pos5['t_est'] - r_neg5['t_est']):24.3f}s")
        print(f"  {'T_c (Total Convergence)':<25s} | {r_pos5['tc']:11.3f}s | {r_neg5['tc']:11.3f}s | {abs(r_pos5['tc'] - r_neg5['tc']):24.3f}s")
        print(f"  {'Max Drift |e_x|_max':<25s} | {r_pos5['max_x_err_mag']:10.2f}mm | {r_neg5['max_x_err_mag']:10.2f}mm | {abs(r_pos5['max_x_err_mag'] - r_neg5['max_x_err_mag']):23.2f}mm")
        print(f"  {'Max Pitch Dev':<25s} | {r_pos5['max_pitch_dev']:10.3f}° | {r_neg5['max_pitch_dev']:10.3f}° | {abs(r_pos5['max_pitch_dev'] - r_neg5['max_pitch_dev']):23.3f}°")
        print(f"  {'Max Model Torque':<25s} | {r_pos5['max_u']:10.3f}Nm | {r_neg5['max_u']:10.3f}Nm | {abs(r_pos5['max_u'] - r_neg5['max_u']):23.3f}Nm")
        print(f"  {'Final Target':<25s} | {r_pos5['final_target']:+10.4f}mm | {r_neg5['final_target']:+10.4f}mm | {abs(r_pos5['final_target'] + r_neg5['final_target']):23.4f}mm")
        print(f"  {'Target Error':<25s} | {abs(r_pos5['final_target'] - (-5.0)):10.4f}mm | {abs(r_neg5['final_target'] - (+5.0)):10.4f}mm | {abs(abs(r_pos5['final_target'] - (-5.0)) - abs(r_neg5['final_target'] - (+5.0))):23.4f}mm")
        print("#" * 60)

    # ----------------------------------------------------
    # Generate Multi-Panel Comparison Figures
    # ----------------------------------------------------
    generate_repeatability_plot()
    generate_amplitude_symmetry_plot()


def generate_repeatability_plot():
    """Overlay 3 repeatability trials for +10 mm."""
    paths = [
        os.path.join(DATA_DIR, "adaptive_twostage_pos100.csv"),
        os.path.join(DATA_DIR, "adaptive_twostage_pos100_run2.csv"),
        os.path.join(DATA_DIR, "adaptive_twostage_pos100_run3.csv"),
    ]
    colors = ["#1565c0", "#2e7d32", "#c2185b"]
    labels = ["Run 1", "Run 2", "Run 3"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, dpi=300)

    for path, color, label in zip(paths, colors, labels):
        if not os.path.exists(path):
            continue
        data = np.genfromtxt(path, delimiter=",", names=True)
        t = data["time"]
        diff_xref = np.diff(data["x_ref"])
        reset_idx = np.where(np.abs(diff_xref) > 0.01)[0]
        t0 = t[reset_idx[0] + 1] if len(reset_idx) > 0 else t[0]
        t_rel = t - t0
        mask = t_rel >= -1.0

        # Subplot 1: Position Error
        axes[0].plot(t_rel[mask], data["filtered_x_error"][mask] * 1000.0, color=color, linewidth=1.6, label=f"{label}")

        # Subplot 2: Target & Hat
        axes[1].plot(t_rel[mask], data["delta_y_hat"][mask] * 1000.0, color=color, linewidth=1.6, label=f"{label} $\hat{{\Delta y}}$")

        # Subplot 3: Adaptive Phase
        axes[2].step(t_rel[mask], data["adapt_state"][mask], color=color, linewidth=1.6, where="post", label=f"{label}")

    axes[0].axhline(5.0, color="#d32f2f", linestyle="--", linewidth=1.0, label="Deadband ±5 mm")
    axes[0].axhline(-5.0, color="#d32f2f", linestyle="--", linewidth=1.0)
    axes[0].axhline(0.0, color="black", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("Position Error (mm)", fontsize=10, fontweight="bold")
    axes[0].set_title(r"Two-Stage Adaptive GS-LQR Repeatability ($b_y = +10.0\rm\,mm$, N=3)", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    axes[1].axhline(-10.0, color="black", linestyle=":", linewidth=1.2, label=r"True Offset $-b_y$ (-10 mm)")
    axes[1].set_ylabel("COM Offset (mm)", fontsize=10, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    axes[2].set_yticks([0, 1, 2, 3, 4, 5])
    axes[2].set_yticklabels(["0: WAIT_C", "1: APPLY_C", "2: WAIT_F", "3: APPLY_F", "4: VERIFY", "5: HOLD"], fontsize=8.5)
    axes[2].set_ylim(-0.5, 5.5)
    axes[2].set_ylabel("Adaptive Phase", fontsize=10, fontweight="bold")
    axes[2].set_xlabel("Relative Simulation Time from Reset (s)", fontsize=10, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, "twostage_repeatability_pos100.png")
    plt.savefig(out_path)
    plt.close()
    print(f"[Campaign] Saved repeatability plot to: {out_path}")
    shutil.copy(out_path, os.path.join(ARTIFACT_DIR, os.path.basename(out_path)))


def generate_amplitude_symmetry_plot():
    """Compare multi-amplitude (+1, +5, +10 mm) and directional symmetry (+5 vs -5 mm)."""
    case_specs = [
        {"path": os.path.join(DATA_DIR, "adaptive_twostage_pos10.csv"), "label": r"$b_y = +1.0\rm\,mm$", "color": "#1976d2", "ls": "-"},
        {"path": os.path.join(DATA_DIR, "adaptive_twostage_pos50.csv"), "label": r"$b_y = +5.0\rm\,mm$", "color": "#388e3c", "ls": "-"},
        {"path": os.path.join(DATA_DIR, "adaptive_twostage_pos100.csv"), "label": r"$b_y = +10.0\rm\,mm$", "color": "#d32f2f", "ls": "-"},
        {"path": os.path.join(DATA_DIR, "adaptive_twostage_neg50.csv"), "label": r"$b_y = -5.0\rm\,mm$ (Symmetry)", "color": "#7b1fa2", "ls": "--"},
    ]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True, dpi=300)

    for spec in case_specs:
        if not os.path.exists(spec["path"]):
            continue
        data = np.genfromtxt(spec["path"], delimiter=",", names=True)
        t = data["time"]
        diff_xref = np.diff(data["x_ref"])
        reset_idx = np.where(np.abs(diff_xref) > 0.01)[0]
        t0 = t[reset_idx[0] + 1] if len(reset_idx) > 0 else t[0]
        t_rel = t - t0
        mask = t_rel >= -1.0

        # Subplot 1: Position Error
        axes[0].plot(t_rel[mask], data["filtered_x_error"][mask] * 1000.0, color=spec["color"], linestyle=spec["ls"], linewidth=1.7, label=spec["label"])

        # Subplot 2: Applied Compensation
        axes[1].plot(t_rel[mask], data["delta_y_hat"][mask] * 1000.0, color=spec["color"], linestyle=spec["ls"], linewidth=1.7, label=spec["label"])

        # Subplot 3: State Machine Phase
        axes[2].step(t_rel[mask], data["adapt_state"][mask], color=spec["color"], linestyle=spec["ls"], linewidth=1.6, where="post", label=spec["label"])

    axes[0].axhline(5.0, color="#d32f2f", linestyle=":", linewidth=1.0, label="Deadband ±5 mm")
    axes[0].axhline(-5.0, color="#d32f2f", linestyle=":", linewidth=1.0)
    axes[0].axhline(0.0, color="black", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("Position Error (mm)", fontsize=10, fontweight="bold")
    axes[0].set_title("Two-Stage Adaptive GS-LQR: Amplitude Scaling & Directional Symmetry", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    axes[1].axhline(0.0, color="black", linestyle=":", linewidth=0.8)
    axes[1].set_ylabel("Applied Offset $\hat{\Delta y}$ (mm)", fontsize=10, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="lower right", fontsize=8.5, framealpha=0.9)

    axes[2].set_yticks([0, 1, 2, 3, 4, 5])
    axes[2].set_yticklabels(["0: WAIT_C", "1: APPLY_C", "2: WAIT_F", "3: APPLY_F", "4: VERIFY", "5: HOLD"], fontsize=8.5)
    axes[2].set_ylim(-0.5, 5.5)
    axes[2].set_ylabel("Adaptive Phase", fontsize=10, fontweight="bold")
    axes[2].set_xlabel("Relative Simulation Time from Reset (s)", fontsize=10, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, "twostage_multiamplitude_comparison.png")
    plt.savefig(out_path)
    plt.close()
    print(f"[Campaign] Saved amplitude & symmetry comparison plot to: {out_path}")
    shutil.copy(out_path, os.path.join(ARTIFACT_DIR, os.path.basename(out_path)))


if __name__ == "__main__":
    execute_campaign()
