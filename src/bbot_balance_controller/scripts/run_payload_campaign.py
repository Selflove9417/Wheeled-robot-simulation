#!/usr/bin/env python3
"""
Automated campaign runner and statistical analyzer for the Real Physical Payload Experiment in Gazebo.
Executes the 5-trial matrix:
  - P0: 0 kg Baseline (Nominal GS-LQR)
  - P1: 1 kg Front (+0.10 m) Nominal GS-LQR (Uncompensated drift)
  - P2: 1 kg Front (+0.10 m) Two-Stage Adaptive GS-LQR
  - P3: 1 kg Rear  (-0.10 m) Nominal GS-LQR (Uncompensated drift)
  - P4: 1 kg Rear  (-0.10 m) Two-Stage Adaptive GS-LQR
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

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
from run_state_machine_trial import run_state_machine_trial, analyze_and_plot, cleanup_lingering_processes

DATA_DIR = "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs"
ARTIFACT_DIR = "/home/admin/.gemini/antigravity/brain/afffd935-6e87-4bce-b7c1-7a2c9a70b720"


def execute_payload_campaign(rerun_nominal_only=True):
    trials = [
        {
            "id": "P0",
            "name": "0kg Baseline (Nominal)",
            "mode": "nominal",
            "payload_mass": 0.0,
            "payload_y_offset": 0.0,
            "output": os.path.join(DATA_DIR, "payload_P0_baseline.csv"),
            "max_time": 80.0,
            "hold_sec": 20.0,
            "color": "#757575",
            "linestyle": ":",
        },
        {
            "id": "P1",
            "name": "1kg Front (+0.10m) Nominal",
            "mode": "nominal",
            "payload_mass": 1.0,
            "payload_y_offset": 0.10,
            "output": os.path.join(DATA_DIR, "payload_P1_front_nominal.csv"),
            "max_time": 80.0,
            "hold_sec": 20.0,
            "color": "#d32f2f",
            "linestyle": "--",
        },
        {
            "id": "P2",
            "name": "1kg Front (+0.10m) Adaptive",
            "mode": "adaptive",
            "payload_mass": 1.0,
            "payload_y_offset": 0.10,
            "output": os.path.join(DATA_DIR, "payload_P2_front_adaptive.csv"),
            "max_time": 120.0,
            "hold_sec": 25.0,
            "color": "#1976d2",
            "linestyle": "-",
        },
        {
            "id": "P3",
            "name": "1kg Rear (-0.10m) Nominal",
            "mode": "nominal",
            "payload_mass": 1.0,
            "payload_y_offset": -0.10,
            "output": os.path.join(DATA_DIR, "payload_P3_rear_nominal.csv"),
            "max_time": 80.0,
            "hold_sec": 20.0,
            "color": "#f57c00",
            "linestyle": "--",
        },
        {
            "id": "P4",
            "name": "1kg Rear (-0.10m) Adaptive",
            "mode": "adaptive",
            "payload_mass": 1.0,
            "payload_y_offset": -0.10,
            "output": os.path.join(DATA_DIR, "payload_P4_rear_adaptive.csv"),
            "max_time": 120.0,
            "hold_sec": 25.0,
            "color": "#388e3c",
            "linestyle": "-",
        },
    ]

    results = {}

    for idx, t_info in enumerate(trials, 1):
        print(f"\n{'='*65}")
        print(f"  [Payload Campaign] Starting Trial {idx}/{len(trials)}: {t_info['id']} - {t_info['name']}")
        print(f"{'='*65}")

        # If rerun_nominal_only is True, reuse existing P2 and P4 if they already completed HOLD
        skip = False
        if rerun_nominal_only and t_info["id"] in ["P2", "P4"] and os.path.exists(t_info["output"]):
            if os.path.getsize(t_info["output"]) > 1000:
                print(f"[Payload Campaign] Reusing existing completed trial {t_info['id']}.")
                skip = True

        if not skip:
            run_state_machine_trial(
                output_path=t_info["output"],
                bias=0.0,
                apply_rate=0.0010,
                two_stage=True,
                hold_record_sec=t_info["hold_sec"],
                max_sim_time=t_info["max_time"],
                mode=t_info["mode"],
                payload_mass=t_info["payload_mass"],
                payload_y_offset=t_info["payload_y_offset"],
                payload_z=0.170,
            )

        metrics = analyze_and_plot(t_info["output"])
        results[t_info["id"]] = {
            "info": t_info,
            "metrics": metrics,
        }

    # ----------------------------------------------------
    # Multi-Curve Diagnostic Plotting
    # ----------------------------------------------------
    print(f"\n[Payload Campaign] Generating multi-trial comparison plots...")
    fig, axes = plt.subplots(4, 1, figsize=(13, 16), sharex=False)

    for t_info in trials:
        csv_path = t_info["output"]
        if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
            continue
        data = np.genfromtxt(csv_path, delimiter=",", names=True)
        t = data["time"]
        x_ref = data["x_ref"]
        diff_xref = np.diff(x_ref)
        reset_idx = np.where(np.abs(diff_xref) > 0.01)[0]
        t0 = t[reset_idx[0] + 1] if len(reset_idx) > 0 else t[0]

        mask = t >= t0
        t_rel = t[mask] - t0
        x_err = data["filtered_x_error"][mask] * 1000.0  # mm
        delta_y_hat = data["delta_y_hat"][mask] * 1000.0  # mm
        u_model = data["u_model"][mask]
        theta_err = np.rad2deg(data["theta_error"][mask])  # deg (theta - theta_eq_adaptive)

        label = f"{t_info['id']}: {t_info['name']}"

        # 1. Position Error
        axes[0].plot(t_rel, x_err, color=t_info["color"], linestyle=t_info["linestyle"], linewidth=1.8, label=label)

        # 2. Adaptive Equivalent Offset
        axes[1].plot(t_rel, delta_y_hat, color=t_info["color"], linestyle=t_info["linestyle"], linewidth=1.8, label=label)

        # 3. Control Torque
        axes[2].plot(t_rel, u_model, color=t_info["color"], linestyle=t_info["linestyle"], linewidth=1.4, label=label)

        # 4. Attitude Control Error e_theta = theta - theta_eq
        axes[3].plot(t_rel, theta_err, color=t_info["color"], linestyle=t_info["linestyle"], linewidth=1.4, label=label)

    # Subplot styling
    axes[0].axhline(+5.0, color="#d32f2f", linestyle=":", linewidth=1.0, alpha=0.7, label=r"Task Deadband $\pm 5$ mm")
    axes[0].axhline(-5.0, color="#d32f2f", linestyle=":", linewidth=1.0, alpha=0.7)
    axes[0].axhline(0.0, color="black", linestyle="-", linewidth=0.6, alpha=0.5)
    axes[0].set_ylabel("Position Error $e_x$ (mm)", fontsize=11, fontweight="bold")
    axes[0].set_title("Physical Payload Robustness Verification (P0 - P4, Gazebo Dynamics)", fontsize=13, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.6)
    axes[0].legend(loc="best", fontsize=9, framealpha=0.9)

    axes[1].axhline(0.0, color="black", linestyle="-", linewidth=0.6, alpha=0.5)
    axes[1].set_ylabel(r"$\hat{\Delta y}_c$ (mm)", fontsize=11, fontweight="bold")
    axes[1].set_title("Online Adaptive Equilibrium Offset Compensation", fontsize=11, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.6)
    axes[1].legend(loc="best", fontsize=9, framealpha=0.9)

    axes[2].axhline(0.0, color="black", linestyle="-", linewidth=0.6, alpha=0.5)
    axes[2].set_ylabel("Torque $u_{\\rm model}$ (Nm)", fontsize=11, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.6)
    axes[2].legend(loc="best", fontsize=9, framealpha=0.9)

    axes[3].axhline(0.0, color="black", linestyle="-", linewidth=0.6, alpha=0.5)
    axes[3].set_ylabel(r"Attitude Error $e_\theta = \theta - \hat{\theta}_{\rm eq}$ (deg)", fontsize=11, fontweight="bold")
    axes[3].set_xlabel("Relative Simulation Time from Reset (s)", fontsize=12, fontweight="bold")
    axes[3].grid(True, linestyle=":", alpha=0.6)
    axes[3].legend(loc="best", fontsize=9, framealpha=0.9)

    plt.tight_layout()
    comp_plot = os.path.join(DATA_DIR, "payload_campaign_comparison.png")
    plt.savefig(comp_plot, dpi=200)
    plt.close()
    print(f"[Payload Campaign] Saved comparison plot to {comp_plot}")

    if os.path.exists(ARTIFACT_DIR):
        artifact_plot = os.path.join(ARTIFACT_DIR, "payload_campaign_comparison.png")
        shutil.copy(comp_plot, artifact_plot)
        print(f"[Payload Campaign] Copied comparison plot to artifact dir: {artifact_plot}")

    # ----------------------------------------------------
    # Print and Format Summary Table
    # ----------------------------------------------------
    print(f"\n{'='*115}")
    print(f"  REAL PHYSICAL PAYLOAD EXPERIMENT SUMMARY (P0 - P4)")
    print(f"{'='*115}")
    header = f"{'ID':<4} | {'Description':<26} | {'T_est (s)':<9} | {'|e_x|_max (mm)':<14} | {'T_c (s)':<8} | {'e_x,end (mm)':<16} | {'max |e_theta|':<13} | {'max |u| (Nm)':<12} | {'Final hat (mm)':<14}"
    print(header)
    print("-" * len(header))

    md_table = []
    md_table.append("| 编号 | 附加载荷工况 | 控制方法 | 首次粗捕 $T_{\\rm est}$ | 最大位置漂移 $|e_x|_{\\max}$ | 总收敛 $T_c$ | 末端平均位置误差 $e_{x,{\\rm end}}$ (末15s) | 最大姿态偏差 $\\max|e_\\theta|$ | 峰值力矩 $\\max|u|$ | 最终等效补偿 $\\hat{\\Delta y}_c$ |")
    md_table.append("| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for t_info in trials:
        t_id = t_info["id"]
        res = results.get(t_id, {})
        m = res.get("metrics", {})
        if not m:
            continue
        t_est_str = f"{m['t_est']:.2f} s" if m.get("t_est") is not None else "—"
        max_drift_str = f"{m.get('max_x_err_mag', 0.0):.2f} mm"
        tc_str = f"{m['tc']:.2f} s" if m.get("tc") is not None else "— (未补偿)" if t_info["mode"] == "nominal" else "DNF"
        ss_str = f"{m.get('ss_x_err_mean', 0.0):+.2f} ± {m.get('ss_x_err_std', 0.0):.2f} mm"
        hat_str = f"{m.get('final_hat', 0.0):+.3f} mm"
        max_theta_str = f"{m.get('max_theta_err', 0.0):.2f}°"
        max_u_str = f"{m.get('max_u', 0.0):.2f} Nm"
        ctrl_method = "Nominal GS-LQR" if t_info["mode"] == "nominal" else "Adaptive GS-LQR"

        row_str = f"{t_id:<4} | {t_info['name']:<26} | {t_est_str:<9} | {max_drift_str:<14} | {tc_str:<8} | {ss_str:<16} | {max_theta_str:<13} | {max_u_str:<12} | {hat_str:<14}"
        print(row_str)

        md_table.append(f"| **{t_id}** | {t_info['name']} | {ctrl_method} | {t_est_str} | {max_drift_str} | {tc_str} | {ss_str} | {max_theta_str} | {max_u_str} | {hat_str} |")

    print(f"{'='*115}\n")

    summary_file = os.path.join(DATA_DIR, "payload_campaign_summary.md")
    with open(summary_file, "w") as f:
        f.write("# Real Physical Payload Experiment (P0 - P4) Results\n\n")
        f.write("\n".join(md_table))
        f.write("\n")
    print(f"[Payload Campaign] Saved summary table to {summary_file}")


if __name__ == "__main__":
    execute_payload_campaign(rerun_nominal_only=True)

