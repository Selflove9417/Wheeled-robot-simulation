#!/usr/bin/env python3
"""
Figure generator for the gain-scheduling study (paper Sec. 4.2).

Produces, with unified English labels, line widths and fonts, and each figure
exported as SVG + PDF + PNG:
  fig4  spectral-radius sweep           (from MATLAB rho_sweep.csv)
  fig5  lift transition, 3-panel compare (from lift trial CSVs)
  fig6  push-recovery curves            (from push trial CSVs)
  table3/table4 markdown snippets       (from campaign_summary.csv)

Usage:
  python3 plot_height_campaign.py \
      --campaign-dir .../data_logs/height_campaign \
      --rho-csv .../动力学建模/rho_sweep.csv \
      --out-dir .../data_logs/height_campaign/figures
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLOR = {"scheduled": "#1565c0", "fixed_midpoint": "#d32f2f"}
LABEL = {"scheduled": "Scheduled (interpolated)",
         "fixed_midpoint": "Fixed midpoint (0.40 m)"}

plt.rcParams.update({
    "font.size": 10,
    "axes.linewidth": 0.9,
    "lines.linewidth": 1.6,
    "legend.framealpha": 0.9,
    "legend.fontsize": 8.5,
})

HEIGHTS_PUSH = [0.30, 0.40, 0.50]
HEIGHT_COLORS = {0.30: "#2e7d32", 0.40: "#ef6c00", 0.50: "#6a1b9a"}
LIFT_SCHEDULE = {"rise": 5.0, "descend": 19.0, "end": 38.0}
PULSE_OFFSET = 5.0
PULSE_DURATION = 0.2
RESET_DETECT_THRESHOLD = 0.01


def load(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def col(data, index):
    return data[data.dtype.names[index]]


def reset_time(data):
    """Reset instant from the x_ref jump (column 5)."""
    t = col(data, 0)
    x_ref = col(data, 5)
    jumps = np.where(np.abs(np.diff(x_ref)) > RESET_DETECT_THRESHOLD)[0]
    return t[jumps[0] + 1] if len(jumps) > 0 else t[0]


def save_all(fig, out_dir, stem):
    for ext in ("svg", "pdf", "png"):
        fig.savefig(os.path.join(out_dir, f"{stem}.{ext}"),
                    bbox_inches="tight", dpi=300)
    print(f"  saved {stem}.svg/.pdf/.png")


def target_height_curve(t, t0):
    """Scheduled hip-axle height: 0.30 ->(0.05 m/s)-> 0.50 ->(0.05 m/s)-> 0.30."""
    target = np.full_like(t, 0.30)
    up = (t >= t0 + LIFT_SCHEDULE["rise"]) & (t < t0 + LIFT_SCHEDULE["rise"] + 4.0)
    target[up] = 0.30 + 0.05 * (t[up] - (t0 + LIFT_SCHEDULE["rise"]))
    high = (t >= t0 + LIFT_SCHEDULE["rise"] + 4.0) & (t < t0 + LIFT_SCHEDULE["descend"])
    target[high] = 0.50
    down = (t >= t0 + LIFT_SCHEDULE["descend"]) & (t < t0 + LIFT_SCHEDULE["descend"] + 4.0)
    target[down] = 0.50 - 0.05 * (t[down] - (t0 + LIFT_SCHEDULE["descend"]))
    return target


# ---------------------------------------------------------------------------
# Fig. 4: spectral radius sweep
# ---------------------------------------------------------------------------

def plot_fig4(rho_csv, out_dir):
    data = np.genfromtxt(rho_csv, delimiter=",", names=True)
    hip = data["hip_height_m"]
    fig, ax1 = plt.subplots(figsize=(7.5, 4.4), dpi=300)

    # Left axis: Spectral radius
    line1, = ax1.plot(hip, data["rho_scheduled"], color=COLOR["scheduled"],
                      linewidth=1.8, label=r"Spectral radius $\rho$ (Scheduled)")
    line2, = ax1.plot(hip, data["rho_fixed"], color=COLOR["fixed_midpoint"],
                      linestyle="--", linewidth=1.6, label=r"Spectral radius $\rho$ (Fixed midpoint)")
    line_bound = ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.0,
                             label="Stability limit (1.0)")

    # Real design node values on scheduled curve
    node_heights = [0.30, 0.35, 0.40, 0.45, 0.50]
    node_indices = [np.argmin(np.abs(hip - nh)) for nh in node_heights]
    node_rhos = [data["rho_scheduled"][idx] for idx in node_indices]
    line_nodes, = ax1.plot(node_heights, node_rhos, "o", color="#1565c0",
                           markersize=5.5, label="Design nodes (Scheduled)")

    ax1.set_xlabel("Hip-axle height H (m)")
    ax1.set_ylabel(r"Closed-loop spectral radius $\rho$")
    ax1.set_ylim(0.99920, 1.00010)

    # Right axis: 6~12 Hz Modal damping ratio
    ax2 = ax1.twinx()
    line3, = ax2.plot(hip, data["damp_6_12_sched"], color="#2e7d32",
                      linestyle="-.", linewidth=1.8, label="Modal damping ratio (6-12 Hz, Scheduled)")
    line4, = ax2.plot(hip, data["damp_6_12_fixed"], color="#e65100",
                      linestyle=":", linewidth=1.8, label="Modal damping ratio (6-12 Hz, Fixed midpoint)")
    ax2.set_ylabel(r"Modal damping ratio $\zeta$ (6-12 Hz)")
    ax2.set_ylim(0.02, 0.11)

    ax1.set_title("Closed-loop multi-rate spectral radius and modal damping over height")
    ax1.grid(True, linestyle=":", alpha=0.6)

    # Combine legends
    lines = [line1, line2, line_nodes, line_bound, line3, line4]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc="upper center", bbox_to_anchor=(0.5, -0.15),
               ncol=3, framealpha=0.9, fontsize=8.0)

    fig.tight_layout()
    save_all(fig, out_dir, "fig4_spectral_radius_sweep")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Fig. 5: lift transition, 3-panel compare
# ---------------------------------------------------------------------------

def plot_fig5(campaign_dir, out_dir, rep=1):
    fig, axes = plt.subplots(3, 1, figsize=(7.5, 7.2), sharex=True, dpi=300)
    plotted = False
    for gm in ("scheduled", "fixed_midpoint"):
        path = os.path.join(campaign_dir, f"lift_{gm}_{rep}.csv")
        if not os.path.exists(path):
            print(f"  [Warn] missing {path}; skipping {gm} curve")
            continue
        data = load(path)
        t = col(data, 0)
        t0 = reset_time(data)
        rel = t - t0
        mask = (rel >= -1.0) & (rel <= LIFT_SCHEDULE["end"] + 2.0)
        hip = col(data, 1)
        pitch_err = col(data, 12) * 180.0 / np.pi
        x_err = col(data, 25) * 1000.0
        target = target_height_curve(t, t0)

        axes[0].plot(rel[mask], target[mask], color="black",
                     linestyle=":", linewidth=1.2)
        axes[0].plot(rel[mask], hip[mask], color=COLOR[gm], label=LABEL[gm])
        axes[1].plot(rel[mask], pitch_err[mask], color=COLOR[gm], label=LABEL[gm])
        axes[2].plot(rel[mask], x_err[mask], color=COLOR[gm], label=LABEL[gm])
        plotted = True

    if not plotted:
        plt.close(fig)
        print("  [Warn] no lift CSVs found; fig5 skipped")
        return

    axes[0].set_ylabel("Hip-axle height (m)")
    axes[0].set_title("Height transition 0.30 -> 0.50 -> 0.30 m "
                      "(dotted: target height)")
    axes[1].set_ylabel(r"Pitch tracking error $\theta-\hat{\theta}_{\rm eq}$ (deg)")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[2].axhline(0.0, color="black", linewidth=0.7)
    axes[2].set_ylabel("Position error (mm)")
    axes[2].set_xlabel("Time from position reset (s)")
    axes[2].set_xlim(-1.0, LIFT_SCHEDULE["end"] + 2.0)
    for ax in axes:
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(loc="best")
    save_all(fig, out_dir, "fig5_height_transition")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Fig. 6: push-recovery curves
# ---------------------------------------------------------------------------

def plot_fig6(campaign_dir, out_dir, rep=1, force=20):
    """Representative position recovery for both pulse directions.

    Pulse onset comes from the exact-once protocol status instead of the old
    fixed offset, because the current campaign fires only after a continuous
    five-second steady-state gate.
    """
    fig, axes = plt.subplots(2, 3, figsize=(7.5, 5.0), sharex=True,
                             sharey=True, dpi=300)
    plotted = False
    for row, signed_force in enumerate((force, -force)):
        force_tag = f"{signed_force:+g}"
        for col_idx, h in enumerate(HEIGHTS_PUSH):
            ax = axes[row, col_idx]
            for gm in ("scheduled", "fixed_midpoint"):
                path = os.path.join(
                    campaign_dir,
                    f"push_{h:g}_{gm}_f{force_tag}_{rep}.csv")
                state_path = path + ".pulse.json"
                if not os.path.exists(path) or not os.path.exists(state_path):
                    continue
                with open(state_path, encoding="utf-8") as state_file:
                    pulse_state = json.load(state_file)
                pulse_start = float(pulse_state["pulse_start_sim_time"])
                data = load(path)
                t = col(data, 0)
                rel = t - pulse_start
                mask = (rel >= -1.0) & (rel <= 28.0)
                style = "-" if gm == "scheduled" else "--"
                ax.plot(rel[mask], col(data, 25)[mask] * 1000.0,
                        color=COLOR[gm], linestyle=style, label=LABEL[gm])
                plotted = True

            ax.axvspan(0.0, PULSE_DURATION, color="#b0bec5", alpha=0.35,
                       linewidth=0)
            ax.axhspan(-10.0, 10.0, color="#80cbc4", alpha=0.16,
                       linewidth=0)
            ax.axhline(0.0, color="black", linewidth=0.6)
            ax.grid(True, linestyle=":", alpha=0.55)
            ax.set_xlim(-1.0, 28.0)
            if row == 0:
                ax.set_title(f"H = {h:.2f} m")
            if row == 1:
                ax.set_xlabel("Time from pulse onset (s)")
        axes[row, 0].set_ylabel(
            f"{signed_force:+g} N pulse\nPosition error (mm)")

    if not plotted:
        plt.close(fig)
        print("  [Warn] no push CSVs found; fig6 skipped")
        return

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2,
               bbox_to_anchor=(0.5, 1.01), frameon=False)
    fig.suptitle("Position recovery after longitudinal force pulses",
                 y=0.955, fontsize=10.5)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    save_all(fig, out_dir, "fig6_push_recovery")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Tables 3/4 markdown snippets
# ---------------------------------------------------------------------------

def write_tables(campaign_dir, out_dir):
    path = os.path.join(campaign_dir, "campaign_summary.csv")
    if not os.path.exists(path):
        print("  [Warn] campaign_summary.csv missing; tables skipped")
        return
    import csv as csvmod
    with open(path, encoding="utf-8") as f:
        rows = list(csvmod.DictReader(f))

    def _fnum(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    def get(row, key):
        v = row.get(key, "")
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    def fmt(v, digits=2, scale=1.0):
        return "—" if v is None else f"{v * scale:.{digits}f}"

    lines = ["<!-- 表3 草稿（由 plot_height_campaign.py 生成，数值为 3 次运行均值） -->",
             "| 髋轴高 H | 增益模式 | 稳态位置误差 (mm) | 俯仰误差 RMS (deg) | 峰值总力矩 (Nm) | RMS 总力矩 (Nm) |",
             "|---|---|---|---|---|---|"]
    for row in rows:
        if row["job"] != "constant":
            continue
        lines.append(
            f"| {float(row['height']):.2f} m | {row['gain_mode']} "
            f"| {fmt(get(row, 'ss_x_err_mean_mean'))}±{fmt(get(row, 'ss_x_err_std_mean'))} "
            f"| {fmt(get(row, 'pitch_rms_deg_mean'))} "
            f"| {fmt(get(row, 'peak_u_mean'))} "
            f"| {fmt(get(row, 'u_rms_mean'))} |")

    lines += ["", "<!-- 表4 草稿（升降 + 推扰汇总，均值±样本标准差） -->",
              "| 工况 | 指标 | scheduled | fixed_midpoint |",
              "|---|---|---|---|"]
    lift_metrics = [("ramp_max_x_err_mm_mean", "最大位置偏移 (mm)", 1.0),
                    ("ramp_pitch_rms_deg_mean", "升降段俯仰误差 RMS (deg)", 1.0),
                    ("ramp_peak_u_mean", "升降段峰值力矩 (Nm)", 1.0)]
    push_metrics = [("max_x_err_mm_mean", "最大位置偏移 (mm)", 1.0),
                    ("max_pitch_err_deg_mean", "最大俯仰误差 (deg)", 1.0),
                    ("peak_u_mean", "峰值总力矩 (Nm)", 1.0),
                    ("recovery_s_mean", "恢复时间 (s)", 1.0),
                    ("recovery_pos_s_mean", "位置判据恢复时间 (s)", 1.0)]
    for job, metrics, label in (("lift", lift_metrics, "连续升降"),
                                ("push", push_metrics, "推扰 +20 N"),
                                ("push", push_metrics, "推扰 −20 N")):
        want = None if "−" not in label else "-20.0"
        want = "20.0" if "+" in label else want
        for key, text, scale in metrics:
            cells = []
            for gm in ("scheduled", "fixed_midpoint"):
                match = [r for r in rows if r["job"] == job
                         and r["gain_mode"] == gm
                         and (want is None
                              or _fnum(r.get("force")) == float(want))]
                if not match:
                    cells.append("—")
                    continue
                mean = np.mean([get(r, key) for r in match
                                if get(r, key) is not None])
                std = np.mean([get(r, key.replace("_mean", "_std"))
                               for r in match
                               if get(r, key.replace("_mean", "_std")) is not None])
                cells.append(f"{fmt(mean, scale=scale)}±{fmt(std, scale=scale)}")
            lines.append(f"| {label} | {text} | {cells[0]} | {cells[1]} |")

    out = os.path.join(out_dir, "table3_table4_draft.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  saved table3_table4_draft.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campaign-dir", required=True,
                    help="height_campaign data directory (trial CSVs + summary)")
    ap.add_argument("--rho-csv", required=True,
                    help="rho_sweep.csv exported by sweep_spectral_radius.m")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--rep", type=int, default=1,
                    help="which repeat to draw as the representative curve")
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.join(args.campaign_dir, "figures")
    os.makedirs(out_dir, exist_ok=True)

    print("Fig 4 (spectral radius sweep):")
    plot_fig4(args.rho_csv, out_dir)
    print("Fig 5 (lift transition):")
    plot_fig5(args.campaign_dir, out_dir, rep=args.rep)
    print("Fig 6 (push recovery):")
    plot_fig6(args.campaign_dir, out_dir, rep=args.rep)
    print("Tables:")
    write_tables(args.campaign_dir, out_dir)


if __name__ == "__main__":
    main()
