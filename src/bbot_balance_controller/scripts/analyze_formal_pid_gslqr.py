#!/usr/bin/env python3
"""Recompute formal Sec. 4.2 metrics, table, and compact vector figures.

All inputs are raw formal CSV files plus their pulse JSON sidecars.  The
analysis intentionally excludes the exploratory PID and old GS-LQR folders.
The six push groups report mean and unbiased sample standard deviation (N=3);
no significance test is performed.
"""

import argparse
import csv
import json
import math
import re
import shutil
import statistics
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(SCRIPT_DIR))
from run_formal_pid_gslqr_campaign import (  # noqa: E402
    arrays,
    continuous_time,
    read_rows,
    response_metrics,
    pulse_state_valid,
    PULSE_DURATION_TOLERANCE,
    RECOVERY_HOLD_S,
    RESPONSE_WINDOW_S,
)


BLUE = "#3569B9"
RED = "#D64A3A"
GRID = "#D8DEE7"


def parse_tag(tag):
    if "_constant_h" in tag:
        match = re.search(r"(?:^|_)(pid|gs_lqr)_constant_h([0-9.]+)_rep([0-9]+)$", tag)
        return {"job": "constant", "height": float(match.group(2)),
                "force": 0.0, "rep": int(match.group(3))}
    if "_lift_rep" in tag:
        return {"job": "lift", "height": 0.30, "force": 0.0,
                "rep": int(tag.rsplit("_rep", 1)[1])}
    match = re.search(r"(?:^|_)(pid|gs_lqr)_push_h([0-9.]+)_f([+-][0-9]+)_rep([0-9]+)$", tag)
    return {"job": "push", "height": float(match.group(2)),
            "force": float(match.group(3)), "rep": int(match.group(4))}


def raw_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def corrected_results(root):
    old = json.loads((root / "formal_results.json").read_text())
    corrected = []
    for old_row in old:
        path = Path(old_row["csv"])
        trial = parse_tag(old_row["trial_tag"])
        pulse = None
        pulse_path = Path(str(path) + ".pulse.json")
        if pulse_path.exists():
            pulse = json.loads(pulse_path.read_text())
        row = dict(old_row)
        row.update(response_metrics(path, old_row["controller"], trial, pulse))
        row["pulse_protocol_valid"] = (
            True if trial["job"] != "push" else pulse_state_valid(pulse, 0.20))
        row["position_target_lock_once_ok"] = bool(
            row.get("position_target_lock_once_ok", False))
        row["formal_sample_valid"] = bool(
            row.get("protocol_valid") and row["position_target_lock_once_ok"] and
            (trial["job"] != "push" or row.get("pulse_protocol_valid")))
        row["analysis_definition"] = (
            "raw position; push baseline=mean(t0-2s,t0); peaks=t0..t0+1.5s; "
            "recovery from t1, thresholds held 2s; pitch=pitch-theta_eq_nominal")
        corrected.append(row)
    return corrected


def write_dict_csv(rows, path):
    fields = sorted({key for row in rows for key in row
                     if not isinstance(row[key], (dict, list))})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in fields} for row in rows)


def mean_sd(values):
    finite = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not finite:
        return None, None, 0
    return statistics.mean(finite), statistics.stdev(finite) if len(finite) >= 2 else None, len(finite)


def push_table(rows, out_dir):
    metrics = (
        ("position_peak_relative_mm", "position_peak_mm"),
        ("pitch_peak_relative_deg", "pitch_peak_deg"),
        ("torque_peak_postpulse_1p5s_nm", "torque_peak_nm"),
        ("torque_rms_postpulse_1p5s_nm", "torque_rms_nm"),
        ("velocity_peak_mps", "velocity_peak_mps"),
        ("attitude_velocity_recovery_s", "attitude_velocity_recovery_s"),
        ("position_10mm_recovery_s", "position_10mm_recovery_s"),
        ("initial_velocity_change_mps", "initial_velocity_change_mps"),
        ("position_saturation_ratio_response", "torque_saturation_ratio"),
    )
    table = []
    for height in (0.30, 0.40, 0.50):
        for force in (20.0, -20.0):
            row = {"height_m": height, "force_N": force, "N": 3,
                   "definition": "mean +/- unbiased sample SD; push baseline t0-2s raw position"}
            for controller in ("position_pid", "gs_lqr"):
                group = [x for x in rows if x["controller"] == controller and
                         parse_tag(x["trial_tag"])["job"] == "push" and
                         abs(parse_tag(x["trial_tag"])["height"] - height) < 1e-9 and
                         parse_tag(x["trial_tag"])["force"] == force]
                for source, label in metrics:
                    values = [g.get(source) for g in group]
                    avg, sd, n = mean_sd(values)
                    prefix = "pid" if controller == "position_pid" else "gs_lqr"
                    row[f"{prefix}_{label}_mean"] = avg
                    row[f"{prefix}_{label}_sd_unbiased"] = sd
                    row[f"{prefix}_{label}_n_finite"] = n
                row[f"{('pid' if controller == 'position_pid' else 'gs_lqr')}_position_not_returned_n"] = sum(
                    not math.isfinite(float(g.get("position_10mm_recovery_s", "nan"))) for g in group)
            table.append(row)
    write_dict_csv(table, out_dir / "push_summary_mean_sd.csv")
    (out_dir / "push_summary_mean_sd.json").write_text(json.dumps(table, indent=2, default=str))
    return table


def static_lift_table(rows, out_dir):
    metrics = (
        ("position_peak_relative_mm", "position_peak_mm"),
        ("pitch_peak_relative_deg", "pitch_peak_deg"),
        ("torque_peak_postpulse_1p5s_nm", "torque_peak_nm"),
        ("steady_pitch_rms_deg", "steady_pitch_rms_deg"),
        ("steady_velocity_rms_mps", "steady_velocity_rms_mps"),
    )
    table = []
    for job, height in (("constant", 0.30), ("constant", 0.40),
                        ("constant", 0.50), ("lift", 0.30)):
        for controller in ("position_pid", "gs_lqr"):
            group = [row for row in rows if row["controller"] == controller and
                     parse_tag(row["trial_tag"])["job"] == job and
                     (job != "constant" or abs(parse_tag(row["trial_tag"])["height"] - height) < 1e-9)]
            if not group:
                continue
            item = {"job": job, "height_m": height, "controller": controller, "N": len(group)}
            for source, label in metrics:
                avg, sd, n = mean_sd([row.get(source) for row in group])
                item[f"{label}_mean"] = avg
                item[f"{label}_sd_unbiased"] = sd
                item[f"{label}_n_finite"] = n
            table.append(item)
    write_dict_csv(table, out_dir / "static_lift_summary.csv")
    return table


def read_curve(path, controller, push=False):
    data = arrays(path, controller)
    t = data["time"]
    pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    if push:
        pulse = json.loads(Path(str(path) + ".pulse.json").read_text())
        t0 = float(pulse["pulse_start_sim_time"])
        baseline_mask = (t >= t0 - 2.0) & (t < t0)
        baseline = float(np.mean(data["p"][baseline_mask]))
        return t - t0, (data["p"] - baseline) * 1000.0, pitch, t0
    return t - 6.74, data["p"], pitch, 6.74


def interp_stats(curves, grid):
    values = []
    for time, value in curves:
        mask = (time >= grid[0]) & (time <= grid[-1])
        values.append(np.interp(grid, time[mask], value[mask], left=np.nan, right=np.nan))
    array = np.asarray(values)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(array, axis=0), np.nanstd(array, axis=0, ddof=1)


def style():
    plt.rcParams.update({
        "font.family": ["Liberation Serif", "DejaVu Serif"],
        "font.size": 8.5, "axes.labelsize": 9, "axes.linewidth": 0.8,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
        "legend.fontsize": 8, "svg.fonttype": "path", "pdf.fonttype": 42,
    })


def draw_lift(rows, root, out_dir):
    style()
    grid = np.linspace(0.0, 31.0, 621)
    fig, axes = plt.subplots(3, 1, figsize=(3.7, 6.6), sharex=True)
    for controller, color, label in (("gs_lqr", BLUE, "GS-LQR"),
                                     ("position_pid", RED, "Four-loop PID")):
        paths = [Path(row["csv"]) for row in rows
                 if row["controller"] == controller and row["trial_tag"].find("_lift_") >= 0]
        height_curves, position_curves, pitch_curves = [], [], []
        for path in paths:
            data = arrays(path, controller)
            t = data["time"] - 6.74
            height_name = "height" if controller == "position_pid" else "hip_axle_height"
            with path.open(newline="", encoding="utf-8") as handle:
                raw = list(csv.DictReader(handle))
            height = np.asarray([float(row[height_name]) for row in raw[:len(t)]])
            pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
            h0 = float(np.median(height[(t >= 0.0) & (t <= 2.0)]))
            rise = np.flatnonzero((t >= 0.0) & (height > h0 + 0.01))
            rise_time = float(t[rise[0]]) if rise.size else 6.0
            baseline_mask = (t >= rise_time - 2.0) & (t <= rise_time)
            if not np.any(baseline_mask):
                baseline_mask = (t >= 0.0) & (t <= 2.0)
            position = (data["p"] - float(np.mean(data["p"][baseline_mask]))) * 1000.0
            height_curves.append((t, height))
            position_curves.append((t, position))
            pitch_curves.append((t, pitch))
        if not height_curves:
            continue
        h_mean, h_sd = interp_stats(height_curves, grid)
        position_mean, position_sd = interp_stats(position_curves, grid)
        p_mean, p_sd = interp_stats(pitch_curves, grid)
        axes[0].plot(grid, h_mean, color=color, label=label)
        axes[0].fill_between(grid, h_mean-h_sd, h_mean+h_sd, color=color, alpha=0.16)
        axes[1].plot(grid, position_mean, color=color, label=label)
        axes[1].fill_between(grid, position_mean-position_sd, position_mean+position_sd,
                             color=color, alpha=0.16)
        axes[2].plot(grid, p_mean, color=color, label=label)
        axes[2].fill_between(grid, p_mean-p_sd, p_mean+p_sd, color=color, alpha=0.16)
    axes[0].set_ylabel("Hip-axle height (m)")
    axes[1].set_ylabel("Position from pre-rise mean (mm)")
    axes[2].set_ylabel("Pitch error (deg)")
    axes[2].set_xlabel("Time after reset command (s)")
    for ax in axes:
        ax.grid(color=GRID, linestyle=":", linewidth=0.5)
        ax.legend(frameon=False, loc="best")
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out_dir / f"fig42_formal_lift_response.{ext}", dpi=600, bbox_inches="tight")
    plt.close(fig)


def draw_low_push(rows, out_dir):
    style()
    grid = np.linspace(-2.0, 25.0, 541)
    fig, axes = plt.subplots(1, 2, figsize=(6.8, 2.6), sharey=True)
    for ax, force in zip(axes, (20.0, -20.0)):
        for controller, color, label in (("gs_lqr", BLUE, "GS-LQR"),
                                         ("position_pid", RED, "Four-loop PID")):
            curves = []
            for row in rows:
                tag = parse_tag(row["trial_tag"])
                if row["controller"] != controller or tag["job"] != "push" or \
                        abs(tag["height"] - 0.30) > 1e-9 or tag["force"] != force:
                    continue
                t, pos, _, _ = read_curve(Path(row["csv"]), controller, push=True)
                curves.append((t, pos))
            if not curves:
                continue
            mean, sd = interp_stats(curves, grid)
            ax.plot(grid, mean, color=color, lw=1.5, label=label)
            ax.fill_between(grid, mean-sd, mean+sd, color=color, alpha=0.17)
        ax.axvspan(0.0, 0.20, color="#B9C4D5", alpha=0.35, zorder=0)
        ax.axhline(0.0, color="#5C6670", linewidth=0.7)
        ax.grid(color=GRID, linestyle=":", linewidth=0.5)
        ax.set_title(f"H=0.30 m, {force:+.0f} N")
        ax.set_xlabel("Time from pulse start (s)")
    axes[0].set_ylabel("Displacement from pre-pulse mean (mm)")
    axes[0].legend(frameon=False, loc="upper right")
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out_dir / f"fig42_formal_low_push.{ext}", dpi=600, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    root = Path(args.root)
    out_dir = root / "analysis"
    out_dir.mkdir(exist_ok=True)
    rows = corrected_results(root)
    write_dict_csv(rows, out_dir / "formal_results_corrected.csv")
    (out_dir / "formal_results_corrected.json").write_text(json.dumps(rows, indent=2, default=str))
    table = push_table(rows, out_dir)
    static_lift_table(rows, out_dir)
    draw_lift(rows, root, out_dir)
    draw_low_push(rows, out_dir)
    definitions = """Formal Sec. 4.2 metric definitions

    position_peak_relative_mm: raw position displacement from the mean raw
      position over [pulse_start-2 s, pulse_start], evaluated over
      [pulse_start, pulse_start+1.5 s].
    pitch_peak_relative_deg: |pitch - theta_eq_nominal| over the same 1.5 s
      response window.
    torque_peak_postpulse_1p5s_nm: |total wheel torque| over the same window.
    attitude_velocity_recovery_s: first time after pulse end where |pitch
      error| <= 0.5 deg and |velocity| <= 0.02 m/s continuously for 2 s.
    position_10mm_recovery_s: first time after pulse end where displacement
      from the pre-pulse position mean is <= 10 mm continuously for 2 s.
    lift plot position: raw position relative to the mean over the 2 s before
      the detected first height rise (>10 mm above the initial low-height mean).
    push_summary_mean_sd.csv uses N=3 and unbiased sample standard deviation;
    no significance test is performed. NaN recovery means the target band was
    not returned to within the available trial window.
    """
    (out_dir / "metric_definitions.txt").write_text(definitions)
    print(f"Corrected results: {len(rows)}; push groups: {len(table)}; output: {out_dir}")


if __name__ == "__main__":
    main()
