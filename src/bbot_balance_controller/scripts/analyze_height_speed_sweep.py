#!/usr/bin/env python3
"""Analyse gated continuous-height trials across multiple transition speeds.

The script keeps every attempted trial in the protocol table, while only
completed trials with an event record contribute to response statistics.  Old
0.05 m/s recheck roots may be supplied as historical references; they are
labelled separately and never merged with the current speed-sweep batch.
"""

import argparse
import csv
import json
import math
import re
import statistics
import sys
import warnings
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# The manuscript uses Times New Roman.  Keep Liberation Serif as a metrically
# compatible fallback for headless Linux environments where the licensed font
# is not installed.
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Liberation Serif"],
    "mathtext.fontset": "stix",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from run_formal_pid_gslqr_campaign import arrays, read_rows  # noqa: E402

LOW_HEIGHT_M = 0.30
HIGH_HEIGHT_M = 0.50
HEIGHT_TOL_M = 0.005
SATURATION_NM = 19.5
COLORS = {"gs_lqr": "#3569B9", "position_pid": "#D64A3A"}
LABELS = {"gs_lqr": "GS-LQR", "position_pid": "Four-loop PID"}
RESPONSE_LINESTYLES = {"gs_lqr": "-", "position_pid": "--"}


def tag_info(tag):
    match = re.search(r"(?:^|_)(pid|gs_lqr)_lift_v([0-9.]+)_rep([0-9]+)$", tag)
    if match:
        return {
            "controller": "position_pid" if match.group(1) == "pid" else "gs_lqr",
            "speed_mps": float(match.group(2)), "rep": int(match.group(3)),
        }
    match = re.search(r"(?:^|_)(pid|gs_lqr)_lift_rep([0-9]+)$", tag)
    if match:
        return {
            "controller": "position_pid" if match.group(1) == "pid" else "gs_lqr",
            "speed_mps": 0.05, "rep": int(match.group(2)),
        }
    raise ValueError(f"unrecognised lift trial tag: {tag}")


def csv_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def height_data(path, controller):
    raw = csv_rows(path)
    data = arrays(Path(path), controller)
    key = "height" if controller == "position_pid" else "hip_axle_height"
    height = np.asarray([float(row[key]) for row in raw[:len(data["time"])]])
    return data, height


def target_lock(path, controller):
    raw = csv_rows(path)
    t = np.asarray([float(row["time"]) for row in raw])
    if controller == "position_pid":
        latch = np.asarray([float(row.get("position_target_latched", 0.0)) > 0.5
                            for row in raw])
        if not np.any(latch):
            return math.nan, 0
        return (float(t[np.flatnonzero(latch)[0]]),
                int(max(float(row.get("position_latch_count", 0.0)) for row in raw)))
    target = np.asarray([float(row["x_ref"]) for row in raw])
    changes = np.where(np.abs(target - target[0]) > 1e-4)[0]
    if not len(changes):
        return math.nan, 0
    return float(t[changes[0]]), int(np.ptp(target[changes[0]:]) <= 1e-8)


def fit_slope(t, h, start, stop, increasing):
    inner = ((h >= LOW_HEIGHT_M + 0.02) & (h <= HIGH_HEIGHT_M - 0.02) &
             (t >= start) & (t <= stop))
    if np.count_nonzero(inner) < 10:
        return math.nan
    slope = float(np.polyfit(t[inner], h[inner], 1)[0])
    if (increasing and slope <= 0.0) or (not increasing and slope >= 0.0):
        return math.nan
    return slope


def completed_metric(result, source_root, batch):
    info = tag_info(result["trial_tag"])
    controller = info["controller"]
    csv_path = Path(result["csv"])
    event_path = source_root / controller / f"{csv_path.stem}.events.json"
    event = json.loads(event_path.read_text(encoding="utf-8"))
    data, height = height_data(csv_path, controller)
    t = data["time"]
    rise = float(event["actual_rise_start_sim_time"])
    high = float(event["high_reached_sim_time"])
    down = float(event["lift_command_down_sim_time"])
    low = float(event["actual_low_return_sim_time"])
    end = min(float(event["observation_end_sim_time"]), float(t[-1]))
    pre = (t >= rise - 2.0) & (t < rise)
    window = (t >= rise) & (t <= end)
    terminal = t >= max(rise, end - 10.0)
    if not np.any(window):
        raise ValueError("empty lift response window")
    baseline = float(np.mean(data["p"][pre])) if np.any(pre) else float(data["p"][0])
    pitch_deg = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    torque = np.abs(data["torque"])
    requested = float(result.get("lift_commanded_speed_mps") or
                      event.get("commanded_height_speed_mps") or info["speed_mps"])
    rise_slope = fit_slope(t, height, rise, high, increasing=True)
    fall_slope = fit_slope(t, height, down, low, increasing=False)
    rise_speed = rise_slope
    fall_speed = -fall_slope if math.isfinite(fall_slope) else math.nan
    speed_ok = (math.isfinite(rise_speed) and math.isfinite(fall_speed) and
                abs(rise_speed - requested) / requested <= 0.02 and
                abs(fall_speed - requested) / requested <= 0.02)
    lock_time, latch_count = target_lock(csv_path, controller)
    return {
        "batch": batch, "controller": controller, "trial_tag": result["trial_tag"],
        "rep": info["rep"], "commanded_speed_mps": requested,
        "csv": str(csv_path), "events": str(event_path),
        "protocol_valid": bool(result.get("protocol_valid")),
        "static_gate_passed": bool(event["static_gate"].get("passed")),
        "completed": bool(event.get("completed")),
        "target_lock_time_s": lock_time,
        "position_latch_count": latch_count,
        "target_lock_once_ok": bool(latch_count == 1),
        "actual_rise_start_sim_time": rise,
        "lift_command_up_sim_time": float(event["lift_command_up_sim_time"]),
        "actual_low_return_sim_time": low,
        "observation_end_sim_time": end,
        "actual_rise_speed_mps": rise_speed,
        "actual_descend_speed_mps": fall_speed,
        "rise_speed_error_percent": 100.0 * (rise_speed - requested) / requested
            if math.isfinite(rise_speed) else math.nan,
        "descend_speed_error_percent": 100.0 * (fall_speed - requested) / requested
            if math.isfinite(fall_speed) else math.nan,
        "speed_within_2pct": speed_ok,
        "height_min_m": float(np.min(height[window])),
        "height_max_m": float(np.max(height[window])),
        "height_coverage_ok": bool(np.min(height[window]) <= LOW_HEIGHT_M + HEIGHT_TOL_M and
                                   np.max(height[window]) >= HIGH_HEIGHT_M - HEIGHT_TOL_M),
        "position_peak_relative_pre_rise_mm": float(np.max(np.abs(
            (data["p"][window] - baseline) * 1000.0))),
        "pitch_peak_relative_nominal_deg": float(np.max(np.abs(pitch_deg[window]))),
        "velocity_peak_mps": float(np.max(np.abs(data["velocity"][window]))),
        "torque_peak_nm": float(np.max(torque[window])),
        "torque_saturation_ratio": float(np.mean(torque[window] >= SATURATION_NM)),
        "final_position_residual_mm": float(np.mean(
            (data["p"][terminal] - baseline) * 1000.0)),
        "pre_rise_position_mean_m": baseline,
    }


def protocol_record(result, source_root, batch):
    info = tag_info(result["trial_tag"])
    controller_root = source_root / info["controller"]
    log_path = controller_root / "launch_logs" / (
        f"{result['trial_tag']}_attempt{result.get('attempt', 1)}.log")
    try:
        log_text = log_path.read_text(encoding="utf-8", errors="ignore").lower()
    except OSError:
        log_text = ""
    fail = str(result.get("fail_reason") or "")
    return {
        "batch": batch, "controller": info["controller"], "trial_tag": result["trial_tag"],
        "rep": info["rep"],
        "commanded_speed_mps": float(result.get("lift_commanded_speed_mps") or info["speed_mps"]),
        "protocol_valid": bool(result.get("protocol_valid")),
        "static_gate_passed": bool(result.get("lift_static_gate_valid")),
        "completed": bool(result.get("lift_completed")),
        "retryable_protocol_error": bool(result.get("retryable_protocol_error")),
        "recording_protocol_error": bool(result.get("recording_protocol_error")),
        "controller_protection_failure": ("controller disabled" in log_text or
                                          "pitch error" in log_text),
        "fail_reason": fail,
        "csv": str(result.get("csv", "")),
    }


def finite(values):
    return [float(value) for value in values
            if value is not None and math.isfinite(float(value))]


def mean_sd(values):
    values = finite(values)
    if not values:
        return math.nan, math.nan, 0
    return (statistics.mean(values),
            statistics.stdev(values) if len(values) >= 2 else math.nan,
            len(values))


def write_csv(path, rows):
    fields = sorted({key for row in rows for key in row})
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def summarise(metrics, attempts):
    groups = defaultdict(list)
    attempt_groups = defaultdict(list)
    for row in metrics:
        groups[(row["batch"], row["controller"], row["commanded_speed_mps"])].append(row)
    for row in attempts:
        attempt_groups[(row["batch"], row["controller"], row["commanded_speed_mps"])].append(row)
    numeric = (
        "actual_rise_speed_mps", "actual_descend_speed_mps", "rise_speed_error_percent",
        "descend_speed_error_percent", "position_peak_relative_pre_rise_mm",
        "pitch_peak_relative_nominal_deg", "velocity_peak_mps", "torque_peak_nm",
        "torque_saturation_ratio", "final_position_residual_mm",
    )
    rows = []
    for key in sorted(set(groups) | set(attempt_groups)):
        batch, controller, speed = key
        values, attempts_here = groups[key], attempt_groups[key]
        valid_attempts = [item for item in attempts_here if item["protocol_valid"]]
        physical_failures = [item for item in valid_attempts if not item["completed"]]
        row = {"batch": batch, "controller": controller, "commanded_speed_mps": speed,
               "N_attempted": len(attempts_here), "N_completed_analyzed": len(values),
               "N_protocol_valid": len(valid_attempts),
               "N_protocol_invalid": len(attempts_here) - len(valid_attempts),
               "N_failures": len(physical_failures),
               "failure_rate": (len(physical_failures) / len(valid_attempts))
                               if valid_attempts else math.nan}
        for name in numeric:
            avg, sd, count = mean_sd([item.get(name) for item in values])
            row[f"{name}_mean"] = avg
            row[f"{name}_sd_unbiased"] = sd
            row[f"{name}_n_finite"] = count
        for name in ("speed_within_2pct", "height_coverage_ok", "target_lock_once_ok",
                     "static_gate_passed"):
            row[f"{name}_count"] = sum(bool(item.get(name)) for item in values)
        rows.append(row)
    return rows


def interp(curves, grid):
    values = []
    for t, y in curves:
        usable = (t >= grid[0]) & (t <= grid[-1])
        if np.count_nonzero(usable) >= 2:
            values.append(np.interp(grid, t[usable], y[usable], left=np.nan, right=np.nan))
    if not values:
        return np.full_like(grid, np.nan), np.full_like(grid, np.nan)
    data = np.asarray(values)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(data, axis=0), np.nanstd(data, axis=0, ddof=1)


def save_figure(fig, out, stem):
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out / f"{stem}.{ext}", dpi=600, bbox_inches="tight")
    plt.close(fig)


def draw_response(metrics, speed, out):
    selected = [row for row in metrics
                if abs(row["commanded_speed_mps"] - speed) < 1e-9 and row["batch"] == "current"]
    if not selected:
        return
    # Time zero is the issued ascent command, rather than the first sample
    # that crosses the rise detector.  This retains the required 0.30 m
    # initial state in every speed panel.
    end = 10.0 + 0.20 / speed + 10.0
    grid = np.linspace(0.0, end, max(401, int(end * 100) + 1))
    fig, axes = plt.subplots(3, 1, figsize=(3.25, 5.15), sharex=True)
    for controller in ("gs_lqr", "position_pid"):
        h_curves, p_curves, theta_curves = [], [], []
        for row in selected:
            if row["controller"] != controller:
                continue
            data, height = height_data(Path(row["csv"]), controller)
            t = data["time"] - row["lift_command_up_sim_time"]
            pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
            h_curves.append((t, height))
            p_curves.append((t, (data["p"] - row["pre_rise_position_mean_m"]) * 1000.0))
            theta_curves.append((t, pitch))
        for axis, curves in zip(axes, (h_curves, p_curves, theta_curves)):
            mean, sd = interp(curves, grid)
            axis.plot(grid, mean, color=COLORS[controller],
                      linestyle=RESPONSE_LINESTYLES[controller],
                      label=LABELS[controller])
    axes[0].set_ylabel("Hip-axle height (m)")
    axes[1].set_ylabel("Position from pre-rise mean (mm)")
    axes[2].set_ylabel("Pitch error (deg)")
    axes[2].set_xlabel("Time from ascent command (s)")
    axes[0].set_ylim(0.29, 0.51)
    for axis in axes:
        axis.grid(color="#D8DEE7", linestyle=":", linewidth=0.5)
        axis.legend(frameon=False, loc="best", fontsize=7)
        axis.tick_params(labelsize=7)
        axis.yaxis.label.set_size(8)
    axes[2].xaxis.label.set_size(8)
    fig.tight_layout()
    save_figure(fig, out, f"fig42_height_speed_v{speed * 100:03.0f}")


def draw_speed_summary(summary_rows, out):
    metrics = (
        ("position_peak_relative_pre_rise_mm_mean", "Peak position deviation (mm)"),
        ("pitch_peak_relative_nominal_deg_mean", "Peak pitch error (deg)"),
        ("torque_peak_nm_mean", "Peak wheel torque (Nm)"),
        ("failure_rate", "Failure rate"),
    )
    fig, axes = plt.subplots(2, 2, figsize=(7.3, 5.0), sharex=True)
    for controller in ("gs_lqr", "position_pid"):
        for batch, style, marker in (("current", "-", "o"), ("historical", "--", "s")):
            data = sorted([row for row in summary_rows
                           if row["controller"] == controller and row["batch"] == batch],
                          key=lambda row: row["commanded_speed_mps"])
            if not data:
                continue
            x = np.asarray([row["commanded_speed_mps"] for row in data])
            for axis, (key, label) in zip(axes.flat, metrics):
                y = np.asarray([row.get(key, math.nan) for row in data])
                axis.plot(x, y, linestyle=style, marker=marker, color=COLORS[controller],
                          label=f"{LABELS[controller]} ({batch})")
                axis.set_ylabel(label)
                axis.grid(color="#D8DEE7", linestyle=":", linewidth=0.5)
    for axis in axes[-1]:
        axis.set_xlabel("Commanded height speed (m/s)")
    axes[0, 0].legend(frameon=False, fontsize=7.5)
    fig.tight_layout()
    save_figure(fig, out, "fig42_height_speed_sweep_summary")


def load_root(source_root, batch, attempts, metrics):
    results = json.loads((source_root / "formal_results.json").read_text(encoding="utf-8"))
    for result in results:
        if "_lift_" not in result.get("trial_tag", ""):
            continue
        try:
            tag_info(result["trial_tag"])
        except ValueError:
            continue
        attempt = protocol_record(result, source_root, batch)
        attempts.append(attempt)
        csv_path = Path(result.get("csv", ""))
        event_path = source_root / attempt["controller"] / f"{csv_path.stem}.events.json"
        if attempt["completed"] and event_path.exists() and csv_path.exists():
            try:
                metrics.append(completed_metric(result, source_root, batch))
            except (KeyError, OSError, ValueError, TypeError) as exc:
                attempt["analysis_error"] = str(exc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--historical-root", action="append", type=Path, default=[])
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metrics, attempts = [], []
    load_root(args.campaign_root, "current", attempts, metrics)
    for root in args.historical_root:
        load_root(root, "historical", attempts, metrics)
    write_csv(args.output / "height_speed_protocol_attempts.csv", attempts)
    write_csv(args.output / "height_speed_trial_metrics.csv", metrics)
    summary_rows = summarise(metrics, attempts)
    write_csv(args.output / "height_speed_summary.csv", summary_rows)
    for speed in (0.10, 0.15, 0.20):
        draw_response(metrics, speed, args.output)
    draw_speed_summary(summary_rows, args.output)
    audit = {
        "current_campaign_root": str(args.campaign_root),
        "historical_reference_roots": [str(root) for root in args.historical_root],
        "attempt_count": len(attempts),
        "analyzed_completed_count": len(metrics),
        "definitions": {
            "speed_fit": "least-squares height slope over the central 0.32-0.48 m segment",
            "speed_acceptance": "both rise and descent fitted magnitudes within 2% of the commanded speed",
            "position_baseline": "mean raw position over 2 s before actual rise start",
            "response_window": "actual rise start through 10 s after actual low return",
            "failure_rate": "non-completed protocol-valid trials divided by protocol-valid trials; protocol-invalid attempts are reported separately",
            "historical_reference": "0.05 m/s data are plotted separately and not pooled with current trials",
        },
        "summary": summary_rows,
    }
    (args.output / "height_speed_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Analysed {len(metrics)} completed trials from {len(attempts)} attempts: {args.output}")


if __name__ == "__main__":
    main()
