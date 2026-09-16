#!/usr/bin/env python3
"""Recompute the Sec. 4.2 height metrics with corrected event windows.

Static runs are read from the original formal campaign and are not rerun.
Lift runs are read from the gated recheck campaign and are aligned to the
detected actual rise start, not to reset or command time.
"""

import argparse
import csv
import json
import math
import re
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
from run_formal_pid_gslqr_campaign import arrays, read_rows, continuous_time  # noqa: E402

RESET_SIM_TIME = 6.74
LOW_HEIGHT = 0.30
HEIGHT_TOL = 0.005
BLUE = "#3569B9"
RED = "#D64A3A"
GRID_COLOR = "#D8DEE7"


def tag_info(tag):
    m = re.search(r"(?:^|_)(pid|gs_lqr)_constant_h([0-9.]+)_rep([0-9]+)$", tag)
    if m:
        return {"controller": "position_pid" if m.group(1) == "pid" else "gs_lqr",
                "job": "constant", "height": float(m.group(2)),
                "rep": int(m.group(3))}
    m = re.search(r"(?:^|_)(pid|gs_lqr)_lift_rep([0-9]+)$", tag)
    if m:
        return {"controller": "position_pid" if m.group(1) == "pid" else "gs_lqr",
                "job": "lift", "height": LOW_HEIGHT, "rep": int(m.group(2))}
    raise ValueError(f"unrecognised trial tag: {tag}")


def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite(values):
    return [float(x) for x in values if x is not None and math.isfinite(float(x))]


def mean_sd(values):
    vals = finite(values)
    return (statistics.mean(vals), statistics.stdev(vals) if len(vals) >= 2 else None, len(vals)) if vals else (None, None, 0)


def height_arrays(path, controller):
    raw = rows(path)
    data = arrays(Path(path), controller)
    height_key = "height" if controller == "position_pid" else "hip_axle_height"
    height = np.asarray([float(row[height_key]) for row in raw[:len(data["time"])]] )
    return data, height


def target_lock(path, controller):
    raw = rows(path)
    t = np.asarray([float(row["time"]) for row in raw])
    if controller == "position_pid":
        mask = np.asarray([float(row.get("position_target_latched", 0.0)) > 0.5 for row in raw])
        if not np.any(mask):
            return math.nan, 0
        return float(t[np.flatnonzero(mask)[0]]), int(max(float(row.get("position_latch_count", 0.0)) for row in raw))
    target = np.asarray([float(row["x_ref"]) for row in raw])
    changes = np.where((t >= RESET_SIM_TIME) & (np.abs(target - target[0]) > 1e-4))[0]
    if not len(changes):
        return math.nan, 0
    return float(t[changes[0]]), 1 if np.ptp(target[changes[0]:]) <= 1e-8 else 0


def first_continuous(t, condition, start, hold):
    for i in np.where(t >= start)[0]:
        if not condition[i]:
            continue
        j = i
        while j < len(t) and condition[j]:
            if t[j] - t[i] >= hold:
                return float(t[i])
            j += 1
    return math.nan


def static_metric(path, controller, height_target):
    data, height = height_arrays(path, controller)
    t = data["time"]
    lock_time, latch_count = target_lock(path, controller)
    target = float(np.median(data["p_target"][t >= lock_time])) if math.isfinite(lock_time) else float(data["p_target"][-1])
    height_ok = np.abs(height - height_target) <= HEIGHT_TOL
    start = first_continuous(t, height_ok, lock_time + 2.0, 2.0) if math.isfinite(lock_time) else math.nan
    metric_start = start if math.isfinite(start) else math.nan
    start_mask = t >= metric_start if math.isfinite(metric_start) else np.zeros_like(t, dtype=bool)
    actual_pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    torque_abs = np.abs(data["torque"])
    steady = t >= t[-1] - 10.0
    return {
        "controller": controller, "height_m": height_target,
        "csv": str(path), "target_lock_time_s": lock_time,
        "metric_start_time_s": metric_start, "position_latch_count": latch_count,
        "target_lock_once_ok": bool(latch_count == 1 and math.isfinite(metric_start)),
        "position_peak_relative_target_mm": float(np.max(np.abs((data["p"][start_mask] - target) * 1000.0))) if np.any(start_mask) else math.nan,
        "pitch_peak_relative_nominal_deg": float(np.max(np.abs(actual_pitch[start_mask]))) if np.any(start_mask) else math.nan,
        "torque_peak_nm": float(np.max(torque_abs[start_mask])) if np.any(start_mask) else math.nan,
        "torque_saturation_ratio": float(np.mean(torque_abs[start_mask] >= 19.5)) if np.any(start_mask) else math.nan,
        "steady_position_error_rms_mm": float(np.sqrt(np.mean(((data["p"][steady] - target) * 1000.0) ** 2))),
        "steady_pitch_rms_deg": float(np.sqrt(np.mean(actual_pitch[steady] ** 2))),
        "steady_velocity_rms_mps": float(np.sqrt(np.mean(data["velocity"][steady] ** 2))),
        "final_position_residual_mm": float(np.mean((data["p"][steady] - target) * 1000.0)),
    }


def lift_metric(path, controller, event_path):
    event = json.loads(Path(event_path).read_text())
    data, height = height_arrays(path, controller)
    t = data["time"]
    rise = float(event["actual_rise_start_sim_time"])
    end = min(float(event["observation_end_sim_time"]), float(t[-1]))
    pre = (t >= rise - 2.0) & (t < rise)
    window = (t >= rise) & (t <= end)
    baseline = float(np.mean(data["p"][pre])) if np.any(pre) else float(data["p"][0])
    actual_pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    torque_abs = np.abs(data["torque"])
    terminal = t >= max(rise, end - 10.0)
    lock_time, latch_count = target_lock(path, controller)
    return {
        "controller": controller, "csv": str(path), "events": str(event_path),
        "target_lock_time_s": lock_time, "position_latch_count": latch_count,
        "target_lock_once_ok": bool(latch_count == 1),
        "static_gate_passed": bool(event["static_gate"]["passed"]),
        "completed": bool(event["completed"]),
        "actual_rise_start_sim_time": rise,
        "actual_low_return_sim_time": float(event["actual_low_return_sim_time"]),
        "observation_end_sim_time": end,
        "pre_rise_position_mean_m": baseline,
        "position_peak_relative_pre_rise_mm": float(np.max(np.abs((data["p"][window] - baseline) * 1000.0))) if np.any(window) else math.nan,
        "pitch_peak_relative_nominal_deg": float(np.max(np.abs(actual_pitch[window]))) if np.any(window) else math.nan,
        "velocity_peak_mps": float(np.max(np.abs(data["velocity"][window]))) if np.any(window) else math.nan,
        "torque_peak_nm": float(np.max(torque_abs[window])) if np.any(window) else math.nan,
        "torque_saturation_ratio": float(np.mean(torque_abs[window] >= 19.5)) if np.any(window) else math.nan,
        "final_position_residual_mm": float(np.mean((data["p"][terminal] - baseline) * 1000.0)) if np.any(terminal) else math.nan,
        "height_start_m": float(np.mean(height[t >= rise][:3])) if np.any(t >= rise) else math.nan,
        "height_end_m": float(np.mean(height[terminal][-3:])) if np.any(terminal) else math.nan,
    }


def summary(records, out, prefix):
    keys = sorted({key for r in records for key in r if key not in {"csv", "events"}})
    result = []
    groups = {}
    for record in records:
        group = (record["controller"], record.get("height_m", LOW_HEIGHT))
        groups.setdefault(group, []).append(record)
    for (controller, height), group in sorted(groups.items()):
        item = {"controller": controller, "height_m": height, "N": len(group)}
        for key in keys:
            if key in {"controller", "height_m", "N"}:
                continue
            avg, sd, n = mean_sd([r.get(key) for r in group])
            if avg is not None:
                item[f"{key}_mean"] = avg
                item[f"{key}_sd_unbiased"] = sd
                item[f"{key}_n_finite"] = n
        result.append(item)
    fields = sorted({k for r in result for k in r})
    with (out / f"{prefix}_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result)
    return result


def interp(curves, grid):
    vals = []
    for t, v in curves:
        mask = (t >= grid[0]) & (t <= grid[-1])
        vals.append(np.interp(grid, t[mask], v[mask], left=np.nan, right=np.nan))
    array = np.asarray(vals)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(array, axis=0), np.nanstd(array, axis=0, ddof=1)


def draw_lift(records, out):
    plt.rcParams.update({"font.family": ["Liberation Serif", "DejaVu Serif"],
                         "font.size": 8.5, "axes.labelsize": 9,
                         "axes.linewidth": 0.8, "xtick.labelsize": 8,
                         "ytick.labelsize": 8, "legend.fontsize": 8,
                         "svg.fonttype": "path", "pdf.fonttype": 42})
    grid = np.linspace(0.0, 32.0, 641)
    fig, axes = plt.subplots(3, 1, figsize=(3.7, 6.6), sharex=True)
    for controller, color, label in (("gs_lqr", BLUE, "GS-LQR"),
                                     ("position_pid", RED, "Four-loop PID")):
        h_curves, p_curves, theta_curves = [], [], []
        for record in records:
            if record["controller"] != controller:
                continue
            data, height = height_arrays(Path(record["csv"]), controller)
            rise = float(record["actual_rise_start_sim_time"])
            baseline = float(record["pre_rise_position_mean_m"])
            t = data["time"] - rise
            theta = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
            h_curves.append((t, height))
            p_curves.append((t, (data["p"] - baseline) * 1000.0))
            theta_curves.append((t, theta))
        h_mean, h_sd = interp(h_curves, grid)
        p_mean, p_sd = interp(p_curves, grid)
        theta_mean, theta_sd = interp(theta_curves, grid)
        for ax, mean, sd in ((axes[0], h_mean, h_sd), (axes[1], p_mean, p_sd), (axes[2], theta_mean, theta_sd)):
            ax.plot(grid, mean, color=color, label=label)
            ax.fill_between(grid, mean - sd, mean + sd, color=color, alpha=0.16)
    axes[0].set_ylabel("Hip-axle height (m)")
    axes[1].set_ylabel("Position from pre-rise mean (mm)")
    axes[2].set_ylabel("Pitch error (deg)")
    axes[2].set_xlabel("Time from actual rise start (s)")
    for ax in axes:
        ax.grid(color=GRID_COLOR, linestyle=":", linewidth=0.5)
        ax.legend(frameon=False, loc="best")
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(out / f"fig42_height_recheck_lift.{ext}", dpi=600, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal-root", required=True)
    parser.add_argument("--lift-root", action="append", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    formal_root = Path(args.formal_root)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    formal_results = json.loads((formal_root / "formal_results.json").read_text())
    static = []
    for result in formal_results:
        if "_constant_" not in result["trial_tag"]:
            continue
        info = tag_info(result["trial_tag"])
        if info["job"] != "constant":
            continue
        static.append(static_metric(Path(result["csv"]), info["controller"], info["height"]))
    lift_results = []
    lift_sources = []
    for source_name in args.lift_root:
        source_root = Path(source_name)
        lift_results.extend((row, source_root)
                            for row in json.loads((source_root / "formal_results.json").read_text()))
        lift_sources.append(str(source_root))
    lift = []
    lift_attempts = []
    for result, source_root in lift_results:
        info = tag_info(result["trial_tag"])
        event_path = source_root / info["controller"] / (Path(result["csv"]).stem + ".events.json")
        recording_error = result.get("recording_protocol_error")
        controller_protection_failure = False
        log_text = ""
        if recording_error in (None, "") and result.get("fail_reason") in ("no_csv_data", "controller_stopped_logging"):
            log_path = source_root / info["controller"] / "launch_logs" / f"{result['trial_tag']}_attempt{result.get('attempt', 1)}.log"
            try:
                log_text = log_path.read_text(encoding="utf-8", errors="ignore")
                recording_error = (
                    "Failed to activate controller" in log_text or
                    ("[ERROR] [spawner-" in log_text and "]: process has died" in log_text))
            except OSError:
                recording_error = False
        if not log_text:
            try:
                log_text = (source_root / info["controller"] / "launch_logs" /
                            f"{result['trial_tag']}_attempt{result.get('attempt', 1)}.log").read_text(
                                encoding="utf-8", errors="ignore")
            except OSError:
                pass
        controller_protection_failure = "controller disabled" in log_text.lower() or "pitch error" in log_text.lower()
        lift_attempts.append({
            "controller": info["controller"], "trial_tag": result["trial_tag"],
            "source_root": str(source_root), "protocol_valid": result.get("protocol_valid"),
            "retryable_protocol_error": result.get("retryable_protocol_error"),
            "recording_protocol_error": recording_error,
            "controller_protection_failure": controller_protection_failure,
            "fail_reason": result.get("fail_reason"),
            "static_gate_passed": result.get("lift_static_gate_valid"),
            "completed": result.get("lift_completed"),
        })
        if event_path.exists() and result.get("lift_completed") is True:
            lift.append(lift_metric(Path(result["csv"]), info["controller"], event_path))
    all_static_fields = sorted({k for r in static for k in r})
    with (out / "recomputed_static_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=all_static_fields)
        writer.writeheader(); writer.writerows(static)
    all_lift_fields = sorted({k for r in lift for k in r})
    with (out / "recomputed_lift_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=all_lift_fields)
        writer.writeheader(); writer.writerows(lift)
    attempt_fields = sorted({key for record in lift_attempts for key in record})
    with (out / "lift_protocol_attempts.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=attempt_fields)
        writer.writeheader(); writer.writerows(lift_attempts)
    summary(static, out, "static")
    summary(lift, out, "lift")
    draw_lift(lift, out)
    audit = {
        "static_runs_recomputed": len(static),
        "static_by_controller": {c: sum(r["controller"] == c for r in static) for c in ("position_pid", "gs_lqr")},
        "lift_attempts_including_protocol_failures": len(lift_attempts),
        "lift_runs_completed_and_analyzed": len(lift),
        "lift_by_controller": {c: sum(r["controller"] == c for r in lift) for c in ("position_pid", "gs_lqr")},
        "lift_sources": lift_sources,
        "definitions": {
            "static_start": "first time >= target lock + 2s with target height held within 5mm continuously for 2s",
            "static_steady": "last 10s",
            "lift_position_baseline": "mean raw position over 2s before actual rise start",
            "lift_window": "actual rise start through 10s after actual low return",
            "pitch": "actual pitch minus nominal equilibrium angle",
            "torque_saturation": "absolute total torque >= 19.5 Nm",
            "statistics": "N=3 mean +/- unbiased sample SD; no significance test",
        },
    }
    (out / "height_recheck_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2))
    (out / "metric_definitions.txt").write_text(json.dumps(audit["definitions"], ensure_ascii=False, indent=2))
    print(f"Static recomputed: {len(static)}; lift analyzed: {len(lift)}; output: {out}")


if __name__ == "__main__":
    main()
