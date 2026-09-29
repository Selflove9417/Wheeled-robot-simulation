#!/usr/bin/env python3
"""Recompute matched fixed-LQR/GS-LQR comparisons from raw trial logs."""

import argparse
import csv
import json
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from run_formal_pid_gslqr_campaign import arrays, read_rows

COLORS = {"fixed_lqr": "#D64A3A", "gs_lqr": "#3569B9"}
LABELS = {"fixed_lqr": "Fixed LQR (0.40 m)", "gs_lqr": "GS-LQR"}
plt.rcParams.update({"font.family": "serif",
                     "font.serif": ["Times New Roman", "Liberation Serif"],
                     "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
                     "pdf.fonttype": 42, "ps.fonttype": 42})


def save_csv(path, rows):
    if not rows:
        return
    columns = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def save_figure(fig, out, stem):
    for suffix in ("png", "pdf", "svg"):
        fig.savefig(out / f"{stem}.{suffix}", dpi=450, bbox_inches="tight")
    plt.close(fig)


def mean_sd(values):
    finite = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    return (statistics.mean(finite) if finite else math.nan,
            statistics.stdev(finite) if len(finite) > 1 else math.nan, len(finite))


def parse_tag(tag):
    push = re.fullmatch(r"(fixed_lqr|gs_lqr)_push_h([0-9.]+)_f([+-][0-9]+)_rep([0-9]+)", tag)
    if push:
        return "push", float(push[2]), float(push[3]), int(push[4])
    lift = re.fullmatch(r"(fixed_lqr|gs_lqr)_lift_v([0-9.]+)_rep([0-9]+)", tag)
    if lift:
        return "lift", 0.30, 0.0, int(lift[3])
    raise ValueError(tag)


def lift_details(path, event):
    data = arrays(path, "gs_lqr")
    raw = read_rows(path)
    t = data["time"]
    h = np.asarray([float(row["hip_axle_height"]) for row in raw[:len(t)]])
    wheel_torque = np.asarray([float(row["tau_each"]) for row in raw[:len(t)]])
    up = float(event["lift_command_up_sim_time"])
    rise = float(event["actual_rise_start_sim_time"])
    high = float(event["high_reached_sim_time"])
    down = float(event["lift_command_down_sim_time"])
    low = float(event["actual_low_return_sim_time"])
    end = min(float(event["observation_end_sim_time"]), float(t[-1]))
    pre = (t >= up - 2.0) & (t < up)
    active = (t >= up) & (t <= end)
    last = (t >= end - 2.0) & (t <= end)
    baseline = float(np.mean(data["p"][pre]))
    pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
    def speed(a, b, sign):
        mask = (t >= a) & (t <= b) & (h >= 0.32) & (h <= 0.48)
        return sign * float(np.polyfit(t[mask], h[mask], 1)[0]) if np.count_nonzero(mask) > 9 else math.nan
    return {
        "position_peak_mm": float(np.max(np.abs((data["p"][active] - baseline) * 1000.0))),
        "pitch_peak_deg": float(np.max(np.abs(pitch[active]))),
        "velocity_peak_mps": float(np.max(np.abs(data["velocity"][active]))),
        "torque_peak_nm": float(np.max(np.abs(wheel_torque[active]))),
        "torque_saturation_ratio": float(np.mean(np.abs(wheel_torque[active]) >= 9.75)),
        "final_position_residual_mm": float(np.mean((data["p"][last] - baseline) * 1000.0)),
        "actual_rise_speed_mps": speed(rise, high, 1.0),
        "actual_descend_speed_mps": speed(down, low, -1.0),
        "height_min_m": float(np.min(h[active])),
        "height_max_m": float(np.max(h[active])),
        "position_reference_span_mm": float(np.ptp(data["p_target"][(t >= up) & (t <= end)]) * 1000.0),
    }


def collect(root):
    results = json.loads((root / "formal_results.json").read_text(encoding="utf-8"))
    rows = []
    for result in results:
        job, height, force, rep = parse_tag(result["trial_tag"])
        path = Path(result["csv"])
        valid = bool(result.get("protocol_valid")) and result.get("fail_reason") not in (
            "no_csv_data", "controller_stopped_logging")
        row = {"controller": result["controller"], "job": job, "height_m": height,
               "force_N": force, "rep": rep, "trial_tag": result["trial_tag"],
               "protocol_valid": valid, "fail_reason": result.get("fail_reason") or "",
               "csv": str(path),
               "controller_protection_failure": bool(result.get("controller_protection_failure"))}
        if job == "push":
            row["pulse_valid"] = bool(result.get("protocol_pulse_valid"))
            row["force_direction_ok"] = bool(result.get("force_direction_ok"))
            for source, dest in (
                    ("position_peak_relative_mm", "position_peak_mm"),
                    ("pitch_peak_relative_deg", "pitch_peak_deg"),
                    ("velocity_peak_mps", "velocity_peak_mps"),
                    ("torque_peak_postpulse_1p5s_nm", "torque_peak_nm"),
                    ("torque_rms_postpulse_1p5s_nm", "torque_rms_nm"),
                    ("attitude_velocity_recovery_s", "attitude_velocity_recovery_s"),
                    ("position_10mm_recovery_s", "position_10mm_recovery_s"),
                    ("final_position_residual_mm", "final_position_residual_mm"),
                    ("position_saturation_ratio_response", "torque_saturation_ratio")):
                row[dest] = result.get(source)
            pulse_path = Path(str(path) + ".pulse.json")
            if pulse_path.exists():
                pulse = json.loads(pulse_path.read_text(encoding="utf-8"))
                row["pulse_start_sim_time"] = pulse.get("pulse_start_sim_time")
                if pulse.get("pulse_end_sim_time") is not None and pulse.get("pulse_start_sim_time") is not None:
                    row["pulse_duration_s"] = (float(pulse["pulse_end_sim_time"]) -
                                               float(pulse["pulse_start_sim_time"]))
                if pulse.get("pulse_start_sim_time") is not None and path.exists():
                    data = arrays(path, "gs_lqr")
                    t = data["time"]
                    raw = read_rows(path)
                    wheel_torque = np.asarray([float(item["tau_each"]) for item in raw[:len(t)]])
                    start = float(pulse["pulse_start_sim_time"])
                    window = (t >= start) & (t <= start + 1.5)
                    if np.any(window):
                        row["torque_peak_nm"] = float(np.max(np.abs(wheel_torque[window])))
                        row["torque_rms_nm"] = float(np.sqrt(np.mean(wheel_torque[window] ** 2)))
                        row["torque_saturation_ratio"] = float(np.mean(np.abs(wheel_torque[window]) >= 9.75))
        else:
            event_path = root / result["controller"] / f"{path.stem}.events.json"
            row["events"] = str(event_path)
            if event_path.exists():
                event = json.loads(event_path.read_text(encoding="utf-8"))
                row["static_gate_passed"] = bool(event["static_gate"]["passed"])
                row["completed"] = bool(event["completed"])
                if row["completed"] and path.exists():
                    row.update(lift_details(path, event))
                    row["speed_within_2pct"] = all(
                        abs(row[key] - 0.15) / 0.15 <= 0.02
                        for key in ("actual_rise_speed_mps", "actual_descend_speed_mps"))
        rows.append(row)
    return rows


METRICS = ("position_peak_mm", "pitch_peak_deg", "velocity_peak_mps",
           "torque_peak_nm", "torque_saturation_ratio", "final_position_residual_mm",
           "attitude_velocity_recovery_s", "position_10mm_recovery_s",
           "actual_rise_speed_mps", "actual_descend_speed_mps")


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["job"], row["height_m"], row["force_N"], row["controller"])].append(row)
    summary = []
    for (job, height, force, controller), items in sorted(groups.items()):
        valid = [row for row in items if row["protocol_valid"]]
        entry = {"job": job, "height_m": height, "force_N": force,
                 "controller": controller, "N_attempted": len(items),
                 "N_protocol_valid": len(valid),
                 "N_protocol_invalid": len(items) - len(valid),
                 "N_push_delivered": sum(row.get("pulse_valid", False) for row in valid)
                 if job == "push" else "",
                 "N_pre_push_failure": sum(bool(row["fail_reason"]) and
                                           not row.get("pulse_valid", False) for row in valid)
                 if job == "push" else "",
                 "N_speed_within_2pct": sum(row.get("speed_within_2pct", False) for row in valid)
                 if job == "lift" else "",
                 "N_physical_noncompletion": sum(
                     bool(row["fail_reason"]) or
                     (job == "lift" and not row.get("completed", False))
                     for row in valid),
                 "N_force_direction_ok": sum(row.get("force_direction_ok", False) for row in valid)
                 if job == "push" else "",
                 "N_controller_protection_stop": sum(
                     row.get("controller_protection_failure", False) for row in valid),
                 "N_static_gate_passed": sum(row.get("static_gate_passed", False) for row in valid)
                 if job == "lift" else "",
                 "N_completed": sum(row.get("completed", False) for row in valid)
                 if job == "lift" else ""}
        for metric in METRICS:
            avg, sd, n = mean_sd([row.get(metric) for row in valid])
            entry[metric + "_mean"] = avg
            entry[metric + "_sd"] = sd
            entry[metric + "_N"] = n
        summary.append(entry)
    return summary


def comparisons(summary):
    paired = defaultdict(dict)
    for row in summary:
        paired[(row["job"], row["height_m"], row["force_N"])][row["controller"]] = row
    output = []
    for (job, height, force), group in sorted(paired.items()):
        if set(group) != {"fixed_lqr", "gs_lqr"}:
            continue
        item = {"job": job, "height_m": height, "force_N": force,
                "N_protocol_valid_fixed": group["fixed_lqr"]["N_protocol_valid"],
                "N_protocol_valid_gs": group["gs_lqr"]["N_protocol_valid"],
                "N_push_delivered_fixed": group["fixed_lqr"]["N_push_delivered"],
                "N_push_delivered_gs": group["gs_lqr"]["N_push_delivered"],
                "N_physical_noncompletion_fixed": group["fixed_lqr"]["N_physical_noncompletion"],
                "N_physical_noncompletion_gs": group["gs_lqr"]["N_physical_noncompletion"]}
        for metric in ("position_peak_mm", "pitch_peak_deg", "torque_peak_nm",
                       "velocity_peak_mps", "final_position_residual_mm"):
            fixed = group["fixed_lqr"][metric + "_mean"]
            gs = group["gs_lqr"][metric + "_mean"]
            item[metric + "_fixed"] = fixed
            item[metric + "_gs"] = gs
            complete_pair = all(group[controller][metric + "_N"] == 3 and
                                group[controller]["N_physical_noncompletion"] == 0
                                for controller in ("fixed_lqr", "gs_lqr"))
            item[metric + "_gs_reduction_pct"] = 100.0 * (fixed - gs) / fixed if complete_pair and fixed else math.nan
        output.append(item)
    return output


def push_summary_plot(summary, out):
    fig, axes = plt.subplots(3, 2, figsize=(7.0, 6.6), sharex=True)
    for column, force in enumerate((20.0, -20.0)):
        for row_index, (metric, ylabel) in enumerate((("position_peak_mm", "Peak position (mm)"),
                                                    ("pitch_peak_deg", "Peak pitch (deg)"),
                                                    ("N_push_delivered", "Push delivered (of 3)"))):
            ax = axes[row_index, column]
            for controller in ("fixed_lqr", "gs_lqr"):
                group = sorted((r for r in summary if r["job"] == "push" and
                                r["force_N"] == force and r["controller"] == controller),
                               key=lambda r: r["height_m"])
                values = [r[metric] if metric == "N_push_delivered" else r[metric + "_mean"] for r in group]
                ax.plot([r["height_m"] for r in group], values,
                        "-o", color=COLORS[controller], label=LABELS[controller])
            ax.set_title(f"{force:+.0f} N, 0.20 s")
            ax.set_ylabel(ylabel)
            ax.set_xticks((0.30, 0.40, 0.50))
            ax.grid(True, linestyle=":", linewidth=0.6)
    for ax in axes[-1]:
        ax.set_yticks((0, 1, 2, 3))
        ax.set_ylim(-0.1, 3.2)
    for ax in axes[-1]:
        ax.set_xlabel("Hip-axle height (m)")
    axes[0, 0].legend(frameon=False)
    fig.tight_layout()
    save_figure(fig, out, "fixed_vs_gslqr_push_summary")


def mean_curve(curves, grid):
    aligned = []
    for t, y in curves:
        if len(t) < 2:
            continue
        aligned.append(np.interp(grid, t, y, left=np.nan, right=np.nan))
    if not aligned:
        return np.full_like(grid, np.nan)
    with np.errstate(invalid="ignore"):
        return np.nanmean(np.asarray(aligned), axis=0)


def lift_plot(rows, out):
    grid = np.linspace(0.0, 22.0, 2201)
    fig, axes = plt.subplots(3, 1, figsize=(3.4, 5.3), sharex=True)
    for controller in ("fixed_lqr", "gs_lqr"):
        curves = [[], [], []]
        for row in rows:
            if row["controller"] != controller or row["job"] != "lift" or not row.get("completed"):
                continue
            path = Path(row["csv"])
            data = arrays(path, "gs_lqr")
            raw = read_rows(path)
            event = json.loads(Path(row["events"]).read_text(encoding="utf-8"))
            up = float(event["lift_command_up_sim_time"])
            t = data["time"] - up
            h = np.asarray([float(item["hip_axle_height"]) for item in raw[:len(t)]])
            pre = (t >= -2.0) & (t < 0.0)
            p = (data["p"] - float(np.mean(data["p"][pre]))) * 1000.0
            pitch = (data["pitch"] - data["theta_eq"]) * 180.0 / np.pi
            for dest, y in zip(curves, (h, p, pitch)):
                mask = t >= -0.1
                dest.append((t[mask], y[mask]))
        for ax, values in zip(axes, curves):
            if values:
                ax.plot(grid, mean_curve(values, grid), color=COLORS[controller],
                        label=LABELS[controller], linewidth=1.4)
    for ax, ylabel in zip(axes, ("Hip-axle height (m)", "Position from pre-rise mean (mm)",
                                  "Pitch error (deg)")):
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle=":", linewidth=0.6)
        ax.legend(frameon=False)
    axes[0].set_ylim(0.29, 0.51)
    axes[-1].set_xlabel("Time from ascent command (s)")
    fig.tight_layout()
    save_figure(fig, out, "fixed_vs_gslqr_lift_0p15")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = collect(args.campaign_root.resolve())
    summary = summarize(rows)
    comparison = comparisons(summary)
    save_csv(out / "trial_metrics.csv", rows)
    save_csv(out / "condition_summary.csv", summary)
    save_csv(out / "paired_comparison.csv", comparison)
    (out / "analysis.json").write_text(json.dumps({
        "source": str(args.campaign_root.resolve()),
        "definitions": {
            "push_peak_window_s": "pulse start through 1.5 s after pulse start",
            "push_position_baseline": "mean position during 2 s before pulse",
            "lift_position_baseline": "mean position during 2 s before ascent command",
            "speed_fit": "least squares over 0.32 to 0.48 m between detected transition events",
            "standard_deviation": "unbiased sample standard deviation",
            "fixed_gain": "legacy_safe feedback gain at H=0.40 m",
            "scheduled_gain": "legacy_safe five-node interpolation",
            "wheel_torque": "absolute actual tau_each per wheel; saturation at 9.75 of 10 Nm",
        }, "summary": summary, "comparison": comparison},
        ensure_ascii=False, indent=2), encoding="utf-8")
    if any(row["job"] == "push" for row in summary):
        push_summary_plot(summary, out)
    if any(row["job"] == "lift" and row.get("completed") for row in rows):
        lift_plot(rows, out)
    print(f"Analysed {len(rows)} trials into {out}")


if __name__ == "__main__":
    main()
