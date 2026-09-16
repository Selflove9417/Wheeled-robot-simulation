#!/usr/bin/env python3
"""Recompute single-run comparison metrics and scientific figures.

The comparison uses one selected existing GS-LQR log per matching condition
(scheduled, representative ``*_1.csv``) and never starts a controller.  No
mean, standard deviation, significance test, or paper file is produced here.
"""

import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    names = set(rows[0])
    def col(name):
        if name not in names:
            raise ValueError(f"{path}: missing column {name}")
        return np.asarray([float(row[name]) for row in rows])
    return {name: col(name) for name in names if name not in (None, "")}


def continuous_time(t, mask, start, hold=2.0):
    indices = np.where(t >= start)[0]
    for i in indices:
        j = i
        while j < len(t) and mask[j]:
            if t[j] - t[i] >= hold:
                return float(t[i] - start)
            j += 1
    return np.nan


def metrics(path, kind, force=0.0):
    d = load(path)
    t = d["time"]
    if kind == "position_pid":
        position = d["delta_p"] * 1000.0
        pitch = d["theta_error"] * 180.0 / np.pi
        velocity = d["v"]
        torque = d["u_clamped"]
        sat = d["is_saturated"] > 0.5
        latched = d["position_target_latched"] > 0.5
        first = int(np.where(latched)[0][0]) if np.any(latched) else 0
        t_ref = t[first]
        pulse = None
        pulse_path = path + ".pulse.json"
        if os.path.exists(pulse_path):
            import json
            with open(pulse_path, encoding="utf-8") as handle:
                pulse = json.load(handle)
    else:
        position = d["filtered_x_error"] * 1000.0
        pitch = d["theta_error"] * 180.0 / np.pi
        velocity = d["x_dot"]
        torque = d["u_model"]
        sat = np.abs(torque) >= 19.5
        t_ref = t[0]
        pulse = None
    post = t >= t_ref
    result = {
        "file": path,
        "controller": kind,
        "position_definition": "new:delta_p=p-p_target; GS-LQR:filtered_x_error",
        "position_peak_mm": float(np.max(np.abs(position[post]))),
        "pitch_peak_deg": float(np.max(np.abs(pitch[post]))),
        "torque_peak_nm": float(np.max(np.abs(torque[post]))),
        "torque_rms_nm": float(np.sqrt(np.mean(torque[post] ** 2))),
        "torque_saturation_ratio": float(np.mean(sat[post])),
        "final_position_error_mm": float(position[-1]),
        "velocity_peak_mps": float(np.max(np.abs(velocity[post]))),
    }
    if force and pulse and pulse.get("pulse_start_sim_time") is not None:
        start = float(pulse["pulse_start_sim_time"])
        end = float(pulse["pulse_end_sim_time"])
        before = (t >= start - 0.25) & (t < start)
        response = (t >= start + 0.10) & (t <= start + 0.50)
        result["pulse_duration_s"] = end - start
        result["initial_velocity_change_mps"] = float(
            np.median(velocity[response]) - np.median(velocity[before])) \
            if np.any(before) and np.any(response) else np.nan
        result["attitude_velocity_recovery_s"] = continuous_time(
            t, (np.abs(pitch) <= 0.5) & (np.abs(velocity) <= 0.02), end, 2.0)
        result["position_10mm_recovery_s"] = continuous_time(
            t, np.abs(position) <= 10.0, end, 2.0)
    else:
        result["attitude_velocity_recovery_s"] = continuous_time(
            t, (np.abs(pitch) <= 0.5) & (np.abs(velocity) <= 0.01), t_ref, 2.0)
        result["position_10mm_recovery_s"] = continuous_time(
            t, np.abs(position) <= 10.0, t_ref, 2.0)
    return result


def pick_new(new_dir, pattern):
    candidates = sorted(glob.glob(os.path.join(new_dir, pattern)))
    return candidates[0] if candidates else None


def pick_gs(gs_dir, name):
    path = os.path.join(gs_dir, name)
    return path if os.path.exists(path) else None


def representative_pairs(new_dir, gs_dir):
    pairs = []
    for h in (0.3, 0.4, 0.5):
        new = pick_new(new_dir, f"constant_{h:g}_1.csv")
        gs = pick_gs(gs_dir, f"constant_{h:g}_scheduled_1.csv")
        if new and gs:
            pairs.append((f"constant H={h:.2f}", new, gs, 0.0))
    new = pick_new(new_dir, "lift_1.csv")
    gs = pick_gs(gs_dir, "lift_scheduled_1.csv")
    if new and gs:
        pairs.append(("lift 0.30-0.50-0.30", new, gs, 0.0))
    for h in (0.3, 0.4, 0.5):
        for sign in (1, -1):
            sign_text = "+20" if sign > 0 else "-20"
            force = sign * 20.0
            new = pick_new(new_dir, f"push_{h:g}_f{force:+g}_1.csv")
            gs = pick_gs(gs_dir, f"push_{h:g}_scheduled_f{force:+g}_1.csv")
            if new and gs:
                pairs.append((f"push H={h:.2f} {sign_text} N", new, gs, sign * 20.0))
    return pairs


def make_timeseries(pairs, out_dir):
    selected = [p for p in pairs if p[0].startswith("constant") or p[0].startswith("lift")]
    if not selected:
        return
    fig, axes = plt.subplots(len(selected), 2, figsize=(12, 3.2 * len(selected)), squeeze=False)
    for row, (label, new_path, gs_path, force) in enumerate(selected):
        new = load(new_path)
        gs = load(gs_path)
        ax = axes[row, 0]
        ax.plot(new["time"], new["delta_p"] * 1000, label="four-loop PID")
        ax.plot(gs["time"], gs["filtered_x_error"] * 1000, label="GS-LQR")
        ax.set_ylabel("position error (mm)")
        ax.set_title(label)
        ax.grid(alpha=0.25)
        ax = axes[row, 1]
        ax.plot(new["time"], new["theta_error"] * 180 / np.pi, label="four-loop PID")
        ax.plot(gs["time"], gs["theta_error"] * 180 / np.pi, label="GS-LQR")
        ax.set_ylabel("pitch error (deg)")
        ax.grid(alpha=0.25)
        if row == 0:
            axes[row, 0].legend()
            axes[row, 1].legend()
    for ax in axes[-1]:
        ax.set_xlabel("simulation time (s)")
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(os.path.join(out_dir, f"position_pid_exploration_timeseries.{ext}"), dpi=180)
    plt.close(fig)


def make_push_plot(pairs, out_dir):
    selected = [p for p in pairs if p[0].startswith("push")]
    if not selected:
        return
    fig, axes = plt.subplots(2, 3, figsize=(14, 7), squeeze=False)
    for col, height in enumerate((0.3, 0.4, 0.5)):
        for row, sign in enumerate((1, -1)):
            label = f"push H={height:.2f} {'+20' if sign > 0 else '-20'} N"
            match = next((p for p in selected if p[0] == label), None)
            ax = axes[row, col]
            if match:
                _, new_path, gs_path, _ = match
                new, gs = load(new_path), load(gs_path)
                ax.plot(new["time"], new["v"], label="four-loop PID")
                ax.plot(gs["time"], gs["x_dot"], label="GS-LQR")
            ax.set_title(label)
            ax.set_xlabel("simulation time (s)")
            ax.set_ylabel("velocity (m/s)")
            ax.grid(alpha=0.25)
            if row == 0 and col == 0:
                ax.legend()
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(os.path.join(out_dir, f"position_pid_push_timeseries.{ext}"), dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--new-dir", required=True)
    parser.add_argument("--gs-dir", default="/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/height_campaign")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()
    if args.out_dir is None:
        args.out_dir = args.new_dir
    os.makedirs(args.out_dir, exist_ok=True)
    pairs = representative_pairs(args.new_dir, args.gs_dir)
    rows = []
    for label, new_path, gs_path, force in pairs:
        new_metrics = metrics(new_path, "position_pid", force)
        gs_metrics = metrics(gs_path, "gs_lqr", force)
        new_metrics.update({"condition": label, "source": "new_position_pid"})
        gs_metrics.update({"condition": label, "source": "existing_gs_lqr"})
        rows.extend((new_metrics, gs_metrics))
    if rows:
        fields = sorted({key for row in rows for key in row})
        with open(os.path.join(args.out_dir, "exploration_comparison.csv"), "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    make_timeseries(pairs, args.out_dir)
    make_push_plot(pairs, args.out_dir)
    print(f"Compared {len(pairs)} representative conditions; outputs: {args.out_dir}")


if __name__ == "__main__":
    main()
