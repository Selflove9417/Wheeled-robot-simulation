#!/usr/bin/env python3
"""Recompute paper Sec. 4.2 pulse metrics and draw a compact vector figure."""

from pathlib import Path
import csv
import glob
import json
import math
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1] / "src" / "data_logs"
PID_OLD = ROOT / "torque_pid_trials"
PID_NEW = ROOT / "pid_publication_campaign"
LQR = ROOT / "height_campaign"
OUT = ROOT / "paper42_pid_gslqr"
BLUE, RED, GRID = "#3569B9", "#D64A3A", "#D8DEE7"


def style():
    plt.rcParams.update({
        "font.family": ["Liberation Serif", "DejaVu Serif"],
        "mathtext.fontset": "stix", "font.size": 8.5,
        "axes.labelsize": 9, "axes.linewidth": 0.8,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
        "xtick.direction": "in", "ytick.direction": "in",
        "legend.fontsize": 8, "svg.fonttype": "path", "pdf.fonttype": 42,
        "savefig.facecolor": "white", "figure.facecolor": "white",
    })


def read_trial(path, controller):
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    with open(str(path) + ".pulse.json") as handle:
        pulse = json.load(handle)
    if not (pulse.get("completed") and pulse.get("clear_complete") and
            pulse.get("wrench_publish_count") == 1 and
            0.19 <= float(pulse.get("actual_duration", 0)) <= 0.21):
        raise ValueError(f"Invalid force pulse: {path}")
    t0 = float(pulse["pulse_start_sim_time"])
    t1 = float(pulse["pulse_end_sim_time"])
    fields = ("p", "p_0", "v", "theta_eq", "u_clamped") if controller == "pid" else (
        "x", "x_ref", "x_dot", "theta_eq_nominal", "u_model")
    position, origin, velocity, eq, torque = fields
    data = []
    for row in rows:
        try:
            t = float(row["time"])
            data.append((t, float(row[position]), float(row[origin]),
                         float(row[velocity]),
                         (float(row["pitch"]) - float(row[eq])) * 180 / math.pi,
                         float(row[torque])))
        except (ValueError, KeyError):
            continue
    data.sort()
    pre = [p for t, p, _, _, _, _ in data if t0 - 2 <= t < t0]
    if len(pre) < 100:
        raise ValueError(f"Insufficient pre-pulse position: {path}")
    pre_mean = statistics.mean(pre)
    onset = [(t, p, o, v, th, u) for t, p, o, v, th, u in data if t0 <= t <= t0 + 25]
    response = [(t, p, o, v, th, u) for t, p, o, v, th, u in data if t0 <= t <= t0 + 1.5]
    metrics = {
        "pre_origin_mm": 1000 * statistics.mean(p - o for t, p, o, _, _, _ in data if t0 - 2 <= t < t0),
        "peak_relative_mm": 1000 * max(abs(p - pre_mean) for _, p, _, _, _, _ in onset),
        "peak_velocity_mps": max(abs(v) for _, _, _, v, _, _ in response),
        "peak_pitch_deg": max(abs(th) for _, _, _, _, th, _ in response),
        "peak_torque_nm": max(abs(u) for _, _, _, _, _, u in response),
        "final_origin_mm": 1000 * statistics.mean(p - o for t, p, o, _, _, _ in data if t >= data[-1][0] - 2),
    }
    after = [(t, abs(v), abs(th)) for t, _, _, v, th, _ in data if t >= t1]
    recovery = None
    for i, (t, _, _) in enumerate(after):
        if t + 2 > after[-1][0]:
            break
        window = [(v, th) for tt, v, th in after[i:] if tt < t + 2]
        if window and max(v for v, _ in window) <= 0.02 and max(th for _, th in window) <= 0.5:
            recovery = t - t1
            break
    metrics["theta_velocity_recovery_s"] = recovery
    return metrics, ([(t - t0, 1000 * (p - pre_mean)) for t, p, _, _, _, _ in onset], t1 - t0)


def accepted_new_paths():
    manifest = PID_NEW / "manifest.json"
    if not manifest.exists():
        return []
    data = json.loads(manifest.read_text())
    return [PID_NEW / item["file"] for key, item in data["trials"].items()
            if key.startswith("push_") and item["accepted"]]


def trials():
    items = []
    pid_paths = sorted(PID_OLD.glob("trial[56]_push_*_rep*.csv")) + accepted_new_paths()
    lqr_paths = sorted(LQR.glob("push_*_scheduled_f*_*.csv"))
    for controller, paths in (("pid", pid_paths), ("gslqr", lqr_paths)):
        for path in paths:
            name = path.name
            if controller == "pid":
                height = 0.30 if "trial" in name else int(name.split("_")[1][1:]) / 100
                force = 20 if ("pos" in name or "trial5" in name) else -20
            else:
                height = float(name.split("_")[1])
                force = 20 if "f+20" in name else -20
            try:
                metrics, curve = read_trial(path, "pid" if controller == "pid" else "lqr")
            except (ValueError, OSError) as exc:
                print(f"[Exclude] {path.name}: {exc}")
                continue
            items.append({"controller": controller, "height": height,
                          "force": force, "file": str(path), "metrics": metrics,
                          "curve": curve})
    return items


def summarize(items):
    rows = []
    for height in (0.30, 0.40, 0.50):
        for force in (20, -20):
            for controller in ("gslqr", "pid"):
                group = [x["metrics"] for x in items if x["height"] == height and
                         x["force"] == force and x["controller"] == controller]
                if not group:
                    continue
                row = {"height": height, "force": force, "controller": controller,
                       "n": len(group)}
                for key in group[0]:
                    vals = [g[key] for g in group if g[key] is not None]
                    row[key + "_mean"] = statistics.mean(vals) if vals else None
                    row[key + "_sd"] = statistics.stdev(vals) if len(vals) >= 2 else None
                rows.append(row)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "pulse_metrics.json", "w") as handle:
        json.dump(rows, handle, indent=2)
    for row in rows:
        print(row["height"], row["force"], row["controller"], row["n"],
              "peak_rel_mm=", round(row["peak_relative_mm_mean"], 2),
              "T_thv=", round(row["theta_velocity_recovery_s_mean"], 2)
              if row["theta_velocity_recovery_s_mean"] is not None else None)


def draw_h030(items):
    style()
    fig, axes = plt.subplots(1, 2, figsize=(6.7, 2.55), sharex=True, sharey=True)
    for ax, force in zip(axes, (20, -20)):
        for controller, color, linestyle, label in (
                ("gslqr", BLUE, "-", "GS-LQR"),
                ("pid", RED, "--", "Cascade PID")):
            group = [x for x in items if x["height"] == 0.30 and
                     x["force"] == force and x["controller"] == controller]
            if not group:
                continue
            for run in group:
                xy, pulse_length = run["curve"]
                ax.plot([x for x, _ in xy], [y for _, y in xy], color=color,
                        linestyle=linestyle, lw=1.0, alpha=0.18)
            xy, pulse_length = group[0]["curve"]
            ax.plot([x for x, _ in xy], [y for _, y in xy], color=color,
                    linestyle=linestyle, lw=1.7, label=label)
        ax.axvspan(0, 0.20, color="#B9C4D5", alpha=0.3, zorder=0)
        ax.axhline(0, color="#5C6670", lw=0.7)
        ax.grid(color=GRID, linewidth=0.5, linestyle=":")
        ax.set_xlim(0, 25)
        ax.set_title(f"({ 'a' if force > 0 else 'b' }) {force:+d} N, 0.20 s")
        ax.set_xlabel("Time after pulse onset (s)")
    axes[0].set_ylabel("Displacement from pre-pulse position (mm)")
    axes[0].legend(loc="upper right", frameon=False)
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(OUT / f"fig42_pid_gslqr_push_h030.{ext}", dpi=600,
                    bbox_inches="tight")
    plt.close(fig)


def lift_curve(path, controller):
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    hf = "height" if controller == "pid" else "hip_axle_height"
    pf = "p" if controller == "pid" else "x"
    eqf = "theta_eq" if controller == "pid" else "theta_eq_nominal"
    data = [(float(r["time"]), float(r[hf]), float(r[pf]),
             180 / math.pi * (float(r["pitch"]) - float(r[eqf]))) for r in rows]
    active = [i for i in range(1, len(data)) if abs(data[i][1] - data[i-1][1]) > 1e-5]
    spans, current = [], [active[0]]
    for i in active[1:]:
        if i - current[-1] > 30:
            spans.append(current)
            current = [i]
        else:
            current.append(i)
    spans.append(current)
    rise = next(s for s in spans if data[s[0]][1] < 0.31 and data[s[-1]][1] > 0.49)
    descend = next(s for s in spans if data[s[0]][1] > 0.49 and data[s[-1]][1] < 0.31)
    t0 = data[rise[0]][0]
    t_end = data[descend[-1]][0] + 2.0
    p0 = statistics.mean(p for t, _, p, _ in data if t0 - 2 <= t < t0)
    curve = [(t - t0, h, 1000 * (p - p0), th) for t, h, p, th in data
             if t0 - 0.5 <= t <= t_end]
    metrics = {
        "pitch_peak_deg": max(abs(th) for t, _, _, th in curve if t >= 0),
        "position_peak_mm": max(abs(p) for t, _, p, _ in curve if t >= 0),
    }
    return curve, metrics


def draw_lift():
    style()
    paths = (("gslqr", LQR / "lift_scheduled_1.csv", BLUE, "-", "GS-LQR"),
             ("pid", PID_NEW / "lift_rep1.csv", RED, "--", "Cascade PID"))
    fig, axes = plt.subplots(3, 1, figsize=(3.55, 5.3), sharex=True)
    for controller, path, color, ls, label in paths:
        if not path.exists():
            continue
        curve, metrics = lift_curve(path, "pid" if controller == "pid" else "lqr")
        print("lift", label, path.name, metrics)
        x = [row[0] for row in curve]
        for ax, idx in zip(axes, (1, 2, 3)):
            ax.plot(x, [row[idx] for row in curve], color=color, ls=ls,
                    lw=1.6, label=label)
    for ax in axes:
        ax.grid(color=GRID, linewidth=0.5, linestyle=":")
        ax.axvline(0, color="#5C6670", linewidth=0.6)
        ax.tick_params(direction="in")
    axes[0].set_ylabel("Height (m)")
    axes[1].set_ylabel("Relative position (mm)")
    axes[2].set_ylabel("Pitch error (deg)")
    axes[2].set_xlabel("Time from ascent onset (s)")
    axes[0].legend(loc="upper left", frameon=False)
    axes[2].set_xlim(-0.5, 18)
    fig.tight_layout()
    for ext in ("svg", "pdf", "png"):
        fig.savefig(OUT / f"fig42_pid_gslqr_height.{ext}", dpi=600,
                    bbox_inches="tight")
    plt.close(fig)


def main():
    items = trials()
    summarize(items)
    draw_h030(items)
    draw_lift()


if __name__ == "__main__":
    main()
