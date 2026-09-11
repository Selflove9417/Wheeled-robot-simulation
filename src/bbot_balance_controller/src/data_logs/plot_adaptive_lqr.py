#!/usr/bin/env python3
"""Plot Adaptive GS-LQR enable state, gate state, and tracking errors."""

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch


REQUIRED_COLUMNS = {
    "time",
    "x_error",
    "x_dot",
    "theta_eq_nominal",
    "theta_eq_adaptive",
    "delta_y_hat",
    "gate_open",
    "adapt_enabled",
}


def load_log(path):
    """Load and validate an adaptive LQR CSV log."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"日志文件不存在: {path}")
    if os.path.getsize(path) == 0:
        raise ValueError(f"日志文件为空: {path}")

    data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")
    if data.dtype.names is None:
        raise ValueError("CSV 缺少有效表头")
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)

    missing = REQUIRED_COLUMNS.difference(data.dtype.names)
    if missing:
        raise ValueError("CSV 缺少字段: " + ", ".join(sorted(missing)))

    finite = np.isfinite(data["time"])
    data = data[finite]
    if len(data) < 2:
        raise ValueError("有效日志数据少于两行，无法绘图")
    return np.sort(data, order="time")


def enabled_intervals(time, enabled):
    """Return [start, end] intervals in which adaptation is enabled."""
    enabled = np.asarray(enabled, dtype=bool)
    boundaries = np.flatnonzero(np.diff(enabled.astype(int)) != 0) + 1
    starts = np.concatenate(([0], boundaries))
    ends = np.concatenate((boundaries, [len(time)]))
    intervals = []
    for start, end in zip(starts, ends):
        if not enabled[start]:
            continue
        left = time[start]
        right = time[end - 1]
        if end < len(time):
            right = 0.5 * (time[end - 1] + time[end])
        intervals.append((left, right))
    return intervals


def add_enable_background(axes, time, enabled):
    """Shade all periods in which adaptation is commanded on."""
    for left, right in enabled_intervals(time, enabled):
        for axis in axes:
            axis.axvspan(left, right, color="#66bb6a", alpha=0.12, linewidth=0)

    switches = np.flatnonzero(np.diff(enabled.astype(int)) != 0) + 1
    for index in switches:
        for axis in axes:
            axis.axvline(time[index], color="#388e3c", linestyle=":", linewidth=1.0)


def stage_statistics(label, mask, x_error, x_dot):
    """Calculate position and speed metrics for one enable state."""
    count = int(np.count_nonzero(mask))
    if count == 0:
        return f"{label}: no samples"
    position_rmse = np.sqrt(np.mean(np.square(x_error[mask])))
    position_mae = np.mean(np.abs(x_error[mask]))
    speed_mae = np.mean(np.abs(x_dot[mask]))
    return (
        f"{label}: N={count}, position RMSE={position_rmse:.5f} m, "
        f"position MAE={position_mae:.5f} m, mean |speed|={speed_mae:.5f} m/s"
    )


def plot_log(data, output_path):
    """Create the adaptive-controller diagnostic figure."""
    time = data["time"] - data["time"][0]
    enabled = data["adapt_enabled"] >= 0.5
    gate_open = data["gate_open"] >= 0.5
    updates_paused = (
        data["adapt_update_paused"] >= 0.5
        if "adapt_update_paused" in data.dtype.names
        else np.zeros(len(data), dtype=bool)
    )
    has_window_state = (
        "window_progress" in data.dtype.names
        and "cooldown_remaining" in data.dtype.names
    )
    has_range_state = "window_position_range" in data.dtype.names

    x_error = data["x_error"]
    x_dot = data["x_dot"]
    filtered_x_error = (
        data["filtered_x_error"]
        if "filtered_x_error" in data.dtype.names
        else x_error
    )
    filtered_x_dot = (
        data["filtered_x_dot"] if "filtered_x_dot" in data.dtype.names else x_dot
    )

    row_count = 6 if has_range_state else (5 if has_window_state else 4)
    figure_height = 16 if has_range_state else (14 if has_window_state else 12)
    fig, axes = plt.subplots(row_count, 1, figsize=(14, figure_height), sharex=True)
    add_enable_background(axes, time, enabled)

    axes[0].plot(time, x_error, color="#1565c0", linewidth=1.0, label="x error")
    axes[0].plot(
        time,
        filtered_x_error,
        color="#ef6c00",
        linewidth=1.5,
        label="filtered x error",
    )
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("Position error [m]")
    axes[0].set_title("Adaptive GS-LQR: enable state and tracking offsets")
    axes[0].legend(loc="upper right")

    axes[1].plot(time, x_dot, color="#6a1b9a", linewidth=1.0, label="x velocity")
    axes[1].plot(
        time,
        filtered_x_dot,
        color="#00897b",
        linewidth=1.5,
        label="filtered x velocity",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].legend(loc="upper right")

    offset_line, = axes[2].plot(
        time,
        1000.0 * data["delta_y_hat"],
        color="#c62828",
        linewidth=1.5,
        label="estimated COM offset",
    )
    if "delta_y_target" in data.dtype.names:
        axes[2].plot(
            time,
            1000.0 * data["delta_y_target"],
            color="#ff8f00",
            linewidth=1.2,
            linestyle="--",
            label="captured COM-offset target",
        )
    if "delta_y_true" in data.dtype.names:
        axes[2].plot(
            time,
            1000.0 * data["delta_y_true"],
            color="black",
            linewidth=1.2,
            linestyle=":",
            label="known injected true offset",
        )
    if "delta_y_applied" in data.dtype.names:
        axes[2].plot(
            time,
            1000.0 * data["delta_y_applied"],
            color="#00838f",
            linewidth=1.1,
            linestyle="--",
            label="offset applied to controller",
        )
    if "delta_y_obs" in data.dtype.names:
        axes[2].plot(
            time,
            1000.0 * data["delta_y_obs"],
            color="#8e24aa",
            linewidth=0.7,
            alpha=0.45,
            label="geometric COM-offset observation",
        )
    if "filtered_delta_y_obs" in data.dtype.names:
        axes[2].plot(
            time,
            1000.0 * data["filtered_delta_y_obs"],
            color="#6a1b9a",
            linewidth=1.2,
            label="filtered geometric observation",
        )
    if "target_updated" in data.dtype.names:
        update_samples = data["target_updated"] >= 0.5
    elif "delta_y_step" in data.dtype.names:
        update_samples = np.abs(data["delta_y_step"]) > 1.0e-12
    else:
        update_samples = np.zeros(len(data), dtype=bool)
    if np.any(update_samples):
        target_values = (
            data["delta_y_target"]
            if "delta_y_target" in data.dtype.names
            else data["delta_y_hat"]
        )
        axes[2].scatter(
            time[update_samples],
            1000.0 * target_values[update_samples],
            color="#ff8f00",
            edgecolors="black",
            linewidths=0.4,
            s=32,
            zorder=4,
            label="captured target update",
        )
    axes[2].axhline(0.0, color="black", linewidth=0.7)
    axes[2].set_ylabel("COM offset [mm]")
    pitch_axis = axes[2].twinx()
    pitch_correction = np.degrees(
        data["theta_eq_adaptive"] - data["theta_eq_nominal"]
    )
    pitch_line, = pitch_axis.plot(
        time,
        pitch_correction,
        color="#455a64",
        linewidth=1.0,
        linestyle="--",
        label="equilibrium pitch correction",
    )
    pitch_axis.set_ylabel("Pitch correction [deg]")
    offset_handles, offset_labels = axes[2].get_legend_handles_labels()
    axes[2].legend(
        offset_handles + [pitch_line],
        offset_labels + [pitch_line.get_label()],
        loc="upper right",
    )

    axes[3].step(
        time,
        enabled.astype(float),
        where="post",
        color="#2e7d32",
        linewidth=1.8,
        label="adaptation enabled",
    )
    axes[3].step(
        time,
        0.75 * gate_open.astype(float),
        where="post",
        color="#f9a825",
        linewidth=1.5,
        label="update gate open (scaled to 0.75)",
    )
    if np.any(updates_paused):
        axes[3].step(
            time,
            0.40 * updates_paused.astype(float),
            where="post",
            color="#6d4c41",
            linewidth=1.4,
            label="estimator hold (scaled to 0.40)",
        )
    axes[3].set_ylim(-0.08, 1.12)
    axes[3].set_yticks([0.0, 0.40, 0.75, 1.0])
    axes[3].set_yticklabels(["off", "hold", "gate", "enabled"])
    axes[3].set_ylabel("Controller state")
    axes[3].legend(loc="upper right")

    if has_window_state:
        window_progress = np.clip(data["window_progress"], 0.0, 1.0)
        cooldown_remaining = np.maximum(data["cooldown_remaining"], 0.0)
        window_line, = axes[4].plot(
            time,
            window_progress,
            color="#0277bd",
            linewidth=1.5,
            label="stable-window progress",
        )
        axes[4].fill_between(
            time, 0.0, window_progress, color="#4fc3f7", alpha=0.18
        )
        axes[4].set_ylim(-0.05, 1.05)
        axes[4].set_ylabel("Window progress")
        cooldown_axis = axes[4].twinx()
        cooldown_line, = cooldown_axis.plot(
            time,
            cooldown_remaining,
            color="#ad1457",
            linewidth=1.2,
            linestyle="--",
            label="cooldown remaining",
        )
        cooldown_axis.set_ylim(bottom=0.0)
        cooldown_axis.set_ylabel("Cooldown [s]")
        axes[4].legend(
            [window_line, cooldown_line],
            [window_line.get_label(), cooldown_line.get_label()],
            loc="upper right",
        )

    if has_range_state:
        axes[5].plot(
            time,
            1000.0 * np.maximum(data["window_position_range"], 0.0),
            color="#6a1b9a",
            linewidth=1.3,
            label="position range collected in current window",
        )
        axes[5].axhline(
            1.0,
            color="#d32f2f",
            linewidth=1.0,
            linestyle="--",
            label="default 1 mm rejection limit",
        )
        axes[5].set_ylim(bottom=0.0)
        axes[5].set_ylabel("Window range [mm]")
        axes[5].legend(loc="upper right")

    axes[-1].set_xlabel("Time [s]")

    enable_patch = Patch(
        facecolor="#66bb6a", alpha=0.18, label="adaptation commanded ON"
    )
    axes[0].legend(handles=axes[0].get_legend_handles_labels()[0] + [enable_patch])

    for axis in axes:
        axis.grid(True, alpha=0.25, linestyle="--")

    enabled_stats = stage_statistics("enabled", enabled, x_error, x_dot)
    disabled_stats = stage_statistics("disabled", ~enabled, x_error, x_dot)
    fig.text(0.01, 0.006, enabled_stats + "\n" + disabled_stats, fontsize=9)
    fig.tight_layout(rect=(0.0, 0.055, 1.0, 1.0))
    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(fig)

    print(enabled_stats)
    print(disabled_stats)
    print(f"图片已保存: {output_path}")


def main():
    script_directory = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="绘制 Adaptive GS-LQR 诊断曲线")
    parser.add_argument(
        "csvfile",
        nargs="?",
        default=os.path.join(script_directory, "adaptive_lqr_log.csv"),
        help="adaptive_lqr_log.csv 的路径",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=os.path.join(script_directory, "adaptive_lqr_analysis.png"),
        help="输出 PNG 路径",
    )
    args = parser.parse_args()

    try:
        data = load_log(args.csvfile)
        plot_log(data, args.output)
    except (FileNotFoundError, ValueError, OSError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
