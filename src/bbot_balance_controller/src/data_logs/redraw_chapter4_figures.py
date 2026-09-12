#!/usr/bin/env python3
"""Redraw Chapter 4 figures directly from the recorded Gazebo CSV logs.

The script intentionally keeps all numerical curves tied to the source logs.
It produces SVG/PDF vector files and a 300 dpi PNG preview for each figure.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/bbot_chapter4_matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
DEADBAND_MM = 5.0

BLUE = "#3569B9"
RED = "#D64A3A"
GREEN = "#188977"
GRAY = "#5C6670"
GRID = "#D8DEE7"


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": ["Liberation Serif", "DejaVu Serif"],
            "font.weight": "normal",
            "mathtext.fontset": "custom",
            "mathtext.rm": "Liberation Serif",
            "mathtext.it": "Liberation Serif:italic",
            "mathtext.bf": "Liberation Serif:bold",
            "mathtext.cal": "Liberation Serif:italic",
            "mathtext.sf": "Liberation Sans",
            "mathtext.tt": "Liberation Mono",
            "font.size": 9.0,
            "axes.labelsize": 9.5,
            "axes.titlesize": 10.0,
            "axes.titleweight": "normal",
            "axes.labelweight": "normal",
            "axes.linewidth": 0.8,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.major.width": 0.8,
            "ytick.major.width": 0.8,
            "legend.fontsize": 8.0,
            "lines.linewidth": 1.55,
            "savefig.facecolor": "white",
            "figure.facecolor": "white",
            # Outline SVG text so browser/office renderers cannot reflow math
            # glyphs or substitute fonts; the PDF remains searchable/editable.
            "svg.fonttype": "path",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def load_csv(name: str) -> np.ndarray:
    data = np.genfromtxt(HERE / name, delimiter=",", names=True, encoding="utf-8")
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)
    return data[np.argsort(data["time"])]


def reset_relative(data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return relative time and mask beginning at the recorded position reset."""
    changes = np.flatnonzero(np.abs(np.diff(data["x_ref"])) > 1.0e-6) + 1
    if not len(changes):
        raise ValueError("No position-reset event was found in the log")
    t0 = float(data["time"][changes[0]])
    mask = data["time"] >= t0
    return data["time"][mask] - t0, mask


def tail_mean_mm(data: np.ndarray, field: str = "filtered_x_error", duration: float = 15.0) -> float:
    t, mask = reset_relative(data)
    values = 1000.0 * data[field][mask]
    return float(np.mean(values[t >= t[-1] - duration]))


def first_state_time(data: np.ndarray, state: int) -> float:
    t, mask = reset_relative(data)
    states = data["adapt_state"][mask].astype(int)
    indices = np.flatnonzero(states == state)
    if not len(indices):
        raise ValueError(f"State {state} was not reached")
    return float(t[indices[0]])


def save_all(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f"{stem}.svg", bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.png", dpi=600, bbox_inches="tight")
    plt.close(fig)


def style_axis(ax: plt.Axes) -> None:
    ax.grid(True, color=GRID, linewidth=0.65, linestyle="--", dashes=(3, 3), zorder=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def draw_figure4(output_dir: Path) -> None:
    cases = [
        (-0.50, "adaptive_nominal_neg05.csv"),
        (0.00, "adaptive_nominal_zero.csv"),
        (+0.25, "adaptive_nominal_pos025.csv"),
        (+0.50, "adaptive_nominal_pos05.csv"),
        (+1.00, "adaptive_nominal_pos10.csv"),
    ]
    zero_ss = tail_mean_mm(load_csv("adaptive_nominal_zero.csv"))
    x = np.array([bias for bias, _ in cases])
    y = np.array([tail_mean_mm(load_csv(name)) - zero_ss for _, name in cases])

    nonzero = np.abs(x) > 1.0e-12
    slope = float(np.dot(x[nonzero], y[nonzero]) / np.dot(x[nonzero], x[nonzero]))
    prediction = slope * x
    r2 = 1.0 - float(np.sum((y - prediction) ** 2) / np.sum((y - np.mean(y)) ** 2))

    fig, ax = plt.subplots(figsize=(6.45, 3.75), constrained_layout=True)
    style_axis(ax)
    ax.axhline(0.0, color=GRAY, linewidth=0.85, zorder=1)
    ax.axvline(0.0, color=GRAY, linewidth=0.85, zorder=1)

    fit_x = np.linspace(-0.58, 1.08, 200)
    ax.plot(
        fit_x,
        slope * fit_x,
        color=BLUE,
        linestyle="--",
        linewidth=1.5,
        label=rf"Origin-constrained fit: $\Delta e_{{s,ss}}={slope:.2f}b_y$, $R^2={r2:.4f}$",
        zorder=2,
    )
    ax.scatter(
        x,
        y,
        s=40,
        facecolor="white",
        edgecolor=RED,
        linewidth=1.7,
        label="Steady-state simulation data",
        zorder=3,
    )

    annotation_offsets = {
        -0.50: (7, 7),
        +0.00: (7, 7),
        +0.25: (7, 7),
        +0.50: (7, 7),
        +1.00: (-7, 7),
    }
    for bias, drift in zip(x, y):
        align = "right" if bias == 1.0 else "left"
        ax.annotate(
            f"({bias:+.2f}, {drift:+.2f})",
            (bias, drift),
            xytext=annotation_offsets[float(bias)],
            textcoords="offset points",
            fontsize=7.8,
            color=GRAY,
            ha=align,
            va="bottom",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 1.0, "pad": 0.8},
            zorder=5,
        )

    ax.set_xlabel(r"Injected model bias $b_y$ (mm)")
    ax.set_ylabel(r"Incremental steady-state position drift $\Delta e_{s,ss}$ (mm)")
    ax.set_xlim(-0.62, 1.12)
    ax.set_ylim(-80, 45)
    ax.set_xticks([-0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0])
    ax.legend(loc="lower left", frameon=True, framealpha=0.95, edgecolor="#C7CED8")
    save_all(fig, output_dir, "fig4_bias_vs_drift")


def draw_figure5(output_dir: Path) -> None:
    single = load_csv("adaptive_online_state_machine_pos100.csv")
    two_stage = load_csv("adaptive_twostage_pos100.csv")
    t_single, m_single = reset_relative(single)
    t_two, m_two = reset_relative(two_stage)
    e_single = 1000.0 * single["filtered_x_error"][m_single]
    e_two = 1000.0 * two_stage["filtered_x_error"][m_two]
    dy_single = 1000.0 * single["delta_y_applied"][m_single]
    dy_two = 1000.0 * two_stage["delta_y_applied"][m_two]

    test_single = first_state_time(single, 1)
    test_two = first_state_time(two_stage, 1)
    fine_start = first_state_time(two_stage, 2)
    tc_single = first_state_time(single, 3)
    tc_two = first_state_time(two_stage, 5)

    fig, (ax1, ax_phase, ax2) = plt.subplots(
        3,
        1,
        figsize=(7.15, 5.45),
        sharex=True,
        gridspec_kw={"height_ratios": [2.05, 0.18, 1.0], "hspace": 0.055},
    )
    fig.subplots_adjust(left=0.11, right=0.985, bottom=0.105, top=0.98)
    style_axis(ax1)
    style_axis(ax2)

    ax1.axhspan(-DEADBAND_MM, DEADBAND_MM, color="#CDEBE3", alpha=0.75, label="Position deadband $\pm5$ mm", zorder=0)
    ax1.axhline(0.0, color=GRAY, linewidth=0.8, zorder=1)
    ax1.plot(t_single, e_single, color=RED, linestyle="--", label="Single-stage state machine", zorder=2)
    ax1.plot(t_two, e_two, color=BLUE, label="Two-stage state machine", zorder=3)

    i_single = int(np.argmin(e_single))
    i_two = int(np.argmin(e_two))
    ax1.scatter(t_single[i_single], e_single[i_single], s=22, color=RED, zorder=4)
    ax1.scatter(t_two[i_two], e_two[i_two], s=22, color=BLUE, zorder=4)
    ax1.annotate(
        f"Peak: {abs(e_single[i_single]):.2f} mm",
        (t_single[i_single], e_single[i_single]),
        xytext=(12, -2),
        textcoords="offset points",
        color=RED,
        fontsize=8,
        va="top",
    )
    ax1.annotate(
        f"Peak: {abs(e_two[i_two]):.2f} mm",
        (t_two[i_two], e_two[i_two]),
        xytext=(10, 8),
        textcoords="offset points",
        color=BLUE,
        fontsize=8,
    )

    ax1.axvline(tc_two, color=BLUE, linewidth=1.0, linestyle=":")
    ax1.axvline(tc_single, color=RED, linewidth=1.0, linestyle=":")
    ax1.text(tc_two + 0.8, -92, f"Two-stage HOLD\n$T_c$ = {tc_two:.2f} s", color=BLUE, fontsize=8)
    ax1.text(tc_single - 1.2, -92, f"Single-stage convergence\n$T_c$ = {tc_single:.2f} s", color=RED, fontsize=8, ha="right")
    ax1.set_ylabel(r"Position error $e_s$ (mm)")
    ax1.set_ylim(-700, 35)
    ax1.legend(loc="lower right", frameon=True, framealpha=0.95, edgecolor="#C7CED8")

    ax_phase.set_ylim(0, 1)
    ax_phase.set_xlim(0, 80)
    ax_phase.axvspan(0.0, fine_start, color="#DDE9F7", alpha=1.0)
    ax_phase.axvspan(fine_start, tc_two, color="#F8E7C8", alpha=1.0)
    ax_phase.axvspan(tc_two, 80.0, color="#DDF0E4", alpha=1.0)
    ax_phase.axvline(fine_start, color="white", linewidth=1.0)
    ax_phase.axvline(tc_two, color="white", linewidth=1.0)
    ax_phase.text(fine_start / 2.0, 0.5, "Stage 1: fast coarse capture", color="#315B8A", fontsize=7.4, ha="center", va="center")
    ax_phase.text((fine_start + tc_two) / 2.0, 0.5, "Stage 2: slow fine tuning", color="#9A681F", fontsize=7.4, ha="center", va="center")
    ax_phase.text((tc_two + 80.0) / 2.0, 0.5, "HOLD", color="#38744A", fontsize=7.4, ha="center", va="center")
    ax_phase.tick_params(left=False, labelleft=False, bottom=False, labelbottom=False)
    for spine in ax_phase.spines.values():
        spine.set_visible(False)
    ax2.axhline(-10.0, color=GRAY, linewidth=1.0, linestyle=":", label=r"Ideal compensation $-b_y=-10$ mm")
    ax2.plot(t_single, dy_single, color=RED, linestyle="--", label="Single-stage compensation")
    ax2.plot(t_two, dy_two, color=BLUE, label="Two-stage compensation")
    ax2.axvline(test_two, color=BLUE, linewidth=0.9, linestyle=":")
    ax2.axvline(test_single, color=RED, linewidth=0.9, linestyle=":")
    ax2.annotate(
        f"Two-stage estimate\n$T_{{est}}$ = {test_two:.2f} s",
        (test_two, 0.0),
        xytext=(8, -18),
        textcoords="offset points",
        color=BLUE,
        fontsize=7.5,
        va="top",
        bbox={"facecolor": "white", "edgecolor": "#C7CED8", "linewidth": 0.4, "alpha": 1.0, "pad": 1.2},
        arrowprops={"arrowstyle": "-", "color": BLUE, "linewidth": 0.6, "shrinkA": 3, "shrinkB": 3},
        zorder=5,
    )
    ax2.annotate(
        f"Single-stage first estimate\n$T_{{est}}$ = {test_single:.2f} s",
        (test_single, 0.0),
        xytext=(10, -49),
        textcoords="offset points",
        color=RED,
        fontsize=7.5,
        va="top",
        bbox={"facecolor": "white", "edgecolor": "#C7CED8", "linewidth": 0.4, "alpha": 1.0, "pad": 1.2},
        arrowprops={"arrowstyle": "-", "color": RED, "linewidth": 0.6, "shrinkA": 3, "shrinkB": 3},
        zorder=5,
    )
    ax2.set_ylabel(r"Applied compensation $\Delta y_{\rm applied}$ (mm)")
    ax2.set_xlabel("Simulation time after reset (s)")
    ax2.set_xlim(0, 80)
    ax2.set_ylim(-12.2, 0.6)
    ax2.set_yticks([0, -5, -10])
    ax2.legend(loc="upper right", ncol=1, frameon=True, framealpha=1.0, edgecolor="#C7CED8", fontsize=7.2)
    save_all(fig, output_dir, "fig5_single_vs_twostage_pos100")


def _payload_series(name: str) -> tuple[np.ndarray, np.ndarray]:
    data = load_csv(name)
    t, mask = reset_relative(data)
    return t, 1000.0 * data["filtered_x_error"][mask]


def _tail_mean(t: np.ndarray, values: np.ndarray, duration: float = 15.0) -> float:
    return float(np.mean(values[t >= t[-1] - duration]))


def _compact_metric(ax: plt.Axes, x: float, y: float, text: str, color: str, xytext: tuple[float, float]) -> None:
    ax.scatter([x], [y], s=18, color=color, zorder=4)
    ax.annotate(text, (x, y), xytext=xytext, textcoords="offset points", color=color, fontsize=7.7)


def draw_figure6(output_dir: Path) -> None:
    front_nom_t, front_nom = _payload_series("payload_P1_front_nominal.csv")
    front_adp_t, front_adp = _payload_series("payload_P2_front_adaptive.csv")
    rear_nom_t, rear_nom = _payload_series("payload_P3_rear_nominal.csv")
    rear_adp_t, rear_adp = _payload_series("payload_P4_rear_adaptive.csv")

    fig = plt.figure(figsize=(7.25, 5.55))
    gs = fig.add_gridspec(
        2,
        2,
        height_ratios=[2.35, 1.0],
        left=0.105,
        right=0.985,
        bottom=0.105,
        top=0.91,
        hspace=0.12,
        wspace=0.12,
    )
    full_front = fig.add_subplot(gs[0, 0])
    full_rear = fig.add_subplot(gs[0, 1], sharey=full_front)
    zoom_front = fig.add_subplot(gs[1, 0])
    zoom_rear = fig.add_subplot(gs[1, 1], sharey=zoom_front)

    for ax in (full_front, full_rear, zoom_front, zoom_rear):
        style_axis(ax)
        ax.axhspan(-DEADBAND_MM, DEADBAND_MM, color="#CDEBE3", alpha=0.75, zorder=0)
        ax.axhline(0.0, color=GRAY, linewidth=0.75, zorder=1)

    full_front.plot(front_nom_t, front_nom, color=RED, linestyle="--", label="Nominal GS-LQR")
    full_front.plot(front_adp_t, front_adp, color=BLUE, label="Adaptive GS-LQR")
    full_rear.plot(rear_nom_t, rear_nom, color=RED, linestyle="--")
    full_rear.plot(rear_adp_t, rear_adp, color=BLUE)

    full_front.set_title(r"(a) Front payload: $m_p=1.0$ kg, $\Delta y_p=+0.10$ m")
    full_rear.set_title(r"(b) Rear payload: $m_p=1.0$ kg, $\Delta y_p=-0.10$ m")
    full_front.set_ylabel(r"Position error $e_s$ (mm)")
    full_front.set_xlim(0, 80)
    full_rear.set_xlim(0, 80)
    full_front.set_ylim(-360, 700)
    full_front.tick_params(labelbottom=False)
    full_rear.tick_params(labelbottom=False, labelleft=False)

    front_nom_ss = _tail_mean(front_nom_t, front_nom)
    rear_nom_ss = _tail_mean(rear_nom_t, rear_nom)
    _compact_metric(full_front, float(front_nom_t[-1]), float(front_nom[-1]), f"Steady state: {front_nom_ss:+.2f} mm", RED, (-86, -14))
    i = int(np.argmax(front_adp))
    _compact_metric(full_front, float(front_adp_t[i]), float(front_adp[i]), f"Peak: {front_adp[i]:.2f} mm", BLUE, (6, 7))
    _compact_metric(full_rear, float(rear_nom_t[-1]), float(rear_nom[-1]), f"Steady state: {rear_nom_ss:+.2f} mm", RED, (-92, 8))
    i = int(np.argmin(rear_adp))
    _compact_metric(full_rear, float(rear_adp_t[i]), float(rear_adp[i]), f"Peak: {abs(rear_adp[i]):.2f} mm", BLUE, (7, -13))

    zoom_front.plot(front_adp_t, front_adp, color=BLUE)
    zoom_rear.plot(rear_adp_t, rear_adp, color=BLUE)
    zoom_front.set_xlim(28, 80)
    zoom_rear.set_xlim(28, 80)
    zoom_front.set_ylim(-8.5, 8.5)
    zoom_front.set_ylabel(r"Steady-state detail $e_s$ (mm)")
    zoom_front.set_xlabel("Simulation time after reset (s)")
    zoom_rear.set_xlabel("Simulation time after reset (s)")
    zoom_rear.tick_params(labelleft=False)

    front_ss = _tail_mean(front_adp_t, front_adp)
    rear_ss = _tail_mean(rear_adp_t, rear_adp)
    zoom_front.text(0.97, 0.12, f"Steady state: {front_ss:+.2f} mm", transform=zoom_front.transAxes, ha="right", color=GREEN, fontsize=8)
    zoom_rear.text(0.97, 0.12, f"Steady state: {rear_ss:+.2f} mm", transform=zoom_rear.transAxes, ha="right", color=GREEN, fontsize=8)
    zoom_front.text(0.02, 0.88, r"$\pm5$ mm deadband", transform=zoom_front.transAxes, color=GREEN, fontsize=7.7)

    handles = [
        plt.Line2D([0], [0], color=RED, linestyle="--", label="Nominal GS-LQR"),
        plt.Line2D([0], [0], color=BLUE, label="Adaptive GS-LQR"),
        plt.Rectangle((0, 0), 1, 1, facecolor="#CDEBE3", edgecolor="none", label=r"Position deadband $\pm5$ mm"),
    ]
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.55, 0.995),
        ncol=3,
        frameon=False,
        handlelength=2.7,
    )
    save_all(fig, output_dir, "fig6_payload_comparison")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=HERE / "chapter4_figures",
        help="Directory for SVG, PDF, and PNG outputs",
    )
    args = parser.parse_args()
    configure_style()
    draw_figure4(args.output_dir)
    draw_figure5(args.output_dir)
    draw_figure6(args.output_dir)
    print(f"Saved Chapter 4 figures to {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
