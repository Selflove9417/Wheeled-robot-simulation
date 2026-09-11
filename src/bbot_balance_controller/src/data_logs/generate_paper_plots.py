#!/usr/bin/env python3
"""Generate the two key paper figures directly from adaptive GS-LQR CSV logs."""

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/bbot_matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
DEADBAND_MM = 5.0


def load_csv(name):
    path = HERE / name
    data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")
    if data.ndim == 0:
        data = np.array([data], dtype=data.dtype)
    return data[np.argsort(data["time"])]


def tail_mean_mm(data, field="x_error", duration=10.0):
    mask = data["time"] >= data["time"][-1] - duration
    return 1000.0 * float(np.mean(data[field][mask]))


def reset_time(data):
    changes = np.flatnonzero(np.abs(np.diff(data["x_ref"])) > 1.0e-6) + 1
    if not len(changes):
        raise ValueError("No Space/reset_position event found in log")
    return float(data["time"][changes[0]])


def convergence_time(data):
    """Time after reset from which filtered error remains inside the 5 mm band."""
    t0 = reset_time(data)
    post = data[data["time"] >= t0]
    outside = np.flatnonzero(np.abs(post["filtered_x_error"]) > DEADBAND_MM / 1000.0)
    first_persistent = int(outside[-1] + 1) if len(outside) else 0
    if first_persistent >= len(post):
        raise ValueError("Log ends before persistent deadband entry")
    return float(post["time"][first_persistent] - t0)


def update_count(data):
    t0 = reset_time(data)
    post = data[data["time"] >= t0]
    return int(np.count_nonzero(np.abs(post["delta_y_step"]) > 1.0e-12))


def save_figure(fig, stem):
    fig.savefig(HERE / f"{stem}.png", dpi=300, bbox_inches="tight")
    fig.savefig(HERE / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


# The nominal sensitivity metric removes the measured zero-bias residual:
# |Delta e_x,ss| = |e_x,ss(b_y) - e_x,ss(0)|. The resulting physical sensitivity
# fit is constrained through the origin.
nominal_files = {
    0.00: "adaptive_nominal_zero.csv",
    0.25: "adaptive_nominal_pos025.csv",
    0.50: "adaptive_nominal_pos05.csv",
    1.00: "adaptive_nominal_pos10.csv",
}
nominal_x = np.array(sorted(nominal_files))
nominal_signed_error = np.array(
    [tail_mean_mm(load_csv(nominal_files[x])) for x in nominal_x]
)
nominal_y = np.abs(nominal_signed_error - nominal_signed_error[0])
fit_mask = nominal_x > 0.0
fit_slope = float(
    np.dot(nominal_x[fit_mask], nominal_y[fit_mask])
    / np.dot(nominal_x[fit_mask], nominal_x[fit_mask])
)
fit_pred = fit_slope * nominal_x[fit_mask]
fit_residual = np.sum((nominal_y[fit_mask] - fit_pred) ** 2)
fit_total = np.sum((nominal_y[fit_mask] - np.mean(nominal_y[fit_mask])) ** 2)
fit_r2 = 1.0 - fit_residual / fit_total


# Only +1.00 mm currently has independent repeats. A singleton is not assigned
# a zero uncertainty; the plot explicitly reports its sample count instead.
adaptive_files = {
    0.25: ["adaptive_online_pos_morefast025.csv"],
    0.50: ["adaptive_online_pos_morefast05.csv"],
    1.00: [
        "adaptive_online_pos_morefast10.csv",
        "adaptive_online_pos_morefast10_run2.csv",
        "adaptive_online_pos_morefast10_run3.csv",
    ],
}
adaptive_x = np.array(sorted(adaptive_files))
tc_samples = {
    x: np.array([convergence_time(load_csv(name)) for name in adaptive_files[x]])
    for x in adaptive_x
}
tc_mean = np.array([np.mean(tc_samples[x]) for x in adaptive_x])
tc_std = np.array(
    [np.std(tc_samples[x], ddof=1) if len(tc_samples[x]) > 1 else np.nan for x in adaptive_x]
)
tc_n = np.array([len(tc_samples[x]) for x in adaptive_x])
updates = np.array(
    [int(round(np.mean([update_count(load_csv(name)) for name in adaptive_files[x]]))) for x in adaptive_x]
)


def draw_sensitivity(ax, panel_title=None):
    ax.plot(
        nominal_x,
        nominal_y,
        "o-",
        color="#d62728",
        linewidth=2.0,
        markersize=7,
        label="Nominal GS-LQR",
        zorder=3,
    )
    fit_x = np.linspace(0.0, 1.05, 100)
    ax.plot(
        fit_x,
        fit_slope * fit_x,
        "--",
        color="#d62728",
        alpha=0.55,
        linewidth=1.3,
        label=rf"Sensitivity fit: $k_s={fit_slope:.2f}$ mm/mm, $R^2={fit_r2:.4f}$",
    )
    for x, y in zip(nominal_x, nominal_y):
        offset = (8, 14) if x < 1.0 else (-48, 14)
        ax.annotate(
            f"{y:.2f} mm", (x, y), xytext=offset,
            textcoords="offset points", color="#d62728", fontsize=9,
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.82, "pad": 1.2},
            zorder=5,
        )
    ax.set_title(panel_title or r"Nominal Model-Mismatch Sensitivity: $|b_y|$ vs. $|\Delta e_{x,ss}|$")
    ax.set_xlabel(r"Injected COM-y Bias Magnitude $|b_y|$ (mm)")
    ax.set_ylabel(r"Incremental Steady-State Position Drift $|\Delta e_{x,ss}|$ (mm)")
    ax.set_xlim(-0.05, 1.10)
    ax.set_ylim(0.0, 75.0)
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="upper left", fontsize=9)


def draw_convergence(ax, panel_title=None):
    ax.plot(
        adaptive_x, tc_mean, "o-", color="#1f77b4", linewidth=2.2,
        markersize=7, label="Fast adaptive GS-LQR", zorder=3
    )
    for x, mean, count_updates in zip(adaptive_x, tc_mean, updates):
        offset = (8, -20) if x < 0.75 else (-78, 10)
        ax.annotate(
            f"{mean:.2f} s\n{count_updates} updates", (x, mean),
            xytext=offset, textcoords="offset points", color="#1f77b4", fontsize=9
        )
    ax.set_title(panel_title or r"Adaptive Convergence Time: $|b_y|$ vs. $T_c$")
    ax.set_xlabel(r"Injected COM-y Bias Magnitude $|b_y|$ (mm)")
    ax.set_ylabel(r"Persistent-Deadband Entry Time $T_c$ (s)")
    ax.set_xlim(0.15, 1.10)
    ax.set_ylim(20.0, 120.0)
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="upper left", fontsize=9)


plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.labelsize": 10.5,
    "axes.titleweight": "bold",
})

fig1, ax1 = plt.subplots(figsize=(7.2, 5.2))
draw_sensitivity(ax1)
fig1.tight_layout()
save_figure(fig1, "sensitivity_by_vs_ex_ss")

fig2, ax2 = plt.subplots(figsize=(7.2, 5.2))
draw_convergence(ax2)
fig2.tight_layout()
save_figure(fig2, "convergence_by_vs_tc")

combined, axes = plt.subplots(1, 2, figsize=(13.0, 5.0))
draw_sensitivity(axes[0], r"(a) Nominal Sensitivity: $|b_y|$ vs. $|\Delta e_{x,ss}|$")
draw_convergence(axes[1], r"(b) Adaptive Convergence: $|b_y|$ vs. $T_c$")
combined.tight_layout()
save_figure(combined, "paper_by_sensitivity_and_convergence")

print(f"Nominal fit: slope={fit_slope:.6f} mm/mm, R2={fit_r2:.9f}")
for x in adaptive_x:
    values = tc_samples[x]
    if len(values) == 1:
        print(f"|b_y|={x:.2f} mm: Tc={values[0]:.3f} s (n=1)")
    else:
        print(
            f"|b_y|={x:.2f} mm: Tc={np.mean(values):.3f} ± "
            f"{np.std(values, ddof=1):.3f} s (n={len(values)})"
        )
