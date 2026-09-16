#!/usr/bin/env python3
"""Reproduce the deployed LQR design model in Python, validate it against
the gain-table nodes compiled into adaptive_lqr_balance_controller.cpp,
and export the closed-loop spectral radius sweep used by paper Fig. 4 (rho_sweep.csv).

Model chain (paper Sec. 2, planar sagittal model, all angles absolute from
+Z toward +Y):
    axle --[l1 shank]--> knee --[l2 thigh]--> hip,  hip target = [d_y, H]
COM synthesis uses the exact URDF joint-frame transformation chain; inertias use
the URDF ixx (about the wheel axis) with the parallel-axis theorem.

Multi-rate closed-loop modeling:
- 200 Hz (Ts = 5 ms) inner control loop and wheel encoder velocity measurement.
- 100 Hz (2 Ts = 10 ms) IMU measurement with sample-and-hold (S&H) on odd steps.
- First-order discrete low-pass filters:
    alpha_x = 0.05 on linear velocity x_dot
    alpha_theta = 0.10 on pitch rate theta_dot
- Exact 2-step lifted monodromy transition matrix Phi in R^(8x8) over 10 ms period.
- Spectral radius: rho_multirate = sqrt(max |eig(Phi)|).

Outputs (next to this script's data_logs directory):
  rho_sweep.csv
"""

import argparse
import os
import sys
import numpy as np
from scipy.linalg import expm, solve_discrete_are, eigvals

# ---------------------------------------------------------------------------
# Plant physical parameters (matching bbot.urdf.xacro and kinematics.cpp)
# ---------------------------------------------------------------------------
R_W = 0.07                     # wheel radius (m)
L1 = 0.34325                   # shank length (m)
L2 = 0.30000                   # thigh length (m)
D_Y = 0.01137221               # hip y-offset ahead of axle (m)

M_BODY = 9.50                  # base_link (kg)
M_THIGH = 1.20                 # one thigh (kg)
M_SHANK = 0.80                 # one shank (kg)
M_WHEEL = 2.00                 # one wheel (kg)
J_WHEEL = 0.006481             # wheel ixx (kg m^2)
J_BODY = 0.159013 * (9.5 / 14.0)
J_THIGH = 0.017921
J_SHANK = 0.013130
G = 9.81

M_B = M_BODY + 2 * M_THIGH + 2 * M_SHANK          # 13.5 kg
M1 = M_B + 2 * M_WHEEL + 2 * J_WHEEL / (R_W ** 2)

TS = 0.005                     # 200 Hz (s)

# Exact URDF vectors in link frames:
V_KNEE_HIP = np.array([-0.29348091, -0.06220095])
C_THIGH = np.array([-0.13690699, -0.02116697])
V_AXLE_KNEE = np.array([0.28210870, -0.19553796])
C_SHANK = np.array([0.11538205, -0.08532288])
C_BODY = np.array([0.13261282 - 0.125, 0.05396677 + 0.07])

# Chapter 2 baseline LQR weighting
Q0 = np.diag([100.0, 5000.0, 3000.0, 1200.0])
R0 = 1.0

# Deployed gain tables (must strictly match adaptive_lqr_balance_controller.cpp)
LEGACY_SAFE_TABLE = [
    (0.30, -5.622712, -42.666506, -156.508986, -35.846746, -0.0276264, 0.3592154),
    (0.35, -5.791278, -43.976682, -169.544125, -39.563240, -0.0258066, 0.4016316),
    (0.40, -5.931603, -45.087129, -181.972969, -43.390225, -0.0234056, 0.4443432),
    (0.45, -6.052411, -46.061241, -193.948669, -47.349665, -0.0203401, 0.4872441),
    (0.50, -6.157159, -46.922551, -205.518217, -51.430573, -0.0164269, 0.5302655),
]

# OPTIMIZED_V2_TABLE: updated when Milestone C optimization runs
OPTIMIZED_V2_TABLE = [
    (0.30, -5.622712, -42.666506, -156.508986, -35.846746, -0.0276264, 0.3592154),
    (0.35, -5.791278, -43.976682, -169.544125, -39.563240, -0.0258066, 0.4016316),
    (0.40, -5.931603, -45.087129, -181.972969, -43.390225, -0.0234056, 0.4443432),
    (0.45, -6.052411, -46.061241, -193.948669, -47.349665, -0.0203401, 0.4872441),
    (0.50, -6.157159, -46.922551, -205.518217, -51.430573, -0.0164269, 0.5302655),
]


def rot(delta):
    c, s = np.cos(delta), np.sin(delta)
    return np.array([[c, -s], [s, c]])


def ik_leg(height):
    dZ_down = height
    target_dY = -D_Y
    d_sq = target_dY ** 2 + dZ_down ** 2
    d = np.sqrt(d_sq)
    cos_gamma = (L2 ** 2 + L1 ** 2 - d_sq) / (2.0 * L2 * L1)
    gamma = np.arccos(np.clip(cos_gamma, -1.0, 1.0))
    theta_d = np.arctan2(target_dY, dZ_down)
    cos_psi = (L2 ** 2 + d_sq - L1 ** 2) / (2.0 * L2 * d)
    psi = np.arccos(np.clip(cos_psi, -1.0, 1.0))
    phi1 = theta_d - psi
    phi2 = phi1 + (np.pi - gamma)
    phi1_0 = np.arctan2(-0.29348091, 0.06220095)
    phi2_0 = np.arctan2(0.28210870, 0.19553796)
    q_hip = phi1 - phi1_0
    q_knee = (phi2 - phi1) - (phi2_0 - phi1_0)
    return q_hip, q_knee


def com_at(height):
    q_hip, q_knee = ik_leg(height)
    r_shank = rot(q_hip + q_knee)
    r_thigh = rot(q_hip)
    knee = - r_shank @ V_AXLE_KNEE
    hip = knee - r_thigh @ V_KNEE_HIP
    p_shank = knee + r_shank @ C_SHANK
    p_thigh = hip + r_thigh @ C_THIGH
    p_body = hip + C_BODY
    com = (M_BODY * p_body + 2 * M_THIGH * p_thigh + 2 * M_SHANK * p_shank) / M_B
    return com, (p_body, p_thigh, p_shank)


def inertia_at(com, pts):
    p_body, p_thigh, p_shank = pts
    d2 = lambda p: float(np.sum((np.asarray(p) - com) ** 2))
    return (J_BODY + M_BODY * d2(p_body)
            + 2 * (J_THIGH + M_THIGH * d2(p_thigh))
            + 2 * (J_SHANK + M_SHANK * d2(p_shank)))


def design_matrices(height):
    com, pts = com_at(height)
    l = np.linalg.norm(com)
    inertia = inertia_at(com, pts)
    m2 = inertia + M_B * (l ** 2)
    m3 = M_B * l
    d = M1 * m2 - (m3 ** 2)
    mgl = M_B * G * l
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -m3 * mgl / d, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, M1 * mgl / d, 0.0],
    ])
    B = np.array([
        [0.0],
        [(m2 / R_W + m3) / d],
        [0.0],
        [-(M1 + m3 / R_W) / d],
    ])
    return A, B, com, l, inertia


def discretize(A, B, ts=TS):
    n, m = A.shape[0], B.shape[1]
    aug = np.zeros((n + m, n + m))
    aug[:n, :n] = A
    aug[:n, n:] = B
    E = expm(aug * ts)
    return E[:n, :n], E[:n, n:]


def dare_gain(Ad, Bd, Q=Q0, R=R0):
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad).flatten()
    return K


def interp_table(table, height):
    hs = [r[0] for r in table]
    if height <= hs[0]:
        return np.array(table[0][1:5]), table[0][5], table[0][6]
    if height >= hs[-1]:
        return np.array(table[-1][1:5]), table[-1][5], table[-1][6]
    for i in range(len(table) - 1):
        if hs[i] <= height <= hs[i + 1]:
            ratio = (height - hs[i]) / (hs[i + 1] - hs[i])
            K = (1.0 - ratio) * np.array(table[i][1:5]) + ratio * np.array(table[i + 1][1:5])
            yc = (1.0 - ratio) * table[i][5] + ratio * table[i + 1][5]
            zc = (1.0 - ratio) * table[i][6] + ratio * table[i + 1][6]
            return K, yc, zc
    return np.array(table[0][1:5]), table[0][5], table[0][6]


def multirate_monodromy(Ad, Bd, K):
    ax, ath = 0.05, 0.10
    M0 = np.zeros((8, 8))
    u0_from_x0 = -np.array([K[0], K[1] * ax, K[2], K[3] * ath])
    u0_from_xf = -K[1] * (1.0 - ax)
    u0_from_thf = -K[3] * (1.0 - ath)
    M0[:4, :4] = Ad + Bd @ u0_from_x0.reshape(1, 4)
    M0[:4, 4] = Bd.flatten() * u0_from_xf
    M0[:4, 7] = Bd.flatten() * u0_from_thf
    M0[4, 1] = ax; M0[4, 4] = 1.0 - ax
    M0[5, 2] = 1.0; M0[6, 3] = 1.0
    M0[7, 3] = ath; M0[7, 7] = 1.0 - ath

    M1 = np.zeros((8, 8))
    u1_from_x1 = -np.array([K[0], K[1] * ax, 0.0, 0.0])
    u1_from_xf0 = -K[1] * (1.0 - ax)
    u1_from_th_imu = -K[2]
    u1_from_thd_imu = -K[3] * ath
    u1_from_thf0 = -K[3] * (1.0 - ath)
    M1[:4, :4] = Ad + Bd @ u1_from_x1.reshape(1, 4)
    M1[:4, 4] = Bd.flatten() * u1_from_xf0
    M1[:4, 5] = Bd.flatten() * u1_from_th_imu
    M1[:4, 6] = Bd.flatten() * u1_from_thd_imu
    M1[:4, 7] = Bd.flatten() * u1_from_thf0
    M1[4, 1] = ax; M1[4, 4] = 1.0 - ax
    M1[5, 5] = 1.0; M1[6, 6] = 1.0
    M1[7, 6] = ath; M1[7, 7] = 1.0 - ath
    return M1 @ M0


def multirate_spectral_radius_and_damping(Ad, Bd, K):
    Phi = multirate_monodromy(Ad, Bd, K)
    evs = eigvals(Phi)
    rho = float(np.sqrt(np.max(np.abs(evs))))
    min_damping = 1.0
    for val in evs:
        if abs(val) < 1e-6:
            continue
        s = np.log(val) / (2.0 * TS)
        freq_hz = abs(s.imag) / (2.0 * np.pi)
        if 6.0 <= freq_hz <= 12.0:
            damp = float(-s.real / abs(s)) if abs(s) > 1e-6 else 0.0
            if damp < min_damping:
                min_damping = damp
    return rho, min_damping


def verify_table_reproduction(table, name="legacy_safe"):
    print(f"\n=== Node-by-node reproduction check [{name}] ===")
    worst_yz = 0.0
    for row in table:
        h, kx, kxd, kth, kthd, y_ref, z_ref = row
        com, _ = com_at(h)
        ey = com[0] - y_ref
        ez = com[1] - z_ref
        worst_yz = max(worst_yz, abs(ey), abs(ez))
        print(f"H={h:.2f}  y_err={ey:+.2e} m  z_err={ez:+.2e} m  "
              f"K=[{kx:.2f}, {kxd:.2f}, {kth:.2f}, {kthd:.2f}]")
    print(f"worst |y/z| error = {worst_yz:.3e} m (tolerance 1e-6 m)")
    ok = worst_yz < 1.0e-6
    print(f"[{'PASS' if ok else 'FAIL'}] Analytical geometry matches deployed table")
    return ok


def run_sweep(table, out_path):
    print("\n=== Spectral radius sweep: H = 0.300..0.500 m, 1 mm grid ===")
    midpoint_K = np.array(table[2][1:5])
    rows = []
    max_rho_sched_ideal = (0.0, 0.0)
    max_rho_fix_ideal = (0.0, 0.0)
    max_rho_sched_mr = (0.0, 0.0)
    max_rho_fix_mr = (0.0, 0.0)

    for h in np.arange(0.300, 0.500 + 1e-9, 0.001):
        h_round = round(float(h), 3)
        A, B, *_ = design_matrices(h_round)
        Ad, Bd = discretize(A, B)

        Ks, *_ = interp_table(table, h_round)
        Kf = midpoint_K

        rho_s_ideal = float(np.max(np.abs(eigvals(Ad - Bd @ Ks.reshape(1, -1)))))
        rho_f_ideal = float(np.max(np.abs(eigvals(Ad - Bd @ Kf.reshape(1, -1)))))

        rho_s_mr, damp_s = multirate_spectral_radius_and_damping(Ad, Bd, Ks)
        rho_f_mr, damp_f = multirate_spectral_radius_and_damping(Ad, Bd, Kf)

        rows.append((h_round, rho_s_ideal, rho_f_ideal, rho_s_mr, rho_f_mr, damp_s, damp_f))

        if rho_s_ideal > max_rho_sched_ideal[1]:
            max_rho_sched_ideal = (h_round, rho_s_ideal)
        if rho_f_ideal > max_rho_fix_ideal[1]:
            max_rho_fix_ideal = (h_round, rho_f_ideal)
        if rho_s_mr > max_rho_sched_mr[1]:
            max_rho_sched_mr = (h_round, rho_s_mr)
        if rho_f_mr > max_rho_fix_mr[1]:
            max_rho_fix_mr = (h_round, rho_f_mr)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("hip_height_m,rho_scheduled,rho_fixed,rho_sched_filtered,rho_fixed_filtered,"
                "damp_6_12_sched,damp_6_12_fixed\n")
        for r in rows:
            f.write("%.3f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f\n" % r)

    print(f"Ideal:      max rho sched = {max_rho_sched_ideal[1]:.4f} at H={max_rho_sched_ideal[0]:.3f} m")
    print(f"Ideal:      max rho fixed = {max_rho_fix_ideal[1]:.4f} at H={max_rho_fix_ideal[0]:.3f} m")
    print(f"Multi-rate: max rho sched = {max_rho_sched_mr[1]:.4f} at H={max_rho_sched_mr[0]:.3f} m")
    print(f"Multi-rate: max rho fixed = {max_rho_fix_mr[1]:.4f} at H={max_rho_fix_mr[0]:.3f} m")
    print(f"Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="legacy_safe", choices=["legacy_safe", "optimized_v2"])
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    table = OPTIMIZED_V2_TABLE if args.profile == "optimized_v2" else LEGACY_SAFE_TABLE
    ok = verify_table_reproduction(table, args.profile)

    here = os.path.dirname(os.path.abspath(__file__))
    out_path = args.output or os.path.normpath(
        os.path.join(here, "..", "src", "data_logs", "rho_sweep.csv")
    )
    run_sweep(table, out_path)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
