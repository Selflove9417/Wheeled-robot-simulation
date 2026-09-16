#!/usr/bin/env python3
"""
Exhaustive comparison between legacy_safe and candidate DARE optimizations.
"""

import numpy as np
import verify_lqr_model_and_sweep as v

HEIGHTS = [0.30, 0.35, 0.40, 0.45, 0.50]

print("=== Theoretical Profile Comparison ===")
# Profile 1: legacy_safe
print("\n--- 1. legacy_safe (current deployed conservative) ---")
for row in v.LEGACY_SAFE_TABLE:
    h = row[0]
    K = np.array(row[1:5])
    A, B, *_ = v.design_matrices(h)
    Ad, Bd = v.discretize(A, B)
    Phi = v.multirate_monodromy(Ad, Bd, K)
    evs = np.linalg.eigvals(Phi)
    rho_mr = float(np.sqrt(np.max(np.abs(evs))))
    for val in evs:
        if abs(val) < 1e-6: continue
        s = np.log(val) / (2.0 * v.TS)
        f_hz = abs(s.imag) / (2.0 * np.pi)
        if 5.0 <= f_hz <= 14.0:
            d = float(-s.real / abs(s)) if abs(s) > 1e-6 else 0.0
            print(f"  H={h:.2f} m: K=[{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}, {K[3]:.2f}] | rho_mr={rho_mr:.4f}, zeta_8hz={d:.4f} at {f_hz:.2f} Hz")
            break

# Profile 2: Aggressive table (which failed in earlier experiments with 8 Hz limit cycle)
# H=0.30: [-6.86, -55.47, -190.94, -46.60]
AGGRESSIVE_K = [
    (0.30, [-6.86, -55.47, -190.94, -46.60]),
    (0.35, [-6.90, -56.00, -200.00, -50.00]),
    (0.40, [-7.00, -57.00, -210.00, -53.00]),
    (0.45, [-7.10, -58.00, -220.00, -56.00]),
    (0.50, [-7.20, -59.00, -230.00, -59.00]),
]
print("\n--- 2. Aggressive table (known limit cycle at H=0.30m) ---")
for h, K in AGGRESSIVE_K:
    K = np.array(K)
    A, B, *_ = v.design_matrices(h)
    Ad, Bd = v.discretize(A, B)
    Phi = v.multirate_monodromy(Ad, Bd, K)
    evs = np.linalg.eigvals(Phi)
    rho_mr = float(np.sqrt(np.max(np.abs(evs))))
    for val in evs:
        if abs(val) < 1e-6: continue
        s = np.log(val) / (2.0 * v.TS)
        f_hz = abs(s.imag) / (2.0 * np.pi)
        if 5.0 <= f_hz <= 14.0:
            d = float(-s.real / abs(s)) if abs(s) > 1e-6 else 0.0
            print(f"  H={h:.2f} m: K=[{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}, {K[3]:.2f}] | rho_mr={rho_mr:.4f}, zeta_8hz={d:.4f} at {f_hz:.2f} Hz")
            break

# Profile 3: DARE candidate with Q = diag([50, 2000, 1000, 150])
print("\n--- 3. DARE Candidate (Enhanced Damping zeta=0.074~0.083) ---")
Q_cand = np.diag([50.0, 2000.0, 1000.0, 150.0])
for h in HEIGHTS:
    A, B, *_ = v.design_matrices(h)
    Ad, Bd = v.discretize(A, B)
    K = v.dare_gain(Ad, Bd, Q_cand, 1.0)
    Phi = v.multirate_monodromy(Ad, Bd, K)
    evs = np.linalg.eigvals(Phi)
    rho_mr = float(np.sqrt(np.max(np.abs(evs))))
    for val in evs:
        if abs(val) < 1e-6: continue
        s = np.log(val) / (2.0 * v.TS)
        f_hz = abs(s.imag) / (2.0 * np.pi)
        if 5.0 <= f_hz <= 14.0:
            d = float(-s.real / abs(s)) if abs(s) > 1e-6 else 0.0
            print(f"  H={h:.2f} m: K=[{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}, {K[3]:.2f}] | rho_mr={rho_mr:.4f}, zeta_8hz={d:.4f} at {f_hz:.2f} Hz")
            break

