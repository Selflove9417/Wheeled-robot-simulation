import numpy as np
import verify_lqr_model_and_sweep as v

for i, row in enumerate(v.LEGACY_SAFE_TABLE):
    h = row[0]
    K_leg = np.array(row[1:5])
    A, B, *_ = v.design_matrices(h)
    Ad, Bd = v.discretize(A, B)
    Phi = v.multirate_monodromy(Ad, Bd, K_leg)
    evs = np.linalg.eigvals(Phi)
    rho_mr = float(np.sqrt(np.max(np.abs(evs))))
    for val in evs:
        if abs(val) < 1e-4: continue
        s = np.log(val)/(2*v.TS)
        f = abs(s.imag)/(2*np.pi)
        if 5.0 <= f <= 14.0:
            d = -s.real/abs(s) if abs(s)>1e-6 else 0.0
            print(f"H={h:.2f} m: K_leg={K_leg.round(2)} | zeta={d:.4f} at {f:.2f} Hz")
            break
