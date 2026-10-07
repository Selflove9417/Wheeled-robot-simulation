# Round 2 inverse compatibility review

Scope: independent model-only linear-algebra audit of the full precision age-4 ms first failure retained in the round-1 v4 log. No files in the round-1 record were changed. This is not a physical response gate, braking result, or jump datum.

Replayed failure sample: `sim_ms=6885`, state age `4 ms`; `q=[0.65881672026013416, 0.51199177778941907, -0.060680581366889419, 0.26520497206377536, -0.38925489170835548, 0.265204977940106, -0.38925489151770176, -9.0656783425930314, -9.0656783205539373]`; `v=[-0.002317450765813031, 4.0038760744420541e-05, -0.00041518951306884647, 0.00011519310753815741, 1.8866597917510158e-17, 0.00011519308459181988, -5.7061825142311026e-17, 0.035428117664294649, 0.035428117760872818]`; `a_ref=[-0.0067680000000107923, 0.013536000000021585, -0.0067680000000107923, 0.013536000000021585]`; model controller's requested wheel torque supplied to the inverse is `[0.01081191159339033, 0.010811911496812161] Nm`.

These `q/v`, reference and wheel torques are the corrected round-1 offline model trajectory at its model failure time, not a native physical failure sample. Only the replay's initial state seed comes from the frozen old first-motion native record.

## Why the former equal-acceleration/equal-torque system failed

The fixed equations use unknowns `x=[a(9), contact_force(4), leg_torque(4)]`. Nine dynamics equations plus four exact bilateral contact-acceleration equations and four separate leg-acceleration equations use 17 unknowns. The dynamics/contact/exact-four-acceleration 17x17 system has rank 16: one bilateral load-split/actuator-allocation null direction remains. Adding exact hip and knee torque symmetry removes that null direction, but now the 19 equations are slightly inconsistent for the measured left/right state and wheel-torque micro-asymmetry. On the preserved 4 ms failure row, the original system's maximum residual is `1.0088991604824404e-8`, above its unchanged `1e-8` admission limit; the dominant residual is the opposite-sign left/right wheel dynamics pair (`-1.00889916048e-8`, `+1.00889871908e-8`). No friction-cone or rate/position guard is the cause.

## Compatible physical task

Keep raw `q/v`, measured wheel effort, exact dynamics, all four contact acceleration constraints, and exact same left/right hip and knee actuator torques. Replace the four independent exact leg-acceleration equations with two exact common-task mean equations:

`(a_hip_L+a_hip_R)/2 = (a_ref_hip_L+a_ref_hip_R)/2`

`(a_knee_L+a_knee_R)/2 = (a_ref_knee_L+a_ref_knee_R)/2`

Do not force left/right contact-force equality: bilateral load split is solved by the dynamics/task equations. The resulting 17x17 matrix on the same preserved frame has rank 17, `sigma_max=18.1841395473`, `sigma_min=0.01342397136`, and maximum equation residual `2.84e-13`. It yields torques `[-0.576756109, -15.168920635, -0.576756109, -15.168920635] Nm`; contacts `(t,n)` are approximately `(-0.0105412,85.7785)` and `(-0.0105413,85.7785) N`, inside the friction cone. The full 81-mode forward model accepts the torque with all four legs in positive slip mode; acceleration differs from the 17x17 prediction by at most `1.55e-12 rad/s^2`.

The computed leg accelerations are `[-0.006768110746, 0.013535984694, -0.006767889254, 0.013536015306] rad/s^2`. Their left/right differences are `-2.2149e-7` hip and `-3.0612e-8` knee. Gate the actual predicted next state, not acceleration difference alone: require `abs(vL+dt*aL-vR-dt*aR) <= 1e-6 rad/s` and `abs(qL+dt*(vL+dt*aL)-qR-dt*(vR+dt*aR)) <= 1e-6 rad`, preserving the existing symmetry tolerance. The audited prediction has rate gaps `-1.9855e-10/-3.0612e-11 rad/s` and position gaps `-5.8765e-9/-1.9068e-10 rad`. Continue applying all existing per-leg torque, rate, anchor, soft-limit, pitch, wheel, contact-cone and friction-branch guards individually.

## Reproduction artifacts

`audit_inverse_compatibility_round2.cpp`, `audit_inverse_compatibility_round2` and `audit_inverse_compatibility_round2.txt` reproduce the matrix ranks, residual, torque/contact solution and full-81 check. The input constants are the full-precision 4 ms model failure fixture at 6.885 s, including `wheel_tau=[0.01081191159339033,0.010811911496812161]` from the first-failure detail. Earlier exploratory 3 ms audit initially used the CSV's applied `active_wheel` values instead of the distinct newly requested wheel effort; that was corrected by switching this independent audit to the complete 4 ms model log record. Round-1 v4 itself remains immutable.

The offline qualification replay required by round 2 is pending the stage-1 helper update. It must use the helper's real `command_blend_alpha` path and be labeled MODEL ONLY; external torque blending is not evidence that helper admission passed.
