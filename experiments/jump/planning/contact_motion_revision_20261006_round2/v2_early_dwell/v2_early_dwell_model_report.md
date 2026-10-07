# Ground contact motion revision: offline round 2

This round independently audited the inverse-task equations, then ran one frozen model replay with the revised helper. All results are **MODEL ONLY**. The replay starts from the first already-moving row of the old native record; the later `q/v` samples and stop outcomes are model-generated, not physical observations. The wheel and COM clock chain is an explicit simulation assumption, not a reproduction of asynchronous ROS/Gazebo histories. This does not pass a physical response gate or certify real braking, real handoff/startup, or jumps.

## Inverse compatibility

The old inverse fixes all four individual leg accelerations and adds exact left/right equal hip/knee torques. Dynamics9 + contact4 + leg-acceleration4 form 17 equations in 17 unknowns but have rank 16: bilateral load split leaves one actuator-allocation null direction. The two exact torque-equality rows remove that null direction, while making the 19-row system slightly incompatible with the recorded model state. At the full-precision 4 ms round-1 model fixture the max residual is `1.00889916048e-8`, just above the unchanged `1e-8` gate; opposing wheel-dynamics residuals dominate. No limit was widened.

The revised physical task keeps raw per-side `q/v`, exact fixed-model dynamics, all four bilateral contact constraints, exact equal hip/knee actuator torques, and exact mean hip/knee acceleration tasks. It leaves contact load split free. A separate full-precision audit gives rank 17, `sigma_max=18.1841395473`, `sigma_min=0.01342397136`, and residual `2.84e-13`. The chosen leg accelerations are `[-0.006768110746, 0.013535984694, -0.006767889254, 0.013536015306] rad/s^2` for model-input `a_ref=[-0.006768,0.013536,-0.006768,0.013536]`. Exact symmetric leg torques are `[-0.576756109,-15.168920635,-0.576756109,-15.168920635] Nm`; both contact normals are `85.7785 N` with tangent loads about `-0.01054 N`, inside the friction cone. The independent full 81-mode forward response is valid, all four final leg modes are positive slip, and its acceleration differs by at most `1.55e-12 rad/s^2`.

The resulting left/right hip and knee acceleration differences are `-2.2149e-7` and `-3.0612e-8 rad/s^2`. The helper guards the actual predicted next side-to-side rate and position gaps against the existing `1e-6` symmetry tolerance; the fixture's gaps are `-1.9855e-10/-3.0612e-11 rad/s` and `-5.8765e-9/-1.9068e-10 rad`. The individual per-leg torque, rate, anchor, soft-limit, pitch, wheel, contact-cone, and final friction-branch guards remain active.

The exact full-precision fixture and independent matrix/full-81 audit are in [inverse_compatibility_review.md](inverse_compatibility_review.md) and [audit_inverse_compatibility_round2.txt](audit_inverse_compatibility_round2.txt).

## Frozen model replay result

The helper now receives the true smoothstep `command_blend_alpha` and first-row baseline leg effort, returns the final blended leg torque, and checks that actual final torque with the full 81-mode friction response. There is no external second blend. The frozen v4 old helper exploration remains untouched and is only a comparison: it stopped the first two model trials then rejected trial 3 because its old 19x17 inverse was incompatible. Round 2 uses the corrected square task and passes all four model trials at both state ages.

| State age | Whole model replay | Trial stop time | Peak leg rate | Peak pitch delta | Minimum soft margin | Maximum anchor error | Peak leg / wheel torque |
|---:|---|---:|---:|---:|---:|---:|---:|
| 3 ms | PASS, 8,579 physics steps | 0.581 s each | 0.020210 rad/s | 0.003725 rad | 1.170746 rad | 0.0199995 rad | 15.5051 / 0.011555 Nm |
| 4 ms | PASS, 8,575 physics steps | 0.580 s each | 0.020249 rad/s | 0.003725 rad | 1.170745 rad | 0.0200004 rad | 15.5052 / 0.011546 Nm |

All four trials in both runs recorded `[200,200,200,200]` hip/knee excitation steps at 3 ms. At 4 ms, trial 1 has `[200,200,200,200]`; trials 2–4 have `[200,199,200,199]`. Every trial is recorded as `MODEL_STOPPED` after the required dwell. Per-trial release costs (wheel absolute effort integral and baseline leg-effort deviation integral) are retained in `contact_com_round2_summary.json`; for trials 1–4 they are respectively `0.030604/0.813579`, `0.028477/0.529599`, `0.029923/0.813978`, `0.028458/0.529887` at 3 ms, and `0.030582/0.811819`, `0.028440/0.529517`, `0.029912/0.812855`, `0.028432/0.529539` at 4 ms.

The same-model comparison between helper predicted acceleration and the 1 ms full-81 replay has RMS errors (base forward, base vertical, pitch, left hip, left knee, left wheel) of `6.14e-5, 3.28e-4, 1.77e-3, 1.90e-3, 1.27e-3, 7.51e-3` at 3 ms and `7.09e-5, 3.79e-4, 1.97e-3, 2.23e-3, 1.46e-3, 7.55e-3` at 4 ms. These compare two uses of the same fixed model and do not represent independent physical residuals.

## Clocks and reproducibility

The first input row supplies the original initial wheel torque (`0.00899719 Nm` per side), initial wheel command (`0.00434578 m/s`), old baseline hold torque, and native anchor `q/v`. At each 5 ms model control tick, the helper gets q/v selected at 3 or 4 ms source age and the wheel torque requested by the COM law at that tick. COM uses the actual controller functions `centroidal_balance_state(..., body_mass=9.5)` and `centroidal_catch_target(..., 0.07, 5.0, 0.0)`, world COM from base Y/Z plus native pitch rotation, and +Y forward heading. `WorldPoseVelocity` is seeded by a 20 ms q/v model extrapolation and updated/held every 20 ms. Wheel target P gain is 1, clamped to +/-30 rad/s, torque to +/-10 Nm, and target slew to 8 m/s^2. The requested command takes effect 1 ms later in the modeled physics loop.

The clock sidecars list local and absolute source/control/apply timestamps, source age, observer-update time/hold age, requested model wheel torque, baseline and final leg torque, alpha, and exact state/reference values for each control tick. The absolute clock is `native_anchor.source_ns + replay-relative time`; it does not turn the simulated samples into native samples.

Replay artifacts are [contact_com_round2_summary.json](contact_com_round2_summary.json), the full 1 ms state/forward traces `contact_com_round2_stateage3.csv` and `contact_com_round2_stateage4.csv`, their `.clock.csv` sidecars, [contact_com_round2.log](contact_com_round2.log), and [frozen_replay_manifest.json](frozen_replay_manifest.json). The manifest freezes source, executable, helper, model, friction model, input, and protocol hashes before the single replay invocation. No ROS or Gazebo was run. This model-only PASS is not authorization or qualification for a ground run.
