# Round 2 inverse and final-command certificate

## Inverse system

The unknown vector is `x = [a(9), lambda(4), u_leg(4)]`, so there are 17 unknowns. The rows are the 9 generalized dynamics equations, 4 bilateral contact-acceleration equations, 2 mean joint-acceleration tasks, and 2 actuator policy rows `u_hL=u_hR`, `u_kL=u_kR`. This makes a square 17x17 system. Its equal-torque rows are the explicit branch/load policy that closes the inverse problem; the two contact loads per side remain free and are solved by the physical contact equations. There is no minimum-norm or arbitrary normal-load split.

The model receives the original 9-vector `q` and `v`; no state is averaged or projected. Only the task reference is expressed as a hip and a knee mean. The solution retains separate left/right accelerations. The helper records each pair's acceleration, next-rate, and next-position difference and rejects if `dt*abs(delta_accel)`, `abs(delta_rate)`, or `abs(delta_position)` exceeds the existing `1e-6` synchronization tolerance. It also applies the existing per-joint rate, anchor, soft-limit, pitch, effort, contact-cone, finiteness, and residual guards to the final response.

For incremental mobility, the system is the 13x13 constrained KKT matrix `[M -J^T; J 0]`: 9 dynamics and 4 contact rows for 9 accelerations and 4 physical contact forces. It imposes no left/right force equality. Each unit task perturbation is applied to both actuators of the corresponding pair, and mobility is the measured mean hip/knee acceleration response.

## Final friction certification

After PD and the requested command blend are applied, the exact final leg torque and recorded wheel torque are passed to the existing, unchanged `ground_friction_response` enumerator. Its 81 stick/slip assignments solve static reaction as an unknown bounded force and check sliding direction, contact cone, and the existing equation/contact residuals. The helper rejects absent or non-unique acceleration solutions and enforces the unchanged absolute `1e-8` equation/contact residual gate on the result. It does not infer final friction from the inverse reference's zero-friction member, and it does not iterate a sign selected from a tiny predicted velocity.

This full-81 implementation is a correctness baseline. It has not been connected to the controller; measured runtime does not grant integration or physical-trial approval. ROS/controller scheduling, full COM closed-loop replay, real-time fault injection, and a native physical trial remain not tested.

## Offline results

The saved stationary first-motion anchor now passes alpha 0, 0.25, 0.5, and 1. The final-command certificate selects a single all-stick response for each blend and reports static reactions of `0.065329485180110433`, `0.048997113394964972`, `0.032664742590444669`, and `4.5751361527415768e-9 Nm`. The test makes a separate full-forward call and obtains zero acceleration difference at printed precision; equation and contact residuals are below `5.8e-13`. Each full-mode solve reports one candidate acceleration. The inverse and forward calculations are distinct mathematical formulations, but both final-command certificate and test reference call the same existing `ground_friction_response` implementation; this is a self-consistency check, not an independent second implementation. The full-precision rows are in `static_blend_witness.csv`.

The corrected 4 ms first-failure fixture uses the requested wheel command from `audit_inverse_compatibility_round2.cpp`. It now gives rank 17/17, minimum singular value `0.013424`, maximum `18.1841`, equation residual `2.84217e-13`, contact residual `2.77482e-13`, and all-positive slip modes. The independent full-81 response matches the inverse acceleration to `1.54969e-12`; hip and knee left/right acceleration differences are `-2.21492e-7` and `-3.06119e-8 rad/s^2`, with one-step predicted rate and position differences below the existing `1e-6` synchronization bound. This is an isolated model fixture, not a complete COM closed-loop replay result.

The targeted binary passed. Its `-O2` 100-call mixed benchmark (moving/slip, static blend, asymmetric slip fixtures) reported mean `3174.44 µs`, p50 `3167.86 µs`, p95 `3196.38 µs`, max `3217.94 µs`. The helper evaluates all 81 friction assignments per final command. These measurements exclude ROS scheduling, and the max leaves little room in a 5 ms control period; no live-path approval follows.

Source and artifact hashes at this result: helper `f08266531b8c18a61d8b24e3aeaa4ead0fc0cb322c3105731f668c9203f03b8d`; targeted test `17a37ba376da80da24c3b575dedde5e544a470036b81241e050c7891b80fdcf6`; binary `/tmp/test_ground_contact_motion_control` `be9d692d3db3a23c96270ec72aeb33d94c5e668d42ee698bbd3537beddd84488`; alpha CSV `84408031554851380a836340b634ca56f2895dd6b7eb782900f72fa7d1e5a8ac`. The round-1 files were preserved unchanged under `frozen_round1/` with their original hashes recorded in the preservation snapshot.
