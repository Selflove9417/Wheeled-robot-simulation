# Round 3 contact-COM closed-loop replay

**Disposition: model-design replay PASS for the two simulated state-age cases; no physical-run admission.** This run starts from the first already-moving sample of the old replay input. It is not a standing-start replay and is not evidence of a real landing, contact recovery, braking response, or sensor/actuator timing.

The frozen executable ran once for each case, with 9,275 1 ms profile-2 plant steps per case. The COM reference was recomputed from the controller's centroidal capture law (`centroidal_balance_state(..., body_mass=9.5)` and `centroidal_catch_target(..., 0.07, 5.0, 0)`) using +Y as forward. The replay updates/holds its WorldPoseVelocity observer at 20 ms, runs the wheel P=1 servo each 5 ms (target ±30 rad/s, torque ±10 Nm and target slew 8 m/s²), and applies output after 1 ms. The helper receives the selected delayed snapshot's simulated BeforePhysics wheel torque; the newly requested COM-P wheel torque remains a separate, later command. The helper receives its actual command blend alpha and the first-row baseline torque, so final blending is inside the helper and its result is checked against the full 81-mode response.

Both state-age cases completed the two-second blend, guarded handoff settle and all four alternating C2 trials. Each trial had 0.5 s acceleration, 0.5 s cruise and 0.5 s normal deceleration; quiet-stop dwell was sampled at 5 ms ticks and stopped after 0.255 s from the normal-stop trigger. This is 0.25 s plus one control tick, not an early dwell. All trials satisfied their stopping/excitation checks. No failure was observed in the frozen model replay.

| State age | Trial | Stop after normal-stop trigger (s) | Excitation samples (4 tasks) | Max leg rate (rad/s) | Min soft margin (rad) | Max anchor error (rad) | Max pitch delta (rad) | Max leg torque (Nm) | Max wheel torque (Nm) | Joint stop span (rad) | Reverse excursion (rad) | Release cost: wheel (Nm·s) | Release cost: leg baseline delta (Nm·s) |
|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3 ms | 1 | 0.255 | 200/200/200/200 | 0.020217 | 1.170746 | 0.0199997 | 0.003705 | 15.5050 | 0.011563 | 0.00498121 | 0 | 0.0340471 | 0.855263 |
| 3 ms | 2 | 0.255 | 200/199/200/199 | 0.020218 | 1.170758 | 0.0199879 | 0.003605 | 15.4832 | 0.010840 | 0.00498122 | 0 | 0.0312089 | 0.570471 |
| 3 ms | 3 | 0.255 | 200/199/200/199 | 0.020218 | 1.170746 | 0.0199996 | 0.003721 | 15.5049 | 0.011565 | 0.00498118 | 0 | 0.0336047 | 0.857148 |
| 3 ms | 4 | 0.255 | 200/199/200/199 | 0.020217 | 1.170758 | 0.0199877 | 0.003615 | 15.4834 | 0.010845 | 0.00498121 | 0 | 0.0312091 | 0.571471 |
| 4 ms | 1 | 0.255 | 200/200/200/200 | 0.020263 | 1.170745 | 0.0200007 | 0.003707 | 15.5052 | 0.011545 | 0.00497907 | 0 | 0.0340478 | 0.860049 |
| 4 ms | 2 | 0.255 | 200/199/200/199 | 0.020263 | 1.170758 | 0.0199878 | 0.003602 | 15.4834 | 0.010833 | 0.00497908 | 0 | 0.0312022 | 0.572238 |
| 4 ms | 3 | 0.255 | 200/199/200/199 | 0.020264 | 1.170745 | 0.0200006 | 0.003724 | 15.5051 | 0.011545 | 0.00497901 | 0 | 0.0336114 | 0.858909 |
| 4 ms | 4 | 0.255 | 200/199/200/199 | 0.020263 | 1.170758 | 0.0199875 | 0.003611 | 15.4830 | 0.010837 | 0.00497909 | 0 | 0.0312031 | 0.573745 |

`stop_joint_range_conservative_span_rad` is the joint range over the stop interval; it is not rebound. The separately measured model reverse excursion is zero in these trials. Release costs are the two separate model integrals defined in the JSON summary, not a combined score.

## Qualification limits

The replay is a fixed-model design gate, not the independent response gate in the frozen protocol: forward and inverse share the same model. It does not reproduce asynchronous ROS, native IMU/odom/joint acquisition, or native actuator delivery. The 3/4 ms cases are assumed state ages, not measured live sensor ages. The later root review found observed publication-change delays vary by phase (about 1–5 ms) and repeated-value age can be ambiguous up to 13.8 ms. Those live timing cases are therefore not covered by these two PASS results. Do not use this result alone to authorize a physical run; additional timing coverage awaits a certified fast candidate and an approved comparison plan.

The previously archived `v2_early_dwell` result remains intact and exploratory; its early dwell was superseded by this control-tick protocol replay. No ROS/Gazebo or physical run was performed, and no old record was changed.

## Artifacts and frozen inputs

- Executable/source: `audit_contact_com_closed_loop_round3` and `audit_contact_com_closed_loop_round3.cpp`.
- Full model state traces: `contact_com_round3_stateage3.csv`, `contact_com_round3_stateage4.csv`.
- 5 ms control-clock sidecars, including state age, observer clock, requested-vs-selected wheel torques, inverse feedforward, implicit-PD correction, alpha, baseline, final blend and validity: corresponding `.clock.csv` files.
- Machine-readable results and one-line run log: `contact_com_round3_summary.json`, `contact_com_round3.log`.
- Pre-run hashes and replay assumptions: `frozen_replay_manifest.json`; `SHA256SUMS` covers the V3 source, executable and generated outputs.

The manifest freezes source SHA-256 `824bd2b275699b1f47e3128e1e2ca8f44c2b05696dcced8006955727dcb64d1e`, executable `fab001c629dacca859d8aa34708566c78bb17a07d6c36dde55881a2fdf9e20d6`, helper `f08266531b8c18a61d8b24e3aeaa4ead0fc0cb322c3105731f668c9203f03b8d`, model/protocol/input hashes and the one-invocation run policy.
