# V4 corrected stop-metric replay

**Disposition: both offline cases completed and passed the frozen model protocol; physical-run admission remains NO.** This is a MODEL ONLY replay from the old input's already-moving first anchor. It provides no standing-start, real landing, sensor-timing, actuator-transport, braking-response, or jump evidence. No ROS/Gazebo or physical run occurred (physical runs: 0).

The run used the frozen profile-2 full81 forward contact/friction model at 1 ms, the fixed COM capture law, 20 ms held observer, 5 ms controller, and 1 ms publication delay. Controller state ages were 3 and 4 ms. The helper received the selected model snapshot's actual BeforePhysics wheel torque; the new COM-P wheel request remained separate and was queued for its 1 ms apply time. The full output from each 5 ms controller tick includes inverse feedforward torque, the implicit-PD correction that is passed into output, alpha, baseline, and final helper torque.

For each trial, stop metrics now begin at the 1.0 s normal-deceleration trigger using current physical-model q/v and axle state. Joint and axle extrema accumulate from every 1 ms model sample through quiet completion. The reverse displacement is measured opposite each joint's trigger velocity direction. At 1.5 s only the 5 ms quiet-dwell counter resets; it does not reset the distance anchors. Every plant sample, including any interval where an opposing command is active while the actual joint continues along its initial motion direction, remains in the CSV. The reported full stop interval is 0.755 s: 0.5 s deceleration plus 0.255 s from the end of the reference deceleration to the quiet-dwell completion.

Each execution completed both age cases with 9,275 1 ms samples per case, 1,854 5 ms controller samples, all helper/FF/PD/final validity flags true, and four stopped C2 trials. There were no physics-model or stopping failures.

| State age | Trial | Full interval from 1.0 s trigger (s) | Max leg rate (rad/s) | Min soft margin (rad) | Max joint anchor error (rad) | Max pitch delta (rad) | Max leg torque (Nm) | Max wheel torque (Nm) | Full-stop max joint displacement (rad) | Reverse displacement (rad) | Axle displacement (m) | Release cost wheel (Nm·s) | Release cost leg baseline delta (Nm·s) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3 ms | 1 | 0.755 | 0.020217 | 1.170746 | 0.0199997 | 0.003705 | 15.5050 | 0.011563 | 0.00492124 | 0 | 0.00170424 | 0.0340471 | 0.855263 |
| 3 ms | 2 | 0.755 | 0.020218 | 1.170758 | 0.0199879 | 0.003605 | 15.4832 | 0.010840 | 0.00492125 | 0 | 0.00133004 | 0.0312089 | 0.570471 |
| 3 ms | 3 | 0.755 | 0.020218 | 1.170746 | 0.0199996 | 0.003721 | 15.5049 | 0.011565 | 0.00492121 | 0 | 0.00164291 | 0.0336047 | 0.857148 |
| 3 ms | 4 | 0.755 | 0.020217 | 1.170758 | 0.0199877 | 0.003615 | 15.4834 | 0.010845 | 0.00492124 | 0 | 0.00132702 | 0.0312091 | 0.571471 |
| 4 ms | 1 | 0.755 | 0.020263 | 1.170745 | 0.0200007 | 0.003707 | 15.5052 | 0.011545 | 0.00489911 | 0 | 0.00170612 | 0.0340478 | 0.860049 |
| 4 ms | 2 | 0.755 | 0.020263 | 1.170758 | 0.0199878 | 0.003602 | 15.4834 | 0.010833 | 0.00489912 | 0 | 0.00132781 | 0.0312022 | 0.572238 |
| 4 ms | 3 | 0.755 | 0.020264 | 1.170745 | 0.0200006 | 0.003724 | 15.5051 | 0.011545 | 0.00489905 | 0 | 0.00164504 | 0.0336114 | 0.858909 |
| 4 ms | 4 | 0.755 | 0.020263 | 1.170758 | 0.0199875 | 0.003611 | 15.4830 | 0.010837 | 0.00489913 | 0 | 0.00132490 | 0.0312031 | 0.573745 |

Release costs are two separate model effort integrals: absolute wheel torque, and absolute per-joint deviation from the first-row baseline torque. They are not combined into a score. The joint/axle displacement is a full-stop excursion from the 1.0 s physical anchor, not rebound.

## Torque curves and trace verification

The plots show four joints with the helper's inverse feedforward, actual implicit-PD correction, and final blended torque. They are generated from normalized sidecars and labeled MODEL ONLY:

- [State age 3 ms torque components](contact_com_round4_torque_components_stateage3.png)
- [State age 4 ms torque components](contact_com_round4_torque_components_stateage4.png)

The frozen emitter has a clock-CSV header-order defect: its header places FF/PD/final columns before q/v/reference fields, while the writer serializes q/v/reference/dwell first and then FF/PD/final. Thus 49 of the 78 raw header labels do not identify the values at those positions. Raw `.clock.csv` files remain unchanged and must not be parsed by their header. The derived `.clock_aligned.csv` files follow the actual frozen writer order; [clock_writer_index_mapping.json](clock_writer_index_mapping.json) provides the machine-readable old-index-to-writer-field mapping.

The alignment checks in the per-age `trace_validation.json` files pass: 78 columns in each stream; q/v equal the 1 ms plant trace exactly at each recorded selected-state timestamp (max difference 0); q/v/a references equal the controller-time plant trace exactly (max difference 0); all helper, inverse, PD, and final validity flags are true; `final=(1-alpha)*baseline+alpha*(inverse_FF+implicit_PD)` has zero measured residual; and every 1 ms active leg torque matches the latest helper output whose 1 ms publication delay has elapsed (zero mismatches and zero torque residual). Plots use these aligned values, not raw header names.

The execution status is recorded in [run_attempts.json](run_attempts.json). Both the first invocation and the retry completed. The first tool response was initially misread because the nested command's session metadata had not been printed; the on-disk log and summaries show it did complete. The retry was allowed to complete and is preserved; its raw traces, summary, and logs are byte-identical to the first. No third execution was made.

## Freeze and qualification limits

The pre-run freeze is [frozen_replay_manifest.json](frozen_replay_manifest.json) and [SHA256SUMS](SHA256SUMS). It records source, executable, input, protocol and 46 controller-include-tree file hashes. After the run, the only include-tree hash mismatch is `ground_contact_motion_fast_certificate.hpp`, timestamped after the frozen executable was built. The V4 source's transitive include closure does not include that file; the actual helper/model inputs remained frozen. Source and executable hashes still match the pre-run manifest.

Both completed model cases use a fixed forward model for the response comparison. They do not recreate live asynchronous acquisition or publication timing. Root has separately held physical admission because real-time mode monitoring and finite fault recovery remain incomplete. Do not use the PASS here to authorize a physical ground run.
