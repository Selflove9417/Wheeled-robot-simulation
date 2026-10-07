# Proposed timing coverage for the certified fast model

**Proposal only; not run.** Root review is required after the fast controller is certified. V4's two full81 model cases (state age 3/4 ms, joint and wheel publication delay 1 ms) are the only timing runs completed in this directory.

## Candidate matrix

Use the unchanged V4 already-moving anchor, COM capture law, profile-2 plant, C2 references, 2 s blend, all caps, dwell, output ownership, and contact/friction policies. Sweep the four state-age values with leg and wheel publication delays independently at both endpoints:

| State age (ms) | Leg publication delay (ms) | Wheel publication delay (ms) |
|---:|---:|---:|
| 1, 3, 4, 10 | 1 or 5 | 1 or 5 |

This is a 4 × 2 × 2 = 16-case factorial, including equal-delay pairs and both asymmetric corners (leg 1/wheel 5 and leg 5/wheel 1). Age means the selected q/v snapshot age at a 5 ms control update. Each output channel gets its own fixed queue due time; the helper's wheel torque must continue to come from the selected snapshot's applied BeforePhysics input, never the newly requested wheel output. Keep the COM observer's source clock/hold age distinct from the selected state age and record all four clocks (control, selected state, wheel input application, and command apply) per row.

For each case, retain all 1 ms plant samples and every 5 ms controller row, including inverse FF, implicit PD correction, alpha, baseline and final torque; predicted and forward acceleration; friction/contact modes, cone and rate diagnostics; stop q/v/axle anchor at the 1.0 s normal-stop trigger; full-step extrema; reverse-motion samples; and stop completion. Use the same 0.25 s quiet dwell and 2 s timeout. A first violation is a failure record; do not change gains, limits, residual thresholds or protocol to make the candidate pass.

## Suggested model cross-check after root approval

Run all 16 cases on the certified fast model, then select the most adverse candidate rows/trials by minimum contact margin, maximum equation/forward-prediction residual, maximum anchor/pitch/rate value, and maximum full-stop displacement. Compare those states and final torques to the full81 baseline at the same state, wheel input, reference, alpha and actuator timing. Require exact friction-mode/input agreement where applicable and preserve any disagreement as a blocking discrepancy. The timing matrix is a model-design screen only; it does not test asynchronous ROS delivery, native sensing, live mode supervision or fault recovery.

The requested top age is 10 ms. The separately observed repeated-value age ambiguity up to 13.8 ms is outside this proposal and requires an explicit root decision before any claim that the matrix bounds live acquisition age.
