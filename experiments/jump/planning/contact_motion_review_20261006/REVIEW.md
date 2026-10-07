# Contact-motion reachability and inverse-interface review

Date: 2026-10-06  
Scope: read-only source/data review. No controller, model, record, build, or simulation was changed or run.

## Conclusion

A new interface should treat the four leg trajectories as references under bilateral rolling contact, not as independent accelerations that can be imposed alongside arbitrary body pitch, body height, forward motion, and wheel motion. The nine-coordinate planar model has four contact-acceleration constraints, leaving five independent generalized acceleration directions while bilateral support persists. The body and wheel accelerations are therefore consequences of the selected leg references, wheel-servo torque, and constrained dynamics. Pitch and height can remain feedback objectives, but must be expressed as prioritized tasks with feasibility slack when all four leg references are active.

Symmetric left/right contact does not by itself prove a rank deficiency. The two normal rows share base-z and pitch columns, but retain side-specific hip/knee columns in the current Jacobian; the rolling rows also have separate wheel-relative columns. At ordinary geometry the 4-row contact Jacobian should be full row rank. A configuration can still make it ill-conditioned or redundant if those side-specific normal derivatives vanish or if a reduced model collapses both normal rows. The implementation must report the Jacobian singular values/rank at the measured anchor and fail closed on deficient or poorly conditioned solves, instead of assuming either full rank or a fixed normal-force split.

## What the existing path actually commands

`ground_motion_probe_reference` generates position, velocity, and acceleration references for four leg joints. The live support path passes position and velocity to the leg effort controller, with support feedforward and discrete PD. Although the reference includes acceleration, the existing publisher does not use it as acceleration feedforward. The stage-two path therefore tests reference tracking through a bounded torque controller; it does not command a requested four-joint acceleration exactly. The prior probe's 484 knee threshold samples and zero hip threshold samples are direct evidence that the generated reference was not realized equally by all joints. The measured stop was useful but does not establish four-joint moving reachability.

The existing support effort combines leg gravity/support feedforward, joint PD, and torso feedback, with hip/knee effort limits. It is not an inverse dynamics solve for simultaneous leg, base, wheel, and contact tasks. Preserve that distinction in any new report: desired acceleration, inverse-feasible acceleration, published effort, and native pre-Physics `JointForceCmd` are separate signals.

The wheel path publishes velocity targets on `/diff_drive_controller/cmd_vel`; a separate servo converts those targets and measured wheel rates to bounded torque (`gain * (target_rate - measured_rate)`, clipped to ±10 Nm). Target wheel rate is not wheel torque. The servo log provides target/rate/torque and source validity, while the native engine's before-step `JointForceCmd` remains the authoritative physical input for the dynamics audit. Match the servo torque to native input with source and wall-time bounds; do not replace the native input with a target or infer identity from a repeated value.

## Reachability and rank accounting

The model coordinate order is `[base-forward, base-z, pitch-axis, hipL, kneeL, hipR, kneeR, wheelL-relative, wheelR-relative]`. It has nine accelerations. Bilateral wheel support imposes two rolling/tangential and two normal acceleration constraints, `J a + b = 0`; thus at a regular contact pose the instantaneous allowable acceleration space is five-dimensional.

This does not make four low-speed leg references inherently impossible. If the four leg accelerations are fixed and the two bounded-P wheel torques are treated as known inputs, the dynamics plus four contact equations can determine the remaining base/wheel response, four leg efforts, and contact forces. What cannot also be independently hard-prescribed is every body and wheel acceleration. For example, adding exact base-z and pitch acceleration targets to four exact leg accelerations supplies six acceleration targets in a five-dimensional contact-compatible space. Depending on pose and targets, that system is infeasible or requires relaxing at least one target. Height and pitch should therefore be feedback objectives or bounded-slack tasks, not simultaneous exact acceleration constraints.

The current contact Jacobian gives each wheel its own normal row; despite left/right symmetry, each row contains its side's leg-coordinate derivatives. Its structure is not the same as two duplicate `base_z` rows. Still, the rank and condition number must be computed at the measured anchor and along the proposed reference. Contact-force uniqueness is a separate question from acceleration uniqueness: symmetric support can make force distribution sensitive or poorly conditioned. Accept a solution only when the original dynamics/contact residuals pass, normal forces are nonnegative, tangential forces remain inside the friction cone, and effort/velocity limits hold. Do not resolve a non-unique contact-force split by silently picking equal forces unless that rule is an explicit, validated secondary objective.

A practical inverse interface can use decision variables `(a[9], tau_leg[4], lambda_contact[4])`, with actual bounded-P wheel torque held fixed as the input:

```
M(q) a + C(q,v) + G(q) + D(v) = B_leg tau_leg + B_wheel tau_wheel + J(q)^T lambda
J(q) a + b(q,v) = 0
```

Add the four desired leg accelerations as hard equalities only for the small validated motion domain. Represent base pitch/height/forward targets as prioritized tasks with explicit bounded slack. Bound leg torque, wheel torque, wheel target rate, and joint/soft-limit margins; enforce contact-force cones. If no feasible point exists, return a named infeasibility (contact rank/conditioning, cone, torque bound, or task conflict) and do not fall back to an unconstrained or sign-friction model. This is a feasibility/reachability interface, not a parameter-fitting or gain-tuning step.

## Runtime state and input evidence

The controller's moving guard checks a recent `/joint_states` sample (at most 20 ms old), fresh IMU/odom/COM state, effort mode, and a bilateral contact snapshot. Those checks are useful runtime guards, but they are not same-Physics-step native state. The native command-state subscriber validates and logs the `NCS1` packet, but the current controller only pushes it into history; no active motion-guard read of that history was found. Do not describe the ROS JointState guard as a fresh direct-Physics q/v guard.

The direct-Physics engine CSV and the derived native records are the right offline source for per-step pre-state and actual six `JointForceCmd` inputs. Keep their iteration/sim-time key and before/after phase pairing. Pair controller command-publication logs and servo logs using their source/publish stamps and steady-wall time against the engine before-step observation time. For each step retain both the actual native input and all plausible causal publication candidates; repeated equal torques remain ambiguous. A missing/invalid publication candidate should fail publication provenance, while a finite native input may still support a state/model calculation. These are distinct gates.

## Minimum implementation and acceptance contract

1. **Offline reachability preflight:** at the measured standing anchor, evaluate the planned four-leg reference and wheel P law across the fixed low-speed profile. Record the 4x9 contact-Jacobian singular values/rank/condition, full constrained solve residuals, normal/friction-cone margins, and required hip/knee torques. Include same-direction and return-direction references. Reject the reference before any motion if rank, cone, actuator, velocity, or soft-limit checks fail.
2. **Prioritized task test:** demonstrate a feasible reference with all four low-rate leg references while pitch and height are feedback/slack objectives. Separately request an intentionally incompatible pitch/height acceleration combination and verify that the interface reports task conflict or bounded slack; it must not output an unqualified exact solution.
3. **Physics-tick logging:** for every relevant before/after engine step record full-scoped q/v, actual six pre-step joint-force commands, phase/iteration/dt, validity, and raw contact frames. Log command publication IDs and wall bounds, servo source stamps/torques, actual reference q/v/a, solver outputs, active constraints, residuals, contact forces/margins, and saturation/clipping. No NaN or missing input may be rewritten as zero or dropped from a window.
4. **Fail-closed runtime guard:** abort the motion reference on stale or invalid joint/IMU/odom/contact data, loss of bilateral contact, controller-mode transition, clock discontinuity, solver invalidity, unexpected saturation that prevents tracking, hard/soft-limit approach, or body guard trip. Saturation is diagnostic; it is a failure only when it makes the commanded reference infeasible or violates a fixed protection bound.
5. **Independent observed trials:** preserve the existing fixed small protocol and run at least two identical frozen-profile trials only after preflight. Require at least 30 consecutive valid native steps above the minimum actual-rate threshold for each of the four legs in acceleration/cruise, successful ordinary stop, then the opposite-direction return. Keep all steps, including stalls, reversals, contact conversions, and post-stop motion. Do not use the existing stand-only response gate as a moving-domain certificate; evaluate the existing nine-DOF response gate separately on fixed motion windows, with raw integrity AND model response AND stop/replay gates.

The current probe's fixed limits and protections remain the boundary for this review. Failure to excite a joint is a failed reachability/realization result, not authorization to increase gains or limits.

## Questions to settle before implementation

- Which body tasks are hard safety constraints versus soft goals during the four-leg probe? The contact manifold permits only five independent accelerations, so the interface must specify which tasks yield when the references conflict.
- Is the wheel-servo command included as an exact bounded torque computed from the same fresh wheel-rate sample, or as a measured native input in offline replay? Online planning and offline replay must not silently use different torque semantics.
- What numerical condition-number threshold will trigger an explicit `contact_jacobian_ill_conditioned` rejection? Set it using scale-aware residual tests at the measured anchor and nearby poses, not by selecting a threshold to pass one trace.
- Which logged timestamp is the source age for each state/input pair? Preserve simulator key/phase and same-host wall order; do not subtract unrelated ROS and steady clocks.
- How are contact wrench ambiguity and normal-force allocation reported? Require a feasible cone witness and report non-uniqueness or poor conditioning rather than claiming a measured load split.

## Read-only sources reviewed

- `src/bbot_balance_controller/include/bbot_balance_controller/ground_motion_probe.hpp`
- `src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp` (ground-motion state/guards, support effort, logging, wheel publication)
- `src/bbot_balance_controller/include/bbot_balance_controller/thrust_support_dynamics.hpp` (nine-coordinate geometry and bilateral contact Jacobian)
- `src/bbot_balance_controller/include/bbot_balance_controller/ground_friction_response.hpp` (offline forward response; not an inverse controller)
- `src/bbot_balance_controller/include/bbot_balance_controller/native_command_state.hpp` (NCS1 parsing/history contract)
- `src/bbot_balance_controller/scripts/wheel_effort_velocity_servo.py`
- `experiments/jump/run_ground_motion_trial.py`
- `experiments/jump/audit_ground_motion_trial_v3.py`
- `src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_motion_probe_20261006_134820/motion_audit_v3/AUDIT_REVIEW.md`
