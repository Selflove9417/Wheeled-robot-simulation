# Offline fast fixed-mode certificate

This folder freezes the second-round offline prototype. It is not connected to the controller and does not authorize or establish a physical trial. The unchanged 81-mode implementation remains the reference baseline.

## Model and certificate

At a fixed 1 ms step, DART's free velocity stage uses the existing implicit viscous damping matrix `D`; the subsequent contact and Coulomb impulses use ordinary `M`. For each fixed friction mode, the solver uses

`(M + dt D) a_free = -(C + G) - D v + u`,

then solves the same fixed-mode KKT equations as the baseline: `M a = M a_free + Jᵀ lambda + tau_friction`, `J a + contactBias = 0`, and `a_j = -v_j/dt` for stick coordinates. Sliding torques use `-mu * sign(v_j + dt*a_j)`. The baseline solver's strict sign-roundoff test, static effort tolerance `1e-8`, near-zero sliding postprocess tolerance `1e-9`, and acceleration clustering tolerance `1e-7` remain unchanged.

The friction response minimizes the strictly convex kinetic quadratic plus the Coulomb `L1` term over the affine contact-acceleration set. `M` is explicitly checked positive definite and the contact Jacobian is checked rank 4. Therefore the acceleration is unique in the exact problem. Reaction multipliers can be non-unique: the fast path reuses the reference solver's SVD pseudoinverse/minimum-norm reaction witness and permits a rank-deficient fixed-mode KKT only when its dynamics/contact residuals, stick interval, sliding sign, and contact cone checks pass. At numerical boundaries the code checks adjacent modes, removes a sliding mode only when its stick complement is independently feasible, and clusters accelerations with the original `1e-7` threshold. If a neighbor introduces a boundary coordinate outside the enumerated local set, it rejects closed. The two-knee boundary fixture also rejects when local exploration reaches the 1 ms deadline; it is not counted as fast-path qualification.

The complete `ground_contact_motion_control_fast` wrapper retains the existing inverse dynamics, constrained mobility, torque PD, implicit correction, alpha blend, and final state guards. It gives the final certificate only the time remaining from one shared 1 ms deadline. Any overrun rejects; there is no fallback to the old controller law.

## Offline evidence

The targeted test compares fast outputs against the unchanged `ground_friction_response` 81-mode result on the stationary alpha 0, 0.25, 0.5, and 1.0 blend points; the corrected 4 ms requested-wheel fixture; a negative-slip inverse request; and both sides of the `±0.1 Nm` stick boundary at offsets `1e-10`, `1e-9`, and `1e-8`. On all those accepted cases, acceleration, contact force, and leg friction match at printed precision. Their raw state and command values, both responses, mode, KKT rank, residuals, and timing are preserved in `fast_certificate_witness.csv`.

The all-stick alpha-0 KKT witness has dimension 17, rank 16, nullity 1, minimum singular value `1.0154152489182456e-16`, mass minimum eigenvalue `0.00619836138117178`, and contact-Jacobian rank 4. The null vector has acceleration norm `8.943551422433104e-10` and reaction norm 1.0; the reaction-space nullity does not change the acceleration. The reference and fast path select the same minimum-norm reaction witness with maximum static effort `0.065329485180110433 Nm` and equation residual `5.710338949564828e-13`.

On this machine, the complete wrapper took about 200–225 microseconds for the four alpha hold points. The corrected positive-slip fixture needed four candidate modes and took about 160 microseconds; the negative-slip case needed 16 and took about 543 microseconds. Single-knee boundary fixtures needed three candidates and took about 175–183 microseconds. A two-knee boundary fixture was valid under full81 but reached the fast path's 1 ms limit after 29 tested candidates, so the fast path rejected it as designed. An 80-call hold/positive-slip timing sample reported p50 about 147 microseconds and p95 about 152 microseconds. These are prototype measurements only; they do not prove worst-case callback scheduling or real-time suitability.

The root agent's separate frozen-header 300-case constrained perturbation audit reports 47 accepted cases, all valid under full81 with maximum nine-acceleration difference 0; 253 fast rejections, all due to the 1 ms deadline; full81 rejected 4 cases; there were zero false accepts or acceleration mismatches. This is `NUMERICAL_ONLY`: it shows the fast path is conservative on that sample, and also that its accepted region is narrow. The full81 comparison shares the fixed-mode `solve_mode` implementation, so it verifies mode enumeration, boundary selection, and response equivalence against the reference, but it is not an independent dynamics implementation or plant validation. No ROS execution, controller integration, or Gazebo run was performed.

## Reproduction

From the workspace root:

```sh
g++ -O2 -std=c++17 \
  -I src/bbot_balance_controller/include -I /usr/include/eigen3 \
  src/bbot_balance_controller/test/offline/test_ground_contact_motion_fast_certificate.cpp \
  -o /tmp/test_ground_contact_motion_fast_certificate
GROUND_FAST_WITNESS_CSV=experiments/jump/planning/contact_motion_revision_20261006_round2/fast_certificate_final/fast_certificate_witness.csv \
  /tmp/test_ground_contact_motion_fast_certificate
```

The fixed-mode nullspace witness can be rebuilt with:

```sh
g++ -O2 -std=c++17 \
  -I src/bbot_balance_controller/include -I /usr/include/eigen3 \
  experiments/jump/planning/contact_motion_revision_20261006_round2/audit_static_kkt_nullspace.cpp \
  -o /tmp/audit_static_kkt_nullspace
/tmp/audit_static_kkt_nullspace
```

The output binary and captured test/nullspace logs are included here with SHA-256 hashes. The repository's current full81 sources are copied under `dependencies/`; their hashes identify exactly what was used. Input/controller integration and live-callback deadline fault injection remain untested. Per-tick qualification over a complete movement protocol remains outstanding.
