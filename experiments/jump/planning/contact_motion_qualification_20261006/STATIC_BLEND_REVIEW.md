# Static-friction blend check

This is an offline check only; it is not a controller qualification or a physical test. The input snapshot is the NCS1 record at `sim_ns=18567000000` in the frozen `ground_motion_probe_20261006_134820` trial. The old trial was read-only. Its measured leg command and wheel command are copied at full precision into the targeted test; no historical file was changed.

The measured leg rates were approximately `4.2e-13 rad/s` at each hip and `4.88e-13 rad/s` at each knee. The baseline leg command was `[-0.72345083572675517, -15.31390561244141, -0.72345084855659703, -15.313905619364968] Nm`; the wheel effort was `[0.0089971939605110185, 0.0089971940042035264] Nm`. With the joint reference held at the measured anchor and zero velocity/acceleration reference, the inverse helper's zero-friction static member is `[-0.698641, -15.3796] Nm` per symmetric pair.

For blend fractions 0, 0.25, and 0.5, `ground_contact_motion_control` rejects with `friction_mode_not_converged`. The separately evaluated full 81-mode forward response of the baseline-to-static-inverse interpolated command is feasible in all three cases and chooses static friction at all four joints. The exact unblended inverse torque is `[-0.69864078597180235,-15.379579004454303,-0.6986407860734718,-15.379579004468228] Nm`. The required maximum static reaction falls from `0.065329485180110433` to `0.048997113397272245` to `0.032664742588097567 Nm`; predicted hip acceleration remains around `4.2e-10 rad/s^2`. At alpha 1, the helper and full forward solver agree: all four joints are in the static mode, required static reaction is `4.5244293290190585e-9 Nm`, and the maximum acceleration difference is `1.6147762710305713e-9`.

This establishes a concrete gap in the current helper: the legitimate intermediate hold commands need a nonzero set-valued static-friction reaction, while the helper selects the zero member and rejects when tiny predicted velocities toggle its friction branch. The evidence supports retaining the rejection; it does not support adding a friction floor, relaxing model residuals, or changing gains. A later helper must solve or certify the static reaction on the final blended command before controller integration.

Reproduce with:

```bash
g++ -O2 -std=c++17 \
  -I src/bbot_balance_controller/include -I /usr/include/eigen3 \
  src/bbot_balance_controller/test/offline/test_ground_contact_motion_control.cpp \
  -o /tmp/test_ground_contact_motion_control
/tmp/test_ground_contact_motion_control /tmp/static_blend_witness_fresh.csv
```

The saved test wrote full-precision `static_blend_witness.csv` in this directory; the current test accepts a fresh output path to avoid overwriting records (SHA256 `0a8dec282e7d11b589b3485966fc647240c8cb60d87ae1f103fa697136b0a0e0`). It also exercises one moving sample against the independent 81-mode forward solver and prints a 100-call local host timing distribution. The latest `-O2` run reported mean `116.082 µs`, p50 `115.678 µs`, p95 `119.636 µs`, and max `135.631 µs` per helper call on this host. These measurements exclude ROS scheduling and do not certify a callback budget. Test binary SHA256: `97464ce84c57dd90369efee1277120317a3681bb8fca49cb51bab29bc0937873`.

Root independently compared all nine q/v and all six actual ForceCmd values with the direct Physics/native input records at iteration 18567, end-step 18.567 s. All differences are exactly zero; see `static_witness_native_crosscheck.json`. The original snapshot originated in NCS1, but this witness is now also supported by the direct engine record. Static friction values remain model-inferred reactions, not a measured friction sensor.
