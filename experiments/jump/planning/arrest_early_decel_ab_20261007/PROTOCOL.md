# Fixed early ARREST shape causal experiment

Six fresh-world runs, alternating B1/T1/B2/T2/B3/T3; no retries or tuning. Baseline current source; test only ARREST sampling correction dv*T*u^3*(1-u)^3 plus exact derivatives. End q/v/a, 60ms duration, all ROS parameters except per-run data paths frozen; gain=1.60. Same unoptimized compiler flags in two independent builds. Default executable untouched.

Each command follows existing startup stable gate, then observe fixed20s after native touchdown. Retain all failures; infrastructure failures stop campaign, do not replace samples. Actual state switching times may change as a consequence of measured state; switching conditions and durations are not edited.

Native contact-frame t0=first zero-contact frame after last contact before continuous airborne interval. Analyze0..80ms q/v targets and actual, pitch/rate, wheel command/actual, states. Onset criterion original rate<-.20 continuously50ms; include pre-off onset and label carry-in. Report min rate0..50ms, pitch delta0..80ms, true contact pitch, bilateral native clearance, states/anomalies. Also measure actual absolute joint-speed change0..30/50ms and finite-difference acceleration RMS, to establish whether the intervention actually weakened braking. No selective accepted samples.

Exploratory evidence only (3/group). A meaningful trend requires actual intervention effect and consistent reduction/delay of early negative rate; report individual outcomes and initial q/v/rate, not only means. If actual braking is not reduced, negative outcome is inconclusive about the original mechanism. No production adoption, no automatic further tuning.
