# Invalid model replay seed runs

Every trajectory and summary in this directory is retained for audit only and is `INVALID_SOFTWARE_SEED`; none of its trial stop/fail outcomes is evidence about candidate/controller reliability.

The observer seed used an incorrect timestamp origin/unit: a 20 ms extrapolation from the native anchor was effectively treated as a 1 ms interval after clipping the timestamp to local zero. This inflated initial COM velocity. These artifacts predate the corrected absolute-nanosecond observer initialization in `contact_com_fixed_replay_v4`.

The archived executables are preserved under explicit invalid-software-seed names. They must not be used to reproduce candidate conclusions. The corrected v4 run is still an offline model-design replay from the already-moving first-row anchor, not a faithful new 2 s handoff, native sensor-chain qualification, or physical run.
