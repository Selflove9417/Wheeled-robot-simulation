# v2 early-dwell exploratory replay — not full protocol qualification

This directory preserves the round-2 model-only replay exactly as generated, including source, executable, summary, full 1 ms CSV trajectories, clock sidecars, frozen pre-run manifest, output hashes, and its original report.

The replay's stop logic began dwell when `trial_elapsed > 1.0 s` and advanced dwell at 1 ms plant cadence. The frozen protocol instead begins normal stop at 1.5 s, then counts the required 0.25 s dwell on 5 ms control ticks; it also requires the 0.25 s handoff settle to be earned by quiet-state guards, not elapsed time alone. Therefore the archived v2 `PASS` fields only describe its incorrect early-dwell model implementation. They are withdrawn as full-protocol PASS or qualification evidence. The COM helper/alpha/full-81 behavior remains an exploratory model observation.

Do not overwrite or delete these artifacts. The round-3 replay in the parent directory uses separate source, binary, prefix and manifest after correcting the timing semantics. Both rounds remain MODEL ONLY, not physical response/braking/jump results.
