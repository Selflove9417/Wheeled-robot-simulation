# Hip-only momentum causal A/B: frozen protocol

Question: does reducing airborne early hip braking reduce the negative hip momentum contribution and body rearward pitch-rate consistently, without materially losing positive knee compensation?

Six fresh-world jumps B1/T1/B2/T2/B3/T3, no replacement samples or tuning. Same existing startup gate held1s, fixed20s observation after real native touchdown. Infrastructure failure stops campaign. Same1ms step, configured real-time factor1, jump_height.25, gain1.60 and all other resolved parameters frozen apart from paths. Seven-link readonly direct Physics diagnostic reused unchanged.

Baseline rebuilt current original; test only end_v_hip(v)=v-.25*(v-clamp(v,-2,2)). ARREST60ms and end_q formula unchanged, so ARREST intermediate hip endpoint moves; final TUCK/EXTEND/landing target generation unchanged. Knee reference formula untouched. Coupled dynamics FF may change knee effort in response to changed hip reference acceleration; do not claim output isolation or identical physical knee response. Full state conditions untouched; actual transitions may vary downstream. Default executable/launch untouched.

Native t0=first permanently no-contact frame. Analyze all81 frames0..80ms, per-run native momentum inventory validation and fixed-initial-H prediction. Same decomposition as prior review in fixed18..54ms and0..80ms, plus descriptive per-run peak-to-min. Fixed windows are primary to avoid selecting favorable extrema. Record pre-air initial states, ARREST/TUCK timings, actual/reference speeds, pitch/rate, wheel velocity, contact. Native onset rate<-.20 for50ms, including carry-in; minimum0..50/80 and pitch change0..80, touchdown and anomalies retained.

Exploratory3/group, no claim statistical significance. Support requires reduced measured negative hip contribution and weaker negative body-rate in all test runs relative to baseline range, with knee positive compensation retained (descriptively quantify all changes). If intervention not achieved, no inference against mechanism. If body-rate not consistently weaker or knee compensation substantially lost, no adoption. No further tuning, simulation stops after six.
