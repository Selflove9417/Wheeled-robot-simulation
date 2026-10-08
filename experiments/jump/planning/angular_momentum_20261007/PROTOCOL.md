# Read-only angular momentum diagnostic

One default-control jump D1, initial stable gate unchanged, fixed20s after native first contact. No controller or parameter edits; gain1.60. World difference only sole Physics plugin replaced by same-source private Physics with getter-only logging extended from3 to7 discovered rigid links, plus runtime inertial inventory. No second Physics system. Native both-phase state timestamps and validity retained. Capture runtime robot_description and converted model; verify masses/inertias against runtime ECM.

Analysis t0=first continuously no-contact frame,0..80ms. Compute all seven links spin+orbital angular momentum about instantaneous systemCOM in world, fixed pitch projection=-worldX. Include merged IMU inertia. Check inventory, clocks, contacts, coordinate transforms, total residual before interpreting transfer. One run supplements existing records, no new tuning or A/B. If invalid, report gap rather than invent state.
