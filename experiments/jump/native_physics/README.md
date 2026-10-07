# Opt-in native Gazebo physics state recorder

This private target is a diagnostic copy of Gazebo Sim 6.18.0's built-in
`Physics` system. It is loaded only when the trial uses the private plugin and
sets `BBOT_NATIVE_PHYSICS_CSV=/path/to/file.csv`. It writes direct physics-engine
state around `ForwardStep::Step`; it does not create, remove, or set ECM
components, or change the engine/control path. The default Gazebo world and
installed plugin are not modified.

`before_step` and `after_step` rows use the same `sim_time_ns` and `iteration`
key. `UpdateInfo.simTime` is the end time for the upcoming step: before-step
physical state time is `sim_time_ns - dt_ns`, and after-step physical state
time is `sim_time_ns`. The logger marks a step clock valid only for 1 ms
increments of both sim time and iteration (the first sampled step is valid when
dt is 1 ms). Missing target entities, missing physics objects, non-finite
values, non-1-DOF joints, or invalid clocks produce validity 0 and numeric
`nan`; they are not replaced by zero. Link values come from
`FrameDataRelativeToWorld`; joint values and generalized applied force come
from `JointPtr` getters. `joint_force_0` is a generalized joint force, not a
transmitted wrench. Wall timestamps use `steady_clock`.

The plugin buffers samples, flushes on a paused update, and does not flush every
row. CSV open/write failures are logged and do not alter the physics path.

## Source and build

The vendored source is from the exact Debian source package
`ignition-gazebo6 6.18.0-1~jammy`, including the upstream orig archive and
Debian packaging archive. The package has no `debian/patches` series. The
source delta is recorded in `patches/raw_native_readonly.patch`.

Configure and build privately:

```sh
cmake -S experiments/jump/native_physics -B build_ground_native_physics -DCMAKE_BUILD_TYPE=Release
cmake --build build_ground_native_physics --target ignition-gazebo-physics-system -j2
```

The resulting private plugin is
`build_ground_native_physics/plugin/libignition-gazebo-physics-system.so`.
The host runner must use a private world with this plugin as its sole physics
system, and point the plugin path at the private build output. Do not load this
alongside the installed Physics system in the same Gazebo process.

This target has been compiled only; no simulation was run as part of its
implementation.
