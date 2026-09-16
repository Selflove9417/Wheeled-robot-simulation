#!/usr/bin/env python3
"""
Height-campaign runner for the gain-scheduling study (paper Sec. 4.2).

Automates three experiment groups on top of adaptive_lqr_balance_controller:
  constant : hold a fixed hip-axle height for 30 s (five design nodes x 2 gain modes)
  lift     : 0.30 -> 0.50 -> 0.30 m height transition at 0.05 m/s (2 gain modes)
  push     : horizontal force pulse at base_link (3 heights x 2 directions x 2 gain modes)

Both gain modes run experiment.mode=nominal with adaptation disabled, so the only
difference is gain.mode=scheduled vs fixed_midpoint (H = 0.40 m gains, geometry
still scheduled with height).

Usage examples:
  python3 run_height_campaign.py --ws-root /home/admin/bbot_ws_new --job constant
  python3 run_height_campaign.py --ws-root /home/admin/bbot_ws_new --job all
  python3 run_height_campaign.py --job push --repeats 3 --force 20 --force-duration 0.2
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# Trial definitions
# ---------------------------------------------------------------------------

HEIGHTS_CONSTANT = [0.30, 0.35, 0.40, 0.45, 0.50]
HEIGHTS_PUSH = [0.30, 0.40, 0.50]
GAIN_MODES = ["scheduled", "fixed_midpoint"]
STARTUP_VALIDATION_HOLD = 30.0

RESET_SIM_TIME = 6.74          # matches run_state_machine_trial.py protocol
CONSTANT_HOLD_AFTER_RESET = 35.0
LIFT_SCHEDULE = {              # relative to the reset instant
    "rise": 5.0,               # publish /target_height 0.50
    "descend": 19.0,           # 5 + 4 s rise + 10 s high hold
    "end": 38.0,               # + 4 s descent + 15 s low hold
}
PUSH_SCHEDULE = {              # relative to the reset instant
    "bridge": 0.8,             # start ros_gz_bridge
    "pulse": 5.0,              # legacy nominal marker; trigger is steady-state gated
    "pulse_off": 5.2,          # legacy nominal marker; helper uses the sim clock
    "end": 40.0,               # 35 s recovery window
}
PULSE_DURATION_TOLERANCE = (0.19, 0.21)

STATE_NAMES = {"scheduled": "SCHED", "fixed_midpoint": "FIXED"}


class StartupAbort(Exception):
    """Internal control flow for a failed paused-startup readiness check."""


def build_trials(job, repeats, force, height_filter=None, gain_mode_filter=None):
    """Return a list of trial dicts (order fixed for reproducibility).

    Push trials alternate the force sign; the sign is part of the trial tag
    so +/- runs can never overwrite each other's CSV files.
    """
    trials = []
    if job == "startup_validation":
        for rep in range(1, repeats + 1):
            trials.append({"job": job, "height": 0.30,
                           "gain_mode": "scheduled", "rep": rep,
                           "force": 0.0})
        return trials
    if job in ("constant", "all"):
        for h in HEIGHTS_CONSTANT:
            if height_filter is not None and abs(h - height_filter) > 1e-9:
                continue
            for gm in GAIN_MODES:
                if gain_mode_filter is not None and gm != gain_mode_filter:
                    continue
                for rep in range(1, repeats + 1):
                    trials.append({"job": "constant", "height": h,
                                   "gain_mode": gm, "rep": rep,
                                   "force": 0.0})
    if job in ("lift", "all"):
        for gm in GAIN_MODES:
            if gain_mode_filter is not None and gm != gain_mode_filter:
                continue
            for rep in range(1, repeats + 1):
                trials.append({"job": "lift", "height": None,
                               "gain_mode": gm, "rep": rep, "force": 0.0})
    if job in ("push", "all"):
        for h in HEIGHTS_PUSH:
            if height_filter is not None and abs(h - height_filter) > 1e-9:
                continue
            for gm in GAIN_MODES:
                if gain_mode_filter is not None and gm != gain_mode_filter:
                    continue
                for sign in (+1.0, -1.0):
                    for rep in range(1, repeats + 1):
                        trials.append({"job": "push", "height": h,
                                       "gain_mode": gm, "rep": rep,
                                       "force": sign * force})
    return trials


def trial_tag(trial):
    """Unique per-trial file tag (force sign included for push)."""
    if trial["job"] == "push":
        return (f"push_{trial['height']:g}_{trial['gain_mode']}"
                f"_f{trial['force']:+g}_{trial['rep']}")
    if trial["height"] is None:
        return f"{trial['job']}_{trial['gain_mode']}_{trial['rep']}"
    return f"{trial['job']}_{trial['height']:g}_{trial['gain_mode']}_{trial['rep']}'".replace("'", "")


# ---------------------------------------------------------------------------
# Process helpers
# ---------------------------------------------------------------------------

def cleanup_lingering_processes():
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "adaptive_lqr_balance_controller"],
                   stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "spawner"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "bbot_force_pulse"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "bbot_trial_helper"], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def _ros_cli(env, args, timeout=3.0):
    """Run a small ROS graph query without relying on wall-clock simulation."""
    try:
        return subprocess.run(["ros2", *args], env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _world_control(env, world_name, request):
    try:
        return subprocess.run(
            ["ros2", "service", "call", f"/world/{world_name}/control",
             "ros_gz_interfaces/srv/ControlWorld", request],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=5.0).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _service_succeeded(output):
    """Accept both ros2 CLI response spellings (YAML and Python repr)."""
    normalized = output.lower().replace(" ", "")
    return "success:true" in normalized or "success=true" in normalized


def step_paused_world(env, world_name):
    """Advance a few milliseconds while paused so ros2_control can activate."""
    output = _world_control(
        env, world_name, "{world_control: {step: true, multi_step: 10}}")
    ok = _service_succeeded(output)
    if not ok:
        print(f"[Runner] paused step failed: {output.strip()[-240:]}")
    return ok


def pause_world(env, world_name):
    """Force physics paused before probing controller readiness."""
    output = _world_control(
        env, world_name, "{world_control: {pause: true}}")
    ok = _service_succeeded(output)
    if not ok:
        print(f"[Runner] pause request failed: {output.strip()[-240:]}")
    return ok


def wait_for_adaptive_startup(env, world_name, timeout=60.0):
    """Verify the paused startup graph before allowing physics to advance."""
    deadline = time.monotonic() + timeout
    required_subscriptions = ("/imu", "/joint_states", "/target_height",
                              "/adaptive_lqr/command")
    control_service = f"/world/{world_name}/control"
    last_status = ""
    physics_paused = False
    controller_probe_printed = False
    while time.monotonic() < deadline:
        services = _ros_cli(env, ["service", "list"])
        if control_service in services:
            if not physics_paused:
                physics_paused = pause_world(env, world_name)
            if physics_paused:
                step_paused_world(env, world_name)
        nodes = _ros_cli(env, ["node", "list"])
        controllers = _ros_cli(
            env, ["control", "list_controllers", "-c", "/controller_manager"])
        node_info = _ros_cli(env, ["node", "info", "/adaptive_lqr_balance_controller"])
        wheel_active = any(
            "wheel_effort_controller" in line and
            "active" in line.lower()
            for line in controllers.splitlines())
        if (not wheel_active and not controller_probe_printed and
                "wheel_effort_controller" in controllers):
            print(f"[Runner] controller probe output: {controllers.strip()}")
            controller_probe_printed = True
        subscriptions_ready = all(topic in node_info for topic in required_subscriptions)
        if (physics_paused and control_service in services and
                "/adaptive_lqr_balance_controller" in nodes and
                wheel_active and subscriptions_ready):
            return True, "ready"
        last_status = (f"control={'yes' if control_service in services else 'no'}, "
                       f"node={'yes' if '/adaptive_lqr_balance_controller' in nodes else 'no'}, "
                       f"wheel_active={'yes' if wheel_active else 'no'}, "
                       f"subscriptions={'yes' if subscriptions_ready else 'no'}")
        time.sleep(0.5)
    return False, f"startup readiness timeout ({last_status})"


def unpause_world(env, world_name):
    """Release the physics pause only after the startup graph is complete."""
    service = f"/world/{world_name}/control"
    try:
        result = subprocess.run(
            ["ros2", "service", "call", service,
             "ros_gz_interfaces/srv/ControlWorld",
             "{world_control: {pause: false}}"],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False, "unpause service call timed out"
    output = result.stdout or ""
    if result.returncode != 0 or not _service_succeeded(output):
        return False, f"unpause service failed: {output[-400:]}"
    return True, "unpaused"


HELPER_NODE_CODE = r'''
import json, os, sys, time
import rclpy
from std_msgs.msg import Float64, String


def main():
    spec_path, trigger_dir = sys.argv[1], sys.argv[2]
    spec = json.load(open(spec_path))
    rclpy.init()
    node = rclpy.create_node("bbot_trial_helper")
    pubs = []
    for s in spec:
        cls = String if s["type"] == "String" else Float64
        pub = node.create_publisher(cls, s["topic"], 10)
        pubs.append((s, pub))
    time.sleep(2.0)  # discovery warm-up
    done = set()
    deadline = time.time() + 900.0
    while len(done) < len(pubs) and time.time() < deadline:
        for s, pub in pubs:
            if s["trigger"] in done:
                continue
            path = os.path.join(trigger_dir, s["trigger"])
            if not os.path.exists(path):
                continue
            # Never fire before the subscriber is matched (10 s budget).
            waited = 0.0
            while pub.get_subscription_count() == 0 and waited < 10.0:
                time.sleep(0.05)
                waited += 0.05
            msg = String() if s["type"] == "String" else Float64()
            msg.data = s["payload"] if s["type"] == "String" else float(s["payload"])
            for _ in range(5):
                pub.publish(msg)
                time.sleep(0.002)
            done.add(s["trigger"])
            print(f"[Helper] published {s['trigger']}", flush=True)
            os.remove(path)
    node.destroy_node()
    rclpy.shutdown()


main()
'''


class TrialHelper:
    """Pre-warmed publisher node for timed trial commands.

    ros2 CLI invocations take 1-12 s to start and discover the graph on this
    machine, which collapses any timing-critical command window.  A single
    rclpy helper process is started (and matched) at trial start instead; the
    runner then fires each command by creating its trigger file, which the
    helper picks up within milliseconds.
    """

    def __init__(self, env, spec):
        self.env = env
        self.trigger_dir = "/tmp/bbot_height_triggers_%d" % os.getpid()
        os.makedirs(self.trigger_dir, exist_ok=True)
        self.spec_path = os.path.join(self.trigger_dir, "spec.json")
        with open(self.spec_path, "w") as f:
            json.dump(spec, f)
        self.proc = None

    def start(self):
        self.proc = subprocess.Popen(
            [sys.executable, "-c", HELPER_NODE_CODE,
             self.spec_path, self.trigger_dir],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=self.env)

    def fire(self, trigger):
        with open(os.path.join(self.trigger_dir, trigger), "w") as f:
            f.write("fire")

    def shutdown(self):
        if self.proc is not None:
            self.proc.terminate()
            self.proc = None
        try:
            os.remove(self.spec_path)
            os.rmdir(self.trigger_dir)
        except OSError:
            pass


PULSE_NODE_CODE = r'''
import json, os, sys, time
import rclpy
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import EntityWrench, Entity
from std_msgs.msg import Float64


def main():
    (topic_persistent, topic_instant, topic_clear,
     link_name, axis, force, duration, trigger, state_path) = sys.argv[1:10]
    force, duration = float(force), float(duration)
    state = {
        "bridge_ready": False,
        "wrench_publish_count": 0,
        "clear_publish_count": 0,
        "marker_publish_count": 0,
        "zero_marker_publish_count": 0,
        "pulse_start_sim_time": None,
        "pulse_end_sim_time": None,
        "actual_duration": None,
        "completed": False,
        "clear_complete": False,
        "invalid_reason": None,
    }

    def write_state():
        tmp = state_path + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(state, handle, sort_keys=True)
        os.replace(tmp, state_path)

    def fail(reason):
        state["invalid_reason"] = reason
        write_state()

    rclpy.init()
    node = rclpy.create_node("bbot_force_pulse")
    pub_persistent = node.create_publisher(EntityWrench, topic_persistent, 10)
    pub_clear = node.create_publisher(Entity, topic_clear, 10)
    pub_dist_force = node.create_publisher(Float64, "/disturbance_force_y", 10)
    sim_time = [None]
    node.create_subscription(Clock, "/clock", lambda msg: sim_time.__setitem__(0, msg.clock.sec + msg.clock.nanosec * 1e-9), 10)
    # The exact-once protocol requires both persistent and clear subscribers
    # before the pre-trial clear is sent.
    waited = 0.0
    while (pub_persistent.get_subscription_count() == 0 or
           pub_clear.get_subscription_count() == 0) and waited < 120.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
        waited += 0.05
    if (pub_persistent.get_subscription_count() == 0 or
            pub_clear.get_subscription_count() == 0):
        fail("BRIDGE_NOT_CONNECTED")
        node.destroy_node()
        rclpy.shutdown()
        return

    state["bridge_ready"] = True
    clear = Entity()
    clear.name = link_name
    clear.type = Entity.LINK
    for _ in range(5):
        pub_clear.publish(clear)
        state["clear_publish_count"] += 1
        rclpy.spin_once(node, timeout_sec=0.01)
    state["pre_clear_complete"] = True
    write_state()

    while not os.path.exists(trigger):
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.005)
    try:
        with open(trigger) as handle:
            trigger_sim_time = float(handle.read().strip())
    except (OSError, ValueError):
        fail("invalid_trigger")
        node.destroy_node()
        rclpy.shutdown()
        return

    while sim_time[0] is None or sim_time[0] < trigger_sim_time:
        rclpy.spin_once(node, timeout_sec=0.01)

    msg = EntityWrench()
    msg.entity.name = link_name
    msg.entity.type = Entity.LINK
    setattr(msg.wrench.force, axis, force)

    # Exactly one persistent wrench publication.  Only the separate marker
    # topic is periodic while the simulated pulse is active.
    pulse_start = sim_time[0]
    pub_persistent.publish(msg)
    state["wrench_publish_count"] += 1
    state["pulse_start_sim_time"] = pulse_start
    write_state()
    force_marker = Float64()
    force_marker.data = force
    while sim_time[0] is None or sim_time[0] < pulse_start + duration:
        pub_dist_force.publish(force_marker)
        state["marker_publish_count"] += 1
        rclpy.spin_once(node, timeout_sec=0.005)

    state["pulse_end_sim_time"] = sim_time[0]
    state["actual_duration"] = state["pulse_end_sim_time"] - pulse_start
    for _ in range(5):
        pub_clear.publish(clear)
        state["clear_publish_count"] += 1
        rclpy.spin_once(node, timeout_sec=0.01)
    state["clear_complete"] = True
    zero_marker = Float64()
    zero_marker.data = 0.0
    pub_dist_force.publish(zero_marker)
    state["zero_marker_publish_count"] += 1
    state["completed"] = (state["wrench_publish_count"] == 1 and
                          0.19 <= state["actual_duration"] <= 0.21 and
                          state["clear_complete"])
    if not state["completed"]:
        state["invalid_reason"] = "invalid_pulse_protocol"
    write_state()
    node.destroy_node()
    rclpy.shutdown()


main()
'''


class ForcePulse:
    """Exact-duration force pulse via the ApplyLinkWrench persistent topic.

    The gz-sim ApplyLinkWrench system applies each message on the
    instantaneous topic for a single physics step only, so a short pulse
    published through a slow CLI collapses to a near-zero impulse.  Instead,
    a parameter_bridge maps the instantaneous/persistent/clear topics to ROS
    and a pre-warmed rclpy helper node waits for a trigger file, publishes
    one persistent wrench, holds it for `force_duration` seconds of simulation
    time, then publishes the clear command.  No zero-wrench overwrite is used.
    """

    def __init__(self, env, world_name, link_name, force_axis, force_value,
                 force_duration):
        self.env = env
        self.world_name = world_name
        self.link_name = link_name
        self.force_axis = force_axis
        self.force_value = force_value
        self.force_duration = force_duration
        self.bridge = None
        self.node = None
        self.trigger_path = "/tmp/bbot_force_trigger_%d" % os.getpid()
        self.state_path = "/tmp/bbot_force_state_%d.json" % os.getpid()
        for stale_path in (self.trigger_path, self.state_path):
            if os.path.exists(stale_path):
                os.remove(stale_path)

    def _gz_url(self, suffix):
        return f"/world/{self.world_name}{suffix}"

    def start_bridge(self, debug_dir=None):
        out = (open(os.path.join(debug_dir, "force_bridge.log"), "w")
               if debug_dir else subprocess.DEVNULL)
        self.bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             f"{self._gz_url('/wrench')}@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
             f"{self._gz_url('/wrench/persistent')}@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
             f"{self._gz_url('/wrench/clear')}@ros_gz_interfaces/msg/Entity]gz.msgs.Entity"],
            stdout=out, stderr=subprocess.STDOUT, env=self.env)
        time.sleep(1.5)  # allow the bridge to discover the gz topics

    def arm(self, debug_dir=None):
        if self.node is None:
            out = (open(os.path.join(debug_dir, "force_pulse.log"), "w")
                   if debug_dir else subprocess.DEVNULL)
            self.node = subprocess.Popen(
                [sys.executable, "-u", "-c", PULSE_NODE_CODE,
                 self._gz_url("/wrench/persistent"),
                 self._gz_url("/wrench"),
                 self._gz_url("/wrench/clear"),
                 self.link_name, self.force_axis,
                 str(self.force_value), str(self.force_duration),
                 self.trigger_path, self.state_path],
                stdout=out, stderr=subprocess.STDOUT,
                env=self.env)

    def fire(self, sim_time):
        with open(self.trigger_path, "w") as f:
            f.write(f"{float(sim_time):.9f}")

    def read_state(self):
        try:
            with open(self.state_path) as f:
                state = json.load(f)
            return state if isinstance(state, dict) else None
        except (OSError, ValueError, TypeError):
            return None

    def shutdown(self):
        for proc in (self.node, self.bridge):
            if proc is not None:
                proc.terminate()
        self.node = None
        self.bridge = None
        if os.path.exists(self.trigger_path):
            os.remove(self.trigger_path)


# ---------------------------------------------------------------------------
# Single trial
# ---------------------------------------------------------------------------

def run_trial(trial, args, env, ws_root, csv_path):
    job = trial["job"]
    gm = trial["gain_mode"]
    height = trial["height"] if trial["height"] is not None else 0.30
    print("=" * 60)
    print(f"  Trial {job} H={height} gain={STATE_NAMES[gm]} rep={trial['rep']} "
          f"force={trial['force']:+.1f} N")
    print(f"  CSV: {csv_path}")
    print("=" * 60)

    cleanup_lingering_processes()
    if os.path.exists(csv_path):
        os.remove(csv_path)

    target_height = height if job in ("constant", "push", "startup_validation") else 0.30
    cmd_str = (
        f"source {ws_root}/install/setup.bash && "
        f"ros2 launch bbot_bringup bbot_gazebo.launch.py "
        f"controller_type:=adaptive_lqr "
        f"adaptive_experiment_mode:=nominal "
        f"adaptive_gain_mode:={gm} "
        f"adaptive_com_y_bias:=0.0 "
        f"adaptive_two_stage_enabled:=false "
        f"gazebo_start_paused:=true "
        f"gazebo_world_name:={args.world_name} "
        f"adaptive_target_height:={target_height:.3f} "
        f"adaptive_startup_height:={args.startup_height:.3f} "
        f"adaptive_log_path:={csv_path}"
    )

    tag = trial_tag(trial)
    log_path = os.path.join(args.log_dir, f"{tag}.log")
    log_file = open(log_path, "w")
    pulse = ForcePulse(env, args.world_name, args.link_name,
                       args.force_axis, trial["force"],
                       args.force_duration) if job == "push" else None

    # Pre-warm the force pipeline BEFORE the sim starts: spawning the bridge
    # and the pulse node mid-run stalls the Gazebo clock for 10-25 s, which
    # shifts the pulse onset and can even collide with the trial end.
    if job == "push":
        pulse.start_bridge(args.log_dir)
        pulse.arm(args.log_dir)
    print(f"[Runner] Launching: {cmd_str}")
    proc = subprocess.Popen(cmd_str, shell=True, executable="/bin/bash",
                            stdout=log_file, stderr=subprocess.STDOUT,
                            env=env, preexec_fn=os.setsid, text=True)

    # Trial helper node: pre-warmed publishers for reset/height commands, so
    # the ~1 s ros2 CLI startup latency never lands inside the timing path.
    tag = f"{job}_{height}_{gm}_{trial['rep']}"
    spec = [] if job == "startup_validation" else [
        {"trigger": f"reset_{tag}.trg", "topic": "/adaptive_lqr/command",
         "type": "String", "payload": "reset_position"}]
    if job == "lift":
        spec.append({"trigger": f"up_{tag}.trg", "topic": "/target_height",
                     "type": "Float64", "payload": "0.50"})
        spec.append({"trigger": f"down_{tag}.trg", "topic": "/target_height",
                     "type": "Float64", "payload": "0.30"})
    helper = TrialHelper(env, spec)
    helper.start()

    reset_issued = False
    reset_sent = False
    reset_sim_t = None
    prev_x_ref = None
    bridge_started = False
    force_on = False
    steady_since = None
    h_risen = False
    h_lowered = False
    fail_reason = None
    pulse_state = None
    last_progress_real = time.time()
    last_progress_sim = -1.0
    t_end = {"constant": CONSTANT_HOLD_AFTER_RESET,
             "lift": LIFT_SCHEDULE["end"],
             "push": PUSH_SCHEDULE["end"],
             "startup_validation": STARTUP_VALIDATION_HOLD}[job]

    try:
        ready, readiness_detail = wait_for_adaptive_startup(
            env, args.world_name, timeout=args.startup_timeout)
        if not ready:
            fail_reason = readiness_detail
            print(f"[Runner] {readiness_detail}")
            raise StartupAbort()
        unpaused, unpause_detail = unpause_world(env, args.world_name)
        if not unpaused:
            fail_reason = unpause_detail
            print(f"[Runner] {unpause_detail}")
            raise StartupAbort()
        print("[Runner] startup graph ready; Gazebo physics unpaused")
        last_progress_real = time.time()
        while True:
            if proc.poll() is not None:
                fail_reason = f"launch process exited (rc={proc.poll()})"
                break

            record = parse_last_csv_line(csv_path)
            if record is None:
                if time.time() - last_progress_real > 60.0:
                    fail_reason = "no CSV data for 60 s (sim stuck?)"
                    break
                time.sleep(0.2)
                continue

            t_sim = record["time"]
            if t_sim > last_progress_sim + 0.05:
                last_progress_sim = t_sim
                last_progress_real = time.time()
            elif time.time() - last_progress_real > 45.0:
                # Controller stopped logging (self-disabled after a fall or
                # sensor loss): fail fast instead of waiting out the clock.
                fail_reason = "controller stopped logging (early disable?)"
                break

            # Safety/failure detection: controller self-disable or hard fallback.
            if fail_reason is None and abs(record["pitch"]) > args.fall_pitch_deg:
                fail_reason = f"robot fell (pitch={record['pitch']:.2f} deg)"

            if job == "startup_validation":
                if t_sim >= t_end:
                    print(f"[Runner] startup validation finished (t={t_sim:.2f}s)")
                    break
            elif not reset_issued and t_sim >= RESET_SIM_TIME:
                helper.fire(spec[0]["trigger"])
                reset_issued = True
                print(f"[Runner] reset_position triggered at t={t_sim:.2f}s")
            elif reset_issued and not reset_sent:
                # Confirm the reset by the actual x_ref jump in the CSV.
                if prev_x_ref is not None and \
                        abs(record["x_ref"] - prev_x_ref) > 0.01:
                    reset_sent = True
                    reset_sim_t = t_sim
                    print(f"[Runner] reset confirmed at t={t_sim:.2f}s")
                elif t_sim > RESET_SIM_TIME + 30.0:
                    fail_reason = "reset not confirmed within 30 s"
                    break
            elif reset_sent:
                dt_reset = t_sim - reset_sim_t

                if job == "lift":
                    if dt_reset >= LIFT_SCHEDULE["rise"] and not h_risen:
                        helper.fire(spec[1]["trigger"])
                        h_risen = True
                        print(f"[Runner] target 0.50 fired at t={t_sim:.2f}s")
                    if dt_reset >= LIFT_SCHEDULE["descend"] and not h_lowered:
                        helper.fire(spec[2]["trigger"])
                        h_lowered = True
                        print(f"[Runner] target 0.30 fired at t={t_sim:.2f}s")

                if job == "push":
                    # Adaptive logs do not expose the PID p_0_latched flag;
                    # after the explicit reset_position command, use the
                    # same physical steady-state gate for this controller:
                    # pitch <=0.2 deg, pitch rate <=0.02 rad/s, and speed
                    # <=0.01 m/s for five continuous simulated seconds.
                    stable = (abs(record["theta_error"]) <= 0.20 and
                              abs(record["pitch_rate"]) <= 0.02 and
                              abs(record["x_dot"]) <= 0.01)
                    if stable:
                        if steady_since is None:
                            steady_since = t_sim
                    else:
                        steady_since = None
                    if (steady_since is not None and
                            t_sim - steady_since >= 5.0 and not force_on):
                        pulse.fire(t_sim)
                        force_on = True
                        print(f"[Runner] force {trial['force']:+.1f} N fired at t={t_sim:.2f}s")

                if dt_reset >= t_end:
                    print(f"[Runner] {job} trial finished (t={t_sim:.2f}s).")
                    break

            prev_x_ref = record["x_ref"]
            # Watchdog: 90 s wall-clock without data means the sim is stuck.
            time.sleep(0.2)

    except StartupAbort:
        pass
    finally:
        print("[Runner] Stopping simulation...")
        if pulse is not None:
            pulse.shutdown()
            pulse_state = pulse.read_state()
        helper.shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(10):
                if proc.poll() is not None:
                    break
                time.sleep(0.5)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                time.sleep(1.0)
        except Exception as exc:
            print(f"[Runner] Exception while stopping: {exc}")
        cleanup_lingering_processes()
        log_file.close()

    metrics = analyze_height_trial(csv_path, job, trial["force"],
                                   args.force_duration, pulse_state=pulse_state)
    if pulse_state is not None:
        with open(csv_path + ".pulse.json", "w") as state_file:
            json.dump(pulse_state, state_file, indent=2, sort_keys=True)
    metrics.update({"job": job, "height": height if job != "lift" else np.nan,
                    "gain_mode": gm, "rep": trial["rep"],
                    "force": trial["force"],
                    "fail_reason": fail_reason or metrics.get("fail_reason")})
    return metrics


# ---------------------------------------------------------------------------
# CSV access (adaptive controller log).  Resolve fields by header name: the
# adaptive controller has several optional diagnostic columns and their order
# is not a stable interface.
# ---------------------------------------------------------------------------

FIELD = {"time": "time", "hip_height": "hip_axle_height",
         "base_height": "base_link_height", "height_rate": "height_rate",
         "x": "x", "x_ref": "x_ref", "x_error": "x_error", "x_dot": "x_dot",
         "pitch": "pitch", "pitch_rate": "pitch_rate",
         "theta_eq_nominal": "theta_eq_nominal", "theta_error": "theta_error",
         "filtered_x_error": "filtered_x_error", "u_model": "u_model"}


def parse_last_csv_line(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        last = None
        with open(path, "r", newline="") as f:
            for row in csv.DictReader(f):
                if row.get(FIELD["time"]) not in (None, ""):
                    last = row
        if last is None:
            return None
        return {"time": float(last[FIELD["time"]]),
                "x": float(last[FIELD["x"]]),
                "x_ref": float(last[FIELD["x_ref"]]),
                "pitch": float(last[FIELD["pitch"]]) * 180.0 / np.pi,
                "theta_error": float(last[FIELD["theta_error"]]) * 180.0 / np.pi,
                "pitch_rate": float(last[FIELD["pitch_rate"]]),
                "x_dot": float(last[FIELD["x_dot"]]),
                "u_model": float(last[FIELD["u_model"]])}
    except Exception:
        return None


def load_csv(path):
    # Drop any truncated trailing row (the controller can be killed mid-write)
    # and parse leniently, always by named fields.
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    rows = [row for row in rows if row.get(FIELD["time"]) not in (None, "")]
    if not rows:
        raise ValueError("no complete CSV rows")
    out = {}
    for key, name in FIELD.items():
        if name not in rows[0]:
            raise ValueError(f"missing CSV column: {name}")
        out[key] = np.array([float(row[name]) for row in rows])
    out["pitch_err_deg"] = out["theta_error"] * 180.0 / np.pi
    out["x_err_mm"] = out["filtered_x_error"] * 1000.0
    return out


def recovery_time(t, x_err_mm, pitch_err_deg, t_from, x_tol=10.0,
                  pitch_tol=0.5, hold=2.0, pitch_criterion=True):
    """Time after t_from until |x|<=x_tol (and |pitch|<=pitch_tol when
    pitch_criterion is set) held for `hold` s."""
    mask = t >= t_from
    tt = t[mask]
    ok = np.abs(x_err_mm[mask]) <= x_tol
    if pitch_criterion:
        ok = ok & (np.abs(pitch_err_deg[mask]) <= pitch_tol)
    n = len(tt)
    need = max(int(hold / 0.005), 1)
    for i in range(n - need):
        if np.all(ok[i:i + need]):
            return tt[i] - t_from
    return np.nan


def pulse_state_valid(state, force_duration):
    if not state or not state.get("bridge_ready"):
        return False
    if int(state.get("wrench_publish_count", 0)) != 1:
        return False
    if int(state.get("marker_publish_count", 0)) <= 0:
        return False
    if int(state.get("zero_marker_publish_count", 0)) <= 0:
        return False
    actual = state.get("actual_duration")
    if actual is None or not (PULSE_DURATION_TOLERANCE[0] <= float(actual) <= PULSE_DURATION_TOLERANCE[1]):
        return False
    return bool(state.get("completed")) and bool(state.get("clear_complete"))


def analyze_height_trial(csv_path, job, force_value, force_duration,
                         pulse_state=None):
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return {"fail_reason": "csv missing or empty"}
    try:
        d = load_csv(csv_path)
    except Exception as exc:
        return {"fail_reason": f"csv parse error: {exc}"}
    t = d["time"]
    planned = {"constant": CONSTANT_HOLD_AFTER_RESET, "lift": LIFT_SCHEDULE["end"],
               "push": PUSH_SCHEDULE["end"],
               "startup_validation": STARTUP_VALIDATION_HOLD}[job]
    if len(t) < 100 or t[-1] - t[0] < 0.6 * planned:
        return {"fail_reason": "csv too short (controller disabled early?)"}

    x_ref = d["x_ref"]

    # Reset instant: x_ref jump (same protocol as run_state_machine_trial).
    t0 = t[0]
    if x_ref is not None:
        jumps = np.where(np.abs(np.diff(x_ref)) > 0.01)[0]
        if len(jumps) > 0:
            t0 = t[jumps[0] + 1]
    post = t >= t0

    m = {"fail_reason": None}
    m["max_pitch_err_deg"] = float(np.max(np.abs(d["pitch_err_deg"][post])))
    m["max_x_err_mm"] = float(np.max(np.abs(d["x_err_mm"][post])))
    m["peak_u"] = float(np.max(np.abs(d["u_model"][post])))
    m["saturated"] = bool(m["peak_u"] > 19.5)
    post_indices = np.where(post)[0]
    x0 = float(d["x"][post_indices[0]]) if len(post_indices) else float(d["x"][0])
    m["final_position_drift_mm"] = float((d["x"][-1] - x0) * 1000.0)
    m["max_position_drift_mm"] = float(
        np.max(np.abs(d["x"][post] - x0)) * 1000.0)

    if job in ("constant", "startup_validation"):
        ss = (t >= t[-1] - 15.0) & post
        m["ss_x_err_mean"] = float(np.mean(d["x_err_mm"][ss]))
        m["ss_x_err_std"] = float(np.std(d["x_err_mm"][ss]))
        m["pitch_rms_deg"] = float(np.sqrt(np.mean(d["pitch_err_deg"][ss] ** 2)))
        m["u_rms"] = float(np.sqrt(np.mean(d["u_model"][ss] ** 2)))
    elif job == "lift":
        hr = d["height_rate"]
        ramp = post & (np.abs(hr) > 0.02)
        m["ramp_pitch_rms_deg"] = float(np.sqrt(np.mean(d["pitch_err_deg"][ramp] ** 2))) \
            if np.any(ramp) else np.nan
        m["ramp_max_x_err_mm"] = float(np.max(np.abs(d["x_err_mm"][ramp]))) \
            if np.any(ramp) else np.nan
        m["ramp_peak_u"] = float(np.max(np.abs(d["u_model"][ramp]))) \
            if np.any(ramp) else np.nan
        descend_end = t0 + LIFT_SCHEDULE["descend"] + 4.0
        m["recovery_s"] = recovery_time(t, d["x_err_mm"], d["pitch_err_deg"],
                                        descend_end)
    elif job == "push":
        m["pulse_state"] = pulse_state or {}
        if not pulse_state_valid(pulse_state, force_duration):
            m["fail_reason"] = "invalid_pulse_protocol"
        pulse_t0 = float((pulse_state or {}).get("pulse_start_sim_time") or
                         (t0 + PUSH_SCHEDULE["pulse"]))
        pulse_t1 = float((pulse_state or {}).get("pulse_end_sim_time") or
                         (pulse_t0 + force_duration))
        win = (t >= pulse_t0) & (t <= pulse_t1 + 1.0)
        after = t >= pulse_t1
        m["max_x_err_mm"] = float(np.max(np.abs(d["x_err_mm"][after]))) \
            if np.any(after) else np.nan
        m["max_pitch_err_deg"] = float(np.max(np.abs(d["pitch_err_deg"][after]))) \
            if np.any(after) else np.nan
        m["peak_velocity_mps"] = float(np.max(np.abs(d["x_dot"][post])))
        m["peak_pitch_abs_deg"] = float(np.max(np.abs(d["pitch_err_deg"][post])))
        win_peak = float(np.max(np.abs(d["u_model"][win]))) \
            if np.any(win) else 0.0
        m["peak_u"] = max(m["peak_u"], win_peak)
        # Actual force onset: first sustained wheel-speed excursion after the
        # nominal trigger instant (filters the low-height chatter).
        onset = pulse_t0
        window = (t >= pulse_t0) & (t <= pulse_t1 + 16.5)
        idx = np.where(window)[0]
        for i in idx:
            j = i
            while j < len(t) - 1 and abs(d["x_err_mm"][j + 1] -
                                         d["x_err_mm"][j]) > 0.0035 * 5.0:
                j += 1
            if t[j] - t[i] >= 0.30 and abs(d["x_err_mm"][j] -
                                           d["x_err_mm"][i]) > 0.010:
                onset = t[i]
                break
        m["pulse_onset_s"] = float(onset)
        m["force_applied"] = bool(np.max(np.abs(d["x_err_mm"][window])) > 12.0
                                  if np.any(window) else False)
        before = (t >= pulse_t0 - 0.25) & (t < pulse_t0)
        response_window = (t >= pulse_t0 + 0.10) & (t <= pulse_t0 + 0.50)
        if np.any(before) and np.any(response_window):
            v_before = float(np.median(d["x_dot"][before]))
            v_after = float(np.median(d["x_dot"][response_window]))
            m["initial_velocity_change_mps"] = v_after - v_before
        else:
            m["initial_velocity_change_mps"] = np.nan
        expected_sign = 1.0 if force_value >= 0.0 else -1.0
        m["expected_velocity_sign"] = expected_sign
        m["velocity_response_ok"] = bool(
            np.isfinite(m["initial_velocity_change_mps"]) and
            abs(m["initial_velocity_change_mps"]) >= 0.03 and
            expected_sign * m["initial_velocity_change_mps"] > 0.0)
        if not m["force_applied"]:
            m["fail_reason"] = m["fail_reason"] or "no_physical_force_response"
        elif not m["velocity_response_ok"]:
            m["fail_reason"] = m["fail_reason"] or "wrong_velocity_response_sign"
        m["recovery_s"] = recovery_time(t, d["x_err_mm"], d["pitch_err_deg"],
                                        onset + force_duration)
        m["recovery_pos_s"] = recovery_time(t, d["x_err_mm"], d["pitch_err_deg"],
                                            onset + force_duration,
                                            pitch_criterion=False)
    return m


# ---------------------------------------------------------------------------
# Campaign driver
# ---------------------------------------------------------------------------

def aggregate(metrics):
    """Mean +/- sample std over repeats, grouped by (job, height, gain_mode)."""
    groups = {}
    for m in metrics:
        key = (m["job"], m["height"], m["gain_mode"], m.get("force", 0.0))
        groups.setdefault(key, []).append(m)
    summary = []
    for (job, height, gm, force), runs in sorted(groups.items(),
                                                  key=lambda kv: str(kv[0])):
        row = {"job": job, "height": height, "gain_mode": gm,
               "force": force, "n": len(runs)}
        n_fail = sum(1 for r in runs if r.get("fail_reason"))
        row["n_fail"] = n_fail
        if job == "push":
            row["validity"] = (
                "valid_exact_once_pulse_and_response" if n_fail == 0
                else "invalid_pulse_or_response")
        elif job == "startup_validation":
            row["validity"] = "valid_paused_startup" if n_fail == 0 else "invalid_startup"
        numeric_keys = set()
        for r in runs:
            for k, v in r.items():
                if isinstance(v, (int, float)) and not isinstance(v, bool) \
                        and k not in ("job", "height", "rep"):
                    numeric_keys.add(k)
        for k in sorted(numeric_keys):
            vals = [r[k] for r in runs if isinstance(r.get(k), (int, float))
                    and not isinstance(r.get(k), bool) and np.isfinite(r[k])]
            if vals:
                row[k + "_mean"] = float(np.mean(vals))
                row[k + "_std"] = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
        summary.append(row)
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--job", default="all",
                    choices=["constant", "lift", "push", "startup_validation", "all"])
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--ws-root", default="/home/admin/bbot_ws_new",
                    help="ROS 2 workspace containing install/setup.bash")
    ap.add_argument("--data-dir",
                    default=None,
                    help="Output directory for CSV logs (default: <ws>/src/.../data_logs/height_campaign)")
    ap.add_argument("--force", type=float, default=20.0,
                    help="Push force magnitude in N (applied along --force-axis)")
    ap.add_argument("--height-filter", type=float, default=None,
                    help="Restrict the campaign to one height, e.g. 0.30")
    ap.add_argument("--gain-mode-filter", choices=GAIN_MODES, default=None,
                    help="Restrict the campaign to one gain mode")
    ap.add_argument("--force-duration", type=float, default=0.2)
    ap.add_argument("--force-axis", default="y", choices=["x", "y"],
                    help="World axis of the push (y = longitudinal forward)")
    ap.add_argument("--world-name", default="balance_test_world")
    ap.add_argument("--link-name", default="base_link")
    ap.add_argument("--startup-height", type=float, default=0.36)
    ap.add_argument("--fall-pitch-deg", type=float, default=40.0,
                    help="|pitch| beyond this marks the run as a fall")
    ap.add_argument("--startup-timeout", type=float, default=60.0,
                    help="Wall-clock timeout for paused controller startup readiness")
    ap.add_argument("--skip-existing", action="store_true",
                    help="Skip trials whose CSV already exists")
    args = ap.parse_args()

    if not (PULSE_DURATION_TOLERANCE[0] <= args.force_duration <= PULSE_DURATION_TOLERANCE[1]):
        ap.error("--force-duration must be 0.20 s (within the 0.19-0.21 s validation window)")

    if args.data_dir is None:
        args.data_dir = os.path.join(
            args.ws_root, "src/bbot_balance_controller/src/data_logs/height_campaign")
    args.log_dir = os.path.join(args.data_dir, "launch_logs")
    os.makedirs(args.data_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    env = os.environ.copy()
    env["ROS_HOME"] = "/tmp/ros_home"
    env["ROS_LOG_DIR"] = "/tmp/ros_log"
    os.makedirs("/tmp/ros_home", exist_ok=True)
    os.makedirs("/tmp/ros_log", exist_ok=True)

    trials = build_trials(args.job, args.repeats, args.force,
                          args.height_filter, args.gain_mode_filter)
    print(f"Planned trials: {len(trials)}")

    metrics = []
    for i, trial in enumerate(trials, 1):
        tag = trial_tag(trial)
        csv_path = os.path.join(args.data_dir, f"{tag}.csv")
        print(f"\n[{i}/{len(trials)}] {tag}")
        if args.skip_existing and os.path.exists(csv_path) \
                and os.path.getsize(csv_path) > 1000:
            print("  CSV exists, skipping (analyze only).")
            saved_state = None
            state_path = csv_path + ".pulse.json"
            if os.path.exists(state_path):
                try:
                    with open(state_path) as state_file:
                        saved_state = json.load(state_file)
                except (OSError, ValueError):
                    saved_state = None
            analyzed = analyze_height_trial(csv_path, trial["job"],
                                            trial["force"],
                                            args.force_duration,
                                            pulse_state=saved_state)
            metrics.append(analyzed |
                           {"job": trial["job"], "height": trial["height"],
                            "gain_mode": trial["gain_mode"], "rep": trial["rep"],
                            "force": trial["force"],
                            "fail_reason": analyzed.get("fail_reason")})
            continue
        result = run_trial(trial, args, env, args.ws_root, csv_path)
        # Self-heal: an early-death trial (spawn fall / sensor loss) gets one
        # clean retry; the failed CSV is kept as *_failed.csv for the record.
        force_missing = (trial["job"] == "push"
                         and result.get("force_applied") is False)
        retryable = any(s in str(result.get("fail_reason") or "")
                        for s in ("no CSV", "stopped logging", "csv too short"))
        if retryable or force_missing:
            failed_path = csv_path.replace(".csv", "_failed.csv")
            if os.path.exists(csv_path):
                os.replace(csv_path, failed_path)
            print(f"[Runner] retrying trial after failure: "
                  f"{result.get('fail_reason')}")
            result = run_trial(trial, args, env, args.ws_root, csv_path)
        metrics.append(result)
        summary_path = os.path.join(args.data_dir, "campaign_summary.csv")
        write_summary(aggregate(metrics), summary_path)

    summary_path = os.path.join(args.data_dir, "campaign_summary.csv")
    rows = aggregate(metrics)
    write_summary(rows, summary_path)
    print("\n================ CAMPAIGN SUMMARY ================")
    for row in rows:
        fails = f" FAILS={row['n_fail']}" if row["n_fail"] else ""
        rec = row.get("recovery_s_mean")
        rec = f" rec={rec:.2f}s" if rec is not None and np.isfinite(rec) else ""
        print(f"  {row['job']:8s} H={str(row['height']):6s} "
              f"{STATE_NAMES[row['gain_mode']]:8s} n={row['n']}{fails}{rec}")
    print(f"\nSummary written to {summary_path}")


def write_summary(rows, path, merge=True):
    if not rows:
        return
    if merge and os.path.exists(path):
        import csv as _csv
        with open(path, encoding="utf-8") as f:
            existing = list(_csv.DictReader(f))
        def key(r):
            # Normalize types: existing rows come back as strings from CSV.
            def norm(v):
                try:
                    return f"{float(v):.6g}"
                except (TypeError, ValueError):
                    return str(v)
            return (norm(r.get("job", "")), norm(r.get("height", "nan")),
                    str(r.get("gain_mode", "")), norm(r.get("force", 0.0)))
        new_keys = {key(r) for r in rows}
        rows = rows + [r for r in existing if key(r) not in new_keys]
    keys = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for row in rows:
            f.write(",".join(
                "" if row.get(k) is None else
                (f"{row[k]:.6g}" if isinstance(row[k], float) else str(row[k]))
                for k in keys) + "\n")


if __name__ == "__main__":
    main()
