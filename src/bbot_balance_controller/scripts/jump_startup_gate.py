#!/usr/bin/env python3
"""Keep jump physics paused until controllers, sensors and commands are live."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Sequence, Tuple


STEP_SIZE_S = 0.001
DEFAULT_MAX_PHYSICS_S = 0.100
STEP_BATCH = 5
SENSOR_MAX_AGE_S = 0.080


def world_control(world: str, request: str, timeout: float = 3.0) -> Tuple[bool, str]:
    cmd = [
        "ign", "service", "-s", f"/world/{world}/control",
        "--reqtype", "ignition.msgs.WorldControl",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", str(int(timeout * 1000)), "--req", request,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 2.0)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0 and "data: true" in output.lower(), output


def ensure_world_paused(world: str, wall_timeout: float = 3.0) -> Tuple[bool, str]:
    deadline = time.monotonic() + wall_timeout
    detail = "world control service unavailable"
    while time.monotonic() < deadline:
        ok, detail = world_control(world, "pause: true", timeout=1.0)
        if ok:
            return True, detail
    return False, detail


def failure_pause_message(world: str, message: str) -> str:
    paused, detail = ensure_world_paused(world)
    return (message + ("; failure pause confirmed" if paused else
                       f"; FAILURE: unable to confirm paused world: {detail}"))


def finite_values(values: Iterable[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def physics_step_budget(max_physics_s: float, step_size: float = STEP_SIZE_S) -> int:
    if (not math.isfinite(max_physics_s) or not math.isfinite(step_size) or
            max_physics_s <= 0.0 or step_size <= 0.0 or
            max_physics_s > DEFAULT_MAX_PHYSICS_S + 1e-12):
        raise ValueError("physics budget must be in (0, 0.100] seconds")
    return int(math.floor(max_physics_s / step_size + 1e-9))


def initial_balance_row_stable(row: dict) -> bool:
    """Preserve the original quiet thresholds while requiring fresh readiness."""
    try:
        return (row.get("state_name") == "BALANCE" and
                row.get("capture_world_valid", "0") == "1" and
                row.get("jump_ready", "0") == "1" and
                abs(float(row["pitch"]) - 0.034) < 0.05 and
                abs(float(row["pitch_rate"])) < 0.15 and
                abs(float(row["capture_com_velocity"])) < 0.10)
    except (KeyError, TypeError, ValueError):
        return False


def drive_readiness_interlock(evidence: "StartupEvidence", probe, step, unpause,
                              max_steps: int, deadline: float,
                              monotonic=time.monotonic, wait_after_budget=False,
                              idle=None, steps_used=None) -> Tuple[bool, str, int]:
    """Exercise the exact bounded gate loop using injectable runtime hooks."""
    # `step` may advance a batch of physics ticks. When supplied, steps_used is
    # the authoritative cumulative tick counter shared with activation; never
    # count service calls as if each represented one tick.
    steps = int(steps_used()) if steps_used is not None else 0
    while monotonic() < deadline:
        now, active, exclusive, endpoints = probe()
        if evidence.ready(now, active, exclusive, endpoints):
            ok, detail = unpause()
            if ok:
                return True, "readiness proven", steps
            return False, f"unpause rejected: {detail}", steps
        current_steps = int(steps_used()) if steps_used is not None else steps
        steps = current_steps
        if current_steps >= max_steps:
            if not wait_after_budget:
                return False, "paused physics-step budget exhausted", steps
            if idle is not None:
                idle()
            else:
                time.sleep(0.005)
            continue
        previous_steps = current_steps
        if not step():
            return False, "paused physics step failed", steps
        if steps_used is not None:
            steps = int(steps_used())
            if steps <= previous_steps or steps > max_steps:
                return False, "physics step counter did not advance within budget", steps
        else:
            steps += 1
    return False, "startup readiness timed out", steps


@dataclass
class StartupEvidence:
    """Pure readiness state, also used by the startup regression tests."""

    sensor_stamps: Dict[str, deque] = field(default_factory=lambda: {
        name: deque(maxlen=4) for name in ("joint", "imu", "odom")
    })
    command_stamps: Dict[str, deque] = field(default_factory=lambda: {
        name: deque(maxlen=4) for name in ("wheel", "leg")
    })
    sensor_values: Dict[str, dict] = field(default_factory=lambda: {
        name: {} for name in ("joint", "imu", "odom")
    })
    command_values: Dict[str, dict] = field(default_factory=lambda: {
        name: {} for name in ("wheel", "leg")
    })
    malformed: bool = False

    @staticmethod
    def ingest(samples: deque, values_by_stamp: dict,
               stamp: float, values: Iterable[float]) -> None:
        try:
            stamp = float(stamp)
            values = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            samples.clear()
            values_by_stamp.clear()
            return
        if (not math.isfinite(stamp) or stamp < 0.0 or not values or
                not finite_values(values)):
            samples.clear()
            values_by_stamp.clear()
            return
        if not samples:
            samples.append(stamp)
            values_by_stamp[stamp] = values
            return
        previous = samples[-1]
        if stamp < previous - 1e-9:
            samples.clear()
            values_by_stamp.clear()
            return
        if abs(stamp - previous) <= 1e-9:
            if values_by_stamp.get(previous) != values:
                samples.clear()
                values_by_stamp.clear()
            return
        samples.append(stamp)
        values_by_stamp[stamp] = values
        while len(values_by_stamp) > samples.maxlen:
            del values_by_stamp[next(iter(values_by_stamp))]

    def add_sensor(self, name: str, stamp: float, values: Iterable[float]) -> None:
        if name not in self.sensor_stamps:
            return
        self.ingest(self.sensor_stamps[name], self.sensor_values[name], stamp, values)

    def add_command(self, name: str, stamp: float, values: Iterable[float]) -> None:
        if name not in self.command_stamps:
            return
        self.ingest(self.command_stamps[name], self.command_values[name], stamp, values)

    def ready(self, now: float, active: bool, exclusive: bool,
              endpoints: bool, max_age: float = SENSOR_MAX_AGE_S) -> bool:
        if not (active and exclusive and endpoints and math.isfinite(now)):
            return False
        for stamps in self.sensor_stamps.values():
            if len(stamps) < 2 or now - stamps[-1] < -1e-6 or now - stamps[-1] > max_age:
                return False
        for stamps in self.command_stamps.values():
            if len(stamps) < 2 or now - stamps[-1] < -1e-6 or now - stamps[-1] > max_age:
                return False
        return True


class JumpStartupGate:
    def __init__(self, world: str, wheel_actuation: str, activate_staged: bool,
                 timeout_s: float, max_physics_s: float, step_batch: int = STEP_BATCH):
        import rclpy
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float64MultiArray

        self.rclpy = rclpy
        self.ListControllers = ListControllers
        self.SwitchController = SwitchController
        self.world = world
        self.wheel_actuation = wheel_actuation
        self.activate_staged = activate_staged
        self.timeout_s = timeout_s
        self.max_steps = physics_step_budget(max_physics_s)
        self.step_batch = max(1, min(step_batch, self.max_steps or 1))
        self.steps_used = 0
        self.evidence = StartupEvidence()
        self.last_clock = float("nan")
        self.errors = []
        self.diagnostics = []
        self.node = Node("jump_startup_gate")
        self.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.state_client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.switch_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller")
        self.wheel_controller = ("wheel_effort_controller" if wheel_actuation == "effort"
                                 else "diff_drive_controller")
        self.other_wheel_controller = ("diff_drive_controller" if wheel_actuation == "effort"
                                       else "wheel_effort_controller")
        self.required_active = {"joint_state_broadcaster", self.wheel_controller,
                                "leg_position_controller"}
        self.required_loaded = set(self.required_active) | {"leg_effort_controller"}
        self.node.create_subscription(JointState, "/joint_states", self._joint, 30)
        self.node.create_subscription(Imu, "/imu", self._imu, 20)
        self.node.create_subscription(Odometry, "/model/bbot/odometry", self._odom, 20)
        self.node.create_subscription(TwistStamped, "/diff_drive_controller/cmd_vel",
                                      self._wheel_command, 20)
        self.node.create_subscription(Float64MultiArray,
                                      "/leg_position_controller/commands",
                                      self._leg_command, 20)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _joint(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        required = ("link_002_joint", "link_003_joint", "link_004_joint",
                    "link_005_joint", "link_006_joint", "link_007_joint")
        if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            self.evidence.add_sensor("joint", float("nan"), ())
            return
        if any(name not in msg.name for name in required):
            self.evidence.add_sensor("joint", float("nan"), ())
            return
        vals = []
        for name in required:
            index = msg.name.index(name)
            vals.extend((msg.position[index], msg.velocity[index]))
        self.evidence.add_sensor("joint", stamp, vals)

    def _imu(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation
        w = msg.angular_velocity
        a = msg.linear_acceleration
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0.8 <= q_norm <= 1.2:
            self.evidence.add_sensor("imu", float("nan"), ())
            return
        self.evidence.add_sensor("imu", stamp, (q.x, q.y, q.z, q.w,
                                                 w.x, w.y, w.z, a.x, a.y, a.z))

    def _odom(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0.8 <= q_norm <= 1.2:
            self.evidence.add_sensor("odom", float("nan"), ())
            return
        self.evidence.add_sensor("odom", stamp,
                                 (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                  v.x, v.y, v.z, w.x, w.y, w.z))

    def _wheel_command(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.linear
        w = msg.twist.angular
        if not finite_values((v.x, v.y, v.z, w.x, w.y, w.z)):
            self.evidence.add_command("wheel", float("nan"), ())
            return
        self.evidence.add_command("wheel", stamp, (v.x, v.y, v.z, w.x, w.y, w.z))

    def _leg_command(self, msg) -> None:
        stamp = self._now()
        if len(msg.data) != 4:
            self.evidence.add_command("leg", float("nan"), ())
            return
        self.evidence.add_command("leg", stamp, msg.data)

    def _spin(self, timeout=0.01) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def _drain_callbacks(self, count=20) -> None:
        for _ in range(count):
            self._spin(0.001)

    def _record_diag(self, label: str, states=None) -> None:
        sensors = {name: (values[-1] if values else None)
                   for name, values in self.evidence.sensor_stamps.items()}
        commands = {name: (values[-1] if values else None)
                    for name, values in self.evidence.command_stamps.items()}
        self.diagnostics.append(
            f"{label}: sim={self._now():.6f} steps={self.steps_used}/{self.max_steps} "
            f"sensor_stamps={sensors} command_stamps={commands} controllers={states}")

    def _list_states(self, timeout=0.5) -> Optional[Dict[str, str]]:
        if not self.state_client.service_is_ready():
            return None
        future = self.state_client.call_async(self.ListControllers.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self._spin(0.02)
        if not future.done():
            return None
        try:
            response = future.result()
        except Exception as exc:  # service transition can race unload/reload
            self.errors.append(str(exc))
            return None
        return {item.name: item.state for item in response.controller}

    def _step(self) -> bool:
        if self.steps_used >= self.max_steps:
            return False
        count = min(self.step_batch, self.max_steps - self.steps_used)
        ok, detail = world_control(self.world, f"pause: true multi_step: {count}")
        if not ok:
            self.errors.append(f"paused physics step failed: {detail}")
            return False
        self.steps_used += count
        self._drain_callbacks()
        return True

    def _endpoint_ready(self) -> bool:
        return (self.node.count_publishers("/diff_drive_controller/cmd_vel") >= 1 and
                self.node.count_publishers("/leg_position_controller/commands") >= 1)

    def _switch_staged(self, deadline: float) -> Tuple[bool, str]:
        service_deadline = min(deadline, time.monotonic() + min(5.0, self.timeout_s))
        while (not self.switch_client.service_is_ready() and
               time.monotonic() < service_deadline):
            self._spin(0.05)
        if not self.switch_client.service_is_ready():
            return False, "controller switch service unavailable while physics remained paused"
        request = self.SwitchController.Request()
        request.activate_controllers = sorted(self.required_active)
        request.deactivate_controllers = []
        request.strictness = self.SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        future = self.switch_client.call_async(request)
        self._record_diag("switch_request_pending")
        budget_reported = False
        while not future.done() and time.monotonic() < deadline:
            self._drain_callbacks(count=5)
            if future.done():
                break
            if self.steps_used < self.max_steps:
                if not self._step():
                    return False, "unable to step paused physics while applying controller switch"
            else:
                if not budget_reported:
                    self._record_diag("switch_pending_after_step_budget")
                    budget_reported = True
                self._spin(0.05)
        if not future.done():
            states = self._list_states(timeout=0.5)
            self._record_diag("switch_ack_wall_timeout", states)
            return False, "controller switch response timed out at shared startup wall deadline"
        try:
            response = future.result()
        except Exception as exc:
            self._record_diag(f"switch_ack_exception={exc}")
            return False, f"controller switch response failed: {exc}"
        if response is None or not response.ok:
            states = self._list_states(timeout=0.5)
            self._record_diag("switch_ack_rejected", states)
            return False, "controller manager rejected staged activation"
        states = self._list_states(timeout=0.5)
        self._record_diag("switch_ack_ok", states)
        return True, ""

    def run(self) -> Tuple[bool, str]:
        deadline = time.monotonic() + self.timeout_s
        paused = False
        detail = "world control service is not ready"
        while time.monotonic() < deadline:
            paused, detail = world_control(self.world, "pause: true", timeout=1.0)
            if paused:
                break
            self._spin(0.05)
        if not paused:
            return False, f"could not confirm world paused before startup gate: {detail}"
        if self.activate_staged:
            while time.monotonic() < deadline:
                states = self._list_states()
                if states is not None and self.required_loaded.issubset(states):
                    break
                self._spin(0.05)
            else:
                return False, "required staged controllers did not load before timeout"
            if not self._endpoint_ready():
                # Do not spend physics budget until the jump node has created
                # both command endpoints; it starts from a separate timer.
                while time.monotonic() < deadline and not self._endpoint_ready():
                    self._spin(0.05)
                if not self._endpoint_ready():
                    return False, "jump controller command endpoints did not appear"
            ok, why = self._switch_staged(deadline)
            if not ok:
                return False, why
        else:
            # A non-staged launch may use its existing spawners, but the gate
            # never activates controllers itself. Wait for them to become
            # active while paused; fail safely if that startup profile cannot.
            while time.monotonic() < deadline:
                states = self._list_states()
                loaded = states is not None and self.required_loaded.issubset(states)
                if loaded and self._endpoint_ready():
                    active = self.required_active.issubset(
                        {name for name, state in states.items() if state == "active"})
                    exclusive = (states.get(self.other_wheel_controller) != "active" and
                                 states.get("leg_effort_controller") != "active")
                    if active and exclusive:
                        break
                    if not exclusive:
                        return False, "non-staged wheel/leg controller exclusivity check failed"
                self._spin(0.05)
            else:
                return False, "non-staged controllers/endpoints did not become ready while paused"

        def probe():
            states = self._list_states()
            endpoints = self._endpoint_ready()
            active = states is not None and self.required_active.issubset(
                {name for name, state in states.items() if state == "active"})
            exclusive = (states is not None and
                         states.get(self.other_wheel_controller) != "active" and
                         states.get("leg_effort_controller") != "active")
            now = self._now()
            self._last_states = states
            return now, active, exclusive, endpoints

        def step():
            ok = self._step()
            return ok

        def unpause():
            return world_control(self.world, "pause: false")

        ok, detail, _ = drive_readiness_interlock(
            self.evidence, probe, step, unpause, self.max_steps, deadline,
            wait_after_budget=True, idle=lambda: self._spin(0.01),
            steps_used=lambda: self.steps_used)
        self._record_diag("ready" if ok else f"not_ready={detail}",
                          getattr(self, "_last_states", None))
        if not ok:
            return False, (f"{detail} (steps={self.steps_used}, sim={self._now():.3f})")
        return True, (f"ready after {self.steps_used} paused physics steps "
                      f"({self.steps_used * STEP_SIZE_S:.3f}s); controllers and "
                      "two distinct sensor/command frames verified")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="flat_jump_world")
    parser.add_argument("--wheel-actuation", choices=("velocity", "effort"), default="velocity")
    parser.add_argument("--activate-staged", action="store_true")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--max-physics-seconds", type=float, default=DEFAULT_MAX_PHYSICS_S)
    args = parser.parse_args(argv)
    try:
        physics_step_budget(args.max_physics_seconds)
    except ValueError as exc:
        parser.error(str(exc))
    import rclpy
    rclpy.init(args=None)
    gate = None
    try:
        gate = JumpStartupGate(args.world, args.wheel_actuation, args.activate_staged,
                               args.timeout, args.max_physics_seconds)
        ok, message = gate.run()
        for item in gate.diagnostics:
            print(f"[JumpStartupGate][DIAG] {item}", flush=True)
        if not ok:
            message = failure_pause_message(args.world, message)
        print(f"[JumpStartupGate] {'READY' if ok else 'FAILED'}: {message}", flush=True)
        return 0 if ok else 1
    except Exception as exc:
        paused, detail = ensure_world_paused(args.world)
        suffix = "; failure pause confirmed" if paused else f"; pause failed: {detail}"
        if gate is not None:
            for item in gate.diagnostics:
                print(f"[JumpStartupGate][DIAG] {item}", flush=True)
        print(f"[JumpStartupGate] FAILED: {exc}{suffix}", file=sys.stderr, flush=True)
        return 1
    finally:
        if gate is not None:
            gate.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
