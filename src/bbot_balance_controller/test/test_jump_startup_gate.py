#!/usr/bin/env python3
"""Regression tests for the paused jump-startup interlock."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


MODULE = Path(__file__).resolve().parents[1] / "scripts" / "jump_startup_gate.py"
spec = importlib.util.spec_from_file_location("jump_startup_gate", MODULE)
gate = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = gate
spec.loader.exec_module(gate)


class StartupGateTests(unittest.TestCase):
    def setUp(self):
        self.evidence = gate.StartupEvidence()
        for name in ("joint", "imu", "odom"):
            self.evidence.add_sensor(name, 0.090, (1.0, 2.0))
            self.evidence.add_sensor(name, 0.095, (1.0, 2.0))
        self.evidence.add_command("wheel", 0.090, (0.1, 0.0))
        self.evidence.add_command("wheel", 0.095, (0.2, 0.0))
        self.evidence.add_command("leg", 0.090, (0.0, 0.0, 0.0, 0.0))
        self.evidence.add_command("leg", 0.095, (0.01, 0.0, 0.01, 0.0))

    def test_delayed_activation_never_allows_unpause(self):
        # Even with fresh commands/sensors and discovered endpoints, waiting
        # for a delayed controller switch must keep the world paused.
        self.assertFalse(self.evidence.ready(0.100, False, True, True))
        self.assertFalse(self.evidence.ready(0.100, True, False, True))
        self.assertFalse(self.evidence.ready(0.100, True, True, False))
        self.assertTrue(self.evidence.ready(0.100, True, True, True))

    def test_duplicate_or_invalid_samples_do_not_prove_ready(self):
        evidence = gate.StartupEvidence()
        for name in ("joint", "imu", "odom"):
            evidence.add_sensor(name, 0.090, (1.0,))
            evidence.add_sensor(name, 0.090, (2.0,))
            evidence.add_sensor(name, 0.095, (float("nan"),))
        for name in ("wheel", "leg"):
            evidence.add_command(name, 0.090, (1.0,))
            evidence.add_command(name, 0.090, (2.0,))
        self.assertFalse(evidence.ready(0.100, True, True, True))

    def test_timestamp_conflict_rollback_and_empty_frame_reset_channel(self):
        evidence = gate.StartupEvidence()
        evidence.add_sensor("odom", 0.010, (1.0, 2.0))
        evidence.add_sensor("odom", 0.015, (1.0, 2.0))
        self.assertEqual(len(evidence.sensor_stamps["odom"]), 2)
        evidence.add_sensor("odom", 0.015, (9.0, 2.0))
        self.assertEqual(len(evidence.sensor_stamps["odom"]), 0)
        evidence.add_sensor("odom", 0.020, (1.0, 2.0))
        evidence.add_sensor("odom", 0.018, (1.0, 2.0))
        self.assertEqual(len(evidence.sensor_stamps["odom"]), 0)
        evidence.add_sensor("odom", 0.025, ())
        self.assertEqual(len(evidence.sensor_stamps["odom"]), 0)

    def test_sensor_command_freshness_is_required(self):
        self.assertFalse(self.evidence.ready(0.200, True, True, True))
        self.assertFalse(self.evidence.ready(0.080, True, True, True))

    def test_readiness_control_flow_waits_for_delayed_activation(self):
        probe_count = 0
        step_calls = []
        unpause_calls = []

        def probe():
            nonlocal probe_count
            probe_count += 1
            now = 0.100 + 0.005 * max(0, probe_count - 1)
            activated = probe_count >= 3
            return now, activated, True, True

        ok, _, steps = gate.drive_readiness_interlock(
            self.evidence, probe,
            lambda: step_calls.append(True) or True,
            lambda: unpause_calls.append(True) or (True, ""),
            max_steps=5, deadline=10.0, monotonic=lambda: 0.0)
        self.assertTrue(ok)
        self.assertEqual(steps, 2)
        self.assertEqual(len(step_calls), 2)
        self.assertEqual(len(unpause_calls), 1)

    def test_batched_steps_share_physical_tick_budget_including_activation(self):
        # Controller activation has already consumed 10 of the 100 allowed
        # 1 ms ticks. Readiness requires more sensor frames, but the gate may
        # only spend the remaining 90 ticks (18 five-tick calls), then must
        # remain paused and spin until the callbacks complete.
        used = 10
        step_calls = []
        idle_calls = []
        unpause_calls = []

        def probe():
            ready = len(idle_calls) >= 2
            return 0.100, ready, ready, ready

        def step():
            nonlocal used
            used = min(100, used + 5)
            step_calls.append(used)
            return True

        ok, _, ticks = gate.drive_readiness_interlock(
            self.evidence, probe, step,
            lambda: unpause_calls.append(True) or (True, ""),
            max_steps=100, deadline=10.0, monotonic=lambda: 0.0,
            wait_after_budget=True, idle=lambda: idle_calls.append(True),
            steps_used=lambda: used)
        self.assertTrue(ok)
        self.assertEqual(ticks, 100)
        self.assertEqual(step_calls, list(range(15, 101, 5)))
        self.assertEqual(len(idle_calls), 2)
        self.assertEqual(unpause_calls, [True])

    def test_not_ready_or_budget_failure_never_unpauses(self):
        unpause_calls = []
        ok, reason, steps = gate.drive_readiness_interlock(
            self.evidence, lambda: (0.100, False, True, True), lambda: True,
            lambda: unpause_calls.append(True) or (True, ""),
            max_steps=3, deadline=10.0, monotonic=lambda: 0.0)
        self.assertFalse(ok)
        self.assertIn("budget", reason)
        self.assertEqual(steps, 3)
        self.assertEqual(unpause_calls, [])

    def test_failed_unpause_is_not_reported_as_ready(self):
        ok, reason, steps = gate.drive_readiness_interlock(
            self.evidence, lambda: (0.100, True, True, True), lambda: self.fail(),
            lambda: (False, "service refused"), max_steps=1, deadline=10.0,
            monotonic=lambda: 0.0)
        self.assertFalse(ok)
        self.assertIn("service refused", reason)
        self.assertEqual(steps, 0)

    def test_real_step_method_obeys_remaining_physics_budget(self):
        obj = gate.JumpStartupGate.__new__(gate.JumpStartupGate)
        obj.world = "flat_jump_world"
        obj.steps_used = 95
        obj.max_steps = 100
        obj.step_batch = 5
        obj.errors = []
        obj._spin = lambda _timeout=0.01: None
        with mock.patch.object(gate, "world_control", return_value=(True, "data: true")) as control:
            self.assertTrue(obj._step())
            self.assertEqual(obj.steps_used, 100)
            self.assertFalse(obj._step())
        self.assertEqual(control.call_count, 1)
        self.assertIn("multi_step: 5", control.call_args.args[1])

    def test_real_switch_method_keeps_world_stepping_paused_until_ack(self):
        class Request:
            STRICT = 2

            def __init__(self):
                self.activate_controllers = []
                self.deactivate_controllers = []
                self.strictness = 0
                self.activate_asap = False
                self.timeout = type("Timeout", (), {"sec": 0})()

        class Response:
            ok = True

        class Future:
            def __init__(self, obj):
                self.obj = obj

            def done(self):
                return (self.obj.steps_used >= 10 and
                        self.obj.wait_spins >= 1)

            def result(self):
                return Response()

        class Client:
            def __init__(self, obj):
                self.obj = obj
                self.request = None

            def service_is_ready(self):
                return True

            def call_async(self, request):
                self.request = request
                return Future(self.obj)

        obj = gate.JumpStartupGate.__new__(gate.JumpStartupGate)
        obj.switch_client = Client(obj)
        obj.state_client = type("StateClient", (), {"service_is_ready": lambda self: False})()
        obj.SwitchController = type("Switch", (), {"Request": Request})
        obj.timeout_s = 2.0
        obj.steps_used = 0
        obj.max_steps = 20
        obj.max_steps = 10
        obj.required_active = {"joint_state_broadcaster", "diff_drive_controller",
                               "leg_position_controller"}
        obj.wait_spins = 0

        def spin(_timeout=0.01):
            if obj.steps_used >= obj.max_steps:
                obj.wait_spins += 1

        obj._spin = spin
        obj._drain_callbacks = lambda count=20: None
        obj._record_diag = lambda *_args, **_kwargs: None
        step_calls = []

        def step():
            obj.steps_used = min(obj.max_steps, obj.steps_used + 5)
            step_calls.append(obj.steps_used)
            return True

        obj._step = step
        ok, message = obj._switch_staged(gate.time.monotonic() + 2.0)
        self.assertTrue(ok, message)
        self.assertEqual(step_calls, [5, 10])
        self.assertEqual(obj.wait_spins, 1)
        self.assertEqual(obj.switch_client.request.strictness, Request.STRICT)
        self.assertEqual(obj.switch_client.request.deactivate_controllers, [])

    def test_switch_wall_timeout_after_budget_does_not_step_again(self):
        class Request:
            STRICT = 2

            def __init__(self):
                self.activate_controllers = []
                self.deactivate_controllers = []
                self.strictness = 0
                self.activate_asap = False
                self.timeout = type("Timeout", (), {"sec": 0})()

        class Future:
            def done(self):
                return False

        class Client:
            def service_is_ready(self):
                return True

            def call_async(self, _request):
                return Future()

        obj = gate.JumpStartupGate.__new__(gate.JumpStartupGate)
        obj.switch_client = Client()
        obj.state_client = type("StateClient", (), {"service_is_ready": lambda self: False})()
        obj.SwitchController = type("Switch", (), {"Request": Request})
        obj.timeout_s = 1.0
        obj.steps_used = 0
        obj.max_steps = 5
        obj.required_active = {"joint_state_broadcaster", "diff_drive_controller",
                               "leg_position_controller"}
        obj._drain_callbacks = lambda count=20: None
        obj._record_diag = lambda *_args, **_kwargs: None
        obj._spin = lambda _timeout=0.01: __import__("time").sleep(0.005)
        step_calls = []

        def step():
            obj.steps_used += 5
            step_calls.append(obj.steps_used)
            return True

        obj._step = step
        ok, message = obj._switch_staged(gate.time.monotonic() + 0.03)
        self.assertFalse(ok)
        self.assertIn("timed out", message)
        self.assertEqual(step_calls, [5])

    def test_failure_path_confirms_pause_after_unpause_error(self):
        with mock.patch.object(gate, "world_control",
                               side_effect=[(False, "lost ack"), (True, "paused")]) as control:
            message = gate.failure_pause_message("flat_jump_world", "ready but unpause failed")
        self.assertIn("failure pause confirmed", message)
        self.assertEqual(control.call_count, 2)
        self.assertEqual(control.call_args.args[1], "pause: true")

    def test_default_launch_and_runner_share_the_installed_gate(self):
        root = Path(__file__).resolve().parents[3]
        launch = (root / "src/bbot_bringup/launch/bbot_gazebo.launch.py").read_text()
        runner = (root / "src/bbot_balance_controller/scripts/run_flat_ground_jump_trial.py").read_text()
        self.assertIn("'gs_lqr', 'jump_velocity'", launch)
        self.assertIn("'jump_velocity' else 'false'", launch)
        self.assertIn("'gs_lqr', 'mpc', 'jump_velocity'", launch)
        self.assertIn("staged_jump_velocity_startup_gate", launch)
        self.assertIn('"jump_startup_gate.py"', launch)
        self.assertIn('"jump_startup_gate.py"', runner)
        self.assertIn('jump_default("thrust_release_velocity_ratio", 0.70)', launch)
        self.assertIn('"thrust_release_velocity_ratio"', launch)
        self.assertIn('thrust_release_velocity_ratio:=', runner)
        self.assertIn('choices=(0.70, 0.74, 0.78)', runner)
        self.assertIn('launch_gate_scope_expression(', launch)
        self.assertIn('validate_fast_rate_launch', launch)

    def test_runner_saves_startup_gate_stdout_for_success_and_failure(self):
        runner_path = Path(__file__).resolve().parents[1] / "scripts" / "run_flat_ground_jump_trial.py"
        runner_spec = importlib.util.spec_from_file_location("jump_trial_runner_test", runner_path)
        runner = importlib.util.module_from_spec(runner_spec)
        sys.modules[runner_spec.name] = runner
        runner_spec.loader.exec_module(runner)

        class LaunchProcess:
            def poll(self):
                return None

        class GateProcess:
            def __init__(self, returncode):
                self.returncode = returncode

            def communicate(self, timeout=None):
                return ("[JumpStartupGate][DIAG] switch_ack_ok\n"
                        "[JumpStartupGate] READY: verified", None)

        with tempfile.TemporaryDirectory() as temp:
            for returncode, expected in ((0, True), (1, False)):
                path = Path(temp) / f"gate_{returncode}.txt"
                with mock.patch.object(runner.subprocess, "Popen",
                                       return_value=GateProcess(returncode)) as popen:
                    ok, output = runner.activate_controllers_with_first_physics_tick(
                        LaunchProcess(), diagnostic_file=path)
                self.assertEqual(ok, expected)
                self.assertIn("switch_ack_ok", output)
                self.assertIn("jump_startup_gate.py", path.read_text())
                self.assertTrue(popen.call_args.kwargs["start_new_session"])

    def test_runner_exception_pauses_and_terminates_only_its_gate_group(self):
        runner_path = Path(__file__).resolve().parents[1] / "scripts" / "run_flat_ground_jump_trial.py"
        runner_spec = importlib.util.spec_from_file_location("jump_trial_runner_cleanup_test", runner_path)
        runner = importlib.util.module_from_spec(runner_spec)
        sys.modules[runner_spec.name] = runner
        runner_spec.loader.exec_module(runner)

        class LaunchProcess:
            def poll(self):
                return None

        class BrokenGateProcess:
            returncode = None

            def communicate(self, timeout=None):
                raise OSError("pipe failure")

        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "gate.txt"
            with (mock.patch.object(runner.subprocess, "Popen",
                                    return_value=BrokenGateProcess()),
                  mock.patch.object(runner, "pause_flat_world") as pause,
                  mock.patch.object(runner, "terminate_process_tree") as terminate):
                ok, output = runner.activate_controllers_with_first_physics_tick(
                    LaunchProcess(), diagnostic_file=path)
            self.assertFalse(ok)
            self.assertIn("unable to start shared jump startup gate", output)
            pause.assert_called_once_with()
            terminate.assert_called_once()
            self.assertTrue(path.exists())

    def test_physics_budget_is_bounded_and_failure_does_not_extend_it(self):
        self.assertEqual(gate.physics_step_budget(0.100), 100)
        self.assertEqual(gate.physics_step_budget(0.025), 25)
        for value in (0.0, -0.01, 0.101, float("nan")):
            with self.assertRaises(ValueError):
                gate.physics_step_budget(value)

    def test_initial_runner_gate_requires_world_valid_and_jump_ready(self):
        row = {"state_name": "BALANCE", "pitch": "0.034", "pitch_rate": "0.0",
               "capture_com_velocity": "0.0", "capture_world_valid": "1",
               "jump_ready": "1"}
        self.assertTrue(gate.initial_balance_row_stable(row))
        stale = dict(row, capture_world_valid="0")
        self.assertFalse(gate.initial_balance_row_stable(stale))
        not_ready = dict(row, jump_ready="0")
        self.assertFalse(gate.initial_balance_row_stable(not_ready))
        self.assertFalse(gate.initial_balance_row_stable(dict(row, pitch="0.10")))


if __name__ == "__main__":
    unittest.main()
