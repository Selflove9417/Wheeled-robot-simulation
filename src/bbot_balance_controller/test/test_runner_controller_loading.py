#!/usr/bin/env python3
"""Behavioral regression for paused controller-manager cold-start polling."""

import importlib.util
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_flat_ground_jump_trial.py"
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location("jump_runner_loading_test", SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class RunningProcess:
    def poll(self):
        return None


def main():
    clock = FakeClock()
    original_monotonic = runner.time.monotonic
    original_time = runner.time.time
    original_sleep = runner.time.sleep
    original_controller_states = runner.controller_states
    calls = []
    try:
        # A bogus wall clock must not control the monotonic deadline.
        runner.time.monotonic = clock.monotonic
        runner.time.time = lambda: 1e9
        runner.time.sleep = clock.sleep
        ready_states = {
            "joint_state_broadcaster": "active",
            "diff_drive_controller": "active",
            "leg_position_controller": "active",
            "leg_effort_controller": "inactive",
        }

        def delayed_states(timeout):
            calls.append(timeout)
            return (ready_states if len(calls) == 3 else {}), "warming"

        runner.controller_states = delayed_states
        success, detail = runner.wait_for_controllers_loaded(RunningProcess(), timeout_sec=2.0)
        assert success
        assert "loaded_after_wall_sec=0.500" in detail
        assert len(calls) == 3
        assert all(0.0 < call <= 2.0 for call in calls)

        clock.now = 20.0
        calls.clear()

        def delayed_forever(timeout):
            calls.append(timeout)
            clock.now += 0.4
            return {}, "controller manager warming"

        runner.controller_states = delayed_forever
        success, detail = runner.wait_for_controllers_loaded(RunningProcess(), timeout_sec=1.0)
        assert not success
        assert "timed out after" in detail
        assert "physics remained paused" in detail
        assert calls and calls[-1] < calls[0]
    finally:
        runner.time.monotonic = original_monotonic
        runner.time.time = original_time
        runner.time.sleep = original_sleep
        runner.controller_states = original_controller_states
    print("PASS: paused controller loading uses monotonic remaining budget")


if __name__ == "__main__":
    main()
