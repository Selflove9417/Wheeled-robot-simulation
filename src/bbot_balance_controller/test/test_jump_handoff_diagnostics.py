#!/usr/bin/env python3
"""Regression tests for observational jump failure classification."""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_jump_handoff import analyze, classify_attempt  # noqa: E402


def row(state, request="-1", ack="-1", result="-1", gate="-1"):
    return {"state_name": state, "effort_switch_request_stamp": request,
            "effort_switch_ack_stamp": ack, "effort_switch_result": result,
            "thrust_gate_open_stamp": gate, "thrust_gate_stable_elapsed": "0.005"}


class HandoffDiagnosticsTest(unittest.TestCase):
    def test_earliest_failure_classification(self):
        self.assertEqual(classify_attempt([], {"reason": "Initial balance stabilization timed out"}),
                         "STARTUP_OR_BALANCE")
        self.assertEqual(classify_attempt([row("BALANCE")],
                                          {"reason": "Jump command not received by controller within 2 s"}),
                         "COMMAND_NOT_RECEIVED")
        self.assertEqual(classify_attempt([row("PRE_JUMP"), row("SQUAT")], {}),
                         "PRE_THRUST_FAILURE")
        prefix = [row("PRE_JUMP"), row("THRUST", "1.0", "-1", "0")]
        self.assertEqual(classify_attempt(prefix, {}), "SWITCH_UNCONFIRMED")
        self.assertEqual(classify_attempt(prefix + [row("THRUST", "1.0", "1.05", "2")], {}),
                         "SWITCH_FAILED")
        self.assertEqual(classify_attempt(prefix + [row("THRUST", "1.0", "1.05", "1")], {}),
                         "THRUST_GATE_TIMEOUT")
        self.assertEqual(classify_attempt(prefix + [row("THRUST", "1.0", "1.25", "1")], {}),
                         "SWITCH_DELAYED")
        self.assertEqual(classify_attempt(prefix + [row("THRUST", "1.0", "1.05", "1", "1.1")], {}),
                         "POST_GATE_FAILURE")
        self.assertEqual(classify_attempt(prefix + [row("FLIGHT", "1.0", "1.05", "1")], {}),
                         "FLIGHT_REACHED")

    def test_early_failure_is_reported_without_summary_or_log(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "trial_01_early_outcome.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=("trial", "success", "reason", "valid_flight"))
                writer.writeheader()
                writer.writerow({"trial": 1, "success": False,
                                 "reason": "Initial balance stabilization timed out"})
            report = analyze(temp)
            self.assertIn("STARTUP_OR_BALANCE", report)
            self.assertIn("Initial balance stabilization timed out", report)


if __name__ == "__main__":
    unittest.main()
