#!/usr/bin/env python3
"""Offline observation regression tests; no simulator or controller required."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_flat_jump_repeatability import (  # noqa: E402
    EVENTS, FIELDS, analyze_campaign, matched_flight_pairs, validity, write_event_csv,
)
from analyze_command_transport import accepted_timing, precommand_matched  # noqa: E402


def log_row(event, stamp, pitch=0.0, rate=0.0, com_vx=0.0):
    return {
        "timestamp": f"{stamp:.3f}", "state_name": event,
        "pitch": f"{pitch:.3f}", "pitch_rate": f"{rate:.3f}",
        "capture_com_velocity": f"{com_vx:.3f}", "com_world_vz": "0.1",
        "com_sample_stamp": f"{stamp - 0.020:.3f}",
        "odom_sample_stamp": f"{stamp - 0.010:.3f}",
        "imu_sample_stamp": f"{stamp - 0.005:.3f}",
        "joint_sample_stamp": f"{stamp - 0.005:.3f}",
        "jump_cmd_rx_stamp": "1.050", "jump_cmd_accept_stamp": "1.050",
    }


class RepeatabilityTest(unittest.TestCase):
    def test_event_alignment_and_freshness(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            log = root / "trial_01_log.csv"
            events_path = root / "trial_01_events.csv"
            rows = [
                log_row("BALANCE", 1.0),
                log_row("PRE_JUMP", 1.1),
                log_row("SQUAT", 1.2),
                log_row("THRUST", 1.3),
                log_row("FLIGHT", 1.5, -0.1, -0.7),
                log_row("TOUCHDOWN_BUFFER", 1.9),
            ]
            with log.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            rows[0]["command_transport"] = "persistent"
            rows[0]["dispatch_sim_stamp"] = "1.040"
            observed = write_event_csv(log, events_path, rows[0])
            self.assertEqual(list(observed), list(EVENTS))
            self.assertEqual(observed["FLIGHT"]["com_age"], "0.020000")
            self.assertEqual(validity(observed), (True, ""))
            self.assertEqual(observed["PRE_COMMAND"]["command_transport"], "persistent")
            self.assertEqual(observed["PRE_COMMAND"]["dispatch_sim_stamp"], "1.040")
            self.assertEqual(observed["PRE_JUMP"]["jump_cmd_rx_stamp"], "1.050")
            with events_path.open(newline="") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 6)

    def test_stale_and_missing_samples_are_invalid(self):
        row = log_row("FLIGHT", 1.5)
        observed = {event: dict(row) for event in EVENTS}
        for event in observed:
            observed[event]["event"] = event
            for sensor in ("com", "odom", "imu", "joint"):
                observed[event][sensor + "_age"] = "0.005"
        observed["FLIGHT"]["joint_age"] = "0.090"
        self.assertFalse(validity(observed)[0])
        observed["FLIGHT"]["joint_age"] = ""
        self.assertFalse(validity(observed)[0])
        observed["FLIGHT"]["joint_age"] = "0.005"
        del observed["SQUAT"]  # Previous campaign sidecars remain readable.
        self.assertEqual(validity(observed), (True, ""))

    def test_command_timing_and_precommand_matching(self):
        before = log_row("BALANCE", 1.0)
        before.update({key: "0.0" for key in (
            "left_wheel_vel", "right_wheel_vel", "hip_pos_left", "hip_pos_right",
            "knee_pos_left", "knee_pos_right")})
        before.update({sensor + "_age": "0.005" for sensor in ("com", "odom", "imu", "joint")})
        a = {"PRE_COMMAND": dict(before, dispatch_sim_stamp="1.010"),
             "PRE_JUMP": {"jump_cmd_rx_stamp": "1.060",
                          "jump_cmd_accept_stamp": "1.060", "timestamp": "1.065"}}
        b = {"PRE_COMMAND": dict(before)}
        self.assertTrue(precommand_matched(a, b))
        timing, reason = accepted_timing(a)
        self.assertEqual(reason, "")
        self.assertAlmostEqual(timing["dispatch_to_receive_ms"], 50.0)
        a["PRE_JUMP"]["jump_cmd_accept_stamp"] = "1.050"
        self.assertIsNone(accepted_timing(a)[0])

    def test_matching_and_insufficient_campaign(self):
        left = {"FLIGHT": {"pitch": "0.000", "pitch_rate": "-0.700"}}
        right = {"FLIGHT": {"pitch": "0.050", "pitch_rate": "-0.400"}}
        far = {"FLIGHT": {"pitch": "0.090", "pitch_rate": "-1.200"}}
        self.assertEqual(len(matched_flight_pairs([left, right, far])), 1)
        with tempfile.TemporaryDirectory() as temp:
            report = analyze_campaign(temp)
            self.assertIn("未达到至少 6 次有效样本", report)


if __name__ == "__main__":
    unittest.main()
