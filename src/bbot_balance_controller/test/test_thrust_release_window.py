"""Regression checks for stamp-aligned thrust-window comparisons."""

import contextlib
import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_thrust_release_window import Trial, print_matched_momentum  # noqa: E402


def make_row(stamp, wheel_rate):
    row = {
        "timestamp": f"{stamp:.3f}",
        "imu_sample_stamp": f"{stamp:.3f}",
        "joint_sample_stamp": f"{stamp:.3f}",
        "pitch_rate_raw": "0",
        "left_wheel_vel": str(wheel_rate),
        "right_wheel_vel": str(wheel_rate),
    }
    for side in ("left", "right"):
        for joint in ("hip", "knee"):
            row[f"{joint}_pos_{side}"] = "0"
            row[f"{joint}_vel_{side}"] = "0"
    return row


def make_trial(tag, stamps, wheel_rate):
    trial = Trial.__new__(Trial)
    trial.tag = tag
    trial.name = tag
    trial.t0 = 0.0
    trial.thrust = [make_row(stamp, wheel_rate) for stamp in stamps]
    return trial


class MatchedMomentumTests(unittest.TestCase):
    def setUp(self):
        self.trials = [
            make_trial("A", (0.150, 0.160, 0.170, 0.180, 0.190), 1.0),
            make_trial("B", (0.155, 0.165, 0.175, 0.185, 0.195), 2.0),
            make_trial("C", (0.155, 0.165, 0.175, 0.185, 0.195), 2.0),
        ]

    def output(self, start, stop):
        args = SimpleNamespace(ref="A", start=start, stop=stop, step=0.005)
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            print_matched_momentum(self.trials, args, 0.005)
        return stream.getvalue()

    def test_off_grid_window_start_keeps_aligned_rows(self):
        output = self.output(0.145, 0.200)
        self.assertIn("+0.150", output)
        self.assertIn("+0.190", output)
        self.assertNotIn("UNAVAILABLE", output)

    def test_narrow_off_grid_window_starts_at_next_valid_sample(self):
        output = self.output(0.155, 0.195)
        self.assertIn("+0.160", output)
        self.assertIn("+0.190", output)
        self.assertNotIn("+0.150", output)


if __name__ == "__main__":
    unittest.main()
