#!/usr/bin/env python3
"""Boundary tests for the offline latch-height comparison."""

import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "analyze_latch_height.py"
SPEC = importlib.util.spec_from_file_location("analyze_latch_height", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(**changes):
    values = {
        "state_name": "THRUST", "velocity_reached": "0",
        "com_velocity_valid": "1", "target_takeoff_velocity": "2.0",
        "timestamp": "1.01", "com_sample_stamp": "1.00",
        "com_world_vz": "1.8", "com_world_z": "0.4",
    }
    values.update(changes)
    return values


def valid_case():
    return [
        row(),
        row(timestamp="1.015"),  # repeated 50 Hz sensor frame
        row(timestamp="1.03", com_sample_stamp="1.02",
            com_world_vz="2.0", com_world_z="0.44", velocity_reached="1"),
        row(state_name="FLIGHT", timestamp="1.05",
            com_sample_stamp="1.04", com_world_z="0.55"),
        row(state_name="FLIGHT", timestamp="1.20",
            com_sample_stamp="1.20", com_world_z="0.64"),
    ], {"takeoff_com_z": "0.44", "apex_com_z_delta": "0.20"}


class LatchHeightAnalysisTest(unittest.TestCase):
    def test_preserves_legacy_and_uses_distinct_source_samples(self):
        rows, summary = valid_case()
        result = MODULE.compute(rows, summary)
        self.assertAlmostEqual(result["sample_gap_ms"], 20.0)
        self.assertAlmostEqual(result["interpolated_z_m"], 0.42)
        self.assertAlmostEqual(result["legacy_rise_m"], 0.20)
        self.assertAlmostEqual(result["interpolated_rise_m"], 0.22)
        self.assertAlmostEqual(result["flight_entry_rise_m"], 0.09)

    def test_missing_previous_distinct_sample_is_unavailable(self):
        rows, summary = valid_case()
        with self.assertRaisesRegex(ValueError, "no independent COM sample"):
            MODULE.compute(rows[2:], summary)

    def test_non_bracketing_velocity_is_unavailable(self):
        rows, summary = valid_case()
        rows[0]["com_world_vz"] = rows[1]["com_world_vz"] = "1.95"
        with self.assertRaisesRegex(ValueError, "do not bracket"):
            MODULE.compute(rows, summary)

    def test_missing_or_inconsistent_summary_is_unavailable(self):
        rows, summary = valid_case()
        with self.assertRaisesRegex(ValueError, "missing controller log or summary"):
            MODULE.compute(rows, {})
        summary["takeoff_com_z"] = "0.47"
        with self.assertRaisesRegex(ValueError, "disagrees"):
            MODULE.compute(rows, summary)

    def test_large_sensor_gap_is_unavailable(self):
        rows, summary = valid_case()
        rows[0]["com_sample_stamp"] = rows[1]["com_sample_stamp"] = "0.98"
        with self.assertRaisesRegex(ValueError, "unexpected COM sample gap"):
            MODULE.compute(rows, summary)


if __name__ == "__main__":
    unittest.main()
