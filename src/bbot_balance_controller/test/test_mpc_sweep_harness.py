#!/usr/bin/env python3
"""Regression tests for the MPC sweep harness.

Covers the two defects that were left unfixed while the R sweep was still
collecting data, so that a future batch cannot silently repeat them:

  1. an empty post-pulse response window crashed run_mpc_balance_trials.py on
     np.max() of an empty array (and the exception killed the whole cell);
  2. run_mpc_r_sweep.py counted completed repetitions on run["trial"], which is
     the display label "<name>_rep<k>" whenever repeats > 1, so no trial ever
     looked finished and every R value re-launched all four trials three times.
"""

import sys
import unittest
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import run_mpc_balance_trials as trials  # noqa: E402
import run_mpc_r_sweep as sweep  # noqa: E402

ALL_TRIALS = ("static", "position", "speed", "push")


def sample(time, theta="0.0", x_error="0.0", u="0.0", sat="0"):
    return {"time": time, "theta_error": theta, "x_error": x_error, "u_mpc": u,
            "total_torque_saturated": sat}


class PulseResponseWindowTest(unittest.TestCase):
    def test_window_past_end_of_log_is_unavailable_not_zero(self):
        rows = [sample("1.0"), sample("2.0")]
        metrics = trials.pulse_response_metrics(rows, 50.0)
        self.assertFalse(metrics["available"])
        self.assertEqual(metrics["samples"], 0)
        self.assertIn("empty response window", metrics["reason"])
        for key in ("pitch_peak_deg", "x_error_peak_m", "torque_peak_Nm",
                    "saturated_fraction"):
            self.assertIsNone(metrics[key], f"{key} must be unavailable, never 0")

    def test_single_sample_window_is_unavailable(self):
        metrics = trials.pulse_response_metrics([sample("10.0", "0.1")], 10.0)
        self.assertFalse(metrics["available"])
        self.assertEqual(metrics["samples"], 1)
        self.assertIsNone(metrics["pitch_peak_deg"])

    def test_non_empty_window_measures_peaks(self):
        rows = [sample("10.0", "0.05", "0.10", "2.0"),
                sample("10.5", "-0.10", "-0.30", "-18.0", "1"),
                sample("11.0", "0.02", "0.05", "0.5")]
        metrics = trials.pulse_response_metrics(rows, 10.0)
        self.assertTrue(metrics["available"])
        self.assertEqual(metrics["samples"], 3)
        self.assertAlmostEqual(metrics["pitch_peak_deg"],
                               round(float(np.degrees(0.10)), 3), places=3)
        self.assertAlmostEqual(metrics["x_error_peak_m"], 0.30, places=4)
        self.assertAlmostEqual(metrics["torque_peak_Nm"], 18.0, places=2)
        self.assertAlmostEqual(metrics["saturated_fraction"], 1.0 / 3.0, places=3)

    def test_window_is_bounded_by_response_length(self):
        rows = [sample("10.0", "0.3"), sample("20.0", "0.9")]
        metrics = trials.pulse_response_metrics(rows, 10.0)
        self.assertEqual(metrics["samples"], 1, "row at t=20 s is outside the window")
        self.assertFalse(metrics["available"])


def result(trial, rep, takeover=0.02, fell=False, fail=None):
    """A summary.json entry in the schema run_mpc_balance_trials.py actually writes."""
    entry = {"trial_name": trial, "trial": f"{trial}_rep{rep}", "repetition": rep,
             "notes": {}, "fell": fell}
    if takeover is not None:
        entry["notes"]["first_controlled_time_s"] = takeover
    if fail is not None:
        entry["fail"] = fail
    return entry


class TrialAccountingTest(unittest.TestCase):
    def test_every_repetition_is_counted_under_its_base_trial(self):
        runs = [result(trial, rep) for rep in (1, 2, 3) for trial in ALL_TRIALS]
        controller_results, invalid = sweep.classify_runs(runs)
        self.assertEqual(len(controller_results), 12)
        self.assertEqual(invalid, [])
        counts = sweep.slot_counts(controller_results, ALL_TRIALS)
        self.assertEqual(counts, {trial: 3 for trial in ALL_TRIALS})
        self.assertEqual(sweep.missing_slots(counts, ALL_TRIALS, 3), {})

    def test_label_is_not_the_trial_name(self):
        # Pins the exact confusion behind defect 2.
        runs = [result("static", rep) for rep in (1, 2, 3)]
        self.assertNotIn("static", {run["trial"] for run in runs})
        self.assertEqual(sweep.slot_counts(runs, ALL_TRIALS)["static"], 3)

    def test_engaged_but_never_stabilised_counts_as_a_controller_result(self):
        # Reaching the stability gate is what is being measured, so missing it is a
        # result for R and must not be retried away.
        entry = result("position", 1, fell=False, fail="balance gate never satisfied")
        controller_results, invalid = sweep.classify_runs([entry])
        self.assertEqual(len(controller_results), 1)
        self.assertEqual(invalid, [])
        self.assertEqual(sweep.missing_slots(
            sweep.slot_counts(controller_results, ALL_TRIALS), ALL_TRIALS, 3)["position"], 2)

    def test_a_fall_counts_as_a_controller_result(self):
        controller_results, invalid = sweep.classify_runs([result("speed", 1, fell=True)])
        self.assertEqual(len(controller_results), 1)
        self.assertEqual(invalid, [])

    def test_late_or_absent_takeover_is_a_protocol_failure(self):
        late = result("push", 1, takeover=9.99)
        absent = result("push", 2, takeover=None, fail="no MPC rows after unpause")
        crashed = result("push", 3, takeover=0.02, fail="harness exception: ValueError: x")
        crashed["harness_exception"] = True
        controller_results, invalid = sweep.classify_runs([late, absent, crashed])
        self.assertEqual(controller_results, [])
        self.assertEqual(len(invalid), 3)

    def test_deficit_is_exact_per_trial(self):
        runs = [result("static", 1), result("static", 2), result("speed", 1)]
        counts = sweep.slot_counts(runs, ALL_TRIALS)
        self.assertEqual(sweep.missing_slots(counts, ALL_TRIALS, 3),
                         {"static": 1, "position": 3, "speed": 2, "push": 3})

    def test_trial_name_is_required_and_schema_break_is_contained(self):
        with self.assertRaises(KeyError):
            sweep.trial_name_of({"trial": "static_rep1", "repetition": 1})
        broken = [{"trial": "static_rep1", "notes": {"first_controlled_time_s": 0.01}}]
        controller_results, invalid, note = sweep.partition_runs(broken)
        self.assertEqual(controller_results, [], "a schema break must not yield results")
        self.assertEqual(len(invalid), 1)
        self.assertIn("summary schema error", note)

    def test_valid_takeover_constant_matches_the_trial_runner(self):
        # The driver and the aggregator both gate on takeover latency; if they ever
        # disagree the cohort stops meaning what the ledger says it means.
        import aggregate_mpc_r_sweep as aggregate
        self.assertEqual(sweep.VALID_TAKEOVER_S, aggregate.VALID_TAKEOVER_S)


if __name__ == "__main__":
    unittest.main(verbosity=2)
