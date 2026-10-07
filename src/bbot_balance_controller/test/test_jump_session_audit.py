#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys

SCRIPT = Path(__file__).parents[1] / "scripts" / "run_flat_ground_jump_trial.py"
SPEC = importlib.util.spec_from_file_location("flat_jump_runner", SCRIPT)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def summary(jump_id, *, exit_code="SUCCESS", height="0.20", vz="1.98", vx="0.45"):
    return {
        "jump_id": str(jump_id), "exit_code": exit_code,
        "apex_com_z_delta": height, "takeoff_com_vz": vz,
        "takeoff_com_vx": vx, "tuck_entered": "1", "extend_entered": "1",
        "protective_reason": "",
    }


class JumpSessionAuditTests(unittest.TestCase):
    def test_live_log_reader_seeks_tail_and_ignores_partial_record(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "live.csv"
            with path.open("w", encoding="utf-8") as stream:
                stream.write("timestamp,state_name\n")
                for index in range(120000):
                    stream.write(f"{index},BALANCE\n")
                stream.write("120000,PRE_JUMP\n120001,FLI")
            rows = RUNNER.read_latest_log_rows(str(path), 2)
            self.assertEqual(rows[-1], {"timestamp": "120000", "state_name": "PRE_JUMP"})
            self.assertEqual(rows[0], {"timestamp": "119999", "state_name": "BALANCE"})

    def test_later_success_does_not_mask_failed_middle_jump(self):
        rows = [summary(1), summary(2, exit_code="FAIL_PRE_JUMP", height="-1"), summary(3)]
        failures = RUNNER.audit_jump_summaries(rows, 3)
        self.assertTrue(any("jump_id 2" in item for item in failures))
        self.assertTrue(any("exit code" in item for item in failures))

    def test_missing_or_reused_jump_id_fails_session(self):
        rows = [summary(1), summary(1), summary(3)]
        failures = RUNNER.audit_jump_summaries(rows, 3)
        self.assertTrue(any("summary 2 has jump_id 1" in item for item in failures))

    def test_all_three_independent_rows_can_pass(self):
        self.assertEqual(RUNNER.audit_jump_summaries([summary(1), summary(2), summary(3)], 3), [])

    def test_summary_velocity_remains_diagnostic_not_physical_acceptance(self):
        rows = [summary(1, vx="0.70"), summary(2), summary(3)]
        self.assertEqual(RUNNER.audit_jump_summaries(rows, 3), [])

    def test_observational_baseline_pins_release_ratio_to_point_seven(self):
        source = SCRIPT.read_text()
        self.assertIn('(args.thrust_release_velocity_ratio, 0.70)', source)
        self.assertIn('(args.thrust_release_fast_rate_correction, False)', source)
        self.assertIn('(args.thrust_fast_rate_correction_from_gate, False)', source)
        self.assertIn('(args.thrust_forward_speed_prediction, False)', source)
        self.assertIn('"thrust_release_velocity_ratio:={args.thrust_release_velocity_ratio:.4f}"',
                      source)
        self.assertIn('"thrust_fast_rate_correction_from_gate:=" +', source)

    def test_gate_scope_runner_option_requires_and_forwards_existing_correction(self):
        help_result = subprocess.run(
            [sys.executable, str(SCRIPT), '--help'],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(help_result.returncode, 0)
        self.assertIn('--thrust-fast-rate-correction-from-gate', help_result.stdout)
        invalid = subprocess.run(
            [sys.executable, str(SCRIPT), '--thrust-fast-rate-correction-from-gate',
             '--no-thrust-release-fast-rate-correction'],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(invalid.returncode, 2)
        self.assertIn('requires its master correction', invalid.stderr)

    def test_reported_takeoff_vx_comes_from_fit_not_controller_latch(self):
        result = {}
        RUNNER.attach_independent_fit_metrics(
            result, {"takeoff_com_vx": "0.522973", "takeoff_com_vz": "1.95011",
                     "apex_com_z_delta": "0.207456"},
            [{"jump_id": "1", "takeoff_vx_estimate": 0.562936,
              "takeoff_vz_estimate": 1.948775, "apex_delta_estimate": 0.193728}])
        self.assertAlmostEqual(result["takeoff_vx"], 0.562936)
        self.assertAlmostEqual(result["controller_summary_takeoff_vx"], 0.522973)
        self.assertNotEqual(result["takeoff_vx"], result["controller_summary_takeoff_vx"])

    def test_trace_requires_one_flight_per_sequential_jump_id(self):
        rows = [
            {"jump_id": "1", "state_name": "FLIGHT"},
            {"jump_id": "1", "state_name": "TOUCHDOWN_BUFFER"},
            {"jump_id": "2", "state_name": "FLIGHT"},
        ]
        self.assertEqual(RUNNER.audit_flight_id_sequence(rows, 2), [])
        adjacent = [rows[0], {"jump_id": "1", "state_name": "FLIGHT"},
                    rows[1], rows[2]]
        self.assertEqual(RUNNER.audit_flight_id_sequence(adjacent, 2), [])
        duplicate = rows + [{"jump_id": "1", "state_name": "FLIGHT"}]
        self.assertTrue(any("jump_id 1 has 2 FLIGHT" in item
                            for item in RUNNER.audit_flight_id_sequence(duplicate, 2)))
        missing = [rows[0], rows[1]]
        self.assertTrue(any("not exactly ['1', '2']" in item
                            for item in RUNNER.audit_flight_id_sequence(missing, 2)))
        reused = rows + [{"jump_id": "1", "state_name": "PRE_JUMP"}]
        self.assertTrue(any("not exactly ['1', '2']" in item
                            for item in RUNNER.audit_flight_id_sequence(reused, 2)))

    def test_raw_subphase_trace_rejects_protective_even_when_summary_claims_success(self):
        # A legacy controller summary can have empty protective_reason and
        # T/E flags set despite a later protective subphase in the trace.
        self.assertEqual(RUNNER.audit_jump_summaries([summary(1)], 1), [])
        trace = [
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": "0"},
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": "1"},
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": "3"},
            {"jump_id": "1", "state_name": "TOUCHDOWN_BUFFER", "flight_subphase": "-1"},
        ]
        failures = RUNNER.audit_flight_subphase_trace(trace, 1)
        self.assertTrue(any("protective deploy" in item for item in failures))
        self.assertTrue(any("not exactly [0, 1, 2]" in item for item in failures))
        after_extend = [
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": str(phase)}
            for phase in (0, 1, 2, 3)
        ]
        self.assertTrue(any("protective deploy" in item for item in
                            RUNNER.audit_flight_subphase_trace(after_extend, 1)))

    def test_each_jump_subphase_is_audited_independently(self):
        trace = [
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": str(phase)}
            for phase in (0, 1, 3)
        ] + [
            {"jump_id": "2", "state_name": "FLIGHT", "flight_subphase": str(phase)}
            for phase in (0, 1, 2)
        ]
        failures = RUNNER.audit_flight_subphase_trace(trace, 2)
        self.assertTrue(any("jump_id 1" in item for item in failures))
        self.assertFalse(any("jump_id 2" in item for item in failures))

    def test_raw_subphase_trace_rejects_emergency_and_unknown_or_missing_phase(self):
        trace = [
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": "0"},
            {"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": "4"},
            {"jump_id": "1", "state_name": "EMERGENCY", "flight_subphase": "-1"},
        ]
        failures = RUNNER.audit_flight_subphase_trace(trace, 1)
        self.assertTrue(any("unknown subphase 4" in item for item in failures))
        self.assertTrue(any("entered EMERGENCY" in item for item in failures))
        missing = RUNNER.audit_flight_subphase_trace(
            [{"jump_id": "1", "state_name": "FLIGHT", "flight_subphase": ""}], 1)
        self.assertTrue(any("missing/invalid subphase" in item for item in missing))

    def test_complete_contact_takeoff_requires_source_at_each_jump_entry(self):
        def entry(jump_id, taken):
            return {
                "jump_id": str(jump_id), "state_name": "FLIGHT",
                "complete_contact_takeoff_enabled": "1",
                "contact_source_valid": "1", "contact_source_taken": str(int(taken)),
                "contact_takeoff_confirmed": str(int(taken)),
                "contact_zero_frames": "11", "contact_zero_span_ns": "10000000",
                "contact_source_stamp_ns": "123000000", "contact_takeoff_frame_stamp_ns": "123000000",
                "contact_takeoff_com_stamp_ns": "124000000", "contact_source_seq": "123",
            }
        self.assertEqual(
            RUNNER.audit_complete_contact_takeoff_trace(
                [entry(1, True), entry(2, True)], 2), [])
        failures = RUNNER.audit_complete_contact_takeoff_trace(
            [entry(1, True), entry(2, False)], 2)
        self.assertTrue(any("jump_id 2" in item and "not taken" in item for item in failures))

    def test_complete_contact_takeoff_rejects_short_or_malformed_witness(self):
        row = {
            "jump_id": "1", "state_name": "FLIGHT",
            "complete_contact_takeoff_enabled": "1", "contact_source_valid": "1",
            "contact_source_taken": "1", "contact_takeoff_confirmed": "1",
            "contact_zero_frames": "10", "contact_zero_span_ns": "9000000",
            "contact_source_stamp_ns": "10", "contact_takeoff_frame_stamp_ns": "11",
            "contact_takeoff_com_stamp_ns": "12", "contact_source_seq": "123",
        }
        failures = RUNNER.audit_complete_contact_takeoff_trace([row], 1)
        self.assertTrue(any("11 zero frames over 10 ms" in item for item in failures))
        row["contact_source_seq"] = "bad"
        failures = RUNNER.audit_complete_contact_takeoff_trace([row], 1)
        self.assertTrue(any("malformed" in item for item in failures))


if __name__ == "__main__":
    unittest.main()
