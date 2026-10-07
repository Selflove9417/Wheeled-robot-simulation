#!/usr/bin/env python3
"""Targeted tests for ground-motion audit event and stop semantics."""
import importlib.util
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("audit_ground_motion_trial.py")
SPEC = importlib.util.spec_from_file_location("ground_motion_audit_test", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class GroundMotionTrialAuditTests(unittest.TestCase):
    def setUp(self):
        self.protocol = AUDIT.parse_protocol(
            Path(__file__).resolve().parents[2] /
            "src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_motion_probe_20261006_134820/protocol.json")

    @staticmethod
    def event(trial, replay, phase, name, ns, event_id, command=""):
        return {
            "trial_id": str(trial), "replay_id": str(replay), "phase": phase,
            "event": name, "event_id": event_id, "sim_event_ns": str(ns),
            "wall_event_ns": str(ns + 100), "command_id": command,
            "command_kind": "leg" if command else "", "reason": "fixture",
            "target_hl": "0.01", "target_kl": "-0.02", "target_hr": "0.01",
            "target_kr": "-0.02", "target_wheel_linear": "0", "target_wheel_angular": "0",
        }

    def actual_header_event_fixture(self):
        rows = [
            self.event(0, 0, "support_blend", "law_blend_start", 1_000_000, "blend-1", "cmd-1"),
            self.event(0, 0, "support_blend", "law_blend_complete", 2_000_000, "blend-2", "cmd-2"),
        ]
        # The preparation events may share the same step as the initial trial
        # boundary. trial_start == acceleration_start is explicitly legal.
        for t in self.protocol["trials"]:
            tid, rid = t["trial_id"], t["replay_id"]
            ns = 10_000_000 + tid * 10_000_000
            rows.extend([
                self.event(tid, rid, "acceleration", "trial_start", ns, f"{tid}-start"),
                self.event(tid, rid, "acceleration", "acceleration_start", ns, f"{tid}-accel", f"{tid}-leg-a"),
                self.event(tid, rid, "cruise", "cruise_start", ns + 500_000_000, f"{tid}-cruise", f"{tid}-leg-c"),
                self.event(tid, rid, "normal_stop", "normal_stop_trigger", ns + 1_000_000_000, f"{tid}-stop", f"{tid}-leg-s"),
                self.event(tid, rid, "normal_stop", "stop_complete", ns + 1_800_000_000, f"{tid}-done"),
            ])
        # This reason is a descriptive successful completion, not a fault.
        rows.append(self.event(4, 2, "complete", "campaign_complete", 99_000_000, "campaign", ""))
        return rows

    def test_actual_phase_events_header_and_prep_events_are_accepted(self):
        rows = self.actual_header_event_fixture()
        self.assertTrue(AUDIT.PHASE_EVENT_REQUIRED.issubset(rows[0]))
        groups, problems, global_events = AUDIT.events_by_trial(rows)
        self.assertEqual({}, dict(problems))
        self.assertEqual(4, len(groups))
        self.assertEqual(["law_blend_start"], list(global_events)[0:1])
        starts, event_problems, _ = AUDIT.check_events(self.protocol, groups, [])
        # Command references are checked separately; the trial_start at the
        # same timestamp as acceleration_start must not create an order error.
        self.assertNotIn("1:event_order_or_time_invalid", event_problems)
        self.assertEqual(4, len(starts))

    def test_campaign_reason_does_not_turn_success_into_failure(self):
        row = self.event(4, 2, "complete", "campaign_complete", 1, "end")
        row["reason"] = "four_probe_trials_complete"
        _, problems, _ = AUDIT.events_by_trial([row])
        self.assertNotIn("campaign_complete_reports_failure", problems)

    def test_fault_event_remains_failure(self):
        _, problems, _ = AUDIT.events_by_trial([
            self.event(2, 1, "normal_stop", "fault", 1, "fault-1")])
        self.assertGreater(problems["explicit_fault_event"], 0)

    @staticmethod
    def stop_frame(i, speed=0.0, valid=1, contacts=2):
        v = [0.0] * 9
        for idx in (3, 4, 6, 7):
            v[idx] = speed
        return {"key": (100 + i, 10_000_000 + i * AUDIT.STEP_NS),
                "frame_valid": valid, "contact_count": contacts,
                "after_joint_v": [v[3], v[4], v[5], v[6], v[7], v[8]]}

    def test_stop_complete_checks_last_250_raw_steps_and_allows_late_confirmation(self):
        frames = [self.stop_frame(i, speed=0.02 if i < 5 else 0.004) for i in range(300)]
        first_start, first_end = AUDIT.stop_dwell(frames, .005, 250)
        self.assertEqual((5, 254), (first_start, first_end))
        # Controller can confirm after additional low-speed dwell samples; its
        # confirmation need not be within one millisecond of earliest dwell.
        ok, begin, end = AUDIT.dwell_confirmed_at(
            frames, frames[299]["key"][1], .005, 250)
        self.assertTrue(ok)
        self.assertEqual((50, 299), (begin, end))

    def test_invalid_or_missing_support_breaks_stop_dwell(self):
        frames = [self.stop_frame(i) for i in range(260)]
        frames[130]["frame_valid"] = 0
        self.assertEqual((None, None), AUDIT.stop_dwell(frames, .005, 250))
        self.assertFalse(AUDIT.dwell_confirmed_at(
            frames, frames[-1]["key"][1], .005, 250)[0])

    def test_missing_or_nan_joint_input_is_invalid_not_zero(self):
        frame = {i: {"before_physics_joint_force_cmd_sim_input": "1.0",
                     "before_physics_joint_force_cmd_component_present": "1",
                     "before_physics_joint_force_cmd_valid": "1"} for i in range(6)}
        frame[2]["before_physics_joint_force_cmd_sim_input"] = "nan"
        self.assertEqual([None] * 6, AUDIT.input_vector(frame))
        del frame[2]
        self.assertEqual([None] * 6, AUDIT.input_vector(frame))

    def test_distinct_joint_sentinels_keep_native_and_model_orders_separate(self):
        # Native q9/v9 follows joint indices HL,KL,WL,HR,KR,WR. The response
        # input vector follows HL,KL,HR,KR,WL,WR.
        native_q = [10.0, 20.0, 30.0] + [101.0, 102.0, 103.0, 104.0, 105.0, 106.0]
        mapped = [native_q[AUDIT.native_state_dof(i)] for i in range(6)]
        self.assertEqual([101.0, 102.0, 103.0, 104.0, 105.0, 106.0], mapped)
        input_state = [native_q[i] for i in AUDIT.model_input_state_dofs()]
        self.assertEqual([101.0, 102.0, 104.0, 105.0, 103.0, 106.0], input_state)
        native_inputs = {i: {"before_physics_joint_force_cmd_sim_input": str(201.0 + i),
                             "before_physics_joint_force_cmd_component_present": "1",
                             "before_physics_joint_force_cmd_valid": "1"}
                         for i in range(6)}
        self.assertEqual([201.0, 202.0, 204.0, 205.0, 203.0, 206.0],
                         AUDIT.input_vector(native_inputs))

    def test_repeated_wheel_targets_remain_ambiguous(self):
        command = {"publish_ns": "10000000", "publish_end_ns": "11000000",
                   "wheel_linear": "0.1", "wheel_angular": "0.2"}
        servo = [{"cmd_source_ns": "10500000", "ros_publish_ns": "10600000",
                  "linear_target": "0.1", "angular_target": "0.2"},
                 {"cmd_source_ns": "10800000", "ros_publish_ns": "10900000",
                  "linear_target": "0.1", "angular_target": "0.2"}]
        matches = AUDIT.servo_rows_for_command(servo, command, 12000000)
        self.assertEqual(2, len(matches))


if __name__ == "__main__":
    unittest.main()
