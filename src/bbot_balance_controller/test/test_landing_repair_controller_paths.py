#!/usr/bin/env python3
"""Static checks that safety gates are in the real controller call paths."""

from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "src" / "bbot_landing_repair_controller.cpp"
CODE = SOURCE.read_text()


def body_after(signature):
    start = CODE.index(signature)
    opening = CODE.index("{", start)
    depth = 0
    for pos in range(opening, len(CODE)):
        if CODE[pos] == "{":
            depth += 1
        elif CODE[pos] == "}":
            depth -= 1
            if depth == 0:
                return CODE[opening + 1:pos]
    raise AssertionError(f"unterminated body: {signature}")


class LandingRepairControllerPathsTest(unittest.TestCase):
    def test_thrust_ownership_gate_precedes_legacy_returns(self):
        body = body_after("void run_state_thrust(double now_sec, double dt)")
        gate = body.index("thrust_output_action(")
        lock = body.index("LockAfterHandoff", gate)
        self.assertLess(gate, lock)
        self.assertLess(lock, body.index("if (!thrust_trajectory_initialized_)", lock))
        self.assertIn("lock_owned_outputs(", body[lock:])

    def test_ground_jump_rejection_and_hold_loss_reach_runtime_gate(self):
        trigger = body_after("void trigger_jump()")
        self.assertIn("ground_input_jump_allowed(ground_input_experiment_)", trigger)
        update = body_after("void update_ground_input_experiment(double dt, double now_sec)")
        hold = update.index('ground_input_stage_ == "effort_hold"')
        self.assertIn("ground_input_support_fresh(now_sec)", update[hold:])
        self.assertIn("ground_input_effort_hold_valid(", body_after("bool ground_input_support_fresh(double now_sec)"))

    def test_control_loop_dropouts_lock_owned_commands(self):
        body = body_after("void control_loop()")
        missing = body.index("if (!imu_received_ || !wheel_origin_set_)")
        self.assertIn("controller_has_protected_output_owner(", body[missing:body.index("++timer_calls_since_control_", missing)])
        self.assertIn("lock_owned_outputs(\"required_imu_or_wheel_origin_missing\")", body[missing:])
        self.assertIn("lock_owned_outputs(\"simulation_clock_rollback\")", body)

    def test_publication_record_does_not_delay_actual_api(self):
        body = body_after("void publish_recorded_command(")
        self.assertLess(body.index("const int64_t publish_ns"), body.index("publisher->publish(message)"))
        self.assertLess(body.index("publisher->publish(message)"), body.index("record_command_publication("))
        self.assertIn("publish_end_ns", body)
        self.assertIn("wall_publish_end_ns", body)

    def test_ground_experiment_has_absolute_clock_and_stage_log(self):
        self.assertIn("control_sim_time_ns,ground_input_stage,ground_input_reason", CODE)
        self.assertIn('ground_input_log_ << std::setprecision(17) << now.nanoseconds()', CODE)

    def test_motion_branch_uses_full_four_joint_implicit_feedback(self):
        body = body_after("void publish_effort_leg_control_lr(")
        self.assertIn("ground_motion_law_active() ?", body)
        self.assertIn("bbot_jump::discrete_flight_pd(", body)
        self.assertIn("bbot_jump::JointVector::Zero(), ground_pd_horizon_", body)
        self.assertIn("bbot_jump::discrete_ground_leg_feedback(", body)
        full_support = body.index("ground_motion_direct_full_support(direct_input)")
        self.assertLess(body.index("discrete_flight_pd("), full_support)

    def test_measured_stop_is_logged_before_excitation_failure(self):
        body = body_after("void update_ground_motion_experiment(double dt, double now_sec)")
        stopped = body.index('if (ground_motion_stop_quiet_time_ >= 0.25)')
        stop_event = body.index('queue_ground_motion_event("stop_complete"', stopped)
        qualification = body.index('fail_ground_motion("insufficient_joint_excitation")', stopped)
        self.assertLess(stop_event, qualification)
        self.assertIn('ground_motion_stage_ = "stopping"', body)
        self.assertIn('ground_motion_phase_ = "normal_stop"', body)

    def test_motion_fault_support_does_not_reenter_feedback_law(self):
        body = body_after("void lock_owned_outputs(const std::string &reason)")
        motion_fail = body.index("fail_ground_motion(reason)")
        wheel_stop = body.index("publish_wheel_cmd(0.0, 0.0, true)")
        self.assertLess(motion_fail, wheel_stop)
        start = body.index('ground_input_stage_ == "failed"')
        end = body.index("RCLCPP_ERROR_THROTTLE", start)
        failure = body[start:end]
        self.assertIn("publish_recorded_command(leg_effort_pub_", failure)
        self.assertNotIn("publish_effort_leg_control_lr(", failure)
        self.assertIn("clamp_value(last[i]", failure)

    def test_mode_pending_fault_trace_has_no_leg_publication_claim(self):
        lock = body_after("void lock_owned_outputs(const std::string &reason)")
        pending = lock.index("if (!effort_mode_active_ || leg_mode_switch_pending_)")
        pending_end = lock.index("if (ground_input_experiment_ && ground_input_stage_ == \"failed\")", pending)
        self.assertIn("write_ground_motion_without_leg_publish()", lock[pending:pending_end])
        writer = body_after("void write_ground_motion_record(bool leg_publish_occurred)")
        self.assertIn('phase_events_log_ << "nan,none,"', writer)
        self.assertIn('motion_trace_log_ << "nan," << capture_control_ns_ << ",nan,nan,nan,nan"', writer)

    def test_early_sensor_fault_captures_clock_before_return(self):
        body = body_after("void control_loop()")
        capture = body.index("capture_control_ns_ = this->now().nanoseconds()")
        missing = body.index("if (!imu_received_ || !wheel_origin_set_)")
        self.assertLess(capture, missing)


if __name__ == "__main__":
    unittest.main()
