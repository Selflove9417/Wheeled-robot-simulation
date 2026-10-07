#!/usr/bin/env python3
"""Unit checks for the opt-in simulated wheel effort servo."""

import importlib.util
import math
from pathlib import Path
import unittest
from types import SimpleNamespace
from geometry_msgs.msg import TwistStamped


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "wheel_effort_velocity_servo.py"
spec = importlib.util.spec_from_file_location("wheel_effort_velocity_servo", SCRIPT)
servo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(servo)


class WheelEffortServoTest(unittest.TestCase):
    def test_speed_and_yaw_mapping(self):
        straight = servo.wheel_targets(0.35, 0.0)
        self.assertAlmostEqual(straight[0], 5.0)
        self.assertAlmostEqual(straight[1], 5.0)
        left, right = servo.wheel_targets(0.35, 1.0)
        self.assertLess(left, 5.0)
        self.assertGreater(right, 5.0)
        self.assertEqual(servo.wheel_targets(10.0, 0.0), (30.0, 30.0))

    def test_effort_limit_and_sign(self):
        torques, raw = servo.effort_from_error((20.0, -20.0), (0.0, 0.0), 1.0)
        self.assertEqual(raw, (20.0, -20.0))
        self.assertEqual(torques, (10.0, -10.0))
        self.assertEqual(servo.effort_from_error((2.0, 2.0), (3.0, 1.0), 1.0)[0],
                         (-1.0, 1.0))

    def test_sensor_and_command_watchdog(self):
        self.assertTrue(servo.fresh(1_000_000_000, 960_000_000))
        self.assertFalse(servo.fresh(1_000_000_000, 940_000_000))
        self.assertFalse(servo.fresh(1.0, None))
        self.assertFalse(servo.fresh(1_000_000_000, 1_010_000_000))

    def test_nan_and_infinity_fail_closed(self):
        self.assertEqual(servo.wheel_targets(float("nan"), 0.0), (0.0, 0.0))
        self.assertEqual(servo.wheel_targets(0.0, float("inf")), (0.0, 0.0))
        for bad in (float("nan"), float("inf"), -float("inf")):
            self.assertEqual(servo.effort_from_error((5.0, 5.0), (bad, 0.0), 1.0),
                             ((0.0, 0.0), (0.0, 0.0)))
            self.assertEqual(servo.commanded_effort((5.0, 5.0), (bad, 0.0), 1.0,
                                                    True, True, False)[0], (0.0, 0.0))

    def test_invalid_measurement_is_not_replaced_with_zero(self):
        self.assertFalse(servo.finite_pair(None))
        self.assertFalse(servo.finite_pair((2.0,)))
        self.assertFalse(servo.finite_pair((2.0, float("nan"))))
        self.assertEqual(servo.commanded_effort((5.0, 5.0), None, 1.0,
                                                True, True, False)[0], (0.0, 0.0))

    def test_stamped_zero_effort_gate(self):
        holder = SimpleNamespace(command=None, command_stamp_ns=None, command_reason="missing",
                                 zero_effort_requested=False, post_rollback_cmd=False)
        msg = TwistStamped()
        msg.header.stamp.sec = 3
        msg.header.stamp.nanosec = 47000000
        msg.header.frame_id = servo.ZERO_EFFORT_FRAME
        servo.WheelEffortVelocityServo.on_command(holder, msg)
        self.assertTrue(holder.zero_effort_requested)
        self.assertEqual(holder.command_stamp_ns, 3_047_000_000)
        self.assertEqual(servo.commanded_effort((20.0, -20.0), (0.0, 0.0), 1.0,
                                                True, True, holder.zero_effort_requested)[0],
                         (0.0, 0.0))
        msg.header.frame_id = "base_link"
        servo.WheelEffortVelocityServo.on_command(holder, msg)
        self.assertFalse(holder.zero_effort_requested)
        self.assertEqual(servo.commanded_effort((20.0, -20.0), (0.0, 0.0), 1.0,
                                                True, True, holder.zero_effort_requested)[0],
                         (10.0, -10.0))

    def test_unsupported_gain_fails_closed(self):
        self.assertEqual(servo.effort_from_error((2.0, 2.0), (0.0, 0.0), 1.5),
                         ((0.0, 0.0), (0.0, 0.0)))

    def test_callbacks_preserve_invalidity_and_exact_source_ns(self):
        holder = SimpleNamespace(velocity=(1.0, 2.0), joint_stamp_ns=10,
                                 joint_reason="valid", command=(1.0, 0.0),
                                 command_stamp_ns=10, command_reason="valid",
                                 zero_effort_requested=False, post_rollback_joint=False,
                                 post_rollback_cmd=False)
        joint = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=123)),
            name=[servo.LEFT, servo.RIGHT], velocity=[float("nan"), 2.0])
        servo.WheelEffortVelocityServo.on_joints(holder, joint)
        self.assertIsNone(holder.velocity)
        self.assertEqual(holder.joint_stamp_ns, 2_000_000_123)
        self.assertEqual(holder.joint_reason, "nonfinite_joint_velocity")
        joint.velocity = [1.0]
        servo.WheelEffortVelocityServo.on_joints(holder, joint)
        self.assertIsNone(holder.velocity)
        self.assertEqual(holder.joint_reason, "missing_velocity_dimension")

        command = TwistStamped()
        command.header.stamp.sec = 3
        command.twist.linear.x = float("inf")
        servo.WheelEffortVelocityServo.on_command(holder, command)
        self.assertIsNone(holder.command)
        self.assertEqual(holder.command_stamp_ns, 3_000_000_000)
        self.assertEqual(holder.command_reason, "nonfinite_command")


if __name__ == "__main__":
    unittest.main()
