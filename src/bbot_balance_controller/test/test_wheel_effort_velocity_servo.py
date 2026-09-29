#!/usr/bin/env python3
"""Unit checks for the opt-in simulated wheel effort servo."""

import importlib.util
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
        self.assertTrue(servo.fresh(1.0, 0.96))
        self.assertFalse(servo.fresh(1.0, 0.94))
        self.assertFalse(servo.fresh(1.0, None))
        self.assertFalse(servo.fresh(1.0, 1.01))

    def test_stamped_zero_effort_gate(self):
        holder = SimpleNamespace()
        msg = TwistStamped()
        msg.header.stamp.sec = 3
        msg.header.stamp.nanosec = 47000000
        msg.header.frame_id = servo.ZERO_EFFORT_FRAME
        servo.WheelEffortVelocityServo.on_command(holder, msg)
        self.assertTrue(holder.zero_effort_requested)
        self.assertAlmostEqual(holder.command_stamp, 3.047)
        self.assertEqual(servo.commanded_effort((20.0, -20.0), (0.0, 0.0), 1.0,
                                                True, True, holder.zero_effort_requested)[0],
                         (0.0, 0.0))
        msg.header.frame_id = "base_link"
        servo.WheelEffortVelocityServo.on_command(holder, msg)
        self.assertFalse(holder.zero_effort_requested)
        self.assertEqual(servo.commanded_effort((20.0, -20.0), (0.0, 0.0), 1.0,
                                                True, True, holder.zero_effort_requested)[0],
                         (10.0, -10.0))


if __name__ == "__main__":
    unittest.main()
