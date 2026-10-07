#!/usr/bin/env python3
"""Opt-in flat-jump wheel velocity servo using bounded effort commands.

This is a simulated actuator model, not a measurement of physical motor torque.
It must run only while diff_drive_controller is inactive and
wheel_effort_controller is active.
"""

import time
import argparse
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


RADIUS = 0.07
SEPARATION = 0.364
MAX_WHEEL_RATE = 30.0
MAX_TORQUE = 10.0
MAX_AGE = 0.05
LEFT = "link_004_joint"
RIGHT = "link_007_joint"
ZERO_EFFORT_FRAME = "base_link__zero_wheel_effort"


def clamp(value, bound):
    return max(-bound, min(bound, value))


def wheel_targets(linear, angular):
    if not all(math.isfinite(value) for value in (linear, angular)):
        return (0.0, 0.0)
    return (clamp((linear - 0.5 * SEPARATION * angular) / RADIUS, MAX_WHEEL_RATE),
            clamp((linear + 0.5 * SEPARATION * angular) / RADIUS, MAX_WHEEL_RATE))


def effort_from_error(targets, measured, gain):
    if (not finite_pair(targets) or not finite_pair(measured) or
            not isinstance(gain, (int, float)) or not math.isfinite(gain) or
            gain not in (0.5, 1.0, 2.0)):
        return (0.0, 0.0), (0.0, 0.0)
    raw = tuple(gain * (target - actual) for target, actual in zip(targets, measured))
    return tuple(clamp(value, MAX_TORQUE) for value in raw), raw


def finite_pair(values):
    return values is not None and len(values) == 2 and all(
        isinstance(value, (int, float)) and math.isfinite(value) for value in values)


def fresh(now_ns, stamp_ns):
    return (isinstance(stamp_ns, int) and stamp_ns > 0 and
            isinstance(now_ns, int) and 0 <= now_ns - stamp_ns <= int(MAX_AGE * 1e9))


def commanded_effort(targets, measured, gain, cmd_fresh, joint_fresh, zero_effort):
    if (not cmd_fresh or not joint_fresh or zero_effort or
            not finite_pair(targets) or not finite_pair(measured) or
            not isinstance(gain, (int, float)) or not math.isfinite(gain) or
            gain not in (0.5, 1.0, 2.0)):
        return (0.0, 0.0), (0.0, 0.0)
    return effort_from_error(targets, measured, gain)


class WheelEffortVelocityServo(Node):
    def __init__(self, gain, log_path):
        super().__init__("flat_jump_wheel_effort_servo",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gain = gain
        self.command = None
        self.command_stamp_ns = None
        self.command_reason = "missing"
        self.zero_effort_requested = False
        self.velocity = None
        self.joint_stamp_ns = None
        self.joint_reason = "missing"
        self.last_ros_ns = None
        self.clock_rollback_latched = False
        self.post_rollback_cmd = False
        self.post_rollback_joint = False
        self.publish_seq = 0
        self.log = log_path.open("w", newline="")
        self.writer = csv.writer(self.log)
        self.writer.writerow(("publish_seq", "ros_publish_ns", "wall_publish_ns",
                              "cmd_source_ns", "joint_source_ns", "cmd_valid", "joint_valid",
                              "cmd_reason", "joint_reason", "valid_reason",
                              "linear_target", "angular_target", "left_target_rate", "right_target_rate",
                              "left_rate", "right_rate", "left_torque_cmd", "right_torque_cmd",
                              "left_saturated", "right_saturated", "zero_effort_requested",
                              "zero_effort_active"))
        self.publisher = self.create_publisher(Float64MultiArray,
                                               "/wheel_effort_controller/commands", 10)
        self.command_sub = self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self.on_command, 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.timer = self.create_timer(0.005, self.update)

    def on_command(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        command = (msg.twist.linear.x, msg.twist.angular.z)
        if not finite_pair(command):
            self.command = None
            self.command_stamp_ns = stamp_ns
            self.command_reason = "nonfinite_command"
            return
        if self.command_stamp_ns is not None and stamp_ns < self.command_stamp_ns:
            self.command = None
            self.command_stamp_ns = stamp_ns
            self.command_reason = "command_time_rollback"
            return
        self.command = command
        self.zero_effort_requested = msg.header.frame_id == ZERO_EFFORT_FRAME
        self.command_stamp_ns = stamp_ns
        self.command_reason = "valid"
        self.post_rollback_cmd = True

    def on_joints(self, msg):
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if LEFT not in msg.name or RIGHT not in msg.name:
            self.velocity = None
            self.joint_stamp_ns = stamp_ns
            self.joint_reason = "missing_joint_name"
            return
        left = msg.name.index(LEFT)
        right = msg.name.index(RIGHT)
        if max(left, right) >= len(msg.velocity):
            self.velocity = None
            self.joint_stamp_ns = stamp_ns
            self.joint_reason = "missing_velocity_dimension"
            return
        velocity = (msg.velocity[left], msg.velocity[right])
        if not finite_pair(velocity):
            self.velocity = None
            self.joint_stamp_ns = stamp_ns
            self.joint_reason = "nonfinite_joint_velocity"
            return
        if self.joint_stamp_ns is not None and stamp_ns < self.joint_stamp_ns:
            self.velocity = None
            self.joint_stamp_ns = stamp_ns
            self.joint_reason = "joint_time_rollback"
            return
        self.velocity = velocity
        self.joint_stamp_ns = stamp_ns
        self.joint_reason = "valid"
        self.post_rollback_joint = True

    def update(self):
        sample_ns = int(self.get_clock().now().nanoseconds)
        rollback = self.last_ros_ns is not None and sample_ns < self.last_ros_ns
        self.last_ros_ns = sample_ns
        if rollback:
            self.clock_rollback_latched = True
            self.post_rollback_cmd = False
            self.post_rollback_joint = False
            self.command = None
            self.velocity = None
            self.command_stamp_ns = None
            self.joint_stamp_ns = None
            self.command_reason = "clock_rollback"
            self.joint_reason = "clock_rollback"
        if (self.clock_rollback_latched and self.post_rollback_cmd and self.post_rollback_joint and
                finite_pair(self.command) and finite_pair(self.velocity) and
                fresh(sample_ns, self.command_stamp_ns) and fresh(sample_ns, self.joint_stamp_ns)):
            self.clock_rollback_latched = False
        clock_invalid = rollback or self.clock_rollback_latched
        cmd_ok = (not clock_invalid and self.command_reason == "valid" and
                  finite_pair(self.command) and fresh(sample_ns, self.command_stamp_ns))
        joint_ok = (not clock_invalid and self.joint_reason == "valid" and
                    finite_pair(self.velocity) and fresh(sample_ns, self.joint_stamp_ns))
        cmd_reason = "clock_rollback" if clock_invalid else self.command_reason
        joint_reason = "clock_rollback" if clock_invalid else self.joint_reason
        if not clock_invalid and self.command_reason == "valid" and not fresh(sample_ns, self.command_stamp_ns):
            cmd_reason = "command_stale_or_future"
        if not clock_invalid and self.joint_reason == "valid" and not fresh(sample_ns, self.joint_stamp_ns):
            joint_reason = "joint_stale_or_future"
        targets = wheel_targets(*self.command) if cmd_ok else (0.0, 0.0)
        measured = self.velocity if joint_ok else None
        zero_effort_active = cmd_ok and self.zero_effort_requested
        torques, raw = commanded_effort(targets, measured, self.gain,
                                       cmd_ok, joint_ok, zero_effort_active)
        if not cmd_ok:
            raw = (0.0, 0.0)
        valid_reason = "valid" if cmd_ok and joint_ok else ";".join(
            item for item in (cmd_reason if not cmd_ok else "",
                              joint_reason if not joint_ok else "") if item)
        msg = Float64MultiArray()
        msg.data = list(torques)
        ros_publish_ns = int(self.get_clock().now().nanoseconds)
        wall_publish_ns = time.monotonic_ns()
        self.publisher.publish(msg)
        self.publish_seq += 1
        self.writer.writerow((self.publish_seq, ros_publish_ns, wall_publish_ns,
                              self.command_stamp_ns if self.command_stamp_ns is not None else "",
                              self.joint_stamp_ns if self.joint_stamp_ns is not None else "",
                              int(cmd_ok), int(joint_ok),
                              cmd_reason, joint_reason, valid_reason,
                              self.command[0] if cmd_ok else "", self.command[1] if cmd_ok else "",
                              *targets, *(measured if joint_ok else ("", "")), *torques,
                              int(abs(raw[0]) > MAX_TORQUE), int(abs(raw[1]) > MAX_TORQUE),
                              int(self.zero_effort_requested), int(zero_effort_active)))
        self.log.flush()

    def destroy_node(self):
        if rclpy.ok():
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            ros_publish_ns = int(self.get_clock().now().nanoseconds)
            wall_publish_ns = time.monotonic_ns()
            self.publisher.publish(msg)
            self.publish_seq += 1
            self.writer.writerow((self.publish_seq, ros_publish_ns, wall_publish_ns,
                                  self.command_stamp_ns if self.command_stamp_ns is not None else "",
                                  self.joint_stamp_ns if self.joint_stamp_ns is not None else "",
                                  0, 0, "shutdown_zero_effort", "shutdown_zero_effort",
                                  "shutdown_zero_effort", "", "", 0.0, 0.0, "", "",
                                  0.0, 0.0, 0, 0, int(self.zero_effort_requested), 1))
            self.log.flush()
        self.log.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--gain", type=float, choices=(0.5, 1.0, 2.0), default=1.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = WheelEffortVelocityServo(args.gain, args.output)
    try:
        # The world is still paused.  Wait for DDS discovery before allowing
        # the trial runner to unpause, otherwise the first command can arrive
        # after the robot has already fallen.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (node.count_publishers("/diff_drive_controller/cmd_vel") > 0 and
                    node.count_publishers("/joint_states") > 0 and
                    node.publisher.get_subscription_count() > 0):
                args.output.with_suffix(".ready").touch()
                break
        else:
            raise RuntimeError("wheel effort servo DDS endpoints not ready")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
