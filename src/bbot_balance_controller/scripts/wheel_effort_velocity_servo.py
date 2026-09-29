#!/usr/bin/env python3
"""Opt-in flat-jump wheel velocity servo using bounded effort commands.

This is a simulated actuator model, not a measurement of physical motor torque.
It must run only while diff_drive_controller is inactive and
wheel_effort_controller is active.
"""

import time
import argparse
import csv
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
    return (clamp((linear - 0.5 * SEPARATION * angular) / RADIUS, MAX_WHEEL_RATE),
            clamp((linear + 0.5 * SEPARATION * angular) / RADIUS, MAX_WHEEL_RATE))


def effort_from_error(targets, measured, gain):
    raw = tuple(gain * (target - actual) for target, actual in zip(targets, measured))
    return tuple(clamp(value, MAX_TORQUE) for value in raw), raw


def fresh(now, stamp):
    return stamp is not None and stamp > 0.0 and 0.0 <= now - stamp <= MAX_AGE


def commanded_effort(targets, measured, gain, cmd_fresh, joint_fresh, zero_effort):
    if not cmd_fresh or not joint_fresh or zero_effort:
        return (0.0, 0.0), (0.0, 0.0)
    return effort_from_error(targets, measured, gain)


class WheelEffortVelocityServo(Node):
    def __init__(self, gain, log_path):
        super().__init__("flat_jump_wheel_effort_servo",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gain = gain
        self.command = None
        self.command_stamp = None
        self.zero_effort_requested = False
        self.velocity = None
        self.joint_stamp = None
        self.log = log_path.open("w", newline="")
        self.writer = csv.writer(self.log)
        self.writer.writerow(("sim_time", "cmd_stamp", "joint_stamp", "cmd_fresh", "joint_fresh",
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
        self.command = (msg.twist.linear.x, msg.twist.angular.z)
        self.zero_effort_requested = msg.header.frame_id == ZERO_EFFORT_FRAME
        self.command_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_joints(self, msg):
        if LEFT not in msg.name or RIGHT not in msg.name:
            return
        left = msg.name.index(LEFT)
        right = msg.name.index(RIGHT)
        if max(left, right) >= len(msg.velocity):
            return
        self.velocity = (msg.velocity[left], msg.velocity[right])
        self.joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd_ok = fresh(now, self.command_stamp)
        joint_ok = fresh(now, self.joint_stamp)
        targets = wheel_targets(*self.command) if cmd_ok else (0.0, 0.0)
        measured = self.velocity if joint_ok else (0.0, 0.0)
        zero_effort_active = cmd_ok and self.zero_effort_requested
        torques, raw = commanded_effort(targets, measured, self.gain,
                                       cmd_ok, joint_ok, zero_effort_active)
        msg = Float64MultiArray()
        msg.data = list(torques)
        self.publisher.publish(msg)
        self.writer.writerow((f"{now:.6f}", self.command_stamp, self.joint_stamp,
                              int(cmd_ok), int(joint_ok),
                              self.command[0] if cmd_ok else "", self.command[1] if cmd_ok else "",
                              *targets, *measured, *torques,
                              int(abs(raw[0]) > MAX_TORQUE), int(abs(raw[1]) > MAX_TORQUE),
                              int(self.zero_effort_requested), int(zero_effort_active)))
        self.log.flush()

    def destroy_node(self):
        if rclpy.ok():
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.publisher.publish(msg)
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
