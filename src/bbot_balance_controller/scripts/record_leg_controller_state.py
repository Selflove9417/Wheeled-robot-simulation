#!/usr/bin/env python3
"""Independently observe leg controller state, commands, and joint feedback."""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


LEGS = ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")


def leg_positions(message):
    positions = dict(zip(message.name, message.position))
    return [positions.get(name, "") for name in LEGS]


def controller_state(response, name):
    controller = next((item for item in response.controller if item.name == name), None)
    if controller is None:
        return "NOT_LISTED", ""
    return controller.state, ";".join(controller.claimed_interfaces)


class LegControllerRecorder(Node):
    def __init__(self, output_path: Path):
        super().__init__("leg_controller_state_recorder")
        self.output = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.output)
        self.writer.writerow((
            "query_wall_s", "query_sim_s", "response_wall_s", "response_sim_s",
            "query_status", "position_state", "effort_state", "position_claimed_interfaces",
            "position_cmd_count", "last_cmd_sim_s", "cmd_hip_l", "cmd_knee_l",
            "cmd_hip_r", "cmd_knee_r", "joint_stamp_s", "joint_hip_l", "joint_knee_l",
            "joint_hip_r", "joint_knee_r"))
        self.output.flush()
        self.sim_time = -1.0
        self.command_count = 0
        self.command_stamp = -1.0
        self.command = ["", "", "", ""]
        self.joint_stamp = -1.0
        self.joints = ["", "", "", ""]
        self.pending = None
        self.pending_since = 0.0
        self.query_wall = -1.0
        self.query_sim = -1.0
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Float64MultiArray, "/leg_position_controller/commands",
                                 self.on_command, 50)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.create_timer(0.1, self.poll)

    def on_clock(self, message):
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def on_command(self, message):
        self.command_count += 1
        self.command_stamp = self.sim_time
        self.command = list(message.data[:4]) + [""] * max(0, 4 - len(message.data))

    def on_joints(self, message):
        self.joint_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.joints = leg_positions(message)

    def record(self, status, position="UNKNOWN", effort="UNKNOWN", claimed=""):
        self.writer.writerow((
            f"{self.query_wall:.9f}", f"{self.query_sim:.9f}", f"{time.time():.9f}",
            f"{self.sim_time:.9f}", status, position, effort, claimed,
            self.command_count, f"{self.command_stamp:.9f}", *self.command[:4],
            f"{self.joint_stamp:.9f}", *self.joints))
        self.output.flush()

    def poll(self):
        if self.pending is not None:
            if self.pending.done():
                try:
                    response = self.pending.result()
                    pos, claimed = controller_state(response, "leg_position_controller")
                    effort, _ = controller_state(response, "leg_effort_controller")
                    self.record("OK", pos, effort, claimed)
                except Exception:
                    self.record("UNKNOWN_RESPONSE")
                self.pending = None
            elif time.monotonic() - self.pending_since > 1.0:
                self.record("UNKNOWN_TIMEOUT")
                self.pending = None
            return
        self.query_wall = time.time()
        self.query_sim = self.sim_time
        if not self.client.service_is_ready():
            self.record("UNKNOWN_SERVICE")
            return
        self.pending = self.client.call_async(ListControllers.Request())
        self.pending_since = time.monotonic()

    def destroy_node(self):
        self.output.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = LegControllerRecorder(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
