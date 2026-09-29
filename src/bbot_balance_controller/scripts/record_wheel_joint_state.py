#!/usr/bin/env python3
"""Record wheel joint states independently of the jump controller."""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEELS = ("link_004_joint", "link_007_joint")


class WheelJointRecorder(Node):
    def __init__(self, output_path: Path):
        super().__init__("wheel_joint_recorder")
        self.output = output_path.open("w", newline="")
        self.writer = csv.writer(self.output)
        self.writer.writerow(("stamp", "receive_wall_time", "side", "velocity", "effort",
                              "velocity_valid", "effort_valid"))
        self.subscription = self.create_subscription(JointState, "/joint_states", self.record, 50)

    def record(self, msg: JointState):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        receive_wall_time = time.time()
        for side, name in (("left", WHEELS[0]), ("right", WHEELS[1])):
            if name not in msg.name:
                continue
            index = msg.name.index(name)
            has_velocity = index < len(msg.velocity)
            has_effort = index < len(msg.effort)
            self.writer.writerow((f"{stamp:.9f}", f"{receive_wall_time:.9f}", side,
                                  msg.velocity[index] if has_velocity else "",
                                  msg.effort[index] if has_effort else "",
                                  int(has_velocity), int(has_effort)))
        self.output.flush()

    def destroy_node(self):
        self.output.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = WheelJointRecorder(args.output)
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
