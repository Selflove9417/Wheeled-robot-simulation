#!/usr/bin/env python3
"""Passively record published leg commands and reported four-joint effort.

No torque is computed or commanded. Missing effort stays empty with a false
validity flag. Reported JointState effort is not certified physical torque.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")


def item(values, index):
    if index < len(values) and math.isfinite(values[index]):
        return values[index], 1
    return "", 0


class JointTorqueRecorder(Node):
    def __init__(self, prefix, joint_topic, command_topic):
        super().__init__("joint_torque_recorder")
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.streams = [Path(str(prefix)+suffix).open("x", newline="", encoding="utf-8")
                        for suffix in ("_joint_feedback.csv", "_effort_commands.csv")]
        self.joints, self.commands = [csv.writer(s) for s in self.streams]
        self.joints.writerow(("sample_stamp_s", "receive_sim_s", "receive_wall_s", "joint",
                              "position_rad", "position_valid", "velocity_rad_s", "velocity_valid",
                              "reported_effort_nm", "effort_valid"))
        self.commands.writerow(("receive_sim_s", "receive_wall_s", "clock_received", "data_length",
                                "command_valid", "hip_left_command_nm", "knee_left_command_nm",
                                "hip_right_command_nm", "knee_right_command_nm"))
        self.sim_time = ""
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, joint_topic, self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, command_topic, self.on_command, 100)
        self.create_timer(.5, self.flush)
        self.flush()

    def on_clock(self, message):
        self.sim_time = message.clock.sec+message.clock.nanosec*1e-9

    def on_joints(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        wall = time.time()
        indices = {name: i for i,name in enumerate(message.name)}
        for joint in JOINTS:
            if joint in indices:
                i=indices[joint]
                p,pv=item(message.position,i);v,vv=item(message.velocity,i);e,ev=item(message.effort,i)
            else:
                p=v=e="";pv=vv=ev=0
            self.joints.writerow((f"{stamp:.9f}",self.sim_time,f"{wall:.9f}",joint,p,pv,v,vv,e,ev))

    def on_command(self, message):
        valid=len(message.data)==4 and all(math.isfinite(v) for v in message.data)
        values=[item(message.data,i)[0] for i in range(4)]
        self.commands.writerow((self.sim_time,f"{time.time():.9f}",int(self.sim_time!=""),
                                len(message.data),int(valid),*values))

    def flush(self):
        for stream in self.streams:stream.flush()

    def destroy_node(self):
        self.flush()
        for stream in self.streams:stream.close()
        super().destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_prefix",type=Path)
    parser.add_argument("--joint-topic",default="/joint_states")
    parser.add_argument("--command-topic",default="/leg_effort_controller/commands")
    args=parser.parse_args()
    for suffix in ("_joint_feedback.csv","_effort_commands.csv"):
        if Path(str(args.output_prefix)+suffix).exists():
            parser.error("output exists; choose a new prefix")
    rclpy.init()
    node=JointTorqueRecorder(args.output_prefix,args.joint_topic,args.command_topic)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=="__main__":main()
