#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
import csv
import sys
import os
import math

class WheelContactRecorder(Node):
    def __init__(self, output_path):
        super().__init__('wheel_contact_recorder')
        self.output_path = output_path
        self.sim_time = 0.0
        self.last_ground_message_sim = None
        
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        self.left_sub = self.create_subscription(
            Contacts, '/model/bbot/left_wheel_contact', self.left_cb, 20)
        self.right_sub = self.create_subscription(
            Contacts, '/model/bbot/right_wheel_contact', self.right_cb, 20)
        self.ground_sub = self.create_subscription(
            Contacts, '/model/bbot/ground_contact', self.ground_cb, 100)
        
        self.csv_file = open(self.output_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'sim_time', 'side', 'stamp_sec', 'num_contacts',
            'normal_force_z', 'force_mag', 'collision_1', 'collision_2',
            'num_positions', 'num_normals', 'num_depths', 'num_wrenches',
            'force_status', 'ground_publisher_count', 'ground_message_age_sim'
        ])
        self.get_logger().info(f"Wheel contact recorder writing to: {self.output_path}")
        self.status_timer = self.create_timer(0.1, self.status_cb)

    def clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def parse_contacts(self, side: str, msg: Contacts):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        num = len(msg.contacts)
        if num == 0:
            self.writer.writerow([
                f"{self.sim_time:.4f}", side, f"{stamp:.4f}", 0,
                0.0, 0.0, "", "", 0, 0, 0, 0, 'NO_CONTACT'
                , '', ''
            ])
            self.csv_file.flush()
            return

        for c in msg.contacts:
            c1 = c.collision1.name if hasattr(c.collision1, 'name') else ""
            c2 = c.collision2.name if hasattr(c.collision2, 'name') else ""
            fz_total = 0.0
            f_mag_total = 0.0
            for w in c.wrenches:
                fx = w.body_1_wrench.force.x
                fy = w.body_1_wrench.force.y
                fz = w.body_1_wrench.force.z
                fz_total += abs(fz)
                f_mag_total += math.sqrt(fx*fx + fy*fy + fz*fz)
            
            self.writer.writerow([
                f"{self.sim_time:.4f}", side, f"{stamp:.4f}", num,
                f"{fz_total:.3f}", f"{f_mag_total:.3f}", c1, c2,
                len(c.positions), len(c.normals), len(c.depths),
                len(c.wrenches),
                'WRENCH_PRESENT' if c.wrenches else 'WRENCH_MISSING', '', ''
            ])
        self.csv_file.flush()
        # flush

    def left_cb(self, msg: Contacts):
        self.parse_contacts('LEFT', msg)

    def right_cb(self, msg: Contacts):
        self.parse_contacts('RIGHT', msg)

    def ground_cb(self, msg: Contacts):
        self.last_ground_message_sim = self.sim_time
        self.parse_contacts('GROUND', msg)

    def status_cb(self):
        publishers = self.ground_sub.get_publisher_count()
        age = '' if self.last_ground_message_sim is None else max(
            0.0, self.sim_time - self.last_ground_message_sim)
        self.writer.writerow([
            f"{self.sim_time:.4f}", 'GROUND_STATUS', f"{self.sim_time:.4f}",
            0, 0.0, 0.0, '', '', 0, 0, 0, 0,
            f"PUBLISHERS_{publishers}", publishers,
            '' if age == '' else f"{age:.4f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main():
    if len(sys.argv) < 2:
        out = "/tmp/wheel_contacts.csv"
    else:
        out = sys.argv[1]
    
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    rclpy.init()
    node = WheelContactRecorder(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
