#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

MSG = """
╔══════════════════════════════════════════════════════════╗
║             BBOT 键盘遥控终端 (Teleop Control)             ║
╠══════════════════════════════════════════════════════════╣
║  [移动控制]                                              ║
║      W / ↑   : 前进 (+0.3 m/s)                           ║
║      S / ↓   : 后退 (-0.3 m/s)                           ║
║      A / ←   : 左转 (+0.5 rad/s)                         ║
║      D / →   : 右转 (-0.5 rad/s)                         ║
║      Space   : 停止移动 (保持原地平衡)                     ║
║                                                          ║
║  [高度控制]                                              ║
║      Q       : 升高机身 (+1cm, 范围 0.30 ~ 0.44m)        ║
║      E       : 降低机身 (-1cm, 范围 0.30 ~ 0.44m)        ║
║                                                          ║
║  [状态模式]                                              ║
║      R       : 触发倒地自恢复起立                         ║
║      X       : 紧急停机                                  ║
║      Ctrl+C  : 退出控制终端                              ║
╚══════════════════════════════════════════════════════════╝
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_height = self.create_publisher(Float64, '/target_height', 10)
        self.pub_mode = self.create_publisher(String, '/robot_mode', 10)
        
        self.speed = 0.35
        self.turn = 0.60
        self.height = 0.400
        self.min_height = 0.300
        self.max_height = 0.440
        
        self.linear_x = 0.0
        self.angular_z = 0.0

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(twist)

    def publish_height(self, height):
        msg = Float64()
        msg.data = float(height)
        self.pub_height.publish(msg)

    def publish_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.pub_mode.publish(msg)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Arrow keys (ESC sequence)
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopKeyboard()
    
    print(MSG)
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'w' or key == 'W' or key == '\x1b[A':
                node.linear_x = node.speed
                node.angular_z = 0.0
                node.publish_twist(node.linear_x, node.angular_z)
                print(f"\r[指令] 前进 → 速度: {node.linear_x:+.2f} m/s, 转向: {node.angular_z:+.2f} rad/s    ", end='', flush=True)
            elif key == 's' or key == 'S' or key == '\x1b[B':
                node.linear_x = -node.speed
                node.angular_z = 0.0
                node.publish_twist(node.linear_x, node.angular_z)
                print(f"\r[指令] 后退 → 速度: {node.linear_x:+.2f} m/s, 转向: {node.angular_z:+.2f} rad/s    ", end='', flush=True)
            elif key == 'a' or key == 'A' or key == '\x1b[D':
                node.linear_x = 0.0
                node.angular_z = node.turn
                node.publish_twist(node.linear_x, node.angular_z)
                print(f"\r[指令] 左转 → 速度: {node.linear_x:+.2f} m/s, 转向: {node.angular_z:+.2f} rad/s    ", end='', flush=True)
            elif key == 'd' or key == 'D' or key == '\x1b[C':
                node.linear_x = 0.0
                node.angular_z = -node.turn
                node.publish_twist(node.linear_x, node.angular_z)
                print(f"\r[指令] 右转 → 速度: {node.linear_x:+.2f} m/s, 转向: {node.angular_z:+.2f} rad/s    ", end='', flush=True)
            elif key == ' ':
                node.linear_x = 0.0
                node.angular_z = 0.0
                node.publish_twist(0.0, 0.0)
                print(f"\r[指令] 停止移动 (原地自平衡)                                ", end='', flush=True)
            elif key == 'q' or key == 'Q':
                node.height = min(node.max_height, node.height + 0.01)
                node.publish_height(node.height)
                print(f"\r[指令] 升高机身 → 目标高度: {node.height:.3f} m                   ", end='', flush=True)
            elif key == 'e' or key == 'E':
                node.height = max(node.min_height, node.height - 0.01)
                node.publish_height(node.height)
                print(f"\r[指令] 降低机身 → 目标高度: {node.height:.3f} m                   ", end='', flush=True)
            elif key == 'r' or key == 'R':
                node.publish_mode("standup")
                print(f"\r[指令] 触发自恢复起立模式！                                  ", end='', flush=True)
            elif key == 'x' or key == 'X':
                node.publish_mode("emergency")
                print(f"\r[指令] 紧急停机！                                          ", end='', flush=True)
            elif key == '\x03':  # Ctrl+C
                break
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n键盘控制已退出。")

if __name__ == '__main__':
    main()
