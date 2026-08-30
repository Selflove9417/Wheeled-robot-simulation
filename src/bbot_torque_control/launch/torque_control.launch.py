#!/usr/bin/env python3
"""
torque_control.launch.py
启动力矩监控和高度控制节点

用法:
  ros2 launch bbot_torque_control torque_control.launch.py

参数:
  enable_height_control: 是否启动高度控制器 (默认: true)
  enable_torque_monitor: 是否启动力矩监控 (默认: true)
  default_height: 默认目标高度 [m] (默认: 0.55)
  transition_duration: 高度过渡时间 [s] (默认: 1.5)
  use_feedforward: 是否使用重力前馈 (默认: true)
  ff_gain: 前馈增益 (默认: 1.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    enable_height_ctrl = LaunchConfiguration('enable_height_control')
    enable_torque_mon = LaunchConfiguration('enable_torque_monitor')
    default_height = LaunchConfiguration('default_height')
    transition_duration = LaunchConfiguration('transition_duration')
    use_feedforward = LaunchConfiguration('use_feedforward')
    ff_gain = LaunchConfiguration('ff_gain')

    declare_height_ctrl = DeclareLaunchArgument(
        'enable_height_control', default_value='true',
        description='启用高度控制器节点'
    )
    declare_torque_mon = DeclareLaunchArgument(
        'enable_torque_monitor', default_value='true',
        description='启动力矩监控节点'
    )
    declare_height = DeclareLaunchArgument(
        'default_height', default_value='0.55',
        description='默认目标质心高度 [m] (0.30~0.60)'
    )
    declare_transition = DeclareLaunchArgument(
        'transition_duration', default_value='1.5',
        description='高度过渡时间 [s]'
    )
    declare_ff = DeclareLaunchArgument(
        'use_feedforward', default_value='true',
        description='是否启用重力前馈补偿'
    )
    declare_ff_gain = DeclareLaunchArgument(
        'ff_gain', default_value='1.0',
        description='重力前馈增益 (1.0=完全补偿)'
    )

    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_height_ctrl)
    ld.add_action(declare_torque_mon)
    ld.add_action(declare_height)
    ld.add_action(declare_transition)
    ld.add_action(declare_ff)
    ld.add_action(declare_ff_gain)

    # ======== 高度控制器节点 ========
    height_controller_node = Node(
        package='bbot_torque_control',
        executable='height_controller',
        name='height_controller',
        output='screen',
        parameters=[{
            'pub_rate_hz': 100,
            'default_height': default_height,
            'transition_duration': transition_duration,
            'use_feedforward': use_feedforward,
            'gravity_feedforward_gain': ff_gain,
            'hip_pid.P': 80.0,
            'hip_pid.I': 2.0,
            'hip_pid.D': 4.0,
            'knee_pid.P': 80.0,
            'knee_pid.I': 2.0,
            'knee_pid.D': 4.0,
        }],
        condition=None,  # 始终启动，可通过参数控制行为
    )

    # ======== 力矩监控节点 ========
    torque_monitor_node = Node(
        package='bbot_torque_control',
        executable='torque_monitor',
        name='torque_monitor',
        output='screen',
        parameters=[{
            'pub_rate_hz': 100,
            'log_enabled': True,
            'log_interval': 0.05,
        }],
    )

    ld.add_action(LogInfo(msg='=== bbot 力矩控制启动 ==='))
    ld.add_action(height_controller_node)
    ld.add_action(torque_monitor_node)

    return ld
