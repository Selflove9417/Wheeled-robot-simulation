#!/usr/bin/env python3
"""
bbot_gazebo_torque.launch.py
完整的力矩控制启动文件，包括:
  1. Gazebo 仿真 
  2. 机器人模型 
  3. 基础控制器 
  4. 腿部位置控制器 + 力矩控制器
  5. IMU 桥接
  6. 高度控制器 + 力矩监控节点

用法:
  ros2 launch bbot_bringup bbot_gazebo_torque.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ======== 包路径 ========
    pkg_bbot_description = FindPackageShare('bbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # ======== 启动参数 ========
    controller_config = LaunchConfiguration('controller_config')
    default_height = LaunchConfiguration('default_height')
    transition_duration = LaunchConfiguration('transition_duration')
    use_feedforward = LaunchConfiguration('use_feedforward')
    ff_gain = LaunchConfiguration('ff_gain')
    enable_torque_monitor = LaunchConfiguration('enable_torque_monitor')

    declare_controller_config = DeclareLaunchArgument(
        'controller_config', default_value='bbot_controllers_effort.yaml',
        description='控制器配置文件 (使用 effort 版本以支持力矩控制)'
    )
    declare_default_height = DeclareLaunchArgument(
        'default_height', default_value='0.55',
        description='默认目标质心高度 [m]'
    )
    declare_transition_duration = DeclareLaunchArgument(
        'transition_duration', default_value='1.5',
        description='高度过渡时间 [s]'
    )
    declare_use_feedforward = DeclareLaunchArgument(
        'use_feedforward', default_value='true',
        description='启用重力前馈补偿'
    )
    declare_ff_gain = DeclareLaunchArgument(
        'ff_gain', default_value='1.0',
        description='重力前馈增益'
    )
    declare_enable_torque_monitor = DeclareLaunchArgument(
        'enable_torque_monitor', default_value='true',
        description='启动力矩监控节点 (记录数据)'
    )

    # ======== URDF 模型 ========
    urdf_file = PathJoinSubstitution([
        pkg_bbot_description,
        'urdf',
        'bbot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ======== Gazebo 仿真 ========
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # ======== Robot State Publisher ========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ======== 生成机器人 ========
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bbot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.02'
        ],
        output='screen'
    )

    # ======== 控制器加载 (分阶段) ========
    # t+3s: joint_state_broadcaster
    load_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--param-file', PathJoinSubstitution([
                        FindPackageShare('bbot_bringup'),
                        'config',
                        controller_config
                    ])
                ],
                output='screen'
            )
        ]
    )

    # t+4s: diff_drive_controller
    load_diff_drive_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # t+5s: leg_position_controller (位置控制，向后兼容)
    load_leg_position_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'leg_position_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # t+6s: leg_effort_controller (力矩控制)
    load_leg_effort_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'leg_effort_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # ======== IMU 桥接 ========
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    # ======== 力矩控制节点 ========
    # t+7s: 高度控制器
    height_controller_node = TimerAction(
        period=7.0,
        actions=[
            Node(
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
                    'enable_control': True,
                }]
            )
        ]
    )

    # t+8s: 力矩监控 (数据记录)
    torque_monitor_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='bbot_torque_control',
                executable='torque_monitor',
                name='torque_monitor',
                output='screen',
                parameters=[{
                    'pub_rate_hz': 100,
                    'log_enabled': True,
                    'log_interval': 0.05,  # 20Hz 记录
                }],
                condition=IfCondition(enable_torque_monitor)
            )
        ]
    )

    # ======== 返回 LaunchDescription ========
    return LaunchDescription([
        # 参数声明
        declare_controller_config,
        declare_default_height,
        declare_transition_duration,
        declare_use_feedforward,
        declare_ff_gain,
        declare_enable_torque_monitor,

        # 启动
        LogInfo(msg='=== BBot Torque Control Launch ==='),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        load_leg_position_controller,
        load_leg_effort_controller,
        imu_bridge,
        height_controller_node,
        torque_monitor_node,
        LogInfo(msg='=== All nodes launched ==='),
    ])
