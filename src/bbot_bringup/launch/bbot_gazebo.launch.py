import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='jump',
        description='Type of balance controller to run: jump, lqr, pid, or none'
    )
    controller_type = LaunchConfiguration('controller_type')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='balance_test_world.sdf',
        description='World file to load in Gazebo (e.g. balance_test_world.sdf, empty.sdf)'
    )
    world = LaunchConfiguration('world')

    ws_dir = '/home/admin/bbot_ws_new'
    opt_ros_dir = os.path.join(ws_dir, 'opt_ros/opt/ros/iron')
    
    os.environ['ROS_HOME'] = os.path.join(ws_dir, '.ros')
    os.environ['ROS_LOG_DIR'] = os.path.join(ws_dir, '.ros/log')
    
    if os.path.exists(opt_ros_dir):
        opt_lib = os.path.join(opt_ros_dir, 'lib')
        os.environ['LD_LIBRARY_PATH'] = f"{opt_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        os.environ['AMENT_PREFIX_PATH'] = f"{opt_ros_dir}:{os.environ.get('AMENT_PREFIX_PATH', '')}"
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = f"{opt_lib}:{os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')}"
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = f"{opt_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    resource_paths = f"{os.path.join(ws_dir, 'src')}:{os.path.join(ws_dir, 'src/bbot_bringup/worlds')}:{os.path.join(ws_dir, 'install/bbot_description/share')}:{os.path.join(ws_dir, 'install/bbot_bringup/share/bbot_bringup/worlds')}"
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = f"{resource_paths}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{resource_paths}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"

    pkg_bbot_description = FindPackageShare('bbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    urdf_file = PathJoinSubstitution([
        pkg_bbot_description,
        'urdf',
        'bbot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': PythonExpression(["'-r ' + '", world, "'"])
        }.items()
    )

    # 2. 发布 robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # 3. 生成实体
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bbot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.403'
        ],
        output='screen'
    )

    # 4. IMU 与 Clock 桥接
    imu_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    # 5. 加载 ROS 2 控制器
    load_joint_state_broadcaster = TimerAction(
        period=1.2,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    load_diff_drive_controller = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    load_leg_controller = TimerAction(
        period=1.8,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'leg_position_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # 预加载 leg_effort_controller (不激活，供跳跃推地阶段动态切换)
    load_leg_effort_controller = TimerAction(
        period=2.2,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'leg_effort_controller',
                    '--inactive',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # 6. 平衡控制器 (按 controller_type 参数选择启动: 'pid' 或 'lqr')
    start_pid_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'pid'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="balance_controller_keyboard",
                output="screen"
            )
        ]
    )

    start_lqr_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'lqr'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="lqr_balance_controller_yaokong",
                output="screen"
            )
        ]
    )

    start_jump_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'jump'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="bbot_jump_controller",
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        controller_type_arg,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        imu_clock_bridge,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        load_leg_controller,
        load_leg_effort_controller,
        start_pid_controller,
        start_lqr_controller,
        start_jump_controller
    ])
