import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="jump",
        description="Type of balance controller to run: jump, jump_velocity, lqr, gs_lqr, adaptive_lqr, pid, or none",
    )
    controller_type = LaunchConfiguration("controller_type")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="balance_test_world.sdf",
        description="World file to load in Gazebo (e.g. balance_test_world.sdf, empty.sdf)",
    )
    world = LaunchConfiguration("world")

    jump_height_arg = DeclareLaunchArgument("jump_height", default_value="0.20")
    takeoff_velocity_arg = DeclareLaunchArgument(
        "takeoff_velocity", default_value="0.0"
    )
    thrust_duration_arg = DeclareLaunchArgument("thrust_duration", default_value="0.24")
    thrust_peak_ratio_arg = DeclareLaunchArgument(
        "thrust_peak_ratio", default_value="2.0"
    )
    thrust_shape_early_arg = DeclareLaunchArgument(
        "thrust_shape_early", default_value="0.75"
    )
    thrust_shape_late_arg = DeclareLaunchArgument(
        "thrust_shape_late", default_value="0.75"
    )
    thrust_velocity_kp_arg = DeclareLaunchArgument(
        "thrust_velocity_kp", default_value="8.0"
    )
    thrust_timeout_arg = DeclareLaunchArgument("thrust_timeout", default_value="0.60")
    sim_relax_thrust_limits_arg = DeclareLaunchArgument(
        "sim_relax_thrust_limits",
        default_value="true",
        description="Use existing 150 Nm URDF limit during velocity-jump THRUST only",
    )
    air_wheel_sign_arg = DeclareLaunchArgument("air_wheel_sign", default_value="1.0")
    position_proportional_gain_arg = DeclareLaunchArgument(
        "position_proportional_gain", default_value="0.3"
    )
    body_mass_arg = DeclareLaunchArgument("body_mass", default_value="9.5")
    adaptive_experiment_mode_arg = DeclareLaunchArgument(
        "adaptive_experiment_mode",
        default_value="adaptive",
        description="Adaptive LQR experiment mode: nominal, oracle, or adaptive",
    )
    adaptive_com_y_bias_arg = DeclareLaunchArgument(
        "adaptive_com_y_bias",
        default_value="0.0",
        description="Injected controller-model COM-y bias in metres",
    )
    adaptive_log_path_arg = DeclareLaunchArgument(
        "adaptive_log_path",
        default_value="/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/adaptive_lqr_log.csv",
        description="CSV output path for the Adaptive LQR experiment",
    )
    adaptive_target_height_arg = DeclareLaunchArgument(
        "adaptive_target_height",
        default_value="0.50",
        description="Target wheel-axle to hip height in metres (real-robot convention)",
    )
    adaptive_startup_height_arg = DeclareLaunchArgument(
        "adaptive_startup_height",
        default_value="0.36",
        description="Safe wheel-axle to hip height used when the controller starts",
    )
    adaptive_apply_rate_max_arg = DeclareLaunchArgument(
        "adaptive_apply_rate_max",
        default_value="0.0010",
        description="Maximum rate of applying adaptive equilibrium offset in m/s",
    )
    adaptive_two_stage_enabled_arg = DeclareLaunchArgument(
        "adaptive_two_stage_enabled",
        default_value="true",
        description="Enable two-stage adaptive state machine (WAIT_COARSE -> APPLY_COARSE -> WAIT_FINE -> APPLY_FINE -> VERIFY -> HOLD)",
    )
    enable_position_handoff_arg = DeclareLaunchArgument(
        "enable_position_handoff", default_value="true"
    )

    ws_dir = "/home/admin/bbot_ws_new"
    opt_ros_dir = os.path.join(ws_dir, "opt_ros/opt/ros/iron")

    os.environ["ROS_HOME"] = os.path.join(ws_dir, ".ros")
    os.environ["ROS_LOG_DIR"] = os.path.join(ws_dir, ".ros/log")

    if os.path.exists(opt_ros_dir):
        opt_lib = os.path.join(opt_ros_dir, "lib")
        os.environ["LD_LIBRARY_PATH"] = (
            f"{opt_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        )
        os.environ["AMENT_PREFIX_PATH"] = (
            f"{opt_ros_dir}:{os.environ.get('AMENT_PREFIX_PATH', '')}"
        )
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = (
            f"{opt_lib}:{os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')}"
        )
        os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
            f"{opt_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
        )

    resource_paths = f"{os.path.join(ws_dir, 'src')}:{os.path.join(ws_dir, 'src/bbot_bringup/worlds')}:{os.path.join(ws_dir, 'install/bbot_description/share')}:{os.path.join(ws_dir, 'install/bbot_bringup/share/bbot_bringup/worlds')}"
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
        f"{resource_paths}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"
    )
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        f"{resource_paths}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    pkg_bbot_description = FindPackageShare("bbot_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    urdf_file = PathJoinSubstitution([pkg_bbot_description, "urdf", "bbot.urdf.xacro"])

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " position_proportional_gain:=",
                LaunchConfiguration("position_proportional_gain"),
                " body_mass:=",
                LaunchConfiguration("body_mass"),
            ]
        ),
        value_type=str,
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'-r ' + '", world, "'"])
        }.items(),
    )

    # 2. 发布 robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # 3. 生成实体
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "bbot",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.403",
        ],
        output="screen",
    )

    # 4. IMU 与 Clock 桥接
    imu_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/bbot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
        output="screen",
    )

    # 5. 加载 ROS 2 控制器
    load_joint_state_broadcaster = TimerAction(
        period=1.2,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    load_diff_drive_controller = TimerAction(
        period=1.5,
        # GS-LQR writes wheel *effort* commands directly.  The differential
        # drive controller claims the wheel velocity interfaces, so it must
        # not be active in that mode.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    controller_type,
                    "'.lower() not in ['gs_lqr', 'adaptive_lqr']",
                ]
            )
        ),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    load_leg_controller = TimerAction(
        period=1.8,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "leg_position_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    # 预加载 leg_effort_controller (不激活，供跳跃推地阶段动态切换)
    load_leg_effort_controller = TimerAction(
        period=2.2,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "leg_effort_controller",
                    "--inactive",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    load_wheel_effort_controller = TimerAction(
        # Unlike the other modes, GS-LQR commands this controller directly.
        # Loading it as --inactive silently discards every torque command and
        # leaves the robot uncontrolled.
        period=1.5,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    controller_type,
                    "'.lower() in ['gs_lqr', 'adaptive_lqr']",
                ]
            )
        ),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "wheel_effort_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
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
                output="screen",
            )
        ],
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
                output="screen",
            )
        ],
    )

    start_gs_lqr_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'gs_lqr'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="lqr_gain_scheduled_controller",
                output="screen",
            )
        ],
    )

    start_adaptive_lqr_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'adaptive_lqr'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="adaptive_lqr_balance_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "experiment.mode": LaunchConfiguration(
                            "adaptive_experiment_mode"
                        ),
                        "experiment.com_y_bias": ParameterValue(
                            LaunchConfiguration("adaptive_com_y_bias"),
                            value_type=float,
                        ),
                        "target_height": ParameterValue(
                            LaunchConfiguration("adaptive_target_height"),
                            value_type=float,
                        ),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("adaptive_startup_height"),
                            value_type=float,
                        ),
                        "adaptation.apply_rate_max": ParameterValue(
                            LaunchConfiguration("adaptive_apply_rate_max"),
                            value_type=float,
                        ),
                        "adaptation.two_stage_enabled": ParameterValue(
                            LaunchConfiguration("adaptive_two_stage_enabled"),
                            value_type=bool,
                        ),
                        "log_path": LaunchConfiguration("adaptive_log_path"),
                    }
                ],
            )
        ],
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
                output="screen",
            )
        ],
    )

    start_velocity_jump_controller = TimerAction(
        period=2.0,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'jump_velocity'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="bbot_velocity_jump_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "jump_height": LaunchConfiguration("jump_height"),
                        "takeoff_velocity": LaunchConfiguration("takeoff_velocity"),
                        "thrust_duration": LaunchConfiguration("thrust_duration"),
                        "thrust_peak_ratio": LaunchConfiguration("thrust_peak_ratio"),
                        "thrust_shape_early": LaunchConfiguration("thrust_shape_early"),
                        "thrust_shape_late": LaunchConfiguration("thrust_shape_late"),
                        "thrust_velocity_kp": LaunchConfiguration("thrust_velocity_kp"),
                        "thrust_timeout": LaunchConfiguration("thrust_timeout"),
                        "sim_relax_thrust_limits": ParameterValue(
                            LaunchConfiguration("sim_relax_thrust_limits"),
                            value_type=bool,
                        ),
                        "air_wheel_sign": LaunchConfiguration("air_wheel_sign"),
                        "position_proportional_gain": LaunchConfiguration(
                            "position_proportional_gain"
                        ),
                        "body_mass": LaunchConfiguration("body_mass"),
                        "enable_position_handoff": LaunchConfiguration(
                            "enable_position_handoff"
                        ),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            controller_type_arg,
            world_arg,
            jump_height_arg,
            takeoff_velocity_arg,
            thrust_duration_arg,
            thrust_peak_ratio_arg,
            thrust_shape_early_arg,
            thrust_shape_late_arg,
            thrust_velocity_kp_arg,
            thrust_timeout_arg,
            sim_relax_thrust_limits_arg,
            air_wheel_sign_arg,
            position_proportional_gain_arg,
            body_mass_arg,
            adaptive_experiment_mode_arg,
            adaptive_com_y_bias_arg,
            adaptive_log_path_arg,
            adaptive_target_height_arg,
            adaptive_startup_height_arg,
            adaptive_apply_rate_max_arg,
            adaptive_two_stage_enabled_arg,
            enable_position_handoff_arg,
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
            start_gs_lqr_controller,
            start_adaptive_lqr_controller,
            start_jump_controller,
            start_velocity_jump_controller,
            load_wheel_effort_controller,
        ]
    )
