"""Private allocator-off low-speed ground motion; no jump requests allowed."""
import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location(
        "bbot_ground_input_baseline_launch",
        root / "src/bbot_bringup/launch/bbot_gazebo.launch.py")
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    actions = []
    for action in baseline.generate_launch_description().entities:
        actions.append(action)
        if isinstance(action, DeclareLaunchArgument) and action.name == "jump_controller_executable":
            actions.append(SetLaunchConfiguration(
                "jump_controller_executable",
                str(root / "build_ground_motion/bbot_balance_controller/bbot_landing_repair_controller")))
    return LaunchDescription([
        SetParameter(name="ground_input_experiment", value=True),
        SetParameter(name="ground_motion_experiment", value=True),
        SetParameter(name="native_command_state_diagnostics", value=True),
        SetParameter(name="thrust_support_allocator", value=False),
        SetParameter(name="velocity_capture_experiment", value=False),
        SetParameter(name="wheel_servo_gain", value=1.0),
        *actions,
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="ground_input_native_state_bridge",
             arguments=["/world/flat_jump_world/native_command_state@std_msgs/msg/String[ignition.msgs.StringMsg"],
             output="screen"),
    ])
