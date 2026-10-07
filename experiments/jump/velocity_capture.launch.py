"""One private velocity-jump candidate; baseline launch/default remain intact."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('bbot_velocity_baseline_launch',
        root / 'src/bbot_bringup/launch/bbot_gazebo.launch.py')
    baseline = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(baseline)
    actions = []
    for action in baseline.generate_launch_description().entities:
        actions.append(action)
        if isinstance(action, DeclareLaunchArgument) and action.name == 'jump_controller_executable':
            actions.append(SetLaunchConfiguration('jump_controller_executable',
                str(root / 'build_native_allocator/bbot_balance_controller/bbot_landing_repair_controller')))
    # Scoped ROS parameters are appended by launch_ros to the private node.
    # All baseline launch arguments, physics and rate/effort limits are retained.
    return LaunchDescription([
        SetParameter(name='velocity_capture_experiment', value=True),
        SetParameter(name='thrust_support_allocator', value=False),
        *actions,
    ])
