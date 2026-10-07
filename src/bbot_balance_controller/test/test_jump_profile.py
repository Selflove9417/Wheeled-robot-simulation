#!/usr/bin/env python3
from pathlib import Path
import sys
import os
import tempfile
import xml.etree.ElementTree as ET

os.environ["ROS_HOME"] = "/tmp/bbot_jump_profile_test_ros"
Path(os.environ["ROS_HOME"]).mkdir(parents=True, exist_ok=True)

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration, PythonExpression

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from jump_profile import (  # noqa: E402
    JUMP_VELOCITY_PROFILE,
    OBSERVE_REPEATABILITY_PROFILE,
    launch_default_expression,
    launch_gate_scope_expression,
    rewrite_world_real_time_factor,
    resolve_profile,
    validate_fast_rate_scope,
)


def main():
    normal = resolve_profile("jump_velocity")
    assert normal == JUMP_VELOCITY_PROFILE
    assert normal["world"] == "flat_jump_world.sdf"
    assert normal["real_time_factor"] == 1.0
    assert normal["thrust_release_velocity_ratio"] == 0.78
    assert normal["arrest_dynamics_feedforward"] is True

    no_feature_overrides = {
        "thrust_forward_attitude_taper": False,
        "thrust_release_fast_rate_correction": False,
        "thrust_fast_rate_correction_from_gate": False,
        "thrust_forward_speed_prediction": False,
        "thrust_wheel_kinematics_compensation": False,
        "complete_contact_takeoff_confirmation": False,
        "arrest_dynamics_feedforward": False,
        "thrust_forward_velocity_kp": 0.80,
        "thrust_release_velocity_ratio": 0.70,
        "thrust_wheel_max_decel": 8.0,
    }
    overridden = resolve_profile("jump_velocity", overrides=no_feature_overrides)
    for key, value in no_feature_overrides.items():
        assert overridden[key] == value
    master_off_only = resolve_profile("jump_velocity", overrides={
        "thrust_release_fast_rate_correction": False})
    assert master_off_only["thrust_fast_rate_correction_from_gate"] is False
    validate_fast_rate_scope(False, False)
    try:
        resolve_profile("jump_velocity", overrides={
            "thrust_release_fast_rate_correction": False,
            "thrust_fast_rate_correction_from_gate": True})
        raise AssertionError("explicit gate scope without its master should be rejected")
    except ValueError:
        pass
    try:
        validate_fast_rate_scope(False, True)
        raise AssertionError("explicit gate scope without its master should fail")
    except ValueError:
        pass

    observed = resolve_profile("jump_velocity", observe_repeatability=True)
    assert observed == OBSERVE_REPEATABILITY_PROFILE
    assert observed["real_time_factor"] == 1.0
    assert observed["complete_contact_takeoff_confirmation"] is False
    assert observed["arrest_dynamics_feedforward"] is False
    assert observed["thrust_release_velocity_ratio"] == 0.70
    assert resolve_profile("gs_lqr") == {}
    world_expr = launch_default_expression("world", "balance_test_world.sdf", "controller_type")
    assert world_expr == [
        "'", "flat_jump_world.sdf", "' if '", "controller_type",
        "'.lower() == 'jump_velocity' else '", "balance_test_world.sdf", "'",
    ]
    assert launch_gate_scope_expression("controller_type", "master_enabled") == [
        "'true' if '", "controller_type",
        "'.lower() == 'jump_velocity' and '", "master_enabled",
        "'.lower() in ['true', '1'] else 'false'",
    ]
    context = LaunchContext()
    context.launch_configurations.update({
        "controller_type": "jump_velocity",
        "master_enabled": "false",
    })
    scope_expr = PythonExpression(launch_gate_scope_expression(
        LaunchConfiguration("controller_type"),
        LaunchConfiguration("master_enabled")))
    assert scope_expr.perform(context) == "false"
    context.launch_configurations["master_enabled"] = "true"
    assert scope_expr.perform(context) == "true"

    world_source = Path(__file__).resolve().parents[2] / "bbot_bringup" / "worlds" / "flat_jump_world.sdf"
    original = ET.parse(world_source).getroot()
    assert float(original.findtext("world/physics/real_time_factor")) == 1.0
    assert float(original.findtext("world/physics/max_step_size")) == 0.001
    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir) / "flat_jump_world_override.sdf"
        rewrite_world_real_time_factor(world_source, output, 1.25)
        rewritten = ET.parse(output).getroot()
        assert rewritten.find("world").attrib == original.find("world").attrib
        assert rewritten.findtext("world/physics/max_step_size") == "0.001"
        assert float(rewritten.findtext("world/physics/real_time_factor")) == 1.25
    assert float(ET.parse(world_source).getroot().findtext(
        "world/physics/real_time_factor")) == 1.0
    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir) / "invalid.sdf"
        for invalid_factor in (0.0, -1.0, float("nan"), float("inf")):
            try:
                rewrite_world_real_time_factor(world_source, output, invalid_factor)
                raise AssertionError(f"invalid RTF {invalid_factor} should fail")
            except ValueError:
                pass

    explicit_observe_override = resolve_profile(
        "jump_velocity", observe_repeatability=True,
        overrides={"arrest_dynamics_feedforward": True})
    assert explicit_observe_override["arrest_dynamics_feedforward"] is True
    print("PASS: jump profiles, overrides, and optional world RTF copy")


if __name__ == "__main__":
    main()
