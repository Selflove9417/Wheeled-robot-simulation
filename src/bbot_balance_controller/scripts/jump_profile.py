#!/usr/bin/env python3
"""Resolved defaults for the supported rolling jump profiles."""

from copy import deepcopy
import math
import xml.etree.ElementTree as ET


JUMP_VELOCITY_PROFILE = {
    "world": "flat_jump_world.sdf",
    "real_time_factor": 1.0,
    "gazebo_start_paused": True,
    "staged_controller_startup": True,
    "auto_unpause": True,
    "jump_height": 0.20,
    "jump_forward_speed": 0.35,
    "jump_takeoff_forward_speed": 0.45,
    "thrust_forward_velocity_kp": 1.20,
    "thrust_forward_attitude_taper": True,
    "thrust_release_fast_rate_correction": True,
    "thrust_fast_rate_correction_from_gate": True,
    "thrust_release_velocity_ratio": 0.78,
    "thrust_forward_speed_prediction": True,
    "thrust_wheel_kinematics_compensation": True,
    "complete_contact_takeoff_confirmation": True,
    "arrest_dynamics_feedforward": True,
    "thrust_wheel_max_decel": 16.0,
}

OBSERVE_REPEATABILITY_PROFILE = {
    "real_time_factor": 1.0,
    "jump_height": 0.20,
    "jump_forward_speed": 0.35,
    "jump_takeoff_forward_speed": 0.45,
    "thrust_forward_velocity_kp": 0.80,
    "thrust_forward_attitude_taper": False,
    "thrust_release_fast_rate_correction": False,
    "thrust_fast_rate_correction_from_gate": False,
    "thrust_release_velocity_ratio": 0.70,
    "thrust_forward_speed_prediction": False,
    "thrust_wheel_kinematics_compensation": False,
    "complete_contact_takeoff_confirmation": False,
    "arrest_dynamics_feedforward": False,
    "thrust_wheel_max_decel": 8.0,
}


def resolve_profile(controller_type="jump_velocity", observe_repeatability=False,
                    overrides=None):
    """Return the profile defaults with only explicit non-None overrides applied."""
    if observe_repeatability:
        values = deepcopy(OBSERVE_REPEATABILITY_PROFILE)
    elif str(controller_type).lower() == "jump_velocity":
        values = deepcopy(JUMP_VELOCITY_PROFILE)
    else:
        values = {}
    explicit = overrides or {}
    for key, value in explicit.items():
        if value is not None:
            values[key] = value
    if (explicit.get("thrust_release_fast_rate_correction") is False and
            explicit.get("thrust_fast_rate_correction_from_gate") is None):
        values["thrust_fast_rate_correction_from_gate"] = False
    if (values.get("thrust_fast_rate_correction_from_gate") and
            not values.get("thrust_release_fast_rate_correction")):
        raise ValueError("gate-scoped fast-rate correction requires its master correction")
    return values


def launch_default_expression(parameter, legacy_value, controller_type_substitution):
    """Build a launch expression preserving non-jump defaults and overrides."""
    value = JUMP_VELOCITY_PROFILE[parameter]
    def literal(item):
        if isinstance(item, bool):
            return "true" if item else "false"
        return str(item)
    return [
        "'", literal(value), "' if '", controller_type_substitution,
        "'.lower() == 'jump_velocity' else '", literal(legacy_value), "'",
    ]


def launch_gate_scope_expression(controller_type_substitution, master_substitution):
    """Default gate scope on only when jump mode and its master are enabled."""
    return [
        "'true' if '", controller_type_substitution,
        "'.lower() == 'jump_velocity' and '", master_substitution,
        "'.lower() in ['true', '1'] else 'false'",
    ]


def validate_fast_rate_scope(master_enabled, gate_scope_enabled):
    """Reject an explicitly inconsistent fast-rate feature pair."""
    if gate_scope_enabled and not master_enabled:
        raise ValueError(
            "gate-scoped fast-rate correction requires its master correction")


def rewrite_world_real_time_factor(source_path, output_path, factor):
    """Write a temporary SDF copy changing only the world's real-time factor."""
    factor = float(factor)
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError("real-time factor must be finite and positive")
    tree = ET.parse(source_path)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("selected SDF has no world element")
    physics = world.find("physics")
    if physics is None:
        raise ValueError("selected world has no physics element")
    nodes = physics.findall("real_time_factor")
    if len(nodes) != 1:
        raise ValueError("selected world must contain exactly one physics real_time_factor")
    nodes[0].text = format(factor, ".12g")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path
