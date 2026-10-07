import os
import sys
import atexit
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
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

_jump_profile_lib = os.path.join(get_package_prefix("bbot_balance_controller"),
                                 "lib", "bbot_balance_controller")
if _jump_profile_lib not in sys.path:
    sys.path.insert(0, _jump_profile_lib)
from jump_profile import (
    launch_default_expression,
    launch_gate_scope_expression,
    rewrite_world_real_time_factor,
    validate_fast_rate_scope,
)


def generate_launch_description():
    ws_dir = str(Path(__file__).resolve().parents[3])
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="jump_velocity",
        description="Type of balance controller to run: jump, jump_velocity, lqr, gs_lqr, mpc, adaptive_lqr, gs_lqr_historical, pid, torque_cascade_pid, position_torque_cascade_pid, or none",
    )
    controller_type = LaunchConfiguration("controller_type")

    def jump_default(parameter, legacy_value):
        return PythonExpression(launch_default_expression(
            parameter, legacy_value, controller_type))

    allocator_runner_guard = DeclareLaunchArgument("allocator_runner_guard", default_value="false")
    def validate_fast_rate_launch(context):
        # The runner loads/discovers the bounded P wheel servo while paused.
        if LaunchConfiguration("allocator_runner_guard").perform(context).lower() != "true":
            raise RuntimeError("allocator launch must be started by run_allocator_jump_candidate.sh")
        if LaunchConfiguration("auto_unpause").perform(context).lower() != "false" or LaunchConfiguration("gazebo_start_paused").perform(context).lower() != "true":
            raise RuntimeError("allocator requires paused runner effort-servo startup")
        mode = LaunchConfiguration("controller_type").perform(context).lower()
        master = LaunchConfiguration("thrust_release_fast_rate_correction").perform(context)
        scope = LaunchConfiguration("thrust_fast_rate_correction_from_gate").perform(context)
        if mode == "jump_velocity":
            try:
                validate_fast_rate_scope(master.lower() in ("true", "1"),
                                         scope.lower() in ("true", "1"))
            except ValueError as exc:
                raise RuntimeError(str(exc)) from exc
        momentum = LaunchConfiguration("thrust_momentum_reference").perform(context).lower()
        executable = LaunchConfiguration("jump_controller_executable").perform(context)
        handoff = LaunchConfiguration("thrust_reference_handoff").perform(context).lower()
        if handoff in ("true", "1") and momentum not in ("true", "1"):
            raise RuntimeError("thrust_reference_handoff requires thrust_momentum_reference")
        if momentum in ("true", "1") and (mode != "jump_velocity" or
                executable != "bbot_landing_repair_controller"):
            raise RuntimeError("thrust_momentum_reference requires the independent landing-repair controller")
        return []

    # Linear MPC (phase 1, fixed height).  These defaults mirror
    # config/mpc_balance_params.yaml and the node's own defaults.  The height
    # interface, theta_eq rule, filters, period and torque authority all follow
    # adaptive_lqr_balance_controller.cpp rather than lqr_gain_scheduled_controller.cpp.
    mpc_config_file_arg = DeclareLaunchArgument(
        "mpc_config_file",
        default_value=f"{ws_dir}/src/bbot_balance_controller/config/mpc_balance_params.yaml",
        description="YAML holding linear_mpc_balance_controller ros__parameters",
    )
    mpc_target_height_arg = DeclareLaunchArgument(
        "mpc_target_height",
        default_value="0.40",
        description="Fixed wheel-axle to hip height H [m] the model is designed at "
                    "(same convention as adaptive_target_height)",
    )
    mpc_horizon_arg = DeclareLaunchArgument(
        "mpc_horizon",
        default_value="20",
        description="Prediction horizon N at the 0.005 s control period",
    )
    mpc_r_arg = DeclareLaunchArgument(
        "mpc_r",
        default_value="8.0",
        description="Control weight R of the MPC cost; 8.0 is the nominal value from the "
                    "frozen-configuration coarse sweep (not a claimed optimum, see "
                    "MPC_POSITION_RESIDUAL_ANALYSIS.md). Q stays at the repository baseline",
    )
    mpc_theta_error_limit_arg = DeclareLaunchArgument(
        "mpc_theta_error_limit",
        default_value="0.20",
        description="Pitch-error safety band enforced over the horizon [rad]",
    )
    mpc_use_theta_band_arg = DeclareLaunchArgument(
        "mpc_use_theta_band",
        default_value="true",
        description="Enable the pitch-error band constraint (false = actuator box only)",
    )
    mpc_wheel_torque_max_arg = DeclareLaunchArgument(
        "mpc_wheel_torque_max",
        default_value="10.0",
        description="Per-wheel torque authority [Nm]; RobotParams default is 10",
    )
    mpc_total_torque_max_arg = DeclareLaunchArgument(
        "mpc_total_torque_max",
        default_value="20.0",
        description="Total wheel torque authority [Nm] (2 x per-wheel default)",
    )
    mpc_theta_eq_source_arg = DeclareLaunchArgument(
        "mpc_theta_eq_source",
        default_value="table",
        description="Nominal balance angle: table (adaptive_lqr -atan2(y_com,z_com), tracked at the "
                    "commanded height) | geometric (continuous model port) | override",
    )
    mpc_theta_eq_arg = DeclareLaunchArgument(
        "mpc_theta_eq",
        default_value="0.0",
        description="Nominal balance pitch used only when mpc_theta_eq_source:=override [rad]",
    )
    mpc_startup_height_arg = DeclareLaunchArgument(
        "mpc_startup_height",
        default_value="0.36",
        description="Safe wheel-axle to hip height used when the controller starts "
                    "(same as adaptive_startup_height)",
    )
    mpc_leg_transition_speed_arg = DeclareLaunchArgument(
        "mpc_leg_transition_speed",
        default_value="0.05",
        description="Maximum hip-axle height transition speed [m/s] (same as adaptive_leg_transition_speed)",
    )
    mpc_startup_hold_time_arg = DeclareLaunchArgument(
        "mpc_startup_hold_time",
        default_value="2.0",
        description="Seconds before the leg extension starts; wheel torque is not held back "
                    "(adaptive_lqr_balance_controller.cpp:713-715)",
    )
    mpc_spawn_z_arg = DeclareLaunchArgument(
        "mpc_spawn_z",
        default_value="0.403",
        description="base_link spawn height [m]. 0.403 is the value every other controller in this "
                    "repo has used; the MPC startup posture is hip-axle 0.36 m, i.e. base_link 0.50 m.",
    )
    mpc_log_path_arg = DeclareLaunchArgument(
        "mpc_log_path",
        default_value=f"{ws_dir}/src/bbot_balance_controller/src/data_logs/mpc/mpc_balance_log.csv",
        description="MPC CSV log path",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=jump_default("world", "balance_test_world.sdf"),
        description="World file to load in Gazebo (e.g. balance_test_world.sdf, empty.sdf)",
    )
    world = LaunchConfiguration("world")

    # Empty means keep the selected SDF's own physics rate.  This preserves
    # unrelated worlds; callers can explicitly override it for a run.
    real_time_factor_arg = DeclareLaunchArgument(
        "real_time_factor",
        default_value="",
        description="Optional positive Gazebo real-time-factor override for the selected world; empty keeps the SDF value",
    )

    def apply_explicit_world_rtf(context):
        requested = LaunchConfiguration("real_time_factor").perform(context).strip()
        if not requested:
            return []
        try:
            factor = float(requested)
        except ValueError as exc:
            raise RuntimeError("real_time_factor must be a finite positive number") from exc
        if not math.isfinite(factor) or factor <= 0.0:
            raise RuntimeError("real_time_factor must be a finite positive number")

        selected = LaunchConfiguration("world").perform(context)
        source = Path(selected).expanduser()
        candidates = [source] if source.is_absolute() else [Path.cwd() / source]
        if not source.is_absolute():
            for env_name in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
                for entry in os.environ.get(env_name, "").split(":"):
                    if entry:
                        root = Path(entry)
                        candidates.extend((root / source, root / "worlds" / source))
            candidates.extend((
                Path(__file__).resolve().parents[1] / "worlds" / source,
                Path(get_package_prefix("bbot_bringup")) / "share" / "bbot_bringup" / "worlds" / source,
            ))
        resolved = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
        if resolved is None:
            raise RuntimeError(f"cannot resolve world SDF for real_time_factor override: {selected}")
        fd, copied_name = tempfile.mkstemp(prefix="bbot_world_rtf_", suffix=".sdf")
        os.close(fd)
        copied = Path(copied_name)
        try:
            rewrite_world_real_time_factor(resolved, copied, factor)
        except Exception:
            copied.unlink(missing_ok=True)
            raise
        atexit.register(lambda path=copied: path.unlink(missing_ok=True))
        return [
            SetLaunchConfiguration("effective_world", str(copied)),
            LogInfo(msg=f"[world-physics] explicit real_time_factor={factor:g}; temporary SDF={copied}"),
        ]

    gazebo_world_name_arg = DeclareLaunchArgument(
        "gazebo_world_name",
        default_value=PythonExpression([
            "'", world, "'.split('/')[-1].replace('.sdf', '').replace('.world', '')"
        ]),
        description="Gazebo world entity name used by the world control service",
    )
    gazebo_world_name = LaunchConfiguration("gazebo_world_name")

    gazebo_start_paused_arg = DeclareLaunchArgument(
        "gazebo_start_paused",
        default_value=PythonExpression([
            "'true' if '", controller_type,
            "'.lower() in ['gs_lqr', 'jump_velocity'] else 'false'"
        ]),
        description="Start Gazebo paused (default: true for gs_lqr and jump_velocity)",
    )
    gazebo_start_paused = LaunchConfiguration("gazebo_start_paused")

    auto_unpause_arg = DeclareLaunchArgument(
        "auto_unpause",
        default_value="true",
        description="Automatically unpause paused physics after the controller-specific ready gate; jump_velocity waits for fresh sensors and commands",
    )
    auto_unpause = LaunchConfiguration("auto_unpause")

    staged_controller_startup_arg = DeclareLaunchArgument(
        "staged_controller_startup",
        default_value=PythonExpression([
            "'true' if '", controller_type,
            "'.lower() == 'jump_velocity' else 'false'"
        ]),
        description="Load jump_velocity ground controllers inactive for the shared paused readiness gate; other controller defaults are unchanged",
    )
    staged_controller_startup = LaunchConfiguration("staged_controller_startup")

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode without GUI (server-only)",
    )
    headless = LaunchConfiguration("headless")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Whether to display the Gazebo 3D simulation GUI window",
    )
    gui = LaunchConfiguration("gui")

    gazebo_record_path_arg = DeclareLaunchArgument(
        "gazebo_record_path",
        default_value="",
        description="Optional writable Gazebo record path; empty disables recording",
    )
    gazebo_record_path = LaunchConfiguration("gazebo_record_path")

    jump_controller_executable_arg = DeclareLaunchArgument(
        "jump_controller_executable", default_value="bbot_velocity_jump_controller",
        choices=["bbot_velocity_jump_controller", "bbot_reference_jump_controller",
                 "bbot_landing_repair_controller"],
        description="Select the baseline or an independent jump experiment")
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
    air_wheel_sign_arg = DeclareLaunchArgument("air_wheel_sign", default_value="-1.0")
    air_wheel_extend_kd_arg = DeclareLaunchArgument("air_wheel_extend_kd", default_value="2.50")
    air_wheel_tuck_kd_arg = DeclareLaunchArgument("air_wheel_tuck_kd", default_value="0.45")
    flight_tuck_nominal_duration_arg = DeclareLaunchArgument(
        "flight_tuck_nominal_duration", default_value="0.06")
    flight_arrest_freewheel_duration_arg = DeclareLaunchArgument(
        "flight_arrest_freewheel_duration", default_value="0.0")
    thrust_terminal_hip_floor_arg = DeclareLaunchArgument(
        "thrust_terminal_hip_floor", default_value="false")
    landing_buffer_height_arg = DeclareLaunchArgument(
        "landing_buffer_height", default_value="0.34"
    )
    position_proportional_gain_arg = DeclareLaunchArgument(
        "position_proportional_gain", default_value="0.3"
    )
    body_mass_arg = DeclareLaunchArgument("body_mass", default_value="9.5")
    jump_log_path_arg = DeclareLaunchArgument(
        "jump_log_path", default_value="", description="Path to jump control log CSV"
    )
    jump_summary_path_arg = DeclareLaunchArgument(
        "jump_summary_path", default_value="", description="Path to jump summary CSV"
    )
    auto_return_balance_arg = DeclareLaunchArgument(
        "auto_return_balance", default_value="true", description="Auto return to Effort BALANCE after jump recovery"
    )
    jump_forward_speed_arg = DeclareLaunchArgument(
        "jump_forward_speed", default_value="0.35", description="PRE_JUMP forward approach speed"
    )
    jump_takeoff_forward_speed_arg = DeclareLaunchArgument(
        "jump_takeoff_forward_speed", default_value="0.45", description="Target takeoff forward speed"
    )
    thrust_forward_velocity_kp_arg = DeclareLaunchArgument(
        "thrust_forward_velocity_kp", default_value=jump_default("thrust_forward_velocity_kp", 0.80),
        description="THRUST world COM forward-velocity feedback gain (0 to 2)"
    )
    thrust_forward_attitude_taper_arg = DeclareLaunchArgument(
        "thrust_forward_attitude_taper", default_value=jump_default("thrust_forward_attitude_taper", False),
        description="Fade negative THRUST attitude wheel drive near COM takeoff speed"
    )
    thrust_release_fast_rate_correction_arg = DeclareLaunchArgument(
        "thrust_release_fast_rate_correction", default_value=jump_default("thrust_release_fast_rate_correction", False),
        description="Bounded signed fast/legacy gyro rate correction with confirmed Effort support"
    )
    thrust_fast_rate_correction_from_gate_arg = DeclareLaunchArgument(
        "thrust_fast_rate_correction_from_gate",
        default_value=PythonExpression(launch_gate_scope_expression(
            controller_type, LaunchConfiguration("thrust_release_fast_rate_correction"))),
        description="Blend bounded signed gyro correction in over 20 ms after the THRUST motion gate"
    )
    thrust_reference_handoff_arg = DeclareLaunchArgument(
        "thrust_reference_handoff", default_value="false",
        description="Private landing-repair velocity handoff; requires thrust_momentum_reference",
    )
    thrust_momentum_reference_arg = DeclareLaunchArgument(
        "thrust_momentum_reference", default_value="false",
        description="Private landing-repair supported COM velocity/angular-momentum reference experiment"
    )
    thrust_support_coordination_arg = DeclareLaunchArgument(
        "thrust_support_coordination", default_value="false",
        description="Private landing-repair COM and vertical-impulse coordination experiment",
    )
    thrust_release_velocity_ratio_arg = DeclareLaunchArgument(
        "thrust_release_velocity_ratio", default_value=jump_default("thrust_release_velocity_ratio", 0.70),
        description="THRUST unloading onset fraction of target takeoff COM speed (0.70 to 0.78)"
    )
    thrust_forward_speed_prediction_arg = DeclareLaunchArgument(
        "thrust_forward_speed_prediction", default_value=jump_default("thrust_forward_speed_prediction", False),
        description="Bounded recent-frame forward-speed prediction for THRUST wheel feedback/taper"
    )
    thrust_wheel_kinematics_compensation_arg = DeclareLaunchArgument(
        "thrust_wheel_kinematics_compensation", default_value=jump_default("thrust_wheel_kinematics_compensation", False),
        description="Bounded CAD COM/axle kinematic braking correction during Effort THRUST"
    )
    complete_contact_takeoff_confirmation_arg = DeclareLaunchArgument(
        "complete_contact_takeoff_confirmation", default_value=jump_default("complete_contact_takeoff_confirmation", False),
        description="Use complete per-physics ground contact frames plus aligned COM rise as an additional takeoff witness"
    )
    arrest_dynamics_feedforward_arg = DeclareLaunchArgument(
        "arrest_dynamics_feedforward", default_value=jump_default("arrest_dynamics_feedforward", False),
        description="CAD inverse-dynamics trajectory feedforward in Effort ATTITUDE_ARREST"
    )
    jump_pitch_offset_arg = DeclareLaunchArgument(
        "jump_pitch_offset", default_value="0.020", description="Pitch offset for jump thrust"
    )
    jump_takeoff_pitch_rate_arg = DeclareLaunchArgument(
        "jump_takeoff_pitch_rate", default_value="0.05", description="Forward pitch rate target during thrust"
    )
    thrust_wheel_max_decel_arg = DeclareLaunchArgument(
        "thrust_wheel_max_decel", default_value=jump_default("thrust_wheel_max_decel", 8.0), description="Max deceleration rate limit for thrust ground wheel control (m/s^2)"
    )
    adaptive_experiment_mode_arg = DeclareLaunchArgument(
        "adaptive_experiment_mode",
        default_value="adaptive",
        description="Adaptive LQR experiment mode: nominal, oracle, or adaptive",
    )
    adaptive_gain_mode_arg = DeclareLaunchArgument(
        "adaptive_gain_mode",
        default_value="scheduled",
        description="Gain selection: scheduled (5-node interpolation) or fixed_midpoint (H=0.40 m ablation)",
    )
    adaptive_gain_profile_arg = DeclareLaunchArgument(
        "adaptive_gain_profile",
        default_value="legacy_safe",
        description="Gain profile: legacy_safe (default) or optimized_v2",
    )
    adaptive_com_y_bias_arg = DeclareLaunchArgument(
        "adaptive_com_y_bias",
        default_value="0.0",
        description="Injected controller-model COM-y bias in metres",
    )
    adaptive_log_path_arg = DeclareLaunchArgument(
        "adaptive_log_path",
        default_value=f"{ws_dir}/src/bbot_balance_controller/src/data_logs/adaptive_lqr_log.csv",
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
    adaptive_leg_transition_speed_arg = DeclareLaunchArgument(
        "adaptive_leg_transition_speed",
        default_value="0.05",
        description="Maximum hip-axle height transition speed for Adaptive/GS-LQR in m/s",
    )
    formal_controller_start_delay_arg = DeclareLaunchArgument(
        "formal_controller_start_delay",
        default_value="0.7",
        description="Wall-clock delay before the formal PID/GS-LQR controller starts",
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
    historical_gs_lqr_config_file_arg = DeclareLaunchArgument(
        "historical_gs_lqr_config_file",
        default_value=f"{ws_dir}/src/bbot_balance_controller/config/gs_lqr_historical_experiment.yaml",
        description="Independent nominal legacy-safe GS-LQR formal-comparison configuration",
    )
    historical_gs_lqr_gain_mode_arg = DeclareLaunchArgument(
        "historical_gs_lqr_gain_mode",
        default_value="scheduled",
        description="Gain mode for formal comparison: scheduled or fixed_midpoint",
    )
    enable_position_handoff_arg = DeclareLaunchArgument(
        "enable_position_handoff", default_value="true"
    )
    payload_mass_arg = DeclareLaunchArgument(
        "payload_mass", default_value="0.0", description="Mass of physical payload in kg"
    )
    payload_x_arg = DeclareLaunchArgument(
        "payload_x", default_value="0.200", description="Payload X coordinate in base_link"
    )
    payload_y_offset_arg = DeclareLaunchArgument(
        "payload_y_offset", default_value="0.0", description="Payload Y offset relative to hip center (0.125m) in m"
    )
    payload_z_arg = DeclareLaunchArgument(
        "payload_z", default_value="0.170", description="Payload Z coordinate in base_link"
    )
    payload_size_x_arg = DeclareLaunchArgument(
        "payload_size_x", default_value="0.12", description="Payload X dimension in m"
    )
    payload_size_y_arg = DeclareLaunchArgument(
        "payload_size_y", default_value="0.08", description="Payload Y dimension in m"
    )
    payload_size_z_arg = DeclareLaunchArgument(
        "payload_size_z", default_value="0.02", description="Payload Z dimension in m"
    )

    # Torque Cascade PID arguments
    torque_pid_gains_file_arg = DeclareLaunchArgument(
        "torque_pid_gains_file",
        default_value=f"{ws_dir}/src/bbot_balance_controller/config/torque_cascade_pid_gains.yaml",
        description="Path to YAML file with torque cascade PID gains",
    )
    torque_pid_log_path_arg = DeclareLaunchArgument(
        "torque_pid_log_path",
        default_value=f"{ws_dir}/src/bbot_balance_controller/src/data_logs/torque_pid_log.csv",
    )
    torque_pid_stage_mode_arg = DeclareLaunchArgument(
        "torque_pid_stage_mode", default_value="normal"
    )
    torque_pid_target_height_arg = DeclareLaunchArgument(
        "torque_pid_target_height", default_value="0.50"
    )
    torque_pid_startup_height_arg = DeclareLaunchArgument(
        "torque_pid_startup_height", default_value="0.36"
    )
    torque_pid_initial_roll_arg = DeclareLaunchArgument(
        "torque_pid_initial_roll", default_value="-0.038",
        description="Initial Gazebo roll aligning COM with wheel contact axis to prevent gravity toppling before controllers activate",
    )
    torque_pid_k_x_arg = DeclareLaunchArgument(
        "torque_pid_k_x", default_value="0.0",
        description="Legacy compatibility parameter; position feedback is forced off for the PID baseline",
    )
    torque_pid_rate_limit_u_arg = DeclareLaunchArgument(
        "torque_pid_rate_limit_u", default_value="0.0",
        description="Effective total torque slew-rate limit in N m/s; <=0 disables it",
    )
    torque_pid_total_torque_max_arg = DeclareLaunchArgument(
        "torque_pid_total_torque_max", default_value="20.0",
    )
    torque_pid_wheel_torque_max_arg = DeclareLaunchArgument(
        "torque_pid_wheel_torque_max", default_value="10.0",
    )
    torque_pid_low_kp_rate_arg = DeclareLaunchArgument("torque_pid_low_kp_rate", default_value="20.0")
    torque_pid_low_kd_rate_arg = DeclareLaunchArgument("torque_pid_low_kd_rate", default_value="0.02")
    torque_pid_low_kp_theta_arg = DeclareLaunchArgument("torque_pid_low_kp_theta", default_value="5.5")
    torque_pid_low_kd_theta_arg = DeclareLaunchArgument("torque_pid_low_kd_theta", default_value="0.10")
    torque_pid_low_kp_v_arg = DeclareLaunchArgument("torque_pid_low_kp_v", default_value="0.08")
    torque_pid_low_ki_v_arg = DeclareLaunchArgument("torque_pid_low_ki_v", default_value="0.008")
    torque_pid_low_kd_v_arg = DeclareLaunchArgument("torque_pid_low_kd_v", default_value="0.001")
    torque_pid_high_kp_rate_arg = DeclareLaunchArgument("torque_pid_high_kp_rate", default_value="22.0")
    torque_pid_high_kd_rate_arg = DeclareLaunchArgument("torque_pid_high_kd_rate", default_value="0.025")
    torque_pid_high_kp_theta_arg = DeclareLaunchArgument("torque_pid_high_kp_theta", default_value="6.0")
    torque_pid_high_kd_theta_arg = DeclareLaunchArgument("torque_pid_high_kd_theta", default_value="0.12")
    torque_pid_high_kp_v_arg = DeclareLaunchArgument("torque_pid_high_kp_v", default_value="0.09")
    torque_pid_high_ki_v_arg = DeclareLaunchArgument("torque_pid_high_ki_v", default_value="0.008")
    torque_pid_high_kd_v_arg = DeclareLaunchArgument("torque_pid_high_kd_v", default_value="0.001")
    torque_pid_attitude_disturbance_step_arg = DeclareLaunchArgument("torque_pid_attitude_disturbance_step", default_value="0.0")
    torque_pid_rate_disturbance_step_arg = DeclareLaunchArgument("torque_pid_rate_disturbance_step", default_value="0.0")
    torque_pid_velocity_disturbance_step_arg = DeclareLaunchArgument("torque_pid_velocity_disturbance_step", default_value="0.0")
    torque_pid_disturbance_start_time_arg = DeclareLaunchArgument("torque_pid_disturbance_start_time", default_value="2.0")
    torque_pid_rate_disturbance_start_time_arg = DeclareLaunchArgument("torque_pid_rate_disturbance_start_time", default_value="0.6")
    torque_pid_velocity_disturbance_start_time_arg = DeclareLaunchArgument("torque_pid_velocity_disturbance_start_time", default_value="2.5")
    torque_pid_startup_hold_time_arg = DeclareLaunchArgument("torque_pid_startup_hold_time", default_value="1.0")
    torque_pid_leg_transition_speed_arg = DeclareLaunchArgument("torque_pid_leg_transition_speed", default_value="0.10")

    # Independent four-loop position -> velocity -> attitude -> rate PID
    position_pid_gains_file_arg = DeclareLaunchArgument(
        "position_pid_gains_file",
        default_value=f"{ws_dir}/src/bbot_balance_controller/config/position_torque_cascade_pid_gains.yaml",
        description="Parameter file for the independent four-loop exploratory PID",
    )
    position_pid_log_path_arg = DeclareLaunchArgument(
        "position_pid_log_path",
        default_value=f"{ws_dir}/src/bbot_balance_controller/src/data_logs/position_torque_pid_exploration/position_pid_log.csv",
    )
    position_pid_target_height_arg = DeclareLaunchArgument("position_pid_target_height", default_value="0.50")
    position_pid_startup_height_arg = DeclareLaunchArgument("position_pid_startup_height", default_value="0.36")
    position_pid_startup_hold_time_arg = DeclareLaunchArgument("position_pid_startup_hold_time", default_value="1.0")
    position_pid_leg_transition_speed_arg = DeclareLaunchArgument("position_pid_leg_transition_speed", default_value="0.05")
    position_pid_kp_arg = DeclareLaunchArgument("position_pid_kp", default_value="0.50")
    position_pid_ki_arg = DeclareLaunchArgument("position_pid_ki", default_value="0.010")
    position_pid_kd_arg = DeclareLaunchArgument("position_pid_kd", default_value="0.30")
    position_pid_v_ref_limit_arg = DeclareLaunchArgument("position_pid_v_ref_limit", default_value="0.40")
    position_pid_integral_limit_arg = DeclareLaunchArgument("position_pid_integral_limit", default_value="2.0")
    position_pid_rate_limit_u_arg = DeclareLaunchArgument("position_pid_rate_limit_u", default_value="0.0")
    position_pid_total_torque_max_arg = DeclareLaunchArgument("position_pid_total_torque_max", default_value="20.0")
    position_pid_wheel_torque_max_arg = DeclareLaunchArgument("position_pid_wheel_torque_max", default_value="10.0")

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

    # The passive contact-frame recorder is installed in this package's lib
    # directory. Keep every existing Gazebo plugin path and prepend the
    # package-local directory instead of replacing user/system entries.
    bringup_lib = os.path.join(get_package_prefix("bbot_bringup"), "lib")
    for variable in ("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                     "GZ_SIM_SYSTEM_PLUGIN_PATH", "LD_LIBRARY_PATH"):
        existing = os.environ.get(variable, "")
        entries = [entry for entry in existing.split(":") if entry]
        if bringup_lib not in entries:
            os.environ[variable] = ":".join([bringup_lib] + entries)

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
                " payload_mass:=",
                LaunchConfiguration("payload_mass"),
                " payload_x:=",
                LaunchConfiguration("payload_x"),
                " payload_y_offset:=",
                LaunchConfiguration("payload_y_offset"),
                " payload_z:=",
                LaunchConfiguration("payload_z"),
                " payload_size_x:=",
                LaunchConfiguration("payload_size_x"),
                " payload_size_y:=",
                LaunchConfiguration("payload_size_y"),
                " payload_size_z:=",
                LaunchConfiguration("payload_size_z"),
            ]
        ),
        value_type=str,
    )

    # 1. 启动 Gazebo
    # - 默认启动带 3D 渲染画面的图形界面 GUI (headless=false, gui=true)
    # - 当 headless:=true 或 gui:=false 时以无图形服务器模式运行 (-s)
    # - 当 gazebo_start_paused:=false 时直接运行仿真 (-r)
    is_headless = PythonExpression([
        "('true' if '", headless, "'.lower() in ['true', '1'] or '", gui, "'.lower() in ['false', '0'] else 'false')"
    ])
    gz_args_expr = PythonExpression([
        "('-s ' if '", is_headless, "' == 'true' else '') + "
        "('' if '", gazebo_start_paused, "'.lower() in ['true', '1'] else '-r ') + "
            "'", LaunchConfiguration("effective_world", default=world), "' + "
        "((' --record-path ' + '", gazebo_record_path, "') if '",
        gazebo_record_path, "' else '')"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args_expr}.items(),
    )

    world_control_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            PythonExpression([
                '"/world/" + "', gazebo_world_name,
                '" + "/control@ros_gz_interfaces/srv/ControlWorld"',
            ])
        ],
        output="screen",
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
            LaunchConfiguration("mpc_spawn_z"),
            "-R",
            LaunchConfiguration("torque_pid_initial_roll"),
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
            PythonExpression([
                '"/world/" + "', gazebo_world_name,
                '" + "/model/bbot/link/link_004/sensor/left_wheel_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"',
            ]),
            PythonExpression([
                '"/world/" + "', gazebo_world_name,
                '" + "/model/bbot/link/link_007/sensor/right_wheel_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"',
            ]),
            PythonExpression([
                '"/world/" + "', gazebo_world_name,
                '" + "/model/ground_plane/link/link/sensor/ground_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"',
            ]),
            PythonExpression([
                '"/world/" + "', gazebo_world_name,
                '" + "/ground_contact_frames@std_msgs/msg/String[ignition.msgs.StringMsg"',
            ]),
        ],
        remappings=[
            (
                PythonExpression([
                    '"/world/" + "', gazebo_world_name,
                    '" + "/model/bbot/link/link_004/sensor/left_wheel_contact/contact"',
                ]),
                "/model/bbot/left_wheel_contact",
            ),
            (
                PythonExpression([
                    '"/world/" + "', gazebo_world_name,
                    '" + "/model/bbot/link/link_007/sensor/right_wheel_contact/contact"',
                ]),
                "/model/bbot/right_wheel_contact",
            ),
            (
                PythonExpression([
                    '"/world/" + "', gazebo_world_name,
                    '" + "/model/ground_plane/link/link/sensor/ground_contact/contact"',
                ]),
                "/model/bbot/ground_contact",
            ),
        ],
        output="screen",
    )

    # 5. 加载 ROS 2 控制器 (紧跟 controller_manager 就绪后加载，大幅缩短无控窗口)
    load_joint_state_broadcaster = TimerAction(
        period=0.4,
        condition=IfCondition(PythonExpression([
            "'", staged_controller_startup, "'.lower() not in ['true', '1'] and '",
            controller_type, "'.lower() != 'gs_lqr'"
        ])),
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
        period=0.5,
        # GS-LQR writes wheel *effort* commands directly.  The differential
        # drive controller claims the wheel velocity interfaces, so it must
        # not be active in that mode.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    controller_type,
                    "'.lower() not in ['gs_lqr', 'mpc', 'adaptive_lqr', 'gs_lqr_historical', 'torque_cascade_pid', 'torque_pid', 'position_torque_cascade_pid']",
                    " and not '",
                    staged_controller_startup,
                    "'.lower() in ['true', '1']",
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
        period=0.6,
        condition=IfCondition(PythonExpression([
            "'", staged_controller_startup, "'.lower() not in ['true', '1'] and '",
            controller_type, "'.lower() != 'gs_lqr'"
        ])),
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
        period=0.8,
        condition=IfCondition(PythonExpression([
            "'", staged_controller_startup, "'.lower() not in ['true', '1'] and '",
            controller_type, "'.lower() != 'gs_lqr'"
        ])),
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

    # Deterministic startup for automated jump trials: configuration can
    # complete while physics is paused; the runner then unpauses and activates
    # the three ground controllers in one switch request.
    load_staged_jump_controllers = TimerAction(
        period=0.4,
        condition=IfCondition(staged_controller_startup),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "diff_drive_controller",
                    "leg_position_controller",
                    "leg_effort_controller",
                    "--inactive",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    load_gs_lqr_controllers = TimerAction(
        period=0.4,
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower() == 'gs_lqr' and '",
            staged_controller_startup, "'.lower() not in ['true', '1']"
        ])),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "leg_position_controller",
                    "wheel_effort_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    load_wheel_effort_controller = TimerAction(
        # GS-LQR loads this with its other required controllers above.
        period=0.5,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    controller_type,
                    "'.lower() in ['mpc', 'adaptive_lqr', 'gs_lqr_historical', 'torque_cascade_pid', 'torque_pid', 'position_torque_cascade_pid']",
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

    # 6. 平衡控制器 (按 controller_type 参数选择启动: 'pid' 或 'lqr'，0.7s 即刻介入接管)
    start_pid_controller = TimerAction(
        period=0.7,
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
        period=0.7,
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
        period=0.7,
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

    start_mpc_controller = TimerAction(
        period=0.7,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'mpc'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="linear_mpc_balance_controller",
                output="screen",
                parameters=[
                    # Checked-in YAML first, launch arguments second, so a
                    # command-line override wins over the file.
                    LaunchConfiguration("mpc_config_file"),
                    {
                        "use_sim_time": True,
                        "target_height": ParameterValue(
                            LaunchConfiguration("mpc_target_height"), value_type=float
                        ),
                        "mpc.horizon": ParameterValue(
                            LaunchConfiguration("mpc_horizon"), value_type=int
                        ),
                        "mpc.r": ParameterValue(LaunchConfiguration("mpc_r"), value_type=float),
                        "mpc.theta_error_limit": ParameterValue(
                            LaunchConfiguration("mpc_theta_error_limit"), value_type=float
                        ),
                        "mpc.use_theta_band": ParameterValue(
                            LaunchConfiguration("mpc_use_theta_band"), value_type=bool
                        ),
                        "mpc.wheel_torque_max": ParameterValue(
                            LaunchConfiguration("mpc_wheel_torque_max"), value_type=float
                        ),
                        "mpc.total_torque_max": ParameterValue(
                            LaunchConfiguration("mpc_total_torque_max"), value_type=float
                        ),
                        "mpc.theta_eq_source": LaunchConfiguration("mpc_theta_eq_source"),
                        "mpc.theta_eq": ParameterValue(
                            LaunchConfiguration("mpc_theta_eq"), value_type=float
                        ),
                        "height.startup_hold_time": ParameterValue(
                            LaunchConfiguration("mpc_startup_hold_time"), value_type=float
                        ),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("mpc_startup_height"), value_type=float
                        ),
                        "leg_transition_speed": ParameterValue(
                            LaunchConfiguration("mpc_leg_transition_speed"), value_type=float
                        ),
                        "log_path": LaunchConfiguration("mpc_log_path"),
                    },
                ],
            )
        ],
    )

    start_adaptive_lqr_controller = TimerAction(
        period=0.7,
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
                        "gain.mode": LaunchConfiguration(
                            "adaptive_gain_mode"
                        ),
                        "gain.profile": LaunchConfiguration(
                            "adaptive_gain_profile"
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
                        "leg_transition_speed": ParameterValue(
                            LaunchConfiguration("adaptive_leg_transition_speed"),
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

    # Historical GS-LQR used only by the formal Sec. 4.2 comparison.  It is a
    # separate launch type so the normal adaptive_lqr entry point is unchanged.
    start_historical_gs_lqr_controller = TimerAction(
        period=LaunchConfiguration("formal_controller_start_delay"),
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'gs_lqr_historical'"])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="adaptive_lqr_balance_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("historical_gs_lqr_config_file"),
                    {
                        "use_sim_time": True,
                        "experiment.mode": "nominal",
                        "gain.mode": LaunchConfiguration("historical_gs_lqr_gain_mode"),
                        "gain.profile": "legacy_safe",
                        "experiment.com_y_bias": 0.0,
                        "target_height": ParameterValue(
                            LaunchConfiguration("adaptive_target_height"), value_type=float),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("adaptive_startup_height"), value_type=float),
                        "leg_transition_speed": ParameterValue(
                            LaunchConfiguration("adaptive_leg_transition_speed"), value_type=float),
                        "adaptation.apply_rate_max": ParameterValue(
                            LaunchConfiguration("adaptive_apply_rate_max"), value_type=float),
                        "adaptation.two_stage_enabled": False,
                        "log_path": LaunchConfiguration("adaptive_log_path"),
                    },
                ],
            )
        ],
    )

    start_jump_controller = TimerAction(
        period=0.7,
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
        period=0.7,
        condition=IfCondition(
            PythonExpression(["'", controller_type, "'.lower() == 'jump_velocity'"])
        ),
        actions=[
            Node(
                executable=f"{ws_dir}/build_native_allocator/bbot_balance_controller/bbot_landing_repair_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "sim_imu_reference_roll": ParameterValue(
                            LaunchConfiguration("torque_pid_initial_roll"),
                            value_type=float,
                        ),
                        "thrust_support_allocator": True,
                        "native_command_state_diagnostics": True,
                        "wheel_servo_gain": 1.0,
                        "allocator_target_H": -0.08,
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
                        "air_wheel_extend_kd": ParameterValue(
                            LaunchConfiguration("air_wheel_extend_kd"),
                            value_type=float,
                        ),
                        "air_wheel_tuck_kd": ParameterValue(
                            LaunchConfiguration("air_wheel_tuck_kd"),
                            value_type=float,
                        ),
                        "flight_tuck_nominal_duration": ParameterValue(
                            LaunchConfiguration("flight_tuck_nominal_duration"),
                            value_type=float,
                        ),
                        "flight_arrest_freewheel_duration": ParameterValue(
                            LaunchConfiguration("flight_arrest_freewheel_duration"),
                            value_type=float,
                        ),
                        "thrust_terminal_hip_floor": ParameterValue(
                            LaunchConfiguration("thrust_terminal_hip_floor"),
                            value_type=bool,
                        ),
                        "thrust_wheel_max_decel": ParameterValue(
                            LaunchConfiguration("thrust_wheel_max_decel"),
                            value_type=float,
                        ),
                        "landing_buffer_height": ParameterValue(
                            LaunchConfiguration("landing_buffer_height"),
                            value_type=float,
                        ),
                        "position_proportional_gain": LaunchConfiguration(
                            "position_proportional_gain"
                        ),
                        "body_mass": LaunchConfiguration("body_mass"),
                        "enable_position_handoff": LaunchConfiguration(
                            "enable_position_handoff"
                        ),
                        "target_height": ParameterValue(
                            LaunchConfiguration("adaptive_target_height"),
                            value_type=float,
                        ),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("adaptive_startup_height"),
                            value_type=float,
                        ),
                        "jump_log_path": LaunchConfiguration("jump_log_path"),
                        "jump_summary_path": LaunchConfiguration("jump_summary_path"),
                        "auto_return_balance": ParameterValue(
                            LaunchConfiguration("auto_return_balance"),
                            value_type=bool,
                        ),
                        "jump_forward_speed": ParameterValue(
                            LaunchConfiguration("jump_forward_speed"),
                            value_type=float,
                        ),
                        "jump_takeoff_forward_speed": ParameterValue(
                            LaunchConfiguration("jump_takeoff_forward_speed"),
                            value_type=float,
                        ),
                        "thrust_forward_velocity_kp": ParameterValue(
                            LaunchConfiguration("thrust_forward_velocity_kp"),
                            value_type=float,
                        ),
                        "thrust_forward_attitude_taper": ParameterValue(
                            LaunchConfiguration("thrust_forward_attitude_taper"),
                            value_type=bool,
                        ),
                        "thrust_release_fast_rate_correction": ParameterValue(
                            LaunchConfiguration("thrust_release_fast_rate_correction"),
                            value_type=bool,
                        ),
                        "thrust_fast_rate_correction_from_gate": ParameterValue(
                            LaunchConfiguration("thrust_fast_rate_correction_from_gate"),
                            value_type=bool,
                        ),
                        "thrust_reference_handoff": ParameterValue(
                            LaunchConfiguration("thrust_reference_handoff"), value_type=bool,
                        ),
                        "thrust_momentum_reference": ParameterValue(
                            LaunchConfiguration("thrust_momentum_reference"), value_type=bool,
                        ),
                        "thrust_support_coordination": ParameterValue(
                            LaunchConfiguration("thrust_support_coordination"), value_type=bool,
                        ),
                        "thrust_release_velocity_ratio": ParameterValue(
                            LaunchConfiguration("thrust_release_velocity_ratio"),
                            value_type=float,
                        ),
                        "thrust_forward_speed_prediction": ParameterValue(
                            LaunchConfiguration("thrust_forward_speed_prediction"),
                            value_type=bool,
                        ),
                        "thrust_wheel_kinematics_compensation": ParameterValue(
                            LaunchConfiguration("thrust_wheel_kinematics_compensation"),
                            value_type=bool,
                        ),
                        "complete_contact_takeoff_confirmation": ParameterValue(
                            LaunchConfiguration("complete_contact_takeoff_confirmation"),
                            value_type=bool,
                        ),
                        "arrest_dynamics_feedforward": ParameterValue(
                            LaunchConfiguration("arrest_dynamics_feedforward"),
                            value_type=bool,
                        ),
                        "jump_pitch_offset": ParameterValue(
                            LaunchConfiguration("jump_pitch_offset"),
                            value_type=float,
                        ),
                        "jump_takeoff_pitch_rate": ParameterValue(
                            LaunchConfiguration("jump_takeoff_pitch_rate"),
                            value_type=float,
                        ),
                    }
                ],
            )
        ],
    )

    start_torque_cascade_pid_controller = TimerAction(
        period=0.7,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    controller_type,
                    "'.lower() in ['torque_cascade_pid', 'torque_pid']",
                ]
            )
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="torque_cascade_pid_controller",
                output="screen",
                parameters=[
                    # Load the checked-in/effective file first.  Explicit
                    # launch arguments below are candidate overrides during
                    # tuning and therefore must win over the file.
                    LaunchConfiguration("torque_pid_gains_file"),
                    {
                        "use_sim_time": True,
                        "log_path": LaunchConfiguration("torque_pid_log_path"),
                        "stage_mode": LaunchConfiguration("torque_pid_stage_mode"),
                        "target_height": ParameterValue(
                            LaunchConfiguration("torque_pid_target_height"),
                            value_type=float,
                        ),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("torque_pid_startup_height"),
                            value_type=float,
                        ),
                        "height.startup_hold_time": ParameterValue(
                            LaunchConfiguration("torque_pid_startup_hold_time"),
                            value_type=float,
                        ),
                        "leg_transition_speed": ParameterValue(
                            LaunchConfiguration("torque_pid_leg_transition_speed"),
                            value_type=float,
                        ),
                        "pid.k_x": ParameterValue(
                            LaunchConfiguration("torque_pid_k_x"), value_type=float
                        ),
                        "pid.rate_limit_u": ParameterValue(
                            LaunchConfiguration("torque_pid_rate_limit_u"), value_type=float
                        ),
                        "pid.total_torque_max": ParameterValue(
                            LaunchConfiguration("torque_pid_total_torque_max"), value_type=float
                        ),
                        "pid.wheel_torque_max": ParameterValue(
                            LaunchConfiguration("torque_pid_wheel_torque_max"), value_type=float
                        ),
                        "pid.attitude_disturbance_step": ParameterValue(
                            LaunchConfiguration("torque_pid_attitude_disturbance_step"), value_type=float
                        ),
                        "pid.rate_disturbance_step": ParameterValue(
                            LaunchConfiguration("torque_pid_rate_disturbance_step"),
                            value_type=float,
                        ),
                        "pid.velocity_disturbance_step": ParameterValue(
                            LaunchConfiguration("torque_pid_velocity_disturbance_step"),
                            value_type=float,
                        ),
                        "pid.disturbance_step_start_time": ParameterValue(
                            LaunchConfiguration("torque_pid_disturbance_start_time"),
                            value_type=float,
                        ),
                        "pid.rate_disturbance_start_time": ParameterValue(
                            LaunchConfiguration("torque_pid_rate_disturbance_start_time"),
                            value_type=float,
                        ),
                        "pid.velocity_disturbance_start_time": ParameterValue(
                            LaunchConfiguration("torque_pid_velocity_disturbance_start_time"),
                            value_type=float,
                        ),
                        "pid.low.kp_rate": ParameterValue(LaunchConfiguration("torque_pid_low_kp_rate"), value_type=float),
                        "pid.low.kd_rate": ParameterValue(LaunchConfiguration("torque_pid_low_kd_rate"), value_type=float),
                        "pid.low.kp_theta": ParameterValue(LaunchConfiguration("torque_pid_low_kp_theta"), value_type=float),
                        "pid.low.kd_theta": ParameterValue(LaunchConfiguration("torque_pid_low_kd_theta"), value_type=float),
                        "pid.low.kp_v": ParameterValue(LaunchConfiguration("torque_pid_low_kp_v"), value_type=float),
                        "pid.low.ki_v": ParameterValue(LaunchConfiguration("torque_pid_low_ki_v"), value_type=float),
                        "pid.low.kd_v": ParameterValue(LaunchConfiguration("torque_pid_low_kd_v"), value_type=float),
                        "pid.high.kp_rate": ParameterValue(LaunchConfiguration("torque_pid_high_kp_rate"), value_type=float),
                        "pid.high.kd_rate": ParameterValue(LaunchConfiguration("torque_pid_high_kd_rate"), value_type=float),
                        "pid.high.kp_theta": ParameterValue(LaunchConfiguration("torque_pid_high_kp_theta"), value_type=float),
                        "pid.high.kd_theta": ParameterValue(LaunchConfiguration("torque_pid_high_kd_theta"), value_type=float),
                        "pid.high.kp_v": ParameterValue(LaunchConfiguration("torque_pid_high_kp_v"), value_type=float),
                        "pid.high.ki_v": ParameterValue(LaunchConfiguration("torque_pid_high_ki_v"), value_type=float),
                        "pid.high.kd_v": ParameterValue(LaunchConfiguration("torque_pid_high_kd_v"), value_type=float),
                    },
                ],
            )
        ],
    )

    start_position_torque_cascade_pid_controller = TimerAction(
        period=LaunchConfiguration("formal_controller_start_delay"),
        condition=IfCondition(
            PythonExpression([
                "'", controller_type,
                "'.lower() == 'position_torque_cascade_pid'",
            ])
        ),
        actions=[
            Node(
                package="bbot_balance_controller",
                executable="position_torque_cascade_pid_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("position_pid_gains_file"),
                    {
                        "use_sim_time": True,
                        "log_path": LaunchConfiguration("position_pid_log_path"),
                        "target_height": ParameterValue(
                            LaunchConfiguration("position_pid_target_height"), value_type=float),
                        "height.startup_hip_axle": ParameterValue(
                            LaunchConfiguration("position_pid_startup_height"), value_type=float),
                        "height.startup_hold_time": ParameterValue(
                            LaunchConfiguration("position_pid_startup_hold_time"), value_type=float),
                        "leg_transition_speed": ParameterValue(
                            LaunchConfiguration("position_pid_leg_transition_speed"), value_type=float),
                        "pid.position.kp": ParameterValue(
                            LaunchConfiguration("position_pid_kp"), value_type=float),
                        "pid.position.ki": ParameterValue(
                            LaunchConfiguration("position_pid_ki"), value_type=float),
                        "pid.position.kd": ParameterValue(
                            LaunchConfiguration("position_pid_kd"), value_type=float),
                        "pid.position.v_ref_limit": ParameterValue(
                            LaunchConfiguration("position_pid_v_ref_limit"), value_type=float),
                        "pid.position.integral_limit": ParameterValue(
                            LaunchConfiguration("position_pid_integral_limit"), value_type=float),
                        "pid.rate_limit_u": ParameterValue(
                            LaunchConfiguration("position_pid_rate_limit_u"), value_type=float),
                        "pid.total_torque_max": ParameterValue(
                            LaunchConfiguration("position_pid_total_torque_max"), value_type=float),
                        "pid.wheel_torque_max": ParameterValue(
                            LaunchConfiguration("position_pid_wheel_torque_max"), value_type=float),
                    },
                ],
            )
        ],
    )

    auto_unpause_action = TimerAction(
        period=2.8,
        condition=IfCondition(
            PythonExpression([
                "'", gazebo_start_paused, "'.lower() in ['true', '1'] and '",
                auto_unpause, "'.lower() in ['true', '1'] and '",
                controller_type, "'.lower() not in ['gs_lqr', 'mpc', 'jump_velocity']"
            ])
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign", "service", "-s",
                    PythonExpression(['"/world/" + "', gazebo_world_name, '" + "/control"']),
                    "--reqtype", "ignition.msgs.WorldControl",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", "pause: false",
                ],
                output="screen",
            )
        ],
    )

    jump_velocity_startup_gate = ExecuteProcess(
        cmd=[
            "ros2", "run", "bbot_balance_controller", "jump_startup_gate.py",
            "--world", gazebo_world_name, "--wheel-actuation", "velocity",
            "--timeout", "20", "--max-physics-seconds", "0.100",
        ],
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower() == 'jump_velocity' and '",
            gazebo_start_paused, "'.lower() in ['true', '1'] and '",
            auto_unpause, "'.lower() in ['true', '1'] and '",
            staged_controller_startup, "'.lower() not in ['true', '1']"
        ])),
        output="screen",
    )

    staged_jump_velocity_startup_gate = ExecuteProcess(
        cmd=[
            "ros2", "run", "bbot_balance_controller", "jump_startup_gate.py",
            "--world", gazebo_world_name, "--wheel-actuation", "velocity",
            "--activate-staged", "--timeout", "20",
            "--max-physics-seconds", "0.100",
        ],
        condition=IfCondition(PythonExpression([
            "'", controller_type, "'.lower() == 'jump_velocity' and '",
            gazebo_start_paused, "'.lower() in ['true', '1'] and '",
            auto_unpause, "'.lower() in ['true', '1'] and '",
            staged_controller_startup, "'.lower() in ['true', '1']"
        ])),
        output="screen",
    )

    gs_lqr_ready_unpause = TimerAction(
        period=0.8,
        condition=IfCondition(
            PythonExpression([
                "'", controller_type, "'.lower() in ['gs_lqr', 'mpc'] and '",
                gazebo_start_paused, "'.lower() in ['true', '1'] and '",
                auto_unpause, "'.lower() in ['true', '1'] and '",
                staged_controller_startup, "'.lower() not in ['true', '1']"
            ])
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution([
                        FindPackageShare("bbot_bringup"), "launch", "gs_lqr_ready_unpause.py"
                    ]),
                    "--world", gazebo_world_name,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            controller_type_arg,
            allocator_runner_guard,
            jump_controller_executable_arg,
            mpc_config_file_arg,
            mpc_target_height_arg,
            mpc_horizon_arg,
            mpc_r_arg,
            mpc_theta_error_limit_arg,
            mpc_use_theta_band_arg,
            mpc_wheel_torque_max_arg,
            mpc_total_torque_max_arg,
            mpc_theta_eq_source_arg,
            mpc_startup_hold_time_arg,
            mpc_startup_height_arg,
            mpc_leg_transition_speed_arg,
            mpc_theta_eq_arg,
            mpc_spawn_z_arg,
            mpc_log_path_arg,
            world_arg,
            gazebo_world_name_arg,
            gazebo_start_paused_arg,
            auto_unpause_arg,
            staged_controller_startup_arg,
            headless_arg,
            gui_arg,
            gazebo_record_path_arg,
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
            air_wheel_extend_kd_arg,
            thrust_wheel_max_decel_arg,
            air_wheel_tuck_kd_arg,
            flight_tuck_nominal_duration_arg,
            flight_arrest_freewheel_duration_arg,
            thrust_terminal_hip_floor_arg,
            landing_buffer_height_arg,
            jump_log_path_arg,
            jump_summary_path_arg,
            auto_return_balance_arg,
            jump_forward_speed_arg,
            jump_takeoff_forward_speed_arg,
            thrust_forward_velocity_kp_arg,
            thrust_forward_attitude_taper_arg,
            thrust_release_fast_rate_correction_arg,
            thrust_fast_rate_correction_from_gate_arg,
            thrust_momentum_reference_arg,
            thrust_reference_handoff_arg,
            thrust_support_coordination_arg,
            thrust_release_velocity_ratio_arg,
            thrust_forward_speed_prediction_arg,
            thrust_wheel_kinematics_compensation_arg,
            complete_contact_takeoff_confirmation_arg,
            arrest_dynamics_feedforward_arg,
            jump_pitch_offset_arg,
            jump_takeoff_pitch_rate_arg,
            position_proportional_gain_arg,
            body_mass_arg,
            adaptive_experiment_mode_arg,
            adaptive_gain_mode_arg,
            adaptive_gain_profile_arg,
            adaptive_com_y_bias_arg,
            adaptive_log_path_arg,
            adaptive_target_height_arg,
            adaptive_startup_height_arg,
            adaptive_leg_transition_speed_arg,
            formal_controller_start_delay_arg,
            adaptive_apply_rate_max_arg,
            adaptive_two_stage_enabled_arg,
            historical_gs_lqr_config_file_arg,
            historical_gs_lqr_gain_mode_arg,
            enable_position_handoff_arg,
            payload_mass_arg,
            payload_x_arg,
            payload_y_offset_arg,
            payload_z_arg,
            payload_size_x_arg,
            payload_size_y_arg,
            payload_size_z_arg,
            torque_pid_gains_file_arg,
            torque_pid_log_path_arg,
            torque_pid_stage_mode_arg,
            torque_pid_target_height_arg,
            torque_pid_startup_height_arg,
            torque_pid_initial_roll_arg,
            torque_pid_k_x_arg,
            torque_pid_rate_limit_u_arg,
            torque_pid_total_torque_max_arg,
            torque_pid_wheel_torque_max_arg,
            torque_pid_low_kp_rate_arg,
            torque_pid_low_kd_rate_arg,
            torque_pid_low_kp_theta_arg,
            torque_pid_low_kd_theta_arg,
            torque_pid_low_kp_v_arg,
            torque_pid_low_ki_v_arg,
            torque_pid_low_kd_v_arg,
            torque_pid_high_kp_rate_arg,
            torque_pid_high_kd_rate_arg,
            torque_pid_high_kp_theta_arg,
            torque_pid_high_kd_theta_arg,
            torque_pid_high_kp_v_arg,
            torque_pid_high_ki_v_arg,
            torque_pid_high_kd_v_arg,
            torque_pid_attitude_disturbance_step_arg,
            torque_pid_rate_disturbance_step_arg,
            torque_pid_velocity_disturbance_step_arg,
            torque_pid_disturbance_start_time_arg,
            torque_pid_rate_disturbance_start_time_arg,
            torque_pid_velocity_disturbance_start_time_arg,
            torque_pid_startup_hold_time_arg,
            torque_pid_leg_transition_speed_arg,
            position_pid_gains_file_arg,
            position_pid_log_path_arg,
            position_pid_target_height_arg,
            position_pid_startup_height_arg,
            position_pid_startup_hold_time_arg,
            position_pid_leg_transition_speed_arg,
            position_pid_kp_arg,
            position_pid_ki_arg,
            position_pid_kd_arg,
            position_pid_v_ref_limit_arg,
            position_pid_integral_limit_arg,
            position_pid_rate_limit_u_arg,
            position_pid_total_torque_max_arg,
            position_pid_wheel_torque_max_arg,
            OpaqueFunction(function=validate_fast_rate_launch),
            real_time_factor_arg,
            OpaqueFunction(function=apply_explicit_world_rtf),
            SetEnvironmentVariable("BBOT_NATIVE_COMMAND_STATE_TOPIC", "/world/flat_jump_world/native_command_state"),
            gazebo,
            world_control_bridge,
            Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                 arguments=["/world/flat_jump_world/native_command_state@std_msgs/msg/String[ignition.msgs.StringMsg"]),
            robot_state_publisher,
            spawn_robot,
            imu_clock_bridge,
            load_joint_state_broadcaster,
            load_diff_drive_controller,
            load_leg_controller,
            load_leg_effort_controller,
            load_staged_jump_controllers,
            start_pid_controller,
            start_lqr_controller,
            start_gs_lqr_controller,
            start_mpc_controller,
            start_adaptive_lqr_controller,
            start_historical_gs_lqr_controller,
            start_jump_controller,
            start_velocity_jump_controller,
            start_torque_cascade_pid_controller,
            start_position_torque_cascade_pid_controller,
            load_wheel_effort_controller,
            load_gs_lqr_controllers,
            auto_unpause_action,
            gs_lqr_ready_unpause,
            jump_velocity_startup_gate,
            staged_jump_velocity_startup_gate,
            LogInfo(
                msg="[jump_velocity] WARNING: gazebo_start_paused:=false bypasses the startup readiness interlock.",
                condition=IfCondition(PythonExpression([
                    "'", controller_type, "'.lower() == 'jump_velocity' and '",
                    gazebo_start_paused, "'.lower() not in ['true', '1']"
                ])),
            ),
        ]
    )
