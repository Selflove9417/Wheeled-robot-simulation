#!/usr/bin/env python3
"""One fixed low-speed ground motion protocol. Never requests a jump."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "src/bbot_balance_controller/scripts"
DEFAULT_OBSERVER_BUILD_DIR = ROOT / "build_native_command_observer/bbot_bringup"
REQUIRED_OBSERVER_LIBS = (
    "libbbot_native_command_observer.so",
    "libbbot_ground_contact_frame_recorder.so",
    "libbbot_reference_jump_geometry_recorder.so",
)
sys.path.insert(0, str(SCRIPTS))
import run_flat_ground_jump_trial as startup


def stable(row):
    try:
        return (row["state_name"] == "BALANCE" and
                row["capture_world_valid"] == "1" and
                abs(float(row["pitch"]) - .034) < .05 and
                abs(float(row["pitch_rate"])) < .15 and
                abs(float(row["capture_com_velocity"])) < .10)
    except (KeyError, ValueError, TypeError):
        return False


def sim_stamp(row):
    value = int(row["control_sim_time_ns"]) / 1e9
    if not math.isfinite(value):
        raise RuntimeError("invalid controller clock")
    return value


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def prepend_plugin_search_path(directory, existing):
    """Put the selected observer build first, retaining other search entries."""
    selected = str(directory.resolve())
    entries = [entry for entry in existing.split(os.pathsep) if entry]
    entries = [entry for entry in entries if Path(entry).resolve() != Path(selected)]
    return os.pathsep.join([selected, *entries])


def exclusive(states, effort_legs=False):
    return (states.get("joint_state_broadcaster") == "active" and
            states.get("wheel_effort_controller") == "active" and
            states.get("diff_drive_controller") == "inactive" and
            states.get("leg_effort_controller") == ("active" if effort_legs else "inactive") and
            states.get("leg_position_controller") == ("inactive" if effort_legs else "active"))


def prepare_native_physics_world(source, destination, plugin):
    """Select one private, instrumented physics system without changing dynamics."""
    tree = ET.parse(source)
    systems = [p for p in tree.getroot().findall("world/plugin")
               if p.attrib.get("name", "").endswith("::Physics")]
    if len(systems) != 1:
        raise ValueError("native observation requires exactly one physics system")
    systems[0].set("filename", str(plugin.resolve()))
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def validate_motion_sample(row, anchor_q, anchor_pitch, previous_stamp):
    """Fail closed on stale ownership, malformed state and out-of-domain motion."""
    stamp = sim_stamp(row)
    if stamp < previous_stamp:
        raise RuntimeError("motion controller clock regressed")
    if row["state_name"] != "BALANCE" or row.get("ground_input_stage") == "failed":
        raise RuntimeError("motion left protected BALANCE or ground input failed")
    if row.get("effort_mode_active") != "1" or row.get("leg_mode_switch_pending") != "0":
        raise RuntimeError("motion Effort ownership lost or switch pending")
    pitch, rate = float(row["pitch"]), float(row["pitch_rate"])
    if not all(math.isfinite(x) for x in (pitch, rate, anchor_pitch)):
        raise RuntimeError("motion nonfinite attitude")
    if abs(pitch - anchor_pitch) > .10 or abs(rate) > .5:
        raise RuntimeError("motion attitude protection exceeded")
    positions = [float(row[key]) for key in
                 ("hip_pos_left", "knee_pos_left", "hip_pos_right", "knee_pos_right")]
    velocities = [float(row[key]) for key in
                  ("hip_vel_left", "knee_vel_left", "hip_vel_right", "knee_vel_right")]
    if len(anchor_q) != 4 or not all(math.isfinite(x) for x in positions + velocities + list(anchor_q)):
        raise RuntimeError("motion nonfinite joint state")
    if any(abs(q-q0) > .08 or limit-abs(q) < .30 for q,q0,limit in
           zip(positions,anchor_q,(1.52,1.56,1.52,1.56))):
        raise RuntimeError("motion position/margin protection exceeded")
    if any(abs(v) > .10 for v in velocities):
        raise RuntimeError("motion rate protection exceeded")
    return stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--observer-build-dir", type=Path, default=None,
        help="observer library directory (default: build_native_command_observer/bbot_bringup)")
    parser.add_argument("--native-physics-plugin", type=Path, default=None,
                        help="opt-in private physics system with direct engine state diagnostics")
    args = parser.parse_args()
    if args.native_physics_plugin is None:
        parser.error("motion protocol requires the accepted direct native physics plugin")
    if args.observer_build_dir is None:
        args.observer_build_dir = ROOT / "build_ground_input/bbot_bringup"
    output = args.output_dir.resolve()
    if not (output / "protocol.json").is_file():
        parser.error("write a fixed protocol.json before running")
    protocol = json.loads((output / "protocol.json").read_text())
    if protocol.get("allocator_enabled") is not False or protocol.get("wheel_servo_gain") != 1.0:
        parser.error("motion protocol requires allocator off and wheel gain 1")
    if protocol.get("world") != {"physics_step_ns": 1000000, "real_time_factor": 1.0, "action_speed_scale": 1.0}:
        parser.error("motion protocol requires normal speed, 1ms physics and RTF 1")
    if protocol.get("maximum_ground_runs") != 1 or protocol.get("maximum_jump_candidates") != 0:
        parser.error("motion protocol permits one ground run and zero jump candidates")
    if (output / "launch_command.json").exists():
        parser.error("a protocol directory may be executed only once")
    binary = ROOT / "build_ground_motion/bbot_balance_controller/bbot_landing_repair_controller"
    observer_build_dir = (args.observer_build_dir or DEFAULT_OBSERVER_BUILD_DIR).resolve()
    missing_observer_libs = [name for name in REQUIRED_OBSERVER_LIBS
                             if not (observer_build_dir / name).is_file()]
    if not binary.is_file() or missing_observer_libs:
        detail = ", ".join(missing_observer_libs) if missing_observer_libs else "controller binary"
        parser.error(f"build independent controller and all observer libraries first; missing: {detail}")
    observer = observer_build_dir / "libbbot_native_command_observer.so"
    world_source = ROOT / "src/bbot_bringup/worlds/native_command_observation_world.sdf"
    selected_world = world_source
    native_physics_plugin = args.native_physics_plugin.resolve() if args.native_physics_plugin else None
    if native_physics_plugin is not None:
        if not native_physics_plugin.is_file():
            parser.error("build private native physics plugin before running")
        selected_world = output / "native_engine_world.sdf"
        prepare_native_physics_world(world_source, selected_world, native_physics_plugin)
    paths = {name: output / filename for name, filename in {
        "control": "control.csv", "summary": "controller_summary.csv",
        "native": "native_wrench.csv", "contacts": "ground_frames.csv",
        "geometry": "geometry.csv", "servo": "wheel_servo.csv"}.items()}
    launch = ["ros2", "launch", str(ROOT / "experiments/jump/ground_motion.launch.py"),
              "controller_type:=jump_velocity", "jump_height:=0.25",
              f"world:={selected_world}", "gazebo_world_name:=flat_jump_world",
              "real_time_factor:=1.0", "gazebo_start_paused:=true", "auto_unpause:=false",
              "staged_controller_startup:=true", "headless:=true", "gui:=false",
              f"jump_log_path:={paths['control']}", f"jump_summary_path:={paths['summary']}"]
    environment = os.environ.copy()
    environment.update({"BBOT_NATIVE_WRENCH_CSV": str(paths["native"]),
                        "BBOT_GROUND_CONTACT_FRAME_CSV": str(paths["contacts"]),
                        "BBOT_REFERENCE_GEOMETRY_CSV": str(paths["geometry"]),
                        "BBOT_NATIVE_COMMAND_STATE_TOPIC": "/world/flat_jump_world/native_command_state"})
    if native_physics_plugin is not None:
        paths["engine_state"] = output / "engine_state.csv"
        environment["BBOT_NATIVE_PHYSICS_CSV"] = str(paths["engine_state"])
    for variable in ("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH"):
        environment[variable] = prepend_plugin_search_path(
            observer_build_dir, environment.get(variable, ""))
    write_json(output / "launch_command.json", {"argv": launch,
               "ROS_DOMAIN_ID": environment.get("ROS_DOMAIN_ID"),
               "IGN_PARTITION": environment.get("IGN_PARTITION"),
               "observer_build_dir": str(observer_build_dir),
               "native_physics_plugin": str(native_physics_plugin) if native_physics_plugin else None,
               "selected_world": str(selected_world),
               "plugin_search_paths": {name: environment[name] for name in
                                       ("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH")},
               "paths": {k: str(v) for k, v in paths.items()}})
    sources = [binary, observer, ROOT / "src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp",
               SCRIPTS / "wheel_effort_velocity_servo.py", Path(__file__),
               ROOT / "experiments/jump/ground_motion.launch.py",
               world_source, selected_world]
    if native_physics_plugin is not None:
        sources += [native_physics_plugin]
        sources += [p for p in (ROOT / "experiments/jump/native_physics").rglob("*") if p.is_file()]
    sources += [ROOT / "experiments/jump/run_ground_motion_trial.sh",
                output / "protocol.json"]
    sources += sorted((ROOT / "src/bbot_balance_controller/include").rglob("*.hpp"))
    sources += [ROOT / "src/bbot_balance_controller/CMakeLists.txt",
                ROOT / "src/bbot_bringup/config/bbot_controllers.yaml",
                ROOT / "src/bbot_bringup/launch/bbot_gazebo.launch.py",
                ROOT / "src/bbot_bringup/src/landing_repair_wrench_recorder.cc"]
    sources += sorted((ROOT / "src/bbot_description").rglob("*.xacro"))
    sources += sorted((ROOT / "src/bbot_kinematics/include").rglob("*.hpp"))
    sources += sorted(observer_build_dir.glob("*.so"))
    sources += sorted((ROOT / "src/bbot_bringup/src").glob("*.cc"))
    sources += [ROOT / "experiments/jump/planning/ground_static_configurations.cpp"]
    sources = sorted(set(sources))
    manifest = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    write_json(output / "tested_manifest.json", manifest)
    snapshot = output / "frozen_run_files"
    for source in sources:
        relative = source.relative_to(ROOT) if source.is_relative_to(ROOT) else Path("external") / source.name
        destination = snapshot / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())
    snapshot_checks = {name: hashlib.sha256((snapshot / (Path(name).relative_to(ROOT) if Path(name).is_relative_to(ROOT) else Path("external") / Path(name).name)).read_bytes()).hexdigest() == digest
                       for name, digest in manifest.items()}
    write_json(output / "source_snapshot_verification.json", {
        "gate": "PASS" if all(snapshot_checks.values()) else "FAIL",
        "file_count": len(sources), "checked": snapshot_checks})
    if not all(snapshot_checks.values()):
        raise RuntimeError("source snapshot changed before launch")
    result = {"status": "FAIL", "physical_runs": 0, "airborne_jumps": 0, "accepted_jumps": 0,
              "clearance_m": None, "first_contact_pitch_deg": None, "landing_retreat_m": None,
              "model_gate": "NOT_TESTED", "braking_gate": "NOT_TESTED", "events": []}
    proc = servo = recorder = node = None
    streams = []
    rclpy = None
    try:
        stream = (output / "launch.log").open("w"); streams.append(stream)
        proc = subprocess.Popen(launch, stdout=stream, stderr=subprocess.STDOUT,
                                env=environment, start_new_session=True)
        ok, detail = startup.wait_for_controllers_loaded(proc)
        result["events"].append({"stage": "controllers_loaded", "ok": ok, "detail": detail})
        if not ok:
            raise RuntimeError(detail)
        loaded = subprocess.run(["ros2", "control", "load_controller", "wheel_effort_controller",
                                 "--set-state", "inactive", "-c", "/controller_manager"],
                                capture_output=True, text=True, timeout=15)
        (output / "wheel_controller_load.txt").write_text(loaded.stdout + loaded.stderr)
        if loaded.returncode:
            raise RuntimeError("wheel effort load failed")
        stream = (output / "servo.log").open("w"); streams.append(stream)
        servo = subprocess.Popen([sys.executable, str(SCRIPTS / "wheel_effort_velocity_servo.py"),
                                  str(paths["servo"]), "--gain", "1.0"],
                                 stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while not paths["servo"].with_suffix(".ready").exists():
            if servo.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("P wheel servo discovery failed while paused")
            time.sleep(.05)
        stream = (output / "joint_recorder.log").open("w"); streams.append(stream)
        recorder = subprocess.Popen([sys.executable, str(SCRIPTS / "record_joint_torque.py"),
                                     str(output / "legs")], stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        ok, detail = startup.activate_controllers_with_first_physics_tick(
            proc, "effort", paths["servo"], diagnostic_file=output / "startup_gate.txt")
        # The gate may have advanced up to 100 native 1ms steps even on failure.
        result["physical_runs"] = 1 if paths["native"].is_file() and paths["native"].stat().st_size else 0
        result["events"].append({"stage": "startup_gate", "ok": ok, "detail": detail})
        if not ok:
            raise RuntimeError(detail)
        states, detail = startup.controller_states()
        if not exclusive(states):
            raise RuntimeError("wheel/leg startup exclusivity not established: " + detail)
        result["initial_controller_states"] = states
        dump = subprocess.run(["ros2", "param", "dump", "/bbot_velocity_jump_controller"],
                              capture_output=True, text=True, timeout=5)
        (output / "runtime_parameters.yaml").write_text(dump.stdout)
        if dump.returncode or not dump.stdout.strip():
            result.pop("execution_status", None)
            raise RuntimeError("runtime parameter binding failed: " + dump.stderr)

        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from std_msgs.msg import String
        rclpy.init()
        node = Node("ground_motion_protocol_runner", parameter_overrides=[Parameter("use_sim_time", value=True)])
        publisher = node.create_publisher(String, "/ground_input_cmd", 10)
        started = time.monotonic()
        quiet_start = None
        last_stamp = -1.
        while time.monotonic() - started < 30:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in (proc, servo, recorder)):
                raise RuntimeError("a ground protocol process exited")
            rows = startup.read_latest_log_rows(paths["control"])
            if not rows:
                continue
            row = rows[-1]; stamp = sim_stamp(row)
            if not math.isfinite(stamp):
                raise RuntimeError("invalid controller clock")
            if stamp < last_stamp:
                raise RuntimeError("controller clock regressed")
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            if row["state_name"] != "BALANCE":
                raise RuntimeError("ground protocol left BALANCE")
            if abs(float(row["pitch"]) - .034) > .20 or abs(float(row["pitch_rate"])) > 2.:
                raise RuntimeError("ground support attitude protection exceeded")
            quiet_start = (quiet_start if quiet_start is not None else stamp) if stable(row) else None
            if quiet_start is not None and stamp - quiet_start >= 1. and stamp >= 3.:
                break
        else:
            raise RuntimeError("P wheel Position stand did not satisfy original quiet thresholds")
        if not publisher.get_subscription_count():
            raise RuntimeError("private ground command subscription absent")
        msg = String(); msg.data = "enter_effort"
        publisher.publish(msg)
        result["events"].append({"stage": "enter_effort_requested", "sim_ns": node.get_clock().now().nanoseconds})
        started = time.monotonic(); quiet_start = None; hold_start = None; previous_clock = last_stamp
        while time.monotonic() - started < 20:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in (proc, servo, recorder)):
                raise RuntimeError("a ground protocol process exited")
            rows = startup.read_latest_log_rows(paths["control"])
            if not rows:
                continue
            row = rows[-1]; stamp = sim_stamp(row)
            if not math.isfinite(stamp):
                raise RuntimeError("invalid controller clock")
            if stamp < previous_clock:
                raise RuntimeError("controller clock regressed")
            if stamp == previous_clock:
                continue
            previous_clock = stamp
            if row["state_name"] != "BALANCE" or row.get("ground_input_stage") == "failed":
                raise RuntimeError("ground control rejected/left protected BALANCE")
            if abs(float(row["pitch"]) - .034) > .20 or abs(float(row["pitch_rate"])) > 2.:
                raise RuntimeError("Effort support attitude protection exceeded")
            ready = (row.get("effort_mode_active") == "1" and row.get("leg_mode_switch_pending") == "0"
                     and row.get("ground_input_stage") == "effort_hold" and stable(row))
            if hold_start is not None:
                if not ready:
                    raise RuntimeError("the fixed Effort hold window lost stable support/mode")
                if stamp - hold_start >= 5.:
                    result["hold_start_ns"] = round(hold_start * 1e9)
                    result["hold_end_ns"] = result["hold_start_ns"] + 5_000_000_000
                    break
            else:
                quiet_start = (quiet_start if quiet_start is not None else stamp) if ready else None
                if quiet_start is not None and stamp - quiet_start >= 1.:
                    states, detail = startup.controller_states()
                    if not exclusive(states, True):
                        raise RuntimeError("Effort hold exclusivity failed: " + detail)
                    result["hold_controller_states"] = states
                    # Bind the window prospectively to the next controller sample.
                    hold_start = None
                    quiet_start = None
                    rows = startup.read_latest_log_rows(paths["control"])
                    if not rows or not stable(rows[-1]):
                        raise RuntimeError("support changed while confirming controller modes")
                    hold_start = sim_stamp(rows[-1])
                    result["events"].append({"stage": "hold_window_frozen", "sim_ns": round(hold_start * 1e9)})
        else:
            raise RuntimeError("Effort entry/hold timeout")
        result["baseline_status"] = "HOLD_RECORDED_AWAITING_OFFLINE_INPUT_AUDIT"
        trace_path = Path(str(paths["control"]) + ".motion_trace.csv")
        motion_msg = String(); motion_msg.data = "start_motion"
        publisher.publish(motion_msg)
        result["events"].append({"stage": "motion_requested", "sim_ns": node.get_clock().now().nanoseconds})
        motion_started = time.monotonic()
        last_motion_progress_wall = motion_started
        last_trace_key = None
        last_motion_stamp = previous_clock
        initial_motion_q = [float(row[key]) for key in
                            ("hip_pos_left", "knee_pos_left", "hip_pos_right", "knee_pos_right")]
        initial_motion_pitch = float(row["pitch"])
        while time.monotonic() - motion_started < 60:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in (proc, servo, recorder)):
                raise RuntimeError("a motion protocol process exited")
            trace_rows = startup.read_latest_log_rows(trace_path)
            motion = trace_rows[-1] if trace_rows else None
            if motion and motion.get("failed") == "1":
                raise RuntimeError("motion controller rejected: " + motion.get("reason", "unknown"))
            if motion:
                trace_key = (motion.get("sim_time_ns"), motion.get("command_id"), motion.get("stage"))
                if trace_key != last_trace_key:
                    last_trace_key = trace_key
                    last_motion_progress_wall = time.monotonic()
            if time.monotonic() - last_motion_progress_wall > 1.0:
                raise RuntimeError("motion control publication/trace liveness lost for 1s wall time")
            rows = startup.read_latest_log_rows(paths["control"])
            if not rows:
                continue
            row = rows[-1]; stamp = sim_stamp(row)
            if stamp < last_motion_stamp:
                raise RuntimeError("motion controller clock regressed")
            if stamp == last_motion_stamp:
                continue
            last_motion_stamp = stamp
            validate_motion_sample(row, initial_motion_q, initial_motion_pitch, stamp)
            if motion is None:
                continue
            if motion.get("complete") == "1":
                result["motion_complete_ns"] = int(motion["sim_time_ns"])
                result["execution_status"] = "MOTION_RECORDED_AWAITING_OFFLINE_MODEL_AND_STOP_AUDIT"
                break
        else:
            raise RuntimeError("fixed motion protocol timeout")
    except Exception as exc:
        result["reason"] = str(exc)
        print("Ground input protocol stopped:", exc, flush=True)
    finally:
        if proc is not None:
            paused, detail = startup.pause_flat_world()
            result["failure_or_finish_pause"] = {"ok": paused, "detail": detail}
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        for process in (servo, recorder, proc):
            startup.terminate_process_tree(process)
        for stream in streams:
            stream.close()
        if paths["native"].exists():
            with paths["native"].open() as native_stream:
                native_rows = list(csv.DictReader(native_stream))
            frames = {r.get("physics_iteration") for r in native_rows}
            result["recorded_physics_frames"] = len(frames)
            result["physical_runs"] = int(bool(frames))
        original = json.loads((output / "tested_manifest.json").read_text())
        result["tested_artifacts_changed_during_run"] = [
            name for name, digest in original.items()
            if not Path(name).is_file() or hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest]
        write_json(output / "run_result.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result.get("execution_status") else 1


if __name__ == "__main__":
    sys.exit(main())
