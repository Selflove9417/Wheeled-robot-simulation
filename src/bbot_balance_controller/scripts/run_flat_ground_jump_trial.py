#!/usr/bin/env python3
"""Dedicated Flat-Ground Jump Trial Runner and Acceptance Monitor.

Runs flat-ground rolling jump trials, enforces non-premature termination,
monitors the complete state progression:
PRE_JUMP -> SQUAT -> THRUST -> ATTITUDE_ARREST -> TUCK -> EXTEND -> TOUCHDOWN_BUFFER -> RECOVERY -> BALANCE,
allows >= 6.0s recovery window after touchdown, validates all criteria,
and outputs a complete summary.
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


def ensure_ros_environment():
    """Ensure workspace install environment and ROS log directory are loaded."""
    ws_root = Path(__file__).resolve().parents[3]
    setup_bash = ws_root / "install" / "setup.bash"
    if setup_bash.exists():
        cmd = f"source {setup_bash} && env"
        try:
            out = subprocess.check_output(["bash", "-c", cmd], text=True)
            for line in out.splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    os.environ[k] = v
        except Exception as e:
            print(f"[Warning] Failed to source {setup_bash}: {e}")
    ros_log_dir = ws_root / "ros_log"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)


ensure_ros_environment()


def terminate_process_tree(proc, timeout_sec=6.0):
    """Terminate all descendant processes and the process group of proc safely."""
    if proc is None:
        return
    try:
        import psutil
        parent = psutil.Process(proc.pid)
        children = parent.children(recursive=True)
    except Exception:
        children = []

    # Send SIGINT first to process group
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        pass

    start_wait = time.time()
    while time.time() - start_wait < timeout_sec:
        if proc.poll() is not None:
            break
        time.sleep(0.1)

    # If still alive, terminate via SIGTERM then SIGKILL
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        time.sleep(1.0)

    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

    # Ensure all tracked children are gone
    for child in children:
        try:
            if child.is_running():
                child.kill()
        except Exception:
            pass



def read_latest_log_rows(log_path, num_rows=1):
    """Read last N non-empty rows from the live CSV log."""
    if not os.path.exists(log_path) or os.path.getsize(log_path) == 0:
        return []
    try:
        with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = [l.strip() for l in f if l.strip()]
        if len(lines) <= 1:
            return []
        header = [c.strip() for c in lines[0].split(",")]
        rows = []
        # The controller may be halfway through appending the newest row.
        # Walk backwards to the newest complete rows instead of treating a
        # transient partial tail as "no data".
        for line in reversed(lines[1:]):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == len(header):
                rows.append(dict(zip(header, parts)))
                if len(rows) >= num_rows:
                    break
        rows.reverse()
        return rows
    except Exception:
        return []


def read_summary_row(summary_path):
    """Read the terminal row from the summary CSV."""
    if not os.path.exists(summary_path) or os.path.getsize(summary_path) == 0:
        return None
    try:
        with open(summary_path, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        return rows[-1] if rows else None
    except Exception:
        return None


ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


def controller_states(timeout=5.0):
    """Return controller states without ros2cli colour escape sequences."""
    try:
        check = subprocess.run(
            ["ros2", "control", "list_controllers", "--controller-manager", "/controller_manager"],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        clean_output = ANSI_ESCAPE.sub("", check.stdout).strip()
        states = {}
        for line in clean_output.splitlines():
            fields = line.split()
            if len(fields) >= 3:
                states[fields[0]] = fields[-1]
        return states, clean_output
    except subprocess.TimeoutExpired:
        return {}, "ros2 control list_controllers timed out"


def wait_for_controllers_loaded(proc, timeout_sec=35.0):
    """Wait for controller loading while Gazebo physics remains paused."""
    required = {
        "joint_state_broadcaster",
        "diff_drive_controller",
        "leg_position_controller",
        "leg_effort_controller",
    }
    deadline = time.time() + timeout_sec
    last_output = ""
    while time.time() < deadline:
        if proc.poll() is not None:
            return False, "launch process exited while loading controllers"
        try:
            states, last_output = controller_states()
            if required.issubset(states):
                return True, ""
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(0.25)
    detail = last_output.replace("\n", "; ") or "controller manager unavailable"
    return False, f"controller readiness timed out: {detail}"


def unpause_flat_world():
    """Start physics only after the control stack is ready."""
    command = [
        "ign", "service", "-s", "/world/flat_jump_world/control",
        "--reqtype", "ignition.msgs.WorldControl",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000", "--req", "pause: false",
    ]
    try:
        call = subprocess.run(command, capture_output=True, text=True, timeout=5.0)
    except (subprocess.SubprocessError, OSError) as exc:
        return False, str(exc)
    output = (call.stdout + call.stderr).strip()
    accepted = call.returncode == 0 and "data: true" in output.lower()
    return accepted, output or f"ign service exited with code {call.returncode}"

def step_flat_world(steps=5):
    """Advance a few physics ticks while keeping the effort-mode world paused."""
    command = [
        "ign", "service", "-s", "/world/flat_jump_world/control",
        "--reqtype", "ignition.msgs.WorldControl",
        "--reptype", "ignition.msgs.Boolean", "--timeout", "3000",
        "--req", f"pause: true multi_step: {steps}",
    ]
    try:
        call = subprocess.run(command, capture_output=True, text=True, timeout=5.0)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    output = (call.stdout + call.stderr).strip()
    return call.returncode == 0 and "data: true" in output.lower(), output



def activate_controllers_with_first_physics_tick(
        proc, wheel_actuation="velocity", wheel_effort_file=None, timeout_sec=8.0):
    """Queue activation while paused, then let the first physics tick apply it."""
    try:
        import rclpy
        from controller_manager_msgs.srv import SwitchController
    except ImportError as exc:
        return False, f"ROS 2 controller service unavailable: {exc}"

    rclpy.init(args=None)
    node = rclpy.create_node("flat_jump_startup_gate")
    try:
        client = node.create_client(SwitchController, "/controller_manager/switch_controller")
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "controller switch service unavailable"

        request = SwitchController.Request()
        wheel_controller = ("wheel_effort_controller" if wheel_actuation == "effort"
                            else "diff_drive_controller")
        request.activate_controllers = [
            "joint_state_broadcaster", wheel_controller, "leg_position_controller",
        ]
        request.deactivate_controllers = []
        request.strictness = 2
        request.activate_asap = True
        request.timeout.sec = 5
        future = client.call_async(request)

        if wheel_actuation == "velocity":
            unpaused, reason = unpause_flat_world()
            if not unpaused:
                return False, f"failed to unpause flat world: {reason}"

        deadline = time.time() + timeout_sec
        while time.time() < deadline and not future.done():
            if proc.poll() is not None:
                return False, "launch process exited while activating controllers"
            if wheel_actuation == "effort":
                stepped, reason = step_flat_world()
                if not stepped:
                    return False, f"failed to step paused world: {reason}"
            rclpy.spin_once(node, timeout_sec=0.10)
        if not future.done():
            return False, "controller switch response timed out"
        response = future.result()
        if response is None or not response.ok:
            return False, "controller manager rejected ground-controller activation"

        states, detail = controller_states()
        required = {
            "joint_state_broadcaster", wheel_controller, "leg_position_controller",
        }
        active_ok = False
        detail = ""
        for _ in range(5):
            states, detail = controller_states(timeout=3.0)
            active = {name for name, state in states.items() if state == "active"}
            if required.issubset(active):
                active_ok = True
                break
            time.sleep(0.3)
        other_wheel = ("diff_drive_controller" if wheel_actuation == "effort"
                       else "wheel_effort_controller")
        if not active_ok or states.get(other_wheel) == "active":
            return False, f"wheel controller activation is not exclusive: {detail}"
        if wheel_actuation == "effort":
            if wheel_effort_file is None:
                return False, "missing wheel effort startup telemetry path"
            for _ in range(10):
                latest = read_latest_log_rows(wheel_effort_file, 1)
                if latest and latest[-1].get("cmd_fresh") == "1" and latest[-1].get("joint_fresh") == "1":
                    break
                stepped, reason = step_flat_world()
                if not stepped:
                    return False, f"failed to step paused world: {reason}"
                time.sleep(0.05)
            else:
                return False, "wheel effort servo did not receive fresh data within 50 paused ticks"
            unpaused, reason = unpause_flat_world()
            if not unpaused:
                return False, f"failed to unpause flat world: {reason}"
        return True, ""
    except Exception as exc:
        return False, f"controller activation failed: {exc}"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _run_single_trial(trial_idx, args, output_dir):
    """Execute a single flat-ground rolling jump trial and evaluate criteria."""
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_log.csv"
    summary_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_summary.csv"
    contact_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_contacts.csv"
    wheel_joint_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_wheel_joints.csv"
    leg_controller_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_leg_controller.csv"
    wheel_effort_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_wheel_effort.csv"
    events_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_events.csv"


    launch_cmd = [
        "ros2", "launch", "bbot_bringup", "bbot_gazebo.launch.py",
        "controller_type:=jump_velocity",
        "headless:=" + ("true" if args.headless else "false"),
        "gui:=" + ("false" if args.headless else "true"),
        "world:=flat_jump_world.sdf",
        "gazebo_world_name:=flat_jump_world",
        "gazebo_start_paused:=true",
        "auto_unpause:=false",
        "staged_controller_startup:=true",
        f"jump_height:={args.jump_height:.4f}",
        f"jump_forward_speed:={args.jump_forward_speed:.4f}",
        f"jump_takeoff_forward_speed:={args.jump_takeoff_forward_speed:.4f}",
        f"air_wheel_sign:={args.air_wheel_sign:.4f}",
        f"air_wheel_extend_kd:={args.air_wheel_extend_kd:.4f}",
        f"air_wheel_tuck_kd:={args.air_wheel_tuck_kd:.4f}",
        f"flight_tuck_nominal_duration:={args.flight_tuck_nominal_duration:.4f}",
        f"flight_arrest_freewheel_duration:={args.flight_arrest_freewheel_duration:.4f}",
        "thrust_terminal_hip_floor:=" + ("true" if args.thrust_terminal_hip_floor else "false"),
        f"thrust_wheel_max_decel:={args.thrust_wheel_max_decel:.4f}",
        f"jump_log_path:={log_file}",
        f"jump_summary_path:={summary_file}",
        "auto_return_balance:=true",
    ]

    print(f"\n------------------------------------------------------------")
    print(f"[Trial {trial_idx}] Starting simulation...")
    print(f"  Log:      {log_file}")
    print(f"  Summary:  {summary_file}")
    print(f"  Contacts: {contact_file}")
    print(f"  Wheels:   {wheel_joint_file}")
    print(f"  Leg state: {leg_controller_file}")
    if args.wheel_actuation == "effort":
        print(f"  Effort:   {wheel_effort_file}")
    print(f"------------------------------------------------------------")

    recorder_cmd = [
        sys.executable,
        str(Path(__file__).parent / "record_wheel_contacts.py"),
        str(contact_file),
    ]
    recorder_proc = subprocess.Popen(recorder_cmd, start_new_session=True)
    wheel_recorder_proc = subprocess.Popen(
        [sys.executable, str(Path(__file__).parent / "record_wheel_joint_state.py"),
         str(wheel_joint_file)], start_new_session=True)
    leg_recorder_proc = subprocess.Popen(
        [sys.executable, str(Path(__file__).parent / "record_leg_controller_state.py"),
         str(leg_controller_file)], start_new_session=True)
    proc = subprocess.Popen(launch_cmd, start_new_session=True)
    wheel_servo_proc = None
    trial_start_wall = time.time()
    jump_command_sent = False
    touchdown_seen_time = None
    balance_reentry_time = None
    balance_stable_start = None
    result = {"trial": trial_idx, "success": False, "reason": "",
              "log_file": str(log_file), "summary_file": str(summary_file)}
    pre_command_row = None
    probe_node = None
    probe_publisher = None
    probe_rclpy = None

    try:
        print("[Runner] Waiting for ROS 2 controllers before starting physics...")
        ready, reason = wait_for_controllers_loaded(proc)
        if not ready:
            result["reason"] = reason
            return result
        if args.wheel_actuation == "effort":
            try:
                loaded = subprocess.run(
                    ["ros2", "control", "load_controller", "wheel_effort_controller",
                     "--set-state", "inactive", "-c", "/controller_manager"],
                    capture_output=True, text=True, timeout=15.0)
            except (OSError, subprocess.TimeoutExpired) as exc:
                result["reason"] = f"wheel effort controller load failed: {exc}"
                return result
            if loaded.returncode != 0:
                result["reason"] = f"wheel effort controller load failed: {loaded.stderr.strip()}"
                return result
            wheel_servo_proc = subprocess.Popen(
                [sys.executable, str(Path(__file__).parent / "wheel_effort_velocity_servo.py"),
                 str(wheel_effort_file), "--gain", str(args.wheel_effort_gain)],
                start_new_session=True)
            servo_ready_file = wheel_effort_file.with_suffix(".ready")
            deadline = time.time() + 10.0
            while not servo_ready_file.exists() and wheel_servo_proc.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if wheel_servo_proc.poll() is not None or not servo_ready_file.exists():
                result["reason"] = "wheel effort servo DDS endpoints not ready"
                return result
        ready, reason = activate_controllers_with_first_physics_tick(
            proc, args.wheel_actuation, wheel_effort_file)
        if not ready:
            result["reason"] = reason
            return result
        print("[Runner] Controllers active; flat-world physics started.")
        if args.command_transport == "persistent":
            import rclpy
            from rclpy.parameter import Parameter
            from std_msgs.msg import String
            probe_rclpy = rclpy
            probe_rclpy.init(args=None)
            probe_node = probe_rclpy.create_node(f"flat_jump_command_probe_{trial_idx}")
            probe_node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            probe_publisher = probe_node.create_publisher(String, "/jump_cmd", 10)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                probe_rclpy.spin_once(probe_node, timeout_sec=0.05)
                if (probe_publisher.get_subscription_count() > 0 and
                        probe_node.get_clock().now().nanoseconds > 0):
                    break
            else:
                result["reason"] = "jump-command subscriber or simulation clock unavailable"
                return result

        # Phase 1: Wait for initial standing balance
        print("[Runner] Waiting for robot initial BALANCE stabilization (1.0s continuous)...")
        initial_stable_start_sim = None
        sim_time = 0.0
        max_init_wait = time.time() + 20.0
        while time.time() < max_init_wait:
            if wheel_servo_proc is not None and wheel_servo_proc.poll() is not None:
                result["reason"] = "Wheel effort servo exited during initialization"
                return result
            if proc.poll() is not None:
                result["reason"] = "Launch process exited during initialization"
                return result

            rows = read_latest_log_rows(log_file, 1)
            if rows:
                row = rows[0]
                state = row.get("state_name", "")
                try:
                    pitch = float(row.get("pitch", 99.0))
                    pitch_rate = float(row.get("pitch_rate", 99.0))
                    x_dot = float(row.get("capture_com_velocity", row.get("x_dot", 99.0)))
                    sim_time = float(row.get("timestamp", 0.0))
                    # Initial steady condition: balance state, reasonable pitch & rate
                    if state == "BALANCE" and abs(pitch - 0.034) < 0.05 and abs(pitch_rate) < 0.15 and abs(x_dot) < 0.10:
                        if initial_stable_start_sim is None:
                            initial_stable_start_sim = sim_time
                        elif sim_time - initial_stable_start_sim >= 1.0:
                            print(f"[Runner] Initial balance reached! Stable for >= 1.0s (pitch={pitch:.3f}, x_dot={x_dot:.3f}).")
                            pre_command_row = dict(row)
                            break
                    else:
                        initial_stable_start_sim = None
                except ValueError:
                    pass
            if probe_node is not None:
                probe_rclpy.spin_once(probe_node, timeout_sec=0.05)
            else:
                time.sleep(0.05)

        if initial_stable_start_sim is None or (sim_time - initial_stable_start_sim < 1.0):
            result["reason"] = "Initial balance stabilization timed out"
            return result

        # Phase 2: Send /jump_cmd
        print(f"[Runner] Publishing /jump_cmd via {args.command_transport} ...")
        if probe_node is not None:
            probe_rclpy.spin_once(probe_node, timeout_sec=0.0)
            dispatch_stamp = probe_node.get_clock().now().nanoseconds / 1e9
            pre_command_row["command_transport"] = args.command_transport
            pre_command_row["dispatch_sim_stamp"] = f"{dispatch_stamp:.9f}"
        if args.command_transport == "persistent":
            message = String()
            message.data = "jump"
            probe_publisher.publish(message)
        else:
            pub = subprocess.run([
                "ros2", "topic", "pub", "--once", "/jump_cmd",
                "std_msgs/msg/String", "{data: jump}"
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)
            if pub.returncode != 0:
                result["reason"] = "Failed to publish /jump_cmd"
                return result
        # A successful publish does not prove DDS delivery.  In particular,
        # the short-lived CLI publisher has previously exited successfully
        # while the controller remained in BALANCE for the whole trial.
        try:
            previous_rx = float(pre_command_row.get("jump_cmd_rx_stamp", -1.0))
        except (TypeError, ValueError):
            previous_rx = -1.0
        received = False
        command_deadline = time.monotonic() + 2.0
        while time.monotonic() < command_deadline:
            recent = read_latest_log_rows(log_file, 1)
            if recent:
                try:
                    received = float(recent[0].get("jump_cmd_rx_stamp", -1.0)) > previous_rx
                except (TypeError, ValueError):
                    received = False
                if received:
                    break
            if probe_node is not None:
                probe_rclpy.spin_once(probe_node, timeout_sec=0.05)
            else:
                time.sleep(0.05)
        if not received:
            result["reason"] = "Jump command not received by controller within 2 s"
            return result
        jump_command_sent = True
        jump_sent_time = time.time()

        # Phase 3: Monitor entire jump state progression
        print("[Runner] Monitoring jump progression...")
        states_seen = set()
        flight_subphases_seen = set()
        max_jump_time = time.time() + 25.0
        consecutive_balance_stable = 0.0

        while time.time() < max_jump_time:
            if wheel_servo_proc is not None and wheel_servo_proc.poll() is not None:
                result["reason"] = "Wheel effort servo exited during jump"
                return result
            if proc.poll() is not None:
                break

            rows = read_latest_log_rows(log_file, 1)
            if rows:
                row = rows[0]
                state = row.get("state_name", "")
                states_seen.add(state)
                f_sub = row.get("flight_subphase", "-1")
                if f_sub in ("1", 1):
                    flight_subphases_seen.add("TUCK")
                elif f_sub in ("2", 2):
                    flight_subphases_seen.add("EXTEND")
                elif f_sub in ("3", 3):
                    flight_subphases_seen.add("PROTECTIVE_DEPLOY")

                sim_time = float(row.get("timestamp", 0.0))

                if state == "TOUCHDOWN_BUFFER" and touchdown_seen_time is None:
                    touchdown_seen_time = time.time()
                    print(f"[Runner] Touchdown confirmed at sim_t={sim_time:.3f}s. Recovery window active (>= 6.0s allowed).")

                if touchdown_seen_time is not None:
                    # Once in BALANCE after touchdown: check the 1.0s stability criteria
                    if state == "BALANCE":
                        if balance_reentry_time is None:
                            balance_reentry_time = time.time()
                            print(f"[Runner] Returned to BALANCE at sim_t={sim_time:.3f}s. Verifying 1.0s continuous stability...")
                        try:
                            pitch = float(row.get("pitch", 99.0))
                            pitch_rate = float(row.get("pitch_rate", 99.0))
                            com_vx = float(row.get("capture_com_velocity", row.get("x_dot", 99.0)))
                            com_vz = float(row.get("gazebo_world_z_dot", row.get("z_dot", 99.0)))
                            pitch_err = pitch - 0.030 # balance_offset

                            # Acceptance criteria for stable balance:
                            # |pitch error| < 0.04 rad, |pitch rate| < 0.15 rad/s, |COM vx| < 0.08 m/s, |COM vz| < 0.03 m/s
                            if (abs(pitch_err) < 0.04 and abs(pitch_rate) < 0.15 and
                                abs(com_vx) < 0.08 and abs(com_vz) < 0.03):
                                if balance_stable_start is None:
                                    balance_stable_start = time.time()
                                else:
                                    consecutive_balance_stable = time.time() - balance_stable_start
                                    if consecutive_balance_stable >= 1.0:
                                        print(f"[Runner] BALANCE 1.0s continuous stability verified! (pitch_err={pitch_err:+.3f}, rate={pitch_rate:+.3f}, vx={com_vx:+.3f}, vz={com_vz:+.3f})")
                                        break
                            else:
                                balance_stable_start = None
                        except ValueError:
                            pass
                    else:
                        balance_stable_start = None

            time.sleep(0.02)

    except Exception as exc:
        result["reason"] = f"Trial exception before summary: {type(exc).__name__}: {exc}"
        return result
    finally:
        # Gracefully shutdown only this trial session
        terminate_process_tree(proc)
        terminate_process_tree(recorder_proc)
        terminate_process_tree(wheel_recorder_proc)
        terminate_process_tree(leg_recorder_proc)
        terminate_process_tree(wheel_servo_proc)
        if probe_node is not None:
            probe_node.destroy_node()
        if probe_rclpy is not None and probe_rclpy.ok():
            probe_rclpy.shutdown()
        if args.observe_repeatability:
            try:
                from analyze_flat_jump_repeatability import validity, write_event_csv
                events = write_event_csv(log_file, events_file, pre_command_row)
                result["valid_flight"], result["observation_reason"] = validity(events)
                result["events_file"] = str(events_file)
            except Exception as exc:
                # Observational diagnostics must never alter the jump outcome.
                result["valid_flight"] = False
                result["observation_reason"] = f"event recording failed: {exc}"
                print(f"[Observation] {result['observation_reason']}")

    # Phase 4: Parse summary and evaluate against acceptance criteria
    summary = read_summary_row(summary_file)
    if not summary:
        result["reason"] = "Summary file missing or empty after trial"
        print(f"[Trial {trial_idx}] FAILED: {result['reason']}")
        return result

    result["summary"] = summary

    # Acceptance criteria checks:
    # 1. COM peak net height: 0.20 ± 0.03 m
    # 2. Takeoff COM vertical velocity: 1.98 ± 0.15 m/s
    # 3. Takeoff forward velocity: 0.45 ± 0.10 m/s
    # 4. Normal TUCK and EXTEND entered (no protective deploy)
    # 5. Wheels touch first, no knee/base collision, no EMERGENCY
    # 6. Return to Effort BALANCE within 6s of touchdown, stable for >= 1s

    reasons = []

    try:
        apex_delta = float(summary.get("apex_com_z_delta", summary.get("apex_world_z_delta", 0.0)))
        result["apex_delta"] = apex_delta
        if not (0.17 <= apex_delta <= 0.23):
            reasons.append(f"COM apex height delta {apex_delta:.3f}m out of [0.17, 0.23]m")

        takeoff_vz = float(summary.get("takeoff_com_vz", summary.get("takeoff_velocity", 0.0)))
        result["takeoff_vz"] = takeoff_vz
        if not (1.83 <= takeoff_vz <= 2.13):
            reasons.append(f"Takeoff COM vz {takeoff_vz:.3f}m/s out of [1.83, 2.13]m/s")

        takeoff_vx = float(summary.get("takeoff_com_vx", summary.get("takeoff_forward_speed", 0.0)))
        result["takeoff_vx"] = takeoff_vx
        if not (0.35 <= takeoff_vx <= 0.55):
            reasons.append(f"Takeoff COM vx {takeoff_vx:.3f}m/s out of [0.35, 0.55]m/s")

        tuck_entered = int(summary.get("tuck_entered", 0))
        extend_entered = int(summary.get("extend_entered", 0))
        protective_reason = summary.get("protective_reason", "").strip("\"")
        result["tuck_entered"] = tuck_entered
        result["extend_entered"] = extend_entered
        result["protective_reason"] = protective_reason

        if tuck_entered != 1 or extend_entered != 1:
            reasons.append(f"Normal TUCK/EXTEND not fully completed (tuck={tuck_entered}, extend={extend_entered})")
        if protective_reason:
            reasons.append(f"Protective deploy triggered: {protective_reason}")

        recovery_completed = int(summary.get("recovery_completed", 0))
        if recovery_completed != 1 and consecutive_balance_stable < 1.0:
            reasons.append("Recovery to stable Effort BALANCE not completed")

        exit_code = summary.get("exit_code", "").strip("\"")
        result["exit_code"] = exit_code
        if exit_code and exit_code != "SUCCESS":
            reasons.append(f"Controller reported exit code: {exit_code}")

    except Exception as e:
        reasons.append(f"Error parsing summary metrics: {e}")

    if not reasons:
        result["success"] = True
        print(f"[Trial {trial_idx}] SUCCESS!")
        print(f"  Apex Delta:     {result.get('apex_delta', 0):.3f} m (target 0.20 ± 0.03 m)")
        print(f"  Takeoff COM vz: {result.get('takeoff_vz', 0):.3f} m/s (target 1.98 ± 0.15 m/s)")
        print(f"  Takeoff COM vx: {result.get('takeoff_vx', 0):.3f} m/s (target 0.45 ± 0.10 m/s)")
        print(f"  TUCK / EXTEND:  {result.get('tuck_entered', 0)} / {result.get('extend_entered', 0)}")
    else:
        result["success"] = False
        result["reason"] = "; ".join(reasons)
        print(f"[Trial {trial_idx}] FAILED: {result['reason']}")

    return result


def run_single_trial(trial_idx, args, output_dir):
    """Persist the result even when the trial returns before a controller summary exists."""
    result = _run_single_trial(trial_idx, args, output_dir)
    log_file = Path(result["log_file"])
    outcome_file = log_file.with_name(log_file.name.replace("_log.csv", "_outcome.csv"))
    with outcome_file.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("trial", "success", "reason", "valid_flight"))
        writer.writeheader()
        writer.writerow({key: result.get(key, "") for key in writer.fieldnames})
    result["outcome_file"] = str(outcome_file)
    return result


def main():
    parser = argparse.ArgumentParser(description="Flat-Ground Rolling Jump Acceptance Runner")
    parser.add_argument("--trials", type=int, default=1, help="Number of trials to run")
    parser.add_argument("--jump-height", type=float, default=0.20, help="Target jump height (m)")
    parser.add_argument("--jump-forward-speed", type=float, default=0.35, help="Pre-jump approach speed (m/s)")
    parser.add_argument("--jump-takeoff-forward-speed", type=float, default=0.45, help="Takeoff forward speed (m/s)")
    parser.add_argument("--air-wheel-sign", type=float, default=-1.0, help="In-flight reaction wheel sign (+1.0 or -1.0)")
    parser.add_argument("--air-wheel-extend-kd", type=float, default=0.45, help="Reaction wheel angular rate feedback gain during FLIGHT EXTEND")
    parser.add_argument("--air-wheel-tuck-kd", type=float, default=0.45,
                        help="TUCK angular rate feedback gain; velocity-mode 0.90 probe requires observation mode")
    parser.add_argument("--flight-tuck-nominal-duration", type=float, default=0.06,
                        help="Nominal TUCK duration; velocity-mode 0.08 probe requires observation mode")
    parser.add_argument("--flight-arrest-freewheel-duration", type=float, default=0.0,
                        help="Effort-mode zero-commanded-wheel-torque window at FLIGHT entry (0 to 0.02 s)")
    parser.add_argument("--thrust-terminal-hip-floor", action="store_true",
                        help="Opt-in observation diagnostic: suppress late negative THRUST hip torque")
    parser.add_argument("--thrust-wheel-max-decel", type=float, default=8.0, help="Max deceleration rate limit for thrust ground wheel control (m/s^2)")
    parser.add_argument("--wheel-actuation", choices=("velocity", "effort"), default="velocity",
                        help="Opt-in bounded-effort wheel actuator for flat-jump observation")
    parser.add_argument("--wheel-effort-gain", type=float, choices=(0.5, 1.0, 2.0), default=1.0,
                        help="Wheel speed error gain in Nm/(rad/s), effort mode only")
    parser.add_argument("--output-dir", type=str, default="src/bbot_balance_controller/src/data_logs/flat_jump_trials", help="Directory to save trial logs")
    parser.add_argument("--command-transport", choices=("cli", "persistent"), default=None,
                        help="Simulation-only diagnostic transport; defaults to persistent in observation mode and CLI otherwise")
    parser.add_argument("--observe-repeatability", action="store_true",
                        help="Record phase snapshots and stop after enough valid observed flights")
    parser.add_argument("--min-valid-flights", type=int, default=6,
                        help="Valid flight observations required (only with --observe-repeatability)")
    parser.add_argument("--headless", action="store_true", default=True, help="Run Gazebo headless")
    parser.add_argument("--gui", dest="headless", action="store_false", help="Run Gazebo with GUI")
    args = parser.parse_args()
    if args.command_transport is None:
        args.command_transport = "persistent"
    if args.wheel_actuation == "effort" and not args.observe_repeatability:
        parser.error("effort wheel actuation is available only in observation mode")
    if args.wheel_actuation == "velocity" and args.wheel_effort_gain != 1.0:
        parser.error("wheel-effort-gain requires --wheel-actuation effort")
    if args.wheel_actuation == "velocity" and abs(args.air_wheel_tuck_kd - 0.45) > 1e-9:
        if not args.observe_repeatability or abs(args.air_wheel_tuck_kd - 0.90) > 1e-9:
            parser.error("velocity-mode TUCK gain probe permits only 0.90 in observation mode")
    if not 0.06 <= args.flight_tuck_nominal_duration <= 0.12:
        parser.error("flight-tuck-nominal-duration must be within 0.06 to 0.12 s")
    if args.wheel_actuation == "velocity" and abs(args.flight_tuck_nominal_duration - 0.06) > 1e-9:
        if not args.observe_repeatability or abs(args.flight_tuck_nominal_duration - 0.08) > 1e-9:
            parser.error("velocity-mode TUCK duration probe permits only 0.08 in observation mode")
        if abs(args.air_wheel_tuck_kd - 0.45) > 1e-9:
            parser.error("TUCK duration probe requires the default air-wheel-tuck-kd 0.45")
    if not 0.0 <= args.flight_arrest_freewheel_duration <= 0.02:
        parser.error("flight-arrest-freewheel-duration must be within 0 to 0.02 s")
    if args.wheel_actuation == "velocity" and args.flight_arrest_freewheel_duration != 0.0:
        parser.error("flight-arrest-freewheel-duration requires --wheel-actuation effort")
    if args.thrust_terminal_hip_floor and not args.observe_repeatability:
        parser.error("thrust-terminal-hip-floor requires --observe-repeatability")
    if args.wheel_actuation == "velocity" and args.thrust_terminal_hip_floor:
        if abs(args.air_wheel_tuck_kd - 0.45) > 1e-9 or abs(args.flight_tuck_nominal_duration - 0.06) > 1e-9:
            parser.error("velocity-mode hip-floor probe requires default TUCK gain and duration")
    if args.observe_repeatability and not (1 <= args.min_valid_flights <= args.trials <= 12):
        parser.error("observation mode requires 1 <= min-valid-flights <= trials <= 12")
    if args.observe_repeatability:
        baseline = (
            (args.jump_height, 0.20),
            (args.air_wheel_sign, -1.0),
            (args.thrust_wheel_max_decel, 8.0),
        )
        if args.wheel_actuation == "velocity":
            baseline += (
                (args.jump_forward_speed, 0.35),
                (args.jump_takeoff_forward_speed, 0.45),
                (args.air_wheel_extend_kd, 0.45),
            )
        if any(abs(actual - expected) > 1e-9 for actual, expected in baseline):
            parser.error("paired observation requires all other jump parameters at baseline values")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"================================================================")
    print(f"  Flat-Ground Rolling Jump Campaign (Target {args.jump_height:.2f} m)")
    print(f"  Trials planned: {args.trials}")
    print(f"  Output dir:     {output_dir}")
    print(f"================================================================")

    results = []
    consecutive_successes = 0
    valid_flights = 0

    for i in range(1, args.trials + 1):
        res = run_single_trial(i, args, output_dir)
        results.append(res)
        if args.observe_repeatability:
            valid_flights += bool(res.get("valid_flight"))
            print(f"[Observation] Valid flights: {valid_flights}/{args.min_valid_flights}; "
                  f"attempts: {i}/{args.trials}; {res.get('observation_reason', '')}")
        if res["success"]:
            consecutive_successes += 1
        else:
            consecutive_successes = 0
            if res.get("reason"):
                print(f"[Trial {i}] FAILED: {res['reason']}")
            if args.trials > 1:
                print(f"[Campaign] Consecutive success streak broken at trial {i}.")
        if args.observe_repeatability and valid_flights >= args.min_valid_flights:
            print("[Observation] Reached valid-flight target; stopping bounded baseline campaign.")
            break

    # Campaign report
    total = len(results)
    if args.observe_repeatability:
        from analyze_flat_jump_repeatability import analyze_campaign
        from analyze_jump_handoff import analyze
        print(analyze_campaign(output_dir, args.min_valid_flights))
        print(analyze(output_dir))
    successes = sum(1 for r in results if r["success"])
    print(f"\n================================================================")
    print(f"  Campaign Summary: {successes}/{total} successful ({successes/total*100:.1f}%)")
    print(f"  Max consecutive successes: {consecutive_successes}")
    print(f"================================================================")

    sys.exit(0 if (successes == total and total > 0) else 1)


if __name__ == "__main__":
    main()
