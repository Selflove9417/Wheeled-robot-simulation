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
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

_script_dir = str(Path(__file__).resolve().parent)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from jump_startup_gate import initial_balance_row_stable
from jump_profile import resolve_profile


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
    if not os.path.exists(log_path):
        return []
    try:
        with open(log_path, "rb") as f:
            header_line = f.readline()
            if not header_line:
                return []
            header = [c.strip() for c in header_line.decode("utf-8", errors="ignore").strip().split(",")]
            header_end = f.tell()
            size = os.fstat(f.fileno()).st_size
            if size <= header_end:
                return []
            # The trace can exceed tens of MB. Read only its tail so polling
            # does not repeatedly scan the whole live file and starve ROS.
            tail_size = min(size - header_end, 1024 * 1024)
            start = size - tail_size
            f.seek(start)
            tail = f.read().decode("utf-8", errors="ignore")
        lines = tail.splitlines()
        if tail and not tail.endswith(("\n", "\r")) and lines:
            lines = lines[:-1]  # final row may still be actively written
        if start > header_end and lines:
            lines = lines[1:]  # first tail line may be a partial CSV record
        if not lines:
            return []
        rows = []
        # The controller may be halfway through appending the newest row.
        # Walk backwards to the newest complete rows instead of treating a
        # transient partial tail as "no data".
        for line in reversed(lines):
            if not line.strip():
                continue
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


def read_summary_rows(summary_path):
    """Read all complete controller summaries, preserving their jump ids."""
    if not os.path.exists(summary_path) or os.path.getsize(summary_path) == 0:
        return []
    try:
        with open(summary_path, "r", encoding="utf-8", errors="ignore") as stream:
            return list(csv.DictReader(stream))
    except Exception:
        return []


def audit_jump_summaries(rows, expected_count):
    """Evaluate each jump row independently so a later success cannot hide a failure."""
    failures = []
    if len(rows) != expected_count:
        failures.append(f"Only {len(rows)}/{expected_count} jump summaries were recorded in this session")
    for index, item in enumerate(rows, 1):
        try:
            jump_id = int(item.get("jump_id", 0))
            if jump_id != index:
                failures.append(f"session summary {index} has jump_id {jump_id}, expected {index}")
            # COM takeoff lock values remain diagnostics. Physical height, vz,
            # and vx acceptance comes from synchronized contact-gap fits below.
            if int(item.get("tuck_entered", 0)) != 1 or int(item.get("extend_entered", 0)) != 1:
                failures.append(f"jump_id {jump_id} did not complete normal TUCK and EXTEND")
            if item.get("protective_reason", "").strip('"'):
                failures.append(f"jump_id {jump_id} entered protective deployment")
            if item.get("exit_code", "").strip('"') != "SUCCESS":
                failures.append(f"jump_id {jump_id} controller exit code was {item.get('exit_code')}")
        except (TypeError, ValueError) as exc:
            failures.append(f"jump summary {index} could not be evaluated: {exc}")
    return failures


def audit_flight_id_sequence(rows, expected_count):
    """Require one FLIGHT segment for each sequential jump id in a session."""
    expected = [str(i) for i in range(1, expected_count + 1)]
    recorded = []
    flight_segments = []
    previous_id = None
    previous_state = None
    for row in rows:
        jump_id = str(row.get("jump_id", ""))
        state = row.get("state_name", "")
        if jump_id not in ("", "0") and jump_id != previous_id:
            recorded.append(jump_id)
        if state == "FLIGHT" and (previous_state != "FLIGHT" or jump_id != previous_id):
            flight_segments.append(jump_id)
        previous_id = jump_id
        previous_state = state
    failures = []
    if recorded != expected:
        failures.append(f"Trace jump_id sequence {recorded} is not exactly {expected}")
    for jump_id in expected:
        count = flight_segments.count(jump_id)
        if count != 1:
            failures.append(f"jump_id {jump_id} has {count} FLIGHT segments; expected exactly one")
    return failures


def audit_flight_subphase_trace(rows, expected_count):
    """Require each jump's raw FLIGHT trace to complete ARREST→TUCK→EXTEND."""
    expected_ids = [str(i) for i in range(1, expected_count + 1)]
    transitions = {jump_id: [] for jump_id in expected_ids}
    observed_errors = {jump_id: set() for jump_id in expected_ids}
    failures = []
    for row in rows:
        jump_id = str(row.get("jump_id", ""))
        state = str(row.get("state_name", ""))
        if jump_id in transitions and state == "EMERGENCY":
            observed_errors[jump_id].add("trace entered EMERGENCY")
        if state != "FLIGHT" or jump_id not in transitions:
            continue
        try:
            subphase = int(row["flight_subphase"])
        except (KeyError, TypeError, ValueError):
            observed_errors[jump_id].add("FLIGHT row has missing/invalid subphase")
            continue
        if subphase not in (0, 1, 2):
            label = "protective deploy" if subphase == 3 else f"unknown subphase {subphase}"
            observed_errors[jump_id].add(f"FLIGHT trace entered {label}")
            continue
        if not transitions[jump_id] or transitions[jump_id][-1] != subphase:
            transitions[jump_id].append(subphase)
    for jump_id in expected_ids:
        phases = transitions[jump_id]
        failures.extend(f"jump_id {jump_id} {reason}"
                        for reason in sorted(observed_errors[jump_id]))
        if phases != [0, 1, 2]:
            failures.append(
                f"jump_id {jump_id} FLIGHT subphase sequence {phases} is not exactly [0, 1, 2]")
    return failures


def audit_complete_contact_takeoff_trace(rows, expected_count):
    """Require the opt-in complete-frame witness to be the actual FLIGHT entry source."""
    failures = []
    for jump_id in (str(i) for i in range(1, expected_count + 1)):
        entries = [row for row in rows
                   if str(row.get("jump_id", "")) == jump_id and
                   str(row.get("state_name", "")) == "FLIGHT"]
        if not entries:
            failures.append(f"jump_id {jump_id} has no FLIGHT entry row for contact witness audit")
            continue
        row = entries[0]
        try:
            enabled = int(row.get("complete_contact_takeoff_enabled", "0")) == 1
            source_valid = int(row.get("contact_source_valid", "0")) == 1
            taken = int(row.get("contact_source_taken", "0")) == 1
            confirmed = int(row.get("contact_takeoff_confirmed", "0")) == 1
            zero_frames = int(row.get("contact_zero_frames", "0"))
            zero_span = int(row.get("contact_zero_span_ns", "0"))
            source_stamp = int(row.get("contact_source_stamp_ns", "0"))
            event_stamp = int(row.get("contact_takeoff_frame_stamp_ns", "0"))
            com_stamp = int(row.get("contact_takeoff_com_stamp_ns", "0"))
            source_seq = int(row.get("contact_source_seq", "-1"))
        except (TypeError, ValueError):
            failures.append(f"jump_id {jump_id} FLIGHT entry has malformed contact witness fields")
            continue
        if not enabled:
            failures.append(f"jump_id {jump_id} FLIGHT entry did not enable complete contact confirmation")
        if not (source_valid and taken and confirmed):
            failures.append(f"jump_id {jump_id} FLIGHT entry was not taken from the complete contact witness")
        if zero_frames < 11 or zero_span < 10_000_000:
            failures.append(f"jump_id {jump_id} contact witness lacks 11 zero frames over 10 ms")
        if source_stamp <= 0 or event_stamp != source_stamp or com_stamp <= 0 or source_seq < 0:
            failures.append(f"jump_id {jump_id} contact witness event stamps/sequence are invalid")
    return failures


def audit_contact_flights(log_file, expected_count, ground_frame_file=None,
                          native_contact_file=None, require_contact_witness=False):
    """Independently fit each FLIGHT's synchronized COM samples in its contact gap."""
    scripts_dir = str(Path(__file__).resolve().parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    from audit_contact_flight_metrics import audit
    with open(log_file, newline="") as stream:
        rows = list(csv.DictReader(stream))
    expected_ids = [str(i) for i in range(1, expected_count + 1)]
    parsed_ground_frames = None
    failures = []
    if ground_frame_file is not None:
        from audit_ground_contact_frames import (
            validate_active_trace, crosscheck_native_events)
        valid_frames, frame_reason, parsed_ground_frames = validate_active_trace(
            Path(log_file), Path(ground_frame_file))
        if not valid_frames:
            failures.append(f"complete contact-frame trace invalid: {frame_reason}")
            parsed_ground_frames = None
        elif native_contact_file is not None:
            failures.extend(crosscheck_native_events(parsed_ground_frames,
                                                      Path(native_contact_file)))
    results = [audit(Path(log_file), jump_id, ground_frame_file,
                     parsed_ground_frames) for jump_id in expected_ids]
    failures.extend(audit_flight_id_sequence(rows, expected_count))
    failures.extend(audit_flight_subphase_trace(rows, expected_count))
    if require_contact_witness:
        failures.extend(audit_complete_contact_takeoff_trace(rows, expected_count))
    for index, item in enumerate(results, 1):
        jid = item.get("jump_id", index)
        if not item.get("freefall_verified"):
            failures.append(f"jump_id {jid} contact-gap fit invalid: {item.get('reason')}")
            continue
        height = float(item["apex_delta_estimate"])
        vz = float(item["takeoff_vz_estimate"])
        if not .17 <= height <= .23:
            failures.append(f"jump_id {jid} fitted height {height:.3f}m outside [0.17, 0.23]")
        if not 1.83 <= vz <= 2.13:
            failures.append(f"jump_id {jid} fitted takeoff vz {vz:.3f}m/s outside [1.83, 2.13]")
        if not item.get("horizontal_fit_verified"):
            failures.append(
                f"jump_id {jid} independent horizontal COM fit invalid: "
                f"{item.get('horizontal_reason') or 'no fitted velocity'}")
            continue
        vx = float(item["takeoff_vx_estimate"])
        if not .35 <= vx <= .55:
            failures.append(f"jump_id {jid} fitted horizontal COM vx {vx:.6f}m/s outside [0.35, 0.55]")
    return results, failures


def wait_for_ground_contact_frames(frame_file, launch_process, timeout=5.0):
    """Fail closed until the world plugin has emitted two consecutive valid frames."""
    deadline = time.monotonic() + timeout
    path = Path(frame_file)
    error_path = Path(str(path) + ".error")
    last_error = "contact-frame stream has not produced valid frames"
    while time.monotonic() < deadline:
        if launch_process.poll() is not None:
            return False, "Gazebo launch exited before complete contact frames became ready"
        if error_path.exists():
            try:
                detail = error_path.read_text().strip()
            except OSError:
                detail = "plugin wrote an error marker"
            return False, f"ground contact frame plugin failed: {detail}"
        try:
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            required = {"seq", "sim_time_ns", "physics_iteration", "dt_ns",
                        "frame_valid", "num_contacts", "collision_pairs_json", "error"}
            if rows and required.issubset(rows[0].keys()):
                from audit_ground_contact_frames import ready_from_rows
                if ready_from_rows(rows):
                    return True, "two consecutive 1ms valid frames contain both wheel-ground pairs"
                last_error = "waiting for two consecutive 1ms valid frames with bilateral wheel contacts"
            elif path.exists():
                last_error = "contact frame CSV header is missing required fields"
        except FileNotFoundError:
            pass
        except (OSError, csv.Error) as exc:
            last_error = f"cannot read contact frame CSV: {exc}"
        time.sleep(0.02)
    return False, f"timed out waiting for valid native contact frames: {last_error}"


def attach_independent_fit_metrics(result, controller_summary, contact_metrics):
    """Keep controller latch diagnostics separate from contact-gap physical estimates."""
    result["controller_summary_takeoff_vx"] = float(
        controller_summary.get("takeoff_com_vx", controller_summary.get("takeoff_forward_speed", 0.0)))
    result["controller_summary_takeoff_vz"] = float(
        controller_summary.get("takeoff_com_vz", controller_summary.get("takeoff_velocity", 0.0)))
    result["controller_summary_apex_delta"] = float(
        controller_summary.get("apex_com_z_delta", controller_summary.get("apex_world_z_delta", 0.0)))
    result["takeoff_vx_by_jump"] = [item.get("takeoff_vx_estimate", "") for item in contact_metrics]
    result["takeoff_vz_by_jump"] = [item.get("takeoff_vz_estimate", "") for item in contact_metrics]
    result["apex_delta_by_jump"] = [item.get("apex_delta_estimate", "") for item in contact_metrics]
    if contact_metrics:
        result["takeoff_vx"] = contact_metrics[-1].get("takeoff_vx_estimate", 0.0)
        result["takeoff_vz"] = contact_metrics[-1].get("takeoff_vz_estimate", 0.0)
        result["apex_delta"] = contact_metrics[-1].get("apex_delta_estimate", 0.0)
    else:
        result["takeoff_vx"] = result["takeoff_vz"] = result["apex_delta"] = 0.0


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


def wait_for_controllers_loaded(proc, timeout_sec=60.0):
    """Wait through cold controller-manager initialization while physics is paused."""
    required = {
        "joint_state_broadcaster",
        "diff_drive_controller",
        "leg_position_controller",
        "leg_effort_controller",
    }
    started = time.monotonic()
    deadline = started + timeout_sec
    last_output = ""
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return False, "launch process exited while loading controllers"
        try:
            remaining = max(0.0, deadline - time.monotonic())
            if remaining <= 0.0:
                break
            states, last_output = controller_states(timeout=min(5.0, remaining))
            if required.issubset(states):
                return True, (
                    f"loaded_after_wall_sec={time.monotonic() - started:.3f};"
                    f"controllers={','.join(sorted(required))};states=" +
                    ",".join(f"{name}:{states[name]}" for name in sorted(required)))
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(0.25)
    detail = last_output.replace("\n", "; ") or "controller manager unavailable"
    return False, (
        f"controller loading/startup discovery timed out after "
        f"{time.monotonic() - started:.3f}s (physics remained paused): {detail}")


def unpause_flat_world(world_name="flat_jump_world"):
    """Start physics only after the control stack is ready."""
    command = [
        "ign", "service", "-s", f"/world/{world_name}/control",
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


def pause_flat_world(world_name="flat_jump_world"):
    """Fail closed if an external startup-gate process must be stopped."""
    command = [
        "ign", "service", "-s", f"/world/{world_name}/control",
        "--reqtype", "ignition.msgs.WorldControl",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "1000", "--req", "pause: true",
    ]
    try:
        call = subprocess.run(command, capture_output=True, text=True, timeout=3.0)
    except (subprocess.SubprocessError, OSError) as exc:
        return False, str(exc)
    output = (call.stdout + call.stderr).strip()
    return call.returncode == 0 and "data: true" in output.lower(), output

def step_flat_world(steps=5, world_name="flat_jump_world"):
    """Advance a few physics ticks while keeping the effort-mode world paused."""
    command = [
        "ign", "service", "-s", f"/world/{world_name}/control",
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
        proc, wheel_actuation="velocity", wheel_effort_file=None, timeout_sec=15.0,
        diagnostic_file=None):
    """Run the shared paused-physics gate before a jump trial can move."""
    command = [
        "ros2", "run", "bbot_balance_controller", "jump_startup_gate.py",
        "--world", "flat_jump_world", "--wheel-actuation", wheel_actuation,
        "--activate-staged", "--timeout", str(timeout_sec),
        "--max-physics-seconds", "0.100",
    ]
    def save_diagnostics(content):
        if diagnostic_file is not None:
            Path(diagnostic_file).write_text(
                "command=" + " ".join(command) + "\n" + content.strip() + "\n")
    try:
        gate = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, start_new_session=True)
        deadline = time.monotonic() + timeout_sec + 3.0
        output = ""
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                pause_flat_world()
                terminate_process_tree(gate, timeout_sec=2.0)
                try:
                    output, _ = gate.communicate(timeout=1.0)
                except subprocess.TimeoutExpired:
                    output = ""
                message = (output.strip() + "\n" if output.strip() else "") + \
                    "launch process exited while startup gate was checking readiness"
                save_diagnostics(message)
                return False, message
            try:
                output, _ = gate.communicate(timeout=0.2)
                break
            except subprocess.TimeoutExpired:
                continue
        else:
            pause_flat_world()
            terminate_process_tree(gate, timeout_sec=2.0)
            try:
                output, _ = gate.communicate(timeout=1.0)
            except subprocess.TimeoutExpired:
                output = ""
            message = (output.strip() + "\n" if output.strip() else "") + \
                "shared jump startup gate timed out"
            save_diagnostics(message)
            return False, message
        if gate.returncode != 0:
            message = output.strip() or "shared jump startup gate rejected readiness"
            save_diagnostics(message)
            return False, message
        save_diagnostics(output.strip())
        return True, output.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        output = ""
        if "gate" in locals():
            pause_flat_world()
            terminate_process_tree(gate, timeout_sec=2.0)
            try:
                output, _ = gate.communicate(timeout=1.0)
            except (subprocess.SubprocessError, OSError):
                pass
        message = (output.strip() + "\n" if output.strip() else "") + \
            f"unable to start shared jump startup gate: {exc}"
        save_diagnostics(message)
        return False, message

def _run_single_trial(trial_idx, args, output_dir):
    """Execute a single flat-ground rolling jump trial and evaluate criteria."""
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_log.csv"
    summary_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_summary.csv"
    contact_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_contacts.csv"
    ground_frame_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_ground_frames.csv"
    geometry_experiment = args.jump_controller_executable in (
        "bbot_reference_jump_controller", "bbot_landing_repair_controller")
    geometry_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_ground_geometry.csv"
    native_wrench_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_native_wrench.csv"
    ground_frame_required = Path(str(ground_frame_file) + ".required")
    wheel_joint_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_wheel_joints.csv"
    leg_controller_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_leg_controller.csv"
    startup_gate_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_startup_gate.txt"
    controller_loading_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_controller_loading.txt"
    wheel_effort_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_wheel_effort.csv"
    events_file = output_dir / f"trial_{trial_idx:02d}_{timestamp_str}_events.csv"


    launch_entry = ([str(Path(__file__).resolve().parents[2] / "bbot_bringup" / "launch" / ("bbot_capture_experiment.launch.py" if getattr(args,"native_state_diagnostics",False) else "bbot_allocator_experiment.launch.py"))]
                    if getattr(args,"thrust_support_allocator",False) else
                    ["bbot_bringup","bbot_gazebo.launch.py"])
    launch_cmd = [
        "ros2", "launch", *launch_entry,
        "controller_type:=jump_velocity",
        f"jump_controller_executable:={args.jump_controller_executable}",
        "headless:=" + ("true" if args.headless else "false"),
        "gui:=" + ("false" if args.headless else "true"),
        "world:=" + ("native_command_observation_world.sdf" if
                     getattr(args, "native_command_observation", False) else
                     "reference_flat_jump_world.sdf" if geometry_experiment else "flat_jump_world.sdf"),
        "gazebo_world_name:=flat_jump_world",
        "gazebo_start_paused:=true",
        "auto_unpause:=false",
        "staged_controller_startup:=true",
        f"real_time_factor:={args.real_time_factor:.6g}",
        f"jump_height:={args.jump_height:.4f}",
        f"jump_forward_speed:={args.jump_forward_speed:.4f}",
        f"jump_takeoff_forward_speed:={args.jump_takeoff_forward_speed:.4f}",
        f"thrust_forward_velocity_kp:={args.thrust_forward_velocity_kp:.4f}",
        "thrust_forward_attitude_taper:=" + ("true" if args.thrust_forward_attitude_taper else "false"),
        "thrust_release_fast_rate_correction:=" +
        ("true" if args.thrust_release_fast_rate_correction else "false"),
        "thrust_fast_rate_correction_from_gate:=" +
        ("true" if args.thrust_fast_rate_correction_from_gate else "false"),
        "thrust_momentum_reference:=" + ("true" if args.thrust_momentum_reference else "false"),
        "thrust_reference_handoff:=" + ("true" if args.thrust_reference_handoff else "false"),
        "thrust_support_coordination:=" + ("true" if args.thrust_support_coordination else "false"),
        f"thrust_release_velocity_ratio:={args.thrust_release_velocity_ratio:.4f}",
        "thrust_forward_speed_prediction:=" +
        ("true" if args.thrust_forward_speed_prediction else "false"),
        "thrust_wheel_kinematics_compensation:=" +
        ("true" if args.thrust_wheel_kinematics_compensation else "false"),
        "complete_contact_takeoff_confirmation:=" +
        ("true" if args.complete_contact_takeoff_confirmation else "false"),
        "arrest_dynamics_feedforward:=" +
        ("true" if args.arrest_dynamics_feedforward else "false"),
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
    if getattr(args,"thrust_support_allocator",False):
        launch_cmd.append("allocator_runner_guard:=true")

    print(f"\n------------------------------------------------------------")
    print(f"[Trial {trial_idx}] Starting simulation...")
    print(f"  Log:      {log_file}")
    print(f"  Summary:  {summary_file}")
    print(f"  Contacts: {contact_file}")
    print(f"  Ground frames: {ground_frame_file}")
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
    launch_env = os.environ.copy()
    launch_env["BBOT_GROUND_CONTACT_FRAME_CSV"] = str(ground_frame_file)
    if geometry_experiment:
        launch_env["BBOT_REFERENCE_GEOMETRY_CSV"] = str(geometry_file)
    # A parent environment must not accidentally enable the opt-in observer.
    launch_env.pop("BBOT_NATIVE_WRENCH_CSV", None)
    if getattr(args, "record_native_wrench", False):
        launch_env["BBOT_NATIVE_WRENCH_CSV"] = str(native_wrench_file)
    ground_frame_required.write_text(
        "Required complete per-step Gazebo ground contact frames for this trial.\n")
    proc = subprocess.Popen(launch_cmd, start_new_session=True, env=launch_env)
    wheel_servo_proc = None
    trial_start_wall = time.time()
    jump_command_sent = False
    touchdown_seen_time = None
    touchdown_seen_sim_time = None
    balance_reentry_time = None
    balance_reentry_sim_time = None
    balance_stable_start = None
    result = {"trial": trial_idx, "success": False, "reason": "",
              "log_file": str(log_file), "summary_file": str(summary_file),
              "ground_frame_file": str(ground_frame_file)}
    pre_command_row = None
    probe_node = None
    probe_publisher = None
    probe_rclpy = None

    try:
        controller_loading_file.write_text(
            "stage=paused_controller_manager_loading\n"
            "wall_budget_sec=60\nphysics_ticks=0\n", encoding="utf-8")
        print("[Runner] Waiting up to 60 s for the paused controller manager to load; "
              "this stage advances no physics.")
        ready, reason = wait_for_controllers_loaded(proc)
        controller_loading_file.write_text(
            "stage=paused_controller_manager_loading\n"
            "wall_budget_sec=60\nphysics_ticks=0\n"
            + ("result=loaded\n" if ready else "result=timeout\n")
            + f"detail={reason}\n", encoding="utf-8")
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
            proc, args.wheel_actuation, wheel_effort_file,
            diagnostic_file=startup_gate_file)
        if not ready:
            result["reason"] = reason
            return result
        print("[Runner] Controllers active; flat-world physics started.")
        frame_ready, frame_reason = wait_for_ground_contact_frames(
            ground_frame_file, proc, timeout=5.0)
        if not frame_ready:
            result["reason"] = "Ground contact frame source failed closed before first J: " + frame_reason
            return result
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
                    # Same physical quiet limits, plus fresh world capture and
                    # the controller's own ready flag; stale cached COM cannot
                    # extend a false stable interval.
                    if initial_balance_row_stable(row):
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
        # A received command is not necessarily an accepted jump. Confirm the
        # controller increments jump_id and enters PRE_JUMP. If a transient
        # stale sensor frame rejects the request, wait for fresh evidence and
        # issue a new, explicit request; never leave a rejected first J as an
        # apparently running trial.
        try:
            previous_rx = float(pre_command_row.get("jump_cmd_rx_stamp", -1.0))
            previous_accept = float(pre_command_row.get("jump_cmd_accept_stamp", -1.0))
            previous_jump_id = int(float(pre_command_row.get("jump_id", 0)))
        except (TypeError, ValueError):
            previous_rx, previous_accept, previous_jump_id = -1.0, -1.0, 0
        rejection_reasons = []
        accepted = False
        received_any = False
        for attempt in range(8):
            # Dispatch only while still in BALANCE and both independent
            # centroidal observations are fresh.
            ready_deadline = time.monotonic() + 2.0
            ready_row = None
            while time.monotonic() < ready_deadline:
                candidates = read_latest_log_rows(log_file, 1)
                if candidates:
                    candidate = candidates[0]
                    try:
                        t = float(candidate.get("timestamp", 0.0))
                        aligned_stamp = float(candidate.get("centroidal_aligned_rate_stamp", -1.0))
                        com_stamp = float(candidate.get("com_sample_stamp", -1.0))
                        if (candidate.get("state_name") == "BALANCE" and
                                candidate.get("jump_ready", "0") == "1" and
                                candidate.get("centroidal_aligned_rate_valid", "0") == "1" and
                                candidate.get("com_balance_valid", "0") == "1" and
                                0.0 <= t - aligned_stamp <= 0.08 and 0.0 <= t - com_stamp <= 0.08):
                            ready_row = candidate
                            break
                    except (TypeError, ValueError):
                        pass
                if probe_node is not None:
                    probe_rclpy.spin_once(probe_node, timeout_sec=0.02)
                else:
                    time.sleep(0.02)
            if ready_row is None:
                result["reason"] = "Initial jump not accepted: no fresh BALANCE readiness; " + "; ".join(rejection_reasons)
                return result

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

            command_deadline = time.monotonic() + 0.75
            saw_rejection = False
            while time.monotonic() < command_deadline:
                recent = read_latest_log_rows(log_file, 1)
                if recent:
                    row = recent[0]
                    try:
                        rx = float(row.get("jump_cmd_rx_stamp", -1.0))
                        accept = float(row.get("jump_cmd_accept_stamp", -1.0))
                        jump_id = int(float(row.get("jump_id", 0)))
                        received = rx > previous_rx
                        received_any = received_any or received
                        accepted = (received and accept > previous_accept and
                                    jump_id > previous_jump_id and row.get("state_name") == "PRE_JUMP")
                        if accepted:
                            break
                        if received and row.get("jump_reject_reason", "").strip():
                            reason = row["jump_reject_reason"].strip()
                            if reason not in rejection_reasons:
                                rejection_reasons.append(reason)
                            saw_rejection = True
                            previous_rx = rx
                            break
                    except (TypeError, ValueError):
                        pass
                if probe_node is not None:
                    probe_rclpy.spin_once(probe_node, timeout_sec=0.02)
                else:
                    time.sleep(0.02)
            if accepted:
                break
            if not saw_rejection:
                break
            previous_accept = accept
            previous_jump_id = jump_id

        if not accepted:
            detail = "; ".join(rejection_reasons) if rejection_reasons else "controller did not acknowledge the request"
            result["reason"] = ("Initial jump not accepted after %d attempts: %s" %
                                (len(rejection_reasons) + (1 if received_any else 0), detail))
            return result
        jump_command_sent = True
        jump_sent_time = time.time()

        # Phase 3: Monitor entire jump state progression
        print("[Runner] Monitoring jump progression...")
        states_seen = set()
        flight_subphases_seen = set()
        max_jump_time = time.time() + args.jumps_per_session * (25.0 + 2.0 * max(
            args.post_balance_observe_duration, 12.0 if args.jumps_per_session > 1 else 0.0))
        consecutive_balance_stable = 0.0
        completed_in_session = 1
        session_td_to_balance = []
        failed_jump_balance_stable_start = None
        failed_jump_detected_sim = None

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

                # A failed jump may time out in PRE_JUMP without a touchdown.
                # Keep the session alive until the controller safely returns
                # to a quiet BALANCE, then stop before another J can mask it.
                prior_summaries = read_summary_rows(summary_file)
                current_failed_summary = (
                    len(prior_summaries) >= completed_in_session and
                    prior_summaries[completed_in_session - 1].get("exit_code", "").strip('"') != "SUCCESS")
                if current_failed_summary and failed_jump_detected_sim is None:
                    failed_jump_detected_sim = sim_time
                if current_failed_summary and state == "BALANCE":
                    try:
                        quiet = (abs(float(row.get("pitch", 99.0)) - .030) < .04 and
                                 abs(float(row.get("pitch_rate", 99.0))) < .15 and
                                 abs(float(row.get("capture_com_velocity", row.get("x_dot", 99.0)))) < .08 and
                                 abs(float(row.get("gazebo_world_z_dot", row.get("z_dot", 99.0)))) < .03)
                    except (TypeError, ValueError):
                        quiet = False
                    if quiet:
                        if failed_jump_balance_stable_start is None:
                            failed_jump_balance_stable_start = sim_time
                        elif sim_time - failed_jump_balance_stable_start >= 1.0:
                            result["reason"] = (
                                f"jump_id {prior_summaries[completed_in_session - 1].get('jump_id')} failed; "
                                "stopped after safe BALANCE recovery")
                            break
                    else:
                        failed_jump_balance_stable_start = None
                if (current_failed_summary and failed_jump_detected_sim is not None and
                        sim_time - failed_jump_detected_sim >= 12.0):
                    result["reason"] = (
                        f"jump_id {prior_summaries[completed_in_session - 1].get('jump_id')} failed; "
                        "no stable BALANCE return during 12 s recovery observation")
                    break

                if state == "TOUCHDOWN_BUFFER" and touchdown_seen_time is None:
                    touchdown_seen_time = time.time()
                    touchdown_seen_sim_time = sim_time
                    print(f"[Runner] Touchdown confirmed at sim_t={sim_time:.3f}s. Recovery window active (>= 6.0s allowed).")

                if touchdown_seen_time is not None:
                    # EMERGENCY cannot recover. Retain the full six-second
                    # observation window, then end this failed trial without
                    # waiting through the optional successful-hold extension.
                    if state == "EMERGENCY" and sim_time-touchdown_seen_sim_time >= 6.0:
                        break
                    # Once in BALANCE after touchdown: check the 1.0s stability criteria
                    if state == "BALANCE":
                        if balance_reentry_time is None:
                            balance_reentry_time = time.time()
                            balance_reentry_sim_time = sim_time
                            if (touchdown_seen_sim_time is not None and
                                    balance_reentry_sim_time - touchdown_seen_sim_time > 6.0):
                                result["td_to_balance_seconds"] = balance_reentry_sim_time - touchdown_seen_sim_time
                                result["td_to_balance_pass"] = False
                            else:
                                result["td_to_balance_seconds"] = (
                                    balance_reentry_sim_time - touchdown_seen_sim_time
                                    if touchdown_seen_sim_time is not None else None)
                                result["td_to_balance_pass"] = touchdown_seen_sim_time is not None
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
                                    balance_stable_start = sim_time
                                else:
                                    consecutive_balance_stable = sim_time - balance_stable_start
                                    required_hold = max(args.post_balance_observe_duration,
                                                        12.0 if args.jumps_per_session > 1 else 0.0)
                                    if (consecutive_balance_stable >= 1.0 and
                                        sim_time - balance_reentry_sim_time >= required_hold):
                                        print(f"[Runner] BALANCE 1.0s continuous stability verified! (pitch_err={pitch_err:+.3f}, rate={pitch_rate:+.3f}, vx={com_vx:+.3f}, vz={com_vz:+.3f})")
                                        session_td_to_balance.append(
                                            balance_reentry_sim_time - touchdown_seen_sim_time
                                            if touchdown_seen_sim_time is not None else float("inf"))
                                        if completed_in_session >= args.jumps_per_session:
                                            break
                                        # Keep Gazebo/controller alive, record a full 12 s hold,
                                        # then issue the next jump on the same persistent node.
                                        previous_rx = float(row.get("jump_cmd_rx_stamp", -1.0))
                                        previous_accept = float(row.get("jump_cmd_accept_stamp", -1.0))
                                        previous_jump_id = int(float(row.get("jump_id", 0)))
                                        if probe_node is not None:
                                            probe_rclpy.spin_once(probe_node, timeout_sec=0.0)
                                            message = String()
                                            message.data = "jump"
                                            probe_publisher.publish(message)
                                        else:
                                            pub = subprocess.run([
                                                "ros2", "topic", "pub", "--once", "/jump_cmd",
                                                "std_msgs/msg/String", "{data: jump}"
                                            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)
                                            if pub.returncode != 0:
                                                result["reason"] = "Failed to publish next session jump"
                                                break
                                        next_rx_deadline = time.monotonic() + 2.0
                                        accepted = False
                                        while time.monotonic() < next_rx_deadline:
                                            latest = read_latest_log_rows(log_file, 1)
                                            if latest:
                                                try:
                                                    accepted = (float(latest[-1].get("jump_cmd_rx_stamp", -1.0)) > previous_rx and
                                                                float(latest[-1].get("jump_cmd_accept_stamp", -1.0)) > previous_accept and
                                                                int(float(latest[-1].get("jump_id", 0))) > previous_jump_id and
                                                                latest[-1].get("state_name") == "PRE_JUMP")
                                                except (TypeError, ValueError):
                                                    accepted = False
                                                if accepted: break
                                            if probe_node is not None:
                                                probe_rclpy.spin_once(probe_node, timeout_sec=0.05)
                                            else:
                                                time.sleep(0.05)
                                        if not accepted:
                                            result["reason"] = "Next session jump was rejected or not received"
                                            break
                                        completed_in_session += 1
                                        touchdown_seen_time = touchdown_seen_sim_time = None
                                        balance_reentry_time = balance_reentry_sim_time = None
                                        balance_stable_start = None
                                        consecutive_balance_stable = 0.0
                                        states_seen.clear()
                                        flight_subphases_seen.clear()
                                        continue
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

    result["session_td_to_balance_seconds"] = session_td_to_balance
    if (len(session_td_to_balance) != args.jumps_per_session or
            any(not (0.0 <= value <= 6.0) for value in session_td_to_balance)):
        result["td_to_balance_pass"] = False
        result["td_to_balance_seconds"] = max(session_td_to_balance, default=0.0)

    # Phase 4: Parse summary and evaluate against acceptance criteria
    summary = read_summary_row(summary_file)
    if not summary:
        result["reason"] = "Summary file missing or empty after trial"
        print(f"[Trial {trial_idx}] FAILED: {result['reason']}")
        return result

    result["summary"] = summary
    session_summaries = read_summary_rows(summary_file)
    result["jump_summaries"] = session_summaries
    result["controller_summary_takeoff_vx_by_jump"] = [
        float(item.get("takeoff_com_vx", item.get("takeoff_forward_speed", 0.0)))
        for item in session_summaries]
    per_jump_failures = audit_jump_summaries(session_summaries, args.jumps_per_session)
    physics_failures = []
    try:
        contact_results, contact_failures = audit_contact_flights(
            log_file, args.jumps_per_session, ground_frame_file, contact_file,
            args.complete_contact_takeoff_confirmation)
        result["contact_flight_metrics"] = contact_results
        contact_output = summary_file.with_name(summary_file.stem + "_contact_flight_audit.csv")
        from audit_contact_flight_metrics import FIELDS as CONTACT_FIELDS
        with contact_output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CONTACT_FIELDS)
            writer.writeheader()
            writer.writerows(contact_results)
        result["contact_flight_audit"] = str(contact_output)
        per_jump_failures.extend(contact_failures)
        physics_failures.extend(contact_failures)
    except Exception as exc:
        result["contact_flight_metrics"] = []
        message = f"Independent contact-gap audit failed: {type(exc).__name__}: {exc}"
        per_jump_failures.append(message)
        physics_failures.append(message)
    try:
        from audit_ground_contact_frames import validate_active_trace, audit_session
        frame_ok, frame_reason, frame_rows = validate_active_trace(log_file, ground_frame_file)
        frame_failures = [] if frame_ok else [frame_reason]
        if frame_ok:
            frame_failures.extend(audit_session(log_file, frame_rows))
        result["ground_frame_audit"] = {
            "pass": not frame_failures,
            "reason": "; ".join(frame_failures),
            "frames": len(frame_rows),
            "path": str(ground_frame_file),
        }
        if frame_failures:
            message = "Complete ground-frame audit failed: " + "; ".join(frame_failures)
            per_jump_failures.append(message)
            physics_failures.append(message)
    except Exception as exc:
        message = f"Complete ground-frame audit failed: {type(exc).__name__}: {exc}"
        per_jump_failures.append(message)
        physics_failures.append(message)
    try:
        from audit_ground_contacts import audit as audit_ground_contact_trace
        ground_result = audit_ground_contact_trace(Path(log_file), contact_file)
        result["ground_contact_audit"] = ground_result
        result["ground_contact_pass"] = bool(ground_result.get("ground_contact_pass"))
        ground_output = summary_file.with_name(summary_file.stem + "_ground_contact_audit.csv")
        with ground_output.open("w", newline="") as stream:
            fields = ("ground_contact_pass", "ground_contact_events", "reason")
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerow({key: ground_result.get(key, "") for key in fields})
        result["ground_contact_audit_file"] = str(ground_output)
        if not ground_result.get("ground_contact_pass"):
            message = "Native ground-contact audit failed: " + ground_result.get("reason", "unknown failure")
            per_jump_failures.append(message)
            physics_failures.append(message)
    except Exception as exc:
        result["ground_contact_pass"] = False
        message = f"Native ground-contact audit failed: {type(exc).__name__}: {exc}"
        per_jump_failures.append(message)
        physics_failures.append(message)
    if geometry_experiment:
        from audit_landing_geometry import FIELDS as GEOMETRY_FIELDS, audit_landing_geometry
        geometry_metrics, geometry_failures = audit_landing_geometry(
            log_file, ground_frame_file, geometry_file)
        result["geometry_file"] = str(geometry_file)
        result["landing_geometry_metrics"] = geometry_metrics
        result["landing_geometry_pass"] = not geometry_failures
        result["landing_geometry_failures"] = geometry_failures
        geometry_audit_path = summary_file.with_name(summary_file.stem + "_landing_geometry_audit.csv")
        with geometry_audit_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=GEOMETRY_FIELDS)
            writer.writeheader()
            writer.writerows(geometry_metrics)
        result["landing_geometry_audit_file"] = str(geometry_audit_path)
        per_jump_failures.extend(geometry_failures)
        physics_failures.extend(geometry_failures)
    try:
        scripts_dir = str(Path(__file__).resolve().parent)
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        from analyze_flat_jump_landing_hold import FIELDS as HOLD_FIELDS, audit_session
        hold_results = audit_session(Path(log_file), 12.0, args.jumps_per_session)
        hold_output = summary_file.with_name(summary_file.stem + "_landing_hold_audit.csv")
        with hold_output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=HOLD_FIELDS)
            writer.writeheader()
            writer.writerows(hold_results)
        result["landing_hold_metrics"] = hold_results
        result["landing_hold_audit"] = str(hold_output)
        per_jump_failures.extend(
            f"jump_id {item.get('jump_id')}: landing hold failed ({item.get('reason')})"
            for item in hold_results if not item.get("landing_hold_pass"))
        physics_failures.extend(
            f"jump_id {item.get('jump_id')}: landing hold failed ({item.get('reason')})"
            for item in hold_results if not item.get("landing_hold_pass"))
        # Final recovery timing comes from the complete simulation trace, not
        # the runner's polling cadence. Every jump id must be present once.
        trace_delays = [float(item["balance_delay"]) for item in hold_results
                        if item.get("balance_delay") != ""]
        result["session_td_to_balance_seconds"] = trace_delays
        result["td_to_balance_pass"] = (
            len(hold_results) == args.jumps_per_session and
            len(trace_delays) == args.jumps_per_session and
            all(0.0 <= value <= 6.0 for value in trace_delays))
        result["td_to_balance_seconds"] = max(trace_delays, default=0.0)
        if not result["td_to_balance_pass"]:
            physics_failures.append("Per-jump touchdown-to-BALANCE timing did not pass")
    except Exception as exc:
        result["landing_hold_metrics"] = []
        result["td_to_balance_pass"] = False
        per_jump_failures.append(f"Full-trace landing-hold audit failed: {type(exc).__name__}: {exc}")
        physics_failures.append(f"Full-trace landing-hold audit failed: {type(exc).__name__}: {exc}")

    # Acceptance criteria checks:
    # 1. COM peak net height: 0.20 ± 0.03 m
    # 2. Takeoff COM vertical velocity: 1.98 ± 0.15 m/s
    # 3. Takeoff forward velocity: 0.45 ± 0.10 m/s
    # 4. Normal TUCK and EXTEND entered (no protective deploy)
    # 5. Wheels touch first, no knee/base collision, no EMERGENCY
    # 6. Return to Effort BALANCE within 6s of touchdown, stable for >= 1s

    reasons = []
    if result.get("reason"):
        reasons.append(result["reason"])
    reasons.extend(per_jump_failures)

    if result.get("td_to_balance_pass") is False:
        delays = result.get("session_td_to_balance_seconds", [])
        reasons.append(
            "Touchdown-to-BALANCE audit failed: expected exactly %d per-jump delays, each within 0..6s; observed %s" %
            (args.jumps_per_session, ", ".join(f"{value:.3f}s" for value in delays) if delays else "none"))

    try:
        attach_independent_fit_metrics(
            result, summary, result.get("contact_flight_metrics", []))

        tuck_entered = int(summary.get("tuck_entered", 0))
        extend_entered = int(summary.get("extend_entered", 0))
        protective_reason = summary.get("protective_reason", "").strip("\"")
        result["tuck_entered"] = tuck_entered
        result["extend_entered"] = extend_entered
        result["protective_reason"] = protective_reason

        if tuck_entered != 1 or extend_entered != 1:
            message = f"Normal TUCK/EXTEND not fully completed (tuck={tuck_entered}, extend={extend_entered})"
            reasons.append(message)
            physics_failures.append(message)
        if protective_reason:
            message = f"Protective deploy triggered: {protective_reason}"
            reasons.append(message)
            physics_failures.append(message)

        recovery_completed = int(summary.get("recovery_completed", 0))
        if recovery_completed != 1 and consecutive_balance_stable < 1.0:
            message = "Recovery to stable Effort BALANCE not completed"
            reasons.append(message)
            physics_failures.append(message)

        exit_code = summary.get("exit_code", "").strip("\"")
        result["exit_code"] = exit_code
        if exit_code and exit_code != "SUCCESS":
            reasons.append(f"Controller reported exit code: {exit_code}")

    except Exception as e:
        reasons.append(f"Error parsing summary metrics: {e}")

    result["controller_verdict"] = "PASS" if (
        len(session_summaries) == args.jumps_per_session and
        all(row.get("exit_code", "").strip('"') == "SUCCESS" and
            row.get("recovery_completed", "0") == "1" for row in session_summaries)
    ) else "FAIL"
    result["physics_session_acceptance"] = "PASS" if not physics_failures else "FAIL"

    if not reasons:
        result["success"] = True
        print(f"[Trial {trial_idx}] SUCCESS!")
        print("  Contact-gap COM apex by jump: " + ", ".join(
            f"{float(value):.3f}m" for value in result.get("apex_delta_by_jump", [])))
        print("  Fitted takeoff COM vz by jump: " + ", ".join(
            f"{float(value):.3f}m/s" for value in result.get("takeoff_vz_by_jump", [])))
        print("  Fitted takeoff COM vx by jump: " + ", ".join(
            f"{float(value):.3f}m/s" for value in result.get("takeoff_vx_by_jump", [])))
        print("  Controller summary COM vx (diagnostic): " + ", ".join(
            f"{float(value):.3f}m/s" for value in
            (result.get("controller_summary_takeoff_vx_by_jump", []) or
             [result.get("controller_summary_takeoff_vx", 0.0)])))
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
        writer = csv.DictWriter(stream, fieldnames=(
            "trial", "controller_verdict", "physics_session_acceptance",
            "success", "reason", "valid_flight"))
        writer.writeheader()
        writer.writerow({key: result.get(key, "") for key in writer.fieldnames})
    result["outcome_file"] = str(outcome_file)
    return result


def main():
    parser = argparse.ArgumentParser(description="Flat-Ground Rolling Jump Acceptance Runner")
    parser.add_argument("--jump-controller-executable",
                        choices=("bbot_velocity_jump_controller", "bbot_reference_jump_controller",
                                 "bbot_landing_repair_controller"),
                        default="bbot_velocity_jump_controller",
                        help="Independent experiment executable; baseline remains the default")
    parser.add_argument("--trials", type=int, default=1, help="Number of trials to run")
    parser.add_argument("--jumps-per-session", type=int, default=1,
                        help="Consecutive jumps in one Gazebo/controller session")
    parser.add_argument("--jump-height", type=float, default=0.20, help="Target jump height (m)")
    parser.add_argument("--jump-forward-speed", type=float, default=0.35, help="Pre-jump approach speed (m/s)")
    parser.add_argument("--jump-takeoff-forward-speed", type=float, default=0.45, help="Takeoff forward speed (m/s)")
    parser.add_argument("--real-time-factor", type=float, default=None,
                        help="Gazebo real-time factor (default jump profile 1.0)")
    parser.add_argument("--thrust-forward-velocity-kp", type=float, default=None,
                        help="Existing THRUST world COM forward-velocity feedback gain (0 to 2)")
    for option, dest, help_text in (
        ("thrust-forward-attitude-taper", "thrust_forward_attitude_taper",
         "Fade negative THRUST attitude wheel drive near COM takeoff speed"),
        ("thrust-release-fast-rate-correction", "thrust_release_fast_rate_correction",
         "Use bounded signed fast/legacy gyro correction after confirmed Effort support"),
        ("thrust-fast-rate-correction-from-gate", "thrust_fast_rate_correction_from_gate",
         "Blend bounded signed gyro correction in over 20 ms after the THRUST gate"),
        ("thrust-forward-speed-prediction", "thrust_forward_speed_prediction",
         "Use bounded recent-frame prediction in THRUST forward-speed feedback"),
        ("thrust-wheel-kinematics-compensation", "thrust_wheel_kinematics_compensation",
         "Use bounded CAD COM/axle braking correction in Effort THRUST"),
        ("complete-contact-takeoff-confirmation", "complete_contact_takeoff_confirmation",
         "Use complete Gazebo contact frames as an additional takeoff witness"),
        ("arrest-dynamics-feedforward", "arrest_dynamics_feedforward",
         "Use CAD inverse-dynamics feedforward during Effort ATTITUDE_ARREST"),
    ):
        group = parser.add_mutually_exclusive_group()
        group.add_argument(f"--{option}", dest=dest, action="store_true", help=help_text)
        group.add_argument(f"--no-{option}", dest=dest, action="store_false",
                           help=f"Disable {option.replace('-', ' ')}")
        parser.set_defaults(**{dest: None})
    parser.add_argument("--thrust-reference-handoff", action="store_true",
                        help="Private velocity-reference handoff; requires --thrust-momentum-reference")
    parser.add_argument("--record-native-wrench", action="store_true",
                        help="Opt-in native physics joint-load observations for landing repair")
    parser.add_argument("--native-state-diagnostics", action="store_true", help="Private actual native state and command-publication clock audit")
    parser.add_argument("--thrust-support-allocator", action="store_true", help="Private bounded P allocator/controller experiment")
    parser.add_argument("--native-command-observation", action="store_true",
                        help="Private observer-before-physics world; requires repair controller and native recording")
    parser.add_argument("--thrust-momentum-reference", action="store_true",
                        help="Private landing-repair experiment; preserves the existing supported COM velocity task")
    parser.add_argument("--thrust-support-coordination", action="store_true",
                        help="Private landing-repair COM and vertical-impulse coordination experiment")
    parser.add_argument("--thrust-release-velocity-ratio", type=float, choices=(0.70, 0.74, 0.78),
                        default=None,
                        help="THRUST unloading onset ratio; supported values are 0.70, 0.74, and 0.78")
    parser.add_argument("--air-wheel-sign", type=float, default=-1.0, help="In-flight reaction wheel sign (+1.0 or -1.0)")
    parser.add_argument("--air-wheel-extend-kd", type=float, default=2.50, help="Reaction wheel angular rate feedback gain during FLIGHT EXTEND")
    parser.add_argument("--air-wheel-tuck-kd", type=float, default=0.45,
                        help="TUCK angular rate feedback gain; velocity-mode 0.90 probe requires observation mode")
    parser.add_argument("--flight-tuck-nominal-duration", type=float, default=0.06,
                        help="Nominal TUCK duration; velocity-mode 0.08 probe requires observation mode")
    parser.add_argument("--flight-arrest-freewheel-duration", type=float, default=0.0,
                        help="Effort-mode zero-commanded-wheel-torque window at FLIGHT entry (0 to 0.02 s)")
    parser.add_argument("--thrust-terminal-hip-floor", action="store_true",
                        help="Opt-in observation diagnostic: suppress late negative THRUST hip torque")
    parser.add_argument("--thrust-wheel-max-decel", type=float, default=None, help="Max deceleration rate limit for thrust ground wheel control (m/s^2)")
    parser.add_argument("--wheel-actuation", choices=("velocity", "effort"), default="velocity",
                        help="Opt-in bounded-effort wheel actuator for flat-jump observation")
    parser.add_argument("--wheel-effort-gain", type=float, choices=(0.5, 1.0, 2.0), default=1.0,
                        help="Wheel speed error gain in Nm/(rad/s), effort mode only")
    parser.add_argument("--output-dir", type=str, default="src/bbot_balance_controller/src/data_logs/flat_jump_trials", help="Directory to save trial logs")
    parser.add_argument("--command-transport", choices=("cli", "persistent"), default=None,
                        help="Simulation-only diagnostic transport; defaults to persistent in observation mode and CLI otherwise")
    parser.add_argument("--observe-repeatability", action="store_true",
                        help="Record phase snapshots and stop after enough valid observed flights")
    parser.add_argument("--post-balance-observe-duration", type=float, default=0.0,
                        help="Keep recording at least this many simulation seconds after BALANCE reentry (0 to 30)")
    parser.add_argument("--min-valid-flights", type=int, default=6,
                        help="Valid flight observations required (only with --observe-repeatability)")
    parser.add_argument("--headless", action="store_true", default=True, help="Run Gazebo headless")
    parser.add_argument("--gui", dest="headless", action="store_false", help="Run Gazebo with GUI")
    args = parser.parse_args()
    profile_overrides = {
        "real_time_factor": args.real_time_factor,
        "jump_height": args.jump_height,
        "jump_forward_speed": args.jump_forward_speed,
        "jump_takeoff_forward_speed": args.jump_takeoff_forward_speed,
        "thrust_forward_velocity_kp": args.thrust_forward_velocity_kp,
        "thrust_forward_attitude_taper": args.thrust_forward_attitude_taper,
        "thrust_release_fast_rate_correction": args.thrust_release_fast_rate_correction,
        "thrust_fast_rate_correction_from_gate": args.thrust_fast_rate_correction_from_gate,
        "thrust_release_velocity_ratio": args.thrust_release_velocity_ratio,
        "thrust_forward_speed_prediction": args.thrust_forward_speed_prediction,
        "thrust_wheel_kinematics_compensation": args.thrust_wheel_kinematics_compensation,
        "complete_contact_takeoff_confirmation": args.complete_contact_takeoff_confirmation,
        "arrest_dynamics_feedforward": args.arrest_dynamics_feedforward,
        "thrust_wheel_max_decel": args.thrust_wheel_max_decel,
    }
    try:
        resolved_profile = resolve_profile(
            "jump_velocity", args.observe_repeatability, profile_overrides)
    except ValueError as exc:
        parser.error(str(exc))
    for key, value in resolved_profile.items():
        if hasattr(args, key):
            setattr(args, key, value)
    if args.thrust_reference_handoff and not args.thrust_momentum_reference:
        parser.error("--thrust-reference-handoff requires --thrust-momentum-reference")
    if args.native_state_diagnostics and not args.thrust_support_allocator:
        parser.error("native-state-diagnostics requires the private allocator entry")
    if args.thrust_support_allocator:
        if args.jump_controller_executable != "bbot_landing_repair_controller" or not args.native_command_observation or args.wheel_actuation != "effort" or args.wheel_effort_gain != 1.0:
            parser.error("allocator requires private native observer, repair controller and bounded P wheels gain 1")
        if args.thrust_momentum_reference or args.thrust_reference_handoff or args.thrust_support_coordination:
            parser.error("allocator is mutually exclusive with other THRUST experiments")
        if args.observe_repeatability:
            parser.error("allocator is a candidate, not baseline observation")
    if args.record_native_wrench and args.jump_controller_executable != "bbot_landing_repair_controller":
        parser.error("--record-native-wrench requires --jump-controller-executable bbot_landing_repair_controller")
    if args.native_command_observation and not args.record_native_wrench:
        parser.error("--native-command-observation requires --record-native-wrench")
    if args.thrust_momentum_reference and args.jump_controller_executable != "bbot_landing_repair_controller":
        parser.error("thrust-momentum-reference requires --jump-controller-executable bbot_landing_repair_controller")
    if args.thrust_momentum_reference and args.observe_repeatability:
        parser.error("thrust-momentum-reference is not a baseline observation run")
    if args.thrust_support_coordination:
        if args.jump_controller_executable != "bbot_landing_repair_controller":
            parser.error("thrust-support-coordination requires --jump-controller-executable bbot_landing_repair_controller")
        if args.thrust_momentum_reference or args.thrust_reference_handoff:
            parser.error("thrust-support-coordination cannot be combined with the previous momentum-reference experiment")
        if args.observe_repeatability:
            parser.error("thrust-support-coordination is not a baseline observation run")
    if not 1 <= args.jumps_per_session <= 3:
        parser.error("jumps-per-session must be within 1 to 3")
    if not math.isfinite(args.real_time_factor) or args.real_time_factor <= 0.0:
        parser.error("real-time-factor must be finite and positive")
    if not 0.0 <= args.thrust_forward_velocity_kp <= 2.0:
        parser.error("thrust-forward-velocity-kp must be within 0 to 2")
    if args.thrust_fast_rate_correction_from_gate and not args.thrust_release_fast_rate_correction:
        parser.error("thrust-fast-rate-correction-from-gate requires --thrust-release-fast-rate-correction")
    if not 0.0 <= args.post_balance_observe_duration <= 30.0:
        parser.error("post-balance-observe-duration must be within 0 to 30 s")
    if args.command_transport is None:
        args.command_transport = "persistent"
    if args.wheel_actuation == "effort" and not (args.observe_repeatability or args.thrust_support_allocator):
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
            (args.real_time_factor, 1.0),
            (args.jump_height, 0.20),
            (args.air_wheel_sign, -1.0),
            (args.thrust_wheel_max_decel, 8.0),
            (args.thrust_forward_velocity_kp, 0.80),
            (args.thrust_forward_attitude_taper, False),
            (args.thrust_release_fast_rate_correction, False),
            (args.thrust_fast_rate_correction_from_gate, False),
            (args.thrust_forward_speed_prediction, False),
            (args.thrust_wheel_kinematics_compensation, False),
            (args.complete_contact_takeoff_confirmation, False),
            (args.thrust_release_velocity_ratio, 0.70),
            (args.arrest_dynamics_feedforward, False),
        )
        if args.wheel_actuation == "velocity":
            baseline += (
                (args.jump_forward_speed, 0.35),
                (args.jump_takeoff_forward_speed, 0.45),
                (args.air_wheel_extend_kd, 2.50),
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
