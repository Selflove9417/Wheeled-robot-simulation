#!/usr/bin/env python3
"""Phase-aligned alignment table for flat-ground jump baseline trials.

Read-only: extracts state-entry times and per-offset snapshots of the
observable channels required by the baseline aggregation deliverable.
Never sends commands and never edits logs.

Usage:
  python3 aggregate_baseline_alignment.py trial_dir_or_log.csv [more...]
Prints one table per trial plus a campaign aggregate (state times, outcome,
first-divergence candidates are left to the per-trial tables).
"""
import csv
import math
import sys
from pathlib import Path

# Snapshot offsets (s) relative to first TOUCHDOWN_BUFFER row.
OFFSETS = (0.0, 0.2, 0.4, 0.8, 1.5, 2.0, 6.0, 7.0, 8.0)

# Columns captured at every snapshot.
SNAP_COLUMNS = (
    "state_name", "recovery_subphase", "controller_mode", "flight_subphase",
    "pitch", "pitch_rate", "pitch_rate_raw",
    "com_forward_from_axle", "com_lean", "com_lean_rate",
    "com_world_z", "com_world_vz",
    "capture_com_velocity", "capture_world_active", "capture_world_target",
    "gazebo_world_x_dot", "gazebo_world_z_dot",
    "hip_pos_left", "hip_pos_right", "knee_pos_left", "knee_pos_right",
    "hip_vel_left", "hip_vel_right", "knee_vel_left", "knee_vel_right",
    "left_wheel_vel", "right_wheel_vel",
    "wheel_cmd_target", "cmd_x", "wheel_control_height",
    "hip_pos_cmd_left", "knee_pos_cmd_left",
    "torso_hip_command", "hip_common_before_allocation",
    "hip_cmd_left", "hip_cmd_right", "knee_cmd_left", "knee_cmd_right",
    "thrust_release_blend", "wheel_clearance",
)

STATE_ORDER = ("PRE_JUMP", "SQUAT", "THRUST", "FLIGHT", "TOUCHDOWN_BUFFER",
               "RECOVERY", "BALANCE", "EMERGENCY")


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def load_rows(path):
    with open(path, newline="", encoding="utf-8", errors="replace") as stream:
        return list(csv.DictReader(stream))


def state_first_times(rows):
    times = {}
    for row in rows:
        name = row.get("state_name", "")
        if name and name not in times:
            stamp = number(row.get("timestamp"))
            if stamp is not None:
                times[name] = stamp
    return times


def nearest_row(rows, target, lo=0, hi=None):
    """Row with timestamp closest to target (rows sorted by time)."""
    if hi is None:
        hi = len(rows)
    best, best_diff = None, None
    for row in rows[lo:hi]:
        stamp = number(row.get("timestamp"))
        if stamp is None:
            continue
        diff = abs(stamp - target)
        if best_diff is None or diff < best_diff:
            best, best_diff = row, diff
    return best, best_diff


def last_row_with_state(rows, state, before=None):
    best = None
    for row in rows:
        if before is not None:
            stamp = number(row.get("timestamp"))
            if stamp is not None and stamp >= before:
                break
        if row.get("state_name") == state:
            best = row
    return best


def trial_table(log_path):
    rows = load_rows(log_path)
    if not rows:
        return None
    times = state_first_times(rows)
    touchdown = times.get("TOUCHDOWN_BUFFER")
    final = rows[-1]
    summary = {}
    summary_path = Path(str(log_path).replace("_log.csv", "_summary.csv"))
    if summary_path.is_file():
        with open(summary_path, newline="", encoding="utf-8", errors="replace") as stream:
            entries = list(csv.DictReader(stream))
        if entries:
            summary = entries[-1]

    print(f"\n=== {Path(log_path).name} ===")
    print("state first-entry times (sim s):")
    for name in sorted(times, key=lambda n: times[n]):
        print(f"  {name:18s} {times[name]:9.3f}")
    print(f"  final state: {final.get('state_name','')}  "
          f"exit_code={summary.get('exit_code','')}  "
          f"recovery_completed={summary.get('recovery_completed','')}")
    print("  summary: apex_com_z_delta=%s takeoff_com_vz=%s takeoff_com_vx=%s" % (
        summary.get("apex_com_z_delta", ""), summary.get("takeoff_com_vz", ""),
        summary.get("takeoff_com_vx", "")))
    if summary.get("reason"):
        print(f"  reason: {summary.get('reason','')}")

    anchors = []
    thrust_end = last_row_with_state(rows, "THRUST", before=touchdown)
    if thrust_end is not None:
        anchors.append(("THRUST_end", number(thrust_end.get("timestamp")), thrust_end))
    flight_entry = nearest_row(rows, times["FLIGHT"]) if "FLIGHT" in times else (None, None)
    if flight_entry[0] is not None:
        anchors.append(("FLIGHT_entry", times.get("FLIGHT"), flight_entry[0]))
    if touchdown is not None:
        anchors.append(("TD", touchdown, nearest_row(rows, touchdown)[0]))
        for offset in OFFSETS[1:]:
            row, _ = nearest_row(rows, touchdown + offset)
            if row is not None:
                anchors.append((f"TD+{offset:g}", touchdown + offset, row))

    header = ["anchor", "t"] + list(SNAP_COLUMNS)
    print("\nsnapshots (nearest 5 ms row):")
    print(",".join(header))
    for label, target, row in anchors:
        if row is None:
            continue
        fields = [label, f"{number(row.get('timestamp')):.3f}"]
        for col in SNAP_COLUMNS:
            value = row.get(col, "")
            fields.append(value if value != "" else "-")
        print(",".join(fields))
    return {"log": str(log_path), "times": times, "summary": summary,
            "final_state": final.get("state_name", "")}


def main():
    paths = []
    for arg in sys.argv[1:]:
        path = Path(arg)
        if path.is_dir():
            paths.extend(sorted(path.glob("*_log.csv")))
        elif path.is_file():
            paths.append(path)
        else:
            print(f"skip missing {arg}", file=sys.stderr)
    if not paths:
        print("no logs given", file=sys.stderr)
        return 1
    results = [trial_table(path) for path in paths]
    print("\n=== aggregate ===")
    print("log,state_times(TD,RECOVERY,BALANCE),final_state,exit_code,recovery_completed")
    for res in results:
        if res is None:
            continue
        times = res["times"]
        summary = res["summary"]
        print(f"{Path(res['log']).parent.name}/{Path(res['log']).name},"
              f"{times.get('TOUCHDOWN_BUFFER', float('nan')):.3f},"
              f"{times.get('RECOVERY', float('nan')):.3f},"
              f"{times.get('BALANCE', float('nan')):.3f},"
              f"{res['final_state']},{summary.get('exit_code','')},"
              f"{summary.get('recovery_completed','')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
