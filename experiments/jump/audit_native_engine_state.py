#!/usr/bin/env python3
"""Audit direct Physics before/after state and derive native-state CSV copies.

This tool is offline and read-only with respect to its inputs. It uses the
shared (sim_time_ns, iteration, dt_ns) UpdateInfo key to pair before_step and
after_step rows; it makes no claim about the absolute physical timestamp
represented by that key. Invalid/missing engine state remains invalid in the
derived copies and every source row is preserved.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

STEP_NS = 1_000_000
PHASES = ("before_step", "after_step")
LINKS = (
    "flat_jump_world::bbot::base_link",
    "flat_jump_world::bbot::link_004",
    "flat_jump_world::bbot::link_007",
)
JOINTS = tuple(f"flat_jump_world::bbot::link_{i:03d}_joint" for i in (2, 3, 4, 5, 6, 7))
EXPECTED = {("link", name) for name in LINKS} | {("joint", name) for name in JOINTS}
JOINT_INDEX_TO_NAME = {
    0: "link_002_joint", 1: "link_003_joint", 2: "link_004_joint",
    3: "link_005_joint", 4: "link_006_joint", 5: "link_007_joint",
}
ENGINE_REQUIRED = {
    "sim_time_ns", "iteration", "dt_ns", "phase", "entity_type", "entity_name",
    "entity_id", "entity_present", "time_valid", "position_valid", "velocity_valid",
    "physical_state_time_ns", "wall_steady_time_ns",
    "position_x", "position_y", "position_z", "quat_w", "quat_x", "quat_y", "quat_z",
    "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz",
    "joint_dof", "joint_position_0", "joint_velocity_0", "reason",
}
LINK_FIELDS = (
    "position_x", "position_y", "position_z", "quat_w", "quat_x", "quat_y", "quat_z",
    "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz",
)
MODEL_DOF_NAMES = ("base_forward", "base_z", "roll_x", "hip_left", "knee_left",
                   "wheel_left", "hip_right", "knee_right", "wheel_right")


def flag(value: Any) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "valid"}


def integer(value: Any) -> int | None:
    try:
        text = str(value).strip()
        if not text:
            return None
        return int(text, 10)
    except (TypeError, ValueError, OverflowError):
        return None


def number(value: Any) -> float | None:
    try:
        text = str(value).strip()
        if not text:
            return None
        result = float(text)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        return [dict(row) for row in reader], sorted(fields)


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = list(dict.fromkeys(k for row in rows for k in row))
    if not fields:
        fields = ["audit_empty"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fields})


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def engine_state_errors(row: dict[str, str], required_missing: set[str],
                        expected_state_ns: int | None = None) -> list[str]:
    errors: list[str] = []
    if required_missing:
        errors.append("engine_schema_missing_columns")
    if row.get("phase", "") not in PHASES:
        errors.append("unknown_phase")
    entity_type = row.get("entity_type", "")
    name = row.get("entity_name", "")
    if (entity_type, name) not in EXPECTED:
        errors.append("unexpected_entity_type_or_scope")
    if integer(row.get("entity_id")) is None:
        errors.append("invalid_entity_id")
    if not flag(row.get("entity_present")):
        errors.append("entity_missing")
    if not flag(row.get("time_valid")):
        errors.append("time_invalid" + (":" + row.get("reason", "") if row.get("reason") else ""))
    if integer(row.get("dt_ns")) != STEP_NS:
        errors.append("dt_not_1ms")
    if expected_state_ns is None:
        row_key = key(row)
        if row_key and row.get("phase") in PHASES:
            expected_state_ns = row_key[1] - STEP_NS if row.get("phase") == "before_step" else row_key[1]
    state_ns = integer(row.get("physical_state_time_ns"))
    if state_ns is None:
        errors.append("physical_state_time_missing_or_invalid")
    elif expected_state_ns is not None and state_ns != expected_state_ns:
        errors.append("physical_state_time_phase_mismatch")
    wall_ns = integer(row.get("wall_steady_time_ns"))
    if wall_ns is None or wall_ns < 0:
        errors.append("wall_steady_time_missing_or_invalid")
    if not flag(row.get("position_valid")):
        errors.append("position_invalid")
    if not flag(row.get("velocity_valid")):
        errors.append("velocity_invalid")
    if (entity_type, name) in EXPECTED and entity_type == "link":
        vals = [number(row.get(k)) for k in LINK_FIELDS]
        if any(v is None for v in vals):
            errors.append("nonfinite_or_missing_link_pose_velocity")
        else:
            qw, qx, qy, qz = (number(row.get(k)) for k in ("quat_w", "quat_x", "quat_y", "quat_z"))
            qnorm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if qnorm < 1e-12:
                errors.append("zero_quaternion")
            elif abs(qnorm - 1.0) > 1e-6:
                errors.append("quaternion_norm_outside_1e-6")
    if (entity_type, name) in EXPECTED and entity_type == "joint":
        if integer(row.get("joint_dof")) != 1:
            errors.append("joint_dof_not_one")
        if number(row.get("joint_position_0")) is None or number(row.get("joint_velocity_0")) is None:
            errors.append("nonfinite_or_missing_joint_state")
    if row.get("reason", "").strip():
        errors.append("source_reason:" + row["reason"].strip())
    return errors


def key(row: dict[str, str]) -> tuple[int, int] | None:
    it, ns = integer(row.get("iteration")), integer(row.get("sim_time_ns"))
    if it is None or ns is None:
        return None
    return it, ns


def source_key(row: dict[str, str]) -> tuple[int, int] | None:
    it = integer(row.get("physics_iteration", row.get("iteration")))
    ns = integer(row.get("sim_time_ns"))
    if it is None or ns is None:
        return None
    return it, ns


def clock_audit(rows: list[dict[str, str]], phase: str | None = None,
                source: bool = False) -> dict[str, int]:
    ordered: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    out = Counter()
    for row in rows:
        if phase is not None and row.get("phase") != phase:
            continue
        k = source_key(row) if source else key(row)
        if k is None:
            out["unkeyed_rows"] += 1
            continue
        if integer(row.get("dt_ns")) != STEP_NS:
            out["dt_errors"] += 1
        if k not in seen:
            seen.add(k)
            ordered.append(k)
    for (pi, pn), (ci, cn) in zip(ordered, ordered[1:]):
        if cn < pn:
            out["time_reversals"] += 1
        if ci < pi:
            out["iteration_reversals"] += 1
        if cn - pn != STEP_NS:
            out["time_step_errors"] += 1
        if ci - pi != 1:
            out["iteration_step_errors"] += 1
    return {k: int(out[k]) for k in ("unkeyed_rows", "dt_errors", "time_reversals",
                                     "iteration_reversals", "time_step_errors", "iteration_step_errors")}


def index_source(rows: list[dict[str, str]], kind: str) -> tuple[dict[tuple[int, int], list[dict[str, str]]], Counter]:
    grouped: dict[tuple[int, int], list[dict[str, str]]] = defaultdict(list)
    problems: Counter = Counter()
    for row in rows:
        k = source_key(row)
        if k is None:
            problems["unkeyed_rows"] += 1
            continue
        grouped[k].append(row)
    for k, group in grouped.items():
        if kind == "native":
            indices = [integer(r.get("joint_index")) for r in group]
            if len(group) != 6 or any(i is None for i in indices) or sorted(i for i in indices if i is not None) != list(range(6)):
                problems["native_joint_rows_missing_extra_or_duplicate"] += 1
            expected_names = [JOINT_INDEX_TO_NAME.get(i) for i in indices]
            if any(r.get("joint_name") != n for r, n in zip(group, expected_names)):
                problems["native_joint_name_index_mismatch"] += 1
        elif len(group) != 1:
            problems[f"duplicate_{kind}_frames"] += 1
    return dict(grouped), problems


def record_index(rows: list[dict[str, str]], identity_fields: tuple[str, ...], phase: str | None = None) -> dict[tuple[int, int], dict[tuple[str, ...], list[dict[str, str]]]]:
    result: dict[tuple[int, int], dict[tuple[str, ...], list[dict[str, str]]]] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        if phase is not None and row.get("phase") != phase:
            continue
        k = key(row)
        if k is None:
            continue
        identity = tuple(row.get(f, "") for f in identity_fields)
        result[k][identity].append(row)
    return result


def valid_engine_row(row: dict[str, str] | None, required_missing: set[str]) -> tuple[bool, list[str]]:
    if row is None:
        return False, ["missing_entity_row"]
    errors = engine_state_errors(row, required_missing)
    return not errors, errors


def snapshot(phase_index: dict[tuple[str, str], list[dict[str, str]]],
             k: tuple[int, int], phase: str, required_missing: set[str]) -> tuple[dict[tuple[str, str], dict[str, str]], list[str]]:
    states: dict[tuple[str, str], dict[str, str]] = {}
    errors: list[str] = []
    for identity in sorted(EXPECTED):
        matches = phase_index.get(identity, [])
        if len(matches) != 1:
            errors.append(f"{phase}:{identity[0]}:{identity[1]}:expected_one_row_found_{len(matches)}")
            continue
        row = matches[0]
        states[identity] = row
        expected_state_ns = k[1] - STEP_NS if phase == "before_step" else k[1]
        row_errors = engine_state_errors(row, required_missing, expected_state_ns)
        if row_errors:
            errors.extend(f"{phase}:{identity[0]}:{identity[1]}:{e}" for e in row_errors)
    for identity, matches in phase_index.items():
        if identity not in EXPECTED:
            errors.append(f"{phase}:unexpected_identity:{identity[0]}:{identity[1]}")
        if len(matches) > 1:
            errors.append(f"{phase}:duplicate_identity:{identity[0]}:{identity[1]}")
    return states, errors


def state_link(row: dict[str, str]) -> tuple[list[float], list[float], list[float]]:
    pos = [float(row[f"position_{axis}"]) for axis in "xyz"]
    quat = [float(row[f"quat_{axis}"]) for axis in ("w", "x", "y", "z")]
    vel = [float(row[f"linear_v{axis}"]) for axis in "xyz"] + [float(row[f"angular_v{axis}"]) for axis in "xyz"]
    return pos, quat, vel


def state_joint(row: dict[str, str]) -> tuple[float, float]:
    return float(row["joint_position_0"]), float(row["joint_velocity_0"])


def model_qv(states: dict[tuple[str, str], dict[str, str]]) -> tuple[list[float], list[float]]:
    base = states[("link", LINKS[0])]
    pos, quat, velocity = state_link(base)
    roll = 2.0 * math.atan2(quat[1], quat[0])
    q = [pos[1], pos[2], roll]
    v = [velocity[1], velocity[2], velocity[3]]
    by_name = {name.rsplit("::", 1)[-1]: states[("joint", name)] for name in JOINTS}
    joint_qv = {name: state_joint(row) for name, row in by_name.items()}
    q.extend(joint_qv[JOINT_INDEX_TO_NAME[i]][0] for i in range(6))
    v.extend(joint_qv[JOINT_INDEX_TO_NAME[i]][1] for i in range(6))
    return q, v


def safe_set(row: dict[str, Any], field: str, value: Any) -> None:
    row[field] = value


def valid_entity(states: dict[tuple[str, str], dict[str, str]], kind: str, name: str,
                 required_missing: set[str]) -> bool:
    row = states.get((kind, name))
    return valid_engine_row(row, required_missing)[0]


def invalidate_native_row(row: dict[str, Any]) -> None:
    row.update({
        "before_physics_joint_state_valid": "0", "before_physics_joint_position": "nan",
        "before_physics_joint_velocity": "nan", "joint_valid": "0", "state_valid": "0",
        "joint_position": "nan", "joint_velocity": "nan",
        "before_physics_base_pose_valid": "0", "before_physics_base_x": "nan",
        "before_physics_base_y": "nan", "before_physics_base_z": "nan",
        "before_physics_base_qx": "nan", "before_physics_base_qy": "nan",
        "before_physics_base_qz": "nan", "before_physics_base_qw": "nan",
        "before_physics_base_velocity_valid": "0",
        "before_physics_base_world_vx": "nan", "before_physics_base_world_vy": "nan",
        "before_physics_base_world_vz": "nan", "before_physics_base_world_wx": "nan",
        "before_physics_base_world_wy": "nan", "before_physics_base_world_wz": "nan",
        "post_base_velocity_valid": "0", "post_base_world_vx": "nan",
        "post_base_world_vy": "nan", "post_base_world_vz": "nan",
        "post_base_world_wx": "nan", "post_base_world_wy": "nan", "post_base_world_wz": "nan",
    })


def replace_native(row: dict[str, Any], before: dict[tuple[str, str], dict[str, str]],
                   after: dict[tuple[str, str], dict[str, str]],
                   required_missing: set[str]) -> dict[str, bool]:
    index = integer(row.get("joint_index"))
    joint_short = JOINT_INDEX_TO_NAME.get(index)
    joint_full = f"flat_jump_world::bbot::{joint_short}" if joint_short else ""
    bbase, abase = before.get(("link", LINKS[0])), after.get(("link", LINKS[0]))
    bj = before.get(("joint", joint_full)), after.get(("joint", joint_full))
    bpose_ok = valid_entity(before, "link", LINKS[0], required_missing)
    bvel_ok = bpose_ok  # The source has a single link state validity contract for pose and velocity.
    bjoint_ok = bool(joint_full) and valid_entity(before, "joint", joint_full, required_missing)
    ajoint_ok = bool(joint_full) and valid_entity(after, "joint", joint_full, required_missing)
    avel_ok = valid_entity(after, "link", LINKS[0], required_missing)
    failures = []
    for phase, state_ok in (("before_step", bpose_ok and bvel_ok and bjoint_ok),
                            ("after_step", ajoint_ok and avel_ok)):
        if not state_ok:
            failures.append(f"{phase}_state_missing_or_invalid")
    row["native_engine_before_state_valid"] = str(int(bpose_ok and bvel_ok and bjoint_ok))
    row["native_engine_after_state_valid"] = str(int(ajoint_ok and avel_ok))
    row["native_engine_state_reason"] = "|".join(failures)
    if not (bpose_ok and bvel_ok and bjoint_ok and ajoint_ok and avel_ok):
        invalidate_native_row(row)
    if bbase and bpose_ok:
        pos, quat, vel = state_link(bbase)
        row.update({
            "before_physics_base_pose_valid": "1", "before_physics_base_x": repr(pos[0]),
            "before_physics_base_y": repr(pos[1]), "before_physics_base_z": repr(pos[2]),
            "before_physics_base_qw": repr(quat[0]), "before_physics_base_qx": repr(quat[1]),
            "before_physics_base_qy": repr(quat[2]), "before_physics_base_qz": repr(quat[3]),
        })
        row["before_physics_base_velocity_valid"] = "1"
        for f, v in zip(("vx", "vy", "vz", "wx", "wy", "wz"), vel):
            row[f"before_physics_base_world_{f}"] = repr(v)
    if bjoint_ok and bj[0]:
        q, v = state_joint(bj[0])
        row.update({"before_physics_joint_state_valid": "1",
                    "before_physics_joint_position": repr(q), "before_physics_joint_velocity": repr(v)})
    if ajoint_ok and bj[1]:
        q, v = state_joint(bj[1])
        row.update({"joint_valid": "1", "state_valid": "1", "joint_position": repr(q), "joint_velocity": repr(v)})
    if avel_ok and abase:
        _, _, vel = state_link(abase)
        row["post_base_velocity_valid"] = "1"
        for f, v in zip(("vx", "vy", "vz", "wx", "wy", "wz"), vel):
            row[f"post_base_world_{f}"] = repr(v)
    # Compatibility label for the existing offline response audit. It denotes
    # the derived before_step value and does not rewrite its UpdateInfo key.
    row["before_physics_phase"] = "before_physics_update"
    return {"before_base_pose": bpose_ok, "before_base_velocity": bvel_ok,
            "before_joint": bjoint_ok, "after_joint": ajoint_ok, "after_base_velocity": avel_ok}


def append_error(existing: str, reason: str) -> str:
    values = [x for x in (existing.strip(), reason) if x]
    return ";".join(values)


def replace_geometry(row: dict[str, Any], after: dict[tuple[str, str], dict[str, str]],
                     required_missing: set[str]) -> bool:
    original_frame_valid = row.get("frame_valid", "")
    original_error = row.get("error", "")
    names = (LINKS[0], LINKS[1], LINKS[2])
    joints_short = ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")
    ok = all(valid_entity(after, "link", name, required_missing) for name in names) and all(
        valid_entity(after, "joint", f"flat_jump_world::bbot::{name}", required_missing) for name in joints_short)
    row["native_engine_after_state_valid"] = str(int(ok))
    row["native_engine_state_reason"] = "" if ok else "after_step_state_missing_or_invalid"
    if not ok:
        row["frame_valid"] = "0"
        row["error"] = append_error(original_error, "native_engine_after_step_missing_or_invalid")
        for f in ("base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw",
                  "left_wheel_x", "left_wheel_y", "left_wheel_z", "right_wheel_x", "right_wheel_y", "right_wheel_z",
                  "hip_left", "knee_left", "hip_right", "knee_right"):
            row[f] = "nan"
        return False
    base, left, right = (state_link(after[("link", n)]) for n in names)
    row.update(dict(zip(("base_x", "base_y", "base_z"), map(repr, base[0]))))
    row.update(dict(zip(("base_qw", "base_qx", "base_qy", "base_qz"), map(repr, base[1]))))
    for prefix, value in (("left_wheel", left[0]), ("right_wheel", right[0])):
        row.update(dict(zip((prefix + "_x", prefix + "_y", prefix + "_z"), map(repr, value))))
    for field, short in zip(("hip_left", "knee_left", "hip_right", "knee_right"), joints_short):
        row[field] = repr(state_joint(after[("joint", f"flat_jump_world::bbot::{short}")])[0])
    # Engine values can replace ECM-cached coordinates, but cannot repair an
    # independently invalid/error-marked source geometry frame.
    row["frame_valid"] = original_frame_valid if flag(original_frame_valid) else "0"
    row["error"] = original_error
    return True


def compare_diagnostic(acc: dict[str, list[float]], name: str, a: float | None, b: float | None) -> None:
    if a is not None and b is not None:
        acc[name].append(a - b)


def diagnostics(native_rows: list[dict[str, str]], geometry_rows: list[dict[str, str]],
                native_by_key: dict[tuple[int, int], list[dict[str, str]]],
                geometry_by_key: dict[tuple[int, int], list[dict[str, str]]],
                engine: dict[tuple[int, int], dict[str, dict[tuple[str, str], dict[str, str]]]],
                required_missing: set[str]) -> dict[str, Any]:
    diffs: dict[str, list[float]] = defaultdict(list)
    for k, rows in native_by_key.items():
        if len(rows) != 6:
            continue
        pair = engine.get(k, {})
        before, after = pair.get("before_step", {}), pair.get("after_step", {})
        bbase, abase = before.get(("link", LINKS[0])), after.get(("link", LINKS[0]))
        if bbase and valid_entity(before, "link", LINKS[0], required_missing):
            pos, quat, vel = state_link(bbase)
            r = rows[0]
            for f, v in zip(("x", "y", "z"), pos): compare_diagnostic(diffs, f"base_before_pos_{f}", v, number(r.get(f"before_physics_base_{f}")))
            for f, v in zip(("qw", "qx", "qy", "qz"), quat): compare_diagnostic(diffs, f"base_before_quat_{f}", v, number(r.get(f"before_physics_base_{f}")))
            for f, v in zip(("vx", "vy", "vz", "wx", "wy", "wz"), vel): compare_diagnostic(diffs, f"base_before_vel_{f}", v, number(r.get(f"before_physics_base_world_{f}")))
        if abase and valid_entity(after, "link", LINKS[0], required_missing):
            _, _, vel = state_link(abase)
            for f, v in zip(("vx", "vy", "vz", "wx", "wy", "wz"), vel): compare_diagnostic(diffs, f"base_after_vel_{f}", v, number(rows[0].get(f"post_base_world_{f}")))
        for r in rows:
            idx = integer(r.get("joint_index")); short = JOINT_INDEX_TO_NAME.get(idx)
            if not short: continue
            name = f"flat_jump_world::bbot::{short}"
            for phase, fields in (("before_step", ("before_physics_joint_position", "before_physics_joint_velocity")),
                                  ("after_step", ("joint_position", "joint_velocity"))):
                state = pair.get(phase, {}).get(("joint", name))
                if state and valid_entity(pair.get(phase, {}), "joint", name, required_missing):
                    q, v = state_joint(state)
                    compare_diagnostic(diffs, f"{short}_{phase}_q", q, number(r.get(fields[0])))
                    compare_diagnostic(diffs, f"{short}_{phase}_v", v, number(r.get(fields[1])))
    for k, rows in geometry_by_key.items():
        if len(rows) != 1: continue
        after = engine.get(k, {}).get("after_step", {})
        for short, field_prefix in ((LINKS[0], "base"), (LINKS[1], "left_wheel"), (LINKS[2], "right_wheel")):
            state = after.get(("link", short))
            if not state or not valid_entity(after, "link", short, required_missing): continue
            pos, _, _ = state_link(state)
            for axis, value in zip("xyz", pos): compare_diagnostic(diffs, f"geometry_{field_prefix}_{axis}", value, number(rows[0].get(f"{field_prefix}_{axis}")))
        for field, short in zip(("hip_left", "knee_left", "hip_right", "knee_right"),
                                ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")):
            name = f"flat_jump_world::bbot::{short}"
            state = after.get(("joint", name))
            if state and valid_entity(after, "joint", name, required_missing):
                compare_diagnostic(diffs, f"geometry_{field}", state_joint(state)[0], number(rows[0].get(field)))
    return {name: {"count": len(vals), "RMS": math.sqrt(sum(x*x for x in vals)/len(vals)) if vals else None,
                   "max_abs": max(map(abs, vals)) if vals else None,
                   "meaning": "engine minus original ECM cache; diagnostic only, equality is not a gate"}
            for name, vals in sorted(diffs.items())}


def audit(args: argparse.Namespace) -> dict[str, Any]:
    output: Path = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    engine_rows, engine_fields = read_csv(args.engine_csv)
    native_rows, native_fields = read_csv(args.native_wrench_csv)
    contact_rows, contact_fields = read_csv(args.contact_frames_csv)
    geometry_rows, geometry_fields = read_csv(args.geometry_csv)
    missing_engine_fields = ENGINE_REQUIRED - set(engine_fields)

    engine_index: dict[tuple[int, int], dict[str, dict[tuple[str, str], list[dict[str, str]]]]] = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    engine_ids: dict[tuple[str, str], set[str]] = defaultdict(set)
    engine_row_audit = []
    global_engine_problems: Counter = Counter()
    last_wall_by_phase: dict[str, int] = {}
    wall_reverse_keys: set[tuple[int, int]] = set()
    wall_by_key_phase: dict[tuple[int, int], dict[str, list[int]]] = defaultdict(lambda: defaultdict(list))
    for row_num, row in enumerate(engine_rows, start=2):
        k = key(row)
        ph = row.get("phase", "")
        if k is None:
            errs = ["unkeyed_engine_row"]
            global_engine_problems["unkeyed_rows"] += 1
        else:
            expected_state_ns = k[1] - STEP_NS if ph == "before_step" else k[1] if ph == "after_step" else None
            errs = engine_state_errors(row, missing_engine_fields, expected_state_ns)
            identity = (row.get("entity_type", ""), row.get("entity_name", ""))
            engine_index[k][ph][identity].append(row)
            if identity in EXPECTED and row.get("entity_id", "").strip():
                engine_ids[identity].add(row["entity_id"].strip())
            wall_ns = integer(row.get("wall_steady_time_ns"))
            if ph in PHASES and wall_ns is not None:
                wall_by_key_phase[k][ph].append(wall_ns)
                previous_wall = last_wall_by_phase.get(ph)
                if previous_wall is not None and wall_ns < previous_wall:
                    errs.append("wall_clock_reversal_within_phase")
                    global_engine_problems["wall_clock_reversal_within_phase"] += 1
                    wall_reverse_keys.add(k)
                last_wall_by_phase[ph] = wall_ns
        if missing_engine_fields:
            errs.append("missing_engine_schema_columns")
        engine_row_audit.append({"source_row": row_num, "iteration": row.get("iteration", ""),
                                 "sim_time_ns": row.get("sim_time_ns", ""), "phase": ph,
                                 "entity_type": row.get("entity_type", ""), "entity_name": row.get("entity_name", ""),
                                 "entity_id": row.get("entity_id", ""), "in_hold_window": 0,
                                 "row_valid": int(not errs), "failures": "|".join(errs)})
    for identity, ids in engine_ids.items():
        if len(ids) > 1:
            global_engine_problems["entity_id_changed_for_identity"] += 1
    id_owners: dict[str, set[tuple[str, str]]] = defaultdict(set)
    for identity, ids in engine_ids.items():
        for entity_id in ids: id_owners[entity_id].add(identity)
    if any(len(owners) > 1 for owners in id_owners.values()):
        global_engine_problems["entity_id_aliases_multiple_names"] += 1

    native_by_key, native_problems = index_source(native_rows, "native")
    contact_by_key, contact_problems = index_source(contact_rows, "contact")
    geometry_by_key, geometry_problems = index_source(geometry_rows, "geometry")
    engine_clocks = {p: clock_audit(engine_rows, phase=p) for p in PHASES}
    source_clocks = {
        "native": clock_audit(native_rows, source=True),
        "contacts": clock_audit(contact_rows, source=True),
        "geometry": clock_audit(geometry_rows, source=True),
    }

    window_valid = args.hold_start_ns >= 0 and args.hold_end_ns > args.hold_start_ns and \
                   (args.hold_end_ns - args.hold_start_ns) % STEP_NS == 0
    expected_ns = list(range(args.hold_start_ns, args.hold_end_ns, STEP_NS)) if window_valid else []
    start_iters = {k[0] for k in engine_index if k[1] == args.hold_start_ns}
    start_iteration = min(start_iters) if len(start_iters) == 1 else None
    expected = [(start_iteration + i, ns) for i, ns in enumerate(expected_ns)] if start_iteration is not None else []
    if not window_valid:
        global_engine_problems["hold_window_invalid"] += 1
    if start_iteration is None:
        global_engine_problems["hold_start_iteration_missing_or_ambiguous"] += 1

    # Mark every source engine row with the fixed-hold membership; keep startup
    # rows present and report their validity without making them hold samples.
    for ra in engine_row_audit:
        it, ns = integer(ra["iteration"]), integer(ra["sim_time_ns"])
        ra["in_hold_window"] = int(it is not None and ns is not None and
                                   args.hold_start_ns <= ns < args.hold_end_ns)

    frames = []
    gate_failures: Counter = Counter()
    valid_windows: dict[tuple[int, int], dict[str, dict[tuple[str, str], dict[str, str]]]] = {}
    whole_trace_keys = set(engine_index) | set(native_by_key) | set(contact_by_key) | set(geometry_by_key)
    expected_set = set(expected)
    key_list = sorted(whole_trace_keys | expected_set, key=lambda x: (x[1], x[0]))
    for k in key_list:
        in_window = k in expected_set
        frame_errors: list[str] = []
        phase_states: dict[str, dict[tuple[str, str], dict[str, str]]] = {}
        phase_counts: dict[str, int] = {}
        for phase in PHASES:
            states, errs = snapshot(engine_index.get(k, {}).get(phase, {}), k, phase, missing_engine_fields)
            phase_states[phase] = states
            phase_counts[phase] = sum(len(v) for v in engine_index.get(k, {}).get(phase, {}).values())
            frame_errors.extend(errs)
        if set(engine_index.get(k, {})) - set(PHASES):
            frame_errors.append("unexpected_phase_rows")
        if k in wall_reverse_keys:
            frame_errors.append("wall_clock_reversal_within_phase")
        before_walls = wall_by_key_phase.get(k, {}).get("before_step", [])
        after_walls = wall_by_key_phase.get(k, {}).get("after_step", [])
        if not before_walls or not after_walls or max(before_walls) > min(after_walls):
            frame_errors.append("wall_clock_before_after_order_invalid")
        native_group = native_by_key.get(k, [])
        native_indices = [integer(r.get("joint_index")) for r in native_group]
        if len(native_group) != 6 or any(i is None for i in native_indices) or sorted(i for i in native_indices if i is not None) != list(range(6)):
            frame_errors.append("native_source_missing_extra_or_duplicate_joint_rows")
        elif any(r.get("joint_name") != JOINT_INDEX_TO_NAME[integer(r.get("joint_index"))] for r in native_group):
            frame_errors.append("native_source_joint_name_index_mismatch")
        contact_group = contact_by_key.get(k, [])
        geometry_group = geometry_by_key.get(k, [])
        if len(contact_group) != 1: frame_errors.append("contact_source_frame_missing_or_duplicate")
        if len(geometry_group) != 1: frame_errors.append("geometry_source_frame_missing_or_duplicate")
        for label, rows in (("native", native_group), ("contact", contact_group), ("geometry", geometry_group)):
            if any(integer(r.get("dt_ns")) != STEP_NS for r in rows):
                frame_errors.append(f"{label}_source_dt_not_1ms")
        if in_window and not frame_errors:
            valid_windows[k] = phase_states
        if in_window and frame_errors:
            gate_failures.update(frame_errors)
        frames.append({"iteration": k[0], "sim_time_ns": k[1], "dt_ns": STEP_NS,
                       "in_hold_window": int(in_window),
                       "before_row_count": phase_counts.get("before_step", 0),
                       "after_row_count": phase_counts.get("after_step", 0),
                       "before_complete_valid": int(not any(x.startswith("before_step:") for x in frame_errors)),
                       "after_complete_valid": int(not any(x.startswith("after_step:") for x in frame_errors)),
                       "native_rows": len(native_by_key.get(k, [])),
                       "contact_rows": len(contact_by_key.get(k, [])),
                       "geometry_rows": len(geometry_by_key.get(k, [])),
                       "frame_valid": int(not frame_errors), "failures": "|".join(frame_errors)})

    # Whole-trace clocks are diagnostics; in-window clocks are hard gates.
    def in_window_clock_error(rows: list[dict[str, str]], phase: str | None = None,
                              source: bool = False) -> Counter:
        selected = []
        for r in rows:
            if phase and r.get("phase") != phase: continue
            k = source_key(r) if source else key(r)
            if k and args.hold_start_ns <= k[1] < args.hold_end_ns:
                selected.append(r)
        return Counter(clock_audit(selected, phase=phase, source=source))
    for phase in PHASES:
        e = in_window_clock_error(engine_rows, phase=phase)
        for name, count in e.items():
            if count: gate_failures[f"engine_{phase}_{name}"] += count
    for label, rows in (("native", native_rows), ("contacts", contact_rows), ("geometry", geometry_rows)):
        e = in_window_clock_error(rows, source=True)
        for name, count in e.items():
            if count: gate_failures[f"{label}_{name}"] += count
    if global_engine_problems.get("entity_id_changed_for_identity") or global_engine_problems.get("entity_id_aliases_multiple_names"):
        gate_failures.update({name: n for name, n in global_engine_problems.items() if name.startswith("entity_id_")})
    if missing_engine_fields:
        gate_failures["engine_schema_missing_columns"] += len(missing_engine_fields)

    # Continuity is measured in both generalized position and velocity; each
    # adjacent pair must come from valid engine after/before phase snapshots.
    continuity = {"position": [], "velocity": []}
    continuity_missing = 0
    for a, b in zip(expected, expected[1:]):
        sa, sb = valid_windows.get(a), valid_windows.get(b)
        if not sa or not sb:
            continuity_missing += 1
            continue
        try:
            qa, va = model_qv(sa["after_step"])
            qb, vb = model_qv(sb["before_step"])
            continuity["position"].extend(abs(x-y) for x, y in zip(qa, qb))
            continuity["velocity"].extend(abs(x-y) for x, y in zip(va, vb))
        except (KeyError, ValueError, TypeError):
            continuity_missing += 1
    continuity_max = {k: (max(v) if v else None) for k, v in continuity.items()}
    continuity_ok = (len(expected) > 1 and continuity_missing == 0 and
                     all(v and max(v) <= args.continuity_tolerance for v in continuity.values()))
    if not continuity_ok:
        gate_failures["engine_after_to_next_before_continuity_failed_or_missing"] += 1

    # Derived CSVs preserve all original rows and all non-state columns. Engine
    # invalid/missing rows overwrite states with NaN plus false validity flags.
    derived_native = []
    for original in native_rows:
        row = dict(original); k = source_key(original)
        pair = engine_index.get(k, {}) if k else {}
        before, _ = snapshot(pair.get("before_step", {}), k or (0, 0), "before_step", missing_engine_fields)
        after, _ = snapshot(pair.get("after_step", {}), k or (0, 0), "after_step", missing_engine_fields)
        replace_native(row, before, after, missing_engine_fields)
        derived_native.append(row)
    derived_geometry = []
    for original in geometry_rows:
        row = dict(original); k = source_key(original)
        after, _ = snapshot(engine_index.get(k, {}).get("after_step", {}) if k else {}, k or (0, 0), "after_step", missing_engine_fields)
        replace_geometry(row, after, missing_engine_fields)
        derived_geometry.append(row)

    write_csv(output / "engine_row_audit.csv", engine_row_audit)
    write_csv(output / "frame_key_audit.csv", frames)
    if native_rows:
        native_output_fields = list(dict.fromkeys(
            list(native_rows[0].keys()) + [field for row in derived_native for field in row if field not in native_rows[0]]))
        write_csv(output / "derived_native_wrench.csv", derived_native, native_output_fields)
    else:
        write_csv(output / "derived_native_wrench.csv", [], native_fields)
    if geometry_rows:
        geometry_output_fields = list(dict.fromkeys(
            list(geometry_rows[0].keys()) + [field for row in derived_geometry for field in row if field not in geometry_rows[0]]))
        write_csv(output / "derived_geometry.csv", derived_geometry, geometry_output_fields)
    else:
        write_csv(output / "derived_geometry.csv", [], geometry_fields)

    # ECM-vs-engine differences are explicitly diagnostic only.
    engine_for_diagnostics = {k: {p: {identity: values[0] for identity, values in phase.items() if len(values) == 1}
                                   for p, phase in phases.items()} for k, phases in engine_index.items()}
    cache_diff = diagnostics(native_rows, geometry_rows, native_by_key, geometry_by_key,
                             engine_for_diagnostics, missing_engine_fields)
    gate_ok = bool(expected) and len(valid_windows) == len(expected) and not gate_failures
    metadata = {
        "audit": "direct_native_physics_engine_state",
        "gate": "PASS" if gate_ok else "FAIL",
        "scope": "fixed hold native state integrity only; no controller/model/actuator qualification",
        "phase_pairing": "sim_time_ns/iteration is the shared UpdateInfo key for one physics step; SimulationRunner advances simTime before UpdateSystems, so the key is the step-end timestamp: before_step physical state time is sim_time_ns-dt_ns and after_step physical state time is sim_time_ns; phases bracket world->Step",
        "window": {"start_ns": args.hold_start_ns, "end_ns_exclusive": args.hold_end_ns,
                   "duration_s": (args.hold_end_ns - args.hold_start_ns) * 1e-9 if window_valid else 0,
                   "expected_steps": len(expected), "valid_engine_steps": len(valid_windows),
                   "all_expected_two_phase_nine_entity_frames": "PASS" if gate_ok else "FAIL"},
        "schema": {"missing_columns": sorted(missing_engine_fields),
                   "entities_each_phase": [{"type": t, "name": n} for t, n in sorted(EXPECTED)],
                   "phase_names": list(PHASES), "expected_dt_ns": STEP_NS,
                   "entity_id_stability_checked": True},
        "input_files": {"engine_csv": str(args.engine_csv), "native_wrench_csv": str(args.native_wrench_csv),
                        "contact_frames_csv": str(args.contact_frames_csv), "geometry_csv": str(args.geometry_csv)},
        "input_sha256": {"engine_csv": file_sha256(args.engine_csv),
                         "native_wrench_csv": file_sha256(args.native_wrench_csv),
                         "contact_frames_csv": file_sha256(args.contact_frames_csv),
                         "geometry_csv": file_sha256(args.geometry_csv)},
        "trace": {"engine_rows": len(engine_rows), "distinct_engine_keys": len(engine_index),
                  "all_trace_frame_key_rows_saved": len(frames),
                  "startup_or_outside_hold_engine_rows": sum(not bool(x["in_hold_window"]) for x in engine_row_audit),
                  "source_clock_full_trace": {"engine_before_step": clock_audit(engine_rows, "before_step"),
                                               "engine_after_step": clock_audit(engine_rows, "after_step"),
                                               "native": clock_audit(native_rows, source=True),
                                               "contacts": clock_audit(contact_rows, source=True),
                                               "geometry": clock_audit(geometry_rows, source=True)},
                  "global_engine_problems": dict(global_engine_problems),
                  "source_problems": {"native": dict(native_problems), "contacts": dict(contact_problems),
                                      "geometry": dict(geometry_problems)},
                  "hold_failures": dict(gate_failures)},
        "continuity": {"adjacent_pairs_expected": max(0, len(expected) - 1),
                       "adjacent_pairs_missing": continuity_missing,
                       "position_max_abs_difference": continuity_max["position"],
                       "velocity_max_abs_difference": continuity_max["velocity"],
                       "tolerance": args.continuity_tolerance,
                       "gate": "PASS" if continuity_ok else "FAIL"},
        "ecm_cache_comparison": {"diagnostic_only": True, "does_not_gate_native_engine_state": True,
                                 "engine_minus_ecm": cache_diff},
        "derived_outputs": {"native_wrench_csv": "derived_native_wrench.csv",
                            "geometry_csv": "derived_geometry.csv",
                            "frame_key_audit_csv": "frame_key_audit.csv",
                            "engine_row_audit_csv": "engine_row_audit.csv",
                            "original_force_commands_and_transmitted_wrenches_preserved": True,
                            "original_geometry_frame_validity_and_error_preserved": True,
                            "invalid_state_encoding": "source rows are retained; invalid mapped state values are NaN and corresponding validity flags are zero; no missing value is filled with zero",
                            "native_phase_compatibility": "before_physics_phase is set to before_physics_update solely for the existing offline response auditor; state is sourced from engine before_step using its unchanged shared UpdateInfo key"},
        "failures": [] if gate_ok else ["native_engine_state_window_failed"],
    }
    (output / "summary.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return metadata


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--engine-csv", type=Path, required=True)
    p.add_argument("--native-wrench-csv", type=Path, required=True)
    p.add_argument("--contact-frames-csv", type=Path, required=True)
    p.add_argument("--geometry-csv", type=Path, required=True)
    p.add_argument("--hold-start-ns", type=int, required=True)
    p.add_argument("--hold-end-ns", type=int, required=True)
    p.add_argument("--continuity-tolerance", type=float, default=1e-9)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    result = audit(args)
    print(json.dumps({"gate": result["gate"], "failures": result["failures"],
                      "summary": str(args.output_dir / "summary.json")}, separators=(",", ":")))
    return 0 if result["gate"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
