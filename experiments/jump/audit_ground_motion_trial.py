#!/usr/bin/env python3
"""Offline audit of fixed, low-amplitude ground-motion and normal-stop trials.

This audit preserves every expected physics step. It combines the direct
Physics before/after snapshots with the existing command, contact, and
wheel-servo logs, and runs the immutable ground response gate independently on
each phase of each trial. A successful audit is not a braking qualification.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

STEP_NS = 1_000_000
FIXED_PROFILE_SHA256 = "d6e13b1ee62f04f13c8bccf5642c88b2490d1069fb14666a2bfd45a7e0c32c04"
LEG_INDICES = (0, 1, 3, 4)
NATIVE_ORDER = (0, 1, 2, 3, 4, 5)  # HL, KL, wheelL, HR, KR, wheelR
MODEL_ORDER = (0, 1, 3, 4, 2, 5)  # model q/u order
PHASES = ("acceleration", "cruise", "normal_stop",
          "acceleration_return", "cruise_return", "normal_stop_return")
EVENTS = ("trial_start", "acceleration_start", "cruise_start",
          "normal_stop_trigger", "stop_complete", "return_acceleration_start",
          "return_cruise_start", "return_normal_stop_trigger", "return_stop_complete",
          "campaign_complete", "fault", "trial_abort", "law_blend_start",
          "law_blend_complete")
JOINT_NAMES = ("hip_left", "knee_left", "wheel_left", "hip_right", "knee_right", "wheel_right")
ENGINE_LINKS = (
    "flat_jump_world::bbot::base_link",
    "flat_jump_world::bbot::link_004",
    "flat_jump_world::bbot::link_007",
)
ENGINE_JOINTS = tuple(f"flat_jump_world::bbot::link_{i:03d}_joint" for i in (2, 3, 4, 5, 6, 7))
ENGINE_IDS = {("link", n) for n in ENGINE_LINKS} | {("joint", n) for n in ENGINE_JOINTS}
PHASE_EVENT_REQUIRED = {
    "trial_id", "replay_id", "phase", "event", "event_id", "sim_event_ns",
    "wall_event_ns", "command_id", "command_kind", "reason",
    "target_wheel_linear", "target_wheel_angular",
}
TARGET_ALIASES = {
    "target_hl_rad": ("target_hl_rad", "target_hl"),
    "target_kl_rad": ("target_kl_rad", "target_kl"),
    "target_hr_rad": ("target_hr_rad", "target_hr"),
    "target_kr_rad": ("target_kr_rad", "target_kr"),
}


def read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        r = csv.DictReader(f)
        if not r.fieldnames:
            raise ValueError(f"{path}: CSV header missing")
        return [dict(row) for row in r], list(r.fieldnames)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    names = list(dict.fromkeys(k for r in rows for k in r)) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=names, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def integer(value: Any) -> int | None:
    try:
        s = str(value).strip()
        return int(s, 10) if s else None
    except (TypeError, ValueError, OverflowError):
        return None


def finite(value: Any) -> float | None:
    try:
        x = float(str(value).strip())
        return x if math.isfinite(x) else None
    except (TypeError, ValueError, OverflowError):
        return None


def truth(value: Any) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "valid", "ok"}


def first(row: dict[str, Any], *fields: str, default: Any = "") -> Any:
    for field in fields:
        if field in row and row[field] not in (None, ""):
            return row[field]
    return default


def row_key(row: dict[str, Any], iteration_fields: tuple[str, ...] = ("physics_iteration", "iteration")) -> tuple[int, int] | None:
    it = integer(first(row, *iteration_fields))
    ns = integer(first(row, "sim_time_ns"))
    return (it, ns) if it is not None and ns is not None else None


def index_unique(rows: list[dict[str, str]], kind: str, problems: Counter) -> dict[tuple[int, int], dict[str, str]]:
    out: dict[tuple[int, int], dict[str, str]] = {}
    for row in rows:
        key = row_key(row)
        if key is None:
            problems[f"{kind}_unkeyed_row"] += 1
        elif key in out:
            problems[f"{kind}_duplicate_key"] += 1
        else:
            out[key] = row
    return out


def source_order_check(rows: list[dict[str, str]], *, grouped_native: bool = False) -> Counter:
    problems: Counter = Counter()
    seq: list[tuple[int, int, int | None]] = []
    seen: set[tuple[int, int]] = set()
    for row in rows:
        key = row_key(row)
        if key is None:
            problems["unkeyed_row"] += 1
            continue
        if grouped_native and key in seen:
            continue
        seen.add(key)
        seq.append((key[0], key[1], integer(first(row, "dt_ns", "physics_dt_ns"))))
    for i, (it, ns, dt) in enumerate(seq):
        if dt != STEP_NS:
            problems["dt_not_1ms"] += 1
        if i:
            pit, pns, _ = seq[i - 1]
            if ns < pns:
                problems["time_rollback"] += 1
            if it < pit:
                problems["iteration_rollback"] += 1
            if ns - pns != STEP_NS:
                problems["time_step_not_1ms"] += 1
            if it - pit != 1:
                problems["iteration_step_not_1"] += 1
    return problems


def legal_contact(row: dict[str, str] | None) -> tuple[bool, int | None, str]:
    if row is None:
        return False, None, "missing_contact_frame"
    count = integer(first(row, "num_contacts", "contact_count"))
    if count is None or count < 0 or not truth(first(row, "frame_valid", "valid")) or first(row, "error"):
        return False, count, "invalid_contact_frame"
    raw = first(row, "collision_pairs_json", "pairs_json", "contacts_json")
    try:
        pairs = json.loads(raw)
    except (TypeError, ValueError):
        return False, count, "malformed_contact_pairs"
    if not isinstance(pairs, list) or len(pairs) != count:
        return False, count, "contact_count_payload_mismatch"
    wheels = set()
    for pair in pairs:
        if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(n, str) for n in pair):
            return False, count, "malformed_contact_pair"
        if not all("flat_jump_world::" in n for n in pair):
            return False, count, "unexpected_world_scope"
        grounds = [n for n in pair if "ground_plane::link::collision" in n]
        robots = [n for n in pair if n not in grounds]
        if len(grounds) != 1 or len(robots) != 1:
            return False, count, "unknown_collision_pair"
        wheel = ("link_004::link_004_collision_collision" if "link_004::link_004_collision_collision" in robots[0]
                 else "link_007::link_007_collision_collision" if "link_007::link_007_collision_collision" in robots[0]
                 else None)
        if not wheel:
            return False, count, "non_wheel_ground_contact"
        wheels.add(wheel)
    if count == 2 and len(wheels) != 2:
        return False, count, "bilateral_wheels_not_both_contacting"
    if count not in (0, 1, 2):
        return False, count, "unexpected_contact_count"
    return True, count, "ok"


def load_model_auditor():
    path = Path(__file__).resolve().parents[2] / "src/bbot_balance_controller/scripts/audit_ground_model_response.py"
    spec = importlib.util.spec_from_file_location("ground_model_response_audit_for_motion", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import the frozen ground model response auditor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def events_by_trial(events: list[dict[str, str]]) -> tuple[dict[tuple[str, str], dict[str, list[dict[str, str]]]], Counter, dict[str, list[dict[str, str]]]]:
    grouped: dict[tuple[str, str], dict[str, list[dict[str, str]]]] = defaultdict(lambda: defaultdict(list))
    problems: Counter = Counter()
    global_events: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in events:
        event = row.get("event", "")
        if event in ("campaign_complete", "law_blend_start", "law_blend_complete"):
            # Campaign completion's reason describes the completed action; only
            # explicit fault/abort events are failure evidence. Blend events
            # are preparation timing and do not belong to a numbered trial.
            global_events[event].append(row)
            if integer(row.get("sim_event_ns")) is None or integer(row.get("wall_event_ns")) is None:
                problems[f"{event}:bad_timestamp"] += 1
            if not row.get("event_id", "").strip():
                problems[f"{event}:missing_event_id"] += 1
            if event.startswith("law_blend") and (row.get("trial_id") != "0" or row.get("replay_id") != "0" or row.get("phase") != "support_blend"):
                problems[f"{event}:unexpected_preparation_identity"] += 1
            continue
        if event in ("fault", "trial_abort"):
            problems["explicit_fault_event"] += 1
            continue
        key = (row.get("trial_id", "").strip(), row.get("replay_id", "").strip())
        if not all(key):
            problems["phase_event_missing_trial_or_replay_id"] += 1
        if row.get("event") not in EVENTS:
            problems["unknown_phase_event"] += 1
        if not row.get("phase", "").strip():
            problems["phase_event_missing_phase"] += 1
        if integer(row.get("sim_event_ns")) is None or integer(row.get("wall_event_ns")) is None:
            problems["phase_event_bad_timestamp"] += 1
        grouped[key][row.get("event", "")].append(row)
    for trial, groups in grouped.items():
        for event, rows in groups.items():
            # Leg and wheel publication rows may share an event_id. Event names
            # themselves must identify one phase boundary, not be duplicated.
            ids = {r.get("event_id", "") for r in rows}
            if len(ids) > 1 or not all(ids):
                problems[f"{trial[0]}:{event}:ambiguous_event_identity"] += 1
            command_rows = [r for r in rows if r.get("command_id", "").strip()]
            kinds = [r.get("command_kind", "") for r in command_rows]
            if len(kinds) != len(set(kinds)):
                problems[f"{trial[0]}:{event}:duplicate_command_kind"] += 1
    blend_start = global_events.get("law_blend_start", [])
    blend_end = global_events.get("law_blend_complete", [])
    if len(blend_start) != 1 or len(blend_end) != 1:
        problems["law_blend_preparation_missing_or_duplicate"] += 1
    elif integer(blend_end[0].get("sim_event_ns")) <= integer(blend_start[0].get("sim_event_ns")):
        problems["law_blend_preparation_order_invalid"] += 1
    return dict(grouped), problems, dict(global_events)


def parse_protocol(path: Path) -> dict[str, Any]:
    protocol = json.loads(path.read_text(encoding="utf-8"))
    required = ("trials", "limits", "reference", "world", "profile_sha256", "stop_envelope")
    missing = [k for k in required if k not in protocol]
    if missing:
        raise ValueError("protocol missing required fields: " + ",".join(missing))
    if protocol["world"].get("physics_step_ns") != STEP_NS:
        raise ValueError("protocol world.physics_step_ns must be 1000000")
    if not isinstance(protocol["trials"], list) or len(protocol["trials"]) != 4:
        raise ValueError("protocol must freeze exactly four direction/replay trials")
    if protocol["profile_sha256"] != FIXED_PROFILE_SHA256:
        raise ValueError("protocol profile_sha256 does not match the reviewed fixed probe")
    if protocol.get("allocator_enabled") is not False or float(protocol.get("wheel_servo_gain", -1)) != 1.0:
        raise ValueError("probe requires allocator off and wheel servo gain 1.0")
    limits = protocol["limits"]
    limit_keys = ("hip_torque_nm", "knee_torque_nm", "wheel_torque_nm", "wheel_target_rate_radps",
                  "soft_limits_rad", "minimum_soft_margin_rad", "max_anchor_displacement_rad",
                  "max_leg_rate_radps", "max_pitch_anchor_error_rad", "max_body_rate_radps")
    if any(k not in limits for k in limit_keys):
        raise ValueError("protocol limits are incomplete")
    if len(limits["soft_limits_rad"]) != 4:
        raise ValueError("soft_limits_rad must list hipL,kneeL,hipR,kneeR limits")
    fixed_limits = {"hip_torque_nm": 75.0, "knee_torque_nm": 60.0,
                    "wheel_torque_nm": 10.0, "wheel_target_rate_radps": 30.0,
                    "minimum_soft_margin_rad": 0.30,
                    "max_anchor_displacement_rad": 0.08,
                    "max_leg_rate_radps": 0.10,
                    "max_pitch_anchor_error_rad": 0.10,
                    "max_body_rate_radps": 0.50}
    if any(float(limits[k]) != value for k, value in fixed_limits.items()) or \
       [float(x) for x in limits["soft_limits_rad"]] != [1.52, 1.56, 1.52, 1.56]:
        raise ValueError("protocol safety limits differ from the reviewed fixed probe")
    if float(protocol["reference"].get("stop_timeout_s", 0)) != 2.0 or float(protocol["reference"].get("stop_rate_threshold_radps", 0)) != .005 or float(protocol["reference"].get("stop_dwell_s", 0)) != .25:
        raise ValueError("protocol must freeze stop timeout 2s, leg speed .005rad/s, dwell .25s")
    fixed_reference = {"hip_peak_rate_radps": 0.01, "knee_peak_rate_radps": -0.02,
                       "acceleration_s": 0.5, "cruise_s": 0.5,
                       "normal_deceleration_s": 0.5}
    if any(float(protocol["reference"].get(k, math.nan)) != v for k, v in fixed_reference.items()):
        raise ValueError("protocol movement reference differs from the reviewed low-speed probe")
    fixed_envelope = {"max_stop_joint_displacement_rad": 0.04,
                      "max_stop_joint_rebound_rad": 0.02,
                      "max_stop_axle_displacement_m": 0.05}
    if any(float(protocol["stop_envelope"].get(k, math.nan)) != v for k, v in fixed_envelope.items()):
        raise ValueError("protocol stop envelope differs from the reviewed fixed probe")
    if protocol.get("minimum_physics_steps_per_phase") != 30:
        raise ValueError("protocol must require 30 complete contiguous steps per phase")
    actual_trials = [(int(t.get("trial_id", -1)), int(t.get("replay_id", -1)), int(t.get("direction", 0)))
                     for t in protocol["trials"]]
    if actual_trials != [(1, 1, 1), (2, 1, -1), (3, 2, 1), (4, 2, -1)]:
        raise ValueError("protocol trial/replay/direction order differs from the reviewed fixed probe")
    for trial in protocol["trials"]:
        if not all(k in trial for k in ("trial_id", "replay_id", "direction", "expected_phases",
                                       "minimum_hip_excitation_radps", "minimum_knee_excitation_radps",
                                       "minimum_excitation_steps")):
            raise ValueError("trial protocol fields are incomplete")
        if trial["expected_phases"] != list(PHASES[:3]):
            raise ValueError("each fixed motion trial must contain acceleration, cruise, normal_stop")
        if (float(trial["minimum_hip_excitation_radps"]) != 0.005 or
                float(trial["minimum_knee_excitation_radps"]) != 0.01 or
                int(trial["minimum_excitation_steps"]) != 30):
            raise ValueError("protocol excitation gate differs from the reviewed fixed probe")
    return protocol


def extract_engine_rows(rows: list[dict[str, str]], missing_columns: set[str],
                        start_ns: int, end_ns: int) -> tuple[dict[tuple[int, int], dict[str, dict[tuple[str, str], dict[str, str]]]], Counter, dict[tuple[int, int], int]]:
    """Index raw engine rows and validate entities/phases in requested scope."""
    phases: dict[tuple[int, int], dict[str, dict[tuple[str, str], list[dict[str, str]]]]] = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    event_order = []
    problems: Counter = Counter()
    wall_last: dict[str, int] = {}
    wall_last_global: int | None = None
    entity_ids: dict[tuple[str, str], set[str]] = defaultdict(set)
    for row in rows:
        key = row_key(row, ("iteration",))
        if key is None:
            problems["engine_unkeyed_row"] += 1
            continue
        phases[key][row.get("phase", "")][(row.get("entity_type", ""), row.get("entity_name", ""))].append(row)
        ns = key[1]
        identity = (row.get("entity_type", ""), row.get("entity_name", ""))
        if identity in ENGINE_IDS and row.get("entity_id", "").strip():
            entity_ids[identity].add(row["entity_id"].strip())
        if start_ns <= ns < end_ns:
            bad = []
            if row.get("phase") not in ("before_step", "after_step"):
                bad.append("unknown_phase")
            if integer(row.get("dt_ns")) != STEP_NS:
                bad.append("dt_not_1ms")
            ph = row.get("phase", "")
            expected_state_ns = ns - STEP_NS if ph == "before_step" else ns
            if integer(row.get("physical_state_time_ns")) != expected_state_ns:
                bad.append("physical_state_time_phase_mismatch")
            wall_ns = integer(row.get("wall_steady_time_ns"))
            if wall_ns is None or wall_ns < 0:
                bad.append("invalid_wall_timestamp")
            elif ph in wall_last and wall_ns < wall_last[ph]:
                bad.append("wall_clock_reversal")
            if wall_ns is not None:
                wall_last[ph] = wall_ns
                if wall_last_global is not None and wall_ns < wall_last_global:
                    bad.append("wall_clock_global_reversal")
                wall_last_global = wall_ns
            if (row.get("entity_type", ""), row.get("entity_name", "")) not in ENGINE_IDS:
                bad.append("unknown_or_wrong_scope_entity")
            if not truth(row.get("entity_present")) or not truth(row.get("time_valid")) or not truth(row.get("position_valid")) or not truth(row.get("velocity_valid")):
                bad.append("engine_validity_mask_false")
            for fld in ("position_x", "position_y", "position_z", "quat_w", "quat_x", "quat_y", "quat_z",
                        "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz"):
                if fld not in missing_columns and finite(row.get(fld)) is None:
                    bad.append("engine_nonfinite:" + fld)
            if row.get("entity_type") == "link":
                q = [finite(row.get(f"quat_{x}")) for x in ("w", "x", "y", "z")]
                if all(v is not None for v in q):
                    norm = math.sqrt(sum(v*v for v in q))
                    if abs(norm - 1.0) > 1e-6:
                        bad.append("engine_quaternion_not_unit")
            else:
                if integer(row.get("joint_dof")) != 1 or finite(row.get("joint_position_0")) is None or finite(row.get("joint_velocity_0")) is None:
                    bad.append("engine_joint_state_invalid")
            if row.get("reason", "").strip():
                bad.append("engine_source_reason:" + row["reason"].strip())
            if missing_columns:
                bad.append("engine_schema_missing_columns")
            for issue in bad:
                problems[issue] += 1
        event_order.append((key[0], key[1], row.get("phase", "")))
    # Validate every required entity exactly once on both sides for each key.
    normalized: dict[tuple[int, int], dict[str, dict[tuple[str, str], dict[str, str]]]] = {}
    iterations_by_ns: dict[tuple[int, int], int] = {}
    for key, phase_map in phases.items():
        if not start_ns <= key[1] < end_ns:
            continue
        norm_phases = {}
        for ph in ("before_step", "after_step"):
            identity_rows = phase_map.get(ph, {})
            if set(identity_rows) != ENGINE_IDS:
                problems["engine_missing_or_extra_entity"] += 1
            identities = {}
            for ident, matches in identity_rows.items():
                if len(matches) != 1:
                    problems["engine_duplicate_entity_row"] += 1
                elif ident in ENGINE_IDS:
                    identities[ident] = matches[0]
            norm_phases[ph] = identities
        normalized[key] = norm_phases
    sorted_keys = sorted(normalized, key=lambda k: (k[1], k[0]))
    for k in sorted_keys:
        iterations_by_ns[k] = k[0]
    if any(len(ids) != 1 for ids in entity_ids.values()):
        problems["engine_entity_id_changed_for_identity"] += sum(len(ids) != 1 for ids in entity_ids.values())
    id_owner: dict[str, set[tuple[str, str]]] = defaultdict(set)
    for identity, ids in entity_ids.items():
        for value in ids: id_owner[value].add(identity)
    if any(len(owners) != 1 for owners in id_owner.values()):
        problems["engine_entity_id_aliases_multiple_entities"] += 1
    for (pit, pns), (it, ns) in zip(sorted_keys, sorted_keys[1:]):
        if ns - pns != STEP_NS:
            problems["engine_step_gap_or_clock_error"] += 1
        if it - pit != 1:
            problems["engine_iteration_gap_or_clock_error"] += 1
    return normalized, problems, iterations_by_ns


def event_target(row: dict[str, str]) -> tuple[float | None, ...]:
    fields = [finite(first(row, *aliases)) for aliases in TARGET_ALIASES.values()]
    fields.extend((finite(first(row, "target_wheel_linear", "target_wheel_linear_mps")),
                   finite(first(row, "target_wheel_angular", "target_wheel_angular_radps"))))
    return tuple(fields)


def check_events(protocol: dict[str, Any], groups: dict[tuple[str, str], dict[str, list[dict[str, str]]]],
                 command_rows: list[dict[str, str]]) -> tuple[dict[tuple[str, str], dict[str, int]], Counter, dict[str, dict[str, str]]]:
    problems: Counter = Counter()
    command_by_id: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in command_rows:
        command_by_id[row.get("command_id", "")].append(row)
    command_unique: dict[str, dict[str, str]] = {}
    for cid, rows in command_by_id.items():
        if not cid or len(rows) != 1:
            problems["command_id_missing_or_duplicate"] += 1
        else:
            command_unique[cid] = rows[0]
    target_values = protocol.get("targets", {})
    starts: dict[tuple[str, str], dict[str, int]] = {}
    expected_trials = {(str(t["trial_id"]), str(t["replay_id"])) for t in protocol["trials"]}
    numbered_groups = {k for k in groups if k in expected_trials}
    if numbered_groups != expected_trials or any(k not in expected_trials for k in groups):
        problems["phase_event_trial_set_mismatch"] += 1
    for trial in expected_trials:
        group = groups.get(trial, {})
        times = {}
        required_events = ["trial_start", "acceleration_start", "cruise_start", "normal_stop_trigger", "stop_complete"]
        if protocol.get("return_motion", False):
            required_events += ["return_acceleration_start", "return_cruise_start", "return_normal_stop_trigger", "return_stop_complete"]
        for event in required_events:
            rows = group.get(event, [])
            expected_n = 1 if event in ("trial_start", "stop_complete", "return_stop_complete") else None
            if (expected_n is not None and len(rows) != expected_n) or (expected_n is None and not 1 <= len(rows) <= 2):
                problems[f"{trial[0]}:{event}:missing_or_duplicate_event"] += 1
                continue
            if not rows:
                continue
            stamps = {integer(r.get("sim_event_ns")) for r in rows}
            walls = {integer(r.get("wall_event_ns")) for r in rows}
            ids = {r.get("event_id", "") for r in rows}
            if len(stamps) != 1 or None in stamps or len(walls) != 1 or None in walls or len(ids) != 1:
                problems[f"{trial[0]}:{event}:timestamp_or_id_disagreement"] += 1
                continue
            times[event] = next(iter(stamps))
            for row in rows:
                if row.get("profile_sha256") and row.get("profile_sha256") != protocol["profile_sha256"]:
                    problems[f"{trial[0]}:{event}:profile_hash_mismatch"] += 1
                phase = row.get("phase", "")
                expected_phase = {"trial_start": "stand", "acceleration_start": "acceleration",
                                  "cruise_start": "cruise", "normal_stop_trigger": "normal_stop",
                                  "stop_complete": "normal_stop", "return_acceleration_start": "acceleration_return",
                                  "return_cruise_start": "cruise_return", "return_normal_stop_trigger": "normal_stop_return",
                                  "return_stop_complete": "normal_stop_return"}.get(event)
                if phase != expected_phase and not (event == "trial_start" and phase in ("stand", "trial_start", "acceleration", "")):
                    problems[f"{trial[0]}:{event}:phase_label_mismatch"] += 1
                targets = event_target(row)
                if event not in ("trial_start", "stop_complete", "return_stop_complete") and any(x is None for x in targets):
                    problems[f"{trial[0]}:{event}:target_fields_missing_or_nonfinite"] += 1
                expected = target_values.get(event)
                if expected is not None:
                    fields = ("hip_left", "knee_left", "hip_right", "knee_right", "wheel_linear_mps", "wheel_angular_radps")
                    expected_tuple = tuple(finite(expected.get(f)) for f in fields)
                    if any(x is None for x in targets) or targets != expected_tuple:
                        problems[f"{trial[0]}:{event}:target_mismatch_or_invalid"] += 1
                cid = row.get("command_id", "").strip()
                kind = row.get("command_kind", "").strip()
                if event not in ("trial_start", "stop_complete", "return_stop_complete"):
                    cmd = command_unique.get(cid)
                    if cmd is None:
                        problems[f"{trial[0]}:{event}:command_reference_missing_or_ambiguous"] += 1
                        continue
                    if cmd.get("kind") != kind:
                        problems[f"{trial[0]}:{event}:command_kind_mismatch"] += 1
                    pub_ns = integer(cmd.get("publish_ns"))
                    if pub_ns is None or abs(pub_ns - times[event]) > STEP_NS:
                        problems[f"{trial[0]}:{event}:event_not_at_command_publish"] += 1
                    if cmd.get("state") not in ("0", "BALANCE"):
                        problems[f"{trial[0]}:{event}:command_outside_balance"] += 1
                    if kind == "leg":
                        got = tuple(finite(cmd.get(f"c{i}")) for i in range(4))
                        if any(v is None for v in got):
                            problems[f"{trial[0]}:{event}:nonfinite_leg_command"] += 1
                    elif kind == "wheel":
                        got = (finite(cmd.get("wheel_linear")), finite(cmd.get("wheel_angular")))
                        if any(v is None for v in got):
                            problems[f"{trial[0]}:{event}:nonfinite_wheel_target"] += 1
                    else:
                        problems[f"{trial[0]}:{event}:unknown_command_kind"] += 1
        order = ["trial_start", "acceleration_start", "cruise_start", "normal_stop_trigger", "stop_complete"]
        if len(order) and protocol.get("return_motion", False):
            order += ["return_acceleration_start", "return_cruise_start", "return_normal_stop_trigger", "return_stop_complete"]
        vals = [times.get(x) for x in order]
        if any(v is None for v in vals) or any(b < a if i == 0 else b <= a for i, (a, b) in enumerate(zip(vals, vals[1:]))):
            problems[f"{trial[0]}:event_order_or_time_invalid"] += 1
            continue
        starts[trial] = {"stand": vals[0], "acceleration": vals[1], "cruise": vals[2],
                         "normal_stop": vals[3], "stop_complete": vals[4]}
        if protocol.get("return_motion", False):
            starts[trial].update({"acceleration_return": vals[5], "cruise_return": vals[6],
                                  "normal_stop_return": vals[7], "stop_complete_return": vals[8]})
    # Compare fixed target events only within the same direction across replays.
    if len(expected_trials) >= 2:
        trial_cfg = {(str(t["trial_id"]), str(t["replay_id"])): t for t in protocol["trials"]}
        by_direction = defaultdict(list)
        for trial in sorted(expected_trials):
            values = []
            for event in required_events:
                for row in groups.get(trial, {}).get(event, []):
                    values.append((event, event_target(row)))
            by_direction[str(trial_cfg.get(trial, {}).get("direction", ""))].append(values)
        for direction, profiles in by_direction.items():
            if len(profiles) >= 2 and any(profile != profiles[0] for profile in profiles[1:]):
                problems[f"independent_replay_target_profiles_differ_direction_{direction}"] += 1
    return starts, problems, command_unique


def field_state(row: dict[str, str], prefix: str, axes: str) -> list[float | None]:
    return [finite(row.get(prefix + axis)) for axis in axes]


def engine_dof_state(frame: dict[str, dict[tuple[str, str], dict[str, str]]], phase: str) -> tuple[list[float] | None, list[float] | None, list[float] | None, list[float] | None]:
    states = frame.get(phase, {})
    base = states.get(("link", ENGINE_LINKS[0]))
    if base is None:
        return None, None, None, None
    qbase = finite(base.get("position_y")), finite(base.get("position_z"))
    qquat = [finite(base.get(f"quat_{a}")) for a in ("w", "x", "y", "z")]
    if any(x is None for x in qbase) or any(x is None for x in qquat):
        return None, None, None, None
    roll = 2 * math.atan2(qquat[1], qquat[0])
    vbase = (finite(base.get("linear_vy")), finite(base.get("linear_vz")), finite(base.get("angular_vx")))
    if any(x is None for x in vbase):
        return None, None, None, None
    joints_q = []
    joints_v = []
    for name in ENGINE_JOINTS:
        row = states.get(("joint", name))
        q, v = (finite(row.get("joint_position_0")), finite(row.get("joint_velocity_0"))) if row else (None, None)
        if q is None or v is None:
            return None, None, None, None
        joints_q.append(q); joints_v.append(v)
    q = [qbase[0], qbase[1], roll, *joints_q]
    v = [*vbase, *joints_v]
    return q, v, None, None


def raw_engine_states(frame: dict[str, dict[tuple[str, str], dict[str, str]]], phase: str) -> tuple[list[float] | None, list[float] | None]:
    states = frame.get(phase, {})
    base = states.get(("link", ENGINE_LINKS[0]))
    if base is None:
        return None, None
    qbase = [finite(base.get("position_y")), finite(base.get("position_z"))]
    quat = [finite(base.get(f"quat_{a}")) for a in ("w", "x", "y", "z")]
    vbase = [finite(base.get("linear_vy")), finite(base.get("linear_vz")), finite(base.get("angular_vx"))]
    if any(x is None for x in qbase + quat + vbase):
        return None, None
    q = [*qbase, 2 * math.atan2(quat[1], quat[0])]
    v = list(vbase)
    for name in ENGINE_JOINTS:
        jr = states.get(("joint", name))
        if jr is None:
            return None, None
        qv, vv = finite(jr.get("joint_position_0")), finite(jr.get("joint_velocity_0"))
        if qv is None or vv is None:
            return None, None
        q.append(qv); v.append(vv)
    return q, v


def native_state_dof(joint_index: int) -> int:
    """q9/v9 layout: base xyz-like entries, then native joint index order."""
    return 3 + joint_index


def model_input_state_dofs() -> list[int]:
    """Map model input order HL,KL,HR,KR,WL,WR into native q9/v9 order."""
    return [native_state_dof(joint_index) for joint_index in MODEL_ORDER]


def engine_axle_forward_y(frame: dict[str, dict[tuple[str, str], dict[str, str]]], phase: str = "after_step") -> float | None:
    states = frame.get(phase, {})
    vals = [finite(states.get(("link", name), {}).get("position_y")) for name in ENGINE_LINKS[1:]]
    return sum(vals) / 2.0 if all(v is not None for v in vals) else None


def stop_dwell(frames: list[dict[str, Any]], speed_limit: float, dwell_steps: int) -> tuple[int | None, int | None]:
    """Return first frame index and last frame index of a fully valid dwell."""
    streak = 0
    begin = None
    for i, row in enumerate(frames):
        velocities = row.get("after_joint_v")
        qualifies = (row.get("frame_valid") == 1 and row.get("contact_count") == 2 and
                     isinstance(velocities, list) and len(velocities) == 6 and
                     all(v is not None and abs(v) <= speed_limit for v in (velocities[0], velocities[1], velocities[3], velocities[4])))
        if qualifies:
            if streak == 0:
                begin = i
            streak += 1
            if streak >= dwell_steps:
                return begin, i
        else:
            streak = 0
            begin = None
    return None, None


def dwell_confirmed_at(frames: list[dict[str, Any]], confirm_ns: int | None,
                       speed_limit: float, dwell_steps: int) -> tuple[bool, int | None, int | None]:
    if confirm_ns is None:
        return False, None, None
    eligible = [i for i, f in enumerate(frames) if f["key"][1] <= confirm_ns]
    if not eligible:
        return False, None, None
    end = eligible[-1]
    begin = end - dwell_steps + 1
    if begin < 0:
        return False, None, end
    window = frames[begin:end + 1]
    contiguous = all(b["key"][0] == a["key"][0] + 1 and b["key"][1] == a["key"][1] + STEP_NS
                     for a, b in zip(window, window[1:]))
    good = contiguous and len(window) == dwell_steps and all(
        f["frame_valid"] == 1 and f["contact_count"] == 2 and
        isinstance(f.get("after_joint_v"), list) and len(f["after_joint_v"]) == 6 and
        all(v is not None and abs(v) <= speed_limit for v in
            (f["after_joint_v"][0], f["after_joint_v"][1], f["after_joint_v"][3], f["after_joint_v"][4]))
        for f in window)
    return good, begin, end


def input_vector(native_frame: dict[int, dict[str, str]]) -> list[float | None]:
    # Return the original response bridge order: HL, KL, HR, KR, wheelL, wheelR.
    vals = [finite(first(native_frame.get(idx, {}), "before_physics_joint_force_cmd_sim_input")) for idx in MODEL_ORDER]
    if any(not truth(native_frame.get(idx, {}).get("before_physics_joint_force_cmd_component_present")) for idx in MODEL_ORDER):
        return [None] * 6
    if any(not truth(native_frame.get(idx, {}).get("before_physics_joint_force_cmd_valid")) for idx in MODEL_ORDER):
        return [None] * 6
    if any(value is None for value in vals):
        return [None] * 6
    return vals


def stats(values: list[float | None]) -> dict[str, Any]:
    xs = [x for x in values if x is not None and math.isfinite(x)]
    return {"count": len(xs), "min": min(xs) if xs else None, "max": max(xs) if xs else None,
            "RMS": math.sqrt(sum(x*x for x in xs) / len(xs)) if xs else None}


def servo_rows_for_command(servo_rows: list[dict[str, str]], command: dict[str, str],
                           at_ns: int | None = None) -> list[dict[str, str]]:
    """Match a wheel target to servo input using both target and bounded source time."""
    pub = integer(command.get("publish_ns"))
    end = integer(command.get("publish_end_ns"))
    linear, angular = finite(command.get("wheel_linear")), finite(command.get("wheel_angular"))
    if pub is None or end is None or linear is None or angular is None:
        return []
    earliest, latest = pub - STEP_NS, end + STEP_NS
    matches = []
    for row in servo_rows:
        source, produced = integer(row.get("cmd_source_ns")), integer(row.get("ros_publish_ns"))
        if source is None or produced is None or source < earliest or source > latest:
            continue
        if at_ns is not None and produced > at_ns:
            continue
        sl, sa = finite(row.get("linear_target")), finite(row.get("angular_target"))
        if sl is not None and sa is not None and abs(sl-linear) <= 1e-12 and abs(sa-angular) <= 1e-12:
            matches.append(row)
    return sorted(matches, key=lambda r: integer(r.get("ros_publish_ns"))
                   if integer(r.get("ros_publish_ns")) is not None else -1)


def audit(args: argparse.Namespace, model_module: Any | None = None) -> dict[str, Any]:
    output: Path = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    protocol = parse_protocol(args.protocol_json)
    event_rows, event_fields = read_csv(args.phase_events_csv)
    cmd_rows, cmd_fields = read_csv(args.command_publication_csv)
    servo_rows, servo_fields = read_csv(args.wheel_servo_csv)
    engine_rows, engine_fields = read_csv(args.engine_csv)
    native_rows, native_fields = read_csv(args.derived_native_csv)
    geometry_rows, geometry_fields = read_csv(args.derived_geometry_csv)
    contact_rows, contact_fields = read_csv(args.contact_frames_csv)
    missing_events = PHASE_EVENT_REQUIRED - set(event_fields)
    missing_commands = {"command_id", "publish_ns", "publish_end_ns", "wall_publish_ns", "wall_publish_end_ns", "state", "kind", "c0", "c1", "c2", "c3", "wheel_linear", "wheel_angular"} - set(cmd_fields)
    missing_servo = {"publish_seq", "ros_publish_ns", "cmd_source_ns", "cmd_valid", "joint_valid", "left_torque_cmd", "right_torque_cmd", "left_target_rate", "right_target_rate", "left_saturated", "right_saturated"} - set(servo_fields)
    problems: Counter = Counter()
    if missing_events: problems["phase_event_schema_missing_columns"] += len(missing_events)
    if missing_commands: problems["command_schema_missing_columns"] += len(missing_commands)
    if missing_servo: problems["servo_schema_missing_columns"] += len(missing_servo)
    groups, event_problems, global_events = events_by_trial(event_rows); problems.update(event_problems)
    boundaries, ref_problems, commands_by_id = check_events(protocol, groups, cmd_rows); problems.update(ref_problems)
    contact_problems: Counter = Counter()
    contacts = index_unique(contact_rows, "contact", contact_problems)
    geometry_problems: Counter = Counter()
    geometries = index_unique(geometry_rows, "geometry", geometry_problems)
    native_group: dict[tuple[int, int], dict[int, dict[str, str]]] = defaultdict(dict)
    native_problems: Counter = Counter()
    for row in native_rows:
        key = row_key(row)
        idx = integer(row.get("joint_index"))
        if key is None or idx is None:
            native_problems["native_unkeyed_row"] += 1; continue
        if idx in native_group[key]:
            native_problems["native_duplicate_joint_index"] += 1
        else:
            native_group[key][idx] = row
    for key, idx_rows in native_group.items():
        if set(idx_rows) != set(range(6)):
            native_problems["native_joint_index_set_invalid"] += 1
    contact_problems.update(source_order_check(contact_rows))
    geometry_problems.update(source_order_check(geometry_rows))
    native_problems.update(source_order_check(native_rows, grouped_native=True))
    command_times = [integer(row.get("publish_ns")) for row in cmd_rows]
    if any(x is None for x in command_times) or any(b < a for a, b in zip(command_times, command_times[1:])):
        problems["command_publish_clock_invalid"] += 1
    commands_by_kind = defaultdict(list)
    for row in cmd_rows:
        if row.get("kind") in ("leg", "wheel") and integer(row.get("publish_ns")) is not None:
            commands_by_kind[row["kind"]].append(row)
    for values in commands_by_kind.values():
        values.sort(key=lambda r: integer(r.get("publish_ns"))
                    if integer(r.get("publish_ns")) is not None else -1)
    profile_bytes = args.protocol_json.read_bytes()
    profile_digest = hashlib.sha256(profile_bytes).hexdigest()
    for row in event_rows:
        if row.get("profile_sha256") and row.get("profile_sha256") != protocol["profile_sha256"]:
            problems["event_profile_sha_mismatch"] += 1
    if missing_events:
        for name in sorted(missing_events): problems["event_missing:" + name] += 1

    # Build fixed expected intervals from event timestamps; preserve all rows,
    # including invalid physics values, on each interval.
    trial_intervals: dict[tuple[str, str], dict[str, tuple[int, int]]] = {}
    for trial, b in boundaries.items():
        trial_intervals[trial] = {
            "acceleration": (b["acceleration"], b["cruise"]),
            "cruise": (b["cruise"], b["normal_stop"]),
            "normal_stop": (b["normal_stop"], b["stop_complete"] + STEP_NS),
        }
        if "acceleration_return" in b:
            trial_intervals[trial].update({
                "acceleration_return": (b["acceleration_return"], b["cruise_return"]),
                "cruise_return": (b["cruise_return"], b["normal_stop_return"]),
                "normal_stop_return": (b["normal_stop_return"], b["stop_complete_return"] + STEP_NS),
            })
    if not trial_intervals:
        problems["no_valid_trial_windows"] += 1
    min_ns = min((a for phases in trial_intervals.values() for a, _ in phases.values()), default=0)
    max_ns = max((b for phases in trial_intervals.values() for _, b in phases.values()), default=0)
    engine_missing = set(engine_audit_required_columns()) - set(engine_fields)
    engine_index, engine_problems, engine_iters = extract_engine_rows(engine_rows, engine_missing, min_ns, max_ns)
    if engine_missing: problems["engine_schema_missing_columns"] += len(engine_missing)
    native_index_by_ns = defaultdict(list)
    for k, rows in native_group.items(): native_index_by_ns[k[1]].append(k)
    all_keys_by_ns = {}
    for ns, keys in native_index_by_ns.items():
        if len(keys) != 1:
            native_problems["native_timestamp_ambiguous_iteration"] += 1
        else: all_keys_by_ns[ns] = keys[0]
    anchors: dict[tuple[str, str], list[float]] = {}
    for trial, intervals in trial_intervals.items():
        start = intervals["acceleration"][0]
        anchor_key = all_keys_by_ns.get(start)
        anchor_frame = engine_index.get(anchor_key, {}) if anchor_key else {}
        anchor_q, _ = raw_engine_states(anchor_frame, "before_step") if anchor_frame else (None, None)
        if anchor_q is not None:
            anchors[trial] = anchor_q
        else:
            problems[f"{trial[0]}:anchor_state_missing"] += 1
    problems.update(native_problems); problems.update(contact_problems); problems.update(geometry_problems); problems.update(engine_problems)

    # Servo rows are indexed by sequence and source command stamp. Saturation
    # is kept as an observation; it does not invalidate a legal capped input.
    servo_by_source: dict[int, list[dict[str, str]]] = defaultdict(list)
    servo_problems: Counter = Counter()
    for row in servo_rows:
        seq, pub, src = integer(row.get("publish_seq")), integer(row.get("ros_publish_ns")), integer(row.get("cmd_source_ns"))
        if seq is None or pub is None or src is None:
            servo_problems["servo_unkeyed_or_missing_source_row"] += 1
            continue
        servo_by_source[src].append(row)
    problems.update(servo_problems)

    phase_rows: list[dict[str, Any]] = []
    phase_frames_by_trial: dict[tuple[str, str], dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    reverse_counts: Counter = Counter()
    limits = protocol["limits"]
    input_limits = [float(limits["hip_torque_nm"]), float(limits["knee_torque_nm"]),
                    float(limits["hip_torque_nm"]), float(limits["knee_torque_nm"]),
                    float(limits["wheel_torque_nm"]), float(limits["wheel_torque_nm"])]
    for trial, phases in trial_intervals.items():
        for phase, (start, end) in phases.items():
            if start < 0 or end <= start or (end - start) % STEP_NS:
                problems[f"{trial[0]}:{phase}:window_not_1ms_aligned"] += 1
                continue
            expected_ns = list(range(start, end, STEP_NS))
            for ns in expected_ns:
                frame_key = all_keys_by_ns.get(ns)
                if frame_key is None:
                    # preserve expected missing row, not silently shorten
                    frame_key = (engine_iters.get(next((k for k in engine_index if k[1] == ns), (None, ns)), -1), ns)
                frame = engine_index.get(frame_key, {}) if frame_key[0] >= 0 else {}
                native = native_group.get(frame_key, {}) if frame_key[0] >= 0 else {}
                contact = contacts.get(frame_key)
                geometry = geometries.get(frame_key)
                failures = []
                if frame_key[0] < 0 or frame_key not in engine_index: failures.append("missing_raw_engine_key")
                if frame_key not in native_group: failures.append("missing_derived_native_key")
                if frame_key not in contacts: failures.append("missing_contact_key")
                if frame_key not in geometries: failures.append("missing_derived_geometry_key")
                if len(native) != 6: failures.append("derived_native_six_joint_rows_missing")
                c_ok, c_count, c_reason = legal_contact(contact)
                if not c_ok: failures.append(c_reason)
                if c_count != 2: failures.append("bilateral_wheel_contact_required")
                if geometry is None or not truth(geometry.get("frame_valid")) or geometry.get("error", "").strip():
                    failures.append("derived_geometry_invalid_or_source_error")
                raw_before = raw_engine_states(frame, "before_step") if frame else (None, None)
                raw_after = raw_engine_states(frame, "after_step") if frame else (None, None)
                q_before, v_before = raw_before
                q_after, v_after = raw_after
                if q_before is None or v_before is None or q_after is None or v_after is None:
                    failures.append("raw_engine_9dof_state_invalid")
                if frame:
                    for ph in ("before_step", "after_step"):
                        idmap = frame.get(ph, {})
                        if set(idmap) != ENGINE_IDS: failures.append(f"raw_engine_{ph}_entity_set_invalid")
                        expected_state_ns = ns - STEP_NS if ph == "before_step" else ns
                        for (entity_type, entity_name), erow in idmap.items():
                            if not truth(erow.get("entity_present")) or not truth(erow.get("time_valid")) or not truth(erow.get("position_valid")) or not truth(erow.get("velocity_valid")):
                                failures.append(f"raw_engine_{ph}_validity_mask_false:{entity_name}")
                            if integer(erow.get("physical_state_time_ns")) != expected_state_ns or integer(erow.get("dt_ns")) != STEP_NS:
                                failures.append(f"raw_engine_{ph}_phase_clock_invalid:{entity_name}")
                            if erow.get("reason", "").strip():
                                failures.append(f"raw_engine_{ph}_reason:{entity_name}")
                            if entity_type == "joint":
                                if integer(erow.get("joint_dof")) != 1 or finite(erow.get("joint_position_0")) is None or finite(erow.get("joint_velocity_0")) is None:
                                    failures.append(f"raw_engine_{ph}_joint_state_invalid:{entity_name}")
                            else:
                                if any(finite(erow.get(f"{field}_{axis}")) is None for field in ("position", "linear_v", "angular_v") for axis in "xyz"):
                                    failures.append(f"raw_engine_{ph}_link_state_invalid:{entity_name}")
                                quat = [finite(erow.get(f"quat_{axis}")) for axis in ("w", "x", "y", "z")]
                                if any(x is None for x in quat) or abs(math.sqrt(sum(x*x for x in quat if x is not None)) - 1.0) > 1e-6:
                                    failures.append(f"raw_engine_{ph}_quaternion_invalid:{entity_name}")
                inputs = input_vector(native) if len(native) == 6 else [None] * 6
                if any(x is None for x in inputs): failures.append("native_before_physics_input_invalid_or_nan")
                for j, value in enumerate(inputs):
                    if value is not None and (not math.isfinite(input_limits[j]) or abs(value) > input_limits[j] + 1e-9):
                        failures.append(f"native_input_limit_exceeded:{JOINT_NAMES[MODEL_ORDER[j]]}")
                if q_before and q_after and v_before and v_after:
                    vmax = float(limits["max_leg_rate_radps"])
                    for idx in (0, 1, 3, 4):
                        if abs(v_before[3 + idx]) > vmax + 1e-9 or abs(v_after[3 + idx]) > vmax + 1e-9:
                            failures.append(f"joint_speed_limit_exceeded:{JOINT_NAMES[idx]}")
                    soft = [float(x) for x in limits["soft_limits_rad"]]
                    margin = float(limits["minimum_soft_margin_rad"])
                    for idx, soft_idx in zip((0, 1, 3, 4), range(4)):
                        if abs(q_before[3 + idx]) > soft[soft_idx] - margin + 1e-9 or abs(q_after[3 + idx]) > soft[soft_idx] - margin + 1e-9:
                            failures.append(f"soft_limit_margin_violation:{JOINT_NAMES[idx]}")
                    anchor = anchors.get(trial)
                    if anchor is None:
                        failures.append("trial_anchor_missing")
                    else:
                        if any(max(abs(q_before[3+i]-anchor[3+i]), abs(q_after[3+i]-anchor[3+i])) > float(limits["max_anchor_displacement_rad"]) + 1e-9 for i in (0,1,3,4)):
                            failures.append("joint_anchor_displacement_limit_exceeded")
                        if abs(q_after[2] - anchor[2]) > float(limits["max_pitch_anchor_error_rad"]) + 1e-9:
                            failures.append("pitch_anchor_error_over_protocol_limit")
                    if abs(v_before[2]) > float(limits["max_body_rate_radps"]) + 1e-9 or abs(v_after[2]) > float(limits["max_body_rate_radps"]) + 1e-9:
                        failures.append("body_rate_limit_exceeded")
                # Match direct engine states against the derived native audit copy.
                if q_before and v_before and q_after and v_after and len(native) == 6:
                    for j, idx in enumerate(range(6)):
                        nr = native.get(idx, {})
                        dof = native_state_dof(idx)
                        e_q0 = finite(nr.get("before_physics_joint_position"))
                        e_v0 = finite(nr.get("before_physics_joint_velocity"))
                        e_q1 = finite(nr.get("joint_position"))
                        e_v1 = finite(nr.get("joint_velocity"))
                        for label, a, b in (("before_q", e_q0, q_before[dof]),
                                            ("before_v", e_v0, v_before[dof]),
                                            ("after_q", e_q1, q_after[dof]),
                                            ("after_v", e_v1, v_after[dof])):
                            if a is None or abs(a - b) > 1e-10:
                                failures.append(f"derived_engine_joint_disagreement:{idx}:{label}")
                if geometry is not None and frame:
                    after_entities = frame.get("after_step", {})
                    for scoped, prefix in ((ENGINE_LINKS[0], "base"), (ENGINE_LINKS[1], "left_wheel"),
                                           (ENGINE_LINKS[2], "right_wheel")):
                        erow = after_entities.get(("link", scoped), {})
                        for axis in "xyz":
                            raw_value = finite(erow.get(f"position_{axis}"))
                            derived_value = finite(geometry.get(f"{prefix}_{axis}"))
                            if raw_value is None or derived_value is None or abs(raw_value - derived_value) > 1e-10:
                                failures.append(f"derived_engine_geometry_disagreement:{prefix}_{axis}")
                    for field, short in zip(("hip_left", "knee_left", "hip_right", "knee_right"),
                                            ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")):
                        erow = after_entities.get(("joint", f"flat_jump_world::bbot::{short}"), {})
                        raw_value = finite(erow.get("joint_position_0"))
                        derived_value = finite(geometry.get(field))
                        if raw_value is None or derived_value is None or abs(raw_value - derived_value) > 1e-10:
                            failures.append(f"derived_engine_geometry_disagreement:{field}")
                contact_trans = False
                prev_key = all_keys_by_ns.get(ns - STEP_NS)
                prev_c = contacts.get(prev_key) if prev_key else None
                prev_valid, prev_count, _ = legal_contact(prev_c)
                if prev_valid and prev_count != c_count: contact_trans = True
                # Wheel servo source row matched to the last wheel command stamp.
                servo_source = None
                servo_candidates = []
                wheel_history = [c for c in cmd_rows if c.get("kind") == "wheel" and
                                 integer(c.get("publish_ns")) is not None and integer(c.get("publish_ns")) <= ns]
                wheel_command = max(wheel_history, key=lambda c: integer(c.get("publish_ns"))
                                    if integer(c.get("publish_ns")) is not None else -1) if wheel_history else None
                if wheel_command:
                    servo_candidates = servo_rows_for_command(servo_rows, wheel_command, ns)
                    if servo_candidates: servo_source = servo_candidates[-1]
                if servo_source is None:
                    failures.append("wheel_servo_actual_torque_source_missing_or_invalid")
                else:
                    if not truth(servo_source.get("cmd_valid")) or not truth(servo_source.get("joint_valid")):
                        failures.append("wheel_servo_actual_torque_invalid")
                    for field in ("left_torque_cmd", "right_torque_cmd", "left_target_rate", "right_target_rate"):
                        if finite(servo_source.get(field)) is None:
                            failures.append("wheel_servo_nonfinite:" + field)
                    rate_cap = float(limits["wheel_target_rate_radps"])
                    target_rates = [finite(servo_source.get(f"{side}_target_rate")) for side in ("left", "right")]
                    wheel_torques = [finite(servo_source.get(f"{side}_torque_cmd")) for side in ("left", "right")]
                    if any(x is None for x in target_rates):
                        failures.append("wheel_target_rate_missing_or_nonfinite")
                    elif any(abs(x) > rate_cap + 1e-9 for x in target_rates):
                        failures.append("wheel_target_rate_limit_exceeded")
                    if any(x is None for x in wheel_torques):
                        failures.append("wheel_published_torque_missing_or_nonfinite")
                    elif any(abs(x) > float(limits["wheel_torque_nm"]) + 1e-9 for x in wheel_torques):
                        failures.append("wheel_published_torque_limit_exceeded")
                    # Servo P output is a published request; native input remains
                    # the applied truth and must be independently valid.
                source_command_audit = {}
                for kind in ("leg", "wheel"):
                    history = [c for c in commands_by_kind[kind] if integer(c.get("publish_ns")) is not None and integer(c.get("publish_ns")) <= ns]
                    latest = history[-1] if history else None
                    age = ns - integer(latest.get("publish_ns")) if latest else None
                    source_command_audit[kind] = (latest, age)
                    if latest is None or latest.get("state") not in ("0", "BALANCE"):
                        failures.append(f"{kind}_controller_mode_missing_or_not_balance")
                    elif age is None or age < 0 or age > 150_000_000:
                        failures.append(f"{kind}_controller_command_stale_or_future")
                rev = []
                continued_accel = []
                if inputs and v_before and v_after:
                    # input order HL,KL,HR,KR,wheelL,wheelR; state indexes are mapped
                    state_indices = model_input_state_dofs()
                    for j, (u, qi) in enumerate(zip(inputs, state_indices)):
                        if u is not None and u * v_before[qi] < -1e-12 and (v_after[qi] - v_before[qi]) * v_before[qi] > 1e-12:
                            rev.append(JOINT_NAMES[MODEL_ORDER[j]])
                            reverse_counts[(trial, phase, JOINT_NAMES[MODEL_ORDER[j]])] += 1
                        if (v_after[qi] - v_before[qi]) * v_before[qi] > 1e-12:
                            continued_accel.append(JOINT_NAMES[MODEL_ORDER[j]])
                axle_y = engine_axle_forward_y(frame) if frame else None
                frame_ok = not failures
                row = {
                    "trial_id": trial[0], "replay_id": trial[1], "phase": phase,
                    "iteration": frame_key[0], "sim_time_ns": ns, "dt_ns": STEP_NS,
                    "before_state_time_ns": ns - STEP_NS, "after_state_time_ns": ns,
                    "frame_valid": int(frame_ok), "failures": "|".join(failures),
                    "contact_valid": int(c_ok), "contact_count": c_count,
                    "contact_reason": c_reason, "contact_transition": int(contact_trans),
                    "wheel_servo_publish_seq": servo_source.get("publish_seq", "") if servo_source else "",
                    "wheel_servo_cmd_source_ns": servo_source.get("cmd_source_ns", "") if servo_source else "",
                    "wheel_servo_joint_source_ns": servo_source.get("joint_source_ns", "") if servo_source else "",
                    "wheel_servo_ros_publish_ns": servo_source.get("ros_publish_ns", "") if servo_source else "",
                    "wheel_servo_source_match_ambiguous": int(servo_source is not None and len(servo_candidates) > 1),
                    "wheel_servo_saturated_left": servo_source.get("left_saturated", "") if servo_source else "",
                    "wheel_servo_saturated_right": servo_source.get("right_saturated", "") if servo_source else "",
                    "reverse_effort_acceleration_dofs": "|".join(rev),
                    "reverse_effort_acceleration_count": len(rev),
                    "actual_motion_continued_accelerating_dofs": "|".join(continued_accel),
                    "actual_motion_continued_accelerating_count": len(continued_accel),
                    "axle_forward_world_y_after_m": axle_y,
                }
                for kind in ("leg", "wheel"):
                    latest, age = source_command_audit[kind]
                    row[f"{kind}_command_id"] = latest.get("command_id", "") if latest else ""
                    row[f"{kind}_command_control_ns"] = latest.get("control_ns", "") if latest else ""
                    row[f"{kind}_command_publish_ns"] = latest.get("publish_ns", "") if latest else ""
                    row[f"{kind}_command_publish_end_ns"] = latest.get("publish_end_ns", "") if latest else ""
                    row[f"{kind}_command_wall_publish_ns"] = latest.get("wall_publish_ns", "") if latest else ""
                    row[f"{kind}_command_wall_publish_end_ns"] = latest.get("wall_publish_end_ns", "") if latest else ""
                    row[f"{kind}_command_age_ns"] = age
                for j, name in enumerate(JOINT_NAMES):
                    row[f"{name}_before_q"] = q_before[3 + j] if q_before else None
                    row[f"{name}_before_v"] = v_before[3 + j] if v_before else None
                    row[f"{name}_after_q"] = q_after[3 + j] if q_after else None
                    row[f"{name}_after_v"] = v_after[3 + j] if v_after else None
                for j, name in enumerate(("base_forward", "base_z", "roll_x")):
                    row[f"{name}_before_q"] = q_before[j] if q_before else None
                    row[f"{name}_before_v"] = v_before[j] if v_before else None
                    row[f"{name}_after_q"] = q_after[j] if q_after else None
                    row[f"{name}_after_v"] = v_after[j] if v_after else None
                for j, idx in enumerate(MODEL_ORDER):
                    nm = JOINT_NAMES[idx]
                    row[f"{nm}_native_before_input_nm"] = inputs[j]
                if servo_source:
                    row["wheel_left_published_torque_nm"] = finite(servo_source.get("left_torque_cmd"))
                    row["wheel_right_published_torque_nm"] = finite(servo_source.get("right_torque_cmd"))
                    row["wheel_left_target_rate_rad_s"] = finite(servo_source.get("left_target_rate"))
                    row["wheel_right_target_rate_rad_s"] = finite(servo_source.get("right_target_rate"))
                phase_rows.append(row)
                for issue in failures:
                    problems[f"{trial[0]}:{phase}:{issue}"] += 1
                phase_frames_by_trial[trial][phase].append({
                    "row": row, "key": frame_key, "q_before": q_before, "v_before": v_before,
                    "q_after": q_after, "v_after": v_after, "input": inputs,
                    "geometry": geometry, "contact": contact, "frame_valid": int(frame_ok),
                    "contact_count": c_count,
                    "axle_y_after": axle_y,
                    "after_joint_v": [v_after[3 + i] for i in range(6)] if v_after else None,
                })

    # Full raw engine continuity and adjacent post->next-before continuity use
    # actual rows, never an error-selected subset.
    continuity: dict[str, Any] = {}
    continuity_tolerance = float(protocol.get("limits", {}).get("velocity_continuity_tolerance", 1e-9))
    for trial, phases in phase_frames_by_trial.items():
        trial_frames = sorted((f for values in phases.values() for f in values), key=lambda f: (f["key"][1], f["key"][0]))
        all_deltas = []
        for a, b in zip(trial_frames, trial_frames[1:]):
            if b["key"][0] == a["key"][0] + 1 and b["key"][1] == a["key"][1] + STEP_NS and a["v_after"] and b["v_before"]:
                all_deltas.extend(abs(x-y) for x, y in zip(a["v_after"], b["v_before"]))
        continuity[f"{trial[0]}:all_phases"] = {"pair_count": len(all_deltas) // 9,
                                                 "max_abs_velocity_delta": max(all_deltas) if all_deltas else None,
                                                 "limit": continuity_tolerance,
                                                 "gate": "PASS" if all_deltas and max(all_deltas) <= continuity_tolerance else "FAIL"}
        if not all_deltas or max(all_deltas) > continuity_tolerance:
            problems[f"{trial[0]}:all_phases:post_to_next_before_continuity_failed"] += 1
        for phase, frames in phases.items():
            ds = []
            for a, b in zip(frames, frames[1:]):
                if b["key"][0] == a["key"][0] + 1 and b["key"][1] == a["key"][1] + STEP_NS and a["v_after"] and b["v_before"]:
                    ds.extend(abs(x-y) for x, y in zip(a["v_after"], b["v_before"]))
            continuity[f"{trial[0]}:{phase}"] = {"pair_count": len(ds) // 9,
                                                  "max_abs_velocity_delta": max(ds) if ds else None,
                                                  "limit": continuity_tolerance,
                                                  "gate": "PASS" if ds and max(ds) <= continuity_tolerance else "FAIL"}
            if not ds or max(ds) > continuity_tolerance:
                problems[f"{trial[0]}:{phase}:post_to_next_before_continuity_failed"] += 1

    # Command-application latency is an interval. Applied force carries no
    # command ID, so identical-value recurrence is explicitly ambiguous.
    latency_rows = []
    for event in event_rows:
        cid = event.get("command_id", "").strip()
        cmd = commands_by_id.get(cid)
        if not cmd or event.get("event") in ("trial_start", "stop_complete"):
            continue
        kind = event.get("command_kind", "")
        event_ns = integer(event.get("sim_event_ns"))
        pub_ns = integer(cmd.get("publish_ns"))
        target = event_target(event)
        expected_vector = None
        if kind == "leg":
            expected_vector = [finite(cmd.get(f"c{i}")) for i in range(4)] + [None, None]
        elif kind == "wheel":
            source_rows = servo_rows_for_command(servo_rows, cmd)
            valid = [s for s in source_rows if truth(s.get("cmd_valid")) and truth(s.get("joint_valid"))]
            if valid:
                expected_vector = [None, None, None, None,
                                   finite(valid[0].get("left_torque_cmd")), finite(valid[0].get("right_torque_cmd"))]
        matches = []
        prior_same_count = 0
        if expected_vector and event_ns is not None:
            for key, indices in native_group.items():
                if len(indices) != 6: continue
                actual = input_vector(indices)
                if any(a is None for a in actual): continue
                pairs = [(actual[i], expected_vector[i]) for i in range(6) if expected_vector[i] is not None]
                if pairs and all(abs(a-b) <= 1e-9 for a, b in pairs):
                    if key[1] < event_ns: prior_same_count += 1
                    elif key[1] >= event_ns: matches.append((key, actual))
        first_key = min(matches, key=lambda x: x[0][1])[0] if matches else None
        end_ns = integer(cmd.get("publish_end_ns"))
        start_delay = (first_key[1] - STEP_NS - pub_ns) if first_key and pub_ns is not None else None
        api_end = end_ns if end_ns is not None else pub_ns
        end_delay = (first_key[1] - api_end) if first_key and api_end is not None else None
        latency_rows.append({"trial_id": event.get("trial_id"), "replay_id": event.get("replay_id"),
                             "event": event.get("event"), "event_id": event.get("event_id"),
                             "command_id": cid, "command_kind": kind,
                             "control_source_ns": integer(cmd.get("control_ns")),
                             "publish_ns": pub_ns, "publish_end_ns": end_ns,
                             "wall_publish_ns": integer(cmd.get("wall_publish_ns")),
                             "wall_publish_end_ns": integer(cmd.get("wall_publish_end_ns")),
                             "event_to_publish_delta_ns": event_ns - pub_ns if event_ns is not None and pub_ns is not None else None,
                             "first_matching_input_iteration": first_key[0] if first_key else None,
                             "first_matching_input_step_end_ns": first_key[1] if first_key else None,
                             "apply_interval_start_ns": first_key[1] - STEP_NS if first_key else None,
                             "apply_interval_end_ns": first_key[1] if first_key else None,
                             "latency_lower_bound_ns": start_delay, "latency_upper_bound_ns": end_delay,
                             "same_value_matches_before_publication": prior_same_count,
                             "ambiguous_repeated_value": int(prior_same_count > 0 or not matches or
                                                              (kind == "wheel" and pub_ns is not None and len(servo_rows_for_command(servo_rows, cmd)) > 1)),
                             "latency_semantics": "bounded interval from publish API interval to first post-publish native input with same value; JointForceCmd has no command ID; repeated value is ambiguous"})

    # Stop metrics are calculated from every native engine frame. Any invalid
    # or unsupported frame breaks the dwell streak and remains visible.
    stop_summaries = {}
    stop_limit = float(protocol["reference"]["stop_rate_threshold_radps"])
    dwell_steps = int(round(float(protocol["reference"]["stop_dwell_s"]) * 1e9 / STEP_NS))
    timeout_ns = int(round(float(protocol["reference"]["stop_timeout_s"]) * 1e9))
    for trial, phases in phase_frames_by_trial.items():
        frames = phases.get("normal_stop", [])
        native_dwell_begin, native_dwell_end = stop_dwell(frames, stop_limit, dwell_steps)
        event_group = groups.get(trial, {})
        trigger_row = (event_group.get("normal_stop_trigger") or [{}])[0]
        trigger_ns = integer(trigger_row.get("sim_event_ns"))
        complete_rows = event_group.get("stop_complete", [])
        complete_ns = integer(complete_rows[0].get("sim_event_ns")) if complete_rows else None
        controller_dwell_ok, confirm_dwell_begin, confirm_dwell_end = dwell_confirmed_at(
            frames, complete_ns, stop_limit, dwell_steps)
        first_threshold_index = next((i for i, f in enumerate(frames)
                                      if f["frame_valid"] == 1 and f["contact_count"] == 2 and
                                      f.get("after_joint_v") and all(abs(f["after_joint_v"][j]) <= stop_limit
                                                                      for j in (0, 1, 3, 4))), None)
        trigger_frame = next((i for i, f in enumerate(frames) if trigger_ns is not None and f["key"][1] >= trigger_ns), None)
        base_y0 = frames[trigger_frame]["q_before"][0] if trigger_frame is not None and frames[trigger_frame]["q_before"] else None
        trigger_q = frames[trigger_frame]["q_before"] if trigger_frame is not None else None
        trigger_engine_frame = (engine_index.get(frames[trigger_frame]["key"])
                                if trigger_frame is not None else None)
        trigger_axle_y = (engine_axle_forward_y(trigger_engine_frame, "before_step")
                          if trigger_engine_frame is not None else None)
        displacement = []
        axle_displacement = []
        joint_excursions = [[] for _ in range(4)]
        joint_rebounds = [[] for _ in range(4)]
        pitch_error = []
        speed_peak = [0.0] * 6
        input_peak = [0.0] * 6
        support_steps = 0
        for f in frames:
            if f["q_after"] and base_y0 is not None:
                displacement.append(f["q_after"][0] - base_y0)
            if f["q_after"] and trial in anchors:
                pitch_error.append(abs(f["q_after"][2] - anchors[trial][2]))
            if f.get("axle_y_after") is not None and trigger_axle_y is not None:
                axle_displacement.append(f["axle_y_after"] - trigger_axle_y)
            if f["q_after"] and trigger_q:
                for j, joint_idx in enumerate((0, 1, 3, 4)):
                    joint_excursions[j].append(f["q_after"][3 + joint_idx] - trigger_q[3 + joint_idx])
            if f["v_before"]:
                for j in range(6): speed_peak[j] = max(speed_peak[j], abs(f["v_before"][3+j]))
            if f["input"]:
                for j, value in enumerate(f["input"]):
                    if value is not None: input_peak[j] = max(input_peak[j], abs(value))
            support_steps += int(f["contact_count"] == 2 and f["frame_valid"] == 1)
        stop_elapsed_ns = (complete_ns - trigger_ns) if complete_ns is not None and trigger_ns is not None else None
        stop_completed = controller_dwell_ok and stop_elapsed_ns is not None and 0 <= stop_elapsed_ns <= timeout_ns
        earliest_native_stop_ns = frames[first_threshold_index]["key"][1] if first_threshold_index is not None else None
        native_dwell_end_ns = frames[native_dwell_end]["key"][1] if native_dwell_end is not None else None
        confirm_delay_ns = complete_ns - native_dwell_end_ns if complete_ns is not None and native_dwell_end_ns is not None else None
        max_disp = max((abs(x) for x in displacement), default=None)
        max_axle_disp = max((abs(x) for x in axle_displacement), default=None)
        joint_excursion_max = {JOINT_NAMES[joint_idx]: max((abs(x) for x in joint_excursions[j]), default=None)
                               for j, joint_idx in enumerate((0, 1, 3, 4))}
        joint_rebound_max = {}
        if trigger_frame is not None and trigger_q:
            for j, joint_idx in enumerate((0, 1, 3, 4)):
                initial_v = frames[trigger_frame]["v_before"][3 + joint_idx] if frames[trigger_frame]["v_before"] else None
                sign = 1.0 if initial_v is not None and initial_v > 1e-9 else -1.0 if initial_v is not None and initial_v < -1e-9 else 0.0
                signed_positions = []
                for f in frames:
                    if f["q_after"]:
                        delta = f["q_after"][3 + joint_idx] - trigger_q[3 + joint_idx]
                        if sign == 0.0 and abs(delta) > 1e-9:
                            sign = 1.0 if delta > 0 else -1.0
                        signed_positions.append(sign * delta if sign else 0.0)
                if signed_positions:
                    peak_idx = max(range(len(signed_positions)), key=signed_positions.__getitem__)
                    joint_rebound_max[JOINT_NAMES[joint_idx]] = max(0.0, signed_positions[peak_idx] - min(signed_positions[peak_idx:]))
                else:
                    joint_rebound_max[JOINT_NAMES[joint_idx]] = None
        else:
            joint_rebound_max = {JOINT_NAMES[j]: None for j in (0, 1, 3, 4)}
        max_joint_rebound = max((x for x in joint_rebound_max.values() if x is not None), default=None)
        stop_failures = []
        if not stop_completed: stop_failures.append("controller_stop_complete_lacks_last_250_valid_physics_dwell_steps_or_exceeded_2s")
        if native_dwell_end_ns is None: stop_failures.append("native_leg_stop_dwell_never_observed")
        if complete_ns is None: stop_failures.append("controller_stop_complete_event_missing")
        envelope = protocol["stop_envelope"]
        joint_disp_limit = float(envelope["max_stop_joint_displacement_rad"])
        axle_disp_limit = float(envelope["max_stop_axle_displacement_m"])
        joint_rebound_limit = float(envelope["max_stop_joint_rebound_rad"])
        if any(v is None or v > joint_disp_limit for v in joint_excursion_max.values()): stop_failures.append("stop_joint_excursion_limit_exceeded_or_missing")
        if max_axle_disp is None or max_axle_disp > axle_disp_limit: stop_failures.append("stop_axle_excursion_limit_exceeded_or_missing")
        if max_joint_rebound is None or max_joint_rebound > joint_rebound_limit: stop_failures.append("stop_joint_rebound_limit_exceeded_or_missing")
        if support_steps != len(frames): stop_failures.append("support_or_frame_integrity_lost_during_stop")
        if trigger_ns is None or complete_ns is None or complete_ns < trigger_ns or complete_ns - trigger_ns > timeout_ns:
            stop_failures.append("stop_timeout_exceeded_or_missing_event")
        stop_summaries[f"{trial[0]}:{trial[1]}"] = {
            "gate": "PASS" if not stop_failures else "FAIL", "failures": stop_failures,
            "trigger_ns": trigger_ns, "first_native_all_four_below_threshold_ns": earliest_native_stop_ns,
            "earliest_native_250_step_dwell_start_ns": frames[native_dwell_begin]["key"][1] if native_dwell_begin is not None else None,
            "earliest_native_250_step_dwell_end_ns": native_dwell_end_ns,
            "controller_confirmed_dwell_start_ns": frames[confirm_dwell_begin]["key"][1] if confirm_dwell_begin is not None else None,
            "controller_confirmed_dwell_end_ns": frames[confirm_dwell_end]["key"][1] if confirm_dwell_end is not None else None,
            "complete_event_ns": complete_ns, "stop_elapsed_to_controller_confirmation_ns": stop_elapsed_ns,
            "controller_confirmation_after_earliest_native_dwell_ns": confirm_delay_ns,
            "controller_confirmation_after_first_native_below_threshold_ns": (
                complete_ns - earliest_native_stop_ns
                if complete_ns is not None and earliest_native_stop_ns is not None else None),
            "controller_confirmed_dwell_steps": (confirm_dwell_end - confirm_dwell_begin + 1) if controller_dwell_ok and confirm_dwell_begin is not None and confirm_dwell_end is not None else 0,
            "max_abs_base_forward_displacement_m": max_disp,
            "max_abs_axle_displacement_m": max_axle_disp,
            "max_abs_stop_joint_excursion_rad": joint_excursion_max,
            "max_stop_joint_rebound_rad": joint_rebound_max,
            "peak_abs_leg_and_wheel_rate_rad_s": {JOINT_NAMES[i]: speed_peak[i] for i in range(6)},
            "peak_abs_native_joint_force_nm": {JOINT_NAMES[MODEL_ORDER[i]]: input_peak[i] for i in range(6)},
            "max_abs_pitch_anchor_error_rad": max(pitch_error, default=None),
            "bilateral_support_valid_steps": support_steps,
            "normal_stop_frames": len(frames),
            "qualification": "empirical event metrics only; no braking qualification",
        }

    # Immutable response model: invoke the existing auditor independently for
    # each trial/phase. Its output still labels the first half diagnostic and
    # second half independent validation; this layer labels moving scope.
    model_module = model_module or load_model_auditor()
    model_results = {}
    model_gate_path = args.response_gate_json
    for trial, intervals in trial_intervals.items():
        for phase, (start, end) in intervals.items():
            count = (end - start) // STEP_NS
            key_name = f"{trial[0]}:{phase}"
            if count < 60:
                model_results[key_name] = {"gate": "FAIL", "status": "NOT_TESTED", "required_steps": 60,
                                           "steps": count, "failures": ["fewer_than_60_frames_for_two_30_step_model_splits"]}
                problems[f"{key_name}:insufficient_model_steps"] += 1
                continue
            model_out = output / "model_response" / trial[0] / phase
            ns = argparse.Namespace(native_wrench_csv=args.derived_native_csv,
                                    contact_frames_csv=args.contact_frames_csv,
                                    geometry_csv=args.derived_geometry_csv,
                                    hold_start_ns=start, hold_end_ns=end,
                                    cpp_bridge=args.cpp_bridge,
                                    response_gate_json=model_gate_path,
                                    output_dir=model_out)
            summary = model_module.run_audit(ns)
            model_results[key_name] = {"gate": summary.get("gate", "FAIL"), "status": "TESTED",
                                       "scope": "moving phase; original per-half nine-DOF gate unchanged",
                                       "steps": count, "failures": summary.get("failures", []),
                                       "split_gates": summary.get("split_gates", {})}

    # Each fixed replay requires 30 consecutive full frames in each phase;
    # model response uses two 30-step fixed halves, so it requires 60 total.
    phase_gate = {}
    for trial, phases in phase_frames_by_trial.items():
        for phase, frames in phases.items():
            valid_streak = 0; max_streak = 0
            previous = None
            for f in frames:
                if f["frame_valid"] == 1 and (previous is None or (f["key"][0] == previous[0] + 1 and f["key"][1] == previous[1] + STEP_NS)):
                    valid_streak += 1
                elif f["frame_valid"] == 1:
                    valid_streak = 1
                else:
                    valid_streak = 0
                max_streak = max(max_streak, valid_streak)
                previous = f["key"]
            required_steps = int(protocol.get("minimum_physics_steps_per_phase", 30))
            passed = max_streak >= required_steps
            phase_gate[f"{trial[0]}:{phase}"] = {"expected_steps": len(frames), "max_contiguous_valid_steps": max_streak,
                                                  "required_contiguous_steps": required_steps,
                                                  "gate": "PASS" if passed else "FAIL"}
            if not passed: problems[f"{trial[0]}:{phase}:fewer_than_30_complete_contiguous_steps"] += 1
    trial_config = {(str(t["trial_id"]), str(t["replay_id"])): t for t in protocol["trials"]}
    excitation_gate = {}
    for trial, phases in phase_frames_by_trial.items():
        config = trial_config.get(trial, {})
        frames = phases.get("acceleration", []) + phases.get("cruise", [])
        counts = {}
        threshold_by_native = {
            0: float(config.get("minimum_hip_excitation_radps", math.inf)),
            3: float(config.get("minimum_hip_excitation_radps", math.inf)),
            1: float(config.get("minimum_knee_excitation_radps", math.inf)),
            4: float(config.get("minimum_knee_excitation_radps", math.inf)),
        }
        for joint_idx, threshold in threshold_by_native.items():
            counts[JOINT_NAMES[joint_idx]] = sum(
                1 for f in frames if f["frame_valid"] == 1 and f["v_before"] is not None and
                abs(f["v_before"][3 + joint_idx]) >= threshold)
        needed = int(config.get("minimum_excitation_steps", 30))
        passed = bool(counts) and all(n >= needed for n in counts.values())
        excitation_gate[f"{trial[0]}:{trial[1]}"] = {"gate": "PASS" if passed else "FAIL",
                                                      "required_steps_each_joint": needed,
                                                      "required_rate_rad_s_by_joint": {JOINT_NAMES[i]: v for i, v in threshold_by_native.items()},
                                                      "actual_steps_at_or_above_rate": counts}
        if not passed:
            problems[f"{trial[0]}:actual_joint_excitation_insufficient"] += 1
    model_pass = bool(model_results) and all(r.get("gate") == "PASS" and r.get("status") == "TESTED" for r in model_results.values())
    stop_pass = bool(stop_summaries) and all(x["gate"] == "PASS" for x in stop_summaries.values())
    excitation_pass = bool(excitation_gate) and all(x["gate"] == "PASS" for x in excitation_gate.values())
    raw_pass = (bool(trial_intervals) and all(x["gate"] == "PASS" for x in phase_gate.values()) and
                not problems and not any(native_problems.values()) and not any(engine_problems.values()) and
                not any(contact_problems.values()) and not any(geometry_problems.values()))
    overall = raw_pass and model_pass and stop_pass and excitation_pass
    write_csv(output / "phase_frames.csv", phase_rows)
    write_csv(output / "command_latency_intervals.csv", latency_rows)
    summary = {
        "audit": "ground_motion_trial", "gate": "PASS" if overall else "FAIL",
        "scope": "two frozen low-amplitude replay trials; acceleration, cruise, normal stop; fixed model response per phase",
        "raw_input_integrity_gate": "PASS" if raw_pass else "FAIL",
        "nine_dof_model_gate": "PASS" if model_pass else "FAIL",
        "empirical_stop_event_gate": "PASS" if stop_pass else "FAIL",
        "actual_joint_excitation_gate": "PASS" if excitation_pass else "FAIL",
        "brake_qualification": "NOT_QUALIFIED",
        "airborne_jumps": 0, "accepted_jumps": 0,
        "input_files": {name: str(getattr(args, attr)) for name, attr in {
            "phase_events_csv": "phase_events_csv", "protocol_json": "protocol_json",
            "command_publication_csv": "command_publication_csv", "wheel_servo_csv": "wheel_servo_csv",
            "engine_csv": "engine_csv", "derived_native_csv": "derived_native_csv",
            "derived_geometry_csv": "derived_geometry_csv", "contact_frames_csv": "contact_frames_csv",
            "response_gate_json": "response_gate_json", "cpp_bridge": "cpp_bridge"}.items()},
        "sha256": {name: digest(Path(getattr(args, attr))) for name, attr in {
            "phase_events_csv": "phase_events_csv", "protocol_json": "protocol_json",
            "command_publication_csv": "command_publication_csv", "wheel_servo_csv": "wheel_servo_csv",
            "engine_csv": "engine_csv", "derived_native_csv": "derived_native_csv",
            "derived_geometry_csv": "derived_geometry_csv", "contact_frames_csv": "contact_frames_csv",
            "response_gate_json": "response_gate_json", "cpp_bridge": "cpp_bridge"}.items()},
        "protocol": {"fixed_profile_sha256": protocol["profile_sha256"], "protocol_file_sha256": profile_digest,
                     "trial_count": len(protocol["trials"]), "phase_order": list(PHASES),
                     "limits": protocol["limits"]},
        "phase_windows": {f"{t[0]}:{phase}": {"start_ns": ab[0], "end_ns_exclusive": ab[1],
                                                   "expected_steps": (ab[1]-ab[0])//STEP_NS}
                          for t, phases in trial_intervals.items() for phase, ab in phases.items()},
        "phase_continuity_and_minimum_frames": phase_gate,
        "post_to_next_before_velocity_continuity": continuity,
        "actual_joint_excitation": excitation_gate,
        "command_latency": {"csv": "command_latency_intervals.csv", "records": len(latency_rows),
                             "meaning": "interval only; applied JointForceCmd has no command_id; repeated values remain ambiguous"},
        "reverse_effort_acceleration_counts": {f"{k[0][0]}:{k[1]}:{k[2]}": v for k, v in reverse_counts.items()},
        "stop_events": stop_summaries,
        "preparation_events": {
            name: [{"event_id": row.get("event_id"), "sim_event_ns": integer(row.get("sim_event_ns")),
                    "wall_event_ns": integer(row.get("wall_event_ns")), "phase": row.get("phase"),
                    "trial_id": row.get("trial_id"), "replay_id": row.get("replay_id"),
                    "reason": row.get("reason")}
                   for row in global_events.get(name, [])]
            for name in ("law_blend_start", "law_blend_complete", "campaign_complete")},
        "model_response": model_results,
        "source_problems": {"events": dict(event_problems), "commands_and_protocol": dict(ref_problems),
                            "native": dict(native_problems), "engine": dict(engine_problems),
                            "contacts": dict(contact_problems), "geometry": dict(geometry_problems),
                            "servo": dict(servo_problems)},
        "failures": sorted(problems.keys()),
        "limitations": ["This fixed standing-neighborhood protocol does not qualify other configurations or speeds.",
                        "Empirical stopping metrics are observations, not a braking qualification.",
                        "The model response audit uses unchanged absolute-RMS and 20% relative-RMS gates per phase.",
                        "Wheel-servo saturation is recorded and remains a valid applied input; it is not an automatic failure."],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return summary


def engine_audit_required_columns() -> set[str]:
    return {"sim_time_ns", "iteration", "dt_ns", "phase", "entity_type", "entity_name", "entity_id",
            "entity_present", "time_valid", "position_valid", "velocity_valid", "physical_state_time_ns",
            "wall_steady_time_ns", "position_x", "position_y", "position_z", "quat_w", "quat_x", "quat_y",
            "quat_z", "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz",
            "joint_dof", "joint_position_0", "joint_velocity_0", "reason"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("phase-events-csv", "protocol-json", "command-publication-csv", "wheel-servo-csv",
                 "engine-csv", "derived-native-csv", "derived-geometry-csv", "contact-frames-csv",
                 "response-gate-json", "cpp-bridge", "output-dir"):
        parser.add_argument("--" + name, dest=name.replace("-", "_"), type=Path, required=True)
    args = parser.parse_args()
    try:
        result = audit(args)
    except (OSError, ValueError, csv.Error, json.JSONDecodeError, RuntimeError) as exc:
        parser.error(str(exc))
    print(json.dumps({"gate": result["gate"], "raw_input_integrity_gate": result["raw_input_integrity_gate"],
                      "nine_dof_model_gate": result["nine_dof_model_gate"],
                      "empirical_stop_event_gate": result["empirical_stop_event_gate"],
                      "brake_qualification": result["brake_qualification"],
                      "failures": result["failures"]}, indent=2))
    return 0 if result["gate"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
