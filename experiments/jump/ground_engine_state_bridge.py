#!/usr/bin/env python3
"""Read-only bridge from paired Physics CSV traces to /ground_input_engine_state.

The parser/assembler is ROS-independent so it can be tested and replayed offline.
The ROS node only tails files and publishes a String; it has no actuator, clock,
or service publishers.

ECS1 is the 41-field NCS1-shaped tuple: magic, sim_ns, iteration, dt_ns, four
bitmasks, post q[9], post v[9], before v[9], and actual before-Physics u[6].
State order is base-forward/z/roll-x, hipL/kneeL/hipR/kneeR, wheelL/wheelR;
input order is hipL/kneeL/hipR/kneeR/wheelL/wheelR. The stream merge waits for
both CSV sources to advance beyond a frame before committing it, usually one
1ms physics step. File flush, OS scheduling, and ROS delivery add unbounded
delay; source wall age is rejected above 10ms, and the receiver must still
enforce its own 10ms arrival timeout. This cannot prove a shared run identity
for two stale files that happen to repeat the same keys; the runner must create
both inputs for the same fresh run rather than reuse archived paths.
Each mask bit corresponds to its vector index; an unset bit's value is NaN.
Structural source faults latch the bridge invalid until restart. The existing
NCS1 parser only accepts full masks, so the receiver needs a separate ECS1
parser that preserves partial masks instead of passing ECS1 through NCS1.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator


STEP_NS = 1_000_000
MASK9 = (1 << 9) - 1
MASK6 = (1 << 6) - 1

# Full scoped names are part of the file contract. The engine q/v order is
# model order; native joint_index is HL, KL, WL, HR, KR, WR.
BASE_LINK = "flat_jump_world::bbot::base_link"
ENGINE_JOINTS = (
    "flat_jump_world::bbot::link_002_joint",  # hip L
    "flat_jump_world::bbot::link_003_joint",  # knee L
    "flat_jump_world::bbot::link_004_joint",  # wheel L
    "flat_jump_world::bbot::link_005_joint",  # hip R
    "flat_jump_world::bbot::link_006_joint",  # knee R
    "flat_jump_world::bbot::link_007_joint",  # wheel R
)
NATIVE_JOINTS = (
    "link_002_joint", "link_003_joint", "link_004_joint",
    "link_005_joint", "link_006_joint", "link_007_joint",
)
MODEL_INPUT_FROM_NATIVE = (0, 1, 3, 4, 2, 5)  # HL, KL, HR, KR, WL, WR
ENGINE_ENTITIES = {("link", BASE_LINK)} | {
    ("joint", name) for name in ENGINE_JOINTS
} | {
    ("link", "flat_jump_world::bbot::link_004"),
    ("link", "flat_jump_world::bbot::link_007"),
}

ENGINE_REQUIRED = {
    "sim_time_ns", "iteration", "phase", "entity_type", "entity_name",
    "dt_ns", "physical_state_time_ns", "wall_steady_time_ns", "entity_present", "time_valid",
    "position_valid", "velocity_valid", "position_x", "position_y", "position_z",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz",
    "joint_dof", "joint_position_0", "joint_velocity_0",
}
NATIVE_REQUIRED = {
    "seq", "sim_time_ns", "physics_iteration", "dt_ns", "joint_index", "joint_name",
    "before_physics_phase", "before_physics_sim_time_ns",
    "before_physics_iteration", "before_physics_dt_ns",
    "before_physics_joint_force_cmd_component_present",
    "before_physics_joint_force_cmd_valid", "before_physics_joint_force_cmd_sim_input",
}


def _int(value: Any) -> int | None:
    try:
        if value is None or str(value).strip() == "":
            return None
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


def _float(value: Any) -> float | None:
    try:
        if value is None or str(value).strip() == "":
            return None
        result = float(str(value).strip())
        return result if math.isfinite(result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _true(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _row_key(row: dict[str, str], iteration_field: str = "iteration") -> tuple[int, int, int] | None:
    sim_ns = _int(row.get("sim_time_ns"))
    iteration = _int(row.get(iteration_field))
    dt_ns = _int(row.get("dt_ns"))
    if sim_ns is None or iteration is None or dt_ns is None:
        return None
    if sim_ns < 0 or iteration < 0:
        return None
    return iteration, sim_ns, dt_ns


def _order_key(key: tuple[int, int, int]) -> tuple[int, int]:
    return key[0], key[1]


def _identity_sort(key: tuple[int, int, int]) -> tuple[int, int, int]:
    return key


@dataclass
class SourceFrame:
    key: tuple[int, int, int]
    rows: dict[Any, dict[str, str]] = field(default_factory=dict)
    issues: list[str] = field(default_factory=list)
    wall_min_ns: int | None = None
    wall_max_ns: int | None = None
    phase_wall_bounds: dict[str, list[int | None]] = field(default_factory=dict)
    phase_row_counts: Counter[str] = field(default_factory=Counter)


class OrderedSource:
    """One source's strictly ordered, one-step frame stream."""

    def __init__(self, name: str, required_fields: set[str]):
        self.name = name
        self.required_fields = required_fields
        self.current: SourceFrame | None = None
        self.closed: deque[SourceFrame] = deque()
        self.last_closed_key: tuple[int, int, int] | None = None
        self.errors: Counter[str] = Counter()

    def _issue(self, reason: str) -> None:
        self.errors[reason] += 1

    def add(self, row: dict[str, str], identity: Any,
            iteration_field: str = "iteration") -> bool:
        missing_fields = self.required_fields - row.keys()
        if missing_fields:
            self._issue("missing_columns:" + ";".join(sorted(missing_fields)))
            return False
        key = _row_key(row, iteration_field)
        if key is None:
            self._issue("unparseable_key")
            return False
        if key[2] != STEP_NS:
            self._issue("dt_not_1ms")
        if self.current is None:
            self.current = SourceFrame(key)
        elif key == self.current.key:
            pass
        else:
            prior = self.current.key
            if key[0] <= prior[0] or key[1] <= prior[1]:
                self._issue("clock_rollback_or_duplicate_frame_key")
                self.current.issues.append("clock_rollback_or_duplicate_frame_key")
                return False
            if key[0] != prior[0] + 1 or key[1] != prior[1] + STEP_NS:
                self._issue("step_gap_or_nonmonotonic_clock")
                self.current.issues.append("step_gap_or_nonmonotonic_clock")
            self._close_current()
            self.current = SourceFrame(key)
        assert self.current is not None
        if identity in self.current.rows:
            self.current.issues.append("duplicate_entity")
            self._issue("duplicate_entity")
            return False
        self.current.rows[identity] = row
        if key[2] != STEP_NS:
            self.current.issues.append("dt_not_1ms")
        return True

    def _close_current(self) -> None:
        if self.current is None:
            return
        if self.last_closed_key is not None and _order_key(self.current.key) <= _order_key(self.last_closed_key):
            self.current.issues.append("source_order_rollback_or_duplicate")
            self._issue("source_order_rollback_or_duplicate")
        self.closed.append(self.current)
        self.last_closed_key = self.current.key
        self.current = None

    def close_for_eof(self) -> None:
        self._close_current()


@dataclass
class EmittedFrame:
    key: tuple[int, int, int]
    payload: str
    qmask: int
    vmask: int
    before_vmask: int
    input_mask: int
    reasons: tuple[str, ...]
    source_wall_min_ns: int | None = None
    source_wall_max_ns: int | None = None
    phase_wall_bounds: dict[str, tuple[int | None, int | None]] = field(default_factory=dict)
    phase_row_counts: dict[str, int] = field(default_factory=dict)

    @property
    def fully_valid(self) -> bool:
        return (self.qmask == MASK9 and self.vmask == MASK9 and
                self.before_vmask == MASK9 and self.input_mask == MASK6)


def source_frame_age_ns(frame: EmittedFrame, now_steady_ns: int) -> int | None:
    if frame.source_wall_min_ns is None:
        return None
    return now_steady_ns - frame.source_wall_min_ns


def source_frame_fresh(frame: EmittedFrame, now_steady_ns: int,
                       max_age_ns: int = 10_000_000) -> bool:
    age = source_frame_age_ns(frame, now_steady_ns)
    return age is not None and 0 <= age <= max_age_ns


class StartupHistoryPolicy:
    """Skip only a pre-start source-history prefix; fail closed afterwards."""

    def __init__(self, process_start_steady_ns: int,
                 max_age_ns: int = 10_000_000):
        self.process_start_steady_ns = process_start_steady_ns
        self.max_age_ns = max_age_ns
        self.prefix_closed = False
        self.skipped_frames = 0

    def classify(self, frame: EmittedFrame, now_steady_ns: int,
                 source_fault_latched: bool = False) -> tuple[str, str]:
        # A synthetic fault packet has no source key/wall evidence and can
        # never be treated as ignorable startup history.
        if source_fault_latched:
            return "fault", "source_integrity_fault_latched"
        if frame.key[0] < 0 or frame.source_wall_min_ns is None:
            return "fault", "source_wall_unavailable"
        frame_max = frame.source_wall_max_ns or frame.source_wall_min_ns
        if frame_max < self.process_start_steady_ns:
            if self.prefix_closed:
                return "fault", "pre_start_wall_after_fresh_prefix"
            return "skip", "engine_wall_before_bridge_start"
        if frame.source_wall_min_ns < self.process_start_steady_ns:
            return "fault", "startup_frame_crosses_bridge_start_cutover"
        self.prefix_closed = True
        if not source_frame_fresh(frame, now_steady_ns, self.max_age_ns):
            return "fault", "engine_source_wall_stale_or_future"
        return "fresh", ""


def startup_skip_record(frame: EmittedFrame, process_start_steady_ns: int,
                        reason: str) -> dict[str, str]:
    """Stable audit row for one complete paired frame omitted at startup."""
    before = frame.phase_wall_bounds.get("before_step", (None, None))
    after = frame.phase_wall_bounds.get("after_step", (None, None))
    return {
        "iteration": str(frame.key[0]), "sim_time_ns": str(frame.key[1]),
        "dt_ns": str(frame.key[2]), "reason": reason,
        "process_start_steady_ns": str(process_start_steady_ns),
        "source_wall_min_ns": _csv_optional(frame.source_wall_min_ns),
        "source_wall_max_ns": _csv_optional(frame.source_wall_max_ns),
        "before_step_rows": str(frame.phase_row_counts.get("before_step", 0)),
        "before_step_wall_min_ns": _csv_optional(before[0]),
        "before_step_wall_max_ns": _csv_optional(before[1]),
        "after_step_rows": str(frame.phase_row_counts.get("after_step", 0)),
        "after_step_wall_min_ns": _csv_optional(after[0]),
        "after_step_wall_max_ns": _csv_optional(after[1]),
    }


def _csv_optional(value: int | None) -> str:
    return "" if value is None else str(value)


def _nan_vector(size: int) -> list[float]:
    return [math.nan] * size


def _mask_put(values: list[float], mask: int, index: int, value: float | None) -> int:
    if value is None:
        values[index] = math.nan
        return mask
    values[index] = value
    return mask | (1 << index)


def _engine_entity_valid(row: dict[str, str], kind: str) -> bool:
    if not (_true(row.get("entity_present")) and _true(row.get("time_valid"))):
        return False
    if kind == "link":
        return _true(row.get("position_valid")) and _true(row.get("velocity_valid"))
    return (_true(row.get("position_valid")) and _true(row.get("velocity_valid")) and
            _int(row.get("joint_dof")) == 1)


def _roll_x(row: dict[str, str]) -> float | None:
    qw = _float(row.get("quat_w"))
    qx = _float(row.get("quat_x"))
    qy = _float(row.get("quat_y"))
    qz = _float(row.get("quat_z"))
    if any(x is None for x in (qw, qx, qy, qz)):
        return None
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-6:
        return None
    return 2.0 * math.atan2(qx, qw)


def _link_payload_valid(row: dict[str, str]) -> bool:
    if not _engine_entity_valid(row, "link"):
        return False
    values = [_float(row.get(field)) for field in (
        "position_x", "position_y", "position_z", "quat_w", "quat_x", "quat_y", "quat_z",
        "linear_vx", "linear_vy", "linear_vz", "angular_vx", "angular_vy", "angular_vz")]
    if any(value is None for value in values):
        return False
    qnorm = math.sqrt(sum(value * value for value in values[3:7]))
    return math.isfinite(qnorm) and abs(qnorm - 1.0) <= 1e-6


def _state_vector(frame: SourceFrame | None, phase: str,
                  before: bool) -> tuple[list[float], int, list[float], int, list[str]]:
    q = _nan_vector(9)
    v = _nan_vector(9)
    qmask = 0
    vmask = 0
    reasons: list[str] = []
    if frame is None:
        return q, qmask, v, vmask, [f"missing_engine_{phase}_frame"]
    if frame.issues:
        return q, 0, v, 0, list(frame.issues)
    expected_state_ns = frame.key[1] - frame.key[2] if before else frame.key[1]
    # Group validity is per entity and phase; no missing link/joint is filled.
    base = frame.rows.get((phase, "link", BASE_LINK))
    if base is None:
        reasons.append(f"{phase}:missing_base_link")
    else:
        if _int(base.get("physical_state_time_ns")) != expected_state_ns:
            reasons.append(f"{phase}:base_physical_state_time_mismatch")
        if _link_payload_valid(base):
            qmask = _mask_put(q, qmask, 0, _float(base.get("position_y")))
            qmask = _mask_put(q, qmask, 1, _float(base.get("position_z")))
            qmask = _mask_put(q, qmask, 2, _roll_x(base))
            vmask = _mask_put(v, vmask, 0, _float(base.get("linear_vy")))
            vmask = _mask_put(v, vmask, 1, _float(base.get("linear_vz")))
            vmask = _mask_put(v, vmask, 2, _float(base.get("angular_vx")))
        else:
            reasons.append(f"{phase}:base_link_state_mismatch")
    for wheel_link in ("flat_jump_world::bbot::link_004", "flat_jump_world::bbot::link_007"):
        row = frame.rows.get((phase, "link", wheel_link))
        if row is None or _int(row.get("physical_state_time_ns")) != expected_state_ns or \
                not _link_payload_valid(row):
            reasons.append(f"{phase}:wheel_link_state_mismatch:{wheel_link.rsplit('::', 1)[-1]}")
    for engine_index, name in enumerate(ENGINE_JOINTS):
        native_index_to_model = {0: 3, 1: 4, 2: 7, 3: 5, 4: 6, 5: 8}
        dof = native_index_to_model[engine_index]
        row = frame.rows.get((phase, "joint", name))
        if row is None:
            reasons.append(f"{phase}:missing_joint:{name.rsplit('::', 1)[-1]}")
            continue
        if _int(row.get("physical_state_time_ns")) != expected_state_ns:
            reasons.append(f"{phase}:joint_physical_state_time_mismatch:{name.rsplit('::', 1)[-1]}")
        if not _engine_entity_valid(row, "joint"):
            reasons.append(f"{phase}:joint_invalid:{name.rsplit('::', 1)[-1]}")
            continue
        qmask = _mask_put(q, qmask, dof, _float(row.get("joint_position_0")))
        vmask = _mask_put(v, vmask, dof, _float(row.get("joint_velocity_0")))
    return q, qmask, v, vmask, reasons


def _native_inputs(frame: SourceFrame | None) -> tuple[list[float], int, list[str]]:
    u = _nan_vector(6)
    mask = 0
    reasons: list[str] = []
    if frame is None:
        return u, mask, ["missing_native_frame"]
    if frame.issues:
        return u, 0, list(frame.issues)
    by_index: dict[int, dict[str, str]] = {}
    for index in range(6):
        row = frame.rows.get(index)
        if row is None:
            reasons.append(f"missing_native_joint:{index}")
            continue
        if _int(row.get("before_physics_sim_time_ns")) != frame.key[1] or \
                _int(row.get("before_physics_iteration")) != frame.key[0] or \
                _int(row.get("before_physics_dt_ns")) != frame.key[2] or \
                row.get("before_physics_phase") != "before_physics_update":
            reasons.append(f"native_before_key_or_phase_mismatch:{index}")
            continue
        if _int(row.get("seq")) != frame.key[0] - 1:
            reasons.append(f"native_sequence_iteration_mismatch:{index}")
            continue
        if _int(row.get("joint_index")) != index or row.get("joint_name") != NATIVE_JOINTS[index]:
            reasons.append(f"native_joint_identity_mismatch:{index}")
            continue
        by_index[index] = row
    for model_slot, native_index in enumerate(MODEL_INPUT_FROM_NATIVE):
        row = by_index.get(native_index)
        if row is None:
            continue
        if not (_true(row.get("before_physics_joint_force_cmd_component_present")) and
                _true(row.get("before_physics_joint_force_cmd_valid"))):
            reasons.append(f"native_joint_force_invalid:{native_index}")
            continue
        value = _float(row.get("before_physics_joint_force_cmd_sim_input"))
        if value is None:
            u[model_slot] = math.nan
            reasons.append(f"native_joint_force_nonfinite:{native_index}")
            continue
        u[model_slot] = value
        if math.isfinite(value):
            mask |= 1 << model_slot
    return u, mask, reasons


def _packet(key: tuple[int, int, int], q: list[float], qmask: int,
            v: list[float], vmask: int, before_v: list[float], before_mask: int,
            u: list[float], input_mask: int) -> str:
    fields: list[str] = ["ECS1", str(key[1]), str(key[0]), str(key[2]),
                         str(qmask), str(vmask), str(before_mask), str(input_mask)]
    fields.extend("nan" if not math.isfinite(x) else repr(x) for x in q)
    fields.extend("nan" if not math.isfinite(x) else repr(x) for x in v)
    fields.extend("nan" if not math.isfinite(x) else repr(x) for x in before_v)
    fields.extend("nan" if not math.isfinite(x) else repr(x) for x in u)
    assert len(fields) == 41
    return ",".join(fields)


def invalid_packet(reason: str = "bridge_integrity_fault") -> EmittedFrame:
    key = (-1, -1, STEP_NS)
    q, v, bv, u = _nan_vector(9), _nan_vector(9), _nan_vector(9), _nan_vector(6)
    return EmittedFrame(key, _packet(key, q, 0, v, 0, bv, 0, u, 0),
                        0, 0, 0, 0, (reason,), None)


class GroundStateAssembler:
    """Pairs ordered engine/native streams without sorting or dropping bad frames."""

    def __init__(self):
        self.engine = OrderedSource("engine", ENGINE_REQUIRED)
        self.native = OrderedSource("native", NATIVE_REQUIRED)
        self.outputs: deque[EmittedFrame] = deque()
        self.faults: Counter[str] = Counter()
        self.poisoned = False
        self.poison_reason = ""
        self.last_emitted_order: tuple[int, int] | None = None
        self.last_engine_wall_ns: int | None = None

    def _poison(self, reason: str) -> None:
        self.faults[reason] += 1
        if not self.poisoned:
            self.poisoned = True
            self.poison_reason = reason
            self.outputs.append(invalid_packet(reason))

    def add_engine(self, row: dict[str, str]) -> None:
        if self.poisoned:
            return
        if row.get("entity_type") not in {"link", "joint"} or row.get("phase") not in {"before_step", "after_step"}:
            self._poison("engine_unknown_phase_or_entity_type")
            return
        identity = (row.get("phase"), row.get("entity_type"), row.get("entity_name"))
        wall_ns = _int(row.get("wall_steady_time_ns"))
        if wall_ns is None or wall_ns < 0:
            self._poison("engine_wall_steady_invalid")
        elif self.last_engine_wall_ns is not None and wall_ns < self.last_engine_wall_ns:
            self._poison("engine_wall_steady_rollback")
        if wall_ns is not None and wall_ns >= 0:
            self.last_engine_wall_ns = wall_ns
        if identity[1:] not in ENGINE_ENTITIES:
            self._poison("engine_unexpected_full_scoped_entity")
        accepted = self.engine.add(row, identity)
        if not accepted:
            # Duplicate/malformed source rows and clock errors are not skipped.
            if self.engine.errors:
                self._poison(next(reversed(self.engine.errors)))
        if accepted and self.engine.current is not None and self.engine.current.key == _row_key(row):
            current = self.engine.current
            if wall_ns is not None and wall_ns >= 0:
                current.wall_min_ns = wall_ns if current.wall_min_ns is None else min(current.wall_min_ns, wall_ns)
                current.wall_max_ns = wall_ns if current.wall_max_ns is None else max(current.wall_max_ns, wall_ns)
                bounds = current.phase_wall_bounds.setdefault(identity[0], [None, None])
                bounds[0] = wall_ns if bounds[0] is None else min(bounds[0], wall_ns)
                bounds[1] = wall_ns if bounds[1] is None else max(bounds[1], wall_ns)
            current.phase_row_counts[identity[0]] += 1
        self._drain()

    def add_native(self, row: dict[str, str]) -> None:
        if self.poisoned:
            return
        index = _int(row.get("joint_index"))
        identity: Any = index if index is not None else "invalid_joint_index"
        if not self.native.add(row, identity, "physics_iteration"):
            if self.native.errors:
                self._poison(next(reversed(self.native.errors)))
        self._drain()

    def _drain(self, eof: bool = False) -> None:
        eng_head = self.engine.closed[0] if self.engine.closed else None
        nat_head = self.native.closed[0] if self.native.closed else None
        while eng_head is not None or nat_head is not None:
            candidates = [f for f in (eng_head, nat_head) if f is not None]
            key_frame = min(candidates, key=lambda f: _order_key(f.key))
            order = _order_key(key_frame.key)
            # The opposite stream must have advanced past this key (or EOF) so
            # a delayed native/engine append can still reveal a duplicate.
            eng_watermark = None if eof else (self.engine.current.key if self.engine.current else None)
            nat_watermark = None if eof else (self.native.current.key if self.native.current else None)
            if not eof and (eng_watermark is None or nat_watermark is None or
                    _order_key(eng_watermark) <= order or _order_key(nat_watermark) <= order):
                break
            engine_frame = None
            native_frame = None
            if eng_head is not None and eng_head.key == key_frame.key:
                engine_frame = self.engine.closed.popleft()
                eng_head = self.engine.closed[0] if self.engine.closed else None
            if nat_head is not None and nat_head.key == key_frame.key:
                native_frame = self.native.closed.popleft()
                nat_head = self.native.closed[0] if self.native.closed else None
            if engine_frame is None or native_frame is None:
                self._poison("engine_native_frame_key_unmatched")
            if self.last_emitted_order is not None and order != (self.last_emitted_order[0] + 1,
                    self.last_emitted_order[1] + STEP_NS):
                self._poison("paired_output_clock_gap_or_rollback")
            self.last_emitted_order = order
            self.outputs.append(self._build(engine_frame, native_frame, key_frame.key))

    def _build(self, engine_frame: SourceFrame | None, native_frame: SourceFrame | None,
               key: tuple[int, int, int]) -> EmittedFrame:
        reasons: list[str] = []
        if self.poisoned:
            reasons.append(self.poison_reason)
        if key[2] != STEP_NS:
            reasons.append("dt_not_1ms")
        if engine_frame is None:
            reasons.append("missing_engine_frame")
        if native_frame is None:
            reasons.append("missing_native_frame")
        if engine_frame is not None:
            if engine_frame.issues:
                reasons.extend(engine_frame.issues)
            expected = {(phase, *entity) for phase in ("before_step", "after_step") for entity in ENGINE_ENTITIES}
            actual = set(engine_frame.rows)
            if actual != expected:
                reasons.append("engine_frame_incomplete_or_unexpected_entities")
        if native_frame is not None:
            if native_frame.issues:
                reasons.extend(native_frame.issues)
            if set(native_frame.rows) != set(range(6)):
                reasons.append("native_frame_incomplete_or_unexpected_joint_indices")
        q, qm, v, vm, qr = _state_vector(engine_frame, "after_step", False)
        _, _, before_v, bvm, br = _state_vector(engine_frame, "before_step", True)
        u, um, ur = _native_inputs(native_frame)
        reasons.extend(qr + br + ur)
        # Structure/clock errors invalidate every field; ordinary per-channel
        # source validity remains represented by partial masks and NaNs.
        structural = any(x for x in reasons if (
            "missing_" in x or "duplicate" in x or "unexpected" in x or
            "mismatch" in x or "rollback" in x or "gap" in x or
            "dt_not_1ms" in x or "incomplete" in x or "unclosed" in x))
        if structural and not self.poisoned:
            self._poison(next((x for x in reasons if (
                "missing_" in x or "duplicate" in x or "unexpected" in x or
                "mismatch" in x or "rollback" in x or "gap" in x or
                "dt_not_1ms" in x or "incomplete" in x or "unclosed" in x)),
                "paired_frame_structural_error"))
        if self.poisoned or structural:
            qm = vm = bvm = um = 0
            q, v, before_v, u = _nan_vector(9), _nan_vector(9), _nan_vector(9), _nan_vector(6)
        payload = _packet(key, q, qm, v, vm, before_v, bvm, u, um)
        phase_bounds = {}
        phase_counts = {}
        if engine_frame is not None:
            phase_bounds = {phase: (bounds[0], bounds[1])
                            for phase, bounds in engine_frame.phase_wall_bounds.items()}
            phase_counts = dict(engine_frame.phase_row_counts)
        return EmittedFrame(key, payload, qm, vm, bvm, um,
                            tuple(dict.fromkeys(reasons)),
                            engine_frame.wall_min_ns if engine_frame else None,
                            engine_frame.wall_max_ns if engine_frame else None,
                            phase_bounds, phase_counts)

    def finish(self) -> None:
        self.engine.close_for_eof()
        self.native.close_for_eof()
        self._drain(eof=True)
        # No old frame is replayed after a malformed source.

    def pop_outputs(self) -> list[EmittedFrame]:
        out = list(self.outputs)
        self.outputs.clear()
        return out


class CsvTail:
    """Incremental byte tailer that withholds a trailing partial CSV record."""

    def __init__(self, path: Path):
        self.path = path
        self.offset = 0
        self.pending = b""
        self.header: list[str] | None = None
        self.identity: tuple[int, int] | None = None
        self.fault: str | None = None
        self.started = False

    def read_available(self) -> list[dict[str, str]]:
        if self.fault:
            return []
        try:
            st = self.path.stat()
            self.started = True
            identity = (st.st_dev, st.st_ino)
            if self.identity is None:
                self.identity = identity
            elif identity != self.identity or st.st_size < self.offset:
                self.fault = "source_file_replaced_or_truncated"
                return []
            with self.path.open("rb") as f:
                f.seek(self.offset)
                data = f.read()
                self.offset += len(data)
        except OSError as exc:
            if isinstance(exc, FileNotFoundError) and not self.started:
                return []
            self.fault = f"source_file_read_error:{type(exc).__name__}"
            return []
        data = self.pending + data
        lines = data.split(b"\n")
        self.pending = lines.pop()  # Last record may still be in flight.
        rows: list[dict[str, str]] = []
        for raw in lines:
            raw = raw.rstrip(b"\r")
            if not raw:
                continue
            try:
                fields = next(csv.reader([raw.decode("utf-8")]))
            except (UnicodeDecodeError, csv.Error, StopIteration):
                self.fault = "source_csv_parse_error"
                return rows
            if self.header is None:
                self.header = fields
                if len(set(self.header)) != len(self.header):
                    self.fault = "duplicate_csv_column"
                    return rows
            else:
                if len(fields) != len(self.header):
                    self.fault = "csv_column_count_mismatch"
                    return rows
                rows.append(dict(zip(self.header, fields)))
        return rows


def _frame_rows(path: Path) -> Iterator[list[dict[str, str]]]:
    """Read rows in file order, yielding groups at source-key changes."""
    current_key: tuple[int, int, int] | None = None
    current: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            key = _row_key(row, "iteration" if "iteration" in row else "physics_iteration")
            if current and key != current_key:
                yield current
                current = []
            current_key = key
            current.append(row)
    if current:
        yield current


def replay_pair(engine_path: Path, native_path: Path) -> tuple[GroundStateAssembler, float, int, int, int]:
    """Offline full-range replay using the exact incremental assembler."""
    assembler = GroundStateAssembler()
    start = time.perf_counter()
    engine_groups = iter(_frame_rows(engine_path))
    native_groups = iter(_frame_rows(native_path))
    engine_group = next(engine_groups, None)
    native_group = next(native_groups, None)
    engine_rows_count = 0
    native_rows_count = 0
    while engine_group is not None or native_group is not None:
        if engine_group is not None:
            for row in engine_group:
                assembler.add_engine(row)
                engine_rows_count += 1
            engine_group = next(engine_groups, None)
        if native_group is not None:
            for row in native_group:
                assembler.add_native(row)
                native_rows_count += 1
            native_group = next(native_groups, None)
    assembler.finish()
    elapsed = time.perf_counter() - start
    return assembler, elapsed, len(assembler.outputs), engine_rows_count, native_rows_count


STARTUP_SKIP_FIELDS = (
    "iteration", "sim_time_ns", "dt_ns", "reason", "process_start_steady_ns",
    "source_wall_min_ns", "source_wall_max_ns", "before_step_rows",
    "before_step_wall_min_ns", "before_step_wall_max_ns", "after_step_rows",
    "after_step_wall_min_ns", "after_step_wall_max_ns",
)


def _ros_main(engine_csv: Path, native_csv: Path, poll_ms: int,
              startup_skip_log: Path, process_start_steady_ns: int) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f"ROS Python packages are unavailable: {exc}")

    class BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("ground_engine_state_bridge")
            self.publisher = self.create_publisher(String, "/ground_input_engine_state", 10)
            self.engine_tail = CsvTail(engine_csv)
            self.native_tail = CsvTail(native_csv)
            self.assembler = GroundStateAssembler()
            self.startup_policy = StartupHistoryPolicy(process_start_steady_ns)
            self.startup_skip_log = startup_skip_log
            self.startup_skip_log.parent.mkdir(parents=True, exist_ok=True)
            self.skip_stream = self.startup_skip_log.open("a+", newline="", encoding="utf-8")
            self.skip_stream.seek(0, 2)
            self.skip_writer = csv.DictWriter(self.skip_stream, fieldnames=STARTUP_SKIP_FIELDS)
            if self.skip_stream.tell() == 0:
                self.skip_writer.writeheader()
            self.skip_stream.flush()
            self.startup_skip_phase_rows: Counter[str] = Counter()
            self.startup_skip_first_wall_ns: int | None = None
            self.startup_skip_last_wall_ns: int | None = None
            self.timer = self.create_timer(poll_ms / 1000.0, self.poll)
            self.fault_sent = False
            self.published = 0
            self.invalid_published = 0
            self.last_source_progress_wall_ns: int | None = None
            self.get_logger().info(
                f"Read-only ECS1 bridge: engine={engine_csv} native={native_csv}; "
                "no actuator/clock/service publishers")

        def poll(self) -> None:
            engine_rows = self.engine_tail.read_available()
            native_rows = self.native_tail.read_available()
            if engine_rows or native_rows:
                self.last_source_progress_wall_ns = time.monotonic_ns()
            for row in engine_rows:
                self.assembler.add_engine(row)
            for row in native_rows:
                self.assembler.add_native(row)
            source_faults = [x for x in (self.engine_tail.fault, self.native_tail.fault) if x]
            if source_faults and not self.fault_sent:
                self.assembler._poison(";".join(source_faults))
                self.fault_sent = True
            pending_frame = (self.assembler.engine.current is not None or
                             self.assembler.native.current is not None or
                             bool(self.assembler.engine.closed) or bool(self.assembler.native.closed))
            both_sources_started = self.engine_tail.started and self.native_tail.started
            if (both_sources_started and pending_frame and not self.assembler.poisoned and
                    self.last_source_progress_wall_ns is not None and
                    time.monotonic_ns() - self.last_source_progress_wall_ns > 10_000_000):
                self.assembler._poison("unclosed_or_unpaired_frame_timeout")
            for frame in self.assembler.pop_outputs():
                msg = String()
                now_steady_ns = time.monotonic_ns()
                source_age_ns = source_frame_age_ns(frame, now_steady_ns)
                disposition, disposition_reason = self.startup_policy.classify(
                    frame, now_steady_ns, source_fault_latched=self.assembler.poisoned)
                if disposition == "skip":
                    record = startup_skip_record(
                        frame, self.startup_policy.process_start_steady_ns, disposition_reason)
                    self.skip_writer.writerow(record)
                    self.skip_stream.flush()
                    self.startup_policy.skipped_frames += 1
                    for phase in ("before_step", "after_step"):
                        self.startup_skip_phase_rows[phase] += frame.phase_row_counts.get(phase, 0)
                    if frame.source_wall_min_ns is not None:
                        self.startup_skip_first_wall_ns = (
                            frame.source_wall_min_ns if self.startup_skip_first_wall_ns is None
                            else min(self.startup_skip_first_wall_ns, frame.source_wall_min_ns))
                    if frame.source_wall_max_ns is not None:
                        self.startup_skip_last_wall_ns = (
                            frame.source_wall_max_ns if self.startup_skip_last_wall_ns is None
                            else max(self.startup_skip_last_wall_ns, frame.source_wall_max_ns))
                    continue
                if disposition == "fault":
                    # Keep the wire shape but clear every channel. The runtime
                    # consumer must additionally enforce its own 10ms receipt age.
                    self.assembler._poison(disposition_reason)
                    msg.data = invalid_packet(disposition_reason).payload
                    self.invalid_published += 1
                    if self.invalid_published == 1 or self.invalid_published % 1000 == 0:
                        self.get_logger().warning(
                            f"rejecting ECS1 source disposition={disposition_reason} "
                            f"age={source_age_ns}ns; "
                            f"invalid_packets={self.invalid_published}")
                else:
                    msg.data = frame.payload
                    if not frame.fully_valid:
                        self.invalid_published += 1
                        if self.invalid_published == 1 or self.invalid_published % 1000 == 0:
                            self.get_logger().warning(
                                f"ECS1 partial/invalid source frame key={frame.key}: "
                                f"{'|'.join(frame.reasons[:4])}; "
                                f"invalid_packets={self.invalid_published}")
                self.publisher.publish(msg)
                self.published += 1

        def destroy_node(self) -> bool:
            if self.engine_tail.pending or self.native_tail.pending:
                self.get_logger().error("shutdown with an unterminated CSV record; source is not flushed")
            if self.assembler.engine.current is not None or self.assembler.native.current is not None:
                self.get_logger().error("shutdown with an unclosed source frame; final frame was not published")
            self.get_logger().info(
                f"bridge stopped; published_packets={self.published}, "
                f"startup_history_skipped_frames={self.startup_policy.skipped_frames}, "
                f"startup_skip_wall_range_ns={self.startup_skip_first_wall_ns}.."
                f"{self.startup_skip_last_wall_ns}, "
                f"startup_skip_phase_rows={dict(self.startup_skip_phase_rows)}, "
                f"faults={dict(self.assembler.faults)}")
            self.skip_stream.close()
            return super().destroy_node()

    rclpy.init(args=[])
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine-csv", type=Path, required=True)
    parser.add_argument("--native-wrench-csv", type=Path, required=True)
    parser.add_argument("--startup-skip-log", type=Path,
                        help="CSV audit of complete paired pre-start frames skipped at startup; "
                             "defaults to <engine-csv>.bridge_startup_skips.csv")
    parser.add_argument("--poll-ms", type=int, default=1)
    parser.add_argument("--verify-existing", action="store_true",
                        help="offline full-range paired parse; do not initialize ROS")
    args = parser.parse_args(argv)
    if args.poll_ms <= 0:
        parser.error("--poll-ms must be positive")
    if args.verify_existing:
        assembler, elapsed, count, engine_rows_count, native_rows_count = replay_pair(
            args.engine_csv, args.native_wrench_csv)
        outputs = assembler.pop_outputs()
        valid = sum(frame.fully_valid for frame in outputs)
        partial = sum(not frame.fully_valid for frame in outputs)
        mask_histogram = Counter(
            f"q={frame.qmask:03x},v={frame.vmask:03x},before_v={frame.before_vmask:03x},u={frame.input_mask:02x}"
            for frame in outputs)
        reason_histogram = Counter(reason for frame in outputs for reason in frame.reasons)
        summary = {
            "mode": "offline_read_only",
            "engine_rows": engine_rows_count,
            "native_rows": native_rows_count,
            "paired_packets": count,
            "fully_valid_packets": valid,
            "partial_or_invalid_packets": partial,
            "mask_patterns": dict(mask_histogram),
            "frame_reason_counts": dict(reason_histogram),
            "fatal_faults": dict(assembler.faults),
            "engine_source_errors": dict(assembler.engine.errors),
            "native_source_errors": dict(assembler.native.errors),
            "parse_seconds": elapsed,
            "paired_packets_per_second": count / elapsed if elapsed else None,
            "freshness_contract": "runtime receiver must reject source packets older than 10ms",
        }
        import json
        print(json.dumps(summary, indent=2, sort_keys=True))
        return int(bool(assembler.faults))
    startup_skip_log = args.startup_skip_log or Path(str(args.engine_csv) + ".bridge_startup_skips.csv")
    # Capture the cutover before ROS initialization or any file polling.
    process_start_steady_ns = time.monotonic_ns()
    _ros_main(args.engine_csv, args.native_wrench_csv, args.poll_ms,
              startup_skip_log, process_start_steady_ns)
    return 0


def _iter_csv_rows(path: Path) -> Iterator[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        yield from csv.DictReader(stream)


if __name__ == "__main__":
    raise SystemExit(main())
