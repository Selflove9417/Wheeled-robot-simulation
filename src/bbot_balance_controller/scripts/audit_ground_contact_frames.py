#!/usr/bin/env python3
"""Validate complete per-physics-step ground contact frames from Gazebo ECM."""
import csv
import json
from pathlib import Path

WORLD_PREFIX = "flat_jump_world::"
GROUND = "ground_plane::link::collision"
WHEELS = {
    "bbot::link_004::link_004_collision_collision",
    "bbot::link_007::link_007_collision_collision",
}
HEADER = {"seq", "sim_time_ns", "physics_iteration", "dt_ns", "frame_valid",
          "num_contacts", "collision_pairs_json", "error"}
STEP_NS = 1_000_000


def parse_frame(row):
    """Return (stamp_ns, seq, iteration, dt_ns, wheel-set) or raise ValueError."""
    if not HEADER.issubset(row):
        raise ValueError("missing frame CSV fields")
    if row["frame_valid"] != "1" or row.get("error", ""):
        raise ValueError("invalid frame flag or plugin error")
    try:
        seq = int(row["seq"])
        stamp = int(row["sim_time_ns"])
        iteration = int(row["physics_iteration"])
        dt_ns = int(row["dt_ns"])
        count = int(row["num_contacts"])
        pairs = json.loads(row["collision_pairs_json"])
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"malformed frame payload: {exc}") from exc
    if seq < 0 or stamp < 0 or iteration < 0 or dt_ns != STEP_NS or count < 0:
        raise ValueError("invalid sequence, time, iteration, dt, or contact count")
    if not isinstance(pairs, list) or len(pairs) != count:
        raise ValueError("contact count does not match actual pair payload")
    wheel_set = set()
    canonical_pairs = set()
    for pair in pairs:
        if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(x, str) for x in pair):
            raise ValueError("malformed collision pair")
        names = []
        for name in pair:
            if not name.startswith(WORLD_PREFIX):
                raise ValueError(f"collision outside expected world scope: {name}")
            names.append(name[len(WORLD_PREFIX):])
        if GROUND not in names:
            raise ValueError("contact pair does not include the exact ground collision")
        robots = set(names) - {GROUND}
        if len(robots) != 1 or not robots.issubset(WHEELS):
            raise ValueError(f"non-wheel or unknown ground collision: {sorted(robots)}")
        wheel = next(iter(robots))
        wheel_set.add(wheel)
        canonical_pairs.add(tuple(sorted((GROUND, wheel))))
    return stamp, seq, iteration, dt_ns, wheel_set, canonical_pairs


def read_frame_rows(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not HEADER.issubset(reader.fieldnames):
            raise ValueError("complete contact-frame CSV header is missing")
        return list(reader)


def validate_active_trace(log_path, frame_path):
    """Validate source frames in input order over all nonzero jump trace."""
    with Path(log_path).open(newline="") as stream:
        trace = list(csv.DictReader(stream))
    active = [r for r in trace if r.get("jump_id") not in (None, "", "0")]
    if not active:
        return False, "no accepted jump trace", []
    try:
        start = int(round(float(active[0]["timestamp"]) * 1e9))
        end = int(round(float(trace[-1]["timestamp"]) * 1e9))
        rows = read_frame_rows(frame_path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return False, f"cannot read active trace/frame bounds: {exc}", []
    parsed = []
    active_frames = []
    try:
        previous = None
        for row in rows:
            # Startup frames can be invalid while the world's contact system
            # creates its component. Still enforce source ordering and every
            # physics tick; only active-session payload must be contact-valid.
            seq = int(row["seq"])
            stamp = int(row["sim_time_ns"])
            iteration = int(row["physics_iteration"])
            dt_ns = int(row["dt_ns"])
            frame_meta = (stamp, seq, iteration, dt_ns)
            if previous:
                if (seq != previous[1] + 1 or iteration != previous[2] + 1 or
                        stamp - previous[0] != dt_ns or dt_ns != STEP_NS):
                    raise ValueError("sequence, iteration, or sim-time discontinuity in source order")
            elif dt_ns != STEP_NS:
                raise ValueError("unexpected initial physics step size")
            previous = frame_meta
            if start <= stamp <= end:
                frame = parse_frame(row)
                parsed.append(frame)
                active_frames.append(frame)
    except ValueError as exc:
        return False, str(exc), parsed
    if not active_frames:
        return False, "no complete frames cover accepted jump trace", parsed
    if (active_frames[0][0] > start or active_frames[-1][0] < end or
            any(b[0] - a[0] != STEP_NS for a, b in zip(active_frames, active_frames[1:]))):
        return False, "complete contact frames do not cover every active physics step", parsed
    return True, "complete 1kHz source frames cover active trace", parsed


def ready_from_rows(rows, required_wheels=2):
    """True only when the tail has two consecutive valid wheel-contact frames."""
    if len(rows) < 2:
        return False
    try:
        a, b = parse_frame(rows[-2]), parse_frame(rows[-1])
    except ValueError:
        return False
    return (b[1] == a[1] + 1 and b[2] == a[2] + 1 and
            b[0] - a[0] == STEP_NS and a[4] == WHEELS and b[4] == WHEELS)


def flight_gap(parsed_frames, entry_ns):
    """Return last any-wheel contact and first later contact around FLIGHT."""
    prior = [f for f in parsed_frames if f[0] < entry_ns]
    later = [f for f in parsed_frames if f[0] > entry_ns]
    last_by_wheel = {wheel: next((f for f in reversed(prior) if wheel in f[4]), None)
                     for wheel in WHEELS}
    if any(frame is None for frame in last_by_wheel.values()):
        raise ValueError("missing full-frame contact history for one wheel")
    start_ns = max(frame[0] for frame in last_by_wheel.values())
    start = next(f for f in prior if f[0] == start_ns)
    if any(start_ns - frame[0] > 20_000_000 for frame in last_by_wheel.values()):
        raise ValueError("left/right final contact evidence is not bilateral within 20ms")
    landing = next((f for f in later if f[4]), None)
    if landing is None:
        raise ValueError("no first wheel-ground contact frame after FLIGHT entry")
    # Both wheels must be absent between takeoff boundary and first landing.
    if any(f[4] for f in parsed_frames if start[0] < f[0] < landing[0]):
        raise ValueError("wheel contact resumed before the first post-FLIGHT landing boundary")
    return start[0] * 1e-9, landing[0] * 1e-9


def companion_frame_path(log_path):
    path = Path(log_path)
    if path.name.endswith("_log.csv"):
        return path.with_name(path.name[:-len("_log.csv")] + "_ground_frames.csv")
    return path.with_name(path.stem + "_ground_frames.csv")


def crosscheck_native_events(parsed_frames, contacts_path):
    """Compare overlapping positive native events against complete ECM pairs."""
    try:
        with Path(contacts_path).open(newline="") as stream:
            rows = list(csv.DictReader(stream))
    except OSError as exc:
        return [f"cannot read native event CSV for cross-check: {exc}"]
    frame_by_stamp = {f[0]: f[5] for f in parsed_frames}
    native_by_stamp = {}
    for row in rows:
        if row.get("side") != "GROUND":
            continue
        try:
            if int(float(row.get("num_contacts") or 0)) <= 0:
                continue
            stamp = int(round(float(row["stamp_sec"]) * 1e9))
        except (KeyError, TypeError, ValueError):
            return ["malformed positive native ground event cannot be cross-checked"]
        def normalize(name):
            if name.startswith(WORLD_PREFIX):
                name = name[len(WORLD_PREFIX):]
            return name
        if not row.get("collision_1") or not row.get("collision_2"):
            return ["positive native ground event is missing collision names"]
        pair = tuple(sorted((normalize(row.get("collision_1", "")),
                             normalize(row.get("collision_2", "")))))
        native_by_stamp.setdefault(stamp, set()).add(pair)
    failures = []
    for stamp in sorted(set(frame_by_stamp) & set(native_by_stamp)):
        if frame_by_stamp[stamp] != native_by_stamp[stamp]:
            failures.append(f"native positive events disagree with complete frame at {stamp}ns")
    return failures


def audit_session(log_path, parsed_frames):
    """Check touchdown ordering and continuous ground support over each hold."""
    with Path(log_path).open(newline="") as stream:
        trace = list(csv.DictReader(stream))
    failures = []
    jump_ids = []
    for row in trace:
        jid = row.get("jump_id", "")
        if jid not in ("", "0", None) and (not jump_ids or jump_ids[-1] != jid):
            jump_ids.append(jid)
    for jid in jump_ids:
        hop = [r for r in trace if r.get("jump_id") == jid]
        flights = [r for r in hop if r.get("state_name") == "FLIGHT"]
        touchdown = next((r for r in hop if r.get("state_name") == "TOUCHDOWN_BUFFER"), None)
        balance = next((r for r in hop if r.get("state_name") == "BALANCE"), None)
        if not flights or touchdown is None or balance is None:
            failures.append(f"jump_id {jid} missing FLIGHT/touchdown/BALANCE for full-frame audit")
            continue
        entry_ns = int(round(float(flights[0]["timestamp"]) * 1e9))
        td_ns = int(round(float(touchdown["timestamp"]) * 1e9))
        first_contact = next((f for f in parsed_frames
                              if entry_ns <= f[0] <= td_ns + 150_000_000 and f[4]), None)
        if first_contact is None:
            failures.append(f"jump_id {jid} has no wheel contact at touchdown in complete frames")
        balance_ns = int(round(float(balance["timestamp"]) * 1e9))
        hold_end = balance_ns + 12_000_000_000
        hold_contacts = [f[0] for f in parsed_frames
                         if balance_ns <= f[0] <= hold_end and f[4]]
        if (not hold_contacts or hold_contacts[0] > balance_ns + 20_000_000 or
                hold_contacts[-1] < hold_end - 20_000_000):
            failures.append(f"jump_id {jid} complete frames do not cover the existing 12s hold contact boundaries")
        elif any(b - a > 20_000_000 for a, b in zip(hold_contacts, hold_contacts[1:])):
            failures.append(f"jump_id {jid} wheel support exceeds the existing 20ms hold gap limit")
    return failures
