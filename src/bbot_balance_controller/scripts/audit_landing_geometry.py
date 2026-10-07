#!/usr/bin/env python3
"""Audit contact-synchronous wheel clearance, landing geometry and drift."""
import csv
import math
from pathlib import Path

from audit_ground_contact_frames import (
    STEP_NS, WHEELS, parse_frame, read_frame_rows, validate_active_trace,
)

GEOMETRY_FIELDS = {
    "seq", "sim_time_ns", "physics_iteration", "dt_ns", "frame_valid", "error",
    "base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw",
    "left_wheel_x", "left_wheel_y", "left_wheel_z",
    "right_wheel_x", "right_wheel_y", "right_wheel_z",
    "hip_left", "knee_left", "hip_right", "knee_right",
}
BODY_MASS = 9.5
TOTAL_MASS = BODY_MASS + 8.0
WHEEL_RADIUS = 0.07
FIELDS = (
    "jump_id", "first_contact_sim_time", "contact_pitch_deg", "contact_pitch_rate",
    "peak_bilateral_wheel_clearance_m", "buffer_compression_m",
    "max_com_backward_drift_m", "max_axle_backward_drift_m",
)


def _finite(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"nonfinite geometry {key}")
    return value


def _rotation(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 0.99 or norm > 1.01:
        raise ValueError("invalid base quaternion norm")
    x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
    return (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )


def _rotate(r, v):
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def _cad_com(q):
    """Same seven-body centroid model used by centroidal_state.hpp."""
    def rx(a, v):
        c, s = math.cos(a), math.sin(a)
        return v[0], c*v[1]-s*v[2], s*v[1]+c*v[2]
    out = [BODY_MASS*.20001846, BODY_MASS*.13261282, BODY_MASS*.05396677]
    for side in range(2):
        h, k = 2*side, 2*side+1
        hip = (.3032 if side == 0 else .0965, .125, -.07)
        thigh = rx(q[h], (.06107357 if side == 0 else -.06077357,
                          -.13690699, -.02116697))
        knee = rx(q[h], (.072 if side == 0 else -.0667,
                         -.29348091, -.06220095))
        shank = rx(q[h]+q[k], (-.01104398 if side == 0 else .00604399,
                               .11538205, -.08532288))
        wheel = rx(q[h]+q[k], (-.022 if side == 0 else -.0405,
                               .28210870, -.19553796))
        wheel_com = tuple(hip[i]+knee[i]+wheel[i]+
                          ((.03825001 if side == 0 else .01925) if i == 0 else 0.0)
                          for i in range(3))
        parts = (
            (1.2, tuple(hip[i]+thigh[i] for i in range(3))),
            (0.8, tuple(hip[i]+knee[i]+shank[i] for i in range(3))),
            (2.0, wheel_com),
        )
        for mass, point in parts:
            for i in range(3):
                out[i] += mass*point[i]
    return tuple(value/TOTAL_MASS for value in out)


def parse_geometry_rows(path):
    rows = []
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not GEOMETRY_FIELDS.issubset(reader.fieldnames):
            raise ValueError("synchronous geometry sidecar header is incomplete")
        previous = None
        for row in reader:
            seq = int(row["seq"])
            stamp = int(row["sim_time_ns"])
            iteration = int(row["physics_iteration"])
            dt = int(row["dt_ns"])
            if seq < 0 or stamp < 0 or iteration < 0 or dt != STEP_NS:
                raise ValueError("invalid geometry frame index/time")
            if previous is not None and (seq != previous[0]+1 or
                    iteration != previous[1]+1 or stamp-previous[2] != STEP_NS):
                raise ValueError("geometry frame sequence has a gap, duplicate or rollback")
            previous = (seq, iteration, stamp)
            if row["frame_valid"] != "1" or row.get("error", ""):
                rows.append({"seq": seq, "stamp": stamp, "iteration": iteration,
                             "valid": False, "raw": row})
                continue
            values = {key: _finite(row, key) for key in GEOMETRY_FIELDS
                      if key not in {"seq", "sim_time_ns", "physics_iteration", "dt_ns",
                                     "frame_valid", "error"}}
            rotation = _rotation(*(values[key] for key in
                                   ("base_qx", "base_qy", "base_qz", "base_qw")))
            q = [values[k] for k in ("hip_left", "knee_left", "hip_right", "knee_right")]
            cad = _cad_com(q)
            world_rel_com = _rotate(rotation, cad)
            base = tuple(values[k] for k in ("base_x", "base_y", "base_z"))
            com = tuple(base[i]+world_rel_com[i] for i in range(3))
            left = tuple(values[k] for k in ("left_wheel_x", "left_wheel_y", "left_wheel_z"))
            right = tuple(values[k] for k in ("right_wheel_x", "right_wheel_y", "right_wheel_z"))
            roll = math.atan2(2*(values["base_qw"]*values["base_qx"]+
                                 values["base_qy"]*values["base_qz"]),
                              1-2*(values["base_qx"]**2+values["base_qy"]**2))
            rows.append({"seq": seq, "stamp": stamp, "iteration": iteration,
                         "valid": True, "base": base, "left": left, "right": right,
                         "com": com, "pitch": -roll, "q": q})
    return rows


def audit_landing_geometry(log_path, contact_path, geometry_path):
    """Return per-jump geometry metrics plus hard failures; no pose interpolation."""
    with Path(log_path).open(newline="") as stream:
        trace = list(csv.DictReader(stream))
    ok, reason, parsed_contacts = validate_active_trace(log_path, contact_path)
    if not ok:
        return [], [f"contact source invalid: {reason}"]
    try:
        geometry = parse_geometry_rows(geometry_path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return [], [f"geometry source invalid: {exc}"]
    try:
        contact_rows = read_frame_rows(contact_path)
        contacts_by_stamp = {f[0]: f for f in (parse_frame(r) for r in contact_rows)}
    except (OSError, ValueError) as exc:
        return [], [f"contact source invalid: {exc}"]
    geometry_by_stamp = {r["stamp"]: r for r in geometry}
    failures = []
    active_trace = [r for r in trace if r.get("jump_id") not in ("", "0", None)]
    segments = []
    for row in active_trace:
        jid = row["jump_id"]
        if not segments or segments[-1][0] != jid:
            segments.append((jid, [row]))
        else:
            segments[-1][1].append(row)
    results = []
    for jid, hop in segments:
        try:
            entry = next(r for r in hop if r.get("state_name") == "FLIGHT")
            entry_ns = int(round(float(entry["timestamp"])*1e9))
            start_ns = int(round(float(hop[0]["timestamp"])*1e9))
            end_ns = int(round(float(hop[-1]["timestamp"])*1e9))
            axis_row = next(r for r in hop if r.get("jump_forward_axis_valid") == "1")
            axis = (_finite(axis_row, "jump_forward_axis_x"),
                    _finite(axis_row, "jump_forward_axis_y"))
            axis_norm = math.hypot(*axis)
            if abs(axis_norm-1.0) > 0.02:
                raise ValueError("jump forward axis is invalid")
            axis = axis[0]/axis_norm, axis[1]/axis_norm
            active_geom = [geometry_by_stamp[t] for t in sorted(geometry_by_stamp)
                           if start_ns <= t <= end_ns]
            if not active_geom or active_geom[0]["stamp"] > start_ns or active_geom[-1]["stamp"] < end_ns:
                raise ValueError("geometry does not cover the complete jump through final hold")
            if any(not f["valid"] for f in active_geom):
                raise ValueError("invalid geometry component during jump or hold")
            active_contacts = [contacts_by_stamp.get(f["stamp"]) for f in active_geom]
            if any(c is None for c in active_contacts):
                raise ValueError("geometry and contact frame timestamps do not align")
            if any(g["seq"] != c[1] or g["iteration"] != c[2]
                   for g, c in zip(active_geom, active_contacts)):
                raise ValueError("geometry/contact seq or physics iteration mismatch")

            previous_pitch = None
            previous_q = None
            for g in active_geom:
                g["pitch_rate"] = (0.0 if previous_pitch is None else
                    (g["pitch"]-previous_pitch)/(STEP_NS*1e-9))
                previous_pitch = g["pitch"]
                g["axle"] = tuple((g["left"][i]+g["right"][i])*0.5 for i in range(3))
                if any(abs(value) > 1.57 for value in g["q"]):
                    raise ValueError("actual joint position exceeded the ±1.57rad hard limit")
                if previous_q is not None and any(
                        abs(value-old)/(STEP_NS*1e-9) > 30.0
                        for value, old in zip(g["q"], previous_q)):
                    raise ValueError("actual joint speed exceeded the 30rad/s hard limit")
                previous_q = g["q"]

            flight_frames = [(g, c) for g, c in zip(active_geom, active_contacts)
                             if g["stamp"] >= entry_ns]
            prior_contacts = [(g, c) for g, c in zip(active_geom, active_contacts)
                              if g["stamp"] < entry_ns and c[4]]
            if not prior_contacts:
                raise ValueError("no contact support before FLIGHT")
            first_landing = next(((g, c) for g, c in flight_frames if c[4]), None)
            if first_landing is None:
                raise ValueError("no landing contact after FLIGHT")
            landing_g, landing_c = first_landing
            previous_g = next((g for g in reversed(active_geom)
                               if g["stamp"] == landing_g["stamp"]-STEP_NS), None)
            if previous_g is None:
                raise ValueError("missing synchronized pose before first contact")
            contact_pitch_deg = math.degrees(landing_g["pitch"])
            contact_pitch_rate = landing_g["pitch_rate"]

            takeoff_stamp = max(c[0] for _, c in prior_contacts)
            airborne = [(g, c) for g, c in zip(active_geom, active_contacts)
                        if takeoff_stamp < g["stamp"] < landing_g["stamp"]]
            if not airborne:
                raise ValueError("no complete airborne geometry frames")
            peak_bilateral_clearance = max(
                min(g["left"][2]-WHEEL_RADIUS, g["right"][2]-WHEEL_RADIUS)
                for g, _ in airborne)

            initial_com = landing_g["com"]
            initial_axle = landing_g["axle"]
            initial_relative_com_z = initial_com[2]-initial_axle[2]
            compress_until = landing_g["stamp"]+800_000_000
            compression_rows = [g for g in active_geom
                                if landing_g["stamp"] <= g["stamp"] <= compress_until]
            min_relative_com_z = min(g["com"][2]-g["axle"][2] for g in compression_rows)
            compression = initial_relative_com_z-min_relative_com_z

            com_frontier = 0.0
            axle_frontier = 0.0
            com_back = 0.0
            axle_back = 0.0
            for g in active_geom:
                if g["stamp"] < landing_g["stamp"]:
                    continue
                dx_com = (g["com"][0]-initial_com[0])*axis[0] + (g["com"][1]-initial_com[1])*axis[1]
                dx_axle = (g["axle"][0]-initial_axle[0])*axis[0] + (g["axle"][1]-initial_axle[1])*axis[1]
                com_frontier = max(com_frontier, dx_com)
                axle_frontier = max(axle_frontier, dx_axle)
                com_back = max(com_back, com_frontier-dx_com)
                axle_back = max(axle_back, axle_frontier-dx_axle)

            result = {
                "jump_id": jid,
                "first_contact_sim_time": f"{landing_g['stamp']*1e-9:.9f}",
                "contact_pitch_deg": f"{contact_pitch_deg:.6f}",
                "contact_pitch_rate": f"{contact_pitch_rate:.6f}",
                "peak_bilateral_wheel_clearance_m": f"{peak_bilateral_clearance:.6f}",
                "buffer_compression_m": f"{compression:.6f}",
                "max_com_backward_drift_m": f"{com_back:.6f}",
                "max_axle_backward_drift_m": f"{axle_back:.6f}",
            }
            results.append(result)
            if peak_bilateral_clearance < 0.20:
                failures.append(f"jump_id {jid}: both-wheel clearance below 0.20m")
            if not 0.0 <= contact_pitch_deg <= 5.0:
                failures.append(f"jump_id {jid}: first wheel-contact pitch outside 0..5deg")
            if abs(contact_pitch_rate) > 0.20:
                failures.append(f"jump_id {jid}: first wheel-contact pitch rate exceeds 0.20rad/s")
            if compression < 0.08:
                failures.append(f"jump_id {jid}: measured COM-relative-to-axle buffer below 0.08m")
            if com_back > 0.010:
                failures.append(f"jump_id {jid}: COM backward drift exceeds 0.01m")
            if axle_back > 0.010:
                failures.append(f"jump_id {jid}: axle midpoint backward drift exceeds 0.01m")
        except (StopIteration, KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
            failures.append(f"jump_id {jid}: geometry audit error: {exc}")
    return results, failures
