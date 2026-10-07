#!/usr/bin/env python3
import csv
import math
from pathlib import Path
import sys
import tempfile

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from audit_landing_geometry import audit_landing_geometry  # noqa: E402

GROUND = "flat_jump_world::ground_plane::link::collision"
LEFT = "flat_jump_world::bbot::link_004::link_004_collision_collision"
RIGHT = "flat_jump_world::bbot::link_007::link_007_collision_collision"
CONTACT_FIELDS = ("seq", "sim_time_ns", "physics_iteration", "dt_ns", "frame_valid",
                  "num_contacts", "collision_pairs_json", "error")
GEOMETRY_FIELDS = ("seq", "sim_time_ns", "physics_iteration", "dt_ns", "frame_valid", "error",
                   "base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw",
                   "left_wheel_x", "left_wheel_y", "left_wheel_z",
                   "right_wheel_x", "right_wheel_y", "right_wheel_z",
                   "hip_left", "knee_left", "hip_right", "knee_right")


def fixture(tmp, *, missing_seq=None, bad_clearance=False, backward=False,
            retreat_after_advance=False, invalid=False):
    log = tmp / "log.csv"
    contacts = tmp / "contacts.csv"
    geometry = tmp / "geometry.csv"
    with log.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("timestamp", "jump_id", "state_name",
            "jump_forward_axis_valid", "jump_forward_axis_x", "jump_forward_axis_y"))
        writer.writeheader()
        writer.writerow(dict(timestamp="0.010", jump_id="1", state_name="PRE_JUMP",
                             jump_forward_axis_valid="1", jump_forward_axis_x="1", jump_forward_axis_y="0"))
        writer.writerow(dict(timestamp="1.020", jump_id="1", state_name="FLIGHT",
                             jump_forward_axis_valid="1", jump_forward_axis_x="1", jump_forward_axis_y="0"))
        writer.writerow(dict(timestamp="12.020", jump_id="1", state_name="BALANCE",
                             jump_forward_axis_valid="1", jump_forward_axis_x="1", jump_forward_axis_y="0"))
    with contacts.open("w", newline="") as stream, geometry.open("w", newline="") as geom_stream:
        cw = csv.DictWriter(stream, fieldnames=CONTACT_FIELDS)
        gw = csv.DictWriter(geom_stream, fieldnames=GEOMETRY_FIELDS)
        cw.writeheader(); gw.writeheader()
        for seq in range(12021):
            if seq == missing_seq:
                continue
            t = seq / 1000.0
            stamp = seq * 1_000_000
            in_air = 1.000 < t < 1.300
            contact = not in_air
            pairs = ([[GROUND, LEFT], [GROUND, RIGHT]] if contact else [])
            cw.writerow(dict(seq=seq, sim_time_ns=stamp, physics_iteration=seq+1,
                             dt_ns=1_000_000, frame_valid=1, num_contacts=len(pairs),
                             collision_pairs_json=__import__("json").dumps(pairs), error=""))
            if seq < 1000:
                clearance = 0.0
            elif seq <= 1300:
                u = (seq-1000)/300.0
                clearance = (.15 if bad_clearance else .25)*math.sin(math.pi*u)
            else:
                clearance = 0.0
            touch_pitch = .03
            pitch = touch_pitch
            if seq < 1300:
                pitch = touch_pitch*(seq/1300.0)
            roll = -pitch
            qx, qw = math.sin(roll/2), math.cos(roll/2)
            base_x = (-.02 if backward and seq >= 1400 else 0.0)
            if retreat_after_advance:
                if 1400 <= seq < 1800:
                    base_x = .05
                elif seq >= 1800:
                    base_x = .03
            base_z = .60
            if seq >= 1300:
                base_z -= .10*min(1.0, (seq-1300)/400.0)
            if seq == 1300:
                base_z = .60
            valid = not (invalid and seq == 1400)
            gw.writerow(dict(seq=seq, sim_time_ns=stamp, physics_iteration=seq+1,
                dt_ns=1_000_000, frame_valid=1 if valid else 0,
                error="" if valid else "geometry_component_missing_or_nonfinite",
                base_x=base_x, base_y=0, base_z=base_z, base_qx=qx,
                base_qy=0, base_qz=0, base_qw=qw,
                left_wheel_x=base_x, left_wheel_y=.125, left_wheel_z=.07+clearance,
                right_wheel_x=base_x, right_wheel_y=-.125, right_wheel_z=.07+clearance,
                hip_left=0, knee_left=0, hip_right=0, knee_right=0))
    return log, contacts, geometry


def main():
    with tempfile.TemporaryDirectory() as td:
        log, contacts, geometry = fixture(Path(td))
        rows, failures = audit_landing_geometry(log, contacts, geometry)
        assert len(rows) == 1 and not failures, failures
        assert float(rows[0]["peak_bilateral_wheel_clearance_m"]) >= .20
        assert .08 <= float(rows[0]["buffer_compression_m"]) <= .11
        assert 0 <= float(rows[0]["contact_pitch_deg"]) <= 5
        assert abs(float(rows[0]["contact_pitch_rate"])) <= .20
    for kwargs, expected in (
        ({"missing_seq": 900}, "contact source invalid"),
        ({"bad_clearance": True}, "both-wheel clearance"),
        ({"backward": True}, "backward drift"),
        ({"retreat_after_advance": True}, "backward drift"),
        ({"invalid": True}, "invalid geometry component"),
    ):
        with tempfile.TemporaryDirectory() as td:
            log, contacts, geometry = fixture(Path(td), **kwargs)
            _, failures = audit_landing_geometry(log, contacts, geometry)
            assert any(expected in item for item in failures), (expected, failures)
    print("PASS: synchronous landing clearance, contact posture, compression, drift and fail-closed gaps")


if __name__ == "__main__":
    main()
