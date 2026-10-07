#!/usr/bin/env python3
"""Small synthetic-contract tests for the native-engine state auditor."""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("audit_native_engine_state.py")
SPEC = importlib.util.spec_from_file_location("audit_native_engine_state", MODULE_PATH)
AUDIT = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(AUDIT)


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fixture_rows(start_ns: int = 1_000_000, steps: int = 2):
    engine = []
    native = []
    contact = []
    geometry = []
    entity_names = [("link", AUDIT.LINKS[0]), ("link", AUDIT.LINKS[1]), ("link", AUDIT.LINKS[2])]
    entity_names += [("joint", name) for name in AUDIT.JOINTS]
    for idx in range(steps):
        ns = start_ns + idx * AUDIT.STEP_NS
        iteration = 100 + idx
        phase_states = {}
        for phase in AUDIT.PHASES:
            state_time_ns = ns - AUDIT.STEP_NS if phase == "before_step" else ns
            ti = state_time_ns / AUDIT.STEP_NS
            phase_rows = []
            for entity_index, (entity_type, name) in enumerate(entity_names):
                row = {
                    "sim_time_ns": ns, "iteration": iteration, "dt_ns": AUDIT.STEP_NS,
                    "phase": phase, "entity_type": entity_type, "entity_name": name,
                    "entity_id": 1000 + entity_index, "entity_present": 1, "time_valid": 1,
                    "position_valid": 1, "velocity_valid": 1,
                    "physical_state_time_ns": state_time_ns,
                    "wall_steady_time_ns": 10_000 + idx * 100 + (0 if phase == "before_step" else 50) + entity_index,
                    "position_x": "", "position_y": "", "position_z": "",
                    "quat_w": "", "quat_x": "", "quat_y": "", "quat_z": "",
                    "linear_vx": "", "linear_vy": "", "linear_vz": "",
                    "angular_vx": "", "angular_vy": "", "angular_vz": "",
                    "joint_dof": "", "joint_position_0": "", "joint_velocity_0": "", "reason": "",
                    "joint_force_0": 17.0 + entity_index,
                }
                if entity_type == "link":
                    if name == AUDIT.LINKS[0]:
                        x, y, z = .2 + ti * .001, .01 + ti * .002, .4
                        theta = .1 + ti * .0002
                        qw, qx = math.cos(theta / 2), math.sin(theta / 2)
                        vx, vy, vz, wx, wy, wz = .001, .002, 0.0, .2, 0.0, 0.0
                    elif name == AUDIT.LINKS[1]:
                        x, y, z = .3, .08 + ti * .001, .07
                        qw, qx = 1.0, 0.0
                        vx, vy, vz, wx, wy, wz = 0.0, .001, 0.0, 0.0, 0.0, 1.0
                    else:
                        x, y, z = -.3, .08 + ti * .001, .07
                        qw, qx = 1.0, 0.0
                        vx, vy, vz, wx, wy, wz = 0.0, .001, 0.0, 0.0, 0.0, 1.0
                    row.update({"position_x": x, "position_y": y, "position_z": z,
                                "quat_w": qw, "quat_x": qx, "quat_y": 0.0, "quat_z": 0.0,
                                "linear_vx": vx, "linear_vy": vy, "linear_vz": vz,
                                "angular_vx": wx, "angular_vy": wy, "angular_vz": wz})
                else:
                    j = AUDIT.JOINTS.index(name)
                    row.update({"joint_dof": 1, "joint_position_0": .1 * j + ti * .01,
                                "joint_velocity_0": .01})
                phase_rows.append(row)
            phase_states[phase] = {r["entity_name"]: r for r in phase_rows}
            engine.extend(phase_rows)
        native_names = ("link_002_joint", "link_003_joint", "link_004_joint",
                        "link_005_joint", "link_006_joint", "link_007_joint")
        for joint_index, short in enumerate(native_names):
            full = f"flat_jump_world::bbot::{short}"
            b = phase_states["before_step"][full]
            a = phase_states["after_step"][full]
            native.append({
                "sim_time_ns": ns, "physics_iteration": iteration, "dt_ns": AUDIT.STEP_NS,
                "joint_index": joint_index, "joint_name": short,
                "joint_valid": 1, "state_valid": 1,
                "joint_position": a["joint_position_0"], "joint_velocity": a["joint_velocity_0"],
                "before_physics_joint_state_valid": 1,
                "before_physics_joint_position": b["joint_position_0"],
                "before_physics_joint_velocity": b["joint_velocity_0"],
                "before_physics_base_pose_valid": 1,
                "before_physics_base_x": phase_states["before_step"][AUDIT.LINKS[0]]["position_x"],
                "before_physics_base_y": phase_states["before_step"][AUDIT.LINKS[0]]["position_y"],
                "before_physics_base_z": phase_states["before_step"][AUDIT.LINKS[0]]["position_z"],
                "before_physics_base_qx": phase_states["before_step"][AUDIT.LINKS[0]]["quat_x"],
                "before_physics_base_qy": 0, "before_physics_base_qz": 0,
                "before_physics_base_qw": phase_states["before_step"][AUDIT.LINKS[0]]["quat_w"],
                "before_physics_base_velocity_valid": 1,
                "before_physics_base_world_vx": .001, "before_physics_base_world_vy": .002,
                "before_physics_base_world_vz": 0, "before_physics_base_world_wx": .2,
                "before_physics_base_world_wy": 0, "before_physics_base_world_wz": 0,
                "post_base_velocity_valid": 1, "post_base_world_vx": .001,
                "post_base_world_vy": .002, "post_base_world_vz": 0,
                "post_base_world_wx": .2, "post_base_world_wy": 0, "post_base_world_wz": 0,
                "before_physics_joint_force_cmd_component_present": 1,
                "before_physics_joint_force_cmd_valid": 1,
                "before_physics_joint_force_cmd_sim_input": 9.5 + joint_index,
                "transmitted_axis_torque": .25 + joint_index, "wrench_valid": 1,
            })
        base = phase_states["after_step"][AUDIT.LINKS[0]]
        left = phase_states["after_step"][AUDIT.LINKS[1]]
        right = phase_states["after_step"][AUDIT.LINKS[2]]
        geometry.append({"sim_time_ns": ns, "physics_iteration": iteration, "dt_ns": AUDIT.STEP_NS,
                         "frame_valid": 1, "error": "", "base_x": base["position_x"],
                         "base_y": base["position_y"], "base_z": base["position_z"],
                         "base_qx": base["quat_x"], "base_qy": 0, "base_qz": 0, "base_qw": base["quat_w"],
                         "left_wheel_x": left["position_x"], "left_wheel_y": left["position_y"],
                         "left_wheel_z": left["position_z"], "right_wheel_x": right["position_x"],
                         "right_wheel_y": right["position_y"], "right_wheel_z": right["position_z"],
                         "hip_left": 0, "knee_left": 0, "hip_right": 0, "knee_right": 0})
        contact.append({"sim_time_ns": ns, "physics_iteration": iteration, "dt_ns": AUDIT.STEP_NS,
                        "frame_valid": 1, "num_contacts": 2, "collision_pairs_json": "[]", "error": ""})
    return engine, native, contact, geometry


class NativeEngineAuditTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.engine, self.native, self.contact, self.geometry = fixture_rows()

    def tearDown(self):
        self.tmp.cleanup()

    def run_fixture(self, engine=None):
        paths = {}
        for name, rows in (("engine", engine if engine is not None else self.engine),
                           ("native", self.native), ("contact", self.contact), ("geometry", self.geometry)):
            path = self.root / f"{name}.csv"
            write_rows(path, rows)
            paths[name] = path
        out = self.root / "out"
        args = type("Args", (), {"engine_csv": paths["engine"], "native_wrench_csv": paths["native"],
            "contact_frames_csv": paths["contact"], "geometry_csv": paths["geometry"],
            "hold_start_ns": 1_000_000, "hold_end_ns": 3_000_000,
            "continuity_tolerance": 1e-9, "output_dir": out})()
        summary = AUDIT.audit(args)
        return summary, out

    def test_complete_two_phase_trace_passes_and_derives_engine_states(self):
        summary, out = self.run_fixture()
        self.assertEqual(summary["gate"], "PASS")
        self.assertEqual(summary["continuity"]["position_max_abs_difference"], 0.0)
        self.assertEqual(summary["continuity"]["velocity_max_abs_difference"], 0.0)
        with (out / "derived_native_wrench.csv").open() as f:
            derived = list(csv.DictReader(f))
        self.assertEqual(len(derived), len(self.native))
        self.assertEqual(derived[0]["before_physics_joint_force_cmd_sim_input"], "9.5")
        self.assertEqual(derived[0]["transmitted_axis_torque"], "0.25")
        self.assertEqual(derived[0]["before_physics_phase"], "before_physics_update")
        with (out / "derived_geometry.csv").open() as f:
            geom = list(csv.DictReader(f))
        self.assertEqual(geom[0]["frame_valid"], "1")
        self.assertAlmostEqual(float(geom[0]["left_wheel_y"]), float(self.geometry[0]["left_wheel_y"]))

    def test_missing_phase_entity_nan_rollback_duplicate_and_scope_fail(self):
        cases = []
        missing_phase = [r for r in self.engine if not (r["sim_time_ns"] == 2_000_000 and r["phase"] == "after_step")]
        cases.append(missing_phase)
        missing_entity = [r for r in self.engine if not (r["sim_time_ns"] == 1_000_000 and r["phase"] == "before_step" and r["entity_name"] == AUDIT.LINKS[1])]
        cases.append(missing_entity)
        nan_rows = [dict(r) for r in self.engine]
        next(r for r in nan_rows if r["sim_time_ns"] == 1_000_000 and r["phase"] == "before_step" and r["entity_name"] == AUDIT.LINKS[0])["position_x"] = "nan"
        cases.append(nan_rows)
        bad_quaternion = [dict(r) for r in self.engine]
        next(r for r in bad_quaternion if r["sim_time_ns"] == 1_000_000 and r["phase"] == "before_step" and r["entity_name"] == AUDIT.LINKS[0])["quat_w"] = 1.001
        cases.append(bad_quaternion)
        # A physical-time rollback in source order, without dropping any key.
        rollback_rows = [dict(r) for r in self.engine]
        before = [r for r in rollback_rows if r["phase"] == "before_step"]
        after = [r for r in rollback_rows if r["phase"] == "after_step"]
        cases.append(after + list(reversed(before)))
        duplicate_rows = [dict(r) for r in self.engine] + [dict(self.engine[0])]
        cases.append(duplicate_rows)
        scope_rows = [dict(r) for r in self.engine]
        next(r for r in scope_rows if r["phase"] == "before_step" and r["entity_name"] == AUDIT.LINKS[0])["entity_name"] = "bbot::base_link"
        cases.append(scope_rows)
        for rows in cases:
            with self.subTest(case=len(rows)):
                summary, out = self.run_fixture(rows)
                self.assertEqual(summary["gate"], "FAIL")
                self.assertTrue((out / "derived_native_wrench.csv").exists())
                with (out / "derived_native_wrench.csv").open() as f:
                    self.assertEqual(len(list(csv.DictReader(f))), len(self.native))

    def test_missing_engine_values_remain_nan_in_derived_copy(self):
        rows = [r for r in self.engine if not (r["sim_time_ns"] == 1_000_000 and r["phase"] == "after_step" and r["entity_type"] == "joint" and r["entity_name"] == AUDIT.JOINTS[0])]
        summary, out = self.run_fixture(rows)
        self.assertEqual(summary["gate"], "FAIL")
        with (out / "derived_native_wrench.csv").open() as f:
            derived = list(csv.DictReader(f))
        self.assertEqual(derived[0]["state_valid"], "0")
        self.assertEqual(derived[0]["joint_position"], "nan")
        self.assertEqual(derived[0]["before_physics_joint_force_cmd_sim_input"], "9.5")

    def test_valid_engine_geometry_does_not_clear_original_geometry_failure(self):
        self.geometry[0]["frame_valid"] = 0
        self.geometry[0]["error"] = "original_geometry_component_missing"
        summary, out = self.run_fixture()
        # The direct engine state gate is independent; the derived response
        # geometry must nevertheless preserve the original frame failure.
        self.assertEqual(summary["gate"], "PASS")
        with (out / "derived_geometry.csv").open() as f:
            derived = list(csv.DictReader(f))
        self.assertEqual(derived[0]["frame_valid"], "0")
        self.assertEqual(derived[0]["error"], "original_geometry_component_missing")
        self.assertAlmostEqual(float(derived[0]["left_wheel_y"]), .081)

    def test_after_to_next_before_handoff_is_a_hard_gate(self):
        rows = [dict(r) for r in self.engine]
        target = next(r for r in rows if r["sim_time_ns"] == 2_000_000 and r["phase"] == "before_step" and r["entity_name"] == AUDIT.LINKS[0])
        target["linear_vy"] = .25
        summary, _ = self.run_fixture(rows)
        self.assertEqual(summary["gate"], "FAIL")
        self.assertEqual(summary["continuity"]["gate"], "FAIL")


if __name__ == "__main__":
    unittest.main()
