import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ground_engine_state_bridge as bridge


def engine_rows(iteration: int, sim_ns: int, omit=None, nan_joint=None, dt_ns=bridge.STEP_NS):
    omit = omit or set()
    qw, qx = math.sqrt(0.75), 0.5
    rows = []
    entities = [
        ("link", bridge.BASE_LINK),
        ("link", "flat_jump_world::bbot::link_004"),
        ("link", "flat_jump_world::bbot::link_007"),
        *(('joint', name) for name in bridge.ENGINE_JOINTS),
    ]
    for phase in ("before_step", "after_step"):
        physical = sim_ns - dt_ns if phase == "before_step" else sim_ns
        for kind, name in entities:
            if (phase, kind, name) in omit:
                continue
            row = {field: "" for field in bridge.ENGINE_REQUIRED}
            row.update({
                "sim_time_ns": str(sim_ns), "iteration": str(iteration), "dt_ns": str(dt_ns),
                "phase": phase, "entity_type": kind, "entity_name": name,
                "physical_state_time_ns": str(physical),
                "wall_steady_time_ns": str(80_000_000_000_000 + sim_ns),
                "entity_present": "1",
                "time_valid": "1", "position_valid": "1", "velocity_valid": "1",
                "position_x": "10.0", "position_y": "11.0", "position_z": "12.0",
                "quat_w": repr(qw), "quat_x": repr(qx), "quat_y": "0", "quat_z": "0",
                "linear_vx": "20.0", "linear_vy": "21.0", "linear_vz": "22.0",
                "angular_vx": "23.0", "angular_vy": "24.0", "angular_vz": "25.0",
                "joint_dof": "1", "joint_position_0": "0", "joint_velocity_0": "0",
            })
            if kind == "joint":
                index = bridge.ENGINE_JOINTS.index(name)
                row["joint_position_0"] = repr(index + 0.125 + (100 if phase == "after_step" else 0))
                row["joint_velocity_0"] = repr(index + 0.25 + (100 if phase == "after_step" else 0))
                if (phase, index) == nan_joint:
                    row["joint_position_0"] = "nan"
            rows.append(row)
    return rows


def native_rows(iteration: int, sim_ns: int, invalid_index=None, dt_ns=bridge.STEP_NS):
    rows = []
    for index, name in enumerate(bridge.NATIVE_JOINTS):
        row = {field: "" for field in bridge.NATIVE_REQUIRED}
        valid = "0" if index == invalid_index else "1"
        row.update({
            "seq": str(iteration - 1),
            "sim_time_ns": str(sim_ns), "physics_iteration": str(iteration),
            "dt_ns": str(dt_ns), "joint_index": str(index), "joint_name": name,
            "before_physics_phase": "before_physics_update",
            "before_physics_sim_time_ns": str(sim_ns),
            "before_physics_iteration": str(iteration),
            "before_physics_dt_ns": str(dt_ns),
            "before_physics_joint_force_cmd_component_present": valid,
            "before_physics_joint_force_cmd_valid": valid,
            "before_physics_joint_force_cmd_sim_input": repr(100 + index),
        })
        rows.append(row)
    return rows


def feed_step(assembler, iteration, sim_ns, *, omit=None, nan_joint=None,
              invalid_input=None, duplicate_entity=False, dt_ns=bridge.STEP_NS):
    erows = engine_rows(iteration, sim_ns, omit=omit, nan_joint=nan_joint, dt_ns=dt_ns)
    for row in erows:
        assembler.add_engine(row)
        if duplicate_entity and row["phase"] == "before_step" and row["entity_type"] == "link" and row["entity_name"] == bridge.BASE_LINK:
            assembler.add_engine(dict(row))
            duplicate_entity = False
    for row in native_rows(iteration, sim_ns, invalid_index=invalid_input, dt_ns=dt_ns):
        assembler.add_native(row)


def decode(packet):
    fields = packet.split(",")
    assert len(fields) == 41
    return {
        "magic": fields[0], "sim_ns": int(fields[1]), "iteration": int(fields[2]),
        "dt_ns": int(fields[3]), "masks": tuple(map(int, fields[4:8])),
        "q": list(map(float, fields[8:17])), "v": list(map(float, fields[17:26])),
        "before_v": list(map(float, fields[26:35])), "u": list(map(float, fields[35:41])),
    }


class GroundEngineBridgeTests(unittest.TestCase):
    @staticmethod
    def frame_with_walls(minimum, maximum=None):
        maximum = minimum if maximum is None else maximum
        return bridge.EmittedFrame(
            (7, 7_000_000, bridge.STEP_NS), "", bridge.MASK9, bridge.MASK9,
            bridge.MASK9, bridge.MASK6, (), minimum, maximum,
            {"before_step": (minimum, minimum), "after_step": (maximum, maximum)},
            {"before_step": 9, "after_step": 9})

    def first_frame(self, **kwargs):
        assembler = bridge.GroundStateAssembler()
        feed_step(assembler, 1, 1_000_000, **kwargs)
        feed_step(assembler, 2, 2_000_000)
        outputs = assembler.pop_outputs()
        self.assertTrue(outputs)
        target = next((frame for frame in outputs if frame.key[0] == 1), outputs[0])
        return assembler, target

    def test_actual_csv_schema_fixture_pairs_before_and_after_then_model_order(self):
        _, frame = self.first_frame()
        d = decode(frame.payload)
        self.assertEqual(d["magic"], "ECS1")
        self.assertEqual(d["masks"], (bridge.MASK9, bridge.MASK9, bridge.MASK9, bridge.MASK6))
        self.assertAlmostEqual(d["q"][0], 11.0)
        self.assertAlmostEqual(d["q"][1], 12.0)
        self.assertAlmostEqual(d["q"][2], 2.0 * math.atan2(0.5, math.sqrt(0.75)))
        # q/v uses state order HL,KL,HR,KR,WL,WR after the first three base DOFs.
        self.assertEqual(d["q"][3:], [100.125, 101.125, 103.125, 104.125, 102.125, 105.125])
        self.assertEqual(d["v"][3:], [100.25, 101.25, 103.25, 104.25, 102.25, 105.25])
        self.assertEqual(d["before_v"][3:], [0.25, 1.25, 3.25, 4.25, 2.25, 5.25])
        self.assertEqual(d["u"], [100.0, 101.0, 103.0, 104.0, 102.0, 105.0])
        self.assertTrue(frame.fully_valid)
        self.assertTrue(bridge.source_frame_fresh(frame, frame.source_wall_min_ns + 5_000_000))
        self.assertFalse(bridge.source_frame_fresh(frame, frame.source_wall_min_ns + 10_000_001))

    def test_one_nan_remains_nan_with_its_mask_bit_clear(self):
        _, frame = self.first_frame(nan_joint=("after_step", 2))
        d = decode(frame.payload)
        self.assertFalse(frame.fully_valid)
        self.assertEqual(d["masks"][0] & (1 << 7), 0)  # wheel L q is model dof 7
        self.assertTrue(math.isnan(d["q"][7]))
        self.assertEqual(d["masks"][1], bridge.MASK9)

    def test_invalid_wheel_link_pose_invalidates_the_paired_state(self):
        assembler = bridge.GroundStateAssembler()
        for row in engine_rows(1, 1_000_000):
            if row["phase"] == "after_step" and row["entity_name"] == "flat_jump_world::bbot::link_004":
                row["quat_w"] = "2.0"
            assembler.add_engine(row)
        for row in native_rows(1, 1_000_000):
            assembler.add_native(row)
        feed_step(assembler, 2, 2_000_000)
        frame = next(x for x in assembler.pop_outputs() if x.key[0] == 1)
        self.assertEqual(decode(frame.payload)["masks"], (0, 0, 0, 0))
        self.assertIn("after_step:wheel_link_state_mismatch:link_004", frame.reasons)

    def test_invalid_actual_joint_force_does_not_become_zero(self):
        _, frame = self.first_frame(invalid_input=2)
        d = decode(frame.payload)
        self.assertEqual(d["masks"][3] & (1 << 4), 0)  # native wheel L maps to model slot 4
        self.assertTrue(math.isnan(d["u"][4]))
        self.assertEqual(d["u"][2], 103.0)

    def test_nonfinite_native_force_is_masked_nan(self):
        assembler = bridge.GroundStateAssembler()
        e1 = engine_rows(1, 1_000_000)
        n1 = native_rows(1, 1_000_000)
        for row in e1:
            assembler.add_engine(row)
        n1[3]["before_physics_joint_force_cmd_sim_input"] = "nan"
        for row in n1:
            assembler.add_native(row)
        for row in engine_rows(2, 2_000_000):
            assembler.add_engine(row)
        for row in native_rows(2, 2_000_000):
            assembler.add_native(row)
        frame = next(x for x in assembler.pop_outputs() if x.key[0] == 1)
        d = decode(frame.payload)
        self.assertEqual(d["masks"][3] & (1 << 2), 0)  # native HR maps to MODEL slot 2
        self.assertTrue(math.isnan(d["u"][2]))

    def test_engine_wall_clock_rollback_poisoned(self):
        assembler = bridge.GroundStateAssembler()
        rows = engine_rows(1, 1_000_000)
        for row in rows[:2]:
            assembler.add_engine(row)
        rows[2]["wall_steady_time_ns"] = str(79_999_000_000_000)
        assembler.add_engine(rows[2])
        self.assertTrue(assembler.poisoned)
        self.assertIn("engine_wall_steady_rollback", assembler.faults)

    def test_missing_entity_is_emitted_invalid_not_skipped(self):
        _, frame = self.first_frame(omit={("after_step", "joint", bridge.ENGINE_JOINTS[4])})
        d = decode(frame.payload)
        self.assertEqual(d["masks"], (0, 0, 0, 0))
        self.assertTrue(all(math.isnan(x) for x in d["q"] + d["v"] + d["before_v"] + d["u"]))
        self.assertIn("engine_frame_incomplete_or_unexpected_entities", frame.reasons)

    def test_duplicate_entity_poison_and_invalid_sentinel(self):
        assembler = bridge.GroundStateAssembler()
        feed_step(assembler, 1, 1_000_000, duplicate_entity=True)
        outputs = assembler.pop_outputs()
        self.assertTrue(assembler.poisoned)
        self.assertTrue(any(f.key == (-1, -1, bridge.STEP_NS) for f in outputs))

    def test_source_clock_rollback_poisoned(self):
        assembler = bridge.GroundStateAssembler()
        for row in engine_rows(5, 5_000_000)[:1]:
            assembler.add_engine(row)
        for row in engine_rows(4, 4_000_000)[:1]:
            assembler.add_engine(row)
        self.assertTrue(assembler.poisoned)
        self.assertIn("clock_rollback_or_duplicate_frame_key", assembler.faults)
        # The parser cannot silently recover by publishing an older/clean-looking row.
        feed_step(assembler, 6, 6_000_000)
        self.assertTrue(all(not frame.fully_valid for frame in assembler.pop_outputs()))

    def test_startup_policy_skips_only_prestart_prefix_and_records_phases(self):
        start = 80_000_000_000_000
        policy = bridge.StartupHistoryPolicy(start)
        old = self.frame_with_walls(start - 3_000_000, start - 2_000_000)
        self.assertEqual(policy.classify(old, start),
                         ("skip", "engine_wall_before_bridge_start"))
        self.assertFalse(policy.prefix_closed)
        record = bridge.startup_skip_record(
            old, start, "engine_wall_before_bridge_start")
        self.assertEqual(record["before_step_rows"], "9")
        self.assertEqual(record["after_step_rows"], "9")
        self.assertEqual(record["source_wall_min_ns"], str(start - 3_000_000))
        fresh = self.frame_with_walls(start, start + 100_000)
        self.assertEqual(policy.classify(fresh, start + 1_000_000), ("fresh", ""))
        self.assertTrue(policy.prefix_closed)

    def test_mixed_prestart_frame_fails_closed_at_cutover(self):
        start = 80_000_000_000_000
        policy = bridge.StartupHistoryPolicy(start)
        crossing = self.frame_with_walls(start - 1, start + 20_000_000)
        self.assertEqual(policy.classify(crossing, start + 21_000_000),
                         ("fault", "startup_frame_crosses_bridge_start_cutover"))
        self.assertEqual(crossing.fully_valid, True)  # validity does not override cutover ambiguity
        self.assertFalse(policy.prefix_closed)

    def test_stale_frame_after_prefix_latches_fault_instead_of_skipping(self):
        start = 80_000_000_000_000
        policy = bridge.StartupHistoryPolicy(start)
        first = self.frame_with_walls(start + 1_000_000)
        self.assertEqual(policy.classify(first, start + 2_000_000), ("fresh", ""))
        later_stale = self.frame_with_walls(start + 2_000_000)
        self.assertEqual(policy.classify(later_stale, start + 20_000_000),
                         ("fault", "engine_source_wall_stale_or_future"))

    def test_latched_source_fault_cannot_be_hidden_as_startup_history(self):
        start = 80_000_000_000_000
        policy = bridge.StartupHistoryPolicy(start)
        old = self.frame_with_walls(start - 1)
        self.assertEqual(policy.classify(old, start, source_fault_latched=True),
                         ("fault", "source_integrity_fault_latched"))

    def test_non_1ms_frame_is_invalid(self):
        _, frame = self.first_frame(dt_ns=2_000_000)
        self.assertEqual(decode(frame.payload)["masks"], (0, 0, 0, 0))
        self.assertIn("dt_not_1ms", frame.reasons)

    def test_wrong_native_mode_force_phase_or_identity_is_invalid(self):
        assembler = bridge.GroundStateAssembler()
        feed_step(assembler, 1, 1_000_000)
        # Mutate the current native row's required before-state mode/force fields.
        row = native_rows(2, 2_000_000)[0]
        row["before_physics_phase"] = "after_update"
        assembler.add_native(row)
        self.assertIn("native_before_key_or_phase_mismatch:0", bridge._native_inputs(
            assembler.native.current)[2])

    def test_csv_tail_withholds_partial_line_and_rejects_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace.csv"
            path.write_bytes(b"a,b\n1,")
            tail = bridge.CsvTail(path)
            self.assertEqual(tail.read_available(), [])
            with path.open("ab") as stream:
                stream.write(b"2\n")
            rows = tail.read_available()
            self.assertEqual(rows, [{"a": "1", "b": "2"}])
            path.write_bytes(b"a,b\n")
            self.assertEqual(tail.read_available(), [])
            self.assertEqual(tail.fault, "source_file_replaced_or_truncated")


if __name__ == "__main__":
    unittest.main()
