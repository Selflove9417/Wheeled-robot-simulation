import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).parents[1] / "scripts" / "audit_ground_contact_frames.py"
SPEC = importlib.util.spec_from_file_location("ground_frames", SCRIPT)
gf = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gf)
sys.path.insert(0, str(SCRIPT.parent))

LEFT = "flat_jump_world::bbot::link_004::link_004_collision_collision"
RIGHT = "flat_jump_world::bbot::link_007::link_007_collision_collision"
GROUND = "flat_jump_world::ground_plane::link::collision"


def row(seq, stamp, pairs=(), valid=1, error=""):
    return {"seq": str(seq), "sim_time_ns": str(stamp),
            "physics_iteration": str(seq + 10), "dt_ns": str(gf.STEP_NS),
            "frame_valid": str(valid), "num_contacts": str(len(pairs)),
            "collision_pairs_json": json.dumps(pairs), "error": error}


class GroundContactFrameTests(unittest.TestCase):
    def test_zero_is_real_empty_and_not_ready(self):
        parsed = gf.parse_frame(row(0, 1_000_000))
        self.assertEqual(parsed[4], set())
        self.assertFalse(gf.ready_from_rows([row(0, 1_000_000), row(1, 2_000_000)]))

    def test_ready_requires_two_consecutive_bilateral_pairs(self):
        pairs = [[GROUND, LEFT], [GROUND, RIGHT]]
        self.assertTrue(gf.ready_from_rows([row(4, 5_000_000, pairs),
                                            row(5, 6_000_000, pairs)]))
        self.assertFalse(gf.ready_from_rows([row(4, 5_000_000, pairs),
                                             row(6, 7_000_000, pairs)]))

    def test_rejects_count_scope_and_unknown_collision(self):
        with self.assertRaises(ValueError):
            gf.parse_frame(row(0, 1_000_000, [[GROUND, LEFT]]) | {"num_contacts": "0"})
        with self.assertRaises(ValueError):
            gf.parse_frame(row(0, 1_000_000, [["other_world::ground_plane::link::collision", LEFT]]))
        unknown = "flat_jump_world::bbot::torso::collision"
        with self.assertRaises(ValueError):
            gf.parse_frame(row(0, 1_000_000, [[GROUND, unknown]]))

    def write_inputs(self, directory, frames):
        log = directory / "trace.csv"
        with log.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["timestamp", "jump_id"])
            writer.writeheader()
            writer.writerows([{"timestamp": "0.002", "jump_id": "1"},
                              {"timestamp": "0.004", "jump_id": "1"}])
        path = directory / "frames.csv"
        fields = list(frames[0])
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(frames)
        return log, path

    def test_startup_invalid_allowed_but_active_invalid_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            frames = [row(0, 1_000_000, valid=0, error="component_pending"),
                      row(1, 2_000_000), row(2, 3_000_000), row(3, 4_000_000)]
            log, path = self.write_inputs(d, frames)
            ok, _, _ = gf.validate_active_trace(log, path)
            self.assertTrue(ok)
            frames[1] = row(1, 2_000_000, valid=0, error="missing_data")
            log, path = self.write_inputs(d, frames)
            ok, reason, _ = gf.validate_active_trace(log, path)
            self.assertFalse(ok)
            self.assertIn("invalid frame", reason)

    def test_dropped_step_fails_and_left_right_one_ms_gap_is_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            pairs_l = [[GROUND, LEFT]]
            pairs_r = [[GROUND, RIGHT]]
            bilateral = pairs_l + pairs_r
            seq_rows = [row(0, 1_000_000, bilateral),
                        row(1, 2_000_000, pairs_l),
                        row(2, 3_000_000), row(3, 4_000_000),
                        row(4, 5_000_000, pairs_r)]
            log, path = self.write_inputs(d, seq_rows)
            ok, _, _ = gf.validate_active_trace(log, path)
            self.assertTrue(ok)
            parsed = [gf.parse_frame(r) for r in seq_rows]
            start, end = gf.flight_gap(parsed, 3_000_000)
            self.assertEqual(start, 0.002)
            self.assertEqual(end, 0.005)
            dropped = [seq_rows[0], seq_rows[1], seq_rows[3], seq_rows[4]]
            log, path = self.write_inputs(d, dropped)
            ok, _, _ = gf.validate_active_trace(log, path)
            self.assertFalse(ok)

    def test_duplicate_or_rollback_sequence_fails_in_source_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            log = d / "trace.csv"
            with log.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["timestamp", "jump_id"])
                writer.writeheader()
                writer.writerows([{"timestamp": "0.001", "jump_id": "1"},
                                  {"timestamp": "0.004", "jump_id": "1"}])
            frames = [row(0, 1_000_000), row(1, 2_000_000),
                      row(1, 3_000_000), row(3, 4_000_000)]
            path = d / "frames.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(frames[0]))
                writer.writeheader()
                writer.writerows(frames)
            ok, reason, _ = gf.validate_active_trace(log, path)
            self.assertFalse(ok)
            self.assertIn("discontinuity", reason)

    def test_invalid_full_source_cannot_yield_partial_flight_fit(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            log = d / "trial_log.csv"
            with log.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["timestamp", "jump_id", "state_name"])
                writer.writeheader()
                writer.writerows([{"timestamp": "0.001", "jump_id": "1", "state_name": "PRE_JUMP"},
                                  {"timestamp": "0.002", "jump_id": "1", "state_name": "FLIGHT"},
                                  {"timestamp": "0.004", "jump_id": "1", "state_name": "FLIGHT"}])
            contacts = d / "trial_contacts.csv"
            contacts.write_text("side,stamp_sec,num_contacts,collision_1,collision_2\n")
            frames = [row(0, 1_000_000), row(1, 2_000_000), row(3, 4_000_000)]
            frame_path = d / "trial_ground_frames.csv"
            with frame_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(frames[0]))
                writer.writeheader()
                writer.writerows(frames)
            spec = importlib.util.spec_from_file_location(
                "contact_metrics_partial", Path(__file__).parents[1] / "scripts" / "audit_contact_flight_metrics.py")
            cm = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cm)
            result = cm.audit(log, "1", frame_path)
            self.assertFalse(result["freefall_verified"])
            self.assertEqual(result["contact_source"], "complete_frames")

    def test_short_recontact_is_not_called_free_flight(self):
        pairs_l = [[GROUND, LEFT]]
        pairs_r = [[GROUND, RIGHT]]
        rows = [gf.parse_frame(row(0, 1_000_000, pairs_l + pairs_r)),
                gf.parse_frame(row(1, 2_000_000, pairs_l)),
                gf.parse_frame(row(2, 3_000_000, pairs_r)),
                gf.parse_frame(row(3, 4_000_000)),
                gf.parse_frame(row(4, 5_000_000, pairs_l))]
        start, end = gf.flight_gap(rows, 3_500_000)
        self.assertEqual(start, 0.003)
        self.assertEqual(end, 0.005)
        self.assertLess(end - start, .080)  # no fit may bridge the brief recontact

    def test_declared_but_missing_source_never_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            log = d / "trial_log.csv"
            log.write_text("state_name,jump_id,timestamp\nFLIGHT,1,1.0\n")
            required = d / "trial_ground_frames.csv.required"
            required.write_text("required\n")
            spec = importlib.util.spec_from_file_location(
                "contact_metrics", Path(__file__).parents[1] / "scripts" / "audit_contact_flight_metrics.py")
            cm = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cm)
            result = cm.audit(log, "1")
            self.assertEqual(result["contact_source"], "complete_frames_missing")

    def test_hold_allows_brief_real_zero_but_rejects_over_20ms_support_gap(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "trace.csv"
            with log.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["timestamp", "jump_id", "state_name"])
                writer.writeheader()
                writer.writerows([
                    {"timestamp": "0.000", "jump_id": "1", "state_name": "PRE_JUMP"},
                    {"timestamp": "0.100", "jump_id": "1", "state_name": "FLIGHT"},
                    {"timestamp": "0.200", "jump_id": "1", "state_name": "TOUCHDOWN_BUFFER"},
                    {"timestamp": "0.300", "jump_id": "1", "state_name": "BALANCE"},
                    {"timestamp": "12.300", "jump_id": "1", "state_name": "BALANCE"},
                ])
            contact = gf.WHEELS.copy()
            def frame(t):
                return (t, t // gf.STEP_NS, t // gf.STEP_NS, gf.STEP_NS,
                        contact if t <= 200_000_000 or t >= 300_000_000 else set(), set())
            frames = [frame(t) for t in range(0, 12_301_000_000, gf.STEP_NS)]
            # One 1ms empty frame during hold is a real, observed contact loss,
            # but the existing hold tolerance permits it.
            frames[305] = (frames[305][0], frames[305][1], frames[305][2],
                           frames[305][3], set(), set())
            self.assertEqual(gf.audit_session(log, frames), [])
            for i in range(305, 327):
                frames[i] = (frames[i][0], frames[i][1], frames[i][2],
                             frames[i][3], set(), set())
            failures = gf.audit_session(log, frames)
            self.assertTrue(any("20ms" in reason for reason in failures))


if __name__ == "__main__":
    unittest.main()
