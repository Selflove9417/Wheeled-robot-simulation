#!/usr/bin/env python3
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).parents[1] / "scripts" / "audit_ground_contacts.py"
SPEC = importlib.util.spec_from_file_location("ground_contact_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class GroundContactAuditTests(unittest.TestCase):
    def run_audit(self, trace, contacts):
        with tempfile.TemporaryDirectory() as temp_dir:
            trace_path = Path(temp_dir) / "trace.csv"
            contacts_path = Path(temp_dir) / "contacts.csv"
            for path, rows in ((trace_path, trace), (contacts_path, contacts)):
                fields = sorted({key for row in rows for key in row})
                with path.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
            return AUDIT.audit(trace_path, contacts_path)

    @staticmethod
    def trace(ids=("1",)):
        rows = []
        for index, jump_id in enumerate(ids):
            base = 20.0 * index
            rows.extend([
                {"timestamp": str(base + 1.0), "jump_id": jump_id, "state_name": "FLIGHT"},
                {"timestamp": str(base + 1.1), "jump_id": jump_id, "state_name": "TOUCHDOWN_BUFFER"},
            ])
            for tick in range(1201):
                rows.append({"timestamp": f"{base + 2.0 + tick * .01:.2f}",
                             "jump_id": jump_id, "state_name": "BALANCE"})
        return rows

    @staticmethod
    def contacts(robot_collision="bbot::link_004::link_004_collision_collision"):
        rows = []
        for tick in range(361):
            t = tick * .1
            rows.append({"sim_time": f"{t:.1f}", "side": "GROUND_STATUS", "ground_publisher_count": "1"})
        for base in (0.0, 20.0, 40.0):
            for tick in range(21):
                t = base + 1.1 + tick * .001
                rows.append({"sim_time": f"{t:.3f}", "side": "GROUND", "stamp_sec": f"{t:.3f}",
                             "num_contacts": "1", "collision_1": "ground_plane::link::collision",
                             "collision_2": robot_collision})
            for tick in range(1201):
                t = base + 2.0 + tick * .01
                rows.append({"sim_time": f"{t:.2f}", "side": "GROUND", "stamp_sec": f"{t:.2f}",
                             "num_contacts": "1", "collision_1": "ground_plane::link::collision",
                             "collision_2": robot_collision})
        return rows

    def test_native_wheel_ground_contact_and_status_can_pass(self):
        result = self.run_audit(self.trace(), self.contacts())
        self.assertTrue(result["ground_contact_pass"], result["reason"])

    def test_nonwheel_collision_is_reported_once_per_pair(self):
        rows = self.contacts("bbot::base_link::base_link_collision")
        rows.append(dict(rows[-1]))
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertEqual(result["reason"].count("bbot::base_link::base_link_collision"), 1)

    def test_jump_id_reuse_is_not_hidden_by_deduplication(self):
        result = self.run_audit(self.trace(("1", "2", "1")), self.contacts())
        self.assertIn("not strictly 1..N", result["reason"])

    def test_publisher_status_without_native_hold_contacts_fails(self):
        rows = [r for r in self.contacts() if r["side"] == "GROUND_STATUS"]
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertIn("native ground-contact coverage", result["reason"])

    def test_nonwheel_first_touchdown_collision_fails_even_with_later_wheels(self):
        rows = self.contacts()
        rows.append({"sim_time": "1.10", "side": "GROUND", "stamp_sec": "1.10",
                     "num_contacts": "1", "collision_1": "ground_plane::link::collision",
                     "collision_2": "bbot::base_link::base_link_collision"})
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertIn("first touchdown contact was not a wheel", result["reason"])

    def test_touchdown_requires_dense_native_samples_on_both_sides(self):
        rows = [r for r in self.contacts()
                if r["side"] != "GROUND" or float(r["stamp_sec"]) not in (1.101, 1.102, 1.103, 1.104,
                    1.105, 1.106, 1.107, 1.108, 1.109, 1.110, 1.111, 1.112, 1.113, 1.114,
                    1.115, 1.116, 1.117, 1.118, 1.119)]
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertIn("touchdown contact cadence is insufficient", result["reason"])

    def test_native_contacts_must_cover_entire_balance_hold(self):
        rows = [r for r in self.contacts()
                if r["side"] != "GROUND" or float(r["stamp_sec"]) <= 1.12]
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertIn("native ground-contact coverage", result["reason"])

    def test_native_hold_contact_gaps_over_20ms_fail(self):
        rows = [r for r in self.contacts()
                if r["side"] != "GROUND" or float(r["stamp_sec"]) < 2.0 or
                round(float(r["stamp_sec"])*100) % 3 == 0]
        result = self.run_audit(self.trace(), rows)
        self.assertFalse(result["ground_contact_pass"])
        self.assertIn("native ground-contact coverage", result["reason"])


if __name__ == "__main__":
    unittest.main()
