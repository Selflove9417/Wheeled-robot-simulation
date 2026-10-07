"""Static guardrails for passive JointVelocityCmd recorder instrumentation."""

from pathlib import Path
import re
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "src" / "landing_repair_wrench_recorder.cc"
TEXT = SOURCE.read_text(encoding="utf-8")


class VelocityCommandObserverSourceTest(unittest.TestCase):
    def test_single_finite_vector_semantics_preserve_invalid_as_nan(self):
        helper = TEXT.split("inline CommandComponentObservation ObserveSingleFiniteCommand(", 1)[1]
        helper = helper.split("\n}\n", 1)[0]
        self.assertIn("component.has_value()", helper)
        self.assertIn("if (!component) return observation;", helper)
        self.assertIn("component->size() == 1 && std::isfinite(component->front())", helper)
        self.assertIn("if (observation.valid) observation.value = component->front();", helper)
        self.assertIn("vector_size{-1}", TEXT)
        self.assertIn("quiet_NaN()", TEXT)

    def test_reads_same_command_component_in_all_three_phases(self):
        self.assertEqual(TEXT.count("ComponentData<ignition::gazebo::components::JointVelocityCmd>"), 3)
        for phase in ("PreUpdate", "Update", "PostUpdate"):
            body = TEXT.split(f"void {phase}(", 1)[1]
            body = body.split("\n  }", 1)[0]
            self.assertIn("JointVelocityCmd", body)
            self.assertIn("ComponentData", body)
            self.assertNotRegex(body, r"(?:CreateComponent|RemoveComponent|SetComponentData)\s*\([^\n]*JointVelocityCmd")

    def test_appended_csv_schema_groups_are_ordered_by_phase(self):
        required = []
        for prefix in ("pre_", "before_physics_", "post_"):
            required.extend(
                prefix + "joint_velocity_cmd_" + suffix
                for suffix in ("component_present", "valid", "vector_size", "value")
            )
        positions = [TEXT.index(field) for field in required]
        self.assertEqual(positions, sorted(positions))
        self.assertGreater(TEXT.index("pre_joint_velocity_cmd_component_present"),
                           TEXT.index("post_base_world_wz"))
        self.assertEqual(len(re.findall(r"(?:pre|before_physics|post)_joint_velocity_cmd_component_present", TEXT)), 3)


if __name__ == "__main__":
    unittest.main()
