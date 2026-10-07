#!/usr/bin/env python3
"""Synthetic CSV and fake-bridge tests for the offline model response audit."""
import argparse
import csv
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'audit_ground_model_response.py'
spec = importlib.util.spec_from_file_location('ground_model_response_audit', SCRIPT)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)

PAIR = json.dumps([
    ['flat_jump_world::ground_plane::link::collision', 'flat_jump_world::bbot::link_004::link_004_collision_collision'],
    ['flat_jump_world::ground_plane::link::collision', 'flat_jump_world::bbot::link_007::link_007_collision_collision']])


class Fixture:
    def __init__(self, root, count=120):
        self.root = Path(root)
        self.native = self.root / 'native.csv'
        self.contacts = self.root / 'contacts.csv'
        self.geometry = self.root / 'geometry.csv'
        self.gate = self.root / 'response_gate.json'
        self.bridge = self.root / 'fake_bridge.py'
        self.output = self.root / 'out'
        self.start = 1_000_000_000
        self.end = self.start + count * audit.STEP_NS
        self.count = count
        self.gate.write_text(json.dumps({'minimum_thrust_steps':30,
            'thrust_acceleration_RMS_limits':[0.5,0.5,2.0,5,5,5,5,5,5],
            'thrust_acceleration_relative_RMS_max':0.2,
            'same_step_velocity_continuity_max':1e-9}))
        self.write_bridge(delta_after_ns=None)
        self.make_data()

    def write_bridge(self, delta_after_ns=None):
        change = f"if t >= {delta_after_ns * 1e-9}: pred[0] += 3.0" if delta_after_ns is not None else 'pass'
        self.bridge.write_text('''#!/usr/bin/env python3
import sys
for line in sys.stdin:
    x = [float(v) for v in line.split()]
    t, bilateral = x[0], int(x[1])
    observed = x[20:29]
    pred = list(observed)
    CHANGE
    out = [t, bilateral, 0.0] + pred + observed + [0.0]*4 + [0.0]*4 + [0.0]*6
    print(' '.join(str(v) for v in out))
'''.replace('CHANGE', change))
        self.bridge.chmod(0o755)

    def make_data(self):
        native, contacts, geometry = [], [], []
        # One context frame before and after the fixed window makes every
        # interior candidate's immediate contact neighbors observable.
        for k in range(-1, self.count + 1):
            ns = self.start + k * audit.STEP_NS
            iteration = 5000 + k
            for idx in range(6):
                native.append(self.native_row(iteration, ns, idx))
            contacts.append({'seq':str(k+1),'physics_iteration':str(iteration),'sim_time_ns':str(ns),
                'dt_ns':str(audit.STEP_NS),'frame_valid':'1','num_contacts':'2',
                'collision_pairs_json':PAIR,'error':''})
            geometry.append({'seq':str(k+1),'physics_iteration':str(iteration),'sim_time_ns':str(ns),
                'dt_ns':str(audit.STEP_NS),'frame_valid':'1','error':'',
                'base_x':'0','base_y':'0','base_z':'0.4','base_qx':'0','base_qw':'1',
                'left_wheel_x':'0','right_wheel_x':'0'})
        self.write_rows(self.native, list(native[0]), native)
        self.write_rows(self.contacts, list(contacts[0]), contacts)
        self.write_rows(self.geometry, list(geometry[0]), geometry)

    @staticmethod
    def native_row(iteration, ns, idx):
        # Unique positions/commands make vector-order assertions possible.
        q = (idx + 1) * 0.01
        u = (idx + 1) * 0.1
        return {'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),
            'dt_ns':str(audit.STEP_NS),'joint_index':str(idx),'joint_name':audit.ACTUATOR_NAMES[idx],
            'state_valid':'1','wrench_valid':'1','joint_position':str(q),'joint_velocity':'0',
            'before_physics_phase':'before_physics_update','before_physics_iteration':str(iteration),
            'before_physics_sim_time_ns':str(ns),'before_physics_dt_ns':str(audit.STEP_NS),
            'before_physics_joint_state_valid':'1','before_physics_joint_position':str(q),
            'before_physics_joint_velocity':'0','before_physics_base_pose_valid':'1',
            'before_physics_base_y':'0','before_physics_base_z':'0.4','before_physics_base_qx':'0',
            'before_physics_base_qw':'1','before_physics_base_velocity_valid':'1',
            'before_physics_base_world_vy':'0','before_physics_base_world_vz':'0',
            'before_physics_base_world_wx':'0','post_base_velocity_valid':'1',
            'post_base_world_vy':'0','post_base_world_vz':'0','post_base_world_wx':'0',
            'before_physics_joint_force_cmd_component_present':'1','before_physics_joint_force_cmd_valid':'1',
            'before_physics_joint_force_cmd_sim_input':str(u),
            'transmitted_axis_torque':str(u * 0.5)}

    @staticmethod
    def write_rows(path, fields, rows):
        with path.open('w', newline='') as stream:
            w = csv.DictWriter(stream, fieldnames=fields)
            w.writeheader(); w.writerows(rows)

    def run(self):
        return audit.run_audit(argparse.Namespace(native_wrench_csv=self.native,
            contact_frames_csv=self.contacts, geometry_csv=self.geometry,
            hold_start_ns=self.start, hold_end_ns=self.end, cpp_bridge=self.bridge,
            response_gate_json=self.gate, output_dir=self.output))

    def edit_rows(self, path):
        return audit.read_rows(path)


class GroundModelResponseTests(unittest.TestCase):
    def fixture(self, count=120):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        return Fixture(tmp.name, count=count)

    def test_complete_window_passes_both_separate_split_gates_and_saves_steps(self):
        f = self.fixture()
        report = f.run()
        self.assertEqual(report['gate'], 'PASS', report['failures'])
        split = report['split_gates']
        self.assertEqual(split['first_half_diagnostic_record']['steps'], 60)
        self.assertEqual(split['second_half_independent_validation']['steps'], 60)
        self.assertEqual(split['first_half_diagnostic_record']['gate'], 'PASS')
        self.assertEqual(split['second_half_independent_validation']['gate'], 'PASS')
        rows = audit.read_rows(f.output / 'response_steps.csv')
        self.assertEqual(len(rows), 120)
        self.assertTrue((f.output / 'window_frames.csv').is_file())
        self.assertEqual(len((f.output / 'bridge_input.txt').read_text().splitlines()), 120)

    def test_bridge_protocol_keeps_q_and_command_vector_order(self):
        f = self.fixture(count=60)
        f.run()
        vals = [float(x) for x in (f.output / 'bridge_input.txt').read_text().splitlines()[0].split()]
        self.assertEqual(len(vals), 41)
        # q indices after (t,bilateral): baseY, baseZ, rollX, HL, KL, HR, KR, wheelL, wheelR
        self.assertEqual(vals[5:11], [0.01, 0.02, 0.04, 0.05, 0.03, 0.06])
        # six before-Physics commands follow the same native index order.
        for actual, expected in zip(vals[29:35], [0.1, 0.2, 0.4, 0.5, 0.3, 0.6]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(vals[35:41], [0.05, 0.1, 0.2, 0.25, 0.15, 0.3]):
            self.assertAlmostEqual(actual, expected)

    def test_independent_validation_failure_is_not_pooled_with_first_half(self):
        f = self.fixture()
        f.write_bridge(delta_after_ns=f.start + 60 * audit.STEP_NS)
        report = f.run()
        self.assertEqual(report['split_gates']['first_half_diagnostic_record']['gate'], 'PASS')
        self.assertEqual(report['split_gates']['second_half_independent_validation']['gate'], 'FAIL')
        self.assertIn('validation:base_forward_absolute_or_relative_RMS_failed', report['failures'])

    def test_rejected_prediction_is_retained_and_later_frames_are_still_checked(self):
        f = self.fixture()
        target_s = (f.start + 10 * audit.STEP_NS) * 1e-9
        source = f.bridge.read_text().replace(
            "out = [t, bilateral, 0.0] + pred",
            f"if abs(t - {target_s!r}) < 1e-12: pred = [float('nan')]*9\n"
            "    out = [t, bilateral, 0.0] + pred")
        f.bridge.write_text(source)
        report = f.run()
        self.assertEqual(report['gate'], 'FAIL')
        self.assertEqual(report['diagnostics']['invalid_prediction_steps'], 1)
        self.assertEqual(report['diagnostics']['all_supported_step_rows_saved'], 120)
        self.assertEqual(report['split_gates']['second_half_independent_validation']['gate'], 'PASS')
        rows = audit.read_rows(f.output / 'response_steps.csv')
        self.assertEqual(rows[10]['prediction_valid'], '0')
        self.assertEqual(rows[10]['hip_left_qdd_predicted'], '')
        self.assertEqual(rows[-1]['prediction_valid'], '1')

    def test_missing_native_input_fails_original_window_integrity_but_keeps_full_report(self):
        f = self.fixture()
        rows = f.edit_rows(f.native)
        rows = [r for r in rows if not (int(r['sim_time_ns']) == f.start + 20*audit.STEP_NS and r['joint_index'] == '5')]
        Fixture.write_rows(f.native, list(rows[0]), rows)
        report = f.run()
        self.assertEqual(report['gate'], 'FAIL')
        self.assertIn('original_window_frame_integrity_failed', report['failures'])
        self.assertTrue((f.output / 'summary.json').is_file())
        saved = audit.read_rows(f.output / 'window_frames.csv')
        bad = [r for r in saved if r['sim_time_ns'] == str(f.start + 20*audit.STEP_NS)]
        self.assertEqual(len(bad), 1)
        self.assertEqual(bad[0]['frame_integrity_ok'], '0')

    def test_missing_net_diagnostic_is_reported_without_zero_substitution_or_crash(self):
        f = self.fixture()
        rows = f.edit_rows(f.native)
        target = f.start + 25*audit.STEP_NS
        for row in rows:
            if row['sim_time_ns'] == str(target) and row['joint_index'] == '4':
                row['wrench_valid'] = '0'
                row['transmitted_axis_torque'] = 'nan'
        Fixture.write_rows(f.native, list(rows[0]), rows)
        report = f.run()
        self.assertEqual(report['gate'], 'FAIL')
        self.assertTrue(any('invalid_or_nonfinite_transmitted_wrench_diagnostic_index_4' in x
                            for x in report['frame_integrity_failures']))
        window_rows = audit.read_rows(f.output / 'window_frames.csv')
        bad = next(row for row in window_rows if row['sim_time_ns'] == str(target))
        self.assertEqual(bad['knee_right_transmitted_wrench_raw'], 'nan')
        bridge_rows = (f.output / 'bridge_input.txt').read_text().splitlines()
        self.assertEqual(len(bridge_rows), 119)

    def test_contact_transition_is_reported_and_transition_neighbors_are_excluded(self):
        f = self.fixture()
        rows = f.edit_rows(f.contacts)
        target = f.start + 60*audit.STEP_NS
        for row in rows:
            if int(row['sim_time_ns']) == target:
                row['num_contacts'] = '0'; row['collision_pairs_json'] = '[]'
        Fixture.write_rows(f.contacts, list(rows[0]), rows)
        report = f.run()
        self.assertEqual(report['contact']['transition_count'], 2)
        rows = audit.read_rows(f.output / 'window_frames.csv')
        for ns in (target-audit.STEP_NS, target, target+audit.STEP_NS):
            row = next(r for r in rows if int(r['sim_time_ns']) == ns)
            self.assertEqual(row['support_eligible'], '0')
        self.assertEqual(report['contact']['supported_steps_after_neighbor_contact_rule'], 117)

    def test_wrong_clock_pair_and_clock_rollback_fail_and_remain_saved(self):
        f = self.fixture()
        rows = f.edit_rows(f.geometry)
        rows[10]['physics_iteration'] = '777777'
        rows[11]['sim_time_ns'] = str(int(rows[11]['sim_time_ns']) - 2*audit.STEP_NS)
        Fixture.write_rows(f.geometry, list(rows[0]), rows)
        report = f.run()
        self.assertEqual(report['gate'], 'FAIL')
        self.assertIn('original_window_frame_integrity_failed', report['failures'])
        self.assertIn('source_clock_reversal_or_step_error', report['failures'])
        self.assertTrue((f.output / 'window_frames.csv').exists())

    def test_nan_is_not_replaced_and_never_reaches_bridge_as_zero_effort(self):
        f = self.fixture()
        rows = f.edit_rows(f.native)
        target = f.start + 10*audit.STEP_NS
        for row in rows:
            if row['sim_time_ns'] == str(target) and row['joint_index'] == '0':
                row['before_physics_joint_force_cmd_sim_input'] = 'nan'
        Fixture.write_rows(f.native, list(rows[0]), rows)
        report = f.run()
        self.assertEqual(report['gate'], 'FAIL')
        self.assertIn('original_window_frame_integrity_failed', report['failures'])
        # The bad frame is not silently serialized as an all-zero command vector.
        bridge_rows = (f.output / 'bridge_input.txt').read_text().splitlines()
        self.assertTrue(all(abs(float(line.split()[29]) - 0.1) < 1e-12 for line in bridge_rows))


if __name__ == '__main__':
    unittest.main()
