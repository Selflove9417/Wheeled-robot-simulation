#!/usr/bin/env python3
"""Small synthetic CSV fixtures for ground input audit failure semantics."""
import argparse
import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'audit_ground_input_trial.py'
spec = importlib.util.spec_from_file_location('ground_input_audit', SCRIPT)
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


class Fixture:
    def __init__(self, root):
        self.root = Path(root)
        self.controller = self.root / 'controller.csv'
        self.publications = self.root / 'command_publication.csv'
        self.native = self.root / 'native_wrench.csv'
        self.contacts = self.root / 'contact_frames.csv'
        self.geometry = self.root / 'geometry.csv'
        self.servo = self.root / 'wheel_servo.csv'
        self.output = self.root / 'out'
        self._write(self.controller, ['timestamp','state_name','ground_input_stage','pitch','pitch_rate','capture_com_velocity','effort_mode_active'], [
            {'timestamp':'0.099','state_name':'BALANCE','ground_input_stage':'effort_hold','pitch':'0.034','pitch_rate':'0','capture_com_velocity':'0','effort_mode_active':'1'}])
        self._write(self.publications, ['command_id','publish_ns','control_ns','state','kind','c0','c1','c2','c3','wheel_linear','wheel_angular','zero_effort'], [
            {'command_id':'a','publish_ns':'99000000','control_ns':'98000000','state':'BALANCE','kind':'leg','c0':'0','c1':'0','c2':'0','c3':'0','wheel_linear':'0','wheel_angular':'0','zero_effort':'0'},
            {'command_id':'b','publish_ns':'99500000','control_ns':'98500000','state':'BALANCE','kind':'leg','c0':'0','c1':'0','c2':'0','c3':'0','wheel_linear':'0','wheel_angular':'0','zero_effort':'0'}])
        self._write(self.servo, ['publish_ns','source_ns','wheel_index','published_torque','valid','valid_reason'], [])
        native_rows = []
        contact_rows = []
        geometry_rows = []
        for iteration, ns in ((100,100_000_000),(101,101_000_000)):
            for idx in range(6):
                native_rows.append(self.native_row(iteration, ns, idx))
            contact_rows.append({'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),'dt_ns':'1000000','frame_valid':'1','num_contacts':'2','collision_pairs_json':json.dumps([
                ['flat_jump_world::ground_plane::link::collision','flat_jump_world::bbot::link_004::link_004_collision_collision'],
                ['flat_jump_world::ground_plane::link::collision','flat_jump_world::bbot::link_007::link_007_collision_collision']]),'error':''})
            geometry_rows.append({'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),'dt_ns':'1000000','frame_valid':'1','error':'','base_x':'0','base_y':'0','base_z':'0.5'})
        self._write(self.native, list(native_rows[0]), native_rows)
        self._write(self.contacts, list(contact_rows[0]), contact_rows)
        self._write(self.geometry, list(geometry_rows[0]), geometry_rows)

    @staticmethod
    def native_row(iteration, ns, idx):
        row = {'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),'dt_ns':'1000000','joint_index':str(idx),
               'joint_name':audit_module.ACTUATORS[idx],'state_valid':'1','joint_valid':'1','joint_position':'0','joint_velocity':'0',
               'before_physics_phase':'before_physics_update','before_physics_sim_time_ns':str(ns),'before_physics_iteration':str(iteration),'before_physics_dt_ns':'1000000',
               'before_physics_joint_state_valid':'1','before_physics_joint_position':'0','before_physics_joint_velocity':'0',
               'before_physics_base_pose_valid':'1','before_physics_base_qx':'0','before_physics_base_qw':'1',
               'before_physics_base_velocity_valid':'1','before_physics_base_world_vx':'0','before_physics_base_world_vy':'0','before_physics_base_world_vz':'0',
               'before_physics_joint_force_cmd_component_present':'1','before_physics_joint_force_cmd_valid':'1','before_physics_joint_force_cmd_sim_input':'0',
               'joint_force_cmd_component_present':'1','joint_force_cmd_valid':'1','joint_force_cmd_sim_input':'0','wrench_valid':'1','transmitted_axis_torque':'0',
               'child_pose_valid':'1','child_x':'0','child_y':'0','child_z':'0','post_base_velocity_valid':'1','post_base_world_vx':'0','post_base_world_vy':'0','post_base_world_vz':'0'}
        return row

    @staticmethod
    def _write(path, fields, rows):
        with Path(path).open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)

    def run(self):
        return audit_module.audit(argparse.Namespace(controller_csv=self.controller, command_publication_csv=self.publications,
            native_wrench_csv=self.native, contact_frames_csv=self.contacts, geometry_csv=self.geometry,
            wheel_servo_csv=self.servo, output_dir=self.output, hold_start_ns=100_000_000, hold_end_ns=102_000_000))

    def native_rows(self):
        return audit_module.read_csv(self.native)

    def write_native(self, rows):
        self._write(self.native, list(rows[0]), rows)

    def contact_rows(self):
        return audit_module.read_csv(self.contacts)

    def write_contacts(self, rows):
        self._write(self.contacts, list(rows[0]), rows)


class GroundInputAuditTests(unittest.TestCase):
    def fixture(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        return Fixture(temp.name)

    def test_wrong_physics_pair_does_not_get_joined(self):
        f = self.fixture()
        rows = f.contact_rows(); rows[0]['physics_iteration'] = '999'
        f.write_contacts(rows)
        result = f.run()
        self.assertIn('invalid_or_missing_complete_bilateral_contact_frame', result['failures'])
        self.assertEqual(result['jump_metrics']['accepted_jumps'], 0)

    def test_clock_reversal_is_reported(self):
        f = self.fixture()
        rows = f.native_rows()
        # Move all rows in second frame backward while preserving row grouping.
        for row in rows[6:]:
            row['sim_time_ns'] = '99000000'; row['before_physics_sim_time_ns'] = '99000000'
        f.write_native(rows)
        result = f.run()
        self.assertGreater(result['clock']['time_reversals'], 0)
        self.assertIn('clock_reversal_or_step_error', result['failures'])

    def test_missing_joint_input_fails_six_input_coverage(self):
        f = self.fixture()
        rows = f.native_rows(); rows = [r for r in rows if not (r['physics_iteration']=='100' and r['joint_index']=='5')]
        f.write_native(rows)
        result = f.run()
        self.assertIn('six_physics_inputs_not_all_finite_valid', result['failures'])

    def test_controller_clock_rollback_is_not_hidden_by_sorting(self):
        f = self.fixture()
        rows = audit_module.read_csv(f.controller)
        rows.append(dict(rows[0], timestamp='0.098'))
        Fixture._write(f.controller, list(rows[0]), rows)
        result = f.run()
        self.assertGreater(result['clock']['controller_time_reversals'], 0)
        self.assertIn('controller_trace_clock_invalid', result['failures'])

    def test_duplicate_native_joint_row_is_hard_failure(self):
        f = self.fixture()
        rows = f.native_rows(); rows.append(dict(rows[0]))
        f.write_native(rows)
        result = f.run()
        self.assertGreater(result['clock']['duplicate_joint_rows'], 0)
        self.assertIn('duplicate_native_joint_rows', result['failures'])

    def test_nan_input_remains_invalid(self):
        f = self.fixture()
        rows = f.native_rows()
        rows[0]['before_physics_joint_force_cmd_sim_input'] = 'nan'
        f.write_native(rows)
        result = f.run()
        self.assertIn('six_physics_inputs_not_all_finite_valid', result['failures'])
        self.assertTrue(result['native_input_coverage']['all_invalid_values_remain_invalid'])

    def test_limit_contact_and_short_window_cannot_pass(self):
        f = self.fixture()
        rows = f.native_rows(); rows[0]['before_physics_joint_position'] = '1.52'
        f.write_native(rows)
        contacts = f.contact_rows(); contacts[0]['num_contacts'] = '0'; contacts[0]['collision_pairs_json'] = '[]'
        f.write_contacts(contacts)
        result = f.run()
        self.assertIn('joint_limit_margin_crossed', result['failures'])
        self.assertIn('airborne_or_single_contact_in_hold_window', result['failures'])
        self.assertIn('hold_window_shorter_than_5s', result['failures'])
        self.assertNotEqual(result['gate'], 'PASS')

    def test_physics_effort_and_servo_wheel_rate_caps_are_enforced(self):
        f = self.fixture()
        rows = f.native_rows()
        rows[2]['before_physics_joint_force_cmd_sim_input'] = '10.5'  # left wheel native input
        f.write_native(rows)
        servo_fields = ['publish_seq','ros_publish_ns','wall_publish_ns','cmd_source_ns','joint_source_ns',
            'cmd_valid','joint_valid','cmd_reason','joint_reason','valid_reason','linear_target','angular_target',
            'left_target_rate','right_target_rate','left_rate','right_rate','left_torque_cmd','right_torque_cmd',
            'left_saturated','right_saturated','zero_effort_requested','zero_effort_active']
        Fixture._write(f.servo, servo_fields, [{'publish_seq':'1','ros_publish_ns':'100000000','wall_publish_ns':'1',
            'cmd_source_ns':'99000000','joint_source_ns':'99000000','cmd_valid':'1','joint_valid':'1','cmd_reason':'valid',
            'joint_reason':'valid','valid_reason':'valid','linear_target':'0','angular_target':'0',
            'left_target_rate':'31','right_target_rate':'0','left_rate':'0','right_rate':'0',
            'left_torque_cmd':'11','right_torque_cmd':'0','left_saturated':'0','right_saturated':'0',
            'zero_effort_requested':'0','zero_effort_active':'0'}])
        result = f.run()
        self.assertIn('command_or_native_input_exceeded_effort_or_wheel_rate_limit', result['failures'])
        self.assertTrue(result['hard_input_limits']['native_before_physics_violations'])
        self.assertTrue(result['hard_input_limits']['wheel_target_rate_violations'])
        self.assertTrue(result['hard_input_limits']['wheel_servo_effort_violations'])

    def test_unestablished_window_still_writes_failure_and_full_curves(self):
        f = self.fixture()
        result = audit_module.audit(argparse.Namespace(controller_csv=f.controller, command_publication_csv=f.publications,
            native_wrench_csv=f.native, contact_frames_csv=f.contacts, geometry_csv=f.geometry,
            wheel_servo_csv=f.servo, output_dir=f.output, hold_start_ns=0, hold_end_ns=0))
        self.assertEqual(result['gate'], 'FAIL')
        self.assertIn('hold_window_not_established', result['failures'])
        self.assertTrue((f.output / 'summary.json').is_file())
        self.assertTrue((f.output / 'signals.csv').is_file())
        self.assertGreater(len(audit_module.read_csv(f.output / 'signals.csv')), 0)

    def test_complete_five_second_ground_hold_can_pass(self):
        f = self.fixture()
        controller, native, contacts, geometry = [], [], [], []
        pair_json = json.dumps([
            ['flat_jump_world::ground_plane::link::collision','flat_jump_world::bbot::link_004::link_004_collision_collision'],
            ['flat_jump_world::ground_plane::link::collision','flat_jump_world::bbot::link_007::link_007_collision_collision']])
        for frame in range(5002):
            iteration = 1000 + frame
            ns = 99_000_000 + frame * 1_000_000
            stage = 'position' if ns < 100_000_000 else 'effort_hold'
            controller.append({'timestamp':f'{ns*1e-9:.9f}', 'control_sim_time_ns':str(ns),
                'state_name':'BALANCE','ground_input_stage':stage,'pitch':'0.034','pitch_rate':'0',
                'capture_com_velocity':'0','effort_mode_active':'0' if stage == 'position' else '1'})
            contacts.append({'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),
                'dt_ns':'1000000','frame_valid':'1','num_contacts':'2','collision_pairs_json':pair_json,'error':''})
            geometry.append({'seq':str(iteration),'physics_iteration':str(iteration),'sim_time_ns':str(ns),
                'dt_ns':'1000000','frame_valid':'1','error':'','base_x':'0','base_y':'0','base_z':'0.5'})
            for idx in range(6):
                r = Fixture.native_row(iteration, ns, idx)
                r['before_physics_phase'] = 'before_physics_update'
                native.append(r)
        Fixture._write(f.controller, list(controller[0]), controller)
        Fixture._write(f.native, list(native[0]), native)
        Fixture._write(f.contacts, list(contacts[0]), contacts)
        Fixture._write(f.geometry, list(geometry[0]), geometry)
        servo_fields = ['publish_seq','ros_publish_ns','wall_publish_ns','cmd_source_ns','joint_source_ns',
            'cmd_valid','joint_valid','cmd_reason','joint_reason','valid_reason','linear_target','angular_target',
            'left_target_rate','right_target_rate','left_rate','right_rate','left_torque_cmd','right_torque_cmd',
            'left_saturated','right_saturated','zero_effort_requested','zero_effort_active']
        Fixture._write(f.servo, servo_fields, [{'publish_seq':'1','ros_publish_ns':'100000000','wall_publish_ns':'1',
            'cmd_source_ns':'99000000','joint_source_ns':'99000000','cmd_valid':'1','joint_valid':'1','cmd_reason':'valid',
            'joint_reason':'valid','valid_reason':'valid','linear_target':'0','angular_target':'0',
            'left_target_rate':'0','right_target_rate':'0','left_rate':'0','right_rate':'0',
            'left_torque_cmd':'0','right_torque_cmd':'0','left_saturated':'0','right_saturated':'0',
            'zero_effort_requested':'0','zero_effort_active':'0'}])
        args = argparse.Namespace(controller_csv=f.controller, command_publication_csv=f.publications,
            native_wrench_csv=f.native, contact_frames_csv=f.contacts, geometry_csv=f.geometry,
            wheel_servo_csv=f.servo, output_dir=f.output, hold_start_ns=100_000_000, hold_end_ns=5_100_000_000)
        result = audit_module.audit(args)
        self.assertEqual(result['gate'], 'PASS', result['failures'])
        self.assertEqual(result['continuity']['window_pairs'], 5000 * 6)
        self.assertEqual(result['counts']['window_frames_with_exact_native_identity'], 5000)
        self.assertEqual(result['counts']['window_frames_with_all_six_before_post_joint_states'], 5000)

    def test_physics_match_before_publication_api_end_is_not_latency(self):
        f = self.fixture()
        native, _ = audit_module.native_frames(f.native)
        event = {'actuator':'hip_left', 'value':0., 'publish_ns':99_000_000,
                 'valid':True, 'row':{'publish_end_ns':'101000000'}}
        report = audit_module.latency_audit([event], [], native)[0]
        self.assertFalse(report['latency_interpretable'])
        self.assertIsNone(report['latency_ns'])
        event['row']['publish_end_ns'] = '99500000'
        report = audit_module.latency_audit([event], [], native)[0]
        self.assertTrue(report['latency_interpretable'])
        self.assertEqual(report['latency_lower_bound_ns'], 500_000)
        self.assertEqual(report['latency_upper_bound_ns'], 1_000_000)

    def test_repeated_publication_values_make_latency_ambiguous(self):
        f = self.fixture()
        publications = audit_module.read_publications(f.publications)
        native, _ = audit_module.native_frames(f.native)
        latency = audit_module.latency_audit(publications, [], native)
        hip = [row for row in latency if row['actuator'] == 'hip_left']
        self.assertEqual(len(hip), 2)
        self.assertTrue(all(row['ambiguous_same_value_attribution'] for row in hip))
        self.assertFalse(any(row['latency_interpretable'] for row in hip))
        self.assertTrue(all(row['latency_ns'] != 0 or row['first_matching_physics_ns'] is None for row in hip))


if __name__ == '__main__':
    unittest.main()
