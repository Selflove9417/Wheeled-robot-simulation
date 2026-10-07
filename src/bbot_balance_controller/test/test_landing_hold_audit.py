#!/usr/bin/env python3
import csv
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from analyze_flat_jump_landing_hold import audit, audit_session

class LandingHoldAuditTest(unittest.TestCase):
    def fixture(self):
        rows=[]
        for index in range(1301):
            t=1.+.01*index
            state='FLIGHT' if index==0 else 'TOUCHDOWN_BUFFER' if index<55 else 'BALANCE'
            rows.append(dict(timestamp=t,state_name=state,pitch=.045,pitch_rate=0.,
                com_lean=-.038,capture_com_velocity=0.,gazebo_world_z_dot=0.,z=.503,
                hip_pos_left=.28,hip_pos_right=.28,knee_pos_left=-.378,knee_pos_right=-.378,
                capture_world_valid='1',com_sample_stamp=t-.01,imu_sample_stamp=t-.01,
                joint_sample_stamp=t-.01,odom_sample_stamp=t-.01))
        return rows
    def evaluate(self, rows):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'trial_log.csv'
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
            return audit(path,12.)
    def test_stationary_standing_holds(self):
        self.assertTrue(self.evaluate(self.fixture())['landing_hold_pass'])
    def test_early_success_cannot_hide_late_joint_limit(self):
        rows=self.fixture();rows[-1]['knee_pos_left']=-1.57
        result=self.evaluate(rows);self.assertFalse(result['landing_hold_pass'])
        self.assertIn('joint near hard stop',result['reason'])
    def test_late_emergency_invalidates_recovery(self):
        rows=self.fixture();rows[-1]['state_name']='EMERGENCY'
        self.assertIn('left BALANCE',self.evaluate(rows)['reason'])
    def test_invalid_and_stale_sensing_are_rejected(self):
        rows=self.fixture();rows[-1]['pitch']=float('nan')
        self.assertFalse(self.evaluate(rows)['landing_hold_pass'])
        rows=self.fixture();rows[-1]['com_sample_stamp']=1.
        self.assertIn('stale/invalid',self.evaluate(rows)['reason'])
    def test_quiet_tail_cannot_hide_earlier_invalid_standing(self):
        for channel in ('z', 'knee_pos_left', 'capture_com_velocity'):
            with self.subTest(channel=channel):
                rows=self.fixture();rows[65][channel]=float('nan')
                result=self.evaluate(rows)
                self.assertFalse(result['landing_hold_pass'])
                self.assertIn('nonfinite standing',result['reason'])
    def test_each_jump_id_gets_its_own_full_hold_window(self):
        first=self.fixture()
        for row in first: row['jump_id']='1'
        second=self.fixture()
        for row in second:
            row['jump_id']='2'
            row['timestamp'] += 20.
        second[-1]['state_name']='EMERGENCY'
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'session_log.csv'
            rows=first+second
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
            results=audit_session(path,12.,2)
        self.assertTrue(results[0]['landing_hold_pass'])
        self.assertFalse(results[1]['landing_hold_pass'])
        self.assertIn('left BALANCE',results[1]['reason'])
    def test_missing_jump_id_does_not_inherit_previous_hold(self):
        rows=self.fixture()
        for row in rows: row['jump_id']='1'
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'session_log.csv'
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
            results=audit_session(path,12.,2)
        self.assertFalse(results[0]['landing_hold_pass'])
        self.assertIn('jump_id sequence',results[0]['reason'])
        self.assertFalse(results[1]['landing_hold_pass'])
        self.assertIn('no data',results[1]['reason'])
    def test_reused_jump_id_invalidates_every_session_result(self):
        first=self.fixture()
        for row in first: row['jump_id']='1'
        second=self.fixture()
        for row in second:
            row['jump_id']='2'
            row['timestamp'] += 20.
        reused=self.fixture()
        for row in reused:
            row['jump_id']='1'
            row['timestamp'] += 40.
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'session_log.csv'
            rows=first+second+reused
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
            results=audit_session(path,12.,2)
        self.assertTrue(all(not result['landing_hold_pass'] for result in results))
        self.assertTrue(all('jump_id sequence' in result['reason'] for result in results))
    def test_standalone_session_audit_rejects_reused_id_sequence(self):
        rows=[]
        for jump_id, offset in (('1',0.),('2',20.),('1',40.)):
            hop=self.fixture()
            for row in hop:
                row['jump_id']=jump_id
                row['timestamp'] += offset
            rows.extend(hop)
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'session_log.csv'
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
            results=audit_session(path,12.)
        self.assertEqual(len(results),3)
        self.assertTrue(all(not result['landing_hold_pass'] for result in results))
        self.assertTrue(all('jump_id sequence' in result['reason'] for result in results))
    def test_sparse_or_nonmonotonic_standing_trace_is_rejected(self):
        rows=self.fixture()
        # A full-duration claim cannot be inferred across a missing 100ms interval.
        for row in rows[700:]:
            row['timestamp'] += .10
        self.assertIn('standing trace gap',self.evaluate(rows)['reason'])
        rows=self.fixture()
        rows[700]['timestamp']=rows[699]['timestamp']
        self.assertIn('duplicate or out-of-order',self.evaluate(rows)['reason'])
        rows=self.fixture()
        rows[700]['timestamp']=rows[699]['timestamp']-.001
        self.assertIn('duplicate or out-of-order',self.evaluate(rows)['reason'])
    def test_stale_sensor_sample_anywhere_in_hold_fails(self):
        rows=self.fixture()
        rows[500]['joint_sample_stamp']=rows[500]['timestamp']-.081
        result=self.evaluate(rows)
        self.assertIn('stale/invalid standing sensing',result['reason'])

if __name__=='__main__':
    unittest.main()
