"""Safety admission checks for the private motion runner; no simulation."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('motion_runner', Path(__file__).with_name('run_ground_motion_trial.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

class SafetyAdmission(unittest.TestCase):
    def setUp(self):
        self.anchor = [.2558, -.3693, .2558, -.3693]
        self.row = {'control_sim_time_ns': '1000000000', 'state_name': 'BALANCE',
                    'ground_input_stage': 'effort_hold', 'effort_mode_active': '1',
                    'leg_mode_switch_pending': '0', 'pitch': '.07', 'pitch_rate': '.01'}
        for key, value in zip(('hip_pos_left','knee_pos_left','hip_pos_right','knee_pos_right'), self.anchor):
            self.row[key] = str(value)
        for key in ('hip_vel_left','knee_vel_left','hip_vel_right','knee_vel_right'):
            self.row[key] = '.01'

    def check(self, **changes):
        return runner.validate_motion_sample(dict(self.row, **changes), self.anchor, .07, .999)

    def test_valid_and_repeated_state(self):
        self.assertEqual(self.check(), 1.)
        self.assertEqual(runner.validate_motion_sample(self.row, self.anchor, .07, 1.), 1.)

    def test_mode_loss_pending_and_failure(self):
        for changes in ({'effort_mode_active':'0'}, {'leg_mode_switch_pending':'1'},
                        {'ground_input_stage':'failed'}, {'state_name':'FLIGHT'}):
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                self.check(**changes)

    def test_nonfinite_attitude_or_joint_is_rejected(self):
        for key in ('pitch','pitch_rate','hip_vel_left','knee_pos_right'):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.check(**{key:'nan'})

    def test_clock_rollback_domain_and_limits(self):
        for changes in ({'control_sim_time_ns':'998000000'}, {'hip_pos_left':'.34'},
                        {'knee_vel_right':'.101'}, {'pitch':'.171'}, {'pitch_rate':'.501'}):
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                self.check(**changes)
        with self.assertRaises(RuntimeError):
            runner.validate_motion_sample(dict(self.row, hip_pos_left='1.23'), [1.23,-.3693,.2558,-.3693], .07, .999)

if __name__ == '__main__':
    unittest.main()
