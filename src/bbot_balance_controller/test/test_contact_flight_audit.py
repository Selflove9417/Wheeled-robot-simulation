#!/usr/bin/env python3
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_contact_flight_metrics import audit
from run_flat_ground_jump_trial import audit_contact_flights


class ContactFlightAuditTest(unittest.TestCase):
    def write_full_trace(self, directory, forward_speed):
        path = Path(directory) / 'session_log.csv'
        rows = []
        for index in range(18):
            stamp = 10.0 + .02*index
            elapsed = stamp - 10.0
            rows.append(dict(
                timestamp=stamp+.002, state_name='FLIGHT' if index >= 4 else 'THRUST',
                flight_subphase=(-1 if index < 4 else (0 if index < 6 else (1 if index < 11 else 2))),
                jump_id='1', com_sample_stamp=stamp, com_world_z=.4+1.9*elapsed-.5*9.81*elapsed**2,
                com_velocity_valid='1', odom_sample_stamp=stamp,
                centroidal_world_x=.2, centroidal_world_y=-.4+forward_speed*elapsed,
                centroidal_world_sample_stamp=stamp, centroidal_world_velocity_valid='1',
                jump_forward_axis_x='0', jump_forward_axis_y='1', jump_forward_axis_valid='1'))
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader(); writer.writerows(rows)
        contacts = []
        for side in ('LEFT', 'RIGHT'):
            for base in (9.98, 10.34):
                for index in range(21):
                    contacts.append(dict(side=side, stamp_sec=base+.001*index,
                                         num_contacts=1, force_z=0))
        with path.with_name('session_contacts.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=contacts[0])
            writer.writeheader(); writer.writerows(contacts)
        return path

    def evaluate(self, acceleration=-9.81, omit_right=False, stale=False,
                 velocity=1.9, duplicate=False, sparse_contacts=False,
                 async_duplicate=False, changed_duplicate=False, nan_sample=False,
                 invalid_sample=False, stale_repeat=False, nan_stamp=False,
                 forward_velocity=.45, axis=(0., 1.), omit_horizontal=False,
                 nan_position=False, invalid_axis=False,
                 changed_position_duplicate=False, invalid_world_valid=False,
                 nan_duplicate_axis=False, invalid_axis_valid=False):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'trial_log.csv'
            rows = []
            for index in range(18):
                elapsed = .02 * index
                stamp = 10. + elapsed
                sample_stamp = (10. + .02*(1+index % 4) if duplicate else
                                (10. + .02*(index-1) if async_duplicate and index == 8 else
                                 (10.02 if stale_repeat and index == 8 else stamp)))
                sample_elapsed = sample_stamp - 10.
                sample_z = .4 + velocity*sample_elapsed + .5*acceleration*sample_elapsed**2
                if changed_duplicate and async_duplicate and index == 8:
                    sample_z += .01
                if nan_sample and index == 8:
                    sample_z = float('nan')
                if nan_stamp and index == 8:
                    sample_stamp = float('nan')
                axis_x, axis_y = axis
                if invalid_axis:
                    axis_x, axis_y = .1, .1
                if nan_duplicate_axis and index == 8:
                    axis_x = float('nan')
                row = dict(
                    timestamp=sample_stamp+.001*(index//4) if duplicate else stamp,
                    state_name='FLIGHT' if index >= 4 else 'THRUST',
                    com_sample_stamp=sample_stamp,
                    odom_sample_stamp=sample_stamp + (.01 if stale else 0.),
                    com_world_z=sample_z,
                    com_velocity_valid='0' if invalid_sample and index == 8 else '1')
                if not omit_horizontal:
                    row.update(
                        centroidal_world_x=.2 + (axis_x*forward_velocity-axis_y*.25)*sample_elapsed,
                        centroidal_world_y=-.4 + (axis_y*forward_velocity+axis_x*.25)*sample_elapsed,
                        centroidal_world_sample_stamp=sample_stamp,
                        centroidal_world_velocity_valid=('0' if invalid_world_valid and index == 8 else '1'),
                        jump_forward_axis_valid=('0' if invalid_axis_valid and index == 8 else '1'),
                        jump_forward_axis_x=axis_x,
                        jump_forward_axis_y=axis_y)
                    if nan_position and index == 8:
                        row['centroidal_world_x'] = float('nan')
                    if changed_position_duplicate and index == 8:
                        row['centroidal_world_x'] += .01
                rows.append(row)
            with path.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            contacts = []
            for side in ('LEFT', 'RIGHT'):
                if omit_right and side == 'RIGHT':
                    continue
                for base in (9.98, 10.34):
                    for index in ((0, 20) if sparse_contacts else range(21)):
                        contacts.append(dict(side=side, stamp_sec=base+.001*index,
                                             num_contacts=1, force_z=0))
            with path.with_name('trial_contacts.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=contacts[0])
                writer.writeheader()
                writer.writerows(contacts)
            return audit(path)

    def test_independent_fit_recovers_boundary_velocity_and_apex(self):
        result = self.evaluate()
        self.assertTrue(result['freefall_verified'])
        self.assertAlmostEqual(result['fit_acceleration'], -9.81, places=8)
        self.assertAlmostEqual(result['takeoff_vz_estimate'], 1.9, places=8)
        self.assertAlmostEqual(result['apex_delta_estimate'], 1.9**2/(2*9.81), places=8)
        self.assertAlmostEqual(result['flight_entry_delay'], .08, places=8)

    def test_message_gap_alone_is_not_freefall(self):
        result = self.evaluate(acceleration=-5.)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('not coherent', result['reason'])

    def test_both_wheels_must_bound_the_gap(self):
        result = self.evaluate(omit_right=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('no bilateral', result['reason'])

    def test_unsynchronized_geometry_is_rejected(self):
        result = self.evaluate(stale=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('unsynchronized', result['reason'])

    def test_repeated_frames_cannot_count_as_independent_samples(self):
        result = self.evaluate(duplicate=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('insufficient independent', result['reason'])

    def test_async_repeat_of_previously_paired_frame_is_deduplicated(self):
        result = self.evaluate(async_duplicate=True)
        self.assertTrue(result['freefall_verified'], result['reason'])
        self.assertEqual(result['samples'], 15)

    def test_changed_value_for_repeated_stamp_is_rejected(self):
        result = self.evaluate(async_duplicate=True, changed_duplicate=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('changed value', result['reason'])

    def test_nonfinite_com_sample_is_rejected(self):
        result = self.evaluate(nan_sample=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('invalid or unsynchronized', result['reason'])

    def test_invalid_velocity_sample_is_rejected(self):
        result = self.evaluate(invalid_sample=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('invalid or unsynchronized', result['reason'])

    def test_first_unpaired_frame_is_rejected(self):
        result = self.evaluate(stale=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('unsynchronized', result['reason'])

    def test_stale_repeat_of_known_frame_is_rejected(self):
        result = self.evaluate(stale_repeat=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('invalid or unsynchronized', result['reason'])

    def test_nonfinite_stamp_on_gap_row_is_rejected(self):
        result = self.evaluate(nan_stamp=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('invalid or unsynchronized', result['reason'])

    def test_world_axis_projection_recovers_horizontal_speed(self):
        result = self.evaluate(forward_velocity=.45, axis=(0., 1.))
        self.assertTrue(result['horizontal_fit_verified'], result['horizontal_reason'])
        self.assertAlmostEqual(result['takeoff_vx_estimate'], .45, places=8)
        self.assertAlmostEqual(result['horizontal_fit_rms'], 0.0, places=8)

    def test_rotated_world_axis_projection_recovers_same_speed(self):
        result = self.evaluate(forward_velocity=.42, axis=(.6, .8))
        self.assertTrue(result['horizontal_fit_verified'], result['horizontal_reason'])
        self.assertAlmostEqual(result['takeoff_vx_estimate'], .42, places=8)

    def test_missing_horizontal_observation_preserves_only_vertical_fit(self):
        result = self.evaluate(omit_horizontal=True)
        self.assertTrue(result['freefall_verified'])
        self.assertFalse(result['horizontal_fit_verified'])
        self.assertIn('missing', result['horizontal_reason'])

    def test_nonfinite_position_and_invalid_axis_reject_horizontal_fit(self):
        for options, fragment in (({'nan_position': True}, 'nonfinite'),
                                  ({'invalid_axis': True}, 'axis norm')):
            result = self.evaluate(**options)
            self.assertTrue(result['freefall_verified'])
            self.assertFalse(result['horizontal_fit_verified'])
            self.assertIn(fragment, result['horizontal_reason'])

    def test_stale_world_com_observation_rejects_horizontal_fit(self):
        result = self.evaluate(invalid_world_valid=True)
        self.assertTrue(result['freefall_verified'])
        self.assertFalse(result['horizontal_fit_verified'])
        self.assertIn('stale world COM', result['horizontal_reason'])

    def test_nan_duplicate_position_or_invalid_axis_validity_rejects_horizontal_fit(self):
        for options, fragment in (({'async_duplicate': True, 'nan_duplicate_axis': True}, 'nonfinite'),
                                  ({'invalid_axis_valid': True}, 'forward axis')):
            result = self.evaluate(**options)
            self.assertTrue(result['freefall_verified'])
            self.assertFalse(result['horizontal_fit_verified'])
            self.assertIn(fragment, result['horizontal_reason'])

    def test_runner_accepts_only_independently_fitted_horizontal_speed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            valid_path = self.write_full_trace(temp_dir, .45)
            metrics, failures = audit_contact_flights(valid_path, 1)
            self.assertEqual(failures, [])
            self.assertAlmostEqual(metrics[0]['takeoff_vx_estimate'], .45, places=8)

        with tempfile.TemporaryDirectory() as temp_dir:
            fast_path = self.write_full_trace(temp_dir, .60)
            metrics, failures = audit_contact_flights(fast_path, 1)
            self.assertTrue(metrics[0]['horizontal_fit_verified'])
            self.assertTrue(any('fitted horizontal COM vx 0.600000m/s' in item
                                for item in failures))

    def test_changed_horizontal_duplicate_is_rejected_for_speed_fit(self):
        result = self.evaluate(async_duplicate=True, changed_position_duplicate=True)
        self.assertFalse(result['horizontal_fit_verified'])
        self.assertIn('repeated COM stamp changed', result['horizontal_reason'])

    def test_sparse_contact_telemetry_cannot_bound_flight(self):
        result = self.evaluate(sparse_contacts=True)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('contact cadence', result['reason'])

    def test_extrapolated_apex_outside_gap_is_rejected(self):
        result = self.evaluate(velocity=4.)
        self.assertFalse(result['freefall_verified'])
        self.assertIn('apex outside', result['reason'])


if __name__ == '__main__':
    unittest.main()
