#!/usr/bin/env python3
"""Validate free COM motion inside a wheel-contact event gap.

These are model estimates at the last collision-message boundary, not measured
contact forces or a replacement for the controller's acceptance verdict.
Fit acceleration independently; never assume a message gap alone is flight or
substitute the largest velocity recorded during the jump.
"""
import argparse
import csv
import math
from pathlib import Path
import numpy as np

FIELDS = ('trial', 'jump_id', 'contact_source', 'freefall_verified', 'contact_boundary', 'contact_return',
          'flight_entry_delay', 'fit_acceleration', 'fit_rms', 'samples',
          'takeoff_vz_estimate', 'apex_delta_estimate', 'reason',
          'horizontal_fit_verified', 'takeoff_vx_estimate', 'horizontal_fit_rms',
          'horizontal_samples', 'horizontal_reason')

def audit(path, jump_id=None, ground_frame_path=None, parsed_ground_frames=None):
    out = dict.fromkeys(FIELDS, '')
    out.update(trial=path.stem, jump_id='' if jump_id is None else jump_id,
               freefall_verified=False)
    if ground_frame_path is None:
        from audit_ground_contact_frames import companion_frame_path
        companion = companion_frame_path(path)
        if companion.exists():
            ground_frame_path = companion
        elif (Path(str(companion) + ".required").exists() or
              Path(str(companion) + ".error").exists()):
            out['contact_source'] = 'complete_frames_missing'
            out['reason'] = 'complete ground-frame source was declared but is missing'
            return out
    out['contact_source'] = 'complete_frames' if ground_frame_path is not None else 'legacy_positive_events'
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    flight = next((r for r in rows if r.get('state_name') == 'FLIGHT' and
                   (jump_id is None or str(r.get('jump_id', '')) == str(jump_id))), None)
    if not flight:
        out['reason'] = 'no FLIGHT'
        return out
    entry = float(flight['timestamp'])
    contact_path = path.with_name(path.name.replace('_log.csv', '_contacts.csv'))
    if not contact_path.exists():
        out['reason'] = 'no contact events'
        return out
    with contact_path.open(newline='') as stream:
        contacts = list(csv.DictReader(stream))
    if ground_frame_path is not None:
        try:
            if parsed_ground_frames is None:
                from audit_ground_contact_frames import validate_active_trace
                ok, reason, parsed_ground_frames = validate_active_trace(
                    path, ground_frame_path)
                if not ok:
                    raise ValueError(reason)
            from audit_ground_contact_frames import flight_gap
            start, end = flight_gap(parsed_ground_frames, int(round(entry * 1e9)))
        except (OSError, ValueError) as exc:
            out['reason'] = f'complete contact-frame gap invalid: {exc}'
            return out
    else:
        boundaries = []
        for side in ('LEFT', 'RIGHT'):
            stamps = sorted(set(float(r['stamp_sec']) for r in contacts
                                if r['side'] == side and int(r['num_contacts']) > 0))
            gap = next(((a, b) for a, b in zip(stamps, stamps[1:]) if a < entry < b), None)
            if gap is None:
                out['reason'] = 'no bilateral contact-event gap around FLIGHT'
                return out
            a, b = gap
            before = [t for t in stamps if a-.020 <= t <= a]
            after = [t for t in stamps if b <= t <= b+.020]
            if any(len(ts) < 8 or ts[-1]-ts[0] < .010 or
                   max(y-x for x, y in zip(ts, ts[1:])) > .005+1e-8
                   for ts in (before, after)):
                out['reason'] = 'contact cadence cannot bound the event gap'
                return out
            boundaries.append(gap)
        start = max(a for a, _ in boundaries)
        end = min(b for _, b in boundaries)
    if not .080 <= end-start <= .800:
        out['reason'] = 'event gap outside flight range'
        return out
    samples = {}
    horizontal_fields = ('centroidal_world_x', 'centroidal_world_y',
                         'centroidal_world_sample_stamp', 'jump_forward_axis_x',
                         'jump_forward_axis_y', 'jump_forward_axis_valid',
                         'centroidal_world_velocity_valid')
    has_horizontal_fields = all(name in rows[0] for name in horizontal_fields) if rows else False
    horizontal_reason = '' if has_horizontal_fields else 'missing synchronized COM position/forward-axis fields'
    for row in rows:
        if jump_id is not None and str(row.get('jump_id', '')) != str(jump_id):
            continue
        try:
            stamp = float(row['com_sample_stamp'])
            z = float(row['com_world_z'])
            row_stamp = float(row['timestamp'])
            odom_stamp = float(row['odom_sample_stamp'])
            world_x = float(row['centroidal_world_x']) if has_horizontal_fields else math.nan
            world_y = float(row['centroidal_world_y']) if has_horizontal_fields else math.nan
            world_stamp = (float(row['centroidal_world_sample_stamp'])
                           if has_horizontal_fields else math.nan)
            axis_x = float(row['jump_forward_axis_x']) if has_horizontal_fields else math.nan
            axis_y = float(row['jump_forward_axis_y']) if has_horizontal_fields else math.nan
            axis_valid = row.get('jump_forward_axis_valid') if has_horizontal_fields else ''
            world_valid = row.get('centroidal_world_velocity_valid') if has_horizontal_fields else ''
        except (KeyError, TypeError, ValueError):
            out['reason'] = 'invalid or unsynchronized COM flight sample'
            return out
        if start+.010 < row_stamp < end-.010 and not math.isfinite(stamp):
            out['reason'] = 'invalid or unsynchronized COM flight sample'
            return out
        if start+.010 < stamp < end-.010:
            if (row.get('com_velocity_valid') != '1' or
                not all(math.isfinite(v) for v in (stamp, z, row_stamp, odom_stamp)) or
                not 0 <= row_stamp-stamp <= .080):
                out['reason'] = 'invalid or unsynchronized COM flight sample'
                return out
            if stamp in samples:
                # The control loop can log the last complete COM frame again after
                # odometry advances but before a new joint-history upper bound arrives.
                # It is valid cached evidence only when its value is unchanged.
                previous = samples[stamp]
                if z != previous[0]:
                    out['reason'] = 'COM sample stamp repeated with changed value'
                    return out
                if has_horizontal_fields and not all(math.isfinite(v) for v in
                        (world_x, world_y, world_stamp, axis_x, axis_y)):
                    horizontal_reason = 'nonfinite COM position, sample stamp, or forward axis'
                elif has_horizontal_fields:
                    if (world_x, world_y, world_stamp, axis_x, axis_y) != previous[1:]:
                        horizontal_reason = 'repeated COM stamp changed position or forward axis'
                if has_horizontal_fields and (world_valid != '1' or axis_valid != '1'):
                    horizontal_reason = 'invalid or stale world COM position observation'
                continue
            if abs(odom_stamp-stamp) > .001:
                out['reason'] = 'invalid or unsynchronized COM flight sample'
                return out
            if has_horizontal_fields and math.isfinite(world_stamp) and abs(world_stamp-stamp) > .001:
                horizontal_reason = 'COM position stamp does not match COM velocity stamp'
            if has_horizontal_fields and world_valid != '1':
                horizontal_reason = 'invalid or stale world COM position observation'
            if has_horizontal_fields and axis_valid != '1':
                horizontal_reason = 'invalid or stale world forward axis'
            samples[stamp] = (z, world_x, world_y, world_stamp, axis_x, axis_y)
    if len(samples) < 6 or max(samples, default=start)-min(samples, default=start) < .100:
        out['reason'] = 'insufficient independent COM frames'
        return out
    stamps = sorted(samples)
    t = np.array(stamps)-start
    z = np.array([samples[s][0] for s in stamps])
    fit = np.polynomial.polynomial.polyfit(t, z, 2)
    rms = float(np.sqrt(np.mean((np.polynomial.polynomial.polyval(t, fit)-z)**2)))
    acceleration = float(2*fit[2])
    out.update(contact_boundary=start, contact_return=end, flight_entry_delay=entry-start,
               fit_acceleration=acceleration, fit_rms=rms, samples=len(samples))
    if abs(acceleration+9.81) > .30 or rms > .001:
        out['reason'] = 'contact gap is not coherent free COM fall'
        return out
    apex_t = -fit[1]/acceleration
    if not 0 < apex_t < end-start:
        out['reason'] = 'apex outside observed gap'
        return out
    out.update(freefall_verified=True, takeoff_vz_estimate=float(fit[1]),
               apex_delta_estimate=float(-fit[1]**2/(2*acceleration)))
    if has_horizontal_fields and not horizontal_reason:
        positions = [samples[s][1:] for s in stamps]
        values = np.asarray(positions, dtype=float)
        axis = values[0, 3:5]
        axis_norm = float(np.linalg.norm(axis))
        if not np.isfinite(values).all():
            horizontal_reason = 'nonfinite COM position, sample stamp, or forward axis'
        elif np.any(np.abs(values[:, 2] - np.asarray(stamps)) > .001):
            horizontal_reason = 'COM position stamp does not match COM velocity stamp'
        elif not .999 <= axis_norm <= 1.001:
            horizontal_reason = 'invalid world forward axis norm'
        elif np.max(np.linalg.norm(values[:, 3:5] - axis[None, :], axis=1)) > 1e-6:
            horizontal_reason = 'world forward axis changed during flight fit'
        elif len(stamps) < 6 or stamps[-1]-stamps[0] < .100:
            horizontal_reason = 'insufficient independent COM frames for horizontal fit'
        else:
            projected = values[:, 0]*axis[0] + values[:, 1]*axis[1]
            horizontal_fit = np.polynomial.polynomial.polyfit(t, projected, 1)
            horizontal_rms = float(np.sqrt(np.mean(
                (np.polynomial.polynomial.polyval(t, horizontal_fit)-projected)**2)))
            out.update(takeoff_vx_estimate=float(horizontal_fit[1]),
                       horizontal_fit_rms=horizontal_rms,
                       horizontal_samples=len(stamps))
            if horizontal_rms <= .001:
                out['horizontal_fit_verified'] = True
                horizontal_reason = ''
            else:
                horizontal_reason = 'horizontal COM position fit RMS exceeds 1 mm'
    out['horizontal_reason'] = horizontal_reason
    return out

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = []
    for directory in args.directories:
        for path in sorted(directory.glob('*_log.csv')):
            with path.open(newline='') as stream:
                source_rows = list(csv.DictReader(stream))
            hop_ids = list(dict.fromkeys(r.get('jump_id', '') for r in source_rows
                                         if r.get('state_name') == 'FLIGHT'))
            if hop_ids:
                results.extend(audit(path, hop_id) for hop_id in hop_ids)
            else:
                results.append(audit(path))
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader(); writer.writerows(results)
    valid = [r for r in results if r['freefall_verified']]
    print('Coherent contact-gap free fall:', len(valid), '/', len(results))
    if valid:
        print('FLIGHT-entry delay range:', min(r['flight_entry_delay'] for r in valid),
              max(r['flight_entry_delay'] for r in valid))
    print('Estimates do not overwrite formal jump acceptance:', args.output)

if __name__ == '__main__':
    main()
