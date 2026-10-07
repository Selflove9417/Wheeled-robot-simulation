#!/usr/bin/env python3
"""Audit per-step simulation inputs against native velocity changes, offline."""
import argparse
import csv
import json
import math
import subprocess
from pathlib import Path
import numpy as np

ORDER = (0, 1, 3, 4, 2, 5)
NAMES = ('base_forward', 'base_z', 'theta', 'hipL', 'kneeL', 'hipR', 'kneeR', 'wheelL', 'wheelR')

def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def stat(x):
    x = np.asarray(x, dtype=float)
    if not len(x):
        return {'count': 0}
    return {'count': len(x), 'RMS': float(np.sqrt(np.mean(x*x))),
            'median_abs': float(np.median(np.abs(x))), 'max_abs': float(np.max(np.abs(x)))}

def vector(row, prefix):
    return np.array([float(row[prefix+a]) for a in 'xyz'])

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('--bridge', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--gate', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stem = args.log.name.removesuffix('_log.csv')
    folder = args.log.parent
    native = {}
    coverage = {'rows': 0, 'pre_valid': 0, 'before_physics_valid': 0, 'post_valid': 0,
                'pre_nonzero': 0, 'before_physics_nonzero': 0, 'post_nonzero': 0,
                'invalid_command_values_are_nan': True, 'timing_identity': True}
    for r in rows(folder/(stem+'_native_wrench.csv')):
        ns = int(r['sim_time_ns']); i = int(r['joint_index'])
        native.setdefault(ns, {})[i] = r
        coverage['rows'] += 1
        for label, flag, value in (
            ('pre', 'joint_force_cmd_valid', 'joint_force_cmd_sim_input'),
            ('before_physics', 'before_physics_joint_force_cmd_valid', 'before_physics_joint_force_cmd_sim_input'),
            ('post', 'post_joint_force_cmd_valid', 'post_joint_force_cmd_sim_input')):
            v = float(r[value]); valid = r[flag] == '1'
            coverage[label+'_valid'] += int(valid)
            coverage[label+'_nonzero'] += int(valid and abs(v) > 1e-12)
            if not valid and not math.isnan(v): coverage['invalid_command_values_are_nan'] = False
        coverage['timing_identity'] &= all(r[a] == r[b] for a,b in (
            ('sim_time_ns','before_physics_sim_time_ns'), ('physics_iteration','before_physics_iteration'),
            ('dt_ns','before_physics_dt_ns'), ('sim_time_ns','pre_sim_time_ns'),
            ('physics_iteration','pre_physics_iteration'), ('dt_ns','pre_dt_ns')))
    contact = {int(r['sim_time_ns']): r for r in rows(folder/(stem+'_ground_frames.csv'))}
    geometry = {int(r['sim_time_ns']): r for r in rows(folder/(stem+'_ground_geometry.csv'))}
    logs = rows(args.log)
    times = np.array([float(r['timestamp']) for r in logs])
    inputs, meta, continuity = [], [], []
    skipped = {}
    previous = None
    for ns, joints in native.items():
        reason = None
        r = joints.get(0)
        if len(joints) != 6 or r is None: reason = 'six_joint_coverage'
        elif not all(joints[i][col] == '1' for i in ORDER for col in (
            'before_physics_joint_state_valid','state_valid','before_physics_joint_force_cmd_valid','wrench_valid')):
            reason = 'state_or_command_unavailable'
        elif r['before_physics_base_pose_valid'] != '1' or r['before_physics_base_velocity_valid'] != '1' or r['post_base_velocity_valid'] != '1':
            reason = 'base_unavailable'
        elif ns not in contact or ns not in geometry or contact[ns]['frame_valid'] != '1' or geometry[ns]['frame_valid'] != '1':
            reason = 'contact_geometry_unavailable'
        if reason:
            skipped[reason] = skipped.get(reason,0)+1
            previous = None
            continue
        q = np.array([float(r['before_physics_base_y']), float(r['before_physics_base_z']),
                      2*math.atan2(float(r['before_physics_base_qx']),float(r['before_physics_base_qw']))] +
                     [float(joints[i]['before_physics_joint_position']) for i in ORDER])
        v = np.array([float(r['before_physics_base_world_vy']),float(r['before_physics_base_world_vz']),
                      float(r['before_physics_base_world_wx'])]+[float(joints[i]['before_physics_joint_velocity']) for i in ORDER])
        post = np.array([float(r['post_base_world_vy']),float(r['post_base_world_vz']),float(r['post_base_world_wx'])]+
                        [float(joints[i]['joint_velocity']) for i in ORDER])
        if previous is not None and previous[0]+1_000_000 == ns:
            continuity.append(float(np.max(np.abs(v-previous[1]))))
        previous = (ns, post)
        before = contact.get(ns-1_000_000)
        after = contact.get(ns+1_000_000)
        if before is None or after is None:
            continue
        n = int(contact[ns]['num_contacts'])
        if not all(c['frame_valid'] == '1' and int(c['num_contacts']) == n for c in (before,after)) or n not in (0,2):
            skipped['contact_transition_or_single'] = skipped.get('contact_transition_or_single',0)+1
            continue
        index = np.searchsorted(times,ns*1e-9,side='right')-1
        if index < 0: continue
        phase = logs[index]['state_name']
        if phase not in ('THRUST','FLIGHT'): continue
        dt = int(r['dt_ns'])*1e-9
        if dt != .001: raise ValueError('Physics step changed')
        cmd = np.array([float(joints[i]['before_physics_joint_force_cmd_sim_input']) for i in ORDER])
        net = np.array([float(joints[i]['transmitted_axis_torque']) for i in ORDER])
        acc = (post-v)/dt
        inputs.append(np.r_[ns*1e-9,int(n==2),q,v,acc,cmd,net])
        meta.append({'time':ns*1e-9,'phase':phase,'contact_count':n})
    inp = args.output/'response_input.txt'
    np.savetxt(inp,inputs,fmt='%.17g')
    with inp.open() as f:
        result = subprocess.run([str(args.bridge.resolve())],stdin=f,capture_output=True,text=True,check=True)
    (args.output/'response_output.txt').write_text(result.stdout)
    a = np.array([[float(x) for x in line.split()] for line in result.stdout.splitlines()])
    if len(a) != len(meta) or len(a) == 0: raise ValueError('No paired response rows')
    gate = json.loads(args.gate.read_text())
    coverage['frames'] = len(native)
    coverage['same_step_velocity_continuity'] = stat(continuity)
    report = {'coverage':coverage,'skipped_frames':skipped,'phases':{},'response_gate':gate,
              'semantics':{'command':'JointForceCmd in observer Update before Physics in the private world; simulation input, not measured motor torque.',
                           'net_load':'JointTransmittedWrench projected joint aggregate, not actuator input.',
                           'prediction':'Fixed six observed force commands, CAD 9D model and ideal bilateral rolling contact; no observed wheel acceleration fed into predictor.',
                           'observed_acceleration':'Same-step (post velocity minus before-Physics velocity)/1ms; no centered-window held-command mixing.'}}
    failures = []
    if not coverage['timing_identity'] or not coverage['invalid_command_values_are_nan']:
        failures.append('timing or invalid value handling failed')
    if not continuity or max(continuity) > gate['same_step_velocity_continuity_max']:
        failures.append('before/post velocity continuity failed')
    for label,phase,n in (('bilateral_thrust','THRUST',2),('freeflight','FLIGHT',0)):
        mask = np.array([m['phase']==phase and m['contact_count']==n for m in meta])
        x = a[mask]; errors = x[:,3:12]-x[:,12:21]
        summary = {'steps':len(x),'acceleration_error':{},'native_acceleration':{},'relative_RMS':{},
                   'equation_residual':stat(x[:,2]),'contact_acceleration_residual':{str(i):stat(x[:,25+i]) for i in range(4)}}
        for j,name in enumerate(NAMES):
            summary['acceleration_error'][name] = stat(errors[:,j])
            summary['native_acceleration'][name] = stat(x[:,12+j])
            ratio = summary['acceleration_error'][name].get('RMS',float('inf'))/max(summary['native_acceleration'][name].get('RMS',0),1.)
            summary['relative_RMS'][name] = ratio
            if label == 'bilateral_thrust' and (summary['acceleration_error'][name].get('RMS',float('inf')) > gate['thrust_acceleration_RMS_limits'][j] or ratio > gate['thrust_acceleration_relative_RMS_max']):
                failures.append(name+' forward response fails')
        report['phases'][label] = summary
        if label == 'bilateral_thrust' and len(x) < gate['minimum_thrust_steps']:
            failures.append('insufficient bilateral THRUST steps')
    report['failures'] = failures
    report['integration_gate'] = 'PASS' if not failures else 'FAIL'
    (args.output/'response_review.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'response_metadata.json').write_text(json.dumps(meta))
    print(json.dumps({'integration_gate':report['integration_gate'],'failures':failures,
                      'coverage':coverage,'bilateral_thrust':report['phases']['bilateral_thrust']},indent=2))

if __name__ == '__main__': main()
