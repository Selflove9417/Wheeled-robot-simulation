"""Reuse native records for the stopped single-factor experiment; no ROS calls."""
from pathlib import Path
import csv, json, math, bisect, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

W = Path('/home/xy/bbot_ws_new')
DATA = W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
OLD = DATA/'clearance_apex_ab_20261008'
NEW = DATA/'thrust_height_032_ab_20261008'
OUT = NEW/'analysis'

def read(p):
    return list(csv.DictReader(p.open()))

def dump(p, rows):
    with p.open('w') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader(); w.writerows(rows)

records = []
for name in ['B1', 'B2', 'B3', 'T1']:
    root, physical = (OLD, 'physical') if name.startswith('B') else (NEW, 'physical_runs')
    d = root/physical/name
    m = json.loads((root/'analysis'/name/'metrics.json').read_text())
    flight = read(root/'analysis'/name/'flight.csv')
    ctl = read(d/'velocity_log.csv')
    th = [r for r in ctl if r['state_name'] == 'THRUST' and float(r['timestamp']) < m['off_s']]
    lo = round(float(th[0]['timestamp'])*1e9)
    off = round(m['off_s']*1e9); td = round(m['first_touch_s']*1e9)
    end = td + 600_000_000
    meta = {r['entity_name'].split('::')[-1]: r for r in read(d/'engine_frames.csv.inertials.csv')}
    M = sum(float(r['mass']) for r in meta.values())
    urdf = ET.parse(d/'actual_robot.urdf').getroot()
    for j in ['link_002_joint','link_005_joint','link_004_joint','link_007_joint']:
        assert urdf.find("joint[@name='"+j+"']/axis").attrib['xyz'] == '1 0 0'
    native = {}
    for r in csv.DictReader((d/'engine_frames.csv').open()):
        ns = int(r['sim_time_ns'])
        if ns > end: break
        if ns < lo or r['phase'] != 'after_step': continue
        assert r['time_valid'] == '1' and r['entity_present'] == '1'
        if r['entity_type'] == 'link':
            assert r['position_valid'] == '1' and r['velocity_valid'] == '1'
            native.setdefault(ns, {})[r['entity_name'].split('::')[-1]] = r
    states = []
    for ns, rr in native.items():
        assert set(meta) <= set(rr)
        P = np.zeros(3); V = np.zeros(3)
        for n, mm in meta.items():
            r = rr[n]; mass = float(mm['mass'])
            R = Rotation.from_quat([float(r['quat_'+a]) for a in ['x','y','z','w']])
            c = R.apply([float(mm['c'+a]) for a in 'xyz'])
            p = np.array([float(r['position_'+a]) for a in 'xyz']) + c
            v = np.array([float(r['linear_v'+a]) for a in 'xyz']) + np.cross([float(r['angular_v'+a]) for a in 'xyz'], c)
            P += mass*p; V += mass*v
        P /= M; V /= M
        lengths = []; verticals = []
        for h, wheel in [('link_002','link_004'),('link_005','link_007')]:
            delta = np.array([float(rr[h]['position_'+a])-float(rr[wheel]['position_'+a]) for a in 'xyz'])
            axis = Rotation.from_quat([float(rr[h]['quat_'+a]) for a in ['x','y','z','w']]).apply([1,0,0])
            # Parallel joint axes: remove arbitrary lateral link-origin offsets.
            lengths.append(float(np.linalg.norm(delta-np.dot(delta,axis)*axis)))
            verticals.append(float(delta[2]))
        states.append({'sim_time_ns':ns,'off_ms':(ns-off)/1e6,'contact_ms':(ns-td)/1e6,
                       'com_z':P[2],'com_vz':V[2], 'hip_wheel_length_left':lengths[0],
                       'hip_wheel_length_right':lengths[1], 'hip_wheel_vertical_left':verticals[0],
                       'hip_wheel_vertical_right':verticals[1]})
    bytime = {r['sim_time_ns']:r for r in states}
    assert all(t in bytime for t in range(lo,end+1,1_000_000))
    forces = {}
    for r in csv.DictReader((d/'contact_detail.csv').open()):
        ns = int(r['sim_time_ns'])
        if ns > end: break
        if ns < lo: continue
        assert r['time_valid']=='1' and r['feature_valid']=='1'
        f = forces.setdefault(ns, np.zeros(3))
        if int(r['index']) < 0: continue
        assert r['extra_valid']=='1' and r['point_valid']=='1'
        a = '::bbot::' in r['collision1']; b = '::bbot::' in r['collision2']; assert a != b
        f += (1 if a else -1)*np.array([float(r[k]) for k in ['fx1','fy1','fz1']])
    assert all(t in forces for t in range(lo,end+1,1_000_000))
    for r in states:
        f = forces[r['sim_time_ns']]
        r.update({'ground_fx':f[0],'ground_fy':f[1],'ground_fz':f[2]})
    gross = sum(forces[t][2]*.001 for t in range(lo+1_000_000,off+1,1_000_000))
    net = gross - M*9.81*(off-lo)/1e9
    dp = M*(bytime[off]['com_vz']-bytime[lo]['com_vz'])
    inp = []
    for r in csv.DictReader((d/'native_wrench.csv').open()):
        ns = int(r['sim_time_ns'])
        if ns >= off: break
        if ns < lo or r['joint_name'] not in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']: continue
        assert r['before_physics_joint_force_cmd_valid']=='1' and r['before_physics_joint_state_valid']=='1'
        inp.append({'sim_time_ns':ns, 'joint':r['joint_name'],
                    'q':float(r['before_physics_joint_position']), 'dq':float(r['before_physics_joint_velocity']),
                    'actual_input':float(r['before_physics_joint_force_cmd_sim_input']),
                    'aggregate_load':float(r['transmitted_axis_torque']) if r['wrench_valid']=='1' else float('nan')})
    joints = {}
    for j in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:
        samples = [r for r in inp if r['joint']==j]
        joints[j] = {'max_actual_effort_abs':max(abs(r['actual_input']) for r in samples),
                      'max_actual_speed_abs':max(abs(r['dq']) for r in samples),
                      'effort_ge_142p5_fraction':float(np.mean([abs(r['actual_input'])>=142.5 for r in samples]))}
    C = lambda k: np.array([float(r[k]) for r in th])
    rel = next((float(r['timestamp']) for r in th if r['thrust_release_active']=='1'), None)
    post = [r for r in states if td<=r['sim_time_ns']<=td+300_000_000]
    compression = {}
    for side in ['left','right']:
        for kind in ['length','vertical']:
            field = 'hip_wheel_'+kind+'_'+side
            mn = min(post,key=lambda r:r[field])
            compression[kind+'_'+side] = {'first_touch_m':bytime[td][field],
                'minimum_m':mn[field], 'reduction_m':bytime[td][field]-mn[field],
                'minimum_after_touch_ms':mn['contact_ms']}
    forcepeak = max(post,key=lambda r:r['ground_fz'])
    r = {'run':name,'metrics':m,'off':flight[0],'touch_post_step':flight[-1],
         'pre_touch':flight[-2], 'mass':M, 'thrust_first_log_s':lo/1e9,
         'thrust_last_log_s':float(th[-1]['timestamp']),
         'thrust_control_duration_ms':(float(th[-1]['timestamp'])-float(th[0]['timestamp']))*1000,
         'thrust_release_first_s':rel,'release_relative_off_ms':None if rel is None else (rel-m['off_s'])*1000,
         'thrust_release_blend_max':float(max(C('thrust_release_blend'))),
         'thrust_motion_elapsed_last':float(C('thrust_motion_elapsed')[-1]),
         'force_command_peak_per_leg_N':float(max(C('F_z'))*.5),
         'force_request_peak_per_leg_N':float(max(C('F_z_request'))),
         'force_budget_min_per_leg_N':float(min(C('F_z_limit'))),
         'force_budget_limit_fraction':float(np.mean(C('thrust_force_before_budget')>C('F_z_limit')+1e-4)),
         'extension_scale_min':float(min(C('thrust_extension_scale'))),
         'knee_reference_speed_max':float(max(abs(C('knee_vel_cmd_left')))),
         'knee_reference_at15_fraction':float(np.mean(abs(C('knee_vel_cmd_left'))>=14.999)),
         'thrust_attitude_blocked_fraction':float(np.mean(C('thrust_attitude_blocked')>0)),
         'velocity_reached_any':bool(max(C('velocity_reached'))>0),
         'target_v':float(max(C('target_takeoff_velocity'))),
         'ground_vertical_gross_impulse_Ns':gross,'ground_vertical_net_impulse_Ns':net,
         'native_COM_delta_momentum_Ns':dp, 'impulse_closure_Ns':net-dp,
         'native_thrust_joints':joints, 'compression_0_to_300ms':compression,
         'COM_drop_first_300ms_m':bytime[td]['com_z']-min(r['com_z'] for r in post),
         'ground_Fz_peak_first_300ms_N':forcepeak['ground_fz'],
         'ground_Fz_peak_after_touch_ms':forcepeak['contact_ms'],
         'ground_impulse_first_50ms_Ns':sum(forces[t][2]*.001 for t in range(td,td+50_000_000,1_000_000)),
         'COM_vz_touch_before':bytime[td-1_000_000]['com_vz'],
         'COM_vz_touch_after':bytime[td]['com_vz']}
    dump(OUT/(name+'_physical_window.csv'),states)
    dump(OUT/(name+'_thrust_inputs.csv'),inp)
    records.append(r)
    (OUT/'details.json').write_text(json.dumps(records,indent=2))
    print(name, 'net impulse',net,'contact Fz peak',forcepeak['ground_fz'],'compression',compression,flush=True)

# Decompose changes at equal elapsed time; keep peak-time shift as a separate term.
test = read(OUT/'T1/flight.csv')[:-1]
peak = max(test,key=lambda r:float(r['bilateral_clearance']))
pt = float(peak['off_ms']); tc = float(peak['bilateral_clearance']); tz = float(peak['com_z'])
parts=[]
for name in ['B1','B2','B3']:
    a = read(OLD/'analysis'/name/'flight.csv')[:-1]
    tt = [float(r['off_ms']) for r in a]
    cz = float(np.interp(pt,tt,[float(r['com_z']) for r in a]))
    cc = float(np.interp(pt,tt,[float(r['bilateral_clearance']) for r in a]))
    bp = max(float(r['bilateral_clearance']) for r in a)
    parts.append({'baseline':name,'test_peak_ms':pt,'test_minus_baseline_peak_clearance_m':tc-bp,
        'COM_world_z_difference_at_equal_time_m':tz-cz,
        'COM_initial_z_difference_m':float(test[0]['com_z'])-float(a[0]['com_z']),
        'relative_geometry_clearance_difference_at_equal_time_m':(tc-tz)-(cc-cz),
        'baseline_clearance_loss_at_test_peak_time_m':cc-bp})
(OUT/'clearance_decomposition.json').write_text(json.dumps(parts,indent=2))

fig,axs=plt.subplots(4,2,figsize=(13,15))
for r in records:
    name=r['run'];root=OLD if name.startswith('B') else NEW
    a=read(root/'analysis'/name/'flight.csv');t=np.array([float(v['off_ms']) for v in a])
    axs[0,0].plot(t,[100*float(v['bilateral_clearance']) for v in a],label=name)
    axs[0,1].plot(t,[100*(float(v['com_z'])-float(a[0]['com_z'])) for v in a],label=name)
    axs[1,0].plot(t,[float(v['pitch_deg']) for v in a],label=name)
    axs[1,1].plot(t,[float(v['pitch_rate']) for v in a],label=name)
    p=read(root/'analysis'/name/'post_contact.csv');tp=[float(v['contact_s']) for v in p]
    axs[2,0].plot(tp,[100*float(v['axle_displacement_m']) for v in p],label=name)
    axs[2,1].plot(tp,[float(v['pitch_deg']) for v in p],label=name)
    s=read(OUT/(name+'_physical_window.csv'));sp=[float(v['contact_ms']) for v in s]
    axs[3,0].plot(sp,[float(v['ground_fz']) for v in s],label=name)
    axs[3,1].plot(sp,[100*.5*(float(v['hip_wheel_length_left'])+float(v['hip_wheel_length_right'])) for v in s],label=name)
labels=[('same-frame bilateral clearance cm','COM rise from sustained no-contact cm'),
        ('native pitch deg','native pitch rate rad/s'),('axle displacement from real first touch cm','post-contact native pitch deg'),
        ('actual summed ground Fz N','actual hip-axis to wheel-axis length cm')]
for row,lab in enumerate(labels):
    for col in range(2):
        ax=axs[row,col];ax.set_ylabel(lab[col]);ax.grid(alpha=.2);ax.legend()
        ax.set_xlabel('ms from sustained no contact' if row<2 else 's from real first touch' if row==2 else 'ms from real first touch')
axs[0,0].axhline(20,color='k',ls=':')
axs[3,0].set_xlim(-20,300);axs[3,1].set_xlim(-20,300)
fig.suptitle('MEASURED Gazebo: frozen baseline .25 vs stopped first test .32')
fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=130)

fig,axs=plt.subplots(4,2,figsize=(14,12))
for row,r in enumerate(records):
    name=r['run'];root=OLD if name.startswith('B') else NEW;phys='physical' if name.startswith('B') else 'physical_runs'
    ctl=read(root/phys/name/'velocity_log.csv');th=[v for v in ctl if v['state_name']=='THRUST' and float(v['timestamp'])<r['metrics']['off_s']]
    inp=read(OUT/(name+'_thrust_inputs.csv'))
    for col,(part,joint) in enumerate([('hip','link_002_joint'),('knee','link_003_joint')]):
        rr=[v for v in inp if v['joint']==joint];off=r['metrics']['off_s'];ax=axs[row,col]
        ax.plot([(float(v['timestamp'])-off)*1000 for v in th],[float(v['actual_tau_'+part+'_left']) for v in th],ls='--',label='published command')
        ts=[(int(v['sim_time_ns'])/1e9-off)*1000 for v in rr]
        ax.plot(ts,[float(v['actual_input']) for v in rr],label='before-step actual input')
        ax.plot(ts,[float(v['aggregate_load']) for v in rr],label='after-step aggregate load')
        ax.set_ylabel(name+' '+part+' Nm');ax.grid(alpha=.2);ax.legend(fontsize=7)
for ax in axs[-1]:ax.set_xlabel('ms from sustained no contact')
fig.tight_layout();fig.savefig(OUT/'three_torque_types.png',dpi=130)
