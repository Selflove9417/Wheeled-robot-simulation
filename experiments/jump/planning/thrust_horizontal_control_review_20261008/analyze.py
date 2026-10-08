"""Saved THRUST terminal input-path audit; no ROS, rollout, or parameter scan."""
from pathlib import Path
import csv,json,bisect,hashlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
O=D/'thrust_horizontal_control_review_20261008';O.mkdir(exist_ok=True)
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rr):
    with p.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
summaries=[];curves={};inputs={}
for n in ['B1','B2','B3','T1']:
    root=D/('clearance_apex_ab_20261008' if n[0]=='B' else 'thrust_height_032_ab_20261008')
    p=root/('physical' if n[0]=='B' else 'physical_runs')/n
    sm=json.loads((D/'thrust_momentum_budget_20261008'/n/'summary.json').read_text());off=round(sm['off_s']*1e9)
    rows=read(D/'thrust_momentum_budget_20261008'/n/'steps.csv');ss={int(r['sim_time_ns']):r for r in rows}
    cc=read(p/'velocity_log.csv');tt=[float(r['timestamp']) for r in cc]
    model=ET.parse(p/'actual_robot.urdf').getroot();radii=[];offsets={}
    for link in ['link_004','link_007']:
        e=model.find("link[@name='"+link+"']/collision");offsets[link]=np.array([float(v) for v in e.find('origin').attrib['xyz'].split()])
        radii.append(float(e.find('geometry/cylinder').attrib['radius']))
    assert abs(radii[0]-radii[1])<1e-12;radius=radii[0]
    native={}
    for r in csv.DictReader((p/'engine_frames.csv').open()):
        ns=int(r['sim_time_ns'])
        if ns>off:break
        if ns<off-60000000 or r['phase']!='after_step':continue
        name=r['entity_name'].split('::')[-1]
        if name not in ['link_003','link_006','link_004','link_007']:continue
        assert r['time_valid']=='1' and r['position_valid']=='1' and r['velocity_valid']=='1' and int(r['physical_state_time_ns'])==ns
        native.setdefault(ns,{})[name]=r
    out=[]
    for ns,rr in sorted(native.items()):
        assert len(rr)==4
        s=ss[ns];c=cc[max(0,bisect.bisect_right(tt,ns/1e9)-1)]
        center_vy=[]
        for k in ['link_004','link_007']:
            Q=Rotation.from_quat([float(rr[k]['quat_'+a]) for a in ['x','y','z','w']])
            om=np.array([float(rr[k]['angular_v'+a]) for a in 'xyz'])
            center_vy.append(float(rr[k]['linear_vy'])+np.cross(om,Q.apply(offsets[k]))[1])
        axle_vy=np.mean(center_vy)
        shank_wx=np.mean([float(rr[k]['angular_vx']) for k in ['link_003','link_006']])
        wheel_wx=np.mean([float(rr[k]['angular_vx']) for k in ['link_004','link_007']])
        wheel_dq=np.mean([float(s[k+'_dq']) for k in ['link_004_joint','link_007_joint']])
        # Exact native world velocities; the scalar rolling expression is the
        # planar upright-wheel interpretation, not a measured contact-slip sensor.
        for k in ['link_003','link_006','link_004','link_007']:
            Q=Rotation.from_quat([float(rr[k]['quat_'+a]) for a in ['x','y','z','w']]).as_matrix()
            assert np.linalg.norm(Q[:,0]-np.array([1.,0.,0.]))<1e-4
        row={'sim_time_ns':ns,'off_ms':(ns-off)/1e6,'Fy':float(s['Fy']),'Fz':float(s['Fz']),
            'pitch_moment':float(s['moment_preCOM']),'tangent_moment':float(s['tangent_moment_preCOM']),
            'normal_lever_preCOM_m':-float(s['normal_moment_preCOM'])/float(s['Fz']) if float(s['Fz'])!=0 else float('nan'),
            'COM_vy':float(s['COM_vy']),'COM_vz':float(s['COM_vz']),'axle_vy':axle_vy,
            'shank_world_wx':shank_wx,'wheel_world_wx':wheel_wx,'wheel_relative_dq':wheel_dq,
            'actual_wheel_dq_times_radius':radius*wheel_dq,
            'native_relative_geometry_compensation':float(s['COM_vy'])-axle_vy-radius*shank_wx,
            'planar_rolling_velocity_residual':axle_vy+radius*wheel_wx,
            'axis_angular_identity_residual':wheel_wx-shank_wx-wheel_dq,
            'hip_dq':float(s['link_002_joint_dq']),'knee_dq':float(s['link_003_joint_dq'])}
        for k in ['cmd_x','thrust_wheel_target','thrust_fwd_term','thrust_att_term','thrust_att_term_raw','thrust_attitude_scale',
                  'thrust_forward_speed_predicted','thrust_forward_velocity_kp','jump_takeoff_forward_speed',
                  'thrust_wheel_kinematics_raw_correction','thrust_wheel_kinematics_applied_correction',
                  'actual_tau_hip_left','actual_tau_knee_left','tau_body_hip','thrust_release_active']:
            row[k]=float(c[k])
        out.append(row)
    curves[n]=out;dump(O/(n+'_terminal_control.csv'),out)
    late=[r for r in out if -20<r['off_ms']<=0];Jfy=sum(r['Fy']*.001 for r in late);Jm=sum(r['tangent_moment']*.001 for r in late)
    release=next(float(r['timestamp']) for r in cc if r['state_name']=='THRUST' and r['thrust_release_active']=='1')
    postrel=[r for r in out if release*1e9<int(r['sim_time_ns'])<off]
    summaries.append({'run':n,'last20ms_forward_ground_impulse_Ns':Jfy,'last20ms_tangent_pitch_impulse':Jm,
        'signed_force_weighted_vertical_lever_m':Jm/Jfy if Jfy!=0 else None,
        'last20ms_target_not_reached_frames':sum(abs(r['cmd_x']-r['thrust_wheel_target'])>1e-4 for r in late),
        'last20ms_kinematics_clipped_frames':sum(r['thrust_wheel_kinematics_raw_correction']>.350001 for r in late),
        'postrelease_target_not_reached_frames':sum(abs(r['cmd_x']-r['thrust_wheel_target'])>1e-4 for r in postrel),
        'postrelease_frame_count':len(postrel),'wheel_axis_identity_max_residual':max(abs(r['axis_angular_identity_residual']) for r in out),
        'last20ms_command_range':[min(r['cmd_x'] for r in late),max(r['cmd_x'] for r in late)],
        'last20ms_actual_wheel_dq_range':[min(r['wheel_relative_dq'] for r in late),max(r['wheel_relative_dq'] for r in late)],
        'native_wheel_velocity_command_logged':any('velocity_cmd' in k for k in next(csv.DictReader((p/'native_wrench.csv').open()))),
        'at_off':out[-1]})
    for filename in ['velocity_log.csv','engine_frames.csv','actual_robot.urdf','native_wrench.csv']:
        f=p/filename;inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
    for filename in ['summary.json','steps.csv']:
        f=D/'thrust_momentum_budget_20261008'/n/filename
        inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
(O/'summary.json').write_text(json.dumps(summaries,indent=2))
fig,axs=plt.subplots(5,2,figsize=(15,16))
for n,rr in curves.items():
    t=[r['off_ms'] for r in rr];color={'B1':'C0','B2':'C1','B3':'C2','T1':'C3'}[n]
    axs[0,0].plot(t,[r['cmd_x'] for r in rr],color=color,label=n+' published')
    axs[0,0].plot(t,[r['thrust_wheel_target'] for r in rr],color=color,ls='--',label=n+' target')
    axs[0,1].plot(t,[r['actual_wheel_dq_times_radius'] for r in rr],color=color,label=n+' actual relative wheel dq * radius')
    for i,col,k in [(1,0,'Fy'),(1,1,'normal_lever_preCOM_m'),(2,0,'actual_tau_hip_left'),(2,1,'actual_tau_knee_left'),(3,0,'thrust_att_term'),(3,1,'thrust_fwd_term'),(4,0,'thrust_wheel_kinematics_raw_correction'),(4,1,'native_relative_geometry_compensation')]:
        axs[i,col].plot(t,[r[k] for r in rr],color=color,label=n)
labels=[('published wheel command/target m/s','actual joint-relative wheel velocity * radius m/s'),
        ('actual forward ground force Fy N','actual normal contact lever y-Cy m'),
        ('published per-hip torque Nm','published per-knee torque Nm'),
        ('published wheel attitude term m/s','published wheel forward term m/s'),
        ('logged kinematics raw correction m/s','instantaneous native kinematics expression m/s')]
for i in range(5):
    for j in range(2):
        ax=axs[i,j];ax.grid(alpha=.2);ax.legend(fontsize=7,ncol=2);ax.set_ylabel(labels[i][j]);ax.set_xlabel('ms from real liftoff')
axs[4,0].axhline(.35,color='k',ls=':');fig.suptitle('Saved B1/B2/B3/T1 terminal control paths: timing association, not identified force sensitivity')
fig.tight_layout(rect=(0,0,1,.975));fig.savefig(O/'terminal_control_paths.png',dpi=130)
frozen=json.loads((D/'thrust_height_032_ab_20261008/baseline_frozen_sha256.json').read_text());assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in frozen.items())
(O/'provenance.json').write_text(json.dumps({'physical_runs':0,'control_changed':False,'frozen_count':len(frozen),'frozen_unchanged':True,'input_sha256':inputs},indent=2))
print(json.dumps(summaries,indent=2))
