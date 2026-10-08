"""Finite saved-contact leverage budget; offline geometry, no dynamics or ROS."""
from pathlib import Path
import csv,json,hashlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';P=W/'experiments/jump/planning/thrust_configuration_feasibility_20261008';O=D/'thrust_configuration_feasibility_20261008';O.mkdir(exist_ok=True)
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rr):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
rows=[];inputs={}
for n in ['B1','B2','B3','T1']:
 b=D/'thrust_momentum_budget_20261008'/n;s=json.loads((b/'summary.json').read_text());ss=read(b/'steps.csv');whole=s['whole']
 rows.append({'run':n,'normal_J':sum(float(r['normal_moment_preCOM'])*.001 for r in ss if float(r['thrust_ms'])>0),'tangent_J':sum(float(r['tangent_moment_preCOM'])*.001 for r in ss if float(r['thrust_ms'])>0),'Jz':whole['vertical_gross_impulse'],'Jy':whole['horizontal_contact_impulse_Fy'],'COM_y_change':whole['final']['COM_y']-whole['initial']['COM_y'],'COM_z_change':whole['final']['COM_z']-whole['initial']['COM_z'],'hip_q0':whole['initial']['link_002_joint_q'],'hip_q1':whole['final']['link_002_joint_q'],'hip_v1':whole['final']['link_002_joint_dq']})
 for fn in ['summary.json','steps.csv','contact_points.csv']:
  f=b/fn;inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
dump(O/'four_run_contact_budget.csv',rows)
b=D/'thrust_momentum_budget_20261008/T1';ss=read(b/'steps.csv');s=json.loads((b/'summary.json').read_text());off=round(s['off_s']*1e9);root=D/'thrust_height_032_ab_20261008/physical_runs/T1';meta={r['entity_name'].split('::')[-1]:r for r in read(root/'engine_frames.csv.inertials.csv')};M=sum(float(r['mass']) for r in meta.values());raw={}
for r in csv.DictReader((root/'engine_frames.csv').open()):
 ns=int(r['sim_time_ns'])
 if ns>off:break
 if ns<off-181000000 or r['phase']!='before_step' or r['entity_type']!='link':continue
 assert r['time_valid']=='1' and r['position_valid']=='1' and r['velocity_valid']=='1'
 raw.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
model=ET.parse(root/'actual_robot.urdf').getroot();limits={}
for joint in ['link_002_joint','link_005_joint']:
 j=model.find("joint[@name='"+joint+"']");assert j.find('axis').attrib['xyz']=='1 0 0';limits[joint]={k:float(v) for k,v in j.find('limit').attrib.items()}
# Conditional geometric displacement: same body pitch/knee, both wheel axles
# anchored. Rotate each complete leg about its measured hip, then translate all
# links to restore wheel axle position. This is NOT a reachable dynamic rollout.
geoms={}
for ns,rr in raw.items():
 assert set(rr)==set(meta)
 legs=[];anchors=[]
 for thigh,shank,wheel in [('link_002','link_003','link_004'),('link_005','link_006','link_007')]:
  hip=np.array([float(rr[thigh]['position_'+a]) for a in 'xyz']);Q=Rotation.from_quat([float(rr[thigh]['quat_'+a]) for a in ['x','y','z','w']]);axis=Q.apply([1,0,0]);assert np.linalg.norm(axis-[1,0,0])<1e-4
  anchors.append((np.array([float(rr[wheel]['position_'+a]) for a in 'xyz'])-hip,axis))
  for n in [thigh,shank,wheel]:
   Q=Rotation.from_quat([float(rr[n]['quat_'+a]) for a in ['x','y','z','w']]);pc=np.array([float(rr[n]['position_'+a]) for a in 'xyz'])+Q.apply([float(meta[n]['c'+a]) for a in 'xyz']);legs.append((float(meta[n]['mass']),pc-hip,axis))
 geoms[ns]=(legs,anchors)
def delta(ns,q):
 legs,anchors=geoms[ns];C=sum(m*(Rotation.from_rotvec(ax*q).apply(rad)-rad) for m,rad,ax in legs)/M
 C-=sum(Rotation.from_rotvec(ax*q).apply(rad)-rad for rad,ax in anchors)/2
 return C
cop=[]
for r in ss:
 ns=int(r['sim_time_ns'])
 if ns not in raw or float(r['Fz'])<=0:continue
 axle=.5*(float(raw[ns]['link_004']['position_y'])+float(raw[ns]['link_007']['position_y']))
 cop.append({'off_ms':float(r['off_ms']),'COP_minus_axle_y_m':float(r['COP_y'])-axle,'Fz':float(r['Fz'])})
dump(O/'T1_COP_relative_axle.csv',cop)
cop_summary={'min_m':min(r['COP_minus_axle_y_m'] for r in cop),'max_m':max(r['COP_minus_axle_y_m'] for r in cop),'Fz_weighted_mean_m':sum(r['COP_minus_axle_y_m']*r['Fz'] for r in cop)/sum(r['Fz'] for r in cop)}
windows=[]
for ms in [20,40,60,80,100,181]:
 rr=[r for r in ss if -ms<float(r['off_ms'])<=0];jz=sum(float(r['Fz'])*.001 for r in rr);jy=sum(float(r['Fy'])*.001 for r in rr)
 def smooth(r):
  u=(float(r['off_ms'])+ms)/ms;return 10*u**3-15*u**4+6*u**5
 jr=sum(float(r['Fz'])*smooth(r)*.001 for r in rr)
 def impulse(q):
  return sum((float(r['Fz'])*delta(int(r['sim_time_ns']),q*smooth(r))[1]-float(r['Fy'])*delta(int(r['sim_time_ns']),q*smooth(r))[2])*.001 for r in rr)
 def solve(target):
  q=brentq(lambda q:impulse(q)-target,-1.2,1.2)
  shifts=[delta(int(r['sim_time_ns']),q*smooth(r)) for r in rr]
  # Endpoint joint change vs original off-state, not measured joint headroom claim.
  d=delta(off,q);h_end=s['whole']['final']['link_002_joint_q']+q
  return {'deltaH_requested':target,'conditional_common_hip_delta_rad':q,'conditional_COM_y_endpoint_m':float(d[1]),'conditional_COM_z_endpoint_m':float(d[2]),'incremental_quintic_peak_speed_radps':1.875*abs(q)/(ms*.001),'incremental_quintic_peak_acc_radps2':(10/np.sqrt(3))*abs(q)/(ms*.001)**2,'endpoint_hip_q':h_end,'endpoint_within_URDF_limit':limits['link_002_joint']['lower']<=h_end<=limits['link_002_joint']['upper'],'conditional_frozen_force_impulse':impulse(q)}
 try:sol=[solve(t) for t in [.48,.60]]
 except ValueError:sol=None
 windows.append({'window_ms':ms,'Jz':jz,'Jy':jy,'COM_y_constant_offset_m_for_048_060':[.48/jz,.60/jz],'COM_y_quintic_endpoint_m_for_048_060':[.48/jr,.60/jr],'conditional_hip_solutions':sol})
json.dump({'four_runs':rows,'T1_windows':windows,'hip_URDF_limits':limits,'T1_COP_relative_axle':cop_summary,'assumptions':['Frozen recorded forces and contact points','Body pitch and knee held at recorded values','Symmetric common hip perturbation; wheel axles anchored','Geometric impulse prediction only, not actual actuator or contact response','flight_hip_acc_limit is airborne-only; not a demonstrated grounded THRUST bound']},(O/'summary.json').open('w'),indent=2)
# Explicit observed hip PD residual, to avoid equating nominal target with actual configuration.
controls=[]
for n in ['B1','B2','B3','T1']:
 p=D/('clearance_apex_ab_20261008/physical' if n[0]=='B' else 'thrust_height_032_ab_20261008/physical_runs')/n
 cc=[r for r in read(p/'velocity_log.csv') if r['state_name']=='THRUST'];end=float(cc[-1]['timestamp']);a=[r for r in cc if float(r['timestamp'])>=end-.060]
 controls.append({'run':n,'samples':len(a),'hip_PD_lower_clipped_count':sum(abs(float(r['actual_tau_hip_left'])-float(r['tau_body_hip'])+3)<.0001 for r in a),'hip_PD_zero_count':sum(abs(float(r['actual_tau_hip_left'])-float(r['tau_body_hip']))<.0001 for r in a),'hip_tracking_q_error_range':[min(float(r['hip_pos_cmd_left'])-float(r['hip_pos_left']) for r in a),max(float(r['hip_pos_cmd_left'])-float(r['hip_pos_left']) for r in a)],'hip_actual_velocity_range':[min(float(r['hip_vel_left']) for r in a),max(float(r['hip_vel_left']) for r in a)],'hip_reference_velocity_range':[min(float(r['hip_vel_cmd_left']) for r in a),max(float(r['hip_vel_cmd_left']) for r in a)],'pitch_ref_values':sorted({r['active_jump_pitch_ref'] for r in a})})
 for fn in ['velocity_log.csv','runtime_parameters.yaml','actual_robot.urdf']:
  f=p/fn;inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
json.dump(controls,(O/'hip_control_observations.json').open('w'),indent=2)
for fn in ['engine_frames.csv','engine_frames.csv.inertials.csv','actual_robot.urdf']:
 f=root/fn;inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
frozen=json.loads((D/'thrust_height_032_ab_20261008/baseline_frozen_sha256.json').read_text());assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in frozen.items());json.dump({'physical_runs':0,'control_changed':False,'frozen_count':len(frozen),'frozen_unchanged':True,'input_sha256':inputs},(O/'provenance.json').open('w'),indent=2)
print(json.dumps({'windows':windows,'controls':controls},indent=2))
