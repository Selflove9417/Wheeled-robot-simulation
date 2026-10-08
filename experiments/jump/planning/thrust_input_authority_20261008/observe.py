"""Replay encoder/IMU snapshots only; native H is the validation reference."""
from pathlib import Path
import csv,json,math,hashlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';O=D/'thrust_input_authority_20261008';O.mkdir(exist_ok=False)
def read(p):return list(csv.DictReader(p.open()))
def vec(el,attr,default='0 0 0'):return np.array(list(map(float,el.attrib.get(attr,default).split())))
def dump(p,rr):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
summary=[];inputs={}
for name in ['B1','B2','B3','T1']:
 root=D/('clearance_apex_ab_20261008/physical' if name[0]=='B' else 'thrust_height_032_ab_20261008/physical_runs')/name
 model=ET.parse(root/'actual_robot.urdf').getroot();meta={r['entity_name'].split('::')[-1]:r for r in read(root/'engine_frames.csv.inertials.csv')};params={}
 for n,r in meta.items():
  Q=Rotation.from_quat([float(r[k]) for k in ['qx','qy','qz','qw']]).as_matrix();I=np.array([[float(r[k]) for k in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]])
  params[n]=(float(r['mass']),np.array([float(r['c'+a]) for a in 'xyz']),Q@I@Q.T)
 joints=[]
 for j in model.findall('joint'):
  n=j.attrib['name'];parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link']
  if child not in params or parent not in params:continue
  assert j.attrib['type'] in ['revolute','continuous'];axis=vec(j.find('axis'),'xyz');assert np.linalg.norm(axis-[1,0,0])<1e-12
  origin=j.find('origin');joints.append((n,parent,child,vec(origin,'xyz'),Rotation.from_euler('xyz',vec(origin,'rpy')).as_matrix(),axis))
 assert len(joints)==6
 for n in ['link_004','link_007']:
  m,c,I=params[n];assert abs(c[1])+abs(c[2])+abs(I[0,1])+abs(I[0,2])<1e-12
 # The missing wheel cyclic angle cancels analytically from H_x: wheel COM
 # lies on spin axis and I_xy=I_xz=0. Evaluate that phase at an arbitrary zero;
 # wheel RATE is mandatory and read separately, never filled in as zero.
 def H(row):
  q={};dq={}
  for joint,part,side in [('link_002_joint','hip','left'),('link_003_joint','knee','left'),('link_005_joint','hip','right'),('link_006_joint','knee','right')]:
   q[joint]=float(row[part+'_pos_'+side]);dq[joint]=float(row[part+'_vel_'+side])
  for joint,side in [('link_004_joint','left'),('link_007_joint','right')]:q[joint]=0.;dq[joint]=float(row[side+'_wheel_vel'])
  assert np.isfinite(list(q.values())+list(dq.values())+[float(row['pitch_rate_raw'])]).all()
  states={'base_link':(np.zeros(3),np.eye(3),np.zeros(3),np.array([-float(row['pitch_rate_raw']),0,0]))};remaining=joints.copy()
  while remaining:
   progress=False
   for item in remaining.copy():
    n,parent,child,o,Q,axis=item
    if parent not in states:continue
    p,R,v,om=states[parent];r=R@o;Rc=R@Q@Rotation.from_rotvec(axis*q[n]).as_matrix();states[child]=(p+r,Rc,v+np.cross(om,r),om+R@Q@axis*dq[n]);remaining.remove(item);progress=True
   assert progress
  links={}
  for n,(p,R,v,om) in states.items():
   m,c,I=params[n];r=R@c;links[n]=(m,p+r,v+np.cross(om,r),om,R@I@R.T)
  M=sum(x[0] for x in links.values());C=sum(x[0]*x[1] for x in links.values())/M;V=sum(x[0]*x[2] for x in links.values())/M
  return -sum((I@om+np.cross(p-C,m*(v-V)))[0] for m,p,v,om,I in links.values())
 cc=[r for r in read(root/'velocity_log.csv') if r['state_name']=='THRUST'];truth=read(D/'thrust_momentum_budget_20261008'/name/'steps.csv');native={int(r['sim_time_ns']):r for r in truth};matched=[];unmatched=[]
 for r in cc:
  ti=round(float(r['imu_sample_stamp'])*1e9);tj=round(float(r['joint_sample_stamp'])*1e9);tc=round(float(r['timestamp'])*1e9)
  # Strictly no interpolation/extrapolation from logged asynchronous snapshots.
  if ti!=tj:
   unmatched.append({'control_time_ns':tc,'imu_stamp_ns':ti,'joint_stamp_ns':tj,'reason':'imu_joint_mismatch'});continue
  if ti not in native:continue
  nr=native[ti];hat=H(r);true=float(nr['H_pitch']);nowtruth=float(native[tc]['H_pitch']) if tc in native else float('nan')
  matched.append({'control_time_ns':tc,'sensor_stamp_ns':ti,'age_ms':(tc-ti)/1e6,'H_encoder_imu':hat,'H_native_same_stamp':true,'same_stamp_error':hat-true,'H_native_control_time':nowtruth,'current_time_error':hat-nowtruth,'raw_rate':float(r['pitch_rate_raw']),'native_rate_at_sensor_stamp':float(nr['pitch_rate'])})
 assert matched
 dump(O/(name+'_matched.csv'),matched);dump(O/(name+'_unmatched.csv'),unmatched)
 errors=np.array([r['same_stamp_error'] for r in matched]);now=np.array([r['current_time_error'] for r in matched]);now=now[np.isfinite(now)]
 lags=[]
 for ms in [-10,-5,-2,-1,0,1,2,5,10]:
  ee=[r['H_encoder_imu']-float(native[r['sensor_stamp_ns']+ms*1000000]['H_pitch']) for r in matched if r['sensor_stamp_ns']+ms*1000000 in native]
  lags.append({'reference_offset_ms':ms,'count':len(ee),'rms':float(np.sqrt(np.mean(np.array(ee)**2))) if ee else None})
 summary.append({'run':name,'controller_rows':len(cc),'imu_joint_equal_stamp_rows':sum(round(float(r['imu_sample_stamp'])*1e9)==round(float(r['joint_sample_stamp'])*1e9) for r in cc),'matched_reference_rows':len(matched),'unmatched_rows':len(unmatched),'age_ms_range':[min(r['age_ms'] for r in matched),max(r['age_ms'] for r in matched)],'same_stamp_H_rms_error':float(np.sqrt(np.mean(errors**2))),'same_stamp_H_max_abs_error':float(np.max(abs(errors))),'same_stamp_H_bias':float(np.mean(errors)),'control_time_H_rms_error':float(np.sqrt(np.mean(now**2))),'control_time_H_max_abs_error':float(max(abs(now))),'lag_reference_diagnostic':lags})
 for p in [root/'velocity_log.csv',root/'actual_robot.urdf',root/'actual_robot.sdf',root/'engine_frames.csv.inertials.csv',D/'thrust_momentum_budget_20261008'/name/'steps.csv']:
  inputs[str(p.relative_to(W))]=hashlib.sha256(p.read_bytes()).hexdigest()
json.dump(summary,(O/'summary.json').open('w'),indent=2)
f=json.loads((D/'thrust_height_032_ab_20261008/baseline_frozen_sha256.json').read_text());assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in f.items());json.dump({'physical_runs':0,'default_changed':False,'frozen_count':len(f),'frozen_unchanged':True,'inputs_sha256':inputs},(O/'provenance.json').open('w'),indent=2)
print(json.dumps(summary,indent=2))
