"""Optimistic geometric/time upper scenarios, NOT executable or closed-loop.
Compute finite step-count budgets; no control parameter search or simulation.
"""
from pathlib import Path
import json,csv,math,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import brentq
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials');OLD=R/'clearance_apex_ab_20261008';OUT=R/'clearance_20cm_feasibility_20261008';res=[]
for name in ['B1','B2','B3']:
 d=OLD/'physical'/name;u=E.parse(d/'actual_robot.urdf').getroot();js=[]
 for j in u.findall('joint'):
  if j.attrib['type']=='fixed':continue
  js.append((j.attrib['name'],j.find('parent').attrib['link'],j.find('child').attrib['link'],np.array(list(map(float,j.find('origin').attrib['xyz'].split())))))
 meta={r['entity_name'].split('::')[-1]:r for r in csv.DictReader((d/'engine_frames.csv.inertials.csv').open())};pars={}
 for n,r in meta.items():
  Q=Rot.from_quat([float(r[k]) for k in ['qx','qy','qz','qw']]).as_matrix();I=np.array([[float(r[k]) for k in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]]);pars[n]=(float(r['mass']),np.array([float(r['c'+k]) for k in 'xyz']),Q@I@Q.T)
 index={j[0]:i for i,j in enumerate(js)};legs=[index[n] for n in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']];wheels=[index[n] for n in ['link_004_joint','link_007_joint']];M=sum(x[0] for x in pars.values());m=json.load(open(OLD/'analysis'/name/'metrics.json'));air=list(csv.DictReader((OLD/'analysis'/name/'flight.csv').open()))[:-1];peak=max(air,key=lambda r:float(r['bilateral_clearance']));apex=max(air,key=lambda r:float(r['com_z']));wanted={int(peak['sim_time_ns']),int(apex['sim_time_ns'])};geo={}
 for r in csv.DictReader((d/'geometry.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>max(wanted):break
  if ns in wanted:geo[ns]=r
 def kin(q,dq):
  ll={'base_link':(np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3))};todo=js.copy()
  while todo:
   for n,pa,ch,o in todo.copy():
    if pa not in ll:continue
    p,Rotpa,w,v=ll[pa];dis=Rotpa@o;Rc=Rotpa@Rot.from_rotvec([q[index[n]],0,0]).as_matrix();ll[ch]=(p+dis,Rc,w+Rc[:,0]*dq[index[n]],v+np.cross(w,dis));todo.remove((n,pa,ch,o))
  pc={n:p+Q@pars[n][1] for n,(p,Q,w,v) in ll.items()};C=sum(pars[n][0]*pc[n] for n in ll)/M;I=np.zeros((3,3));H=np.zeros(3)
  for n,(p,Q,w,v) in ll.items():
   mass,c,Ic=pars[n];r=pc[n]-C;Ii=Q@Ic@Q.T;I+=Ii+mass*((r@r)*np.eye(3)-np.outer(r,r));H+=Ii@w+np.cross(r,mass*(v+np.cross(w,Q@c)))
  return I,H,ll,C
 def bottoms(q,Rb):
  _,_,ll,C=kin(q,np.zeros(6));a=[]
  for k in wheels:
   j=js[k];link=u.find("link[@name='"+j[2]+"']");col=link.find('collision');orig=col.find('origin');xyz=np.zeros(3) if orig is None else np.array(list(map(float,orig.attrib['xyz'].split())));rpy=np.zeros(3) if orig is None else np.array(list(map(float,orig.attrib.get('rpy','0 0 0').split())));Qcol=Rot.from_euler('xyz',rpy).as_matrix();cyl=col.find('geometry/cylinder');rad=float(cyl.attrib['radius']);half=float(cyl.attrib['length'])/2;p,Q,_,_=ll[j[2]];axis=Rb@Q@Qcol[:,2];center=Rb@(p+Q@xyz-C);a.append(float(center[2]-rad*math.sqrt(max(0,1-axis[2]**2))-half*abs(axis[2])))
  return a
 q0=np.zeros(6)
 for k,(part,side) in zip(legs,[(part,side) for side in ['left','right'] for part in ['hip','knee']]):q0[k]=float(m['tuck_entry'][part+'_pos_'+side])
 # One analytical momentum-neutral hip increment per protected knee step.
 def step(q,d):
  I,_,_,_=kin(q,np.zeros(6));vh=np.zeros(6);vk=np.zeros(6);vh[legs[::2]]=1;vk[legs[1::2]]=1;_,Ah,_,_=kin(q,vh);_,Ak,_,_=kin(q,vk);ratio=-np.linalg.solve(I,Ak)[0]/np.linalg.solve(I,Ah)[0];new=q.copy();new[legs[::2]]+=ratio*d;new[legs[1::2]]+=d;return new,ratio
 # A current protected .04 knee step has mathematical rest/rest min 25 ms.
 step_ms=25;start_ms=m['extend_ms'];before_apex=m['com_apex_ms']-start_ms;full_budget=(m['flight_ms']-start_ms)-90-55-20;counts={'no_settle_before_apex':math.floor((before_apex+1e-8)/step_ms),'no_settle_before_deploy_deadline':math.floor((full_budget+1e-8)/step_ms),'65ms_settle_before_apex':max(0,math.floor((before_apex-65+1e-8)/step_ms))};scenarios=[]
 for scenario,N in counts.items():
  q=q0.copy();ratios=[]
  for _ in range(N):q,ratio=step(q,.04);ratios.append(ratio)
  assert np.max(abs(q[legs]))<1.57
  pred=[]
  for tag,original in [('old_COM_apex',apex),('old_clearance_peak',peak)]:
   g=geo[int(original['sim_time_ns'])];Rb=Rot.from_quat([float(g['base_'+a]) for a in ['qx','qy','qz','qw']]).as_matrix();pp=float(original['com_z'])+min(bottoms(q,Rb));pred.append({'at':tag,'old_ms':float(original['off_ms']),'conditional_clearance_m':pp,'change_from_original_same_frame_m':pp-float(original['bilateral_clearance'])})
  scenarios.append({'scenario':scenario,'step_count':N,'zero_velocity_stops':N,'extra_retraction_ms':N*step_ms,'joint_delta_from_q0':(q-q0)[legs].tolist(),'configuration_q':q[legs].tolist(),'conditional_geometry_at_original_frames':pred,'kinematic_only_no_tracking_or_momentum_pose_proof':True})
 # Root for total knee motion needed from settled q0 to 20cm at old COM apex.
 g=geo[int(apex['sim_time_ns'])];Rb=Rot.from_quat([float(g['base_'+a]) for a in ['qx','qy','qz','qw']]).as_matrix()
 def endpoint(delta):
  # Fine integration of the differential momentum-neutral kinematic branch, not time dynamics.
  q=q0.copy();N=100
  for _ in range(N):q,_=step(q,delta/N)
  return q
 f=lambda delta:float(apex['com_z'])+min(bottoms(endpoint(delta),Rb))-.20
 needed=brentq(f,0.,.8);Nneed=math.ceil(needed/.04);min_segment_time_ms=Nneed*25
 res.append({'run':name,'counts':counts,'time_available_before_apex_ms':before_apex,'time_available_before_90ms_deploy_plus75ms_ms':full_budget,'minimum_shared_knee_delta_to_20_at_old_apex_rad':needed,'minimum_guarded_step_count_for_20':Nneed,'optimistic_guarded_step_time_for_20_ms':min_segment_time_ms,'scenarios':scenarios});print(json.dumps(res[-1]),flush=True)
(OUT/'tuck_bound.json').write_text(json.dumps(res,indent=2)+'\n')
