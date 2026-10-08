"""Offline conditional air-trajectory design; no ROS or controller mutations.
Full seven-body spin+orbital angular momentum, physical model parameters.
The new trajectory is an ideal tracking scenario, not a predicted closed loop.
"""
from pathlib import Path
import csv,json,math,hashlib,xml.etree.ElementTree as ET
import yaml,re
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';OLD=ROOT/'clearance_apex_ab_20261008';OUT=ROOT/'airborne_reallocation_design_20261008'
def read(p):return list(csv.DictReader(p.open()))
def vec(r,p):return np.array([float(r[p+k]) for k in 'xyz'])
def peaks(delta,T):return 1.875*abs(delta)/T,(10/math.sqrt(3))*abs(delta)/T**2
results=[];curve=[];fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
for name in ['B1','B2','B3']:
 d=OLD/'physical'/name;m=json.loads((OLD/'analysis'/name/'metrics.json').read_text());model=ET.parse(d/'actual_robot.urdf').getroot();meta={r['entity_name'].split('::')[-1]:r for r in read(d/'engine_frames.csv.inertials.csv')};pars={}
 for n,r in meta.items():
  c=vec(r,'c');Q=Rot.from_quat([float(r[k]) for k in ['qx','qy','qz','qw']]).as_matrix();I=np.array([[float(r[k]) for k in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]]);pars[n]=(float(r['mass']),c,Q@I@Q.T)
 js=[]
 for j in model.findall('joint'):
  if j.attrib['type']=='fixed':continue
  n=j.find('child').attrib['link']
  if n not in pars:continue
  assert j.find('origin').attrib['rpy']=='0 0 0' and j.find('axis').attrib['xyz']=='1 0 0'
  js.append((j.attrib['name'],j.find('parent').attrib['link'],n,np.array([float(x) for x in j.find('origin').attrib['xyz'].split()])))
 assert len(js)==6 and len(pars)==7
 M=sum(x[0] for x in pars.values()); index={j[0]:i for i,j in enumerate(js)};legs=[index[n] for n in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']];wheels=[index[n] for n in ['link_004_joint','link_007_joint']]
 def kinematics(q,dq):
  ll={'base_link':(np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3))};todo=js.copy()
  while todo:
   for j in todo.copy():
    n,pa,ch,o=j
    if pa not in ll:continue
    p,R,wr,ur=ll[pa];dis=R@o;Rc=R@Rot.from_rotvec([q[index[n]],0,0]).as_matrix();ll[ch]=(p+dis,Rc,wr+Rc[:,0]*dq[index[n]],ur+np.cross(wr,dis));todo.remove(j)
  coms={n:p+R@pars[n][1] for n,(p,R,w,v) in ll.items()};C=sum(pars[n][0]*coms[n] for n in ll)/M;Ilock=np.zeros((3,3));Hr=np.zeros(3)
  for n,(p,R,w,v) in ll.items():
   mass,c,I=pars[n];rad=coms[n]-C;Iw=R@I@R.T;Ilock+=Iw+mass*((rad@rad)*np.eye(3)-np.outer(rad,rad));Hr+=Iw@w+np.cross(rad,mass*(v+np.cross(w,R@c)))
  return Ilock,Hr,ll
 def distance(q,side):
  il=legs[2*side];ik=legs[2*side+1];k=js[ik][3];wheel=js[wheels[side]][3];return float(np.linalg.norm(Rot.from_rotvec([q[il],0,0]).apply(k)+Rot.from_rotvec([q[il]+q[ik],0,0]).apply(wheel)))
 off=m['off_s'];ex=m['extend_entry'];tu=m['tuck_entry'];ns=round(float(ex['timestamp'])*1e9);rr={}
 for r in csv.DictReader((d/'engine_frames.csv').open()):
  t=int(r['sim_time_ns'])
  if t>ns:break
  if t==ns and r['phase']=='after_step':rr[r['entity_name'].split('::')[-1]]=r
 assert set(pars)<=set(rr)
 actualq=np.array([float(rr[j[0]]['joint_position_0']) for j in js]);actualv=np.array([float(rr[j[0]]['joint_velocity_0']) for j in js]);ll={}
 for n in pars:
  r=rr[n];R=Rot.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();p=vec(r,'position_');v=vec(r,'linear_v');om=vec(r,'angular_v');mass,c,I=pars[n];ll[n]=(mass,p+R@c,v+np.cross(om,R@c),R@I@R.T,om)
 C=sum(l[0]*l[1] for l in ll.values())/M;V=sum(l[0]*l[2] for l in ll.values())/M;H=sum(l[3]@l[4]+np.cross(l[1]-C,l[0]*(l[2]-V)) for l in ll.values());Rb=Rot.from_quat([float(rr['base_link']['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();I,Hr,_=kinematics(actualq,actualv);identity=H-Rb@(I@(Rb.T@ll['base_link'][4])+Hr);assert np.linalg.norm(identity)<1e-9
 # Conditional settling reaches old TUCK terminal reference, then rest-to-rest shortening.
 q0=actualq.copy();v0=actualv.copy()
 for i,(k,s) in zip(legs,[(k,s) for s in ['left','right'] for k in ['hip','knee']]):q0[i]=float(tu[k+'_pos_'+s]);v0[i]=0
 qf=q0.copy()
 Iref,_,_=kinematics(q0,np.zeros(6)); ah=np.zeros(6);ak=np.zeros(6);ah[legs[::2]]=1;ak[legs[1::2]]=1
 _,Hhip,_=kinematics(q0,ah);_,Hknee,_=kinematics(q0,ak);axis_world=np.array([1.,0,0]);ratio=-float(axis_world@Rb@np.linalg.solve(Iref,Hknee))/float(axis_world@Rb@np.linalg.solve(Iref,Hhip))
 for side in [0,1]:
  h,k=legs[side*2:side*2+2];o=js[k][3];w=js[wheels[side]][3]
  qf[k]=q0[k]+.04
  before=o+Rot.from_rotvec([q0[k],0,0]).apply(w);after=o+Rot.from_rotvec([qf[k],0,0]).apply(w)
  qf[h]=q0[h]+ratio*.04
 assert max(abs(qf[legs]-q0[legs]))<=.14 and all(abs(qf[k]-q0[k])<=.04000000001 for k in legs[1::2])
 T=.040;dv=qf-q0;pk=[peaks(dv[i],T) for i in legs];Tmin=max([.005]+[max(1.875*abs(dv[i])/(11 if k%2==0 else 13),math.sqrt((10/math.sqrt(3))*abs(dv[i])/(450 if k%2==0 else 500))) for k,i in enumerate(legs)]);Tmin5=math.ceil(Tmin/.005)*.005
 # Coordinates solve the same I_locked/H_relative identity as the validated audit.
 # Integrate a matched hold comparison with the SAME H, initial orientation and frozen wheel dq.
 Rnew=Rb.copy();Rhold=Rb.copy();maxres=0;sample=[];dyn=[]
 for j in range(41):
  t=j*.001;u=t/T;s=10*u**3-15*u**4+6*u**5;ds=(30*u*u-60*u**3+30*u**4)/T;qt=q0+dv*s+v0*t;vt=v0+dv*ds;qh=q0+v0*t;I,Hr,shapen=kinematics(qt,vt);I0,Hr0,shapeh=kinematics(qh,v0);wn=np.linalg.solve(I,Rnew.T@H-Hr);wh=np.linalg.solve(I0,Rhold.T@H-Hr0);pn=-float((Rnew@wn)[0]);ph=-float((Rhold@wh)[0]);maxres=max(maxres,float(np.linalg.norm(H-Rnew@(I@wn+Hr))));contrib={}
  for group,inds in [('hip',legs[::2]),('knee',legs[1::2]),('wheel',wheels)]:
   v=np.zeros(6);v[inds]=vt[inds];_,hg,_=kinematics(qt,v);contrib[group]=float((Rnew@np.linalg.solve(I,hg))[0])
  Cn=sum(pars[n][0]*(p+R@pars[n][1]) for n,(p,R,w,v) in shapen.items())/M;Ch=sum(pars[n][0]*(p+R@pars[n][1]) for n,(p,R,w,v) in shapeh.items())/M
  positions={n:Rnew@(p+R@pars[n][1]-Cn) for n,(p,R,w,v) in shapen.items()};omegas={n:Rnew@(wn+w) for n,(p,R,w,v) in shapen.items()};inertias={n:Rnew@R@pars[n][2]@R.T@Rnew.T for n,(p,R,w,v) in shapen.items()};jointpoints={ch:Rnew@(p-Cn) for ch,(p,R,w,v) in shapen.items()};dyn.append((positions,omegas,inertias,jointpoints,Rnew.copy()))
  wheel_gain=[float((Rnew@(shapen[js[k][2]][0]-Cn))[2]-(Rhold@(shapeh[js[k][2]][0]-Ch))[2]) for k in wheels]
  row={'left_axle_height_gain_vs_hold_m':wheel_gain[0],'right_axle_height_gain_vs_hold_m':wheel_gain[1],'run':name,'ms_after_conditional_retract':j,'left_distance_m':distance(qt,0),'right_distance_m':distance(qt,1),'predicted_pitch_rate':pn,'matched_hold_rate':ph,'delta_rate':pn-ph,'predicted_pitch_deg':-math.degrees(Rot.from_matrix(Rnew).as_euler('xyz')[0]),'hold_pitch_deg':-math.degrees(Rot.from_matrix(Rhold).as_euler('xyz')[0]),**{g+'_rate_term':v for g,v in contrib.items()}};sample.append(row);curve.append(row)
  if j<40:Rnew=Rnew@Rot.from_rotvec(wn*.001).as_matrix();Rhold=Rhold@Rot.from_rotvec(wh*.001).as_matrix()
 # Historical capture target, preserve landing construction (x from archived EXTEND only).
 source=(W/'src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp').read_text();import re
 cad=float(re.search(r'kCadWheelToHipLongitudinal = ([-0-9.]+)',source)[1]);p1,p2=[math.atan2(*map(float,re.search(r'const double '+n+r' = std::atan2\(([-0-9.]+), ([-0-9.]+)\)',source).groups())) for n in ['phi1_0','phi2_0']];x=float(ex['landing_target_x']);ps=(W/'src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp').read_text();l1=float(re.search(r'double l1 = ([0-9.]+)',ps)[1]);l2=float(re.search(r'double l2 = ([0-9.]+)',ps)[1]);runtime=yaml.safe_load((d/'runtime_parameters.yaml').read_text())['/bbot_velocity_jump_controller']['ros__parameters'];balance=float(re.search(r'balance_offset_ = ([0-9.]+);',source)[1]);height=float(re.search(r'L_TOUCH_ = ([0-9.]+);',source)[1]);offset=float(re.search(r'kHipBodyVerticalOffset = ([0-9.]+)',source)[1]);radius=float(re.search(r'double wheel_radius = ([0-9.]+)',ps)[1]);dy=cad-x;down=height-offset-radius;d2=dy*dy+down*down;gamma=math.acos((l1*l1+l2*l2-d2)/(2*l1*l2));psi=math.acos((l1*l1+d2-l2*l2)/(2*l1*math.sqrt(d2)));landq=qf.copy();landq[legs]=[math.atan2(dy,down)-psi-p1+balance+runtime['flight_landing_pitch_bias'],math.pi-gamma-(p2-p1)]*2
 deploy_min=max([.09]+[max(1.875*abs(landq[i]-qf[i])/(11 if k%2==0 else 13),math.sqrt((10/math.sqrt(3))*abs(landq[i]-qf[i])/(450 if k%2==0 else 500))) for k,i in enumerate(legs)]);deploy5=math.ceil(deploy_min/.005)*.005
 # Newton-Euler about each joint, system COM ballistic: uniform gravity cancels.
 model_tau={}
 pcdd={n:np.gradient(np.gradient(np.array([x[0][n] for x in dyn]),.001,axis=0,edge_order=2),.001,axis=0,edge_order=2) for n in pars};alpha={n:np.gradient(np.array([x[1][n] for x in dyn]),.001,axis=0,edge_order=2) for n in pars}
 for i in legs:
  j=js[i];children={j[2]};changed=True
  while changed:
   oldsize=len(children);children.update(x[2] for x in js if x[1] in children);changed=len(children)!=oldsize
  vals=[]
  for k in range(3,38):
   positions,omegas,inertias,jpoint,R=dyn[k];torque=sum(inertias[n]@alpha[n][k]+np.cross(omegas[n],inertias[n]@omegas[n])+np.cross(positions[n]-jpoint[j[2]],pars[n][0]*pcdd[n][k]) for n in children);vals.append(float((R[:,0])@torque))
  model_tau[j[0]]=max(map(abs,vals))
 remaining=m['first_touch_s']-float(ex['timestamp']);settle_hard=remaining-.040-deploy5-.055-.020;settle_before_apex=off+m['com_apex_ms']/1000-float(ex['timestamp'])-.040;settle_allow=math.floor(min(settle_hard,settle_before_apex)/.005)*.005
 deployment=[]
 for delay in [0.,settle_allow]:
  duration=math.floor((remaining-T-delay-.055)/.005)*.005
  deployment.append({'settle_delay_s':delay,'extend_duration_s':duration,'historical_touch_margin_s':remaining-T-delay-duration,'reference_peak_v_a':[peaks(landq[i]-qf[i],duration) for i in legs]})
 # Exact additive budget at midpoint with orientation held to its initial value.
 midq=q0+.5*dv;midv=v0+dv*(1.875/T);Ia,Ha,_=kinematics(q0,v0);Ib,Hbrel,_=kinematics(midq,midv);Hbody=Rb.T@H
 budget={'configuration':-float((Rb@(np.linalg.solve(Ib,Hbody-Ha)-np.linalg.solve(Ia,Hbody-Ha)))[0])}
 for group,inds in [('hip',legs[::2]),('knee',legs[1::2]),('wheel',wheels)]:
  va=np.zeros(6);vb=np.zeros(6);va[inds]=v0[inds];vb[inds]=midv[inds];_,ha,_=kinematics(q0,va);_,hb,_=kinematics(midq,vb);budget[group]=float((Rb@np.linalg.solve(Ib,hb-ha))[0])
 expected=-float((Rb@(np.linalg.solve(Ib,Hbody-Hbrel)-np.linalg.solve(Ia,Hbody-Ha)))[0]);budget['sum']=sum(budget.values());budget['identity_residual']=budget['sum']-expected
 rows=read(OLD/'analysis'/name/'flight.csv');changes=[];last=None
 for r in rows:
  state=(r['ctrl_state_name'],r['ctrl_flight_subphase'])
  if state!=last:changes.append({'t_ms':(int(r['sim_time_ns'])/1e9-off)*1000,'state':state});last=state
 # Logged EXTEND duration via console, where available; otherwise no invented value.
 logs=list(d.glob('*.log'));logtext='\n'.join(p.read_text(errors='replace') for p in logs)
 old_ext_durations=sorted(set(float(v) for v in re.findall(r'\[FLIGHT_PLAN v6\.14\] sub=EXTEND.*?T=[0-9.]+\+([0-9.]+)',logtext)))
 record={'archived_extend_durations_s':old_ext_durations,'model_retract_torque_peak_interior_Nm':model_tau,'hip_knee_momentum_ratio':ratio,'reference_axle_height_gain_vs_hold_m':[sample[-1]['left_axle_height_gain_vs_hold_m'],sample[-1]['right_axle_height_gain_vs_hold_m']],'midpoint_pitch_rate_change_budget_rad_s':budget,'conditional_deployment_cases':deployment,'run':name,'state_transitions':changes,'off_s':off,'touch_s':m['first_touch_s'],'apex_ms':m['com_apex_ms'],'old_extend_ms':m['extend_ms'],'actual_q_at_old_extend':actualq[legs].tolist(),'actual_v_at_old_extend':actualv[legs].tolist(),'conditional_settled_q0':q0[legs].tolist(),'candidate_qf':qf[legs].tolist(),'delta_q':dv[legs].tolist(),'distance_start_m':[distance(q0,k) for k in [0,1]],'distance_end_m':[distance(qf,k) for k in [0,1]],'reference_shorten_m':[distance(q0,k)-distance(qf,k) for k in [0,1]],'retract_T_s':T,'kinematic_min_5ms_s':Tmin5,'reference_peak_v_a':pk,'deploy_min_5ms_s':deploy5,'max_settle_time_with_75ms_landing_budget_s':settle_hard,'max_settle_before_historical_apex_s':settle_before_apex,'conditional_settle_budget_5ms_s':settle_allow,'earliest_extend_ms':m['extend_ms']+40,'latest_extend_ms':m['extend_ms']+1000*(settle_allow+T),'momentum_native_identity_error':float(np.linalg.norm(identity)),'scenario_momentum_residual':maxres,'delta_rate_min':min(r['delta_rate'] for r in sample),'delta_rate_max':max(r['delta_rate'] for r in sample),'delta_pitch_end_deg':sample[-1]['predicted_pitch_deg']-sample[-1]['hold_pitch_deg'],'hip_rate_peak_abs':max(abs(r['hip_rate_term']) for r in sample),'knee_rate_peak_abs':max(abs(r['knee_rate_term']) for r in sample),'wheel_rate_peak_abs':max(abs(r['wheel_rate_term']) for r in sample),'native_H_world':H.tolist(),'native_pitch_rate_at_old_extend':-float(ll['base_link'][4][0]),'conditional_rest_pitch_rate':sample[0]['predicted_pitch_rate']}
 results.append(record);axs[0].plot([r['ms_after_conditional_retract'] for r in sample],[1000*(sample[0]['left_distance_m']-r['left_distance_m']) for r in sample],label=name+' MODEL');axs[1].plot([r['ms_after_conditional_retract'] for r in sample],[r['delta_rate'] for r in sample],label=name+' MODEL vs hold');print(json.dumps(record,ensure_ascii=False),flush=True)
axs[0].set_ylabel('MODEL left-axis shortening mm');axs[1].set_ylabel('MODEL pitch-rate delta rad/s');axs[1].set_xlabel('ms after conditional matched/low-speed entry');
for ax in axs:ax.legend();ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(OUT/'conditional_model.png',dpi=140);(OUT/'summary.json').write_text(json.dumps(results,indent=2))
with (OUT/'conditional_model.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=curve[0].keys());w.writeheader();w.writerows(curve)
