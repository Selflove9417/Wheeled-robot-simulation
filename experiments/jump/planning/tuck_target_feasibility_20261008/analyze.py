"""Readonly TUCK target/geometry bounds; no ROS, no parameter sweep or control writes."""
from pathlib import Path
import csv,json,math,re,hashlib,xml.etree.ElementTree as E
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';OLD=ROOT/'clearance_apex_ab_20261008';O=ROOT/'tuck_target_feasibility_20261008';P=W/'experiments/jump/planning/tuck_target_feasibility_20261008';summary=[];alltraj=[]
params_src=(W/'src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp').read_text();get=lambda n:float(re.search(r'double '+n+r' = ([0-9.]+)',params_src)[1]);l1=get('l1');l2=get('l2');radius=get('wheel_radius');controller_src=(W/'src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp').read_text()
# CAD constants read from the actual helper, not manually guessed geometry.
hipoff=float(re.search(r'kHipBodyVerticalOffset = ([0-9.]+)',controller_src)[1]);cad=float(re.search(r'kCadWheelToHipLongitudinal = ([-0-9.]+)',controller_src)[1]);phi=[]
for n in ['phi1_0','phi2_0']:
 a,b=re.search(r'const double '+n+r' = std::atan2\(([-0-9.]+), ([-0-9.]+)\)',controller_src).groups();phi.append(math.atan2(float(a),float(b)))
def ik(z,pitch):
 down=max(.1,min(.6,z-(hipoff+radius)));d2=cad*cad+down*down;d=math.sqrt(d2);cg=(l2*l2+l1*l1-d2)/(2*l2*l1);cp=(l2*l2+d2-l1*l1)/(2*l2*d)
 if abs(cg)>1 or abs(cp)>1:return None
 gamma=math.acos(cg);psi=math.acos(cp);return np.array([math.atan2(cad,down)-psi-phi[0]+pitch,math.pi-gamma-(phi[1]-phi[0])]*2)
def read(p):return list(csv.DictReader(p.open()))
def dump(p,a):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=a[0].keys());w.writeheader();w.writerows(a)
def poly(q0,v0,qf,T):
 return np.array([q0,v0*T,0.,10*(qf-q0)-6*v0*T,-15*(qf-q0)+8*v0*T,6*(qf-q0)-3*v0*T])
def peaks(q0,v0,qf,T):
 a=poly(q0,v0,qf,T);out=[]
 for order in [0,1,2]:
  b=np.polynomial.polynomial.polyder(a,order)/T**order;der=np.polynomial.polynomial.polyder(b);rr=np.polynomial.polynomial.polyroots(der);ts=[0.,1.]+[float(x.real) for x in rr if abs(x.imag)<1e-9 and 0<x.real<1];out.append(max(abs(np.polynomial.polynomial.polyval(ts,b))))
 return out
def end_interval(q,v,T,limits):
 u=np.linspace(0,1,257);h=u-6*u**3+8*u**4-3*u**5;s=10*u**3-15*u**4+6*u**5;dh=1-18*u*u+32*u**3-15*u**4;ds=30*u*u-60*u**3+30*u**4;ddh=-36*u+96*u*u-60*u**3;dds=60*u-180*u*u+120*u**3;lo=-np.inf;hi=np.inf
 for base,coef,limit in [(q+v*T*h,s,limits[0]),(v*dh,ds/T,limits[1]),(v*ddh/T,dds/T**2,limits[2])]:
  for b,c in zip(base,coef):
   if abs(c)<1e-12:
    if abs(b)>limit+1e-10:return None
   else:
    aa=(-limit-b)/c;bb=(limit-b)/c;lo=max(lo,min(aa,bb));hi=min(hi,max(aa,bb))
 if lo>hi:return None
 bounds=[q+lo,q+hi]
 fun=lambda end:max(pk/limit for pk,limit in zip(peaks(q,v,end,T),limits))-1
 for j,edge in enumerate(bounds):
  if fun(edge)>1e-10:bounds[j]=brentq(fun,min(edge,q),max(edge,q),xtol=1e-13)
 return bounds
fig,axs=plt.subplots(3,2,figsize=(13,12),sharex=True)
for idx,name in enumerate(['B1','B2','B3']):
 d=OLD/'physical'/name;runtime=yaml.safe_load((d/'runtime_parameters.yaml').read_text())['/bbot_velocity_jump_controller']['ros__parameters'];balance=float(re.search(r'balance_offset_ = ([0-9.]+);',controller_src)[1]);m=json.loads((OLD/'analysis'/name/'metrics.json').read_text());tu=m['tuck_entry'];ex=m['extend_entry'];tt=float(tu['timestamp']);te=float(ex['timestamp']);q0=np.array([float(tu[k+'_pos_'+s]) for s in ['left','right'] for k in ['hip','knee']]);v0=np.array([float(tu[k+'_vel_'+s]) for s in ['left','right'] for k in ['hip','knee']]);T=float(runtime['flight_tuck_nominal_duration']);pitch=max(-.24,min(.24,float(tu['pitch'])-balance));requested=ik(.66,pitch);bound=q0+np.clip(requested-q0,-np.array([.14,.04,.14,.04]),np.array([.14,.04,.14,.04]));guard=(abs(v0)>.5)&((bound-q0)*v0<0);qf=np.where(guard,q0,bound);model=E.parse(d/'actual_robot.urdf').getroot();joint={j.attrib['name']:j for j in model.findall('joint')};pairs=[('link_002_joint','link_003_joint','link_004_joint'),('link_005_joint','link_006_joint','link_007_joint')]
 origins={n:np.array([float(x) for x in joint[n].find('origin').attrib['xyz'].split()]) for n in joint};assert all(joint[n].find('origin').attrib['rpy']=='0 0 0' and joint[n].find('axis').attrib['xyz']=='1 0 0' for pp in pairs for n in pp)
 def distance(q,side,planar=False):
  h,k,w=pairs[side];v=Rotation.from_rotvec([q[2*side],0,0]).apply(origins[k])+Rotation.from_rotvec([q[2*side]+q[2*side+1],0,0]).apply(origins[w]);return float(np.linalg.norm(v[1:] if planar else v))
 rows=read(OLD/'analysis'/name/'flight.csv');traj=[]
 for r in rows:
  ts=int(r['sim_time_ns'])/1e9
  if tt-.015<=ts<=te+.015:
   qa=np.array([float(r[n+'_q']) for pp in pairs for n in pp[:2]]);qr=np.array([float(r['ctrl_'+k+'_pos_cmd_'+s]) for s in ['left','right'] for k in ['hip','knee']]);traj.append({'sim_time_ns':int(r['sim_time_ns']),'tuck_ms':(ts-tt)*1000,'left_actual_distance':distance(qa,0),'right_actual_distance':distance(qa,1),'left_reference_distance':distance(qr,0),'right_reference_distance':distance(qr,1),'phase':r['ctrl_flight_subphase']});alltraj.append({'run':name,**traj[-1]})
 # Independent native pose check at the controller TUCK and EXTEND times.
 target_ns={round(tt*1e9),round(te*1e9)};native={}
 for r in csv.DictReader((d/'engine_frames.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>max(target_ns):break
  if ns in target_ns and r['phase']=='after_step' and r['entity_type']=='link':native.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
 native_distance={}
 for ns,rr in native.items():
  b=rr['base_link'];Q=Rotation.from_quat([float(b['quat_'+k]) for k in ['x','y','z','w']]);bp=np.array([float(b['position_'+k]) for k in 'xyz']);native_distance[ns]=[]
  for h,k,w in pairs:
   hip=bp+Q.apply(origins[h]);wheel=np.array([float(rr[joint[w].find('child').attrib['link']]['position_'+a]) for a in 'xyz']);native_distance[ns].append(float(np.linalg.norm(wheel-hip)))
 limits=[]
 for i,n in enumerate([x for pp in pairs for x in pp[:2]]):
  physical=min(abs(float(joint[n].find('limit').attrib[a])) for a in ['lower','upper']);part='hip' if i%2==0 else 'knee';limits.append((min(physical,float(runtime['flight_'+part+'_pos_limit'])),float(runtime['flight_'+part+'_speed_limit']),float(runtime['flight_'+part+'_acc_limit'])))
 intervals=[end_interval(q,v,T,lim) for q,v,lim in zip(q0,v0,limits)];ideal_peaks=[peaks(q,v,end,T) for q,v,end in zip(q0,v0,requested)];nominal_peaks=[peaks(q,v,end,T) for q,v,end in zip(q0,v0,qf)]
 # Continuous IK image of each endpoint interval. Bisection finds boundaries,
 # not a discrete parameter/physical sweep. z domain is the helper's unsaturated range.
 zlo=hipoff+radius+.1;zhi=hipoff+radius+.6
 for i,iv in enumerate(intervals):
  assert iv is not None
  for edge,lower in [(iv[0],True),(iv[1],False)]:
   f=lambda z:ik(z,pitch)[i]-edge
   a=f(zlo);b=f(zhi)
   if a*b<0:
    root=brentq(f,zlo,zhi);increasing=b>a
    if lower==increasing:zlo=max(zlo,root)
    else:zhi=min(zhi,root)
   elif (lower and max(a,b)<0) or (not lower and min(a,b)>0):zhi=zlo-1;break
 # For knee q increases as target_z decreases: shortening in both legs needs z below each q0-matching height.
 same=[]
 for i in [1,3]:
  f=lambda z:ik(z,pitch)[i]-q0[i];same.append(brentq(f,.24,.74))
 short_hi=min([zhi]+same);ideal_interval=[zlo,short_hi] if zlo<=short_hi else None
 # Minimum duration for current unguarded 0.66 IK, under unchanged source limits.
 feasible_T=None
 for steps in range(120):
  duration=.06+.005*steps
  if all(all(pk<=lim+1e-8 for pk,lim in zip(peaks(q,v,end,duration),limits[i])) for i,(q,v,end) in enumerate(zip(q0,v0,requested))):feasible_T=duration;break
 stream=read(OLD/'analysis'/name/'flight_inputs.csv');in_tuck=[r for r in stream if tt<=int(r['sim_time_ns'])/1e9<te];torques={n:max(abs(float(r['before_physics_joint_force_cmd_sim_input'])) for r in in_tuck if r['joint_name']==n and r['before_physics_joint_force_cmd_valid']=='1') for n in [x for pp in pairs for x in pp[:2]]}
 actualend=next(r for r in traj if r['sim_time_ns']==round(te*1e9));actualstart=next(r for r in traj if r['sim_time_ns']==round(tt*1e9));geometry_error=max(abs(native_distance[ns][side]-row[side_name+'_actual_distance']) for ns,row in [(round(tt*1e9),actualstart),(round(te*1e9),actualend)] for side,side_name in enumerate(['left','right']));assert geometry_error<1e-10
 # Same historical landing goal, conditional endpoint-only time estimate.
 down=.69-(hipoff+radius);x=max(-.12,min(.14,float(ex['landing_target_x'])));dy=cad-x;d2=dy*dy+down*down;gamma=math.acos((l2*l2+l1*l1-d2)/(2*l2*l1));psi=math.acos((l2*l2+d2-l1*l1)/(2*l2*math.sqrt(d2)));landing_pitch=balance+float(runtime['flight_landing_pitch_bias']);land=np.array([math.atan2(dy,down)-psi-phi[0]+landing_pitch,math.pi-gamma-(phi[1]-phi[0])]*2);minext=.09
 for i,delta in enumerate(abs(land-requested)):
  minext=max(minext,1.875*delta/limits[i][1],math.sqrt((10/math.sqrt(3))*delta/limits[i][2]))
 minext=math.ceil(minext/.005)*.005
 record={'run':name,'T':T,'native_fk_distance_error':geometry_error,'conditional_deploy_min_duration_to_historical_goal_s':minext,'conditional_time_budget_after_margin_s':m['first_touch_s']-tt-float(runtime['landing_deploy_ready_margin'])-.020,'unguarded_combined_time_s':None if feasible_T is None else feasible_T+minext,'tuck_sim_s':tt,'extend_sim_s':te,'q0':q0.tolist(),'v0':v0.tolist(),'raw_ik_qf':requested.tolist(),'protected_qf':qf.tolist(),'guard_blocks':guard.tolist(),'qf_delta':(qf-q0).tolist(),'native_distance_at_entry_and_extend':native_distance,'actual_entry':actualstart,'actual_extend':actualend,'reference_distance_q0':[distance(q0,s) for s in [0,1]],'reference_distance_qf':[distance(qf,s) for s in [0,1]],'raw_ik_distance':[distance(requested,s) for s in [0,1]],'nominal_reference_peaks_q_v_a':nominal_peaks,'raw_ik_peaks_q_v_a_T60':ideal_peaks,'ideal_endpoint_intervals_T60':intervals,'unguarded_kinematic_shortening_target_z_interval_T60':ideal_interval,'unguarded_0p66_minimum_5ms_duration':feasible_T,'actual_tuck_effort_peak_Nm':torques,'true_flight_remaining_at_tuck_s':m['first_touch_s']-tt,'actual_post_tuck_to_touch_s':m['first_touch_s']-te,'true_com_apex_after_tuck_s':m['off_s']+m['com_apex_ms']/1000-tt,'boundary_reference_delta_q':[float(ex[k+'_pos_cmd_'+s])-qf[2*j+i] for j,s in enumerate(['left','right']) for i,k in enumerate(['hip','knee'])],'boundary_reference_delta_v':[float(ex[k+'_vel_cmd_'+s]) for s in ['left','right'] for k in ['hip','knee']]}
 summary.append(record);dump(O/(name+'_distance.csv'),traj)
 for s,side in enumerate(['left','right']):
  ax=axs[idx,s];ax.plot([r['tuck_ms'] for r in traj],[100*r[side+'_actual_distance'] for r in traj],label='native q + actual URDF');ax.plot([r['tuck_ms'] for r in traj],[100*r[side+'_reference_distance'] for r in traj],label='published qref + actual URDF',ls='--');ax.axvline(T*1000,color='gray',ls=':',label='planned TUCK end');ax.axvline((te-tt)*1000,color='black',ls=':',label='EXTEND actual entry');ax.set_ylabel(name+' '+side+' axis distance cm');ax.grid(alpha=.2);ax.legend(fontsize=7)
 print(name,'guard',guard.tolist(),'actual distance change cm',100*(actualend['left_actual_distance']-actualstart['left_actual_distance']),'rawIK acc',[round(p[2],1) for p in ideal_peaks],'Tmin',feasible_T,'ideal60',ideal_interval,flush=True)
for ax in axs[-1]:ax.set_xlabel('ms from TUCK entry; reference vs measured-state geometry')
fig.tight_layout();fig.savefig(O/'distance_comparison.png',dpi=130);(O/'summary.json').write_text(json.dumps(summary,indent=2));dump(O/'distances.csv',alltraj)
files=[W/'src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp',W/'src/bbot_balance_controller/include/bbot_balance_controller/flight_trajectory.hpp',W/'src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp']
(O/'source_sha256.json').write_text(json.dumps({str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
