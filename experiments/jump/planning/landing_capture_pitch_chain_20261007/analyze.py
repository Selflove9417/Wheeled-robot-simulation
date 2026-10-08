#!/usr/bin/env python3
# Read-only analysis of the existing seven completed Gazebo trials.
import csv,json,math,bisect,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');P=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007';O=Path(__file__).resolve().parent
M=json.loads((P/'metrics.json').read_text());NAMES=[m['run'] for m in M]
PHASES={0:'ATTITUDE_ARREST',1:'TUCK',2:'EXTEND',3:'PROTECTIVE_DEPLOY'}
def f(r,k):
 try:return float(r[k])
 except (ValueError,KeyError):return math.nan
def pit(g):
 x,y,z,w=(f(g,'base_'+k) for k in ['qx','qy','qz','qw'])
 return -math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
def ik(x):
 # Exact algebra from default controller:3717-3765, params from robot_params.hpp.
 a=.3;b=.34325;z=.69-.14;dy=-.01137221-max(-.12,min(.14,x));d=math.hypot(dy,z)
 gamma=math.acos(np.clip((a*a+b*b-d*d)/(2*a*b),-1,1));psi=math.acos(np.clip((a*a+d*d-b*b)/(2*a*d),-1,1));ph1=math.atan2(dy,z)-psi
 return [ph1-math.atan2(-.29348091,.06220095)+.085,math.pi-gamma-(math.atan2(.28210870,.19553796)-math.atan2(-.29348091,.06220095))]
def write_rows(path,rows):
 with path.open('w') as s:
  c=csv.DictWriter(s,fieldnames=list(rows[0]));c.writeheader();c.writerows(rows)
results=[];allcurves={}
for m in M:
 name=m['run'];d=P/name;out=O/name;out.mkdir(exist_ok=True);td=m['native_first_contact_s'];tdns=round(td*1e9);lo=tdns-500_000_000;hi=tdns+300_000_000
 rr=list(csv.DictReader((d/'velocity_log.csv').open()));rs=[r for r in rr if -.5-1e-9<=f(r,'timestamp')-td<=.3+1e-9];rt=[f(r,'timestamp') for r in rr]
 def row_at(t):return rr[max(0,bisect.bisect_right(rt,t)-1)]
 def phase(r):return PHASES.get(int(f(r,'flight_subphase')),'UNKNOWN') if r['state_name']=='FLIGHT' else r['state_name']
 gg=[g for g in csv.DictReader((d/'geometry.csv').open()) if lo<=int(g['sim_time_ns'])<=hi];frames=[g for g in csv.DictReader((d/'ground_frames.csv').open()) if lo<=int(g['sim_time_ns'])<=hi];native=[]
 for r in csv.DictReader((d/'native_wrench.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>hi:break
  if ns>=lo:native.append(r)
 write_rows(out/'controller_window.csv',rs);write_rows(out/'geometry_window.csv',gg);write_rows(out/'contact_window.csv',frames);write_rows(out/'native_window.csv',native)
 joints={n:{} for n in ['link_002_joint','link_003_joint','link_004_joint','link_005_joint','link_006_joint','link_007_joint']};gyro={}
 for r in native:
  ns=int(r['sim_time_ns']);joints[r['joint_name']][ns]=r
  if r['joint_index']=='0' and r['post_base_velocity_valid']=='1':gyro[ns]=-f(r,'post_base_world_wx')
 ts=np.array([int(g['sim_time_ns'])*1e-9 for g in gg]);tt=ts-td;pp=np.array([pit(g) for g in gg]);pr=np.array([gyro.get(int(g['sim_time_ns']),math.nan) for g in gg]);assert np.isfinite(pr).all()
 phases=[phase(row_at(t)) for t in ts];flightentry=next(f(r,'timestamp') for r in rr if r['state_name']=='FLIGHT');tuck=next((f(r,'timestamp') for r in rr if r['state_name']=='FLIGHT' and int(f(r,'flight_subphase'))==1),None);extend=next((f(r,'timestamp') for r in rr if r['state_name']=='FLIGHT' and int(f(r,'flight_subphase'))==2),None)
 def snap(i):
  t=ts[i];ns=int(gg[i]['sim_time_ns']);c=row_at(t);j={n:{'q':f(joints[n][ns],'joint_position'),'v':f(joints[n][ns],'joint_velocity'),'before_input':f(joints[n][ns],'before_physics_joint_force_cmd_sim_input') if joints[n][ns]['before_physics_joint_force_cmd_valid']=='1' else None} for n in joints}
  return {'t_rel_s':float(t-td),'sim_s':float(t),'pitch_deg':math.degrees(pp[i]),'pitch_rate_rad_s':float(pr[i]),'state':c['state_name'],'subphase':phase(c),'native_num_contacts':int(next(fr['num_contacts'] for fr in frames if int(fr['sim_time_ns'])==ns)),'controller_command_timestamp':f(c,'timestamp'),'command':{k:f(c,k) for k in ['cmd_x','air_wheel_cmd_raw','landing_wheel_ground_blend','capture_world_target','hip_pos_cmd_left','knee_pos_cmd_left','hip_pos_cmd_right','knee_pos_cmd_right','hip_vel_cmd_left','knee_vel_cmd_left','hip_vel_cmd_right','knee_vel_cmd_right','actual_tau_hip_left','actual_tau_knee_left','landing_target_x']},'native_joints':j}
 # Reporting criterion only: >=1deg fall over next50ms and rate<-0.20 for15ms, over the whole requested window.
 onset=None
 for i in range(len(ts)-50):
  if ts[i+50]<td and np.all(pr[i:i+15]<-.20) and pp[i+50]-pp[i]<=-math.pi/180:
   onset=i;break
 cross=next((i for i in range(len(ts)-10) if ts[i]<td and np.all(pp[i:i+10]<0)),None)
 ss={'run':name,'gain':m['gain'],'first_contact_s':td,'flight_entry_rel_s':flightentry-td,'tuck_entry_rel_s':tuck-td if tuck else None,'extend_entry_rel_s':extend-td if extend else None,'onset_rule':'whole requested window: native rate<-0.20 continuously15ms and >=1deg native pitch fall over next50ms','backturn_onset':snap(onset) if onset is not None else None,'first_negative_pitch_10ms':snap(cross) if cross is not None else None}
 samples={}
 for key,t in [('flight_entry',flightentry),('tuck_entry',tuck),('extend_entry',extend),('contact_minus_50ms',td-.05),('native_contact',td)]:
  if t is not None:samples[key]=snap(int(np.argmin(abs(ts-t))))
 ss['milestones']=samples
 plan=next(r for r in rr if r['landing_capture_planned']=='1');vx=f(plan,'landing_capture_vx');vx_eff=vx if abs(vx)>=.08 else 0.;raw=vx_eff/math.sqrt(9.81/.4);x16=np.clip(.12-1.6*raw,-.12,.1);x10=np.clip(.12-raw,-.12,.1)
 ss['direct_gain_geometry']={'plan_t_rel_s':f(plan,'timestamp')-td,'v_plan':vx,'x_actual':f(plan,'landing_target_x'),'x_if_1p6':float(x16),'x_if_1p0':float(x10),'delta_x':float(x10-x16),'ik_if_1p6':ik(x16),'ik_if_1p0':ik(x10),'ik_delta':[b-a for a,b in zip(ik(x16),ik(x10))]}
 ss['phase_ranges']={}
 for ph in sorted(set(phases),key=phases.index):
  inds=np.where(np.array(phases)==ph)[0];ss['phase_ranges'][ph]={'start':float(tt[inds[0]]),'end':float(tt[inds[-1]]),'pitch_first_deg':math.degrees(pp[inds[0]]),'pitch_last_deg':math.degrees(pp[inds[-1]]),'rate_min':float(pr[inds].min()),'rate_max':float(pr[inds].max())}
 ss['wheel_limits']={}
 for label,start,end in [('pre',-.5,0.),('post',0.,.3)]:
  cr=[r for r in rs if start<=f(r,'timestamp')-td<end+1e-9];air=[r for r in cr if r['state_name']=='FLIGHT'];nr=[r for r in native if r['joint_name'] in ['link_004_joint','link_007_joint'] and start<=int(r['sim_time_ns'])*1e-9-td<end+1e-9]
  clipped=[f(r,'timestamp')-td for r in air if abs(f(r,'air_wheel_cmd_raw'))>=2.0];published_hit=[f(r,'timestamp')-td for r in air if abs(f(r,'cmd_x'))>=1.9999]
  ss['wheel_limits'][label]={'published_cmd_abs_max_mps':max(abs(f(r,'cmd_x')) for r in cr),'diff_drive_5mps_hits':sum(abs(f(r,'cmd_x'))>=4.9999 for r in cr),'air_raw_abs_max_mps':max([abs(f(r,'air_wheel_cmd_raw')) for r in air],default=None),'air_branch_clipped_samples':len(clipped),'air_branch_clip_times':clipped,'published_flight_at_or_above2_samples':len(published_hit),'native_wheel_velocity_abs_max':max(abs(f(r,'joint_velocity')) for r in nr),'native_over30_samples':sum(abs(f(r,'joint_velocity'))>30.0001 for r in nr),'native_wheel_force_input_valid':sum(r['before_physics_joint_force_cmd_valid']=='1' for r in nr),'native_wheel_samples':len(nr)}
 ss['actual_extend_inputs']={}
 if extend is not None:
  for n in ['link_002_joint','link_003_joint']:
   nr=[r for r in native if r['joint_name']==n and extend<=int(r['sim_time_ns'])*1e-9<td and phase(row_at(int(r['sim_time_ns'])*1e-9))=='EXTEND' and r['before_physics_joint_force_cmd_valid']=='1'];vs=[f(r,'before_physics_joint_force_cmd_sim_input') for r in nr];ss['actual_extend_inputs'][n]={'mean':float(np.mean(vs)),'min':min(vs),'max':max(vs),'impulse_Nms':sum(vs)*.001}
 results.append(ss);allcurves[name]=(tt,pp,pr)
 (out/'summary.json').write_text(json.dumps(ss,indent=2))
 # Per-run complete requested signal panel; native joint input/load remain distinct in raw slice.
 ct=np.array([f(r,'timestamp')-td for r in rs]);fig,axs=plt.subplots(9,1,figsize=(13,19),sharex=True)
 axs[0].plot(tt,np.degrees(pp),label='native pitch');axs[0].plot(ct,[math.degrees(f(r,'pitch')) for r in rs],label='controller IMU pitch');axs[0].set_ylabel('deg')
 axs[1].plot(tt,pr,label='native -post_wx');axs[1].plot(ct,[f(r,'pitch_rate') for r in rs],label='filtered pitch_rate');axs[1].set_ylabel('rad/s')
 for side,hi_,ki_,color in [('left','link_002_joint','link_003_joint','tab:blue'),('right','link_005_joint','link_006_joint','tab:orange')]:
  for part,n,style in [('hip',hi_,'-'),('knee',ki_,'--')]:
   axs[2].plot(tt,[f(joints[n][int(g['sim_time_ns'])],'joint_position') for g in gg],label=part+' '+side+' actual',color=color,linestyle=style)
   axs[2].plot(ct,[f(r,part+'_pos_cmd_'+side) for r in rs],label=part+' '+side+' target',linestyle=':',linewidth=1)
   axs[3].plot(tt,[f(joints[n][int(g['sim_time_ns'])],'joint_velocity') for g in gg],label=part+' '+side,color=color,linestyle=style)
 axs[2].set_ylabel('leg q rad');axs[3].set_ylabel('leg v rad/s')
 for n in ['link_004_joint','link_007_joint']:
  q0=f(joints[n][tdns],'joint_position');axs[4].plot(tt,[f(joints[n][int(g['sim_time_ns'])],'joint_position')-q0 for g in gg],label=n+' q - contact q')
  axs[5].plot(tt,[f(joints[n][int(g['sim_time_ns'])],'joint_velocity') for g in gg],label=n)
 axs[4].set_ylabel('wheel q rad');axs[5].set_ylabel('wheel v rad/s')
 for k in ['cmd_x','air_wheel_cmd_raw','capture_world_target']:axs[6].plot(ct,[f(r,k) for r in rs],label=k)
 axs[6].set_ylabel('wheel cmd m/s')
 for k in ['landing_target_x','landing_capture_offset','landing_capture_raw_offset']:axs[7].plot(ct,[f(r,k) for r in rs],label=k)
 axs[7].set_ylabel('plan m')
 names=list(dict.fromkeys(phases));axs[8].step(tt,[names.index(x) for x in phases],where='post',label='controller state/subphase');axs[8].set_yticks(range(len(names)),names)
 twin=axs[8].twinx();twin.plot([int(r['sim_time_ns'])*1e-9-td for r in frames],[int(r['num_contacts']) for r in frames],color='black',alpha=.5,label='native num_contacts');twin.set_ylabel('contacts')
 for ax in axs:ax.axvline(0,color='k',linestyle=':');ax.grid(alpha=.2);ax.legend(fontsize=7,ncol=3,loc='best')
 axs[-1].set_xlim(-.5,.3);axs[-1].set_xlabel('seconds relative to real native first wheel contact');fig.suptitle(name+' actual gain='+str(m['gain'])+' | recorded Gazebo signals; wheel output is velocity, not motor torque');fig.tight_layout();fig.savefig(out/'window.png',dpi=120);plt.close(fig)
 print(name,'onset',None if onset is None else (tt[onset],phases[onset],math.degrees(pp[onset]),pr[onset]),'cross',None if cross is None else (tt[cross],phases[cross]),'IKdelta',ss['direct_gain_geometry']['ik_delta'],'input',ss['actual_extend_inputs'],flush=True)
(O/'summary.json').write_text(json.dumps(results,indent=2))
fig,axs=plt.subplots(2,1,figsize=(12,8),sharex=True)
for m in results:
 name=m['run'];tt,pp,pr=allcurves[name];style={'A1':'-','B1':'--','A2':':','B2':'-.','T1':'-','T2':'--','T3':':'}[name];color='tab:blue' if m['gain']==1.6 else 'tab:red';axs[0].plot(tt,np.degrees(pp),label=name+' gain='+str(m['gain']),color=color,linestyle=style);axs[1].plot(tt,pr,color=color,linestyle=style)
for ax in axs:ax.grid(alpha=.25);ax.axvline(0,color='k',linestyle=':')
axs[0].legend(ncol=4,fontsize=8);axs[0].set_ylabel('native pitch deg');axs[1].set_ylabel('native pitch rate rad/s');axs[1].set_xlabel('seconds relative to real first wheel contact');axs[1].set_xlim(-.5,.3);fig.suptitle('Recorded Gazebo: rearward rotation begins before physical contact');fig.tight_layout();fig.savefig(O/'native_attitude.png',dpi=150);plt.close(fig)
