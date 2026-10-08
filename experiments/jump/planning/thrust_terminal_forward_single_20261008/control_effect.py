"""Saved-record intervention audit; same-feedback counterfactual is algebra only."""
from pathlib import Path
import csv,json,bisect
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';A=D/'thrust_terminal_forward_single_20261008';O=A/'analysis'
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rr):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
def clip(v,lo,hi):return max(lo,min(hi,v))
info={};curves={}
for n,root,budget in [('T1',D/'thrust_height_032_ab_20261008',D/'thrust_momentum_budget_20261008/T1'),('R1',A,O/'thrust_budget/R1')]:
 cc=read(root/'physical_runs'/n/'velocity_log.csv');ss=read(budget/'steps.csv');sm=json.loads((budget/'summary.json').read_text());off=sm['off_s'];start=sm['THRUST_entry_first_log_s'];th=[r for r in cc if r['state_name']=='THRUST'];rel=next(r for r in th if r['thrust_release_active']=='1');release=float(rel['timestamp']);rows=[];prev_cf=None
 for r in th:
  t=float(r['timestamp']);vp=float(r['thrust_forward_speed_predicted']);base=float(r['jump_takeoff_forward_speed']);k=float(r['thrust_forward_velocity_kp']);active=r['thrust_release_active']=='1'
  mag0=clip(base+k*(base-vp),0,.85);target0=clip(-mag0+float(r['thrust_att_term'])+float(r['thrust_wheel_kinematics_applied_correction']),-1.5,1.5)
  # Replay baseline formula with THIS run's recorded feedback; not a baseline rollout.
  dt=.005 if prev_cf is None else t-float(rows[-1]['time_s'])
  if not active or prev_cf is None:cmdcf=float(r['cmd_x'])
  else:cmdcf=prev_cf+clip(target0-prev_cf,-8*max(dt,.001),16*max(dt,.001))
  prev_cf=cmdcf
  rows.append({'time_s':t,'thrust_ms':(t-start)*1000,'off_ms':(t-off)*1000,'release_ms':(t-release)*1000,'active':int(active),'vp':vp,'m_actual':-float(r['thrust_fwd_term']),'cmd':float(r['cmd_x']),'target':float(r['thrust_wheel_target']),'target_same_feedback_original':target0,'cmd_same_feedback_original_replay':cmdcf,'target_delta_same_feedback':float(r['thrust_wheel_target'])-target0,'cmd_delta_same_feedback_replay':float(r['cmd_x'])-cmdcf,'attitude':float(r['thrust_att_term']),'kinematics':float(r['thrust_wheel_kinematics_applied_correction'])})
 dump(O/(n+'_wheel_intervention.csv'),rows)
 ct=[float(r['timestamp']) for r in cc];phys=[]
 for s in ss:
  t=int(s['sim_time_ns'])/1e9;c=cc[max(0,bisect.bisect_right(ct,t)-1)]
  phys.append({'thrust_ms':float(s['thrust_ms']),'off_ms':float(s['off_ms']),'time_s':t,'actual_dq_times_radius':.07*.5*(float(s['link_004_joint_dq'])+float(s['link_007_joint_dq'])),'cmd':float(c['cmd_x']),'Fy':float(s['Fy']),'Fz':float(s['Fz']),'moment':float(s['moment_preCOM']),'H':float(s['H_pitch']),'vz':float(s['COM_vz'])})
 dump(O/(n+'_wheel_physics.csv'),phys);curves[n]=(rows,phys)
 def get(elapsed):
  s=next(r for r in ss if abs(float(r['thrust_ms'])-elapsed)<1e-6)
  return {k:float(s[k]) for k in ['H_pitch','pitch_deg','pitch_rate','COM_vy','COM_vz','link_002_joint_q','link_002_joint_dq','link_003_joint_q','link_003_joint_dq']}
 firstflight=next(r for r in cc if r['state_name']=='FLIGHT');last=th[-1]
 rr=[r for r in rows if r['active'] and r['time_s']<off]
 ph=[r for r in phys if release<r['time_s']<off]
 info[n]={'entry':get(0),'at_common_145ms_before_either_intervention':get(145),'release_thrust_ms':(release-start)*1000,'release_off_ms':(release-off)*1000,'release_contact_duration_ms':(off-release)*1000,'release_contact_commands':rr,'physical_wheel_at_release_and_after':ph,'last_THRUST_cmd':float(last['cmd_x']),'first_FLIGHT_cmd':float(firstflight['cmd_x']),'first_FLIGHT_baseline':float(firstflight['air_wheel_baseline']),'first_FLIGHT_step':float(firstflight['cmd_x'])-float(last['cmd_x'])}
 if n=='R1':
  affected=[r for r in rows if r['active'] and r['time_s']<off and abs(r['cmd_delta_same_feedback_replay'])>.001]
  info[n]['first_observable_command_change_vs_same_feedback_replay_s']=affected[0]['time_s'] if affected else None
  info[n]['first_non_slew_clipped_contact_command_s']=next((r['time_s'] for r in rr if abs(r['cmd']-r['target'])<1e-5),None)
  if affected:
   t=affected[0]['time_s'];info[n]['first_actual_wheel_changed_command_match_s']=next((r['time_s'] for r in ph if r['time_s']>t and abs(r['actual_dq_times_radius']-affected[0]['cmd'])<1e-5),None)
json.dump(info,(O/'control_effect.json').open('w'),indent=2)
fig,axs=plt.subplots(4,2,figsize=(14,13))
for n,(cc,ss) in curves.items():
 color='C0' if n=='T1' else 'C3'
 for col,axis in enumerate(['thrust_ms','off_ms']):
  x=[r[axis] for r in cc];p=[r[axis] for r in ss]
  axs[0,col].plot(x,[r['cmd'] for r in cc],color=color,label=n+' published wheel speed')
  axs[0,col].plot(p,[r['actual_dq_times_radius'] for r in ss],color=color,ls='--',label=n+' native relative dq*radius')
  if n=='R1':axs[0,col].plot(x,[r['cmd_same_feedback_original_replay'] for r in cc],color='k',ls=':',label='R1 same-feedback old formula (offline)')
  for i,k in [(1,'Fy'),(2,'moment'),(3,'H')]:axs[i,col].plot(p,[r[k] for r in ss],color=color,label=n+' measured')
  a=info[n]['release_thrust_ms' if col==0 else 'release_off_ms']
  for ax in axs[:,col]:ax.axvline(a,color=color,ls=':',alpha=.6)
for i in range(4):
 for col in range(2):
  ax=axs[i,col];ax.grid(alpha=.2);ax.legend(fontsize=7);ax.set_ylabel(['wheel speed m/s','ground forward Fy N','contact pitch moment Nm','total H kg m2/s'][i]);ax.set_xlabel('ms from THRUST entry' if col==0 else 'ms from real liftoff');ax.set_xlim((110,200) if col==0 else (-60,0))
fig.suptitle('ONE physical intervention; old-formula replay is not a physical counterfactual');fig.tight_layout();fig.savefig(O/'wheel_contact_intervention.png',dpi=130)
print('Saved control_effect.json; fixed-feedback replay explicitly separated from physics.')
