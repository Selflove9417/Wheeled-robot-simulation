#!/usr/bin/env python3
"""Read-only boundary audit of the seven existing full physical trials."""
from pathlib import Path
import csv,json,re,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');P=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007';O=Path(__file__).resolve().parent
prev=json.loads((O.parent/'landing_capture_pitch_chain_20261007/summary.json').read_text()); rows=[];summary=[]
keys=[('hip_left','link_002_joint'),('knee_left','link_003_joint'),('hip_right','link_005_joint'),('knee_right','link_006_joint')]
def f(r,k):return float(r[k])
for d in prev:
 n=d['run'];rr=list(csv.DictReader((P/n/'velocity_log.csv').open()));tu=next(r for r in rr if r['state_name']=='FLIGHT' and r['flight_subphase']=='1');ex=next(r for r in rr if r['state_name']=='FLIGHT' and r['flight_subphase']=='2');te=f(ex,'timestamp');tt=f(tu,'timestamp');last=rr[rr.index(ex)-1]
 log=(P/n/'launch.log').read_text();m=re.search(r'FEASIBLE_TUCK[^\n]*T=([0-9.]+)\+([0-9.]+)',log);T=float(m[1]);em=re.search(r'FLIGHT_PLAN[^\n]*sub=EXTEND[^\n]*T=([0-9.]+)\+([0-9.]+)',log);ET=float(em[2]);nrows=[]
 for k,j in keys:
  part,side=k.split('_');q=part+'_pos_'+side;v=part+'_vel_'+side;qc=part+'_pos_cmd_'+side;vc=part+'_vel_cmd_'+side
  q0=f(tu,q);v0=f(tu,v);z=.66-.14;dy=-.01137221;aa=.3;bb=.34325;dist=math.hypot(dy,z);ga=math.acos((aa*aa+bb*bb-dist*dist)/(2*aa*bb));ps=math.acos((aa*aa+dist*dist-bb*bb)/(2*aa*dist));requested=(math.atan2(dy,z)-ps-math.atan2(-.29348091,.06220095)+max(-.24,min(.24,f(tu,'pitch')-.03))) if part=='hip' else math.pi-ga-(math.atan2(.28210870,.19553796)-math.atan2(-.29348091,.06220095));bounded=q0+max(-(.14 if part=='hip' else .04),min((.14 if part=='hip' else .04),requested-q0));qf=q0 if abs(v0)>.5 and (bounded-q0)*v0<0 else bounded;assert abs(qf-q0)<1e-10,(n,k,qf,q0);u=min(1.,max(0.,(te-tt)/T));a=[q0,v0*T,0.,-6*v0*T,8*v0*T,-3*v0*T] # qf=q0, af=a0=vf=0
  qe=sum(a[i]*u**i for i in range(6));ve=sum(i*a[i]*u**(i-1) for i in range(1,6))/T
  native=d['milestones']['extend_entry']['native_joints'][j]
  x={'run':n,'joint':k,'gain':d['gain'],'extend_sim_s':te,'tuck_duration_s':T,'extend_duration_s':ET,'extend_qf':d['direct_gain_geometry']['ik_if_1p6' if d['gain']==1.6 else 'ik_if_1p0'][0 if part=='hip' else 1],'extend_vf':0.,'tuck_elapsed_s':te-tt,'tuck_q0':q0,'tuck_v0':v0,'tuck_qf':q0,'tuck_vf':0.,'tuck_requested_ik':requested,'old_tuck_q_at_boundary':qe,'old_tuck_v_at_boundary':ve,'controller_measured_q':f(ex,q),'controller_measured_v':f(ex,v),'native_q_at_boundary':native['q'],'native_v_at_boundary':native['v'],'extend_published_q0':f(ex,qc),'extend_published_v0':f(ex,vc),'reference_delta_q':f(ex,qc)-qe,'reference_delta_v':f(ex,vc)-ve,'extend_minus_measurement_q':f(ex,qc)-f(ex,q),'extend_minus_measurement_v':f(ex,vc)-f(ex,v),'last_published_t_rel':f(last,'timestamp')-te,'last_published_q':f(last,qc),'last_published_v':f(last,vc)}
  nrows.append(x);rows.append(x)
 # first sign reversal in TUCK target and native recorded velocity, without filtering failure cases
 rev={}
 for k,j in keys:
  part,side=k.split('_');v=part+'_vel_cmd_'+side;sgn=math.copysign(1,f(tu,v));hit=next((r for r in rr if tt<=f(r,'timestamp')<=te and sgn*f(r,v)<-1e-5),None);rev[k]=None if hit is None else f(hit,'timestamp')-te
 summary.append({'run':n,'extend_sim_s':te,'tuck_start_rel_s':tt-te,'rearward_onset_rel_s':d['backturn_onset']['sim_s']-te,'rearward_onset_phase':d['backturn_onset']['subphase'],'target_reversal_rel_s':rev,'boundary':nrows})
 cr=[r for r in rr if -.15<=f(r,'timestamp')-te<=.15];ct=np.array([f(r,'timestamp')-te for r in cr]);src=O.parent/'landing_capture_pitch_chain_20261007'/n;geom=list(csv.DictReader((src/'geometry_window.csv').open()));native=list(csv.DictReader((src/'native_window.csv').open()));nr={j:{} for _,j in keys};nr.update({j:{} for j in ['link_004_joint','link_007_joint']});rate={}
 for r in native:
  t=int(r['sim_time_ns'])*1e-9-te
  if -.15<=t<=.15:
   nr[r['joint_name']][t]=r
   if r['joint_index']=='0':rate[t]=-float(r['post_base_world_wx'])
 ts=sorted(rate);fig,ax=plt.subplots(5,1,figsize=(12,13),sharex=True)
 for k,j in keys:
  part,side=k.split('_');v=part+'_vel_cmd_'+side;ax[0].plot(ct,[f(r,v) if r['state_name']=='FLIGHT' else math.nan for r in cr],label=k+' target',linestyle='--');ax[0].plot(ts,[float(nr[j][t]['joint_velocity']) for t in ts],label=k+' native')
 gp=[];gt=[]
 for g in geom:
  t=int(g['sim_time_ns'])*1e-9-te
  if -.15<=t<=.15:
   x,y,z,w=[float(g['base_'+k]) for k in ['qx','qy','qz','qw']];gp.append(-math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))));gt.append(t)
 ax[1].plot(gt,gp,label='native pitch deg');ax[2].plot(ts,[rate[t] for t in ts],label='native pitch_rate rad/s');ax[3].plot(ct,[f(r,'cmd_x') for r in cr],label='published wheel cmd m/s')
 for j in ['link_004_joint','link_007_joint']:ax[3].plot(ts,[.07*float(nr[j][t]['joint_velocity']) for t in ts],label=j+' native velocity * R m/s')
 names=list(dict.fromkeys(r['state_name']+'/'+r['flight_subphase'] for r in cr));ax[4].step(ct,[names.index(r['state_name']+'/'+r['flight_subphase']) for r in cr],where='post');ax[4].set_yticks(range(len(names)),names)
 for a in ax:a.axvline(0,color='k',linestyle=':');a.grid(alpha=.2)
 for a in ax[:4]:a.legend(fontsize=7,ncol=2)
 ax[0].set_ylabel('rad/s');ax[4].set_xlabel('seconds from EXTEND entry');fig.suptitle(n+' recorded Gazebo, gain='+str(d['gain']));fig.tight_layout();fig.savefig(O/(n+'.png'),dpi=120);plt.close(fig)
with (O/'boundary.csv').open('w') as s:
 c=csv.DictWriter(s,fieldnames=rows[0].keys());c.writeheader();c.writerows(rows)
(O/'summary.json').write_text(json.dumps(summary,indent=2))
for d in summary:
 print(d['run'],'TUCK',round(d['tuck_start_rel_s'],3),'rear onset',round(d['rearward_onset_rel_s'],3),'reversal',d['target_reversal_rel_s'])
 for r in d['boundary']:print(r['joint'],'dq',round(r['reference_delta_q'],6),'dv',round(r['reference_delta_v'],6),'measurementdv',r['extend_minus_measurement_v'])
