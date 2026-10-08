from pathlib import Path
import json,csv,re,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');R=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008';P=W/'experiments/jump/planning/clearance_apex_ab_20261008';O=R/'analysis';names=['B1','T1','B2','T2','B3','T3'];S=[];B=[];A={}
def read(p):return list(csv.DictReader(p.open()))
for n in names:
 p=O/n/'metrics.json'
 if not p.exists():continue
 m=json.loads(p.read_text());S.append(m);a=read(O/n/'flight.csv');A[n]=a;ctrl=read(R/'physical'/n/'velocity_log.csv');tu=m['tuck_entry'];tt=float(tu['timestamp']);log=(R/'physical'/n/'launch.log').read_text();mm=re.search(r'FEASIBLE_TUCK[^\n]*T=([0-9.]+)\+([0-9.]+)',log);T=float(mm[1]);m['tuck_nominal_plan_s']=T;m['tuck_actual_ms']=m['extend_ms']-m['tuck_ms'];m['extend_log']=re.search(r'\[TUCK->EXTEND\][^\n]*',log)[0];seg=[r for r in ctrl if r['state_name']=='FLIGHT' and r['flight_subphase']=='1'];
 for part in ['hip','knee']:
  for side in ['left','right']:
   q0=float(tu[part+'_pos_'+side]);v0=float(tu[part+'_vel_'+side]);ps=[];ys=[]
   for r in seg:
    u=np.clip((float(r['timestamp'])-tt)/T,0,1);h=u-6*u**3+8*u**4-3*u**5;s=10*u**3-15*u**4+6*u**5
    if s>1e-5:ps.append(s);ys.append(float(r[part+'_pos_cmd_'+side])-q0-v0*T*h)
   delta=float(np.dot(ps,ys)/np.dot(ps,ps));qf=q0+delta;pred=[]
   for r in seg:
    u=np.clip((float(r['timestamp'])-tt)/T,0,1);pred.append(q0+v0*T*(u-6*u**3+8*u**4-3*u**5)+delta*(10*u**3-15*u**4+6*u**5))
   B.append({'run':n,'joint':part+'_'+side,'q0':q0,'v0':v0,'qf_reconstructed_from_recorded_reference':qf,'qf_minus_q0':delta,'terminal_v':0.0,'duration_s':T,'max_reference_reconstruction_error_rad':max(abs(q-float(r[part+'_pos_cmd_'+side])) for q,r in zip(pred,seg)),'actual_position_at_extend':float(m['extend_entry'][part+'_pos_'+side]),'actual_velocity_at_extend':float(m['extend_entry'][part+'_vel_'+side]),'tracking_error_at_extend_vs_tuck_qf':float(m['extend_entry'][part+'_pos_'+side])-qf})
(R/'metrics.json').write_text(json.dumps(S,indent=2))
with (R/'tuck_reference_audit.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=B[0].keys());w.writeheader();w.writerows(B)
fields=['com_rise_from_off_m','bilateral_clearance_m','contact_pitch_deg','contact_rate','max_backward_pitch_deg','max_backward_axle_m','final_axle_displacement_m','attitude_recovery_start_s','full_stability_start_s','balance_reentry_s'];groups={}
for g in ['baseline','test']:
 ss=[m for m in S if m['group']==g];groups[g]={'n':len(ss)}
 for k in fields:
  vals=[m[k] for m in ss];valid=[v for v in vals if v is not None];groups[g][k]={'values':vals,'mean':None if len(valid)!=len(vals) else float(np.mean(valid)),'min':None if not valid else min(valid),'max':None if not valid else max(valid)}
(R/'group_summary.json').write_text(json.dumps(groups,indent=2))
fig,axes=plt.subplots(4,1,figsize=(12,12),sharex=True)
for m in S:
 a=A[m['run']];t=[float(r['off_ms']) for r in a];style='-' if m['group']=='baseline' else '--'
 for ax,k,mult in zip(axes,['bilateral_clearance','com_z','pitch_deg','pitch_rate'],[100,100,1,1]):ax.plot(t,[mult*(float(r[k])-(float(a[0][k]) if k=='com_z' else 0)) for r in a],ls=style,label=m['run']+' '+m['group'])
for ax,label in zip(axes,['bilateral bottom clearance cm','COM rise from takeoff cm','native pitch deg','native pitch rate rad/s']):ax.set_ylabel(label);ax.legend(ncol=3,fontsize=8);ax.grid(alpha=.2)
axes[-1].set_xlabel('ms from sustained no-contact native frame');fig.tight_layout();fig.savefig(R/'comparison_flight.png',dpi=130);plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
for m in S:
 a=read(O/m['run']/'post_contact.csv');t=[float(r['contact_s']) for r in a]
 for ax,k,scale in zip(axes,['pitch_deg','pitch_rate','axle_displacement_m'],[1,1,100]):ax.plot(t,[scale*float(r[k]) for r in a],ls='-' if m['group']=='baseline' else '--',label=m['run'])
for ax,label in zip(axes,['native pitch deg','native pitch rate rad/s','axle displacement from true contact cm']):ax.set_ylabel(label);ax.legend(ncol=6,fontsize=8);ax.grid(alpha=.2)
axes[-1].set_xlabel('seconds from true native first contact');fig.tight_layout();fig.savefig(R/'comparison_landing.png',dpi=130);plt.close(fig)
print(json.dumps({m['run']:{k:m[k] for k in ['bilateral_clearance_m','com_rise_from_off_m','contact_pitch_deg','contact_rate','max_backward_axle_m','full_stability_start_s','emergency','secondary_flight']} for m in S},indent=2))
