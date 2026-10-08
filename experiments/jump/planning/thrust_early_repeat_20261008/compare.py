"""Read-only source/physical comparison. No ROS or command output."""
from pathlib import Path
import csv,json,bisect,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';R=ROOT/'thrust_early_repeat_20261008';OLD=ROOT/'thrust_origin_diagnostic_20261007';P=W/'experiments/jump/planning/thrust_early_repeat_20261008'
def read(p):return list(csv.DictReader(p.open()))
def dump(p,a):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=a[0].keys());w.writeheader();w.writerows(a)
def val(r,k):return float(r[k])
S=[];A={};C={}
for name in ['B1','T1','B2','T2','B3','T3']+[f'R{i}' for i in range(1,7)]:
 old=name.startswith(('B','T'));base=OLD if old else R;d=base/name;a=read(d/'window.csv');e=json.loads((d/'events.json').read_text());entry=e['state_entry_anchor_s'];off=e['off_s'];ct=read((ROOT/'hip_momentum_ab_20261007/physical'/name if old else R/'physical'/name)/'velocity_log.csv');tcs=[float(r['timestamp']) for r in ct]
 for r in a:
  ns=int(r['sim_time_ns']);cr=ct[max(0,bisect.bisect_right(tcs,ns/1e9)-1)]
  r['imu_age_ms']=(ns/1e9-float(cr['imu_sample_stamp']))*1000
  r['published_age_ms']=(ns/1e9-float(cr['timestamp']))*1000
  for k in ['pitch_rate','pitch_rate_raw','fast_pitch_rate','imu_sample_stamp','thrust_gate_open_stamp','effort_switch_ack_stamp']:r['ctrl_'+k]=cr[k]
 zero=next(r for r in a if float(r['t_ms'])==0);at=lambda ms:min(a,key=lambda r:abs(float(r['thrust_ms'])-ms))
 ninput=read(d/'native_inputs.csv') if old else []
 if not old:
  ninput=[]
  for r in csv.DictReader((R/'physical'/name/'native_wrench.csv').open()):
   ns=int(r['sim_time_ns'])
   if ns>round(off*1e9)+20000000:break
   if ns>=round(entry*1e9)-20000000:ninput.append(r)
  dump(d/'native_inputs.csv',ninput)
 first=next(r for r in ninput if r['joint_index']=='0' and r['before_physics_joint_force_cmd_valid']=='1' and abs(float(r['before_physics_joint_force_cmd_sim_input']))>1e-8)
 firstpub=next(r for r in ct if float(r['timestamp'])>=entry and abs(float(r['actual_tau_hip_left']))>1e-8)
 stamp=next(r for r in ct if r['state_name']=='THRUST');gate=next((float(r['thrust_gate_open_stamp']) for r in ct if float(r['timestamp'])>=entry and float(r['thrust_gate_open_stamp'])>=entry),float('nan'))
 early=[r for r in a if 0<=float(r['thrust_ms'])<=50];pre=[r for r in a if float(r['thrust_ms'])>=0 and float(r['t_ms'])<=0];onset=next((float(r['thrust_ms']) for i,r in enumerate(pre) if float(r['pitch_rate'])<-.2 and all(float(x['pitch_rate'])<-.2 for x in pre[i:])),None)
 item={'run':name,'old':old,'entry_s':entry,'off_ms':1000*(off-entry),'off_rate':float(zero['pitch_rate']),'off_pitch_deg':float(zero['pitch_deg']),'early_peak_rate':max(float(r['pitch_rate']) for r in early),'continuous_negative_onset_ms':onset,'request_ms':1000*(e['effort_request_s']-entry),'ack_ms':1000*(float(stamp['effort_switch_ack_stamp'])-entry),'first_thrust_log_ms':1000*(e['first_thrust_logged_s']-entry),'first_publish_ms':1000*(float(firstpub['timestamp'])-entry),'first_actual_ms':int(first['sim_time_ns'])/1e6-entry*1000,'pub_to_actual_ms':int(first['sim_time_ns'])/1e6-float(firstpub['timestamp'])*1000,'gate_ms':1000*(gate-entry),'entry':at(0),'at29':at(29),'at50':at(50),'off':zero,'budgets':json.loads((d/'budgets.json').read_text())}
 stamps=sorted(set(float(r['imu_sample_stamp']) for r in ct if entry-.02<=float(r['timestamp'])<=off));item['imu_period_ms']=np.diff(stamps).tolist();item['imu_period_ms']=[1000*x for x in item['imu_period_ms']];item['imu_age_max_ms']=max(float(r['imu_age_ms']) for r in pre);item['imu_age_median_ms']=float(np.median([float(r['imu_age_ms']) for r in pre]));item['early_fast_inactive']=all(float(r['ctrl_thrust_release_fast_rate_comp_applied'])==0 for r in pre if float(r['thrust_ms'])<=29)
 item['early29_budget']={k:float(at(29)[k])-float(at(0)[k]) for k in ['pitch_rate','total_H_pitch','locked_body_rate_term','link_002_joint_body_rate_term','link_003_joint_body_rate_term','link_004_joint_body_rate_term','link_005_joint_body_rate_term','link_006_joint_body_rate_term','link_007_joint_body_rate_term']}
 if not old:item['contact']=json.loads((d/'contact_summary.json').read_text());C[name]=read(d/'contact_budget.csv')
 A[name]=a;S.append(item)
(R/'comparison_summary.json').write_text(json.dumps(S,indent=2))
for group,names in [('old',['B1','T1','B2','T2','B3','T3']),('repeat',[f'R{i}' for i in range(1,7)])]:
 fig,axes=plt.subplots(8,1,figsize=(13,19),sharex=True)
 fields=['pitch_rate','ctrl_pitch_rate','ctrl_fast_pitch_rate','link_002_joint_v','link_003_joint_v','ctrl_actual_tau_hip_left','com_vz','total_H_pitch']
 for name in names:
  a=A[name];t=[float(r['thrust_ms']) for r in a]
  for ax,k in zip(axes,fields):ax.plot(t,[float(r[k]) for r in a],label=name)
 for ax,k in zip(axes,fields):ax.set_ylabel(k);ax.grid(alpha=.2);ax.legend(ncol=6,fontsize=8);ax.axvline(29,color='gray',ls=':');ax.axvline(0,color='black',ls=':')
 axes[-1].set_xlabel('ms from inferred THRUST entry cycle; native and recorded/held controller values');fig.tight_layout();fig.savefig(R/(group+'_aligned.png'),dpi=120);plt.close(fig)
fig,axes=plt.subplots(4,1,figsize=(13,11),sharex=True)
for name,cc in C.items():
 t=[float(r['thrust_ms']) for r in cc];axes[0].plot(t,[float(r['external_pitch_moment_preCOM']) for r in cc],label=name);axes[1].plot(t,np.cumsum([float(r['contact_impulse_pitch_preCOM']) for r in cc]),label=name);axes[2].plot(t,np.cumsum([float(r['measured_dH_pitch']) for r in cc]),label=name);axes[3].plot(t,np.cumsum([float(r['angular_residual_preCOM']) for r in cc]),label=name)
for ax,label in zip(axes,['contact pitch moment Nm','contact pitch impulse kg m2/s','native H change kg m2/s','unclosed term kg m2/s']):ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(ncol=6,fontsize=8)
axes[-1].set_xlabel('ms from inferred THRUST entry cycle');fig.tight_layout();fig.savefig(R/'contact_aligned.png',dpi=120);plt.close(fig)
for s in S:print(s['run'],'offrate',round(s['off_rate'],3),'earlymax',round(s['early_peak_rate'],3),'actual',round(s['first_actual_ms'],2),'gate',round(s['gate_ms'],2),'age',round(s['imu_age_max_ms'],2))

for group,names in [('old',['B1','T1','B2','T2','B3','T3']),('repeat',[f'R{i}' for i in range(1,7)])]:
 fig,axes=plt.subplots(7,1,figsize=(13,17),sharex=True)
 fields=['link_002_joint_q','ctrl_hip_pos_cmd_left','ctrl_hip_vel_cmd_left','ctrl_knee_vel_cmd_left','ctrl_actual_tau_knee_left','link_004_joint_v','published_wheel_cmd_mps']
 for name in names:
  a=A[name];t=[float(r['thrust_ms']) for r in a]
  for ax,k in zip(axes,fields):ax.plot(t,[float(r[k]) for r in a],label=name)
 for ax,k in zip(axes,fields):ax.set_ylabel(k);ax.grid(alpha=.2);ax.legend(ncol=6,fontsize=8);ax.axvline(29,color='gray',ls=':')
 axes[-1].set_xlabel('ms from inferred THRUST entry cycle');fig.tight_layout();fig.savefig(R/(group+'_reference_wheel.png'),dpi=120);plt.close(fig)
fig,axes=plt.subplots(6,2,figsize=(15,17),sharex=True)
for rowidx,name in enumerate([f'R{i}' for i in range(1,7)]):
 s=next(s for s in S if s['run']==name);entry=s['entry_s'];a=A[name];ni=read(R/name/'native_inputs.csv')
 for col,(joint,key) in enumerate([(0,'hip'),(1,'knee')]):
  ax=axes[rowidx,col];b=[r for r in ni if int(r['joint_index'])==joint]
  ax.plot([float(r['thrust_ms']) for r in a],[float(r['ctrl_actual_tau_'+key+'_left']) for r in a],label='published effort',drawstyle='steps-post')
  ax.plot([(int(r['sim_time_ns'])/1e9-entry)*1000 for r in b],[float(r['before_physics_joint_force_cmd_sim_input']) if r['before_physics_joint_force_cmd_valid']=='1' else float('nan') for r in b],label='before-physics actual input')
  ax.plot([(int(r['sim_time_ns'])/1e9-entry)*1000 for r in b],[float(r['transmitted_axis_torque']) if r['wrench_valid']=='1' else float('nan') for r in b],label='after-physics joint load')
  ax.set_ylabel(name+' '+key+' Nm');ax.grid(alpha=.2);ax.legend(fontsize=7)
for ax in axes[-1]:ax.set_xlabel('ms from inferred THRUST entry cycle')
fig.tight_layout();fig.savefig(R/'three_torque_types.png',dpi=120);plt.close(fig)
