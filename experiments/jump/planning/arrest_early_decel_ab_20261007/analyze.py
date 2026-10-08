#!/usr/bin/env python3
# Fixed six-trial analysis; all physical samples retained.
from pathlib import Path
import csv,json,math,bisect,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007');P=R/'physical';N=['B1','T1','B2','T2','B3','T3'];results=[]
JOINTS=['link_002_joint','link_003_joint','link_005_joint','link_006_joint','link_004_joint','link_007_joint']
def f(r,k):return float(r[k])
def pitch(r):
 x,y,z,w=[f(r,'base_'+k) for k in ['qx','qy','qz','qw']];return -math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
def continuous(t,b,span):
 start=None
 for x,good in zip(t,b):
  if good:
   if start is None:start=x
   if x-start>=span-1e-9:return float(start)
  else:start=None
 return None
for name in N:
 d=P/name
 if not (d/'run_result.json').exists():continue
 run=json.loads((d/'run_result.json').read_text());rr=list(csv.DictReader((d/'velocity_log.csv').open()));fr=list(csv.DictReader((d/'ground_frames.csv').open()));gg=list(csv.DictReader((d/'geometry.csv').open()));flight=[r for r in rr if r['state_name']=='FLIGHT'];m={'run':name,'group':run['group'],'stop_reason':run['stop_reason'],'jump_sent':run['jump_sent'],'flight':bool(flight),'states':list(dict.fromkeys(r['state_name'] for r in rr))}
 if not flight:results.append(m);continue
 entry=f(flight[0],'timestamp');last=max(int(r['sim_time_ns']) for r in fr if int(r['sim_time_ns'])*1e-9<entry and int(r['num_contacts'])>0);off=last+1000000;td=next(int(r['sim_time_ns']) for r in fr if int(r['sim_time_ns'])>last and int(r['num_contacts'])>0)
 assert all(r['frame_valid']=='1' and int(r['num_contacts'])==0 for r in fr if off<=int(r['sim_time_ns'])<td)
 stamps=[int(r['sim_time_ns']) for r in fr];assert all(b-a==1000000 for a,b in zip(stamps,stamps[1:]));geom={int(g['sim_time_ns']):g for g in gg};assert all(g['frame_valid']=='1' for g in gg)
 native={j:{} for j in JOINTS};rates={};native_slice=[]
 for r in csv.DictReader((d/'native_wrench.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>td+1000000:break
  if ns<off-80000000:continue
  if ns<=off+80000000:native_slice.append(r)
  native[r['joint_name']][ns]=r
  if r['joint_index']=='0':assert r['post_base_velocity_valid']=='1';rates[ns]=-f(r,'post_base_world_wx')
 ts=np.array(list(range(off,off+80000001,1000000)),dtype=np.int64);tt=(ts-off)*1e-9;pp=np.array([pitch(geom[t]) for t in ts]);pr=np.array([rates[t] for t in ts]);rt=[f(r,'timestamp') for r in rr];cmd=[rr[max(0,bisect.bisect_right(rt,t*1e-9)-1)] for t in ts]
 onset_ts=np.array(sorted(t for t in rates if off-80000000<=t<td));onset=continuous(onset_ts*1e-9,np.array([rates[t]<-.2 for t in onset_ts]),.049)
 pre_t=f(run['pre_jump_row'],'timestamp');pre=run['pre_jump_row'];tuck=next((f(r,'timestamp') for r in flight if r['flight_subphase']=='1'),None)
 m.update(off_s=off*1e-9,last_contact_s=last*1e-9,arrest_entry_rel_ms=(entry-off*1e-9)*1000,tuck_entry_rel_ms=None if tuck is None else (tuck-off*1e-9)*1000,first_contact_s=td*1e-9,negative_onset_rel_ms=None if onset is None else (onset-off*1e-9)*1000,negative_onset_left_censored=bool(onset is not None and abs(onset-onset_ts[0]*1e-9)<1e-9),min_rate_first50=float(pr[:51].min()),pitch_delta_first80_deg=math.degrees(pp[-1]-pp[0]),pitch_at_off_deg=math.degrees(pp[0]),rate_at_off=float(pr[0]),contact_pitch_deg=math.degrees(pitch(geom[td])),contact_rate=rates[td],peak_bilateral_clearance_m=max(min(f(geom[t],'left_wheel_z'),f(geom[t],'right_wheel_z'))-.07 for t in geom if off<=t<td),pre_jump_pitch=f(pre,'pitch'),pre_jump_rate=f(pre,'pitch_rate'),pre_jump_com_velocity=f(pre,'capture_com_velocity'),new_emergency='EMERGENCY' in m['states'],joints={})
 for j in JOINTS[:4]:
  v=np.array([f(native[j][t],'joint_velocity') for t in ts]);m['joints'][j]={'v0':float(v[0]),'v30':float(v[30]),'v50':float(v[50]),'v80':float(v[80]),'total_speed_variation50':float(abs(np.diff(v[:51])).sum()),'acceleration_rms50':float(np.sqrt(np.mean((np.diff(v[:51])/.001)**2)))}
 
 for j in JOINTS[:4]:
  v=np.array([f(native[j][t],'joint_velocity') for t in ts]); assert all(native[j][t]['state_valid']=='1' for t in ts)
  for span in [20,30,50]:
   m['joints'][j]['speed_reduction'+str(span)]=float(abs(v[0])-abs(v[span]));m['joints'][j]['speed_braking_variation'+str(span)]=float(np.maximum(0,abs(v[:span])-abs(v[1:span+1])).sum());m['joints'][j]['total_variation'+str(span)]=float(abs(np.diff(v[:span+1])).sum())
 m['mean_braking_variation30']=float(np.mean([x['speed_braking_variation30'] for x in m['joints'].values()]))
 m['mean_braking_variation50']=float(np.mean([x['speed_braking_variation50'] for x in m['joints'].values()]))
 m['mean_leg_total_variation50']=float(np.mean([x['total_speed_variation50'] for x in m['joints'].values()]));m['mean_leg_acceleration_rms50']=float(np.mean([x['acceleration_rms50'] for x in m['joints'].values()]));m['nonwheel_ground_contact']=any(any('ground_plane' in a+b and not('link_004_collision' in a+b or 'link_007_collision' in a+b) for a,b in json.loads(r['collision_pairs_json'])) for r in fr if int(r['sim_time_ns'])>off)
 # Post-contact recovery using native pitch and controller quiet gate, without claiming jump qualification.
 post=[r for r in rr if f(r,'timestamp')>=td*1e-9];st=np.array([f(r,'timestamp')-td*1e-9 for r in post]);quiet=[abs(f(r,'pitch')-.03)<.04 and abs(f(r,'pitch_rate'))<.15 and abs(f(r,'capture_com_velocity'))<.08 and abs(f(r,'gazebo_world_z_dot'))<.03 for r in post];m['full_quiet_start_s']=continuous(st,quiet,1.);m['post_contact_max_abs_native_pitch_deg']=max(abs(math.degrees(pitch(g))) for t,g in geom.items() if t>=td);m['observation_after_contact_s']=max(int(r['sim_time_ns']) for r in fr)*1e-9-td*1e-9
 zero_start=None;secondary=[]
 for r in fr:
  ns=int(r['sim_time_ns'])
  if ns<=td:continue
  if int(r['num_contacts'])==0:
   if zero_start is None:zero_start=ns
  elif zero_start is not None:
   if ns-zero_start>=10000000 and max(min(f(geom[t],'left_wheel_z'),f(geom[t],'right_wheel_z'))-.07 for t in geom if zero_start<=t<ns)>.002:secondary.append([(zero_start-td)*1e-9,(ns-td)*1e-9])
   zero_start=None
 m['secondary_flight_intervals']=secondary
 for filename,data in [('early_native.csv',native_slice),('early_controller.csv',[r for r in rr if off*1e-9-.005<=f(r,'timestamp')<=off*1e-9+.085])]:
  with (d/filename).open('w') as s:
   c=csv.DictWriter(s,fieldnames=data[0].keys());c.writeheader();c.writerows(data)
 fig,ax=plt.subplots(6,1,figsize=(12,15),sharex=True)
 for part,side,j in [('hip','left',JOINTS[0]),('knee','left',JOINTS[1]),('hip','right',JOINTS[2]),('knee','right',JOINTS[3])]:
  idx=0 if part=='hip' else 1;ax[idx].plot(tt*1000,[f(native[j][t],'joint_velocity') for t in ts],label=side+' native');ax[idx].plot(tt*1000,[f(r,part+'_vel_cmd_'+side) for r in cmd],label=side+' published target',linestyle='--')
 ax[2].plot(tt*1000,np.degrees(pp),label='native pitch deg');ax[3].plot(tt*1000,pr,label='native pitch_rate rad/s');ax[4].plot(tt*1000,[f(r,'cmd_x') for r in cmd],label='published wheel m/s')
 for j in JOINTS[4:]:ax[4].plot(tt*1000,[.07*f(native[j][t],'joint_velocity') for t in ts],label=j+' native v*R m/s')
 phases=[r['state_name']+'/'+r['flight_subphase'] for r in cmd];labels=list(dict.fromkeys(phases));ax[5].step(tt*1000,[labels.index(k) for k in phases],where='post');ax[5].set_yticks(range(len(labels)),labels)
 for a in ax:a.grid(alpha=.2)
 for a in ax[:5]:a.legend(fontsize=8)
 ax[-1].set_xlabel('ms from first permanently no-contact native frame');fig.suptitle(name+' '+m['group']+' recorded Gazebo');fig.tight_layout();fig.savefig(d/'early_window.png',dpi=120);plt.close(fig)
 # Three torque types stay separate: published, physical-step input, aggregate joint load.
 fig,axs=plt.subplots(4,1,figsize=(12,13),sharex=True)
 for part,side,j,a in [('hip','left',JOINTS[0],axs[0]),('knee','left',JOINTS[1],axs[1]),('hip','right',JOINTS[2],axs[2]),('knee','right',JOINTS[3],axs[3])]:
  a.plot(tt*1000,[f(r,'actual_tau_'+part+'_'+side) for r in cmd],label='published command Nm');a.plot(tt*1000,[f(native[j][t],'before_physics_joint_force_cmd_sim_input') if native[j][t]['before_physics_joint_force_cmd_valid']=='1' else np.nan for t in ts],label='BeforePhysics applied input Nm');a.plot(tt*1000,[f(native[j][t],'transmitted_axis_torque') if native[j][t]['wrench_valid']=='1' else np.nan for t in ts],label='aggregate joint load Nm');a.set_ylabel(part+' '+side);a.grid(alpha=.2);a.legend()
 axs[-1].set_xlabel('ms from first permanently no-contact frame');fig.suptitle(name+' physical records; no model torque curves');fig.tight_layout();fig.savefig(d/'early_torques.png',dpi=120);plt.close(fig)
 results.append(m);print(name,'onset',m['negative_onset_rel_ms'],'min50',m['min_rate_first50'],'delta80',m['pitch_delta_first80_deg'],'TV50',m['mean_leg_total_variation50'],'contact',m['contact_pitch_deg'],'height',m['peak_bilateral_clearance_m'],flush=True)
(R/'metrics.json').write_text(json.dumps(results,indent=2));groups={}
fields=['negative_onset_rel_ms','min_rate_first50','pitch_delta_first80_deg','mean_leg_total_variation50','mean_leg_acceleration_rms50','mean_braking_variation30','mean_braking_variation50','contact_pitch_deg','peak_bilateral_clearance_m','rate_at_off','arrest_entry_rel_ms','tuck_entry_rel_ms']
for group in ['baseline','test']:
 a=[r for r in results if r['group']==group and r['flight']];groups[group]={'n':len(a)}
 for k in fields:
  x=[r[k] for r in a if r.get(k) is not None];groups[group][k]={'mean':float(np.mean(x)) if x else None,'median':float(np.median(x)) if x else None,'values':x}
(R/'group_summary.json').write_text(json.dumps(groups,indent=2))
if len(results)==6:
 fig,ax=plt.subplots(2,1,figsize=(12,8),sharex=True)
 for m in results:
  ns=list(csv.DictReader((P/m['run']/'early_native.csv').open()));rr=[r for r in ns if r['joint_index']=='0' and int(r['sim_time_ns'])*1e-9>=m['off_s']];t=[int(r['sim_time_ns'])*1e-6-m['off_s']*1000 for r in rr];v=[-f(r,'post_base_world_wx') for r in rr];ax[0].plot(t,v,label=m['run']+' '+m['group']);ax[1].plot(t,[f(r,'joint_velocity') for r in rr],label=m['run'])
 ax[0].legend(ncol=3);ax[0].set_ylabel('native pitch_rate rad/s');ax[1].set_ylabel('native left hip rad/s');ax[1].set_xlabel('ms from first permanently no-contact frame')
 for a in ax:a.grid(alpha=.2)
 fig.tight_layout();fig.savefig(R/'early_comparison.png',dpi=140)
