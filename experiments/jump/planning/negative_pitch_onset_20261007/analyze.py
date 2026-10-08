#!/usr/bin/env python3
"""Read existing physical data only; no controller or ROS mutations."""
import csv,json,bisect,math
from pathlib import Path
W=Path('/home/xy/bbot_ws_new');O=Path(__file__).resolve().parent;S=O.parent/'landing_capture_pitch_chain_20261007';P=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007'
D=json.loads((S/'summary.json').read_text());results=[];table=[]
for d in D:
 n=d['run'];rr=list(csv.DictReader((P/n/'velocity_log.csv').open()));rt=[float(r['timestamp']) for r in rr];nr=list(csv.DictReader((S/n/'native_window.csv').open()));j={k:{} for k in ['link_002_joint','link_003_joint','link_004_joint','link_005_joint','link_006_joint','link_007_joint']}
 for r in nr:j[r['joint_name']][int(r['sim_time_ns'])]=r
 ts=sorted(j['link_002_joint']);rate=[-float(j['link_002_joint'][t]['post_base_world_wx']) for t in ts];blocks=[];start=None
 for i,v in enumerate(rate):
  if v<-.2 and start is None:start=i
  if start is not None and (v>=-.2 or i==len(rate)-1):
   if i-start>=15:blocks.append({'start_rel_contact_s':ts[start]*1e-9-d['first_contact_s'],'duration_ms':i-start,'index':start})
   start=None
 b=next(x for x in blocks if x['duration_ms']>=50 and x['start_rel_contact_s']<0);idx=b['index'];t0=ts[idx]*1e-9;snaps=[]
 for off in [-30,-15,0,15,30,45,60]:
  i=idx+off
  if not 0<=i<len(ts):continue
  ns=ts[i];t=ns*1e-9;c=rr[max(0,bisect.bisect_right(rt,t)-1)];x={'run':n,'relative_onset_ms':off,'sim_s':t,'relative_contact_s':t-d['first_contact_s'],'phase':c['state_name']+('/'+c['flight_subphase'] if c['state_name']=='FLIGHT' else ''),'native_pitch_rate':rate[i]}
  for k in ['pitch','pitch_rate','cmd_x','air_wheel_cmd_raw','air_wheel_baseline','flight_air_pitch_ref','flight_air_pitch_rate_ref','thrust_cmd_x','thrust_att_term','thrust_fwd_term','thrust_release_active','thrust_release_blend','arrest_dynamics_ff_scope','arrest_dynamics_ff_guard','arrest_dynamics_ff_blend','arrest_ff_applied_hl','arrest_ff_applied_kl','arrest_qdd_hl','arrest_qdd_kl','tau_body_hip','flight_arrest_freewheel_active']:
   x[k]=float(c[k])
  for part,side,jn in [('hip','left','link_002_joint'),('knee','left','link_003_joint'),('hip','right','link_005_joint'),('knee','right','link_006_joint')]:
   r=j[jn][ns];tag=part+'_'+side;x[tag+'_actual_q']=float(r['joint_position']);x[tag+'_actual_v']=float(r['joint_velocity']);x[tag+'_target_q']=float(c[part+'_pos_cmd_'+side]);x[tag+'_target_v']=float(c[part+'_vel_cmd_'+side]);x[tag+'_physical_input']=float(r['before_physics_joint_force_cmd_sim_input']) if r['before_physics_joint_force_cmd_valid']=='1' else None
  x['wheel_left_v']=float(j['link_004_joint'][ns]['joint_velocity']);x['wheel_right_v']=float(j['link_007_joint'][ns]['joint_velocity']);snaps.append(x);table.append(x)
 contacts={int(r['sim_time_ns']):int(r['num_contacts']) for r in csv.DictReader((S/n/'contact_window.csv').open())};x=snaps[2];result={'run':n,'gain':d['gain'],'onset_sim_s':t0,'onset_rel_contact_s':t0-d['first_contact_s'],'onset_rel_flight_entry_s':t0-(d['first_contact_s']+d['flight_entry_rel_s']),'onset_rel_tuck_s':t0-(d['first_contact_s']+d['tuck_entry_rel_s']),'negative_block_ms':b['duration_ms'],'contact_points_at_onset':contacts[ts[idx]],'negative_blocks_ge15ms':blocks,'snapshots':snaps};results.append(result)
 print(n,round(result['onset_rel_contact_s'],3),x['phase'],'rate',round(x['native_pitch_rate'],3),'v',round(x['hip_left_actual_v'],2),round(x['knee_left_actual_v'],2),'cmd',round(x['cmd_x'],3),'ff',x['arrest_ff_applied_hl'],x['arrest_ff_applied_kl'])
(O/'summary.json').write_text(json.dumps(results,indent=2))
with (O/'onset_samples.csv').open('w') as f:
 c=csv.DictWriter(f,fieldnames=table[0].keys());c.writeheader();c.writerows(table)
