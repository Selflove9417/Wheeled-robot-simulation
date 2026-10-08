from pathlib import Path
import csv,json,numpy as np
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007');A={n:list(csv.DictReader((R/n/'window.csv').open())) for n in ['B1','T1','B2','T2','B3','T3']};D={n:{int(float(r['thrust_ms'])):r for r in a} for n,a in A.items()};normal=['B1','T1','T2','B3'];bad=['B2','T3'];fields=['pitch_deg','pitch_rate','ctrl_fast_pitch_rate','link_002_joint_q','link_003_joint_q','link_002_joint_v','link_003_joint_v','system_com_z','com_vz','link_004_joint_v','ctrl_thrust_motion_elapsed','total_H_pitch'];out={}
for k in fields:
 flags=[]
 for t in range(161):
  lo=min(float(D[n][t][k]) for n in normal);hi=max(float(D[n][t][k]) for n in normal);x=[float(D[n][t][k]) for n in bad];flags.append(-1 if max(x)<lo else 1 if min(x)>hi else 0)
 first=next((t for t in range(142) if flags[t]!=0 and all(z==flags[t] for z in flags[t:t+20])),None);out[k]={'first_common_outside_normal_range_20ms':first,'direction':None if first is None else flags[first]}
print(json.dumps(out,indent=2));(R/'early_divergence.json').write_text(json.dumps(out,indent=2))
for n in A:
 a=list(csv.DictReader((R/n/'native_inputs.csv').open()));e=json.loads((R/n/'events.json').read_text());entry=e['state_entry_anchor_s'];h=[r for r in a if r['joint_index']=='0'];first=next(r for r in h if r['before_physics_joint_force_cmd_valid']=='1' and abs(float(r['before_physics_joint_force_cmd_sim_input']))>1e-8);print(n,'first actual nonzero hip',round(int(first['sim_time_ns'])/1e6-entry*1000),'initdelay',round(1000*(e['first_thrust_logged_s']-entry)),'requestdelay',round(1000*(e['effort_request_s']-entry)))
