from pathlib import Path
import csv,json
ROOT=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials');R=ROOT/'takeoff_variability_20261007'
for n in ['B1','T1','B2','T2','B3','T3']:
 d=ROOT/'hip_momentum_ab_20261007/physical'/n;off=round(json.loads((R/n/'events.json').read_text())['off_s']*1e9);a=[]
 for r in csv.DictReader((d/'native_wrench.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>off+20000000:break
  if off-51000000<=ns<=off+20000000:a.append(r)
 with (R/n/'native_inputs.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=a[0].keys());w.writeheader();w.writerows(a)
 h=[r for r in a if r['joint_index']=='0'];z=[]
 for i,r in enumerate(h):
  t=(int(r['sim_time_ns'])-off)/1e6;v=float(r['before_physics_joint_force_cmd_sim_input']);old=None if i==0 else float(h[i-1]['before_physics_joint_force_cmd_sim_input'])
  if i==0 or v!=old:z.append([t,round(v,5)])
 print(n,'hip input changes',z,'contactwrenches',sorted(set(r['contact_wrench_count'] for r in h)))
