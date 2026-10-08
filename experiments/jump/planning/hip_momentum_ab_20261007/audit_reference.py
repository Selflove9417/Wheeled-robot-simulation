from pathlib import Path
import csv,json,numpy as np
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007')
out={}
for d in sorted((R/'physical').glob('[BT][123]')):
 if not (d/'run_result.json').exists():continue
 rows=list(csv.DictReader((d/'velocity_log.csv').open()));fl=[r for r in rows if r['state_name']=='FLIGHT'];first=fl[0];t0=float(first['timestamp']);items={}
 for part in ['hip','knee']:
  for side in ['left','right']:
   q0=float(first[part+'_pos_cmd_'+side]);v0=float(first[part+'_vel_cmd_'+side]);vf=float(np.clip(v0,-(2 if part=='hip' else 4),2 if part=='hip' else 4))
   if d.name[0]=='T' and part=='hip':vf=v0-.25*(v0-vf)
   T=.060;rawq=q0+.5*(v0+vf)*T;qf=float(np.clip(rawq,-1.56,1.56));a0=q0;a1=v0*T;dz=qf-a0-a1;dv=vf*T-a1;a3=10*dz-4*dv;a4=-15*dz+7*dv;a5=6*dz-3*dv
   e=[];ev=[];ea=[]
   for r in fl:
    if r['flight_subphase']!='0':continue
    u=float(np.clip((float(r['timestamp'])-t0)/T,0,1));q=a0+a1*u+a3*u**3+a4*u**4+a5*u**5;v=(a1+3*a3*u*u+4*a4*u**3+5*a5*u**4)/T;a=(6*a3*u+12*a4*u*u+20*a5*u**3)/T**2
    e.append(abs(q-float(r[part+'_pos_cmd_'+side])));ev.append(abs(v-float(r[part+'_vel_cmd_'+side])));k='arrest_qdd_'+('h' if part=='hip' else 'k')+('l' if side=='left' else 'r');ea.append(abs(a-float(r[k])))
   items[part+'_'+side]={'q0':q0,'v0':v0,'qf':qf,'vf':vf,'q_endpoint_clamped':rawq!=qf,'max_q_reference_error':max(e),'max_v_reference_error':max(ev),'max_a_reference_error':max(ea),'samples':len(e)}
 out[d.name]=items
(R/'reference_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
