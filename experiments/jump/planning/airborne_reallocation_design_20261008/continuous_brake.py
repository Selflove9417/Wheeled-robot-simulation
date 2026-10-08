"""Read-only exact polynomial test of C2 arrest-to-brake splice, no ROS."""
import csv,json,numpy as np
from pathlib import Path
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials');P=R/'clearance_apex_ab_20261008'
def poly(q,v,a,qf,vf,af,T):
 c=np.array([q,v*T,.5*a*T*T,0.,0.,0.]);dz=qf-sum(c[:3]);dv=vf*T-c[1]-2*c[2];da=af*T*T-2*c[2];c[3:]=[10*dz-4*dv+.5*da,-15*dz+7*dv-da,6*dz-3*dv+.5*da];return c
def ev(c,u,T,o=0):return np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c,o))/T**o
def pk(c,T,o):
 rr=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(c,o+1));tt=[0.,1.]+[v.real for v in rr if abs(v.imag)<1e-8 and 0<v.real<1];return max(abs(ev(c,x,T,o)) for x in tt)
out=[]
for n in ['B1','B2','B3']:
 m=json.loads((P/'analysis'/n/'metrics.json').read_text());rows=list(csv.DictReader((P/'physical'/n/'velocity_log.csv').open()));ar=next(x for x in rows if x['state_name']=='FLIGHT');tu=m['tuck_entry'];T=.06;u=(float(tu['timestamp'])-float(ar['timestamp']))/T;rec=[]
 for s in ['left','right']:
  for k in ['hip','knee']:
   q=float(ar[k+'_pos_'+s]);v=float(ar[k+'_vel_'+s]);vf=np.clip(v,-(2 if k=='hip' else 4),(2 if k=='hip' else 4));qf=np.clip(q+.5*(v+vf)*T,-1.56,1.56);c=poly(q,v,0,qf,vf,0,T);a0=[ev(c,u,T,o) for o in range(3)];end=float(tu[k+'_pos_'+s]);brake=poly(*a0,end,0,0,.06);rec.append({'joint':k+'_'+s,'arrest_reference_q_v_a':a0,'measured_tuck_q_v':[end,float(tu[k+'_vel_'+s])],'old_boundary_reference_jump':[end-a0[0],float(tu[k+'_vel_'+s])-a0[1]],'continuous_brake_peaks':[pk(brake,.06,o) for o in range(3)]})
 out.append({'run':n,'arrest_s':float(ar['timestamp']),'tuck_s':float(tu['timestamp']),'joints':rec})
(R/'airborne_reallocation_design_20261008/continuous_brake.json').write_text(json.dumps(out,indent=2)+'\n')
