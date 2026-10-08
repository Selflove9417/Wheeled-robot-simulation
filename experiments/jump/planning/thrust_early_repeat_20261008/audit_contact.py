import sys
from pathlib import Path
import csv,json
import numpy as np
from scipy.spatial.transform import Rotation
W=Path('/home/xy/bbot_ws_new');R=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008';D=R/'physical'/sys.argv[1];O=R/sys.argv[1];e=json.loads((O/'events.json').read_text());start=round(e['state_entry_anchor_s']*1e9);end=round(e['off_s']*1e9)+20000000
meta={r['entity_name'].split('::')[-1]:r for r in csv.DictReader((D/'engine_frames.csv.inertials.csv').open())};native={}
for r in csv.DictReader((O/'engine_window.csv').open()):
 if r['entity_type']=='link':native.setdefault((int(r['sim_time_ns']),r['phase']),{})[r['entity_name'].split('::')[-1]]=r
contacts={}
for r in csv.DictReader((D/'contact_detail.csv').open()):
 ns=int(r['sim_time_ns'])
 if ns>end:break
 if ns>=start:contacts.setdefault(ns,[]).append(r)
def state(rows):
 ls=[]
 for n,m in meta.items():
  r=rows[n];mass=float(m['mass']);q=Rotation.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();c=q@np.array([float(m['c'+k]) for k in 'xyz']);p=np.array([float(r['position_'+k]) for k in 'xyz'])+c;om=np.array([float(r['angular_v'+k]) for k in 'xyz']);v=np.array([float(r['linear_v'+k]) for k in 'xyz'])+np.cross(om,c);I=np.array([[float(m[k]) for k in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]]);ls.append((mass,p,v,om,q@I@q.T))
 M=sum(l[0] for l in ls);C=sum(m*p for m,p,v,o,I in ls)/M;V=sum(m*v for m,p,v,o,I in ls)/M;H=sum(I@o+np.cross(p-C,m*(v-V)) for m,p,v,o,I in ls);return M,C,V,H
out=[];points=[]
for ns in sorted(contacts):
 M,Cb,Vb,Hb=state(native[ns,'before_step']);_,Ca,Va,Ha=state(native[ns,'after_step']);F=np.zeros(3);Tb=np.zeros(3);Ta=np.zeros(3);count=0
 for r in contacts[ns]:
  assert r['time_valid']=='1' and r['feature_valid']=='1'
  if int(r['index'])<0:assert r['contact_count']=='0';continue
  assert r['extra_valid']=='1' and r['point_valid']=='1'
  one='::bbot::' in r['collision1'];two='::bbot::' in r['collision2'];ground='::ground_plane::' in (r['collision1']+r['collision2']);assert ground and one!=two
  sign=1 if one else -1;f=sign*np.array([float(r[k]) for k in ['fx1','fy1','fz1']]);n=sign*np.array([float(r[k]) for k in ['nx1','ny1','nz1']]);p=np.array([float(r[k]) for k in ['px','py','pz']]);assert abs(np.linalg.norm(n)-1)<1e-8;fn=float(f@n);ft=f-fn*n;F+=f;Tb+=np.cross(p-Cb,f);Ta+=np.cross(p-Ca,f);count+=1
  points.append({'sim_time_ns':ns,'thrust_ms':(ns-start)/1e6,'off_ms':(ns-round(e['off_s']*1e9))/1e6,'wheel_collision':r['collision1'] if one else r['collision2'],'fn_N':fn,'ft_N':float(np.linalg.norm(ft)),'Fx':f[0],'Fy':f[1],'Fz':f[2],'px':p[0],'py':p[1],'pz':p[2],'pitch_moment_about_pre_COM_Nm':-np.cross(p-Cb,f)[0]})
 dH=Ha-Hb;Jb=Tb*.001;Ja=Ta*.001
 out.append({'sim_time_ns':ns,'thrust_ms':(ns-start)/1e6,'off_ms':(ns-round(e['off_s']*1e9))/1e6,'contact_points':count,'force_z':F[2],'external_pitch_moment_preCOM':-Tb[0],'external_pitch_moment_postCOM':-Ta[0],'measured_dH_pitch':-dH[0],'contact_impulse_pitch_preCOM':-Jb[0],'contact_impulse_pitch_postCOM':-Ja[0],'angular_residual_preCOM':-(dH-Jb)[0],'angular_residual_postCOM':-(dH-Ja)[0]})
def dump(p,a):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=a[0].keys());w.writeheader();w.writerows(a)
dump(O/'contact_budget.csv',out);dump(O/'contact_points.csv',points)
summary={'native_step_count':len(out),'contact_point_count':len(points),'all_contact_extra_valid':True,'normal_min_N':min(p['fn_N'] for p in points),'thrust_to_off':{}}
for label,subset in [('whole_thrust_to_off',[r for r in out if r['off_ms']<0]),('last40ms',[r for r in out if -40<=r['off_ms']<0]),('air0_20',[r for r in out if 0<=r['off_ms']<=20])]:
 summary['thrust_to_off'][label]={k:sum(r[k] for r in subset) for k in ['measured_dH_pitch','contact_impulse_pitch_preCOM','contact_impulse_pitch_postCOM','angular_residual_preCOM','angular_residual_postCOM']};summary['thrust_to_off'][label]['step_residual_rms']=float(np.sqrt(np.mean([r['angular_residual_preCOM']**2 for r in subset])))
(O/'contact_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
