#!/usr/bin/env python3
"""Rigid-body angular-momentum accounting; native physics states, no control writes."""
from pathlib import Path
import csv,json,math,bisect,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
ROOT=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008');D=ROOT/'physical'/sys.argv[1];R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008')/sys.argv[1];R.mkdir(parents=True,exist_ok=True);model=E.parse(D/'actual_robot.sdf').getroot().find('model');urdf=E.parse(D/'actual_robot.urdf').getroot();meta=list(csv.DictReader((D/'engine_frames.csv.inertials.csv').open()));pars={};model_links={x.attrib['name']:x for x in model.findall('link')}
def vec(r,prefix,axes):return np.array([float(r[prefix+k]) for k in axes])
def dumpcsv(p,rows):
 with p.open('w') as f:
  c=csv.DictWriter(f,fieldnames=rows[0].keys());c.writeheader();c.writerows(rows)
for r in meta:
 n=r['entity_name'].split('::')[-1];c=vec(r,'c','xyz');Q=Rotation.from_quat([float(r[k]) for k in ['qx','qy','qz','qw']]).as_matrix();I=np.array([[float(r[k]) for k in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]]);mass=float(r['mass']);assert mass>0 and np.linalg.eigvalsh(I).min()>0;pars[n]=(mass,c,Q@I@Q.T)
 l=model_links[n];assert abs(mass-float(l.findtext('inertial/mass')))<1e-12;ip=np.array([float(v) for v in l.findtext('inertial/pose').split()]);assert np.max(abs(ip[:3]-c))<1e-12
 for k in ['ixx','ixy','ixz','iyy','iyz','izz']:assert abs(float(r[k])-float(l.findtext('inertial/inertia/'+k)))<1e-12
assert set(pars)==set(model_links) and len(pars)==7
joints=[]
for j in urdf.findall('joint'):
 if j.find('child').attrib['link'] not in pars or j.attrib['type']=='fixed':continue
 joints.append({'name':j.attrib['name'],'parent':j.find('parent').attrib['link'],'child':j.find('child').attrib['link'],'axis':np.array([float(x) for x in j.find('axis').attrib['xyz'].split()]),'origin':np.array([float(x) for x in j.find('origin').attrib['xyz'].split()])})
assert len(joints)==6
GROUPS={'body':['base_link'],'left_thigh':['link_002'],'left_shank':['link_003'],'left_wheel':['link_004'],'right_thigh':['link_005'],'right_shank':['link_006'],'right_wheel':['link_007']};groups={k:[l for sub in v for l in GROUPS[sub]] for k,v in {'legs':['left_thigh','left_shank','right_thigh','right_shank'],'wheels':['left_wheel','right_wheel']}.items()};groups.update(GROUPS)
frames=list(csv.DictReader((D/'ground_frames.csv').open()));ctrl=list(csv.DictReader((D/'velocity_log.csv').open()));entry=next(float(r['timestamp']) for r in ctrl if r['state_name']=='FLIGHT');last=max(int(r['sim_time_ns']) for r in frames if int(r['num_contacts'])>0 and int(r['sim_time_ns'])*1e-9<entry);off=last+1000000;contact={int(r['sim_time_ns']):int(r['num_contacts']) for r in frames};assert all(contact[t]==0 for t in range(off,off+20000001,1000000));first_thrust_index=next(i for i,r in enumerate(ctrl) if r['state_name']=='THRUST');state_entry=float(ctrl[first_thrust_index-1]['timestamp']);assert ctrl[first_thrust_index-1]['state_name']=='SQUAT';start_ns=round(state_entry*1e9);allraw=[];raw={}
for r in csv.DictReader((D/'engine_frames.csv').open()):
 ns=int(r['sim_time_ns'])
 if ns>off+21000000:break
 if start_ns-20000000<=ns<=off+21000000:
  allraw.append(r)
  if r['phase']=='after_step' and ns<=off+20000000:raw.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
assert len(raw)==(off+40000000-start_ns)//1000000+1
controller_times=[float(x['timestamp']) for x in ctrl]
rows=[];linkrows=[];Hs=[];Hrels=[];Is=[];omegas=[];kinres=[];poriginres=[]
for ns in sorted(raw):
 rr=raw[ns];links={};vels={}
 for n in pars:
  r=rr[n];assert r['entity_present']=='1' and r['time_valid']=='1' and r['position_valid']=='1' and r['velocity_valid']=='1' and int(r['physical_state_time_ns'])==ns and int(r['dt_ns'])==1000000
  p=vec(r,'position_','xyz');Q=Rotation.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();v=vec(r,'linear_v','xyz');om=vec(r,'angular_v','xyz');mass,c,I=pars[n];rc=Q@c;links[n]={'m':mass,'p':p,'R':Q,'v':v,'w':om,'pc':p+rc,'vc':v+np.cross(om,rc),'I':Q@I@Q.T}
 for j in joints:
  r=rr[j['name']];assert r['entity_present']=='1' and r['time_valid']=='1' and r['velocity_valid']=='1';vels[j['name']]=float(r['joint_velocity_0'])
 M=sum(x['m'] for x in links.values());C=sum(x['m']*x['pc'] for x in links.values())/M;V=sum(x['m']*x['vc'] for x in links.values())/M;wb=links['base_link']['w'];pb=links['base_link']['p'];vb=links['base_link']['v'];wrel={'base_link':np.zeros(3)};urel={'base_link':np.zeros(3)};remaining=joints.copy()
 while remaining:
  progress=False
  for j in remaining.copy():
   if j['parent'] not in wrel:continue
   parent=links[j['parent']];child=links[j['child']];dis=child['p']-parent['p'];poriginres.append(float(np.linalg.norm(dis-parent['R']@j['origin'])));axis=child['R']@j['axis'];wrel[j['child']]=wrel[j['parent']]+axis*vels[j['name']];urel[j['child']]=urel[j['parent']]+np.cross(wrel[j['parent']],dis);remaining.remove(j);progress=True
  assert progress
 H={};Hs_spin={};Hs_orb={};Hr={};Ilock=np.zeros((3,3))
 for n,l in links.items():
  rad=l['pc']-C;spin=l['I']@l['w'];orb=np.cross(rad,l['m']*(l['vc']-V));H[n]=spin+orb;Hs_spin[n]=spin;Hs_orb[n]=orb;Ilock+=l['I']+l['m']*((rad@rad)*np.eye(3)-np.outer(rad,rad));relv=urel[n]+np.cross(wrel[n],l['R']@pars[n][1]);Hr[n]=l['I']@wrel[n]+np.cross(rad,l['m']*relv)
  kinres.append(float(np.linalg.norm(l['w']-wb-wrel[n])));kinres.append(float(np.linalg.norm(l['vc']-vb-np.cross(wb,l['pc']-pb)-relv)))
  linkrows.append({'sim_time_ns':ns,'t_ms':(ns-off)/1e6,'thrust_ms':(ns-start_ns)/1e6,'link':n,'mass':l['m'],'spin_pitch_H':-spin[0],'orbital_pitch_H':-orb[0],'total_pitch_H':-H[n][0],'relative_pitch_H_from_joint_rates':-Hr[n][0],**{f'com_world_{a}':float(l['pc'][i]) for i,a in enumerate('xyz')},**{f'com_world_v{a}':float(l['vc'][i]) for i,a in enumerate('xyz')}})
 ht=sum(H.values());hr=sum(Hr.values());Hs.append(ht);Hrels.append(hr);Is.append(Ilock);omegas.append(wb)
 row={'sim_time_ns':ns,'t_ms':(ns-off)/1e6,'thrust_ms':(ns-start_ns)/1e6,'contact_count':contact[ns],'total_H_pitch':-ht[0],'pitch_deg':-math.degrees(Rotation.from_matrix(links['base_link']['R']).as_euler('xyz')[0]),'pitch_rate':-wb[0],'com_vz':float(V[2]),**{f'system_com_{a}':float(C[i]) for i,a in enumerate('xyz')}}
 for g,names in groups.items():
  row[g+'_H_pitch']=-sum(H[n][0] for n in names);row[g+'_spin_H_pitch']=-sum(Hs_spin[n][0] for n in names);row[g+'_orbital_H_pitch']=-sum(Hs_orb[n][0] for n in names);row[g+'_relative_H_pitch']=-sum(Hr[n][0] for n in names)
 for j in joints:row[j['name']+'_v']=vels[j['name']];row[j['name']+'_q']=float(rr[j['name']]['joint_position_0'])
 rows.append(row)
Hs=np.array(Hs);Hrels=np.array(Hrels);Is=np.array(Is);omegas=np.array(omegas);H0=Hs[0];pred=np.array([np.linalg.solve(I,H0-hr) for I,hr in zip(Is,Hrels)]);identity=np.array([H-I@w-hr for H,I,w,hr in zip(Hs,Is,omegas,Hrels)]);err=-(pred[:,0]-omegas[:,0])
for row,p in zip(rows,pred):
 row['pitch_rate_predicted_constant_H']=-p[0]
 cmd=ctrl[max(0,bisect.bisect_right(controller_times,row['sim_time_ns']*1e-9)-1)];row['state_name']=cmd['state_name'];row['flight_subphase']=cmd['flight_subphase'];row['published_wheel_cmd_mps']=float(cmd['cmd_x'])
# Joint-rate decomposition, independent of measured body angular velocity.
# It distinguishes wheel bodies carried by moving legs from wheel motor rotation.
def joint_motion_vectors(ns):
 rr=raw[ns];LL={}
 for n,(m,c,I) in pars.items():
  r=rr[n];Q=Rotation.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();p=vec(r,'position_','xyz');LL[n]=(m,p,Q,p+Q@c,Q@I@Q.T)
 C=sum(l[0]*l[3] for l in LL.values())/M;parts={}
 for active in joints:
  wr={'base_link':np.zeros(3)};ur={'base_link':np.zeros(3)};todo=joints.copy()
  while todo:
   for j in todo.copy():
    if j['parent'] not in wr:continue
    pc=LL[j['child']];pa=LL[j['parent']];qd=float(rr[j['name']]['joint_velocity_0']) if j['name']==active['name'] else 0.
    wr[j['child']]=wr[j['parent']]+pc[2]@j['axis']*qd;ur[j['child']]=ur[j['parent']]+np.cross(wr[j['parent']],pc[1]-pa[1]);todo.remove(j)
  parts[active['name']]=sum(l[4]@wr[n]+np.cross(l[3]-C,l[0]*(ur[n]+np.cross(wr[n],l[2]@pars[n][1]))) for n,l in LL.items())
 return parts
joint_parts=[joint_motion_vectors(ns) for ns in sorted(raw)]
wheel_joint_names=[j['name'] for j in joints if j['child'] in groups['wheels']];leg_joint_names=[j['name'] for j in joints if j['name'] not in wheel_joint_names]
for row,parts in zip(rows,joint_parts):
 for n,h in parts.items():row[n+'_relative_H_pitch']=-h[0]
 row['leg_joints_relative_H_pitch']=-sum(parts[n][0] for n in leg_joint_names);row['wheel_joints_relative_H_pitch']=-sum(parts[n][0] for n in wheel_joint_names)
assert max(np.linalg.norm(sum(p.values())-hr) for p,hr in zip(joint_parts,Hrels))<1e-10

for idx,row in enumerate(rows):
 ns=int(row['sim_time_ns']);cmd=ctrl[max(0,bisect.bisect_right(controller_times,ns*1e-9)-1)]
 for k,v in cmd.items():row['ctrl_'+k]=v
 for j in joints:row[j['name']+'_body_rate_term']=float(np.linalg.solve(Is[idx],joint_parts[idx][j['name']])[0])
 row['locked_body_rate_term']=-float(np.linalg.solve(Is[idx],Hs[idx])[0])
 row['hip_H_pitch']=sum(row[n+'_relative_H_pitch'] for n in ['link_002_joint','link_005_joint'])
 row['knee_H_pitch']=sum(row[n+'_relative_H_pitch'] for n in ['link_003_joint','link_006_joint'])
 row['thigh_H_pitch']=row['left_thigh_H_pitch']+row['right_thigh_H_pitch']
 row['shank_H_pitch']=row['left_shank_H_pitch']+row['right_shank_H_pitch']
 row['contact_pairs']=next(r['collision_pairs_json'] for r in frames if int(r['sim_time_ns'])==ns)

for r in rows:r.pop('pitch_rate_predicted_constant_H',None)
def budget(t1,t2):
 a=sorted(raw).index(off+t1*1000000);b=sorted(raw).index(off+t2*1000000);ia=Is[a];ib=Is[b];h=Hs[a]-Hrels[a];by={n:float(np.linalg.solve(ib,joint_parts[b][n]-joint_parts[a][n])[0]) for n in joint_parts[a]};return {'hip':sum(by[n] for n in ['link_002_joint','link_005_joint']),'knee':sum(by[n] for n in ['link_003_joint','link_006_joint']),'wheel':sum(by[n] for n in wheel_joint_names),'configuration':-float(np.linalg.solve(ib,h)[0]-np.linalg.solve(ia,h)[0]),'total_H_change':-float(np.linalg.solve(ib,Hs[b]-Hs[a])[0]),'actual':float(-omegas[b,0]+omegas[a,0])}
(R/'budgets.json').write_text(json.dumps({str(a)+'_'+str(b):budget(a,b) for a,b in [(-50,0),(-40,0),(-30,0),(0,20)]},indent=2))
dumpcsv(R/'window.csv',rows);dumpcsv(R/'links.csv',linkrows);dumpcsv(R/'engine_window.csv',allraw)
(R/'events.json').write_text(json.dumps({'state_entry_anchor_s':state_entry,'first_thrust_logged_s':float(ctrl[first_thrust_index]['timestamp']),'effort_request_s':float(ctrl[first_thrust_index]['effort_switch_request_stamp']),'off_s':off/1e9,'last_contact_s':last/1e9,'arrest_s':entry,'thrust_s':next(float(r['timestamp']) for r in ctrl if r['state_name']=='THRUST'),'native_frames':len(rows),'kin_res':max(kinres)},indent=2))
print(sys.argv[1],off/1e9)
