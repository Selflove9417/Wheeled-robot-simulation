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
W=Path('/home/xy/bbot_ws_new');DATA=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';name=sys.argv[1];R=DATA/'height_032_pitch_formation_20261008'/name;R.mkdir(exist_ok=False);ROOT=DATA/('clearance_apex_ab_20261008' if name.startswith('B') else 'thrust_height_032_ab_20261008');D=ROOT/('physical' if name.startswith('B') else 'physical_runs')/name;model=E.parse(D/'actual_robot.sdf').getroot().find('model');urdf=E.parse(D/'actual_robot.urdf').getroot();meta=list(csv.DictReader((D/'engine_frames.csv.inertials.csv').open()));pars={};model_links={x.attrib['name']:x for x in model.findall('link')}
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
frames=list(csv.DictReader((D/'ground_frames.csv').open()));ctrl=list(csv.DictReader((D/'velocity_log.csv').open()));entry=next(float(r['timestamp']) for r in ctrl if r['state_name']=='FLIGHT');last=max(int(r['sim_time_ns']) for r in frames if int(r['num_contacts'])>0 and int(r['sim_time_ns'])*1e-9<entry);off=last+1000000;contact={int(r['sim_time_ns']):int(r['num_contacts']) for r in frames};td=min(t for t,n in contact.items() if t>off and n>0);end_ns=td-1000000;assert all(contact[t]==0 for t in range(off,td,1000000));allraw=[];raw={}
for r in csv.DictReader((D/'engine_frames.csv').open()):
 ns=int(r['sim_time_ns'])
 if ns>td:break
 if off<=ns<=td:
  allraw.append(r)
  if r['phase']=='after_step' and ns<=end_ns:raw.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
assert len(raw)==(td-off)//1000000
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
  linkrows.append({'sim_time_ns':ns,'t_ms':(ns-off)/1e6,'link':n,'mass':l['m'],'spin_pitch_H':-spin[0],'orbital_pitch_H':-orb[0],'total_pitch_H':-H[n][0],'relative_pitch_H_from_joint_rates':-Hr[n][0],**{f'com_world_{a}':float(l['pc'][i]) for i,a in enumerate('xyz')},**{f'com_world_v{a}':float(l['vc'][i]) for i,a in enumerate('xyz')}})
 ht=sum(H.values());hr=sum(Hr.values());Hs.append(ht);Hrels.append(hr);Is.append(Ilock);omegas.append(wb)
 row={'sim_time_ns':ns,'t_ms':(ns-off)/1e6,'contact_count':contact[ns],'total_H_pitch':-ht[0],'pitch_deg':-math.degrees(Rotation.from_matrix(links['base_link']['R']).as_euler('xyz')[0]),'pitch_rate':-wb[0],**{f'system_com_{a}':float(C[i]) for i,a in enumerate('xyz')}}
 for g,names in groups.items():
  row[g+'_H_pitch']=-sum(H[n][0] for n in names);row[g+'_spin_H_pitch']=-sum(Hs_spin[n][0] for n in names);row[g+'_orbital_H_pitch']=-sum(Hs_orb[n][0] for n in names);row[g+'_relative_H_pitch']=-sum(Hr[n][0] for n in names)
 for j in joints:row[j['name']+'_v']=vels[j['name']]
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
minimum=int(np.argmin([-w[0] for w in omegas]));peak=int(np.argmax([-w[0] for w in omegas[:minimum+1]]))
def rate_budget(a,b):
 Href=Hs[a];ia=Is[a];ib=Is[b];ha=Hrels[a];hb=Hrels[b];shape=-(np.linalg.solve(ib,Href-ha)[0]-np.linalg.solve(ia,Href-ha)[0]);by_rigid_group={}
 for g in ['legs','wheels']:
  # Full-vector relative body-group values from measured kinematic residual-free data.
  vals=[]
  for idx in [a,b]:
   ns=sorted(raw)[idx];rt=raw[ns];wb=omegas[idx];pb=vec(rt['base_link'],'position_','xyz');vb=vec(rt['base_link'],'linear_v','xyz');group=np.zeros(3)
   C=np.array([rows[idx]['system_com_'+k] for k in 'xyz'])
   for n in groups[g]:
    m,c,I=pars[n];r=rt[n];Q=Rotation.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]).as_matrix();p=vec(r,'position_','xyz')+Q@c;om=vec(r,'angular_v','xyz');v=vec(r,'linear_v','xyz')+np.cross(om,Q@c)
    group+=Q@I@Q.T@(om-wb)+np.cross(p-C,m*(v-vb-np.cross(wb,p-pb)))
   vals.append(group)
  by_rigid_group[g]=float(np.linalg.solve(ib,vals[1]-vals[0])[0])
 by_joint={n:float(np.linalg.solve(ib,joint_parts[b][n]-joint_parts[a][n])[0]) for n in joint_parts[a]}
 residual=-float(np.linalg.solve(ib,Hs[b]-Href)[0]);return {'start_ms':a,'end_ms':b,'actual_rate_change':float(-omegas[b,0]+omegas[a,0]),'shape_rate_change':float(shape),'rigid_group_relative_rate_changes':by_rigid_group,'joint_rate_changes':by_joint,'leg_joint_rate_change':sum(by_joint[n] for n in leg_joint_names),'wheel_joint_rate_change':sum(by_joint[n] for n in wheel_joint_names),'H_drift_rate_change':residual}

# Exact instantaneous rate terms, with frozen liftoff total H.
for row,I,H,parts in zip(rows,Is,Hs,joint_parts):
 row['rate_from_initial_H']=-float(np.linalg.solve(I,H0)[0])
 row['rate_from_H_drift']=-float(np.linalg.solve(I,H-H0)[0])
 for label,names in [('hip',['link_002_joint','link_005_joint']),('knee',['link_003_joint','link_006_joint']),('wheel',['link_004_joint','link_007_joint'])]:
  row['rate_from_'+label]=float(np.linalg.solve(I,sum(parts[n] for n in names))[0])
 assert abs(sum(row['rate_from_'+label] for label in ['initial_H','H_drift','hip','knee','wheel'])-row['pitch_rate'])<1e-7
# Store full vector and matrices to compare initial-H and configuration terms between trials.
np.savez(R/'vectors.npz',H=Hs,I=Is,Hrel=Hrels,H0=H0,pitch_rate=np.array([r['pitch_rate'] for r in rows]))
dumpcsv(R/'momentum.csv',rows);dumpcsv(R/'link_momentum.csv',linkrows)
summary={'run':name,'off_s':off/1e9,'first_touch_s':td/1e9,'frames':len(rows),'mass':M,'initial_total_pitch_H':-float(H0[0]),'maximum_pitch_H_drift':float(abs(Hs[:,0]-H0[0]).max()),'maximum_vector_H_drift':float(np.linalg.norm(Hs-H0,axis=1).max()),'max_kinematic_residual':max(kinres),'max_joint_origin_residual':max(poriginres),'max_full_H_identity_residual':float(np.linalg.norm(identity,axis=1).max()),'constant_H_rate_rms_error':float(np.sqrt(np.mean(err**2))),'constant_H_rate_max_error':float(abs(err).max()),'first':rows[0],'last':rows[-1]}
(R/'summary.json').write_text(json.dumps(summary,indent=2))
(R/'model_inventory.json').write_text(json.dumps({n:{'mass':m,'com_in_link':c.tolist(),'inertia_in_link':I.tolist()} for n,(m,c,I) in pars.items()},indent=2))
print(name,json.dumps({k:v for k,v in summary.items() if k not in ['first','last']}),flush=True)
