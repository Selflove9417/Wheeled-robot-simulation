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
ROOT=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007');D=ROOT/'physical'/sys.argv[1];R=D/'momentum';R.mkdir(exist_ok=True);model=E.parse(D/'actual_robot.sdf').getroot().find('model');urdf=E.parse(D/'actual_robot.urdf').getroot();meta=list(csv.DictReader((D/'engine_frames.csv.inertials.csv').open()));pars={};model_links={x.attrib['name']:x for x in model.findall('link')}
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
frames=list(csv.DictReader((D/'ground_frames.csv').open()));ctrl=list(csv.DictReader((D/'velocity_log.csv').open()));entry=next(float(r['timestamp']) for r in ctrl if r['state_name']=='FLIGHT');last=max(int(r['sim_time_ns']) for r in frames if int(r['num_contacts'])>0 and int(r['sim_time_ns'])*1e-9<entry);off=last+1000000;contact={int(r['sim_time_ns']):int(r['num_contacts']) for r in frames};assert all(contact[t]==0 for t in range(off,off+80000001,1000000));allraw=[];raw={}
for r in csv.DictReader((D/'engine_frames.csv').open()):
 ns=int(r['sim_time_ns'])
 if ns>off+81000000:break
 if off<=ns<=off+81000000:
  allraw.append(r)
  if r['phase']=='after_step' and ns<=off+80000000:raw.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
assert len(raw)==81
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
budget_fixed=rate_budget(18,54);budget60=rate_budget(0,60);budget80=rate_budget(0,80);budget_fast=rate_budget(peak,minimum);shape=budget80['shape_rate_change'];rate_contrib=budget80['rigid_group_relative_rate_changes'];rate_contrib['body']=0.
summary={'mass_total':M,'link_names':list(pars),'last_contact_s':last/1e9,'off_s':off/1e9,'arrest_entry_ms':(entry-off/1e9)*1000,'native_frames':len(rows),'max_kinematic_velocity_residual':max(kinres),'max_joint_origin_pose_residual_m':max(poriginres),'max_full_vector_H_identity_residual':float(np.linalg.norm(identity,axis=1).max()),'total_pitch_H0':-float(Hs[0,0]),'total_pitch_H80':-float(Hs[-1,0]),'max_pitch_H_drift':float(abs(Hs[:,0]-Hs[0,0]).max()),'max_pitch_H_drift_fraction_H0':float(abs(Hs[:,0]-Hs[0,0]).max()/abs(Hs[0,0])),'max_vector_H_drift':float(np.linalg.norm(Hs-H0,axis=1).max()),'rate_prediction_rms':float(np.sqrt(np.mean(err**2))),'rate_prediction_max_error':float(abs(err).max()),'rate_actual0':rows[0]['pitch_rate'],'rate_actual80':rows[-1]['pitch_rate'],'rate_actual_min':min(r['pitch_rate'] for r in rows),'rate_predicted80':rows[-1]['pitch_rate_predicted_constant_H'],'locked_shape_rate_change0_80':float(shape),'relative_joint_motion_rate_change0_80':rate_contrib,'rate_budget_0_80':budget80,'rate_budget_18_54':budget_fixed,'rate_budget_0_60':budget60,'rate_budget_fast_rearward':budget_fast,'first':rows[0],'last':rows[-1]}
(R/'summary.json').write_text(json.dumps(summary,indent=2));dumpcsv(R/'momentum_window.csv',rows);dumpcsv(R/'link_momentum_window.csv',linkrows);dumpcsv(R/'engine_window.csv',allraw);(R/'model_inventory.json').write_text(json.dumps({n:{'mass':m,'com_in_link':c.tolist(),'inertia_in_link':I.tolist()} for n,(m,c,I) in pars.items()},indent=2))
t=[r['t_ms'] for r in rows];fig,ax=plt.subplots(6,1,figsize=(12,16),sharex=True)
for g in ['total','body','legs','wheels']:ax[0].plot(t,[r[g+'_H_pitch'] for r in rows],label=g+' spin+orbital')
for k in ['body_spin_H_pitch','body_orbital_H_pitch']:ax[1].plot(t,[r[k] for r in rows],label=k)
for k in ['legs_relative_H_pitch','wheels_relative_H_pitch']:ax[2].plot(t,[r[k] for r in rows],label=k+' (joint motion)')
ax[3].plot(t,[r['pitch_rate'] for r in rows],label='measured native pitch_rate');ax[3].plot(t,[r['pitch_rate_predicted_constant_H'] for r in rows],label='predicted from H(t0)+joint rates+inertia',linestyle='--');ax[4].plot(t,[r['pitch_deg'] for r in rows],label='native pitch deg')
for j in joints:ax[5].plot(t,[r[j['name']+'_v'] for r in rows],label=j['name'])
for a in ax:a.grid(alpha=.2);a.legend(fontsize=8,ncol=2)
ax[-1].set_xlabel('ms from first permanently no-contact native frame');fig.suptitle('D1 default control: native-engine momentum accounting, pitch axis = -world X');fig.tight_layout();fig.savefig(R/'momentum.png',dpi=130)
print(json.dumps({k:v for k,v in summary.items() if k not in ['first','last']},indent=2))
print('group deltas',{g:rows[-1][g+'_H_pitch']-rows[0][g+'_H_pitch'] for g in ['body','legs','wheels']});print('body spin delta',rows[-1]['body_spin_H_pitch']-rows[0]['body_spin_H_pitch'])
