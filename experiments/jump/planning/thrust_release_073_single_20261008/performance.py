"""One-factor performance audit. Native states and recorded commands only."""
from pathlib import Path
import csv,json,math,bisect,sys,xml.etree.ElementTree as E,re
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008';OUT=ROOT/'analysis';OUT.mkdir(exist_ok=True)
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rows):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def num(r,k):return float(r[k])
def pitch(g):return -Rotation.from_quat([num(g,'base_'+k) for k in ['qx','qy','qz','qw']]).as_euler('xyz')[0]
def continuous(t,good,span=1.):
 start=None
 for x,b in zip(t,good):
  if b:
   if start is None:start=x
   if x-start>=span-1e-9:return float(start)
  else:start=None
 return None
for name in sys.argv[1:] or ['R1']:
 d=ROOT/'physical_runs'/name
 if not (d/'run_result.json').exists():continue
 out=OUT/name;out.mkdir(exist_ok=True);run=json.loads((d/'run_result.json').read_text());ctrl=read(d/'velocity_log.csv');frames=read(d/'ground_frames.csv');geom=read(d/'geometry.csv');fl=[r for r in ctrl if r['state_name']=='FLIGHT'];assert fl
 entry=num(fl[0],'timestamp');last=max(int(r['sim_time_ns']) for r in frames if int(r['num_contacts'])>0 and int(r['sim_time_ns'])/1e9<entry);off=last+1000000;td=next(int(r['sim_time_ns']) for r in frames if int(r['sim_time_ns'])>off and int(r['num_contacts'])>0);end=td+20_000_000_000;gg={int(r['sim_time_ns']):r for r in geom};fm={int(r['sim_time_ns']):r for r in frames};assert all(fm[t]['frame_valid']=='1' and int(fm[t]['num_contacts'])==0 for t in range(off,td,1000000));assert end in gg or end-1000000 in gg
 rt=[num(r,'timestamp') for r in ctrl];command=lambda ns:ctrl[max(0,bisect.bisect_right(rt,ns/1e9)-1)]
 urdf=E.parse(d/'actual_robot.urdf').getroot();meta={r['entity_name'].split('::')[-1]:r for r in csv.DictReader((d/'engine_frames.csv.inertials.csv').open())};assert len(meta)==7;M=sum(num(m,'mass') for m in meta.values());wheelgeo={}
 for n in ['link_004','link_007']:
  l=urdf.find("link[@name='"+n+"']");c=l.find('collision');origin=c.find('origin');xyz=np.array([float(x) for x in origin.attrib['xyz'].split()]);Q=Rotation.from_euler('xyz',[float(x) for x in origin.attrib['rpy'].split()]);cy=c.find('geometry/cylinder');wheelgeo[n]=(xyz,Q.apply([0,0,1]),float(cy.attrib['radius']),float(cy.attrib['length'])/2)
 world=E.parse(W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/momentum_world.sdf').getroot().find('world');ground=world.find("model[@name='ground_plane']");assert ground.find('pose') is None and ground.find('link/pose') is None and ground.find('link/collision/pose') is None;assert ground.findtext('link/collision/geometry/plane/normal')=='0 0 1';ground_z=0.0
 native={};rates={};inputs=[];pre_ns=round(float(run['pre_jump_row']['timestamp'])*1e9);pre_Cz=0.0;pre_mass=0.0
 for r in csv.DictReader((d/'engine_frames.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>td:break
  if ns==pre_ns and r['phase']=='after_step' and r['entity_type']=='link':
   m=meta[r['entity_name'].split('::')[-1]];Q=Rotation.from_quat([num(r,'quat_'+k) for k in ['x','y','z','w']]);pre_Cz+=num(m,'mass')*(num(r,'position_z')+Q.apply([num(m,'c'+k) for k in 'xyz'])[2]);pre_mass+=num(m,'mass')
  if ns<off or r['phase']!='after_step':continue
  assert r['time_valid']=='1' and r['entity_present']=='1';native.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
 assert len(native)==(td-off)//1000000+1
 rows=[]
 for ns,rr in native.items():
  C=np.zeros(3);V=np.zeros(3);wc={}
  for n,m in meta.items():
   r=rr[n];assert r['position_valid']=='1' and r['velocity_valid']=='1';Q=Rotation.from_quat([num(r,'quat_'+k) for k in ['x','y','z','w']]);p=np.array([num(r,'position_'+k) for k in 'xyz']);c=Q.apply([num(m,'c'+k) for k in 'xyz']);v=np.array([num(r,'linear_v'+k) for k in 'xyz']);om=np.array([num(r,'angular_v'+k) for k in 'xyz']);C+=num(m,'mass')*(p+c);V+=num(m,'mass')*(v+np.cross(om,c))
   if n in wheelgeo:
    o,ax,rad,half=wheelgeo[n];axis=Q.apply(ax);center=p+Q.apply(o);extent=rad*math.sqrt(max(0.,1-axis[2]**2))+half*abs(axis[2]);wc[n]=float(center[2]-extent-ground_z)
  C/=M;V/=M;rbody=rr['base_link'];pr=-num(rbody,'angular_vx');rates[ns]=pr;cmd=command(ns)
  row={'sim_time_ns':ns,'off_ms':(ns-off)/1e6,'contact_ms':(ns-td)/1e6,'com_z':C[2],'com_vz':V[2],'left_clearance':wc['link_004'],'right_clearance':wc['link_007'],'bilateral_clearance':min(wc.values()),'pitch_deg':math.degrees(pitch(gg[ns])),'pitch_rate':pr,'contact_count':fm[ns]['num_contacts']}
  for j in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint','link_004_joint','link_007_joint']:
   row[j+'_q']=num(rr[j],'joint_position_0');row[j+'_v']=num(rr[j],'joint_velocity_0')
  for k,v in cmd.items():row['ctrl_'+k]=v
  rows.append(row)
 dump(out/'flight.csv',rows)
 for r in csv.DictReader((d/'native_wrench.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>end:break
  if ns<off:continue
  if r['joint_index']=='0' and r['post_base_velocity_valid']=='1':rates[ns]=-num(r,'post_base_world_wx')
  if ns<=td:inputs.append(r)
 dump(out/'flight_inputs.csv',inputs)
 allstates=list(dict.fromkeys(r['state_name'] for r in ctrl));post=[g for g in geom if td<=int(g['sim_time_ns'])<=end];st=np.array([(int(g['sim_time_ns'])-td)/1e9 for g in post]);pp=np.array([pitch(g) for g in post]);pr=np.array([rates[int(g['sim_time_ns'])] for g in post]);assert np.isfinite(pr).all();c0=command(td);direction=np.array([num(c0,'jump_forward_axis_x'),num(c0,'jump_forward_axis_y')]);assert abs(np.linalg.norm(direction)-1)<1e-4
 axle=np.array([[.5*(num(g,'left_wheel_'+k)+num(g,'right_wheel_'+k)) for k in 'xy'] for g in post]);dis=(axle-axle[0])@direction
 landed=[r for r in ctrl if td/1e9<=num(r,'timestamp')<=end/1e9];ct=np.array([num(r,'timestamp')-td/1e9 for r in landed]);quiet=[abs(num(r,'pitch')-.03)<.04 and abs(num(r,'pitch_rate'))<.15 and abs(num(r,'capture_com_velocity'))<.08 and abs(num(r,'gazebo_world_z_dot'))<.03 for r in landed];qtime=continuous(ct,quiet)
 pre=run['pre_jump_row'];pretime=num(pre,'timestamp');pregeom=[g for g in geom if pretime-.5<=int(g['sim_time_ns'])/1e9<=pretime];eq=float(np.median([pitch(g) for g in pregeom]))-num(pre,'pitch')+.03;atime=continuous(st,(abs(pp-eq)<.04)&(abs(pr)<.15));bal=next((num(r,'timestamp')-td/1e9 for r in landed if r['state_name']=='BALANCE'),None)
 gaps=[];start=None
 for f in frames:
  ns=int(f['sim_time_ns'])
  if ns<=td or ns>end:continue
  if int(f['num_contacts'])==0:
   if start is None:start=ns
  elif start is not None:gaps.append([start,ns]);start=None
 if start is not None:gaps.append([start,end])
 secondary=[]
 for a,b in gaps:
  clr=max(min(num(gg[t],'left_wheel_z'),num(gg[t],'right_wheel_z'))-wheelgeo['link_004'][2] for t in range(a,b,1000000) if t in gg)
  if b-a>=10000000 and clr>.002:secondary.append({'after_touch_s':(a-td)/1e9,'duration_s':(b-a)/1e9,'clearance_proxy_m':clr})
 abnormal=[r for r in frames if off<=int(r['sim_time_ns'])<=end and any('ground_plane' in a+b and not('link_004_collision' in a+b or 'link_007_collision' in a+b) for a,b in json.loads(r['collision_pairs_json']))]
 air=rows[:-1];peak=max(air,key=lambda r:r['bilateral_clearance']);apex=max(air,key=lambda r:r['com_z']);tuck=next((r for r in fl if r['flight_subphase']=='1'),None);extend=next((r for r in fl if r['flight_subphase']=='2'),None)
 m={'run':name,'group':run['group'],'states':allstates,'stop_reason':run['stop_reason'],'off_s':off/1e9,'first_touch_s':td/1e9,'flight_ms':(td-off)/1e6,'tuck_ms':None if tuck is None else (num(tuck,'timestamp')-off/1e9)*1000,'extend_ms':None if extend is None else (num(extend,'timestamp')-off/1e9)*1000,'com_apex_ms':apex['off_ms'],'wheel_peak_ms':peak['off_ms'],'com_rise_from_off_m':apex['com_z']-rows[0]['com_z'],'max_com_world_z':apex['com_z'],'pre_jump_com_z':pre_Cz/pre_mass if abs(pre_mass-M)<1e-9 else None,'com_rise_from_pre_jump_m':apex['com_z']-pre_Cz/pre_mass if abs(pre_mass-M)<1e-9 else None,'bilateral_clearance_m':peak['bilateral_clearance'],'left_peak_clearance_m':max(r['left_clearance'] for r in air),'right_peak_clearance_m':max(r['right_clearance'] for r in air),'contact_pitch_deg':math.degrees(pitch(gg[td])),'contact_rate':rates[td],'air_pitch_min_deg':min(r['pitch_deg'] for r in air),'air_pitch_max_deg':max(r['pitch_deg'] for r in air),'air_max_abs_rate':max(abs(r['pitch_rate']) for r in air),'max_backward_pitch_deg':max(0.,-math.degrees(pp.min())),'max_backward_axle_m':max(0.,-float(dis.min())),'final_axle_displacement_m':float(dis[-1]),'attitude_recovery_start_s':atime,'full_stability_start_s':qtime,'balance_reentry_s':bal,'secondary_flight':secondary,'all_no_contact_intervals_after_touch_ns':gaps,'nonwheel_ground_contact':bool(abnormal),'emergency':'EMERGENCY' in allstates,'observation_s':float(st[-1]),'tuck_entry':tuck,'extend_entry':extend,'collision_geometry':{k:{'center_offset':v[0].tolist(),'axis':v[1].tolist(),'radius':v[2],'half_width':v[3]} for k,v in wheelgeo.items()}}
 (out/'metrics.json').write_text(json.dumps(m,indent=2));dump(out/'post_contact.csv',[{'contact_s':float(t),'pitch_deg':float(math.degrees(p)),'pitch_rate':float(v),'axle_displacement_m':float(x)} for t,p,v,x in zip(st,pp,pr,dis)])
 fig,axs=plt.subplots(5,1,figsize=(12,13),sharex=True);tt=[r['off_ms'] for r in rows]
 for k in ['left_clearance','right_clearance','bilateral_clearance']:axs[0].plot(tt,[100*r[k] for r in rows],label=k)
 axs[1].plot(tt,[100*(r['com_z']-rows[0]['com_z']) for r in rows],label='native COM rise from takeoff cm')
 for part,j in [('hip','link_002_joint'),('knee','link_003_joint')]:
  axs[2].plot(tt,[r[j+'_q'] for r in rows],label=part+' native q');axs[2].plot(tt,[float(r['ctrl_'+part+'_pos_cmd_left']) for r in rows],ls='--',label=part+' published qref');axs[3].plot(tt,[r[j+'_v'] for r in rows],label=part+' native dq');axs[3].plot(tt,[float(r['ctrl_'+part+'_vel_cmd_left']) for r in rows],ls='--',label=part+' published dqref')
 axs[4].plot(tt,[r['pitch_deg'] for r in rows],label='native pitch deg');axs[4].plot(tt,[r['pitch_rate'] for r in rows],label='native pitch rate rad/s')
 for ax in axs:
  if m['extend_ms'] is not None:ax.axvline(m['extend_ms'],color='orange',ls=':',label='EXTEND entry')
  ax.axvline(m['com_apex_ms'],color='black',ls=':',label='COM apex');ax.grid(alpha=.2);ax.legend(fontsize=7,ncol=3)
 axs[-1].set_xlabel('ms from first sustained no-contact native frame');fig.suptitle(name+' '+run['group']+' measured Gazebo');fig.tight_layout();fig.savefig(out/'flight.png',dpi=120);plt.close(fig)
 fig,axs=plt.subplots(4,1,figsize=(12,11),sharex=True)
 for ax,part,side,j in zip(axs,['hip','knee','hip','knee'],['left','left','right','right'],['link_002_joint','link_003_joint','link_005_joint','link_006_joint']):
  ni=[r for r in inputs if r['joint_name']==j];t=[(int(r['sim_time_ns'])-off)/1e6 for r in ni];ax.plot(tt,[float(r['ctrl_actual_tau_'+part+'_'+side]) for r in rows],label='published effort Nm');ax.plot(t,[num(r,'before_physics_joint_force_cmd_sim_input') if r['before_physics_joint_force_cmd_valid']=='1' else np.nan for r in ni],label='before-physics actual input Nm');ax.plot(t,[num(r,'transmitted_axis_torque') if r['wrench_valid']=='1' else np.nan for r in ni],label='after-physics aggregate load Nm');ax.set_ylabel(part+' '+side);ax.grid(alpha=.2);ax.legend(fontsize=7)
 axs[-1].set_xlabel('ms from sustained no-contact frame');fig.tight_layout();fig.savefig(out/'torques.png',dpi=120);plt.close(fig)
 print(name,'clearance cm',round(100*m['bilateral_clearance_m'],3),'extend/apex',m['extend_ms'],m['com_apex_ms'],'contact',m['contact_pitch_deg'],m['contact_rate'],'retreat cm',100*m['max_backward_axle_m'],'emergency',m['emergency'],flush=True)
