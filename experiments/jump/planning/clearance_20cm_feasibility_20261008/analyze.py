"""Read-only existing-data engineering budget; no ROS, no parameter scan."""
from pathlib import Path
import csv,json,math,bisect,xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';OLD=ROOT/'clearance_apex_ab_20261008';O=ROOT/'clearance_20cm_feasibility_20261008'
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rows):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
results=[];tf,taxs=plt.subplots(3,2,figsize=(13,10));fig,axs=plt.subplots(3,2,figsize=(13,11))
for k,name in enumerate(['B1','B2','B3']):
 d=OLD/'physical'/name;m=json.loads((OLD/'analysis'/name/'metrics.json').read_text());rt=yaml.safe_load((d/'runtime_parameters.yaml').read_text())['/bbot_velocity_jump_controller']['ros__parameters'];meta={r['entity_name'].split('::')[-1]:r for r in read(d/'engine_frames.csv.inertials.csv')};M=sum(float(r['mass']) for r in meta.values());off=m['off_s'];offns=round(off*1e9);air=read(OLD/'analysis'/name/'flight.csv')[:-1];t=np.array([float(r['off_ms'])/1000 for r in air]);clr=np.array([float(r['bilateral_clearance']) for r in air]);v0=float(air[0]['com_vz']);z0=float(air[0]['com_z']);pitch_off=float(air[0]['pitch_deg']);rate_off=float(air[0]['pitch_rate']);ctl=read(d/'velocity_log.csv');th=[r for r in ctl if r['state_name']=='THRUST' and float(r['timestamp'])<off];lo=float(th[0]['timestamp']);lons=round(lo*1e9);target=math.sqrt(2*9.81*rt['jump_height']) if rt['takeoff_velocity']<=0 else rt['takeoff_velocity'];new=[]
 for c in [.20,.22]:
  vals=(c-clr[1:])/t[1:];idx=1+int(np.argmin(vals));dv=float(vals[idx-1]);dh=(2*v0*dv+dv*dv)/(2*9.81);gap=c-float(max(clr));energy_dv=math.sqrt(v0*v0+2*9.81*gap)-v0
  new.append({'target_clearance_m':c,'minimum_added_v_for_recorded_geometry':dv,'required_off_v':v0+dv,'certificate_frame_ms':t[idx]*1000,'additional_apex_height_m':dh,'additional_net_vertical_impulse_Ns':M*dv,'additional_kinetic_energy_J':.5*M*((v0+dv)**2-v0*v0),'fixed_peak_offset_energy_required_v':v0+energy_dv,'fixed_geometry_additional_net_impulse_Ns':M*energy_dv,'fixed_geometry_additional_energy_J':M*9.81*gap,'clearance_gap_m':gap})
 # Existing control publications, no assumptions about actual actuator realization.
 C=lambda field:np.array([float(r[field]) for r in th]);ctl_summary={'control_start_s':lo,'control_window_s':off-lo,'target_v':target,'max_published_knee_reference_speed':max(abs(C('knee_vel_cmd_left'))),'fraction_knee_ref_at_15':float(np.mean(abs(C('knee_vel_cmd_left'))>=14.999)),'force_command_max_per_leg':.5*max(C('F_z')),'force_request_max_per_leg':max(C('F_z_request')),'force_limit_min_per_leg':min(C('F_z_limit')),'force_limit_max_per_leg':max(C('F_z_limit')),'force_before_budget_max':max(C('thrust_force_before_budget')),'fraction_force_budget_limited':float(np.mean(C('thrust_force_before_budget')>C('F_z_limit')+1e-4)),'extension_scale_min':min(C('thrust_extension_scale')),'release_first_s':next((float(r['timestamp']) for r in th if r['thrust_release_active']=='1'),None),'thrust_motion_elapsed_last':float(th[-1]['thrust_motion_elapsed']),'hip_effort_limit':max(C('hip_effort_limit')),'knee_effort_limit':max(C('knee_effort_limit'))}
 # Native before-physics execution input and joint q/dq, retaining all physical steps.
 inp=[]
 for r in csv.DictReader((d/'native_wrench.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>offns:break
  if lons<=ns<offns and r['joint_name'] in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:
   assert r['before_physics_joint_force_cmd_valid']=='1' and r['before_physics_joint_state_valid']=='1'
   inp.append({'sim_time_ns':ns,'joint':r['joint_name'],'q':float(r['before_physics_joint_position']),'dq':float(r['before_physics_joint_velocity']),'actual_input_Nm':float(r['before_physics_joint_force_cmd_sim_input']),'aggregate_load_Nm':float(r['transmitted_axis_torque']) if r['wrench_valid']=='1' else float('nan')})
 dump(O/(name+'_thrust_inputs.csv'),inp)
 joints={}
 for n in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:
  rr=[r for r in inp if r['joint']==n];joints[n]={'q_min':min(r['q'] for r in rr),'q_max':max(r['q'] for r in rr),'dq_abs_max':max(abs(r['dq']) for r in rr),'actual_input_abs_max_Nm':max(abs(r['actual_input_Nm']) for r in rr),'fraction_actual_input_ge_142p5':float(np.mean([abs(r['actual_input_Nm'])>=142.499 for r in rr])),'fraction_actual_input_ge_149p9':float(np.mean([abs(r['actual_input_Nm'])>=149.9 for r in rr])),'actual_input_power_abs_max_W':max(abs(r['actual_input_Nm']*r['dq']) for r in rr)}
 # Total COM and all original physical link states in THRUST, for ground impulse closure.
 raw={}
 for r in csv.DictReader((d/'engine_frames.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>offns:break
  if lons<=ns<=offns and r['phase']=='after_step' and r['entity_type']=='link':raw.setdefault(ns,{})[r['entity_name'].split('::')[-1]]=r
 com={};rates={}
 for ns,rr in raw.items():
  assert set(meta)<=set(rr);p=np.zeros(3);v=np.zeros(3)
  for n,mm in meta.items():
   r=rr[n];R=Rotation.from_quat([float(r['quat_'+a]) for a in ['x','y','z','w']]);c=R.apply([float(mm['c'+a]) for a in 'xyz']);pp=np.array([float(r['position_'+a]) for a in 'xyz'])+c;vv=np.array([float(r['linear_v'+a]) for a in 'xyz'])+np.cross([float(r['angular_v'+a]) for a in 'xyz'],c);p+=float(mm['mass'])*pp;v+=float(mm['mass'])*vv
  com[ns]=(p/M,v/M);rates[ns]=-float(rr['base_link']['angular_vx'])
 forces={};vertical_moment_impulse=0.;horizontal_moment_impulse=0.;lever_samples=[]
 for r in csv.DictReader((d/'contact_detail.csv').open()):
  ns=int(r['sim_time_ns'])
  if ns>offns:break
  if ns<lons:continue
  assert r['time_valid']=='1' and r['feature_valid']=='1'
  forces.setdefault(ns,np.zeros(3))
  if int(r['index'])<0:continue
  assert r['extra_valid']=='1' and r['point_valid']=='1';one='::bbot::' in r['collision1'];two='::bbot::' in r['collision2'];assert one!=two
  ff=(1 if one else -1)*np.array([float(r[a]) for a in ['fx1','fy1','fz1']]);forces[ns]+=ff
  if ns in com:
   arm=np.array([float(r[a]) for a in ['px','py','pz']])-com[ns][0];vertical_moment_impulse+=-arm[1]*ff[2]*.001;horizontal_moment_impulse+=arm[2]*ff[1]*.001
   if ff[2]>1:lever_samples.append(-arm[1])
 ns0=min(com);ns1=max(com);seq=[ns for ns in sorted(forces) if ns0<ns<=ns1];imp=sum(forces[ns][2]*.001 for ns in seq);dt=(ns1-ns0)*1e-9;net=imp-M*9.81*dt;actual=M*(com[ns1][1][2]-com[ns0][1][2]);closure=net-actual
 # Conservative actuator force-budget opportunity, not actual contact force.
 # Only control samples before physical lift-off; does not reuse air force.
 ctrl_t=np.array([float(r['timestamp']) for r in th]);headroom=0.;active_duration=0.
 for a,b in zip(ctrl_t[:-1],ctrl_t[1:]):
  i=np.searchsorted(ctrl_t,a);r=th[i];h=max(0.,float(r['F_z_limit'])-.5*float(r['F_z']));headroom+=2*h*(b-a)
  if float(r['thrust_extension_scale'])>.999 and r['thrust_release_active']=='0' and r['thrust_attitude_blocked']=='0':active_duration+=b-a
 for row in new:
  row['added_average_contact_force_if_same_duration_N']=row['additional_net_vertical_impulse_Ns']/dt;row['added_potential_control_force_each_leg_before_attenuation_N']=.5*(float(rt['body_mass'])+8)*float(rt['thrust_velocity_kp'])*row['minimum_added_v_for_recorded_geometry'];row['ballistic_touchdown_energy_extra_J']=row['additional_kinetic_energy_J']
 # Single proposed design point, not a parameter scan: jump_height .25 -> .32.
 proposal_target=math.sqrt(2*9.81*.32);target_delta=proposal_target-target;root=ET.parse(d/'actual_robot.urdf').getroot();jmap={j.attrib['name']:j for j in root.findall('joint')};rot=lambda a,v:Rotation.from_rotvec([a,0,0]).apply(v);perp=lambda v:np.array([0.,-v[2],v[1]]);pros=[];old_inverse_errors=[]
 for r in th:
  if r['thrust_release_active']=='1' or r['thrust_attitude_blocked']=='1' or float(r['thrust_motion_elapsed'])<=0:continue
  jac=0.;other_hip=0.;Cbody=float(rt['body_mass'])*np.array([.20001846,.13261282,.05396677]);axle=np.zeros(3)
  assert r['wheels_airborne']=='0' and float(r['thrust_extension_scale'])==1.
  for side,kn,wn,sn in [('left','link_003_joint','link_004_joint','link_003'),('right','link_006_joint','link_007_joint','link_006')]:
   qh=float(r['hip_pos_'+side]);qk=float(r['knee_pos_'+side]);sh=rot(qh+qk,np.array([float(meta[sn]['c'+ax]) for ax in 'xyz']));ww=rot(qh+qk,np.array(list(map(float,jmap[wn].find('origin').attrib['xyz'].split()))));vertical=np.array([0.,-math.sin(float(r['pitch'])),math.cos(float(r['pitch']))]);jac+=float(vertical@((.8*perp(sh)+2*perp(ww))/(float(rt['body_mass'])+8)-.5*perp(ww)))
   hj='link_002_joint' if side=='left' else 'link_005_joint';hip=np.array(list(map(float,jmap[hj].find('origin').attrib['xyz'].split())));kk=rot(qh,np.array(list(map(float,jmap[kn].find('origin').attrib['xyz'].split()))));thigh=rot(qh,np.array([float(meta['link_002' if side=='left' else 'link_005']['c'+a]) for a in 'xyz']));wheel_com=np.array([float(meta['link_004' if side=='left' else 'link_007']['c'+a]) for a in 'xyz']);Cbody+=1.2*(hip+thigh)+.8*(hip+kk+sh)+2*(hip+kk+ww+wheel_com);axle+=.5*(hip+kk+ww);jh=float(vertical@((1.2*perp(thigh)+.8*perp(kk+sh)+2*perp(kk+ww))/(float(rt['body_mass'])+8)-.5*perp(kk+ww)));other_hip+=jh*float(r['hip_vel_cmd_'+side])
  assert jac<0
  ramp=min(1,float(r['thrust_motion_elapsed'])/.06);relative=rot(-float(r['pitch']),Cbody/(float(rt['body_mass'])+8)-axle);other=other_hip-relative[1]*float(r['active_jump_pitch_rate_ref']);old_raw=(ramp*target-other)/jac;pred=(ramp*proposal_target-other)/jac;old_inverse_errors.append(abs(old_raw-float(r['knee_vel_cmd_left'])));pros.append(abs(pred))
 same_geometry_proposal_clr=float(max(clr+target_delta*t));constant_loss_off_v=v0+target_delta;old_touch_v=abs(float(air[-1]['com_vz']));new_touch_v=math.sqrt(old_touch_v**2+constant_loss_off_v**2-v0**2);extra_energy=.5*M*(constant_loss_off_v**2-v0**2)
 offlinks=raw[offns];off_com=com[offns][0];Ixx=0.
 for n,mm in meta.items():
  r=offlinks[n];Q=Rotation.from_quat([float(r['quat_'+a]) for a in ['x','y','z','w']]).as_matrix();c=Q@np.array([float(mm['c'+a]) for a in 'xyz']);rrr=np.array([float(r['position_'+a]) for a in 'xyz'])+c-off_com;Ii=np.array([[float(mm[a]) for a in row] for row in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]]);Ixx+=(Q@Ii@Q.T)[0,0]+float(mm['mass'])*(rrr[1]**2+rrr[2]**2)
 mean_lever=vertical_moment_impulse/imp
 proposal={'proportional_existing_vertical_contact_impulse_pitch_rate_sensitivity':mean_lever*M*target_delta/Ixx,'actual_Fz_impulse_weighted_pitch_lever_m':mean_lever,'native_off_locked_Ixx':Ixx,'jump_height':.32,'takeoff_velocity':0.,'target_v':proposal_target,'target_increment':target_delta,'fixed_old_pre_release_window_candidate_raw_knee_reference_abs_max':max(pros),'old_inverse_reconstruction_error_abs_max':max(old_inverse_errors),'predicted_clearance_if_off_v_tracks_target_increment':same_geometry_proposal_clr,'off_v_under_constant_absolute_target_loss':constant_loss_off_v,'additional_net_impulse_under_same_assumption_Ns':M*target_delta,'additional_energy_J':extra_energy,'historical_precontact_com_vz_abs':old_touch_v,'new_impact_speed_same_touch_height_and_constant_loss':new_touch_v,'impact_speed_increase_ratio':new_touch_v/old_touch_v-1,'additional_average_stopping_force_over_8cm_N':extra_energy/.08,'conditional_added_early_feedback_per_leg_N':.5*(float(rt['body_mass'])+8)*float(rt['thrust_velocity_kp'])*target_delta,'force_slew_time_for_feedback_increment_ms':1000*.5*(float(rt['body_mass'])+8)*float(rt['thrust_velocity_kp'])*target_delta/1600}
 record={'single_design_point_conditional':proposal,'run':name,'mass_kg_native':M,'controller_mass_kg':float(rt['body_mass'])+8,'off_v_native':v0,'off_pitch_deg':pitch_off,'off_rate_native':rate_off,'off_z_native':z0,'ballistic_height_from_v_m':v0*v0/(2*9.81),'measured_com_rise_m':m['com_rise_from_off_m'],'native_ballistic_residual_max_m':float(max(abs(np.array([float(r['com_z']) for r in air])-(z0+v0*t-.5*9.81*t*t)))),'clearance_m':max(clr),'requirements':new,'thrust_control':ctl_summary,'native_joint_window':joints,'actual_vertical_force_pitch_impulse_after_COM_Nms':vertical_moment_impulse,'actual_horizontal_force_pitch_impulse_after_COM_Nms':horizontal_moment_impulse,'actual_vertical_pitch_lever_range_m':[min(lever_samples),max(lever_samples)],'actual_peak_ground_vertical_force_N':max(v[2] for v in forces.values()),'ground_vertical_contact_impulse_Ns':imp,'ground_net_vertical_impulse_Ns':net,'native_momentum_change_Ns':actual,'vertical_impulse_closure_Ns':closure,'same_thrust_duration_s':dt,'force_budget_headroom_integral_Ns_not_contact':headroom,'unattenuated_release_inactive_control_duration_s':active_duration,'existing_landing':{key:m[key] for key in ['contact_pitch_deg','contact_rate','max_backward_axle_m','full_stability_start_s','emergency']},'effective_runtime_parameters':{key:rt[key] for key in ['jump_height','takeoff_velocity','thrust_duration','thrust_peak_ratio','thrust_velocity_kp','thrust_release_velocity_ratio','thrust_knee_velocity_limit','sim_relax_thrust_limits','landing_capture_gain']}}
 
 for col,(part,joint) in enumerate([('hip','link_002_joint'),('knee','link_003_joint')]):
  rr=[r for r in inp if r['joint']==joint];tx=[(r['sim_time_ns']-offns)/1e6 for r in rr];ax=taxs[k,col];ax.plot([(float(r['timestamp'])-off)*1000 for r in th],[float(r['actual_tau_'+part+'_left']) for r in th],label='published command',ls='--');ax.plot(tx,[r['actual_input_Nm'] for r in rr],label='before-step actual input');ax.plot(tx,[r['aggregate_load_Nm'] for r in rr],label='after-step aggregate load',alpha=.7);ax.set_ylabel(name+' '+part+' Nm');ax.grid(alpha=.2);ax.legend(fontsize=7)
 results.append(record);print(json.dumps(record,ensure_ascii=False),flush=True)
 axs[k,0].plot(t*1000,clr*100,label='MEASURED same-frame min clearance');axs[k,0].plot(t*1000,(clr+new[0]['minimum_added_v_for_recorded_geometry']*t)*100,ls='--',label='MODEL same geometry + minimum delta vz');axs[k,0].axhline(20,color='k',ls=':');axs[k,0].set_ylabel(name+' clearance cm');axs[k,0].legend(fontsize=7);axs[k,0].grid(alpha=.2)
 for j,n in enumerate(['link_002_joint','link_003_joint']):
  rr=[r for r in inp if r['joint']==n];axs[k,1].plot([(r['sim_time_ns']-offns)/1e6 for r in rr],[r['actual_input_Nm'] for r in rr],label=n+' actual input')
 axs[k,1].set_ylabel(name+' physical input Nm');axs[k,1].legend(fontsize=7);axs[k,1].grid(alpha=.2)
for ax in axs[-1]:ax.set_xlabel('ms from native sustained no-contact frame')
tf.tight_layout();tf.savefig(O/'three_torque_types.png',dpi=130);fig.tight_layout();fig.savefig(O/'ballistic_and_thrust.png',dpi=130);(O/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
