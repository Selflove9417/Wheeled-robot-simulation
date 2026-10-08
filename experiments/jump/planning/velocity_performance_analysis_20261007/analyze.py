import csv,json,math,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new'); P=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/velocity_clearance_landing_20261005_224308'; O=W/'experiments/jump/planning/velocity_performance_analysis_20261007'
rows=list(csv.DictReader((P/'velocity_log.csv').open()))
def f(r,k):
 try:return float(r[k])
 except (ValueError,KeyError):return math.nan
def near(t):return min(rows,key=lambda r:abs(f(r,'timestamp')-t))
def extrema(rs,k):
 a=[(f(r,k),f(r,'timestamp')) for r in rs if math.isfinite(f(r,k))];return {'min':min(a),'max':max(a)} if a else None
th=[r for r in rows if r['state_name']=='THRUST']; fl=[r for r in rows if r['state_name']=='FLIGHT']; landing=[r for r in rows if 4.395<=f(r,'timestamp')<=6.195]
trans=[];prev=None
for r in rows:
 key=(r['state_name'],r['flight_subphase'] if r['state_name']=='FLIGHT' else r['touchdown_phase'] if r['state_name']=='TOUCHDOWN_BUFFER' else r['recovery_subphase'])
 if key!=prev:trans.append([f(r,'timestamp'),*key]);prev=key
s={'source_log':str(P.relative_to(W)),'transitions':trans,'thrust':{},'flight':{},'landing':{},'native':{}}
keys=['hip_pos_left','hip_pos_right','knee_pos_left','knee_pos_right','hip_vel_left','hip_vel_right','knee_vel_left','knee_vel_right','hip_vel_cmd_left','knee_vel_cmd_left','pitch','pitch_rate','pitch_rate_raw','actual_tau_hip_left','actual_tau_knee_left','cmd_x','air_wheel_cmd_raw','left_wheel_vel','right_wheel_vel','thrust_extension_scale','thrust_motion_elapsed','thrust_feedback_vz','com_world_vz','com_world_z','arrest_ff_applied_hl','arrest_ff_applied_kl','capture_com_velocity','com_lean','capture_world_target']
for name,rs in [('thrust',th),('flight',fl),('landing',landing)]:
 s[name]={k:extrema(rs,k) for k in keys}
 s[name]['samples']=len(rs)
 s[name]['hip_left_right_max_abs_rad']=max(abs(f(r,'hip_pos_left')-f(r,'hip_pos_right')) for r in rs)
 s[name]['knee_left_right_max_abs_rad']=max(abs(f(r,'knee_pos_left')-f(r,'knee_pos_right')) for r in rs)
 s[name]['effort_limit_hits']={j:sum(abs(f(r,'actual_tau_'+j+'_left'))>=f(r,j+'_effort_limit')-.01 for r in rs) for j in ['hip','knee']}
 s[name]['wheel_command_at_2mps_samples']=sum(abs(f(r,'cmd_x'))>=1.9999 for r in rs)
s['snapshots']=[{k:r[k] for k in ['timestamp','state_name','flight_subphase','touchdown_phase','pitch','pitch_rate','pitch_rate_raw','cmd_x','left_wheel_vel','right_wheel_vel','x','x_dot','target_x','touchdown_x_ref','capture_com_velocity','capture_world_target','com_world_vz','com_world_z','hip_pos_left','knee_pos_left','hip_pos_cmd_left','knee_pos_cmd_left','hip_vel_left','knee_vel_left']} for r in [near(t) for t in [4.108,4.297,4.337,4.397,4.497,4.687,4.697,4.751,4.899,4.929,4.988,5.202,5.799,6.194,7.923,9.228]]]
# Exact native geometry, no reconstructed wheel height.
g=[]
for r in csv.DictReader((P/'geometry.csv').open()):
 t=int(r['sim_time_ns'])*1e-9
 if 3.9<=t<=9.5:
  qx,qy,qz,qw=(float(r[k]) for k in ['base_qx','base_qy','base_qz','base_qw'])
  r['t']=t;r['pitch_native']=-math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));r['clearance']=min(float(r['left_wheel_z']),float(r['right_wheel_z']))-.07;r['axle_y']=.5*(float(r['left_wheel_y'])+float(r['right_wheel_y']));g.append(r)
contact=[]
for r in csv.DictReader((P/'ground_frames.csv').open()):
 t=int(r['sim_time_ns'])*1e-9
 if 4.1<t<5.0:contact.append((t,int(r['num_contacts'])))
air=[t for t,n in contact if n==0];contact_t=next(t for t,n in contact if t>4.4 and n>0)
s['native']['first_contact_s']=contact_t;s['native']['last_contact_before_flight_s']=max(t for t,n in contact if t<4.4 and n>0);s['native']['first_sustained_no_contact_s']=min(t for t in air if t>s['native']['last_contact_before_flight_s'])
peak=max(g,key=lambda r:r['clearance']);s['native']['peak_bilateral_clearance_m']=peak['clearance'];s['native']['peak_clearance_s']=peak['t']
gc=min(g,key=lambda r:abs(r['t']-contact_t));s['native']['contact_pitch_deg']=math.degrees(gc['pitch_native']);s['native']['axle_retreat_1p5s_m']=gc['axle_y']-min(r['axle_y'] for r in g if contact_t<=r['t']<=contact_t+1.5)
# Selected original native loads and physical-before leg inputs. Wheel velocity actuator has no JointForceCmd here.
native=[]
for r in csv.DictReader((P/'native_wrench.csv').open()):
 t=int(r['sim_time_ns'])*1e-9
 if t>6.2:break
 if t<3.9:continue
 native.append(r)
names=sorted({r['joint_name'] for r in native});s['native']['joint_input_valid_fraction']={n:sum(r['before_physics_joint_force_cmd_valid']=='1' for r in native if r['joint_name']==n)/sum(r['joint_name']==n for r in native) for n in names}
for n in names:
 rs=[r for r in native if r['joint_name']==n];s['native'][n+'_load_minmax']=[min(f(r,'transmitted_axis_torque') for r in rs if r['wrench_valid']=='1'),max(f(r,'transmitted_axis_torque') for r in rs if r['wrench_valid']=='1')]
# Keep an exact selected controller slice; add explicitly labelled errors, not fabricated torque.
fields=list(rows[0])+['position_error_x_minus_touchdown_ref','velocity_error_zero_minus_capture_com_velocity']
with (O/'landing_controller_window.csv').open('w') as h:
 w=csv.DictWriter(h,fieldnames=fields);w.writeheader()
 for r in landing:
  a=dict(r);a[fields[-2]]=f(r,'x')-f(r,'touchdown_x_ref');a[fields[-1]]=-f(r,'capture_com_velocity');w.writerow(a)
with (O/'analysis.json').open('w') as h:json.dump(s,h,indent=2)
hashes={str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [W/'src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp',W/'src/bbot_bringup/launch/bbot_gazebo.launch.py',W/'src/bbot_balance_controller/scripts/jump_profile.py',W/'src/bbot_bringup/config/bbot_controllers.yaml',P/'velocity_log.csv',P/'geometry.csv',P/'runtime_parameters.yaml',P/'launch_command.json']}
for p in [P/n for n in ['ground_frames.csv','native_wrench.csv','wheel_joints.csv','legs_joint_feedback.csv','leg_modes.csv','launch.log']]+[W/'build/bbot_balance_controller/bbot_velocity_jump_controller',W/'src/bbot_bringup/worlds/flat_jump_world.sdf',W/'src/bbot_bringup/worlds/native_command_observation_world.sdf']:
 if p.exists():
  digest=hashlib.sha256()
  with p.open('rb') as stream:
   for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
  hashes[str(p.relative_to(W))]=digest.hexdigest()
(O/'inputs_sha256.json').write_text(json.dumps(hashes,indent=2))
def curve(ax,rs,k,label=None,scale=1):ax.plot([f(r,'timestamp')-contact_t for r in rs],[scale*f(r,k) for r in rs],label=label or k,linewidth=1)
fig,axs=plt.subplots(5,1,figsize=(12,13),sharex=True)
curve(axs[0],landing,'pitch',scale=180/math.pi)
curve(axs[0],[r for r in landing if r['state_name']=='FLIGHT'],'flight_air_pitch_ref',label='flight_air_pitch_ref (FLIGHT only)',scale=180/math.pi)
axs[0].plot([r['t']-contact_t for r in g if 4.395<=r['t']<=6.195],[math.degrees(r['pitch_native']) for r in g if 4.395<=r['t']<=6.195],label='native pitch=-roll',linewidth=.8);axs[0].set_ylabel('deg')
for k in ['pitch_rate','pitch_rate_raw']:curve(axs[1],landing,k)
axs[1].set_ylabel('rad/s')
for k in ['cmd_x','capture_world_target','capture_com_velocity']:curve(axs[2],landing,k)
axs[2].set_ylabel('m/s')
for k in ['left_wheel_vel','right_wheel_vel']:curve(axs[3],landing,k)
axs[3].set_ylabel('rad/s')
for k in ['link_004_joint','link_007_joint']:
 rs=[r for r in native if r['joint_name']==k and 4.395<=int(r['sim_time_ns'])*1e-9<=6.195 and r['wrench_valid']=='1'];axs[4].plot([int(r['sim_time_ns'])*1e-9-contact_t for r in rs],[f(r,'transmitted_axis_torque') for r in rs],label=k+' aggregate joint load')
axs[4].set_ylabel('Nm (NOT command)')
for ax in axs:ax.axvline(0,color='k',linestyle='--');ax.grid(alpha=.25);ax.legend(fontsize=8,loc='best')
axs[-1].set_xlabel('seconds relative to native first wheel contact');fig.suptitle('RECORDED GAZEBO baseline: landing -0.3 to +1.5 s; wheel motor torque unavailable');fig.tight_layout();fig.savefig(O/'landing_window.png',dpi=150);plt.close(fig)
rs=[r for r in rows if 4.1<=f(r,'timestamp')<=4.7];fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
for k in ['hip_pos_left','hip_pos_right','hip_pos_cmd_left','knee_pos_left','knee_pos_right','knee_pos_cmd_left']:curve(axs[0],rs,k)
for k in ['hip_vel_left','hip_vel_cmd_left','knee_vel_left','knee_vel_cmd_left']:curve(axs[1],rs,k)
for k in ['com_world_vz','target_takeoff_velocity']:curve(axs[2],rs,k)
axs[3].plot([r['t']-contact_t for r in g if 4.1<=r['t']<=4.7],[r['clearance'] for r in g if 4.1<=r['t']<=4.7],label='native bilateral wheel clearance')
for ax in axs:ax.grid(alpha=.25);ax.legend(fontsize=8)
axs[0].set_ylabel('rad');axs[1].set_ylabel('rad/s');axs[2].set_ylabel('m/s');axs[3].set_ylabel('m');axs[3].set_xlabel('seconds relative to native first contact');fig.suptitle('RECORDED GAZEBO: thrust / flight targets and actual response');fig.tight_layout();fig.savefig(O/'takeoff_flight.png',dpi=150);plt.close(fig)
print(json.dumps({'native':s['native'],'transitions':trans,'thrust':s['thrust'],'flight':s['flight']},indent=2))

fig,axs=plt.subplots(2,2,figsize=(13,8),sharex=True)
for ax,(joint,label) in zip(axs.flat,[('link_002_joint','hip_left'),('link_003_joint','knee_left'),('link_005_joint','hip_right'),('link_006_joint','knee_right')]):
 nr=[r for r in native if r['joint_name']==joint]
 tx=[int(r['sim_time_ns'])*1e-9-contact_t for r in nr]
 ax.plot(tx,[f(r,'before_physics_joint_force_cmd_sim_input') if r['before_physics_joint_force_cmd_valid']=='1' else math.nan for r in nr],label='native BeforePhysics input',linewidth=.8)
 ax.plot(tx,[f(r,'transmitted_axis_torque') if r['wrench_valid']=='1' else math.nan for r in nr],label='native aggregate joint load',linewidth=.8,alpha=.6)
 cr=[r for r in rows if 3.9<=f(r,'timestamp')<=6.2]
 curve(ax,cr,'actual_tau_'+label,label='published effort command',scale=1)
 ax.set_title(label);ax.set_ylabel('Nm');ax.grid(alpha=.2);ax.legend(fontsize=8)
axs[1,0].set_xlabel('seconds relative to native first contact');axs[1,1].set_xlabel('seconds relative to native first contact')
fig.suptitle('RECORDED GAZEBO: command / BeforePhysics input / aggregate load (NOT motor torque)');fig.tight_layout();fig.savefig(O/'leg_torque_sources.png',dpi=150);plt.close(fig)

fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
for k in ['x','touchdown_x_ref','target_x']:curve(axs[0],landing,k)
axs[0].set_ylabel('wheel odometry m')
for k in ['x_error','position_error_x_minus_touchdown_ref']:curve(axs[1],list(csv.DictReader((O/'landing_controller_window.csv').open())),k)
axs[1].set_ylabel('m; diagnostic / raw')
ng=[r for r in g if 4.395<=r['t']<=6.195];axs[2].plot([r['t']-contact_t for r in ng],[r['axle_y']-gc['axle_y'] for r in ng],label='native axle world-y displacement from contact')
cr=min(rows,key=lambda r:abs(f(r,'timestamp')-contact_t))
axs[2].plot([f(r,'timestamp')-contact_t for r in landing],[f(r,'centroidal_world_y')-f(cr,'centroidal_world_y') for r in landing],label='observer COM world-y displacement (sampled)')
axs[2].set_ylabel('m; negative=backward')
phases=['FLIGHT','CATCH','REVERSE_BRAKE','PREPARE','HOLD','BRAKE']
axs[3].step([f(r,'timestamp')-contact_t for r in landing],[phases.index('FLIGHT' if r['state_name']=='FLIGHT' else r['touchdown_phase']) for r in landing],where='post',label='state / touchdown_phase')
axs[3].set_yticks(range(len(phases)),phases)
for ax in axs:ax.axvline(0,color='k',linestyle='--');ax.grid(alpha=.2);ax.legend(fontsize=8)
axs[-1].set_xlabel('seconds relative to native first contact');fig.suptitle('RECORDED GAZEBO: references, position errors, actual retreat, controller phases');fig.tight_layout();fig.savefig(O/'landing_references_phases.png',dpi=150);plt.close(fig)
