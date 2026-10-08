"""Single authorized physical trial vs saved T1; no simulation or control writes."""
from pathlib import Path
import csv,json,hashlib,math,bisect
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
NEW=D/'thrust_terminal_forward_single_20261008';OLD=D/'thrust_height_032_ab_20261008';O=NEW/'analysis'
def read(p):return list(csv.DictReader(p.open()))
def write(p,x):p.write_text(json.dumps(x,indent=2))
def csvout(p,rr):
    with p.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
info={};allsteps={};allflight={};details={};ctl={};allpost={};release_inputs={}
for n,root,budget in [('T1',OLD,D/'thrust_momentum_budget_20261008/T1'),('R1',NEW,O/'thrust_budget/R1')]:
    physical=root/'physical_runs'/n;m=json.loads((root/'analysis'/n/'metrics.json').read_text())
    dd=next(r for r in json.loads((root/'analysis/details.json').read_text()) if r['run']==n);details[n]=dd
    s=json.loads((budget/'summary.json').read_text());steps=read(budget/'steps.csv');allsteps[n]=steps
    flight=read(root/'analysis'/n/'flight.csv');allflight[n]=flight
    cc=read(physical/'velocity_log.csv');ctl[n]=cc;th=[r for r in cc if r['state_name']=='THRUST']
    start=s['THRUST_entry_first_log_s'];off=m['off_s'];td=m['first_touch_s'];offms=round((off-start)*1000)
    def budget_segment(lo,hi):
        a=next(r for r in steps if float(r['thrust_ms'])==lo);b=next(r for r in steps if float(r['thrust_ms'])==hi)
        rr=[r for r in steps if lo<float(r['thrust_ms'])<=hi]
        z={'delta_H':float(b['H_pitch'])-float(a['H_pitch']),'delta_vz':float(b['COM_vz'])-float(a['COM_vz'])}
        for k in ['moment_preCOM','normal_moment_preCOM','tangent_moment_preCOM','Fz','Fy']:z[k+'_integral']=sum(float(r[k])*.001 for r in rr)
        z['net_vertical_impulse']=z['Fz_integral']-17.51*9.81*(hi-lo)*.001
        z['angular_residual']=z['delta_H']-z['moment_preCOM_integral']
        return z
    rel=next(r for r in th if r['thrust_release_active']=='1');relt=float(rel['timestamp'])
    # Do not invert logged blend: log_data calls blend(this->now()), which can
    # be later than the sampled control timestamp (cpp7340).
    crossings={}
    for ratio in [.78]:
        c=next(r for r in th if r['thrust_com_valid']=='1' and float(r['thrust_motion_elapsed'])>0 and float(r['thrust_feedback_vz'])>=ratio*float(r['target_takeoff_velocity']))
        crossings[str(ratio)]={k:c[k] for k in ['timestamp','thrust_feedback_vz','thrust_com_stamp']}
    inp=[];first_applied=None
    for r in csv.DictReader((physical/'native_wrench.csv').open()):
        t=int(r['sim_time_ns'])/1e9
        if t>off:break
        if t<relt-.012 or r['joint_name']!='link_003_joint':continue
        assert r['before_physics_joint_force_cmd_valid']=='1'
        val=float(r['before_physics_joint_force_cmd_sim_input']);inp.append({'sim_time_ns':int(r['sim_time_ns']),'relative_release_ms':1000*(t-relt),'input_Nm':val})
        if first_applied is None and t>=relt and abs(val-float(rel['actual_tau_knee_left']))<.0001:first_applied=t
    assert first_applied is not None
    release_inputs[n]=inp;csvout(O/(n+'_release_native_input.csv'),inp)
    post=read(root/'analysis'/n/'post_contact.csv');allpost[n]=post
    phases={}
    for r in cc:
        if r['state_name']=='FLIGHT':phases.setdefault(r['flight_subphase'],float(r['timestamp']))
    stages={}
    bytime={int(r['sim_time_ns']):r for r in flight}
    for ph,t in phases.items():
        ns=round(t*1e9);r=bytime[ns]
        stages[ph]={'off_ms':1000*(t-off),'pitch_deg':float(r['pitch_deg']),'pitch_rate':float(r['pitch_rate'])}
    ground=read(physical/'ground_frames.csv');nonwheel=[]
    for r in ground:
        t=int(r['sim_time_ns'])/1e9
        if t<td:continue
        if any('ground_plane' in a+b and not('link_004_collision' in a+b or 'link_007_collision' in a+b) for a,b in json.loads(r['collision_pairs_json'])):nonwheel.append(t)
    firstpairs=json.loads(next(r for r in ground if int(r['sim_time_ns'])==round(td*1e9))['collision_pairs_json'])
    # Retain every no-contact interval; verify the old cylindrical proxy for
    # long intervals using actual native wheel pose and collision geometry.
    bounce=m['secondary_flight'];minclr={}
    windows=[(round((td+r['after_touch_s'])*1e9),round((td+r['after_touch_s']+r['duration_s'])*1e9)) for r in bounce]
    geo=m['collision_geometry']
    for r in csv.DictReader((physical/'engine_frames.csv').open()):
        ns=int(r['sim_time_ns'])
        if not windows or ns>max(b for a,b in windows):break
        if r['phase']!='after_step' or not any(a<=ns<b for a,b in windows):continue
        link=r['entity_name'].split('::')[-1]
        if link not in geo:continue
        assert r['position_valid']=='1' and r['velocity_valid']=='1'
        g=geo[link];Q=Rotation.from_quat([float(r['quat_'+k]) for k in ['x','y','z','w']]);ax=Q.apply(g['axis']);center=float(r['position_z'])+Q.apply(g['center_offset'])[2]
        clear=center-g['radius']*math.sqrt(max(0.,1-ax[2]**2))-g['half_width']*abs(ax[2]);minclr.setdefault(ns,{})[link]=float(clear)
    for event,(a,b) in zip(bounce,windows):
        vals=[min(v.values()) for ns,v in minclr.items() if a<=ns<b and len(v)==2]
        event['actual_collision_bilateral_peak_m']=max(vals) if vals else None
    info[n]={'THRUST_first_log_s':start,'off_s':off,'touch_s':td,'release_condition_first_log_s':relt,
        'release_relative_THRUST_ms':1000*(relt-start),'release_relative_off_ms':1000*(relt-off),
        'first_release_command_native_step_end_s':first_applied,
        'first_release_command_native_relative_THRUST_ms':1000*(first_applied-start),
        'first_release_command_native_relative_off_ms':1000*(first_applied-off),
        'fixed_record_threshold_crossings':crossings,'release_feedback_vz':float(rel['thrust_feedback_vz']),
        'THRUST':budget_segment(0,offms),'last20ms':budget_segment(offms-20,offms),
        'entry':s['whole']['initial'],'takeoff':s['whole']['final'],'stages':stages,
        'metrics':m,'touch_pre_vz':dd['COM_vz_touch_before'],'touch_post_vz':dd['COM_vz_touch_after'],
        'contact_normal_impulse_first50ms_Ns':dd['ground_impulse_first_50ms_Ns'],
        'contact_normal_peak_first300ms_N':dd['ground_Fz_peak_first_300ms_N'],
        'compression':dd['compression_0_to_300ms'],'first_touch_collision_pairs':firstpairs,
        'first_nonwheel_contact_after_touch_s':None if not nonwheel else min(nonwheel)-td,
        'subsequent_no_contact_verified':bounce,'final_state':cc[-1]['state_name']}
write(O/'comparison.json',info)

fig,axs=plt.subplots(5,2,figsize=(15,17))
for n in info:
    ss=[r for r in allsteps[n] if float(r['thrust_ms'])>=0];f=allflight[n];c='C0' if n=='T1' else 'C3'
    for col,key in enumerate(['thrust_ms','off_ms']):
        t=[float(r[key]) for r in ss];H=np.array([float(r['H_pitch']) for r in ss])
        axs[0,col].plot(t,H,color=c,label=n+' total H')
        J=np.cumsum([float(r['moment_preCOM'])*.001 if float(r['thrust_ms'])>0 else 0 for r in ss])
        axs[1,col].plot(t,H-H[0],color=c,label=n+' actual dH');axs[1,col].plot(t,J,color=c,ls='--',label=n+' contact integral')
        for k,ls in [('normal_moment_preCOM','--'),('tangent_moment_preCOM','-')]:
            q=np.cumsum([float(r[k])*.001 if float(r['thrust_ms'])>0 else 0 for r in ss]);axs[2,col].plot(t,q,color=c,ls=ls,label=n+' '+k.split('_')[0])
        axs[3,col].plot(t,[float(r['Fz']) for r in ss],color=c,label=n+' actual ground Fz')
        axs[4,col].plot(t,[float(r['COM_vz']) for r in ss],color=c,label=n+' actual COM vz')
        a=info[n]['release_relative_THRUST_ms' if col==0 else 'release_relative_off_ms']
        for ax in axs[:,col]:ax.axvline(a,color=c,ls=':',alpha=.6)
for i in range(5):
    for col in range(2):
        ax=axs[i,col];ax.grid(alpha=.2);ax.legend(fontsize=8)
        ax.set_ylabel(['H (kg m2/s)','dH / contact impulse (kg m2/s)','normal / tangent angular impulse','actual ground Fz (N)','actual COM vz (m/s)'][i]);ax.set_xlabel('ms from first THRUST log' if col==0 else 'ms from real liftoff')
fig.suptitle('MEASURED physics: T1 original vs one R1 terminal reference -.02 m/s; dotted lines = release-condition logs')
fig.tight_layout(rect=(0,0,1,0.975));fig.savefig(O/'contact_momentum_comparison.png',dpi=130);plt.close(fig)

fig,axs=plt.subplots(4,2,figsize=(14,13))
for n in info:
    f=allflight[n];t=[float(r['off_ms']) for r in f];c='C0' if n=='T1' else 'C3'
    axs[0,0].plot(t,[float(r['bilateral_clearance'])*100 for r in f],color=c,label=n)
    axs[0,1].plot(t,[float(r['com_z']) for r in f],color=c,label=n)
    axs[1,0].plot(t,[float(r['pitch_deg']) for r in f],color=c,label=n)
    axs[1,1].plot(t,[float(r['pitch_rate']) for r in f],color=c,label=n)
    pp=allpost[n];pt=[float(r['contact_s']) for r in pp]
    axs[2,0].plot(pt,[float(r['axle_displacement_m'])*100 for r in pp],color=c,label=n)
    axs[2,1].plot(pt,np.degrees(np.unwrap(np.radians([float(r['pitch_deg']) for r in pp]))),color=c,label=n)
    window=read((OLD if n=='T1' else NEW)/'analysis'/(n+'_physical_window.csv'));tt=[float(r['contact_ms']) for r in window]
    axs[3,0].plot(tt,[float(r['ground_fz']) for r in window],color=c,label=n)
    I=np.cumsum([float(r['ground_fz'])*.001 if 0<=float(r['contact_ms'])<50 else 0 for r in window]);axs[3,1].plot(tt,I,color=c,label=n)
for i in range(4):
    for col in range(2):
        ax=axs[i,col];ax.grid(alpha=.2);ax.legend()
        ax.set_ylabel([['bilateral collision clearance cm','actual COM world z m'],['native pitch deg','native pitch rate rad/s'],['axle displacement from first touch cm','post-contact unwrapped pitch deg'],['actual ground Fz N','normal impulse first 50 ms N s']][i][col])
        ax.set_xlabel('ms from real liftoff' if i<2 else 's from real first touch' if i==2 else 'ms from real first touch')
        if i==3:ax.set_xlim(-20,100)
axs[0,0].axhline(20,color='k',ls=':');fig.suptitle('MEASURED one-run comparison: clearance and landing must both pass')
fig.tight_layout(rect=(0,0,1,0.975));fig.savefig(O/'flight_landing_comparison.png',dpi=130);plt.close(fig)

fig,axs=plt.subplots(2,2,figsize=(13,9))
for row,n in enumerate(['T1','R1']):
    cc=[r for r in ctl[n] if r['state_name']=='THRUST'];start=info[n]['THRUST_first_log_s'];off=info[n]['off_s'];ii=read((OLD if n=='T1' else NEW)/'analysis'/(n+'_thrust_inputs.csv'))
    for col,(part,j) in enumerate([('hip','link_002_joint'),('knee','link_003_joint')]):
        ax=axs[row,col];a=[r for r in ii if r['joint']==j];t=[(int(r['sim_time_ns'])/1e9-off)*1000 for r in a]
        ax.plot([(float(r['timestamp'])-off)*1000 for r in cc],[float(r['actual_tau_'+part+'_left']) for r in cc],ls='--',label='published command')
        ax.plot(t,[float(r['actual_input']) for r in a],label='before-step actual input')
        ax.plot(t,[float(r['aggregate_load']) for r in a],label='after-step aggregate load')
        ax.set_ylabel(n+' '+part+' Nm');ax.set_xlabel('ms from real liftoff');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout(rect=(0,0,1,0.975));fig.savefig(O/'three_torque_types_comparison.png',dpi=130);plt.close(fig)

fig,axs=plt.subplots(4,2,figsize=(14,12))
for col,n in enumerate(['T1','R1']):
    rr=[r for r in allsteps[n] if -60<=float(r['off_ms'])<=0];t=[float(r['off_ms']) for r in rr]
    for i,(k,label) in enumerate([('Fz','actual ground Fz N'),('Fy','actual forward ground Fy N'),('moment_preCOM','actual contact pitch moment Nm')]):
        axs[i,col].plot(t,[float(r[k]) for r in rr],label=n+' '+label);axs[i,col].set_ylabel(label)
    cop=[1000*(float(r['COP_y'])-float(r['COM_y'])) for r in rr]
    axs[3,col].plot(t,cop,label=n+' Fz-weighted contact y minus post-step COM y');axs[3,col].set_ylabel('contact lever mm (post-step COM)')
    for ax in axs[:,col]:
        ax.axvline(info[n]['release_relative_off_ms'],color='orange',ls=':',label='condition / publication')
        ax.axvline(info[n]['first_release_command_native_relative_off_ms'],color='green',ls='--',label='first actual-input frame')
        ax.grid(alpha=.2);ax.legend(fontsize=7);ax.set_xlabel('ms from real liftoff')
fig.suptitle('Actual contact response: no unique force-decline onset inferred from oscillating contact force')
fig.tight_layout(rect=(0,0,1,0.975));fig.savefig(O/'release_contact_detail.png',dpi=130);plt.close(fig)

frozen=json.loads((NEW/'baseline_frozen_sha256.json').read_text());assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in frozen.items())
raw={str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (NEW/'physical_runs').rglob('*') if p.is_file()}
write(NEW/'raw_sha256.json',raw)
m=info['R1']['metrics']
normal=not m['emergency'] and not m['nonwheel_ground_contact'] and not m['secondary_flight'] and m['tuck_ms'] is not None and m['extend_ms'] is not None
experiment=json.loads((NEW/'experiment_frozen_sha256.json').read_text())
assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in experiment.items())
write(NEW/'provenance.json',{'frozen_count':len(frozen),'frozen_unchanged':True,'experiment_frozen_unchanged':True,'physical_jump_runs':1,'airborne':1,'combined_accepted':None,'performance_metrics_valid':normal,'clearance_over20cm':m['bilateral_clearance_m']>=.20,'startup_failures_without_jump':0,'raw_count':len(raw),'runtime_control_parameter_difference':{},'experiment_code_delta_reference_mps':.02,'default_changed':False,'additional_runs':0,'source_sha256':{str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (W/'experiments/jump/planning/thrust_terminal_forward_single_20261008').glob('*.py')}})
print(json.dumps({n:{'release_ms':info[n]['release_relative_THRUST_ms'],'native_ms':info[n]['first_release_command_native_relative_THRUST_ms'],'H_off':info[n]['takeoff']['H_pitch'],'vz_off':info[n]['takeoff']['COM_vz'],'THRUST':info[n]['THRUST'],'last20':info[n]['last20ms'],'stages':info[n]['stages']} for n in info},indent=2))
