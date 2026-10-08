"""Compare saved THRUST budgets; fixed-record threshold check is not a rollout."""
from pathlib import Path
import csv, json, hashlib, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

W=Path('/home/xy/bbot_ws_new')
D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
O=Path(os.environ.get('THRUST_BUDGET_OUTPUT',str(D/'thrust_momentum_budget_20261008')))
summaries=json.loads((O/'summary.json').read_text())
rows={s['run']:list(csv.DictReader((O/s['run']/'steps.csv').open())) for s in summaries}
def at(name,ms):
    return next(r for r in rows[name] if float(r['thrust_ms'])==ms)
def segment(name,lo,hi):
    rr=[r for r in rows[name] if lo<float(r['thrust_ms'])<=hi]
    a,b=at(name,lo),at(name,hi)
    v={'delta_H':float(b['H_pitch'])-float(a['H_pitch']),
       'delta_vz':float(b['COM_vz'])-float(a['COM_vz'])}
    for k in ['moment_preCOM','normal_moment_preCOM','tangent_moment_preCOM','Fz','Fy']:
        v[k+'_integral']=sum(float(r[k])*.001 for r in rr)
    v['vertical_net_impulse']=v['Fz_integral']-17.51*9.81*(hi-lo)*.001
    v['angular_residual']=v['delta_H']-v['moment_preCOM_integral']
    return v
out={'runs':{},'pairwise_T1_minus_baseline':{}}
for s in summaries:
    n=s['run']; w=s['whole']; hi=round((w['end_s']-w['start_s'])*1000)
    out['runs'][n]={'entry_H':w['initial']['H_pitch'],'off_H':w['final']['H_pitch'],
                    'entry_bracket_ms':1000*(s['THRUST_entry_first_log_s']-s['previous_SQUAT_log_s']),
                    'off_vz':w['final']['COM_vz'],'whole':segment(n,0,hi),
                    'late_150_to_off':segment(n,150,hi),
                    'entry_bracket_alternative':s['entry_bracket_alternative']}
for n in ['B1','B2','B3']:
    a,b=out['runs']['T1'],out['runs'][n]
    out['pairwise_T1_minus_baseline'][n]={k:a[k]-b[k] for k in ['entry_H','off_H','off_vz']}
    out['pairwise_T1_minus_baseline'][n]['THRUST_budget_difference']={k:a['whole'][k]-b['whole'][k] for k in a['whole']}
out['T1_last_20ms']=segment('T1',161,181)
out['T1_at_minus20ms']={k:float(at('T1',161)[k]) for k in ['H_pitch','COM_vz','COM_z','pitch_deg','pitch_rate','Fy','Fz','contact_points']}
raw=D/'thrust_height_032_ab_20261008/physical_runs/T1'
ctl=[r for r in csv.DictReader((raw/'velocity_log.csv').open()) if r['state_name']=='THRUST']
# Sole analytic candidate: the recorded 1.83477/2.50567 crossing, rounded down
# to 0.73. No parameter scan and no counterfactual physical state prediction.
target=float(ctl[0]['target_takeoff_velocity'])
def crossing(ratio):
    return next(r for r in ctl if float(r['thrust_motion_elapsed'])>0 and
                r['thrust_com_valid']=='1' and float(r['thrust_feedback_vz'])>=ratio*target)
out['fixed_record_release_check']={'target':target,'baseline_ratio':.78,'candidate_ratio':.73,
   'baseline_threshold':.78*target,'candidate_threshold':.73*target,
   'baseline_crossing':{k:crossing(.78)[k] for k in ['timestamp','thrust_feedback_vz','thrust_com_stamp','thrust_release_active']},
   'candidate_crossing':{k:crossing(.73)[k] for k in ['timestamp','thrust_feedback_vz','thrust_com_stamp','thrust_release_active']},
   'not_a_physical_prediction':True}
out['fixed_record_release_check']['advance_ms']=1000*(float(crossing(.78)['timestamp'])-float(crossing(.73)['timestamp']))
(O/'comparison.json').write_text(json.dumps(out,indent=2))

fig,axes=plt.subplots(4,2,figsize=(14,13))
for n in rows:
    rr=[r for r in rows[n] if float(r['thrust_ms'])>=0]
    t=np.array([float(r['thrust_ms']) for r in rr]); H=np.array([float(r['H_pitch']) for r in rr])
    for col,timekey in enumerate(['thrust_ms','off_ms']):
        x=[float(r[timekey]) for r in rr]
        axes[0,col].plot(x,H,label=n)
        axes[1,col].plot(x,[float(r['COM_vz']) for r in rr],label=n)
        color={'B1':'C0','B2':'C1','B3':'C2','T1':'C3'}[n]
        for key,style,label in [('normal_moment_preCOM','--','normal'),('tangent_moment_preCOM','-','tangent')]:
            J=np.cumsum([float(r[key])*.001 if float(r['thrust_ms'])>0 else 0 for r in rr])
            axes[2,col].plot(x,J,ls=style,color=color,label=n+' '+label)
        J=np.cumsum([float(r['moment_preCOM'])*.001 if float(r['thrust_ms'])>0 else 0 for r in rr])
        axes[3,col].plot(x,H-H[0]-J,label=n)
for col in range(2):
    for i in range(4):
        ax=axes[i,col];ax.grid(alpha=.25);ax.legend(fontsize=7,ncol=2)
        ax.set_ylabel(['Total pitch H (kg m2/s)','Actual COM vz (m/s)','Contact angular impulse (kg m2/s)','dH - contact integral (kg m2/s)'][i])
        ax.set_xlabel('ms from first THRUST record' if col==0 else 'ms from real liftoff')
        ax.axvline(161 if col==0 else -20,color='gray',ls=':',alpha=.6)
fig.suptitle('Saved native physics only: contact moments about pre-step COM; pitch axis -world X')
fig.tight_layout();fig.savefig(O/'primary_budget.png',dpi=135);plt.close(fig)

fig,axs=plt.subplots(3,1,figsize=(12,10),sharex=True)
for n in rows:
    rr=[r for r in rows[n] if float(r['thrust_ms'])>=0];t=[float(r['thrust_ms']) for r in rr]
    for k,style in [('link_002_joint_dq','-'),('link_003_joint_dq','--')]:
        axs[0].plot(t,[float(r[k]) for r in rr],ls=style,label=n+' '+('hip' if '002' in k else 'knee'))
    for k,style,label in [('ctrl_tau_body_hip','-','body compensation'),('ctrl_actual_tau_hip_left','--','hip published'),('ctrl_actual_tau_knee_left',':','knee published')]:
        axs[1].plot(t,[float(r[k]) for r in rr],ls=style,label=n+' '+label)
    axs[2].plot(t,[float(r['ctrl_cmd_x']) for r in rr],label=n+' wheel command')
for i,ax in enumerate(axs):
    ax.grid(alpha=.25);ax.legend(fontsize=7,ncol=3)
    ax.set_ylabel(['Actual dq (rad/s)','Published torque (Nm)','Wheel command (source cmd_x)'][i])
axs[-1].set_xlabel('ms from first THRUST record')
fig.suptitle('Saved physical joint motion and published commands (commands are not contact forces)')
fig.tight_layout();fig.savefig(O/'joint_control.png',dpi=135);plt.close(fig)

checks={};inputs={}
for folder,manifest in [('clearance_apex_ab_20261008','raw_sha256.json'),('thrust_height_032_ab_20261008','raw_sha256.json'),('thrust_height_032_ab_20261008','baseline_frozen_sha256.json')]:
    expected=json.loads((D/folder/manifest).read_text());bad=[]
    for rel,sha in expected.items():
        p=(D/folder/rel) if rel.startswith('physical/') else W/rel
        actual=hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
        if sha!=actual:bad.append(rel)
    checks[folder+'/'+manifest]={'count':len(expected),'mismatches':bad}
    assert not bad, bad
for n in rows:
    p=D/('clearance_apex_ab_20261008/physical' if n[0]=='B' else 'thrust_height_032_ab_20261008/physical_runs')/n
    for filename in ['engine_frames.csv','engine_frames.csv.inertials.csv','contact_detail.csv','ground_frames.csv','native_wrench.csv','velocity_log.csv','actual_robot.sdf','actual_robot.urdf']:
        f=p/filename;inputs[str(f.relative_to(W))]=hashlib.sha256(f.read_bytes()).hexdigest()
for filename in ['src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp','src/bbot_balance_controller/include/bbot_balance_controller/jump_phase_control.hpp']:
    inputs[filename]=hashlib.sha256((W/filename).read_bytes()).hexdigest()
(O/'provenance.json').write_text(json.dumps({'new_physical_runs':0,'controller_changed':False,'hash_checks':checks,'input_sha256':inputs},indent=2))
print(json.dumps({'release':out['fixed_record_release_check'],'T1_last_20ms':out['T1_last_20ms'],'hash_checks':checks},indent=2))
