#!/usr/bin/env python3
"""Plot frozen physical data; missing inputs remain invalid rather than zero."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def values(rows, field):
    return np.array([float(row[field]) for row in rows])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('record', type=Path)
    p = parser.parse_args().record
    out = p/'plots'
    out.mkdir(exist_ok=True)
    trace = [x for x in read(p/'control.csv.motion_trace.csv') if int(x['sim_time_ns']) >= 16065000000]
    events = read(p/'control.csv.phase_events.csv')
    begin = next(int(x['sim_event_ns']) for x in events if x['event']=='trial_start')
    end = next(int(x['sim_event_ns']) for x in events if x['event']=='stop_complete')
    motion = [x for x in trace if begin <= int(x['sim_time_ns']) <= end]
    # Select only required records while retaining the immutable full raw file.
    with (p/'engine_state.csv').open() as stream:
        physical = [x for x in csv.DictReader(stream) if x['phase']=='after_step'
                    and x['entity_type']=='joint' and begin<=int(x['sim_time_ns'])<=end]
    with (p/'engine_audit/derived_native_wrench.csv').open() as stream:
        native = [x for x in csv.DictReader(stream) if int(x['sim_time_ns'])>=16065000000
                  and x['joint_index'] in ('0','1','3','4')]
    time = lambda rows: (values(rows,'sim_time_ns')-begin)*1e-9
    limbs=[(0,'hl','hip left','link_002_joint'),(1,'kl','knee left','link_003_joint'),
           (3,'hr','hip right','link_005_joint'),(4,'kr','knee right','link_006_joint')]
    def decorate(axes):
        for ax in axes.flat:
            for event in events:
                if event['event'] in ('trial_start','cruise_start','normal_stop_trigger','stop_complete'):
                    ax.axvline((int(event['sim_event_ns'])-begin)*1e-9,color='gray',alpha=.5,lw=.6)
            ax.grid(alpha=.25)
            ax.legend(fontsize=8)
        for ax in axes[-1,:]:
            ax.set_xlabel('Time relative to trial 1 start (s)')
    fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True)
    for index,short,label,name in limbs:
        col=0 if short.startswith('h') else 1
        color='C0' if short.endswith('l') else 'C1'
        axes[0,col].plot(time(trace),values(trace,'effort_'+short),label=label,color=color)
        rows=[x for x in native if int(x['joint_index'])==index]
        for ax,field,validfield in [(axes[1,col],'before_physics_joint_force_cmd_sim_input','before_physics_joint_force_cmd_valid'),
                                    (axes[2,col],'transmitted_axis_torque','wrench_valid')]:
            y=values(rows,field)
            y[values(rows,validfield)!=1]=np.nan
            ax.plot(time(rows),y,label=label,color=color,lw=.8)
    for row,title in enumerate(['Published effort','Actual JointForceCmd before Physics',
                                'Transmitted aggregate load (not motor torque)']):
        for col,joint in enumerate(['Hips','Knees']):
            axes[row,col].set_title(joint+': '+title)
            axes[row,col].set_ylabel('N m')
    decorate(axes)
    fig.tight_layout()
    fig.savefig(out/'three_torque_types.png',dpi=150)
    plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True)
    metrics={}
    for index,short,label,name in limbs:
        col=0 if short.startswith('h') else 1
        color='C0' if short.endswith('l') else 'C1'
        rows=[x for x in physical if x['entity_name'].endswith('::'+name)]
        q=values(rows,'joint_position_0')
        v=values(rows,'joint_velocity_0')
        qr=values(motion,'qref_'+short)
        axes[0,col].plot(time(rows),q-q[0],label=label+' native',color=color)
        axes[0,col].plot(time(motion),qr-qr[0],'--',label=label+' reference',color=color)
        axes[1,col].plot(time(rows),v,label=label+' native',color=color)
        axes[1,col].plot(time(motion),values(motion,'vref_'+short),'--',label=label+' reference',color=color)
        axes[2,col].plot(time(motion),values(motion,'joint_feedback_'+short),label=label+' feedback',color=color)
        metrics[short]={'native_peak_speed_rad_s':float(np.max(np.abs(v))),
                       'native_excited_steps_at_protocol_threshold':int(np.sum(np.abs(v)>=(.005 if col==0 else .01))),
                       'native_position_span_rad':float(np.ptp(q)),
                       'feedback_peak_abs_Nm':float(np.max(np.abs(values(motion,'joint_feedback_'+short))))}
    for col in range(2):
        axes[0,col].set_title('Hips' if col==0 else 'Knees')
        for row,label in enumerate(['Position change (rad)','Velocity (rad/s)','Joint feedback (N m)']):
            axes[row,col].set_ylabel(label)
    decorate(axes)
    fig.tight_layout()
    fig.savefig(out/'native_motion_and_feedback.png',dpi=150)
    plt.close(fig)
    (out/'native_motion_metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    unique={x['sim_time_ns']:x for x in trace if 11058000000<=int(x['sim_time_ns'])<=end}
    span=list(unique.values())
    sim=values(span,'sim_time_ns'); wall=values(span,'wall_publish_ns')
    timing={'configured_real_time_factor':1.0,'physics_dt_ns':1000000,
            'control_sim_dt_ms_median':float(np.median(np.diff(sim))/1e6),
            'control_wall_dt_ms_median':float(np.median(np.diff(wall))/1e6),
            'observed_control_span_ratio':float((sim[-1]-sim[0])/(wall[-1]-wall[0])),
            'scope':'Observed timing only; multiplier and physics step were unchanged.'}
    (p/'execution_timing.json').write_text(json.dumps(timing,indent=2,allow_nan=False)+'\n')
    protected=json.loads((p/'protected_manifest.json').read_text())
    changed=[path for path,sha in protected.items() if not Path(path).exists()
             or hashlib.sha256(Path(path).read_bytes()).hexdigest()!=sha]
    (p/'protected_postflight.json').write_text(json.dumps({'status':'FAIL' if changed else 'PASS',
        'checked':len(protected),'changed':changed},indent=2)+'\n')
    print(json.dumps({'native_metrics':metrics,'timing':timing,'protected_changed':changed}))

if __name__=='__main__':
    main()
