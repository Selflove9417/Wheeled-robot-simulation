"""Four saved trials only: event alignment and momentum accounting."""
from pathlib import Path
import csv,json,re,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

W=Path('/home/xy/bbot_ws_new');DATA=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
O=DATA/'height_032_pitch_formation_20261008';names=['B1','B2','B3','T1']
read=lambda p:list(csv.DictReader(p.open()))
allrows={};vectors={};records=[]
for n in names:
    root=DATA/('clearance_apex_ab_20261008' if n.startswith('B') else 'thrust_height_032_ab_20261008')
    d=root/('physical' if n.startswith('B') else 'physical_runs')/n
    m=json.loads((root/'analysis'/n/'metrics.json').read_text())
    a=read(O/n/'momentum.csv');allrows[n]=a;vectors[n]=np.load(O/n/'vectors.npz')
    f=read(root/'analysis'/n/'flight.csv');by_ms={round(float(r['off_ms'])):r for r in f}
    ctl=read(d/'velocity_log.csv');fl=[r for r in ctl if r['state_name']=='FLIGHT']
    duration_text=re.findall(r'sub=EXTEND.*?T=([0-9.]+)\+([0-9.]+)',(d/'launch.log').read_text())
    assert duration_text and len(set(duration_text))==1
    duration=float(duration_text[0][1]);off=m['off_s'];td=m['first_touch_s']
    entry=(float(fl[0]['timestamp'])-off)*1000
    end=m['extend_ms']+duration*1000
    reference_done=next(float(r['timestamp']) for r in fl
        if float(r['timestamp'])>=off+end/1000-.005 and r['flight_subphase']=='2'
        and all(abs(float(r[k]))<1e-7 for k in ['hip_vel_cmd_left','knee_vel_cmd_left','hip_vel_cmd_right','knee_vel_cmd_right']))
    times={'liftoff':0,'ARREST_entry':entry,'ARREST_exit_TUCK_entry':m['tuck_ms'],
           'TUCK_exit_EXTEND_entry':m['extend_ms'],'EXTEND_reference_end':end,
           'EXTEND_reference_done_publication':(reference_done-off)*1000,
           'last_no_contact':m['flight_ms']-1,'first_contact':m['flight_ms']}
    events=[]
    for label,t in times.items():
        i=round(t);r=by_ms[i]
        e={'event':label,'off_ms':i,'touch_ms':i-m['flight_ms'],'pitch_deg':float(r['pitch_deg']),
           'pitch_rate':float(r['pitch_rate']),'com_vz':float(r['com_vz'])}
        if i<len(a):
            h=a[i]
            for k in ['total_H_pitch','body_H_pitch','legs_H_pitch','wheels_H_pitch']:e[k]=float(h[k])
        for j in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:
            for k in ['q','v']:e[j+'_'+k]=float(r[j+'_'+k])
        for part in ['hip','knee']:
            e[part+'_left_position_tracking_error']=float(r['ctrl_'+part+'_pos_cmd_left'])-float(r[('link_002_joint' if part=='hip' else 'link_003_joint')+'_q'])
            e[part+'_left_reference_v']=float(r['ctrl_'+part+'_vel_cmd_left'])
        events.append(e)
    contact={};normal_deviations=[]
    for r in csv.DictReader((d/'contact_detail.csv').open()):
        ns=int(r['sim_time_ns']);start=round(td*1e9)
        if ns>=start+50_000_000:break
        if ns<start:continue
        assert r['time_valid']=='1' and r['feature_valid']=='1'
        value=contact.setdefault(ns,0.)
        if int(r['index'])<0:continue
        assert r['extra_valid']=='1' and r['point_valid']=='1'
        nn=np.array([float(r[k]) for k in ['nx1','ny1','nz1']]);ff=np.array([float(r[k]) for k in ['fx1','fy1','fz1']])
        assert abs(np.linalg.norm(nn)-1)<1e-9
        normal_deviations.append(abs(abs(nn[2])-1))
        contact[ns]+=abs(float(ff@nn))
    assert all(t in contact for t in range(start,start+50_000_000,1_000_000))
    impulses={str(ms)+'ms':sum(contact[t]*.001 for t in range(start,start+ms*1_000_000,1_000_000)) for ms in [1,10,20,50]}
    record={'run':n,'metrics':m,'momentum_summary':json.loads((O/n/'summary.json').read_text()),
            'events':events,'extend_duration_s':duration,'reference_end_before_contact_ms':m['flight_ms']-end,
            'reference_done_publication_before_contact_ms':1000*(td-reference_done),
            'reference_end_before_last_FLIGHT_log_ms':1000*(float(fl[-1]['timestamp'])-off)-end,
            'first_50ms_normal_impulse_Ns':impulses,'maximum_normal_vertical_deviation':max(normal_deviations)}
    records.append(record)

# Common airborne time, no extrapolation through any baseline contact.
common=min(len(a) for a in allrows.values())-1
T=vectors['T1'];difference=[];curve_rows=[]
for n in names[:3]:
    B=vectors[n];count=min(len(allrows[n]),len(allrows['T1']));I=T['I'][:count];Ib=B['I'][:count]
    terms={'initial_H':np.array([-np.linalg.solve(i,T['H0']-B['H0'])[0] for i in I]),
           'configuration':np.array([-(np.linalg.solve(i,B['H0'])-np.linalg.solve(j,B['H0']))[0] for i,j in zip(I,Ib)])}
    for k in ['hip','knee','wheel','H_drift']:
        terms[k]=np.array([float(r['rate_from_'+k]) for r in allrows['T1'][:count]])-np.array([float(r['rate_from_'+k]) for r in allrows[n][:count]])
    actual=np.array([float(r['pitch_rate']) for r in allrows['T1'][:count]])-np.array([float(r['pitch_rate']) for r in allrows[n][:count]])
    assert max(abs(sum(terms.values())-actual))<1e-7
    budgets=[]
    for lo,hi in [(0,110),(110,250),(250,common),(0,common),(0,count-1)]:
        vals={k:math.degrees(float(np.trapz(v[lo:hi+1],dx=.001))) for k,v in terms.items()}
        true=(float(allrows['T1'][hi]['pitch_deg'])-float(allrows[n][hi]['pitch_deg']))-(float(allrows['T1'][lo]['pitch_deg'])-float(allrows[n][lo]['pitch_deg']))
        budgets.append({'start_ms':lo,'end_ms':hi,'angle_contributions_deg':vals,'actual_gap_change_deg':true,'angle_identity_residual_deg':true-sum(vals.values())})
    baseline_last=count-1;pre_touch=len(allrows['T1'])-1
    late=float(allrows['T1'][pre_touch]['pitch_deg'])-float(allrows['T1'][baseline_last]['pitch_deg'])
    difference.append({'baseline':n,'budgets':budgets,'additional_test_airtime_ms':pre_touch-baseline_last,
       'observed_test_angle_during_additional_airtime_deg':late,
       'precontact_pitch_gap_deg':float(allrows['T1'][-1]['pitch_deg'])-float(allrows[n][-1]['pitch_deg'])})
    for i in range(count):
        curve_rows.append({'baseline':n,'off_ms':i,'actual_rate_gap':actual[i],**{k:v[i] for k,v in terms.items()}})
with (O/'rate_difference_terms.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=curve_rows[0].keys());w.writeheader();w.writerows(curve_rows)

samples=[]
for t in [0,14,30,50,84,110,144,180,220,250,300,380,common]:
    b=np.array([float(allrows[n][t]['pitch_deg']) for n in names[:3]]);p=float(allrows['T1'][t]['pitch_deg'])
    samples.append({'off_ms':t,'T1_pitch_deg':p,'baseline_pitch_mean_deg':float(b.mean()),'gap_deg':p-float(b.mean()),'baseline_min':min(b),'baseline_max':max(b)})
summary={'trials':records,'common_airborne_end_ms':common,'aligned_pitch_samples':samples,'paired_difference_budgets':difference}
(O/'comparison.json').write_text(json.dumps(summary,indent=2))

fig,axs=plt.subplots(4,2,figsize=(14,13))
for n in names:
    a=allrows[n];t=np.array([float(r['t_ms']) for r in a]);m=next(r['metrics'] for r in records if r['run']==n)
    for col,x in enumerate([t,t-m['flight_ms']]):
        axs[0,col].plot(x,[float(r['pitch_deg']) for r in a],label=n)
        axs[1,col].plot(x,[float(r['pitch_rate']) for r in a],label=n)
        axs[2,col].plot(x,[float(r['total_H_pitch']) for r in a],label=n)
        axs[3,col].plot(x,[float(r['link_002_joint_v']) for r in a],label=n+' hip')
        axs[3,col].plot(x,[float(r['link_003_joint_v']) for r in a],ls='--',label=n+' knee')
for row in range(4):
    for col in range(2):
        ax=axs[row,col];ax.grid(alpha=.2);ax.legend(fontsize=7,ncol=2)
        ax.set_ylabel(['native pitch deg','native pitch rate rad/s','total pitch H kg m2/s','left joint speed rad/s'][row])
        ax.set_xlabel('ms from native sustained no-contact frame' if col==0 else 'ms to native first contact')
        if col==0:ax.axvspan(110,250,color='red',alpha=.06)
fig.suptitle('MEASURED saved physics: liftoff / first-contact alignment; no new simulation')
fig.tight_layout();fig.savefig(O/'phase_alignment.png',dpi=130)

fig,axs=plt.subplots(2,1,figsize=(12,9))
for n in names[:3]:
    aa=np.array([float(r['pitch_deg']) for r in allrows['T1'][:common+1]])-np.array([float(r['pitch_deg']) for r in allrows[n][:common+1]])
    axs[0].plot(range(common+1),aa,label='T1 minus '+n)
for k in ['initial_H','configuration','hip','knee','wheel','H_drift']:
    v=np.mean([[r[k] for r in curve_rows if r['baseline']==n and r['off_ms']<=common] for n in names[:3]],axis=0)
    integ=np.r_[0,np.cumsum(.5*(v[1:]+v[:-1])*.001)]*180/np.pi
    axs[1].plot(range(common+1),integ,label=k)
for ax in axs:ax.grid(alpha=.2);ax.legend(ncol=3);ax.axvspan(110,250,color='red',alpha=.06);ax.set_xlabel('ms from sustained no contact')
axs[0].set_ylabel('actual pitch difference deg');axs[1].set_ylabel('integrated rate-difference terms deg')
fig.suptitle('Exact kinematic accounting on measured paths; not a causal intervention')
fig.tight_layout();fig.savefig(O/'angle_budget.png',dpi=130)
print(json.dumps({'common_ms':common,'samples':samples,'paired_budgets':difference},indent=2))
