#!/usr/bin/env python3
from pathlib import Path
import json,csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007');metrics={x['run']:x for x in json.loads((R/'metrics.json').read_text())};allrows=[]
for name in ['B1','T1','B2','T2','B3','T3']:
 s=json.loads((R/'physical'/name/'momentum/summary.json').read_text());m=metrics[name];z={'run':name,'group':m['group'],'off_s':s['off_s'],'H_drift_fraction':s['max_pitch_H_drift_fraction_H0'],'prediction_rms':s['rate_prediction_rms'],'prediction_max':s['rate_prediction_max_error'],'rate0':s['rate_actual0'],'min_rate80':s['rate_actual_min'],'min_rate50':m['min_rate_first50'],'rate80':s['rate_actual80'],'pitch0':m['pitch_at_off_deg'],'pitch_delta80_deg':m['pitch_delta_first80_deg'],'negative_onset_ms':m['negative_onset_rel_ms'],'arrest_ms':m['arrest_entry_rel_ms'],'tuck_ms':m['tuck_entry_rel_ms'],'contact_pitch_deg':m['contact_pitch_deg'],'clearance_m':m['peak_bilateral_clearance_m'],'secondary_flight':m['secondary_flight_intervals'],'nonwheel_contact':m['nonwheel_ground_contact'],'states':m['states']}
 for label,key in [('18_54','rate_budget_18_54'),('0_60','rate_budget_0_60'),('0_80','rate_budget_0_80'),('peak_min','rate_budget_fast_rearward')]:
  b=s[key];jc=b['joint_rate_changes'];hip=sum(jc[n] for n in ['link_002_joint','link_005_joint']);knee=sum(jc[n] for n in ['link_003_joint','link_006_joint']);wheel=b['wheel_joint_rate_change'];shape=b['shape_rate_change'];res=b['H_drift_rate_change'];actual=b['actual_rate_change'];assert abs(hip+knee+wheel+shape+res-actual)<1e-9
  z[label]={'hip':hip,'knee':knee,'wheel':wheel,'configuration':shape,'residual':res,'actual_delta_rate':actual,'start_ms':b['start_ms'],'end_ms':b['end_ms']}
 allrows.append(z)
groups={}
for g in ['baseline','test']:
 rows=[r for r in allrows if r['group']==g];o={}
 for k in ['rate0','min_rate80','min_rate50','pitch_delta80_deg','rate80','contact_pitch_deg','clearance_m','H_drift_fraction','prediction_rms']:
  vals=[r[k] for r in rows];o[k]={'mean':float(np.mean(vals)),'range':[min(vals),max(vals)],'values':vals}
 for lab in ['18_54','0_60','0_80']:
  o[lab]={k:{'mean':float(np.mean([r[lab][k] for r in rows])),'values':[r[lab][k] for r in rows]} for k in ['hip','knee','wheel','configuration','residual','actual_delta_rate']}
 groups[g]=o
(R/'momentum_comparison.json').write_text(json.dumps({'individual':allrows,'groups':groups},indent=2))
fig,ax=plt.subplots(5,1,figsize=(12,15),sharex=True)
for name in ['B1','T1','B2','T2','B3','T3']:
 d=R/'physical'/name;rows=list(csv.DictReader((d/'momentum/momentum_window.csv').open()));t=[float(r['t_ms']) for r in rows];ls='-' if name[0]=='B' else '--'
 for i,key in enumerate(['link_002_joint_v','link_003_joint_v','pitch_rate','pitch_deg','link_004_joint_v']):ax[i].plot(t,[float(r[key]) for r in rows],ls,label=name)
for a,lab in zip(ax,['left hip actual rad/s','left knee actual rad/s','native pitch_rate rad/s','native pitch deg','left wheel actual rad/s']):a.set_ylabel(lab);a.grid(alpha=.2);a.legend(ncol=6,fontsize=8);a.axvspan(18,54,alpha=.05,color='gray')
ax[-1].set_xlabel('ms since first permanently no-contact native frame');fig.suptitle('Hip-only intervention: all six native physical traces');fig.tight_layout();fig.savefig(R/'comparison.png',dpi=130)
print(json.dumps({'individual':allrows,'groups':groups},indent=2))
