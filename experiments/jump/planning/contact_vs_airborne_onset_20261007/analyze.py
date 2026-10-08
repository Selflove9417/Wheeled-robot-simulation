#!/usr/bin/env python3
"""Read-only contact/deceleration alignment of seven existing trials."""
from pathlib import Path
import csv,json
W=Path('/home/xy/bbot_ws_new');O=Path(__file__).resolve().parent;P=O.parent/'landing_capture_pitch_chain_20261007';D=json.loads((O.parent/'negative_pitch_onset_20261007/summary.json').read_text());out=[]
for d in D:
 n=d['run'];fr=list(csv.DictReader((P/n/'contact_window.csv').open()));on=round(d['onset_sim_s']*1e9);td=round((d['onset_sim_s']-d['onset_rel_contact_s'])*1e9);last=max(int(r['sim_time_ns']) for r in fr if int(r['sim_time_ns'])<td and int(r['num_contacts'])>0);off=last+1000000
 assert all(r['frame_valid']=='1' and int(r['num_contacts'])==0 for r in fr if off<=int(r['sim_time_ns'])<td)
 jj={k:{} for k in ['link_002_joint','link_003_joint','link_004_joint','link_005_joint','link_006_joint','link_007_joint']}
 for r in csv.DictReader((P/n/'native_window.csv').open()):jj[r['joint_name']][int(r['sim_time_ns'])]=r
 ts=sorted(jj['link_002_joint']);dec={}
 for k in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:
  hits=[];threshold=1 if k in ['link_002_joint','link_005_joint'] else 2
  for i,t in enumerate(ts[:-10]):
   if off-60000000<=t<=off+60000000:
    v=[abs(float(jj[k][a]['joint_velocity'])) for a in ts[i:i+11]]
    if v[0]-v[-1]>=threshold and sum(v[a]>v[a+1] for a in range(10))>=7:hits.append(t)
  dec[k]={'first_in_window_ns':hits[0] if hits else None,'first_after_off_ns':next((t for t in hits if t>=off),None)}
 def snap(ns):
  r=jj['link_002_joint'][ns];return {'sim_s':ns*1e-9,'pitch_rate':-float(r['post_base_world_wx']),'joints':{k:{'q':float(v[ns]['joint_position']),'v':float(v[ns]['joint_velocity'])} for k,v in jj.items()},'num_contacts':next(int(r['num_contacts']) for r in fr if int(r['sim_time_ns'])==ns)}
 frames=[r for r in fr if on-30000000<=int(r['sim_time_ns'])<=off];wr=[jj['link_002_joint'][int(r['sim_time_ns'])] for r in frames]
 x={'run':n,'gain':d['gain'],'last_contact_ns':last,'first_permanently_no_contact_ns':off,'onset_ns':on,'arrest_entry_ns':round((d['onset_sim_s']-d['onset_rel_flight_entry_s'])*1e9),'onset_relative_off_ms':(on-off)/1e6,'deceleration':dec,'last_contact':snap(last),'off':snap(off),'onset':snap(on),'onset_plus30ms':snap(on+30000000),'ground_window_contact_wrench_counts':sorted(set(int(r['contact_wrench_count']) for r in wr)),'contacts_near_terminal':[{'sim_s':int(r['sim_time_ns'])*1e-9,'num_contacts':int(r['num_contacts'])} for r in fr if off-10000000<=int(r['sim_time_ns'])<=off+2000000]};out.append(x)
 print(n,'last',last/1e9,'off',off/1e9,'on-off',x['onset_relative_off_ms'],'rate off/on/+30',*[round(x[k]['pitch_rate'],3) for k in ['off','onset','onset_plus30ms']])
(O/'summary.json').write_text(json.dumps(out,indent=2))
