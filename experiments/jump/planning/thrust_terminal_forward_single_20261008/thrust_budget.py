"""Saved native THRUST steps: contact-moment and linear-impulse budgets only."""
from pathlib import Path
import csv,json,math,bisect,os,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

W=Path('/home/xy/bbot_ws_new');DATA=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials'
OUT=Path(os.environ.get('THRUST_BUDGET_OUTPUT',str(DATA/'thrust_terminal_forward_single_20261008'/'analysis'/'thrust_budget')));OUT.mkdir(exist_ok=False)
names=['R1'];summaries=[]
def read(p):return list(csv.DictReader(p.open()))
def dump(p,rows):
    with p.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

for name in names:
    root=DATA/'thrust_terminal_forward_single_20261008'
    d=root/('physical' if name[0]=='B' else 'physical_runs')/name
    out=OUT/name;out.mkdir();m=json.loads((root/'analysis'/name/'metrics.json').read_text())
    ctl=read(d/'velocity_log.csv');first=next(i for i,r in enumerate(ctl) if r['state_name']=='THRUST')
    start=round(float(ctl[first]['timestamp'])*1e9);prior=round(float(ctl[first-1]['timestamp'])*1e9)
    off=round(m['off_s']*1e9);th=[r for r in ctl if start<=round(float(r['timestamp'])*1e9)<off and r['state_name']=='THRUST']
    meta={r['entity_name'].split('::')[-1]:r for r in read(d/'engine_frames.csv.inertials.csv')}
    sdf={l.attrib['name']:l for l in ET.parse(d/'actual_robot.sdf').getroot().find('model').findall('link')};assert set(meta)==set(sdf) and len(meta)==7
    pars={}
    for n,r in meta.items():
        mass=float(r['mass']);c=np.array([float(r['c'+a]) for a in 'xyz'])
        Q=Rotation.from_quat([float(r[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
        I=np.array([[float(r[k]) for k in ks] for ks in [['ixx','ixy','ixz'],['ixy','iyy','iyz'],['ixz','iyz','izz']]])
        assert abs(mass-float(sdf[n].findtext('inertial/mass')))<1e-12
        assert np.max(abs(c-np.array(list(map(float,sdf[n].findtext('inertial/pose').split()))[:3])))<1e-12
        for k in ['ixx','ixy','ixz','iyy','iyz','izz']:assert abs(float(r[k])-float(sdf[n].findtext('inertial/inertia/'+k)))<1e-12
        pars[n]=(mass,c,Q@I@Q.T)
    M=sum(v[0] for v in pars.values());raw={}
    for r in csv.DictReader((d/'engine_frames.csv').open()):
        ns=int(r['sim_time_ns'])
        if ns>off:break
        if ns<prior:continue
        assert r['time_valid']=='1' and r['entity_present']=='1' and int(r['dt_ns'])==1000000
        assert int(r['physical_state_time_ns'])==ns-(1000000 if r['phase']=='before_step' else 0)
        raw.setdefault((ns,r['phase']),{})[r['entity_name'].split('::')[-1]]=r
    def state(rr):
        links={}
        for n,(mass,c,I) in pars.items():
            r=rr[n];assert r['position_valid']=='1' and r['velocity_valid']=='1'
            Q=Rotation.from_quat([float(r['quat_'+a]) for a in ['x','y','z','w']]).as_matrix()
            rc=Q@c;p=np.array([float(r['position_'+a]) for a in 'xyz'])+rc
            om=np.array([float(r['angular_v'+a]) for a in 'xyz']);v=np.array([float(r['linear_v'+a]) for a in 'xyz'])+np.cross(om,rc)
            links[n]=(mass,p,v,om,Q@I@Q.T)
        C=sum(l[0]*l[1] for l in links.values())/M;V=sum(l[0]*l[2] for l in links.values())/M
        H={n:l[4]@l[3]+np.cross(l[1]-C,l[0]*(l[2]-V)) for n,l in links.items()}
        body=rr['base_link'];p=-math.degrees(Rotation.from_quat([float(body['quat_'+a]) for a in ['x','y','z','w']]).as_euler('xyz')[0])
        row={'H_pitch':-sum(h[0] for h in H.values()),'body_H':-H['base_link'][0],
             'thighs_H':-H['link_002'][0]-H['link_005'][0],'shanks_H':-H['link_003'][0]-H['link_006'][0],
             'wheels_H':-H['link_004'][0]-H['link_007'][0], 'pitch_deg':p,'pitch_rate':-float(body['angular_vx'])}
        row.update({f'COM_{a}':float(C[i]) for i,a in enumerate('xyz')});row.update({f'COM_v{a}':float(V[i]) for i,a in enumerate('xyz')})
        for j in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint','link_004_joint','link_007_joint']:
            assert rr[j]['velocity_valid']=='1'
            row[j+'_q']=float(rr[j]['joint_position_0']);row[j+'_dq']=float(rr[j]['joint_velocity_0'])
        return C,V,row
    states={key:state(rr) for key,rr in raw.items()}
    contact={};points=[]
    for r in csv.DictReader((d/'contact_detail.csv').open()):
        ns=int(r['sim_time_ns'])
        if ns>off:break
        if ns<prior:continue
        assert r['time_valid']=='1' and r['feature_valid']=='1'
        contact.setdefault(ns,[])
        if int(r['index'])<0:assert r['contact_count']=='0';continue
        assert r['extra_valid']=='1' and r['point_valid']=='1'
        one='::bbot::' in r['collision1'];two='::bbot::' in r['collision2']
        assert one!=two and '::ground_plane::' in r['collision1']+r['collision2']
        f=(1 if one else -1)*np.array([float(r[k]) for k in ['fx1','fy1','fz1']]);p=np.array([float(r[k]) for k in ['px','py','pz']])
        nvec=np.array([float(r[k]) for k in ['nx1','ny1','nz1']]);assert abs(abs(nvec[2])-1)<1e-10
        contact[ns].append((f,p,r['collision1'] if one else r['collision2']))
    times=[float(r['timestamp']) for r in ctl];budgets=[]
    for ns in range(prior,off+1,1000000):
        Cb,Vb,b=states[ns,'before_step'];Ca,Va,a=states[ns,'after_step'];assert ns in contact
        F=np.zeros(3);tb=np.zeros(3);ta=np.zeros(3);normal_m=0.;tangent_m=0.;normal_m_pre=0.;tangent_m_pre=0.;cop_numerator=0.
        for f,p,collision in contact[ns]:
            F+=f;tb+=np.cross(p-Cb,f);ta+=np.cross(p-Ca,f)
            normal_m+=-(p[1]-Ca[1])*f[2];tangent_m+=(p[2]-Ca[2])*f[1]
            normal_m_pre+=-(p[1]-Cb[1])*f[2];tangent_m_pre+=(p[2]-Cb[2])*f[1]
            cop_numerator+=p[1]*f[2]
            points.append({'sim_time_ns':ns,'thrust_ms':(ns-start)/1e6,'off_ms':(ns-off)/1e6,'collision':collision,
                           'px':p[0],'py':p[1],'pz':p[2],'Fx':f[0],'Fy':f[1],'Fz':f[2],
                           'normal_pitch_moment_postCOM':-(p[1]-Ca[1])*f[2], 'tangent_pitch_moment_postCOM':(p[2]-Ca[2])*f[1]})
        cmd=ctl[max(0,bisect.bisect_right(times,ns/1e9)-1)]
        row={'sim_time_ns':ns,'thrust_ms':(ns-start)/1e6,'off_ms':(ns-off)/1e6,
             **a,'contact_points':len(contact[ns]),'Fx':F[0],'Fy':F[1],'Fz':F[2],
             'COP_y':cop_numerator/F[2] if F[2]>1e-8 else float('nan'),
             'moment_preCOM':-tb[0],'moment_postCOM':-ta[0], 'normal_moment_postCOM':normal_m,
             'tangent_moment_postCOM':tangent_m,'normal_moment_preCOM':normal_m_pre,'tangent_moment_preCOM':tangent_m_pre,
             'delta_H_step':a['H_pitch']-b['H_pitch'],'angular_residual_postCOM':a['H_pitch']-b['H_pitch']+ta[0]*.001,
             'vertical_delta_momentum_step':M*(Va[2]-Vb[2]),'linear_residual':M*(Va[2]-Vb[2])-(F[2]-M*9.81)*.001}
        for k in ['state_name','pitch','pitch_rate','fast_pitch_rate','pitch_rate_raw','tau_body_hip','thrust_reaction_ff',
                  'thrust_hip_requested_left','thrust_hip_requested_right','thrust_hip_floor_applied','actual_tau_hip_left','actual_tau_hip_right',
                  'actual_tau_knee_left','actual_tau_knee_right','hip_pos_cmd_left','knee_pos_cmd_left','hip_vel_cmd_left','knee_vel_cmd_left',
                  'thrust_attitude_blocked','thrust_release_active','thrust_release_blend','thrust_extension_scale','F_z','F_z_request','F_z_limit',
                  'thrust_force_before_budget','thrust_knee_pd_left','thrust_release_fast_rate_comp_requested','thrust_release_fast_rate_comp_applied',
                  'thrust_wheel_target','cmd_x','jump_forward_axis_x','jump_forward_axis_y']:
            row['ctrl_'+k]=cmd[k]
        budgets.append(row)
    def budget(lo,hi):
        subset=[r for r in budgets if lo<int(r['sim_time_ns'])<=hi];a=states[lo,'after_step'][2];b=states[hi,'after_step'][2]
        post=sum(r['moment_postCOM']*.001 for r in subset);pre=sum(r['moment_preCOM']*.001 for r in subset)
        gross=sum(r['Fz']*.001 for r in subset);net=gross-M*9.81*(hi-lo)*1e-9
        return {'start_s':lo/1e9,'end_s':hi/1e9,'steps':len(subset),'initial':a,'final':b,
                'delta_H':b['H_pitch']-a['H_pitch'],'moment_impulse_postCOM':post,'moment_impulse_preCOM':pre,
                'normal_moment_impulse_postCOM':sum(r['normal_moment_postCOM']*.001 for r in subset),
                'tangent_moment_impulse_postCOM':sum(r['tangent_moment_postCOM']*.001 for r in subset),
                'angular_residual_postCOM':b['H_pitch']-a['H_pitch']-post,'angular_residual_preCOM':b['H_pitch']-a['H_pitch']-pre,
                'vertical_gross_impulse':gross,'vertical_net_impulse':net,'COM_delta_vz':b['COM_vz']-a['COM_vz'],
                'linear_residual':M*(b['COM_vz']-a['COM_vz'])-net,
                'vertical_force_weighted_pitch_lever':sum(r['normal_moment_postCOM']*.001 for r in subset)/gross,
                'horizontal_contact_impulse_Fy':sum(r['Fy']*.001 for r in subset)}
    segs={}
    for lo,hi in [(0,30),(30,60),(60,90),(90,120),(120,150),(150,(off-start)//1000000)]:
        if hi>lo:segs[f'{lo}_{hi}']=budget(start+lo*1000000,start+hi*1000000)
    effort=[]
    for r in csv.DictReader((d/'native_wrench.csv').open()):
        ns=int(r['sim_time_ns'])
        if ns>=off:break
        if ns<start or r['joint_name'] not in ['link_002_joint','link_003_joint','link_005_joint','link_006_joint']:continue
        assert r['before_physics_joint_force_cmd_valid']=='1'
        effort.append({'sim_time_ns':ns,'off_ms':(ns-off)/1e6,'joint':r['joint_name'],'actual_input_Nm':float(r['before_physics_joint_force_cmd_sim_input'])})
    whole=budget(start,off);controls={}
    for k in ['tau_body_hip','thrust_reaction_ff','thrust_release_fast_rate_comp_requested','thrust_release_fast_rate_comp_applied',
              'actual_tau_hip_left','actual_tau_knee_left','thrust_hip_requested_left','fast_pitch_rate','pitch_rate_raw','thrust_knee_pd_left']:
        vv=[float(r[k]) for r in th];controls[k]={'min':min(vv),'max':max(vv),'at_20_fraction':sum(abs(v)>=19.999 for v in vv)/len(vv)}
    summary={'run':name,'THRUST_entry_first_log_s':start/1e9,'previous_SQUAT_log_s':prior/1e9,'off_s':off/1e9,'mass':M,
             'whole':whole,'entry_bracket_alternative':budget(prior,off),'segments':segs,'control_ranges':controls,
             'native_actual_effort_peak_by_joint':{j:max(abs(r['actual_input_Nm']) for r in effort if r['joint']==j) for j in {r['joint'] for r in effort}},
             'all_preoff_ground_contacts_are_wheels':all('link_004_collision' in r['collision'] or 'link_007_collision' in r['collision'] for r in points),
             'contact_gaps':[[r['thrust_ms'],r['off_ms']] for r in budgets if r['contact_points']==0],
             'release_relative_off_ms':next((1000*(float(r['timestamp'])-off/1e9) for r in th if r['thrust_release_active']=='1'),None),
             'force_budget_limited_samples':sum(float(r['thrust_force_before_budget'])>float(r['F_z_limit'])+1e-4 for r in th)}
    dump(out/'steps.csv',budgets);dump(out/'contact_points.csv',points);dump(out/'actual_efforts.csv',effort)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));summaries.append(summary)
    (OUT/'summary.json').write_text(json.dumps(summaries,indent=2))
    print(name,json.dumps({k:whole[k] for k in ['delta_H','moment_impulse_postCOM','moment_impulse_preCOM','normal_moment_impulse_postCOM','tangent_moment_impulse_postCOM','angular_residual_postCOM','vertical_gross_impulse','vertical_net_impulse','vertical_force_weighted_pitch_lever']}),'initial H',whole['initial']['H_pitch'],'off H',whole['final']['H_pitch'],flush=True)

fig,axs=plt.subplots(5,2,figsize=(15,16))
for n in names:
    a=read(OUT/n/'steps.csv');sm=next(r for r in summaries if r['run']==n);start=sm['THRUST_entry_first_log_s'];anchor=next(i for i,r in enumerate(a) if float(r['thrust_ms'])==0)
    for col,key in enumerate(['thrust_ms','off_ms']):
        t=[float(r[key]) for r in a];axs[0,col].plot(t,[float(r['H_pitch']) for r in a],label=n)
        axs[1,col].plot(t,[float(r['moment_postCOM']) for r in a],label=n)
        cum=np.cumsum([float(r['moment_postCOM'])*.001 if float(r['thrust_ms'])>0 else 0 for r in a])
        axs[2,col].plot(t,cum,label=n+' contact integral')
        axs[2,col].plot(t,[float(r['H_pitch'])-float(a[anchor]['H_pitch']) for r in a],ls='--',alpha=.6,label=n+' actual dH')
        axs[3,col].plot(t,[float(r['Fz']) for r in a],label=n)
        axs[4,col].plot(t,[float(r['ctrl_tau_body_hip']) for r in a],label=n+' torso command')
        axs[4,col].plot(t,[float(r['ctrl_actual_tau_hip_left']) for r in a],ls='--',label=n+' published hip total')
for row in range(5):
    for col in range(2):
        ax=axs[row,col];ax.grid(alpha=.2);ax.legend(fontsize=7,ncol=2)
        ax.set_ylabel(['total pitch H kg m2/s','external pitch moment Nm','angular impulse / dH kg m2/s','actual ground Fz N','per-hip published command Nm'][row])
        ax.set_xlabel('ms from first THRUST log' if col==0 else 'ms from real liftoff')
fig.suptitle('Saved native physics: THRUST contact budget (no new simulation)');fig.tight_layout();fig.savefig(OUT/'thrust_budget.png',dpi=130)
