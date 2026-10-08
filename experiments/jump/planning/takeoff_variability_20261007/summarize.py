from pathlib import Path
import csv,json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/xy/bbot_ws_new');ROOT=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';R=ROOT/'takeoff_variability_20261007';P=W/'experiments/jump/planning/takeoff_variability_20261007';summary=[]
fig,ax=plt.subplots(6,1,figsize=(12,15),sharex=True)
for name in ['B1','T1','B2','T2','B3','T3']:
 d=R/name;a=list(csv.DictReader((d/'window.csv').open()));e=json.loads((d/'events.json').read_text());at=lambda t:next(r for r in a if float(r['t_ms'])==t);zero=at(0);pre=[r for r in a if float(r['t_ms'])<=0];onsets={}
 for threshold in [0,-.2,-1,-2]:
  onset=next((r for i,r in enumerate(pre) if float(r['pitch_rate'])<threshold and all(float(x['pitch_rate'])<threshold for x in pre[i:])),None);onsets[str(threshold)]=None if onset is None else float(onset['t_ms'])
 gaps=[];start=None
 for r in a:
  t=float(r['t_ms'])
  if t>=0:break
  if r['contact_count']=='0' and start is None:start=t
  if r['contact_count']!='0' and start is not None:gaps.append([start,t]);start=None
 assert start is None
 inputs=list(csv.DictReader((d/'native_inputs.csv').open()));fields=['joint_name','joint_position','joint_velocity','before_physics_joint_force_cmd_valid','before_physics_joint_force_cmd_sim_input','transmitted_axis_torque','contact_wrench_count'];input0=[{k:r[k] for k in fields} for r in inputs if int(r['sim_time_ns'])==round(e['off_s']*1e9)]
 assert all(r['contact_wrench_count']=='0' for r in inputs)
 b=json.loads((d/'budgets.json').read_text());assert all(abs(x['hip']+x['knee']+x['wheel']+x['configuration']+x['total_H_change']-x['actual'])<1e-9 for x in b.values())
 summary.append({'run':name,**e,'thrust_duration_ms':1000*(e['arrest_s']-e['thrust_s']),'last_contact_to_arrest_ms':1000*(e['arrest_s']-e['last_contact_s']),'onsets_continuous_to_off_ms':onsets,'zero_contact_intervals_recontacted_ms':gaps,'at_minus50':at(-50),'at_off':zero,'input_at_off':input0,'budgets':b})
 t=[float(r['t_ms']) for r in a]
 for i,k in enumerate(['pitch_rate','total_H_pitch','link_002_joint_v','link_003_joint_v','ctrl_fast_pitch_rate','contact_count']):ax[i].plot(t,[float(r[k]) for r in a],label=name)
for a,label in zip(ax,['native pitch rate rad/s','native total H kg m2/s','left hip actual rad/s','left knee actual rad/s','logged fast_pitch_rate rad/s','native ground contacts']):a.set_ylabel(label);a.axvline(0,color='k',ls=':');a.grid(alpha=.2);a.legend(ncol=6,fontsize=8)
ax[-1].set_xlabel('ms relative to first permanently no-contact native frame');fig.tight_layout();fig.savefig(R/'comparison.png',dpi=130)
(R/'summary.json').write_text(json.dumps(summary,indent=2))
lines=['# 六次A/B离地初态差异：只读审查','', '本轮新增实跑0次，控制/参数/原始记录全部未改。分析仅限最后接触前50ms到持续无接触首帧后20ms，即以首个持续无接触帧为0的−51～+20ms，六次各72个连续1ms原生物理帧。', '', '## 回答','', '**B2/T3的约−2rad/s在THRUST末段、尚有轮地接触并反复短暂失接触时已形成，不是ATTITUDE_ARREST或本轮hip干预造成。** 在−40ms到0，膝持续伸展加速产生的后转项各次都约−2.7～−3.2rad/s；B2/T3的髋运动正向补偿却仅+1.103/+1.015，明显低于其余四次+2.433～+2.814，同时接触窗口的总H变化项为−0.735/−0.616（其余−0.002～−0.204），构型项也更负。这三项差异定量解释了两次约−3rad/s的末段下降。', '', '**尚不能确定试次最早分叉的唯一来源。** 在指定窗口左端，B2/T3的实际髋膝构型、速度、COM上升速度、轮速及THRUST轨迹进度已不同；不存在所有试次先重合、再出现唯一先行变量的记录片段。不能把窗口内的第一处观察值冒充全程“最先分叉”。本轮没有向更早阶段扩展分析。', '', '髋机制未被否定。这里揭示空中干预之前的初态混杂，以及接地末段腿运动与外部角动量变化共同形成的后转。', '', '## 六次准确事件','', '|试验|最后接触sim s|持续离地sim s|THRUST入口sim s|THRUST→FLIGHT/ARREST sim s|THRUST时长ms|切换距最后接触ms|连续为负至离地的起点ms|','|---|---:|---:|---:|---:|---:|---:|---:|']
for s in summary:lines.append(f"|{s['run']}|{s['last_contact_s']:.3f}|{s['off_s']:.3f}|{s['thrust_s']:.3f}|{s['arrest_s']:.3f}|{s['thrust_duration_ms']:.0f}|{s['last_contact_to_arrest_ms']:.0f}|{s['onsets_continuous_to_off_ms']['0']}|")
lines += ['', 'B2：−28ms开始一直为负至离地，−26ms低于−.20，−20ms低于−1，−9ms低于−2。T3对应−25/−23/−16/−4ms。这里是“连续至离地”的区间，未套用连续50ms条件：两次离地后很快回正，不能称该早期负段持续50ms。其他四次也有短暂后转，区别是最后接触阶段恢复的程度。', '', '## 离地首帧：真实刚体角动量和运动','', 'H单位kg·m²/s；body/大腿/小腿/轮的每项均含自转+轨道，统一世界坐标、瞬时系统COM，四组和为总H。模型没有名为hip或knee的刚体：模型实体为link_002/005（大腿）、link_003/006（小腿）；髋/膝关节运动项另列JSON的hip_H_pitch/knee_H_pitch，不能与刚体组相加冒充总H。', '', '|试验|pitch °|pitch_rate rad/s|总H|body H|双大腿H|双小腿H|双轮H|COM vz m/s|','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
for s in summary:
 r=s['at_off'];lines.append('|'+s['run']+'|'+'|'.join(f'{float(r[k]):.4f}' for k in ['pitch_deg','pitch_rate','total_H_pitch','body_H_pitch','thigh_H_pitch','shank_H_pitch','wheels_H_pitch','com_vz'])+'|')
lines += ['', '|试验|左髋q/v|左膝q/v|左髋参考q/v|左膝参考q/v|左/右轮实际v rad/s|发布cmd_x m/s|','|---|---|---|---|---|---|---|']
for s in summary:
 r=s['at_off'];pair=lambda k1,k2:f'{float(r[k1]):.4f} / {float(r[k2]):.4f}';lines.append('|'+s['run']+'|'+'|'.join([pair('link_002_joint_q','link_002_joint_v'),pair('link_003_joint_q','link_003_joint_v'),pair('ctrl_hip_pos_cmd_left','ctrl_hip_vel_cmd_left'),pair('ctrl_knee_pos_cmd_left','ctrl_knee_vel_cmd_left'),pair('link_004_joint_v','link_007_joint_v'),f"{float(r['ctrl_cmd_x']):.4f}"])+'|')
lines += ['', '左右四关节全部实测/参考q/v及四腿实际力输入/两轮输入有效位均保留于每次window.csv/native_inputs.csv；表中仅压缩展示左侧。控制数据按真实timestamp向前保持，不伪造成物理步同刻的新发布。参考q/v取真实CSV hip_pos_cmd_*/knee_pos_cmd_*/hip_vel_cmd_*/knee_vel_cmd_*，实际q/v取同一步Physics Joint getter。轮输出区分cmd_x（发布速度指令）与native_inputs中before_physics_joint_force_cmd_sim_input（四腿实际力输入；两轮该项invalid，不能当实际轮轴力矩）；关节合负载transmitted_axis_torque另列，三者不混用。', '', '## 末段定量收支：−40ms→0','', '沿用前轮完整三维locked inertia/关节运动分解；以下单位rad/s，相加严格得到实际Δrate。接触阶段total_H_change是真实总角动量变化的等效项，**不是“守恒残差”，也不是直接测得接触冲量**。', '', '|试验|hip运动|knee运动|wheel轴自转|构型/惯量|总H变化|实际Δrate|','|---|---:|---:|---:|---:|---:|---:|']
for s in summary:
 b=s['budgets']['-40_0'];lines.append('|'+s['run']+'|'+'|'.join(f'{b[k]:+.4f}' for k in ['hip','knee','wheel','configuration','total_H_change','actual'])+'|')
lines += ['', '这里膝项为负，与前轮空中制动时膝正向补偿不矛盾：接地末段膝仍在加速伸展，空中随后膝减速，两段运动变化方向不同。轮轴相对自转项很小，不能凭轮刚体轨道H变化归咎wheel controller。', '', '## 最先可见差异与反馈相位','', '|试验|−50ms左hip q/v|左knee q/v|COM vz|轮速|已记录thrust_motion_elapsed s|','|---|---|---|---:|---:|---:|']
for s in summary:
 r=s['at_minus50'];lines.append(f"|{s['run']}|{float(r['link_002_joint_q']):.4f} / {float(r['link_002_joint_v']):.3f}|{float(r['link_003_joint_q']):.4f} / {float(r['link_003_joint_v']):.3f}|{float(r['com_vz']):.3f}|{float(r['link_004_joint_v']):.3f}|{r['ctrl_thrust_motion_elapsed']}|")
lines += ['', 'B2/T3在该点膝已更伸展、伸展速度更大，COM上升更快，名义轨迹进度也领先约15ms；因此不能从这个窗口认定随后哪个量是第一次产生差异。B2的轮速/命令差异尤其已经很大；T3没有相同量级的轮差异，故不能把某一轮速阈值直接视为两次共同根因。', '', '窗口内可以确认的控制链：', '', '- B2实际髋输入在−37ms转约−6.27Nm，至−7ms仍为负；正向约+9.61Nm到−2ms才进入物理步。T3在−40ms约−1.18Nm、−30ms约−5.01Nm，负输入持续至−1ms，0ms才正向。B1/T2约−20ms、B3约−22ms已转正；T1直到+1ms转正，对应其中等离地后转−.969rad/s。', '- B2在控制timestamp=14.930s（−23ms），实际pitch_rate约已负，记录的pitch_rate_raw=+.570592、fast_pitch_rate=+.634737，快慢速差修正thrust_release_fast_rate_comp_applied=−2.91517Nm，发布髋−5.82588Nm。T3在14.290s（−21ms）类似：raw=+.572018、fast=+.537712、修正−1.70245Nm、发布髋−5.51691Nm。实际状态已后转但反馈仍保留此前前转信息。', '- 这些命令反转还有发布到实际输入的延迟；B2在−8ms发布正向髋，实际到−2ms，T3在−6ms发布正向髋，实际到0ms，均约6ms。不能把发布CSV列当同一步实际作用。', '', '源码精确链路：bbot_velocity_jump_controller.cpp:1735–1751：pitch=-roll，pitch_rate_raw=-IMU.angular_velocity.x，pitch_rate_为alpha=.15低通；torso_pitch_control.hpp:12–35：TorsoImuObserver为时间常数10ms低通、fresh容许80ms；CSV fast_pitch_rate实际写入torso_imu_.rate()（7407附近），并非未滤波gyro。3309–3345采用该观察者rate计算修正：blend*clamp(.5*K_BODY_D_THRUST_*(legacy_rate_error-fast_rate_error),−8,8)，加到原受限body髋力矩。3365–3378组合hip_request及原terminal floor，3412–3420下发。以上为源码与真实输入匹配的相位证据，不是关闭该项后的因果实验。', '', 'CSV torso_imu_fresh在THRUST为0不能据此判定观察者失效：源码该成员只在支撑腿PD分支赋值，实际THRUST直接问torso_imu_.fresh(now_sec)。本轮不修改过滤、期限或门限。', '', '## 最后接触部位与序列','', '六次窗口中的接触对始终只有ground_plane::link::collision ↔ link_004::link_004_collision_collision，以及ground ↔ link_007::link_007_collision_collision；有接触时均num_contacts=2，无机身/膝接触。最后一帧也是双轮，并非单侧腿或机身碰地。下表每个区间为短暂双轮无接触[start,end)，end即再次双轮接触；0之后持续无接触。', '', '|试验|最后接触前50ms内失接触→再接触区间（相对t0，ms）|','|---|---|']
for s in summary:lines.append('|'+s['run']+'|'+', '.join(f"[{a:.0f},{b:.0f})" for a,b in s['zero_contact_intervals_recontacted_ms'])+'|')
lines += ['', 'B2最后有−6～−1ms的5ms失接触后再接触，T3有−4～−1ms的3ms；其他也都有反复接触。因此接触中断次数/某一帧不能独立解释后转。需结合前述输入、运动及总H变化。THRUST→FLIGHT/ATTITUDE_ARREST在真实最后接触后13或15ms，所有hip-only修改都晚于入空负角速度形成。', '', '## 数据边界及最小诊断方案','', 'ground_frames只保存碰撞对和计数，native_wrench的contact_wrench_count在全部六次窗口均0，现有记录没有实际接触点、法向/切向接触力或冲量。两轮BeforePhysics力输入亦invalid：轮为速度接口，Physics GetForce记录0也不能替代速度伺服/约束的实际驱动力矩；本轮轮端控制输出只能确认发布cmd_x及实际轮速，实际电机/约束力矩仍缺失。JointTransmittedWrench是关节合负载，不能补作接触力。因此可以计算总H收支和实际输入相位，但不能把总H变化逐帧分配到左右轮接触冲量，更不能宣称已经证明唯一因果根源。', '', '如需补足接触分账，最小方案是在现有只读Physics/ground recorder处记录已有碰撞接触结果中的实际接触点、每步法/切向冲量（或力与dt）、两碰撞体名称、世界坐标和物理步相位；若后端不提供，保持invalid，不由关节wrench拼造。控制已有日志已含输入/参考/IMU stamp，无需新增控制框架。本轮仅提出方案，未实现、未仿真。', '', '“最早分叉”在本窗口左端已被截断；全程日志仍保留，可在后续明确扩大时间窗口后追查，不需要为此先新增日志。本轮不擅自分析更早阶段。', '', '## 复现与保留','', '原始本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/physical/`；本轮派生本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/takeoff_variability_20261007/`。每次window.csv含全量ctrl_*真实列、原生q/v、COM速度、所有H组和关节相对H，links.csv分离自转/轨道，engine_window/native_inputs保存对应原生窗口，events/budgets及summary.json包含表格全精度。comparison.png为实测状态计算曲线，无模型扭矩冒充物理。', '', '运行工作区根的[分析脚本](analyze.py)并传入B1/T1/B2/T2/B3/T3，再运行[物理输入提取](extract_inputs.py)、[汇总报告](summarize.py)。不含控制输出或仿真操作。原控制/模型/参数/构建冻结文件核对不变；本轮没有实跑、调参、提交或推送。分析完成，停止。']
(P/'REVIEW.md').write_text('\n'.join(lines)+'\n')
print('report complete')
