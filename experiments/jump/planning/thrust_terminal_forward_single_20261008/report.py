"""Write the fixed single-run review from saved audit outputs."""
from pathlib import Path
import json,hashlib
W=Path('/home/xy/bbot_ws_new');D=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials';A=D/'thrust_terminal_forward_single_20261008';P=W/'experiments/jump/planning/thrust_terminal_forward_single_20261008';O=A/'analysis'
x=json.loads((O/'comparison.json').read_text());c=json.loads((O/'control_effect.json').read_text());t=x['T1'];r=x['R1'];tm=t['metrics'];rm=r['metrics']
frozen=json.loads((A/'baseline_frozen_sha256.json').read_text());exp=json.loads((A/'experiment_frozen_sha256.json').read_text())
assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in frozen.items());assert all(hashlib.sha256((W/k).read_bytes()).hexdigest()==v for k,v in exp.items())
assert rm['bilateral_clearance_m']<.20 and not rm['emergency'] and not rm['nonwheel_ground_contact'] and not rm['secondary_flight']
raw={str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (A/'physical_runs').rglob('*') if p.is_file()};(A/'raw_sha256.json').write_text(json.dumps(raw,indent=2))
prov=json.loads((A/'provenance.json').read_text());prov.update(combined_accepted=0,decision='NOT_ADOPTED',reasons=['bilateral_clearance_below_20cm','large_landing_lean_and_retreat_remain','single_trial_initial_state_confounding'],default_implementation_restored=True,default_implementation_never_modified=True,physical_jump_runs=1,additional_runs=0,raw_count=len(raw),experiment_file_count=len(exp));(A/'provenance.json').write_text(json.dumps(prov,indent=2))
def row(label,a,b,fmt='.3f',unit=''):
 return f'|{label}|{a:{fmt}}{unit}|{b:{fmt}}{unit}|\n'
text='''# THRUST释放段局部轮速参考：唯一一次物理实验

日期：2026-10-08。结论：**不采用，实验结束**。新增1实跑/1腾空/1正常首触/0联合通过；无启动失败、无追加试验。正常ARREST→TUCK→EXTEND完成、最终BALANCE，但双轮同帧净空19.608cm未达20cm，落地仍明显后倾/后退。默认实现始终未修改，实验实例已终止；默认0.25/0/0.78/1.60保持，无提交、推送或后续扫描。

## 本轮三个问题

1. **轮速参考是否真正改变水平冲量？** 干预确实改变已发布命令并到达实际轮相对运动；末20ms水平冲量从1.140降至0.540N·s，观测方向符合假设。但试次在干预前已分叉，不能证明冲量减少主要由该干预造成，更不能把同反馈公式回放当实际未干预对照。
2. **是否实际减少离地总负角动量？** 本次实测离地H由−1.633变为−1.450kg·m²/s，负值减小0.183；这是观测事实。共同THRUST+145ms（双方都未干预）时已有0.157差距，其后至离地差距仅净增0.026；现有单次结果不足以建立独立因果效应。
3. **是否同时20cm与可接受落地？** 否。净空不足；首触−9.661°、−1.357rad/s，最大后退87.588cm。相比失败T1姿态角和后退改善，但首触角速度绝对值更大，不能称稳定落地修复。按授权停止，不换幅度、不补跑。

## 唯一代码改动及选择依据

原源码`src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp:3432–3436`；独立副本仅此片段改变，精确diff见[test_source.patch](test_source.patch)。

```
原 v_star = jump_takeoff_forward_speed_ = 0.45
新 v_local = v_star - (thrust_release_.active() ? 0.02 : 0.0)
原 m = clamp(v_star + k*(v_star-v_pred),0,0.85)
新 m = clamp(v_local + k*(v_local-v_pred),0,0.85)
k = thrust_forward_velocity_kp_ = 1.20
```

姿态项及其taper仍使用原全局目标，运动学补偿、腿部竖直推力/PD、限制、安全保护及飞行/落地公式均未改变。未引入0.73释放比例。运行参数与T1相比仅两个日志路径不同，启动前断言通过；实际URDF/SDF与冻结基线逐字节一致。独立源/程序/入口74项冻结，原默认155项冻结前后均一致；独立构建成功。

选幅依据为T1释放13.765s至真实离地13.781s的16ms接地窗口。其v_pred0.550–0.557、原m0.331–0.321，未触及0/0.85；轮目标0.019–0.029，未触及±1.5。唯一选择0.02m/s减量，在固定记录反馈下使目标增加2.2×0.02=0.044m/s，首次从上一条0.05172至0.063454只增0.011734，小于原5ms正向限步0.08。未进行多个幅度扫描。回放结果保存为`preflight_fixed_record.csv`，仅是代数可观测性检查。

阶段边界仍按原逻辑：THRUST在真实离地后直到控制器确认才退出；`air_wheel_baseline_=last_wheel_cmd_x_`（原cpp:3583），空中首帧限步0.25（4815–4819）。T1最后THRUST/首帧FLIGHT命令0.026447→0.276446；本次0.208041→0.458041，均保持原0.25限步。空中基准确实不同，干预的继承影响不能忽略，不能声称物理作用严格只限于接地释放窗口；未额外重置基准或修改空中控制。

## 实际命令到达

本次THRUST首记录11.153s，释放条件/目标改变首记录11.303s（+150ms、离地前23ms），真实离地11.326s，ARREST入口11.338s。T1分别+165ms释放、+181ms离地、离地后14ms进入ARREST。阈值同为0.78；不同触发时刻来自各自反馈，不能称修改了释放时机。

本次11.303s目标比同反馈原公式增加0.044，但首条发布被原slew遮蔽；11.308s首次出现可观测发布差0.01243，11.311s实际`0.07*dq`到达该已改变命令；11.313s目标/发布一致、同反馈公式差完整0.044，仍有13ms真实接触。11.323s再次受原slew影响，所有限制保留。

真实离地轮相对速度×半径0.087639m/s，对应该时已发布命令0.087639；T1为0.028568。轮是相对小腿旋转，此数不等于轮世界角速度或水平力。旧物理记录没有可信实际轮电机力矩，未用全零effort替代。相同反馈的原公式曲线是离线回放，不是物理反事实。

## 逐项T1对照

数据来自实际七刚体状态和接触力，pitch轴−世界X，前向+世界Y；负角冲量为后转方向。接触力矩绕每步前系统COM积分，全段与末20ms采用相同物理步口径。

|指标|原高跳T1|本次R1|
|---|---:|---:|
'''
for label,a,b,fmt,unit in [
 ('THRUST至真实离地时长',181.,173.,'.0f',' ms'),
 ('末20ms前向地面冲量',t['last20ms']['Fy_integral'],r['last20ms']['Fy_integral'],'.3f',' N·s'),
 ('末20ms总外部pitch角冲量',t['last20ms']['moment_preCOM_integral'],r['last20ms']['moment_preCOM_integral'],'.3f',' kg·m²/s'),
 ('末20ms切向pitch角冲量',t['last20ms']['tangent_moment_preCOM_integral'],r['last20ms']['tangent_moment_preCOM_integral'],'.3f',' kg·m²/s'),
 ('全THRUST水平冲量',t['THRUST']['Fy_integral'],r['THRUST']['Fy_integral'],'.3f',' N·s'),
 ('全THRUST外部pitch角冲量',t['THRUST']['moment_preCOM_integral'],r['THRUST']['moment_preCOM_integral'],'.3f',' kg·m²/s'),
 ('全THRUST实际ΔH',t['THRUST']['delta_H'],r['THRUST']['delta_H'],'.3f',' kg·m²/s'),
 ('离地总H',t['takeoff']['H_pitch'],r['takeoff']['H_pitch'],'.3f',' kg·m²/s'),
 ('THRUST净竖直冲量',t['THRUST']['net_vertical_impulse'],r['THRUST']['net_vertical_impulse'],'.3f',' N·s'),
 ('离地COM竖直速度',t['takeoff']['COM_vz'],r['takeoff']['COM_vz'],'.3f',' m/s'),
 ('离地pitch',t['takeoff']['pitch_deg'],r['takeoff']['pitch_deg'],'.3f','°'),
 ('离地pitch_rate',t['takeoff']['pitch_rate'],r['takeoff']['pitch_rate'],'.3f',' rad/s'),
 ('同帧双轮最大净空',100*tm['bilateral_clearance_m'],100*rm['bilateral_clearance_m'],'.3f',' cm'),
 ('COM离地后最大上升',100*tm['com_rise_from_off_m'],100*rm['com_rise_from_off_m'],'.3f',' cm'),
 ('飞行时长',tm['flight_ms'],rm['flight_ms'],'.0f',' ms'),
 ('真实首触pitch',tm['contact_pitch_deg'],rm['contact_pitch_deg'],'.3f','°'),
 ('真实首触pitch_rate（首触步后）',tm['contact_rate'],rm['contact_rate'],'.3f',' rad/s'),
 ('首触前1ms COM下降速度',t['touch_pre_vz'],r['touch_pre_vz'],'.3f',' m/s'),
 ('首触后50ms法向冲量',t['contact_normal_impulse_first50ms_Ns'],r['contact_normal_impulse_first50ms_Ns'],'.3f',' N·s'),
 ('首触后最大后倾',tm['max_backward_pitch_deg'],rm['max_backward_pitch_deg'],'.3f','°'),
 ('真实首触锚定最大后退',100*tm['max_backward_axle_m'],100*rm['max_backward_axle_m'],'.3f',' cm'),
 ('20s末轮轴位移',100*tm['final_axle_displacement_m'],100*rm['final_axle_displacement_m'],'.3f',' cm'),
 ('安静恢复起点（持续1s确认）',tm['full_stability_start_s'],rm['full_stability_start_s'],'.3f',' s')]:text+=row(label,a,b,fmt,unit)
text+='''|异常碰地/二次腾空/EMERGENCY|均无|均无|
|正常ARREST/TUCK/EXTEND、最终状态|是，BALANCE|是，BALANCE|

本次首触前1ms pitch_rate−0.566，T1为+1.044rad/s；表中首触步后值包含冲击响应，二者分开报告。不能把首触后的角速度改变当作空中姿态原因。本次首触后0–300ms髋轴-轮轴垂向压缩5.774cm、空间腿长压缩6.117cm；T1分别5.438/7.185cm。二者不是互相可替代的缓冲量，未声称8cm缓冲验收通过。触地冲量采用完整50ms积分，不用峰值代替。

本次原生腿实际输入及发布/合负载三类曲线均保留；THRUST力预算夹限样本两次均0，但这不能证明没有其他速度/制动限制或竖直余量。离地vz降低0.0967m/s（约4%）、净竖直冲量降低2.389N·s，且离地COM高度更低；净空下降不能简单归为轮速干预的必然代价。

## 因果边界：不能把全部改善归因于干预

|实际状态|T1|R1|
|---|---:|---:|
|THRUST入口H kg·m²/s|+0.1490|+0.1152|
|入口pitch_rate rad/s|+0.2247|+0.1273|
|入口COM vz m/s|−0.1600|−0.1174|
|入口hip/knee dq rad/s|−0.3428 / +0.8343|−0.2750 / +0.6135|
|双方未干预的THRUST+145ms H|−1.3432|−1.1867|
|+145ms pitch_rate rad/s|−0.5295|−0.0353|
|+145ms至离地ΔH|−0.2895|−0.2633|

离地H差0.1827中，0.1565在双方均未干预时已经存在。末20ms窗口对应各自THRUST+161～181与+153～173ms，并不是完全相同的初态/轨迹进度。释放也分别在+165/+150ms，差异发生在该局部干预之前；不得将提前15ms释放归因于尚未生效的修改。

因此本轮是**干预可到达的物理证据、冲量同向改善的观测信号，但不是去除初态混杂的因果通过**。不重新扩展THRUST异常调查，不补跑凑匹配，不继续扫描。高度已失败且落地仍不合格，决定不采用。

## 数据、曲线与复现

本地完整资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/`。

- `physical_runs/R1/`：完整原生双相位状态、接触点/wrench、命令、模型、运行参数、事件和启动日志；`raw_sha256.json`绑定原始文件。
- `source_baseline/`、`source_test/`、`experiment_frozen_sha256.json`、`baseline_frozen_sha256.json`、独立`test.launch.py`及`build_thrust_terminal_forward_single/`均保留。默认源/程序未被覆盖，不存在待恢复的候选默认输出。
- `analysis/comparison.json`、`control_effect.json`、THRUST逐步/接触点CSV、flight/post_contact CSV保留完整定量结果。
- `analysis/wheel_contact_intervention.png`：发布/实际轮速度、接触水平力/力矩、总H；黑色原公式回放明确标为离线。
- `analysis/contact_momentum_comparison.png`：真实全THRUST角冲量闭合、Fz和vz；本次角冲量残差−0.0158，对照−0.0195kg·m²/s。
- `analysis/flight_landing_comparison.png`：净空、COM、姿态、后退、触地法向力及积分。
- `analysis/three_torque_types_comparison.png`及`analysis/R1/torques.png`：发布命令、物理步实际输入、关节合负载分开，未将模型曲线冒充实际输入。

离线复现顺序：本目录performance.py R1 → details.py / thrust_budget.py / momentum.py R1（父目录analysis/momentum预先存在）→ compare.py → control_effect.py → report.py。所有脚本仅读已保存物理数据；使用新输出目录避免覆盖旧验收。最初momentum离线运行因输出父目录缺失退出，原错误日志保留，补建目录后完成；未启动额外物理试验。

本轮结束。原始数据、diff和独立程序保留；不提交、不推送，等待下一项授权。
'''
(P/'REVIEW.md').write_text(text)
(A/'verification.json').write_text(json.dumps({'default_155_unchanged':True,'experiment_74_unchanged':True,'runtime_control_parameters_match_T1':True,'actual_model_matches':True,'physical_runs':1,'airborne':1,'combined_pass':0,'default_original_implementation_active':True,'candidate_process_terminated':True,'no_second_run':True},indent=2))
print('Review complete; NOT_ADOPTED; defaults and frozen files unchanged.')
