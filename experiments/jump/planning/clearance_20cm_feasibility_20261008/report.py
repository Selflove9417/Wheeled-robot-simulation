from pathlib import Path
import json,math
W=Path('/home/xy/bbot_ws_new');O=W/'src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008';P=W/'experiments/jump/planning/clearance_20cm_feasibility_20261008';data=json.loads((O/'summary.json').read_text());tuck=json.loads((O/'tuck_bound.json').read_text());lines=[]
def put(s=''):lines.append(s)
put('''# 20cm同帧双轮净空：工程可行性评估

2026-10-08。唯一问题：在现有Gazebo模型/限制下，哪条路线最可能使每次跳跃都达到20cm同帧双轮轮底净空，并稳定落地？本轮交付源码与现有实测分析、条件式离线模型及一个优先设计点；不是物理验收。新增实跑/腾空/达标均0，不修改控制器、控制参数、限制或launch，不构建、不提交、不推送。默认及landing_capture_gain=1.60保持。原始异常诊断不扩大。

## 决定与适用边界

**优先只提高THRUST的COM速度目标；不把小幅TUCK继续扩展成主线。建议下一次独立实验唯一改动 `jump_height: 0.25 → 0.32`，保留 `takeoff_velocity=0.0`，其余全部冻结。** 当前ROS入口已经暴露该参数，因此无需修改控制器C++逻辑，也无需增加控制框架。这是一个基于所需冲量和已有余量的单一设计点，不是扫描结果，更不是已批准/已实施的修改。

证据：20cm需要约0.15–0.22m/s额外离地速度、2.6–3.8N·s额外净竖直冲量；已有THRUST执行输入与力预算均有余量。反之，保留原反向保护/步幅且考虑现有跟踪误差，单独收腿没有达到20cm的可靠执行方案。组合有降低额外触地能量的理论价值，但要先处理轨迹接续和实际跟踪，修改面及未知数更多。

**“20cm并稳定落地”尚未被证明可交付。** 当前基线虽然最终均回BALANCE，首触仍后倾且落地后退约43–48cm；这不能算稳定落地性能已满足。提高推地后若姿态/恢复恶化，不能因高度通过而采用。当前也不能从3次同参数样本辨识目标速度增量到实际离地速度增量的闭环增益。

## 1. 指标与实际基线

`max_t min(clearance_left(t), clearance_right(t))`。每个物理帧独立计算左右实际轮collision cylinder底部，使用原生link位姿、碰撞origin/axis、半径/半宽及地面高度。禁止拼接两个单轮各自峰值；禁止用平均达标替代每次达标。算法和原数据沿用[上轮净空验收](../clearance_apex_ab_20261008/REVIEW.md)，未改定义。

主证据仅取同一轮原配置B1/B2/B3；不混入被否决的延后EXTEND试次。三次最小15.0335cm、最大16.5636cm、均值15.5521cm。平均缺4.4479cm；按已见最差试次至少缺4.9665cm，不能只设计4.45cm平均增量。

特别核对：这批“默认控制”实验的真实launch_command和runtime是 **jump_height=0.25、takeoff_velocity=0.0**，目标COM速度2.21472m/s。仓库launch声明默认jump_height=0.20（launch:282）与本批启动显式覆盖不同；本设计比较0.25→0.32，不能把0.20误写成本批起点。原始默认入口不改。

实际7刚体总质量17.51kg；控制器body_mass=9.5、TOTAL_MASS=17.5kg，原生base含固定IMU质量，保持既有模型差异。物理冲量计算用17.51，不用手猜质量。

## 2. 不改空中构型时的弹道需求

先用较保守的“最大净空处COM至轮底偏移不变”估算：

`Δh_COM = 0.20 - c_max`

`v_req = sqrt(v_off² + 2*g*Δh_COM)`

`ΔJz_net = M*(v_req-v_off)`，`ΔEz = M*g*Δh_COM`。

这里v_off来自持续无接触首帧的原生7刚体质量加权速度，不用控制器锁存/滤波值或机身速度代替。

|试次|实际净空cm|实际离地vz m/s|需增加COM弹道高度cm|所需离地vz m/s|额外净冲量N·s|额外竖直能量J|
|---|---:|---:|---:|---:|---:|---:|''')
for r in data:
 x=r['requirements'][0];put(f"|{r['run']}|{100*r['clearance_m']:.3f}|{r['off_v_native']:.4f}|{100*x['clearance_gap_m']:.3f}|{x['fixed_peak_offset_energy_required_v']:.4f}|{x['fixed_geometry_additional_net_impulse_Ns']:.3f}|{x['fixed_geometry_additional_energy_J']:.3f}|")
put('''
另对整个已有相对构型/姿态时间轨迹做更明确的条件计算：把COM额外速度增量设δv、其余同帧几何完全冻结，则每帧双轮较小净空增加δv*t。解析地取 `δv_req=min_t[(0.20-c_min(t))/t]`，不是参数扫描。三个结果所需离地vz2.3375–2.3594m/s、额外净冲量2.410–3.517N·s、COM顶点增量3.18–4.63cm。此时新净空峰移到约257–259ms，故与固定峰处偏移估算有小差异。两种算法都表明所需增量是几厘米COM弹道和约0.2m/s速度，远不是需要翻倍推力。

实际离地后COM上升约23.43–24.40cm，明显大于轮底净空15.03–16.56cm；不能再把jump_height参数、COM高度和轮底净空混为一谈。原生COM相对理想无阻力弹道最大残差约4mm，亦应计入模型不确定性。

推地窗口实测净竖直冲量约39.57–40.54N·s；20cm固定偏移需求相当于再增加约6.4–9.5%。地面接触力积分减MgΔt，与原生COM动量增量差0.123–0.139N·s（约0.3%），说明这里有可用的真实推地冲量证据。保持同样约0.175–0.177s窗口，额外平均地面竖直力约15–22N（总量，不是单腿）。这一需求不等于控制器请求F_z会一比一转成地面力。

## 3. THRUST实际控制链与余量

精确源码 `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`：

- 471–497：ROS参数键 `jump_height`、`takeoff_velocity`；C++字段 `jump_height_target_`、`takeoff_velocity_override_`。正takeoff_velocity覆盖高度换算，否则 `target_takeoff_velocity_=sqrt(2*9.81*max(0.01,jump_height_target_))`。
- 2814–2815：名义高度五次从当前高度/零速到 `H_TAKEOFF_=0.475`、终速0.8，时长 `thrust_duration_=0.24`。提高jump_height不会直接改这条名义几何轨迹；硬编码 `T_THRUST_=0.10` 不是当前活动时长参数。
- 2972–3098：Bezier额外力脉冲叠加质量加权COM速度反馈，`feedback_force=(TOTAL_MASS_/2)*thrust_velocity_kp_*(target-vz)`，ramp60ms，受姿态削减、terminal brake和release影响。
- 3168–3200及 `thrust_velocity_reference.hpp:39–62`：质量加权COM/轮轴Jacobian反解膝速度，先clamp至[-15,0]，再经过行程和制动保护；髋速度参考及姿态控制独立，不直接提高全部关节速度上限。
- 3257–3281：最终合力矩预算、signed_force_limit、每腿1600N/s请求斜率与travel/release限制。150Nm仿真THRUST限值是当前已有配置，未提高；预算margin0.95，髋留8Nm姿态余量。其余阶段仍75/60Nm（effort_allocation.hpp:13–15）。
- 3390–3422、3520–3549：真离地确认即可退出THRUST，即使目标速度尚未达到；不能靠改目标保证一定积累到该速度。速度锁存95%目标也不能替代真实离地。现有timeout、姿态保护均保留。

本次实际执行输入来自 `native_wrench.csv` 的 `before_physics_joint_force_cmd_sim_input`，与已发布command、步后 `transmitted_axis_torque` 合负载分开；后者不作为执行器输出。

|试次|髋实际输入峰值Nm（左）|膝实际输入峰值Nm（左）|实际膝最大速度rad/s|原膝参考最大rad/s|单腿力请求峰N /预算最小N|真实推力预算截断比例|
|---|---:|---:|---:|---:|---|---:|''')
for r in data:
 j=r['native_joint_window'];c=r['thrust_control'];put(f"|{r['run']}|{j['link_002_joint']['actual_input_abs_max_Nm']:.3f}|{j['link_003_joint']['actual_input_abs_max_Nm']:.3f}|{j['link_003_joint']['dq_abs_max']:.3f}|{c['max_published_knee_reference_speed']:.3f}|{c['force_request_max_per_leg']:.1f} / {c['force_limit_min_per_leg']:.1f}|{100*c['fraction_force_budget_limited']:.1f}%|")
put('''
左右数值近似但全量分列summary.json。当前行程scale始终1，实际膝输入没有达到142.5Nm预算或150Nm硬限。URDF关节速度上限30rad/s，THRUST膝参考上限15rad/s；实际速度超过参考不是同一限制被放宽。现有450/500rad/s²检查用于飞行轨迹，不能冒充THRUST已有的独立硬加速度限制。

`F_z`在本CSV是两腿总命令，`F_z_request/F_z_limit`是每腿值（cpp3517、7286–7288）。已按此口径核对：总命令峰约443.7–459.7N，不误写为单腿。余量积分约110–114N·s只是控制预算余量，不是可追加真实地面冲量，更不是已验证制动能力。

**判断：现有Gazebo执行器和限值没有显示出“增加约0.2–0.3m/s离地速度必然不可行”的硬瓶颈，支持优先做该单因素的后续物理验证。** 但没有新工况逆动力学/闭环响应，不能从未饱和推导一定可达。模型没有真实电机功率/热/电流曲线，结论仅限当前Gazebo，不能推广硬件。

## 4. 单独收腿：可用幅度与其限制

当前TUCK四终点被 `reaction_safe_configuration_step` 退回入口，实际TUCK期间轴距增加1.79–2.63cm；原60ms段主要制动，没有已验证正向有效收腿。此前一次0.04rad膝步幅、协调髋约−0.0202rad的40ms理想段，单独新增约6–7mm轴距缩短、4–5mm轮轴相对COM高度，不能直接填满近5cm缺口。

但也不能忽略恢复原跟踪误差的潜在收益：原EXTEND入口膝仍比旧TUCK终点差0.15–0.18rad。若先真的回到旧q_b，再追加小收腿，整体静态几何改善可大于4–5mm；此前数值只描述追加小段，不代表整个“跟踪收敛+收腿”的总几何收益。

本轮进一步给出保留helper步幅的乐观算例，**不是新轨迹全局上限/实际可行幅度**：假定原60ms制动结束已经到q_b/零速；每次knee+0.04，髋按七刚体一阶相对角动量抵消协调；采用现有零端速五次，每段满足速度/加速度极值的最短5ms网格时长25ms（膝3rad/s、369.5rad/s²，未超过13/500，髋亦通过）。每一步均需真实helper允许且实际跟上，不以反复重规划绕过保护。

|试次|最高点前最多完整理想步数|冻结旧最高点姿态下条件净空cm|已收敛后达到20cm所需总膝增量rad|至少完整步数/最快时间ms|原制动后最高点前时间ms|
|---|---:|---:|---:|---|---:|''')
for r in tuck:
 z=next(x for x in r['scenarios'] if x['scenario']=='no_settle_before_apex');v=z['conditional_geometry_at_original_frames'][0];put(f"|{r['run']}|{z['step_count']}|{100*v['conditional_clearance_m']:.2f}|{r['minimum_shared_knee_delta_to_20_at_old_apex_rad']:.4f}|{r['minimum_guarded_step_count_for_20']} / {r['optimistic_guarded_step_time_for_20_ms']}|{r['time_available_before_apex_ms']:.0f}|")
put('''
B1/B3达到该静态20cm构型至少7步、175ms；制动结束到最高点只有109ms，到保留90ms展腿及55+20ms预算的最后正常展腿时间也仅133ms。B2需5步125ms，完成已晚于最高点，冻结最高点的20.01cm数值不能视为可达到的动态净空。若先扣65ms用于原跟踪收敛，最高点前只剩1步，条件静态净空约17.38–18.14cm；**65ms只是预算，实测能否收敛未知。**

这些有限段算例显示：复用现有保护和零端速五次的最小分段路径，缺乏在三个试次都稳定实现20cm的时间余量。不能排除重新设计连续多段运动后有更优结果；那需要更广的衔接/跟踪/角动量与部署设计，不能把本算例冒充所有腿轨迹的数学不可达证明。当前可由实测确认的有效收腿增益仍没有正值资格。

膝目标几何/IK本身可达且端点不越关节限位；限制主要来自残余速度、0.04步幅、实际跟踪和部署时间。目标与旧参考/实际q/v不一致时必须连续拼接，并同时核对实测安全；直接以理想参考替代实测速度会绕过保护，禁止。加长TUCK、改变EXTEND时机、规划/执行中间目标和C1衔接必须联动，因此收腿主线修改面明显更大。

## 5. 三条路线比较

|路线|满足20cm的依据|实际修改范围|主要落地风险|优先级|
|---|---|---|---|---|
|提高THRUST速度目标|缺口只需约0.15–0.22m/s；真实输入/预算尚有余量|一个已暴露参数；原空中/落地逻辑保持|触地竖直能量增加；release/残余腿速和空中初态间接变化|首选验证|
|只优化收腿|理论几何可改善，但现有分段与实际跟踪没有三次均过20cm的证据|制动/收腿分段、时间重分配、C1、EXTEND预算及实际跟踪|髋膝角动量交换、部署余量减少、首触目标锁存后移|不作本轮主线|
|较小THRUST增量+有效收腿|理想收敛加小收腿可把几何净空提高到约17.4–18.1cm，再补弹道缺口|同时处理上述两类机制，单因素归因更难|额外冲击可低于纯弹道方案，但跟踪和姿态耦合未解|仅在首选验证被数据否定后另行研究|

组合的理论价值真实存在，但不能现在同时改THRUST和空中轨迹来制造成功，亦不自动进入第二方案。

## 6. 唯一优先设计点及精确位置

**仅独立实验launch覆写 `jump_height:=0.32`，本批对照仍是 `jump_height:=0.25`；`takeoff_velocity:=0.0`冻结。** 不修改仓库默认值。精确接入点：

- `src/bbot_bringup/launch/bbot_gazebo.launch.py:282`：已有 `DeclareLaunchArgument("jump_height", default_value="0.20")`；1137行传给控制器。参数已可覆写，无需改launch源码。
- `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp:471–497`：读取并换算目标COM速度；无需改C++。
- `thrust_duration=0.24`、`thrust_peak_ratio=2.0`、`thrust_shape_early/late=0.75`、`thrust_velocity_kp=8.0`、`thrust_release_velocity_ratio=0.78`、`thrust_knee_velocity_limit=15.0`、`thrust_torque_margin=0.95`、`sim_relax_thrust_limits=true`及所有现有安全/力矩限制保持。
- `L_RETRACT_=0.66`、`L_TOUCH_=0.69`、TUCK/EXTEND时序、ARREST、轮控制、landing_capture_gain=1.60均保持。

为什么选0.32：当前目标2.21472m/s，新目标2.50567m/s，增加0.29095m/s。只刚好补平均4.45cm不能覆盖现有最差试次；这个设计点按同一绝对目标速度损失、同几何轨迹的条件计算留出约2cm净空设计余量。没有宣称3个样本可保证统计可靠性。

|试次|若实际离地增量等于目标增量，条件净空cm|同触地高度额外COM竖直能量J|条件触地竖直速度增幅|旧release前固定构型重算的新raw膝参考峰rad/s|
|---|---:|---:|---:|---:|''')
for r in data:
 q=r['single_design_point_conditional'];put(f"|{r['run']}|{100*q['predicted_clearance_if_off_v_tracks_target_increment']:.2f}|{q['additional_energy_J']:.2f}|{100*q['impact_speed_increase_ratio']:.1f}%|{q['fixed_old_pre_release_window_candidate_raw_knee_reference_abs_max']:.3f}|")
put('''
这一单点需额外净冲量约5.095N·s（比已有净冲量多约13%）；固定旧状态下速度反馈增加约20.37N/腿，额外请求的1600N/s斜率时间约12.7ms，且旧释放前未削弱完整控制窗口约130ms。它没有明显越出现有请求/执行器规模，但这些量不是新实跑能力证书。

上表膝参考不是简单把旧已保护命令加一个量：已按真实质心Jacobian、旧髋参考与active pitch-rate参考重算逆解，旧逆解复现误差≤4.98e−5rad/s；样本限定原release前、travel=1、wheels_airborne=0。该固定窗口13.16–13.28低于15，但**新release会推迟，新的构型与新窗口没有记录**，所以不能保证完整新THRUST不会碰15或触发flight保护。

## 7. 离地姿态、首触及后退风险

|试次|原生离地pitch° / rate rad/s|真实首触pitch° / rate rad/s|首触点锚定最大后退cm|安静恢复起点s|
|---|---|---|---:|---:|''')
for r in data:
 l=r['existing_landing'];put(f"|{r['run']}|{r['off_pitch_deg']:.3f} / {r['off_rate_native']:+.4f}|{l['contact_pitch_deg']:.3f} / {l['contact_rate']:+.4f}|{100*l['max_backward_axle_m']:.2f}|{l['full_stability_start_s']:.3f}|")
put('''
不能称当前着地稳定性已经通过。首触约−4～−5.5°且负角速−0.38～−1.18，后退42.8～47.7cm；最终回BALANCE不抵消这些失败性能。

提高jump_height虽只改一个参数，却会通过同一target同步影响COM力反馈、膝速度逆解、速度锁存及卸力：release速度阈值从0.78*2.21472=1.72748变到1.95443m/s；95%速度锁存门从2.10399到2.38039m/s。这可能改变地面冲量/接触序列、离地q/dq与角动量，不能把空中初态当作不变。髋反作用补偿仍只有原gain0.15和max2.0Nm，其他姿态上限不变，不得同时提高补偿去掩盖后转。

基于实际接触点/力的一个有限敏感性算例：仅按原竖直力分布比例增加5.095N·s，用当时COM处的真实竖直冲量加权力臂约−5.4～−6.7mm及离地locked Ixx约1.24–1.29kg·m²，附加pitch-rate量级约−0.022～−0.027rad/s。**这不包含水平接触力变化、COP改变、关节残速内部角动量或轮反馈，绝非后转风险上界。**此前已验证的髋/膝角动量交换量级远大于此，必须检查实际新离地状态和空中机械分账。

触地风险更直接：在同触地高度、同目标速度损失假设下，额外竖直动能约11.7–12J，触地下降速度约增加17–18%。若仍按8cm缓冲行程设计，额外平均减速力需求约146–149N（两腿总量）；实际缓冲量与峰值冲击尚未知，不能由平均能量算出通过。飞行时间增加不自动改善首触姿态，负pitch_rate若持续，额外空中时间也可增加后倾。

因此优先方案是**单因素验证候选，而非默认采用方案或20cm稳定落地保证**。下一轮若获批准，应同时记录真实每次净空、离地q/dq/pitch-rate、首触pitch/rate、实际部署/缓冲、后退及恢复；任何保护退出、失稳或落地明显退化均不能以高度过线抵消。这里未新增控制安全门、放宽旧门或创建新测试框架。没有证据支持在本轮给出“新首触角度/后退距离”的确定预测。

## 8. 可复现交付与停止

[主分析](analyze.py)重算已有三次原生COM/接触冲量、执行器输入、弹道需求及一个设计点；[收腿时间/几何算例](tuck_bound.py)只算有限段条件边界；[报告生成](report.py)。不调用ROS、不启动仿真。断言及解析重算仅为数据/模型一致性检查，不称单测或物理验收。

本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008/`，含summary.json、tuck_bound.json、各次thrust_inputs.csv、ballistic_and_thrust.png、three_torque_types.png、分析日志及provenance.json。三类扭矩曲线明确区分已发布命令、物理步前实际输入和步后合负载；条件模型曲线标注MODEL。原始资料在clearance_apex_ab_20261008/physical/B1,B2,B3及对应analysis，原位置保留。

**完成后停止，等待批准。未实施0.32，默认控制、默认配置和gain1.60保持；没有新实跑、构建、提交或推送。**
''')
(P/'REVIEW.md').write_text('\n'.join(lines)+'\n')
