# landing_capture_gain 的触地前控制链审查

2026-10-07。本轮仅源码与既有七次完整 Gazebo 数据分析；新增实跑/腾空/达标均为0。未修改控制源码、参数、限制或状态机，默认 gain 保持1.60。A1/B1/A2/B2 实际gain=1.60，T1/T2/T3实际gain=1.00；不以最初期望标签代替实际参数。A3中断观察不加入完整比较。

## 结论与证据强度

**约−6°首触后倾不是触地才产生：推地末段/空中 ATTITUDE_ARREST 已出现向后转，TUCK 的残余腿速制动与反向参考随后继续改变机体姿态；进入 EXTEND 后，腿部追踪落脚终点，后倾在接触前已形成。landing_capture_gain 在触地前改变该终点，并可能通过可行性/剩余时间预算改变阶段时序，因而能够影响首触姿态。**

不能据此把均值−5.95°→−10.78°的全部变化归为该增益的净因果效应：baseline首触范围−14.84°～+0.42°，各试验进入空中的姿态、角速度和腿速并不相同。B1/T2入空时已明显后转；A2的规划速度低于死区，在冻结该输入时两个gain根本得到相同终点。本报告给出已证实的控制路径和逐次物理时序，不虚构同初态反事实物理响应。

## 源码：全部直接读取与间接使用

以下行号对应 `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`，与上一轮冻结版本相同。

|位置|实际用途|
|---|---|
|283、285–286、1127|参数声明、范围[0,2]及成员初值1.60|
|543|启动日志，非控制作用|
|3864 `preview_landing_target_x`|`forward_lead = landing_capture_gain_ * raw_capture`，规划预览|
|3880 `latch_landing_capture_plan`|`landing_capture_offset_ = landing_capture_gain_ * landing_capture_raw_offset_`，锁定落脚计划|

上述为该成员的全部直接读取。精确公式：

```text
vx_raw = predicted_landing_forward_velocity(now_sec)
vx = abs(vx_raw) >= landing_capture_speed_deadband_ ? vx_raw : 0
h = clamp(landing_capture_height_, 0.30, 0.50)
omega = sqrt(9.81/h)
landing_capture_raw_offset_ = vx/omega
landing_capture_offset_ = landing_capture_gain_ * landing_capture_raw_offset_
landing_target_x_ = clamp(landing_wheel_back_bias_ - landing_capture_offset_,
                          -0.12, landing_target_x_max_)
```

本组实际参数为 deadband=0.08m/s、height=0.40m、back_bias=0.12m、target_x_max=0.10m。`target_x>0` 表示机身在轮前、轮相对机身后移。3819起的预测速度优先使用新鲜原生世界速度投影，否则使用 `takeoff_forward_speed_`；不是用真实首触时刻速度重新计算。`landing_capture_comp_=atan2(target_x,max(0.20,L_TOUCH_))`仅日志展示，未再作为body pitch输入。

具体前触地调用链：

- 3922–3934 `plan_tuck_round_trip` 使用预览落脚IK终点进行往返可行性规划。
- 4080–4104 `estimate_remaining_to_touchdown` 在 FLIGHT/ATTITUDE_ARREST 开始使用预览IK构型及质心高度估算触地剩余时间；4254/4265等条件据此决定收腿预算。**从最初空中阶段已间接生效，日志 `landing_capture_planned=0` 不代表参数未使用。**
- 4110–4151 `begin_tuck` 的收腿IK本身target_x=0，不直接乘gain；但前述预算可影响何时进入它。
- 4350–4419 TUCK结束或落地期限到达后锁定 `landing_target_x_`，通过 `normal_landing_plan_feasible` 选择展腿时间，调用3988–4038 `plan_flight_joints`，生成四个关节五次轨迹进入 EXTEND。
- 4155–4192保护部署也锁定计划；4499–4502保护重规划继续使用该target_x。本组不以该分支解释正常展腿。
- 4564采样目标q/v/a，4569–4578可能执行 `enforce_wheel_first_target`；6614–6636写入关节目标日志，6660–6683 `discrete_flight_pd`将目标与实际q/v变为腿输出。**gain最终作用于腿部输出，不是简单轮速系数。**
- 4053–4077的 `air_pitch_ref` 平滑至 `balance_offset_+flight_landing_pitch_bias_`，不直接含gain；4644–4647在 EXTEND/PROTECTIVE_DEPLOY 令空中髋姿态附加力矩限额为0，主要由腿轨迹跟踪及轮控制共同决定机体响应。
- 4751–4779空中轮律读取实际pitch/pitch_rate，限幅后经4791–4816地面捕获混合与变化率限制，再于6551–6562发布。gain改变腿/机体后，轮反馈和几何接近地面的混合时间会间接改变。
- 4901–4934 `landing_capture_target` 是另一函数：调用 `centroidal_catch_target`，使用COM速度、偏移、高度及shank_rate；**不直接读取 landing_capture_gain_ 或 landing_target_x_**。名字相似不代表同一增益。状态切换条件也不直接乘gain，但前述几何/时间/接触判据会受运动影响。

## 数据口径与复现

原始完整输入为本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/{A1,B1,A2,B2,T1,T2,T3}/`。包括 `velocity_log.csv`、`geometry.csv`、`ground_frames.csv`、`native_wrench.csv`、运行参数和launch日志；版本/完整性见上轮报告及 `raw_sha256.json`、`native_step_checks.json`、冻结哈希清单。没有覆盖原始数据。

以原生轮地碰撞在真实腾空后首次重新出现定义t=0，非控制状态切换。原生1ms步；控制日志约5ms，映射采用不晚于物理时刻的最新控制行，状态/发布命令定位精度受此限制。真实pitch从原生基座四元数取负roll，pitch_rate为 `-post_base_world_wx`；与CSV滤波IMU信号分列，尤其触地冲量不能混用。碰撞 `num_contacts` 是接触点数，不是轮数。

分析程序：[analyze.py](analyze.py)。工作区根运行 `python3 experiments/jump/planning/landing_capture_pitch_chain_20261007/analyze.py`。依赖numpy/matplotlib；只读原数据，输出派生切片。

本地派生资料：本目录每个试验的 `controller_window.csv`、`geometry_window.csv`、`contact_window.csv`、`native_window.csv`、`summary.json`、`window.png`，覆盖−0.5～+0.3s。九组图包含原生/IMU姿态与角速度、左右髋膝实际/目标位置、实际腿速、两轮位置/速度、轮控制输出、capture目标、控制阶段及实际碰撞。轮位置图减去各自真实首触位置，原始绝对关节位置仍保留在切片。非FLIGHT阶段部分 `*_pos_cmd_*` 字段是缓存值，不能当成THRUST当时实际启用的位置轨迹；实际施加腿输入须看native字段。`landing_target_x=0`在未锁定时不能当成预览公式实际为零。

## 逐次后转起点

为可复核地定义“明显向负方向变化”，仅在分析中采用：原生pitch_rate连续15ms低于−0.20rad/s，随后50ms内pitch下降至少1°，且该50ms在首触之前。不修改控制门限，也不筛除短暂回正。以下是窗口内首次满足时刻；pitch仍为正也可以正在后转。

|试验/gain|真实首触sim s|后转起点相对s|控制阶段|起点pitch °|pitch_rate rad/s|碰撞点数|左右髋速 rad/s|左右膝速 rad/s|发布cmd_x m/s|
|---|---:|---:|---|---:|---:|---:|---|---|---:|
|A1/1.60|9.419|-0.404|ATTITUDE_ARREST|5.53|-0.235|0|6.62/6.62|-11.90/-11.90|0.513|
|B1/1.60|9.347|-0.448|THRUST|6.94|-0.245|2|8.47/8.47|-13.89/-13.89|-0.162|
|A2/1.60|9.348|-0.456|THRUST|7.15|-0.243|2|6.29/6.29|-9.78/-9.78|-0.149|
|B2/1.60|9.377|-0.373|ATTITUDE_ARREST|6.61|-0.211|0|5.10/5.10|-9.16/-9.16|0.267|
|T1/1.00|9.386|-0.391|ATTITUDE_ARREST|5.62|-0.231|0|6.54/6.53|-11.72/-11.72|0.478|
|T2/1.00|9.452|-0.431|THRUST|5.98|-0.231|0|8.65/8.65|-14.95/-14.95|-0.008|
|T3/1.00|9.370|-0.391|ATTITUDE_ARREST|5.63|-0.255|0|6.73/6.73|-11.71/-11.71|0.413|

A2的−0.456s为推地末段短暂后转，随后恢复至约+7.32°；主要后转再次从约−0.373s ATTITUDE_ARREST开始（pitch_rate约−0.265rad/s）。T2起点控制状态仍THRUST，但该物理步碰撞为0：控制状态与真实接触并非同步。B1起点仍有接触。不能写成“七次最早全部在ARREST”，也不能写成“接触瞬间造成最初后倾”。

|试验|TUCK开始相对s / pitch°|EXTEND开始相对s / pitch°|首触前50ms pitch°|真实首触pitch° / rate rad/s|
|---|---|---|---:|---|
|A1|-0.383 / 4.47|-0.323 / 0.67|-5.83|-6.39 / 1.014|
|B1|-0.382 / 1.28|-0.322 / -2.58|-12.44|-14.84 / -1.522|
|A2|-0.358 / 6.40|-0.298 / 2.55|-2.04|-2.98 / -1.333|
|B2|-0.359 / 6.00|-0.289 / 1.46|1.77|0.42 / -1.096|
|T1|-0.367 / 4.02|-0.307 / -0.42|-7.64|-9.20 / -1.328|
|T2|-0.376 / 1.63|-0.311 / -2.58|-9.09|-10.86 / -0.899|
|T3|-0.365 / 3.77|-0.300 / -0.75|-10.50|-12.29 / -1.387|

以A1为例，ARREST后转起点髋仍约+6.62、膝约−11.90rad/s；TUCK开始pitch+4.47°，EXTEND开始+0.67°，触地前50ms已−5.83°，真实首触−6.39°。因此主因链需要检查推地结束的大腿速、空中制动和展腿反作用，不是先从触地后的参考突变解释。

七次 `FEASIBLE_TUCK` 的q终点都等于q起点，但开始腿速非零，轨迹终速设为0（4110–4151；`reaction_safe_configuration_step`在反向位移且当前速度较大时保留原位置）。这些轨迹不是腿完全静止，而是制动后返回原q，目标速度发生反向；例如A1左髋目标范围−0.972～+1.905、左膝−5.317～+2.712rad/s。身体在此期间持续后转。此为实际目标/响应证据；腿与机身有反作用耦合，但本轮未做完整角动量分项归因，不把单个腿输入积分冒充机体净力矩。

## gain对触地前腿目标的实际贡献

下表采用每次日志实际锁定的 `landing_capture_vx`，仅将同一个公式中的gain分别代入1.60/1.00，并解同一IK。是**源码代数反事实**，不是同初态两次真实物理结果。实际值由该试验gain列选择。目标轨迹/实际q与真实腿输入在每次切片中保留。

|试验|锁定相对s|vx m/s|x(1.60) m|x(1.00) m|差 cm|髋IK差(1.00−1.60) rad|膝IK差 rad|
|---|---:|---:|---:|---:|---:|---:|---:|
|A1|-0.323|0.394595|-0.007487|0.040320|4.781|-0.078689|-0.014584|
|B1|-0.322|0.401779|-0.009808|0.038870|4.868|-0.080700|-0.013838|
|A2|-0.298|0.032806|0.100000|0.100000|0.000|0.000000|0.000000|
|B2|-0.289|0.511141|-0.045141|0.016787|6.193|-0.113515|0.001905|
|T1|-0.307|0.416922|-0.014701|0.035812|5.051|-0.085000|-0.012151|
|T2|-0.311|0.428354|-0.018394|0.033504|5.190|-0.088299|-0.010773|
|T3|-0.300|0.402782|-0.010132|0.038667|4.880|-0.080982|-0.013731|

T1/T2/T3在实际首触前约0.30～0.31s已锁定并执行展腿。降低gain在相同规划速度下将轮轴目标相对机身后移约4.88～5.19cm，髋终点减少0.081～0.088rad（约4.6～5.1°），不是只改变触地后的平移目标。腿部轨迹、身体反作用、反馈轮速和落地几何因而改变，首触pitch可能恶化。直接输出贡献不是一个可从最终轮指令减掉的固定gain项：最终腿输出还依赖实测q/v、轨迹时刻和轮优先约束，轮输出则由改变后的反馈状态间接产生。仅有不同初态物理试验不能给出净pitch贡献数值。

## 轮端限幅事实

轮输出topic为 `/diff_drive_controller/cmd_vel`（TwistStamped linear.x，m/s），不是轮电机力矩。空中分支4751–4779限幅±2m/s，之后仍有接地混合/变化率限制；不能将±2误当成所有阶段最终输出限制。diff_drive配置 `src/bbot_bringup/config/bbot_controllers.yaml` 的linear速度上限为±5m/s。URDF轮关节名为link_004_joint/link_007_joint，名义velocity=30rad/s、effort=50Nm；velocity接口声明±80rad/s，未激活effort接口的±10Nm不能套作本次速度驱动的已知电机上限。

|试验|首触前最终cmd峰值 m/s|空中原始指令触及2的控制行数|最终空中发布达到2的行数|前0.5s实际轮速峰值 rad/s|后0.3s最终cmd峰值 m/s|后0.3s实际轮速峰值 rad/s|
|---|---:|---:|---:|---:|---:|---:|
|A1|2.000|11|6|28.571|1.156|16.509|
|B1|2.000|37|20|28.571|2.146|30.000|
|A2|1.732|1|0|24.746|0.812|11.599|
|B2|1.269|1|0|18.122|0.466|13.419|
|T1|2.000|13|8|28.571|1.123|16.047|
|T2|1.747|0|0|24.951|1.181|16.874|
|T3|2.000|10|10|28.571|1.206|17.230|

四次A1/B1/T1/T3空中发布确实达到2m/s；A2/B2原始空中反馈曾超限，但最终经过slew未到2；T2没有触及。全部未触及diff_drive的5m/s上限。B1触地后实际轮速达到约30rad/s平台，命令2.146m/s对应约30.66rad/s，高于URDF名义30；只能确认观测到名义速度边界，不能仅据声明确定后端哪一级限速。

**轮电机力矩饱和仍未知。** 本次冻结观测程序输出的107列native CSV中，轮BeforePhysics JointForceCmd均invalid，也没有物理步JointVelocityCmd列；joint_states effort=0不能当真实输入为0。`transmitted_axis_torque`是关节合负载，不是电机施力。当前观测源码已含速度命令字段，但该字段不在这些旧记录中，不能追补成历史事实。本轮不启动补测、不修改记录器或控制器。

## 最小后续建议（未实施）

保持gain=1.60及所有落脚几何/限制。优先只处理 `begin_tuck` 的速度边界：当前“非零入段速度、同位置终点、强制零终速”产生制动和反向返回，建议把TUCK终速与随后EXTEND所需方向衔接，作为单一轨迹边界因素验证，而不是继续扫描gain或扩大轮端补偿。不能从本轮日志承诺此方案必然稳定；应在实施前明确唯一改动及同条件对照。

本轮到此停止；未实施建议，无新增仿真、构建或控制修改。完整七次原始样本、失败/振荡样本与缺失输入均保留。
