# 2026-10-07 默认 velocity 跳跃性能：第一阶段分析

本轮问题：现有完整跳跃为什么净空低、落地后后倾并后退？通过条件：绑定当前默认源码与既有原生 Gazebo 记录，分清起跳、空中、真实接触及恢复的信号与时序，给出一个最小修改候选；本轮先分析，不修改控制、不新增实跑。此报告不继续独立 contact-motion 的 manager/恢复框架。

## 证据与复现

分支 `work/jump-repair`。输入为本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/velocity_clearance_landing_20261005_224308/` 下 `velocity_log.csv`、`geometry.csv`、`ground_frames.csv`、`native_wrench.csv`、`wheel_joints.csv`、`legs_joint_feedback.csv`、`leg_modes.csv`、`launch.log`、`runtime_parameters.yaml`、`launch_command.json`。这是**既有一次完整实际 Gazebo 跳跃**，不是本轮新运行或模型回放。

当前 `bbot_velocity_jump_controller.cpp` SHA256 `a17c95bd35eb43b07051ed66777070776bb0d9b4d579ab62fe270e94a7dc09c6` 与当次冻结一致；当前主 launch 哈希也一致。运行命令为 README 的 `controller_type:=jump_velocity jump_height:=0.25`，附加独立日志路径及只读原生观测世界。运行参数来自当次 YAML，不能以脚本默认值代替；尤其旧 `run_flat_ground_jump_trial.py --observe-repeatability` 强制的是旧 0.20 m / 旧推地参数，不能直接拿来做本轮单因素对照。

复算：在工作区根执行 `python3 experiments/jump/planning/velocity_performance_analysis_20261007/analyze.py`。分析脚本、`inputs_sha256.json`、`analysis.json`、完整窗口抽取 CSV 和图片是本轮本地分析产物，不是新的控制或测试框架。原始文件未修改。原生大文件只读取 3.9–6.2 s 段。

## 入口、信号和不能混用的数据

- 默认源：[bbot_velocity_jump_controller.cpp](../../../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp)。launch 选择该 executable（主 launch 278–280、1119 起），轮侧 `/diff_drive_controller/cmd_vel` 是 `TwistStamped.twist.linear.x`，CSV 为 `cmd_x`，单位 m/s，**不是轮力矩**（源 714–721、6551–6562）。左右轮关节为 `link_004_joint`、`link_007_joint`，半径 0.07 m；diff-drive 线速度限幅 ±5 m/s，空中另有 ±2 m/s 限幅。
- 腿输出 `/leg_effort_controller/commands`；四关节顺序 `link_002_joint/link_003_joint/link_005_joint/link_006_joint`。CSV `actual_tau_hip_left/right`、`actual_tau_knee_left/right` 是控制器算出并发布的力矩命令，不是实测电机力矩。`BeforePhysics JointForceCmd` 才是该观测链中物理步前的仿真力输入；`transmitted_axis_torque` 是关节合负载。
- `/imu` 回调：`pitch_=-roll`、`pitch_rate_raw_=-angular_velocity.x`；`pitch_rate_` 为低通量（源 1728–1750）。原生 `geometry.csv` 用 base quaternion 得到 `-roll`。两种姿态在本记录有约 2.18°零点差及采样差，图中分开呈现，不把控制器角度冒充原生绝对姿态。2026-10-07 配对实验前纠正：原生同帧 `post_base_world_wx=-0.569381`，故前倾正方向 `pitch_rate=-wx=+0.569381 rad/s`。此前把它解释为负向是符号错误；接触前负向、接触冲击后已转正，两者不能混用。
- `x`、`x_dot` 是轮里程计反馈，腾空自转期间不能代表世界平移。真实后退用原生轮轴位置、世界重心位置计算；速度检查使用 `capture_com_velocity`，不能据飞行中的轮里程计变化声称机体已经后退。
- `wheel_joints.csv` 和 `legs_joint_feedback.csv` 的 effort 在选定窗口全部报告 0；不能由此认定实际力矩为零。两轮原生 `BeforePhysics JointForceCmd` 全部缺失，保持 invalid，**现有记录不能判断轮执行器力矩是否饱和**。只可确认轮速目标限幅，以及关节合负载冲击。

## 起跳目标与实际响应

当次参数：`jump_height=0.25`、`takeoff_velocity=0`，因此 `target_takeoff_velocity=sqrt(2*9.81*0.25)=2.2147 m/s`；`thrust_duration=0.24 s`、`thrust_timeout=0.60 s`、`thrust_velocity_kp=8`、`thrust_peak_ratio=2`、`thrust_early_ratio/thrust_late_ratio=0.75/0.75`。`jump_height`是目标速度换算量，不保证实际轮底净空。

下蹲 `L_SQUAT_=0.34 m`、`T_SQUAT_=0.50 s`，名义推地高度 `H_TAKEOFF_=0.475 m`（源 363–367）。THRUST 五次轨迹终端速度 0.8 m/s（2814–2821）；实际输出并非只跟踪这一高度：推力脉冲叠加 COM 竖直速度反馈，反馈前 0.06 s 渐入（2972–3004）；髋参考速度由轨迹差分、膝参考由目标 COM 速度逆解形成（3118–3214）。因此不存在一个固定的“全程腿目标速度”，图中保留实时 `hip_vel_cmd_left`、`knee_vel_cmd_left` 与实测值。

| 项目 | 既有实际记录 |
| --- | --- |
| THRUST 首条日志→FLIGHT 首条日志 | 4.108→4.297 s，共 0.189 s；运动计时到 0.165 s，未走满名义 0.24 s |
| 左髋/右髋位置，THRUST首条→FLIGHT首条 | −0.151509/−0.151509 → +0.602826/+0.602777 rad |
| 左膝/右膝位置，同上 | +0.179082/+0.179082 → −0.822507/−0.822489 rad |
| 离地首条左右髋速度 | +8.16938/+8.16975 rad/s（小数末位以原CSV为准） |
| 离地首条左右膝速度 | 约 −13.60/−13.60 rad/s |
| THRUST 最大左右位置差 | 髋 0.000254 rad，膝 0.000357 rad；不是左右不同步 |
| THRUST 髋/膝实际速度极值 | 约 +8.05 / −13.96 rad/s，已超过同段目标峰值约 +1.94 / −11.86 rad/s |
| THRUST 单侧发布力矩 | 髋 −3.68…+9.81 Nm，膝 −77.58…+17.58 Nm；37条控制样本无最终限幅命中 |
| 当时力矩上限 | `sim_relax_thrust_limits=true`，THRUST为150/150 Nm；FLIGHT回75/60 Nm。不能套用独立接地实验的75/60限幅解释该段 |
| THRUST控制器pitch / pitch_rate | pitch约 +0.050…+0.068 rad；离地附近 +0.050 / −0.204 rad/s |
| 离地COM竖直速度 | 2.08238 m/s，约为目标94%；入空中后峰值2.10843 m/s |

图：`takeoff_flight.png`；三类髋膝扭矩曲线：`leg_torque_sources.png`。图中分别标为发布命令、物理步前输入及关节合负载，均来自既有实际运行，无模型曲线。

### 离地和触地判据

当前同时有两条离地确认来源：

1. 对齐的双轮净空通常≥12 mm（累计中6 mm），至少两帧跨10 ms，结合上升和卸载/20 mm几何证据，见 `takeoff_detection.hpp:48–71`。
2. 本次启用 `complete_contact_takeoff_confirmation`：完整接触源连续零接触至少11帧、跨度≥10 ms，近期双轮支撑、推地门已开且新鲜 COM vz>0.35 m/s等条件，见 `complete_contact_takeoff.hpp:297–317`。速度达到目标95%的锁存是独立证据，不是这条接触确认的必要条件。

原生最后接触帧4.286 s、其后持续无接触从4.287 s起，FLIGHT首条4.297 s；没有证据显示该次**提前离地切换**削掉仍可通过地面接触获得的冲量。更早有短暂零接触，不把它当作持续离地。

FLIGHT触地为 `persistent_contact || leg_compressed || torque_spike || imu_impact || sustained_support`（源4821–4852）；几何确认需下降、净空≤2 mm、比力≥3、两个样本跨10 ms，另有腿长压缩等分支。本次触发日志明确为**腿压缩**，当时控制器估计净空24 mm、比力约0.1，仍在空中；首条BUFFER为4.687 s，真实首次轮接触4.695 s，前一条FLIGHT为4.682 s。状态切换发生在这两个控制样本之间，早于真实轮接触约8–13 ms；不能把状态名当作真实接触证明。这是最小逻辑修正的候选，但时间短且后倾早已形成，不认定它是全部后退的根因。

### 跳不高：逐项判断

- **伸展幅度不足：不支持简单结论。**实际腿长到FLIGHT约0.611 m，超过名义0.475 m；腿在惯性继续伸展，不能只提高名义终点。
- **伸展速度不足/执行器跟不上：不支持。**实际髋膝速度高于目标，离地还残留高速；目前更像缺少有效地面末段减速，而非执行器太慢。完整跟踪误差见图，不能用一个峰值代表全段跟踪通过。
- **加速时间太短：名义时间未走满是事实，但因真实接触已经消失而退出。**简单延长 `thrust_duration`不能恢复接触后的冲量，不能把“0.189<0.24”直接定为根因。
- **轮控抵消起跳：未证实。**有姿态和关节运动学修正，轮命令末段转正，但缺实际轮力矩，不能量化其对竖直冲量的贡献。
- **提前离地：本记录未见；提前触地状态切换另列。**
- **饱和：推地最终髋膝命令无150 Nm限幅命中；空中轮命令明确达到2 m/s、轮速28.5714 rad/s。轮执行器力矩饱和未知。**
- **直接净空问题：空中有效收腿很少，且过早再展腿。**4.337 s TUCK计划从 `(hip,knee)=(0.838,−1.229)` 到同一组角度；主要是把残余速度刹下去。4.397 s即EXTEND，比COM最高点4.517 s早120 ms；双轮轮底峰值在4.522 s仅0.153368 m。同一日志的 `com_world_z` 从FLIGHT首条0.427444升至0.676189 m（+0.248745 m；两个时刻并非完全相同采样年龄），不能把这个COM升幅当作轮底净空。腿收展几何对轮净空有直接影响；增加推力不是本轮第一改动。

## 落地前0.3 s到后1.5 s

窗口严格以**原生真实轮接触4.695 s**为零点，抽取4.395–6.195 s，见 `landing_controller_window.csv`、`landing_window.png`。

| 时刻 | 状态/现象 | 控制器量（不是原生接触绝对角） |
| --- | --- | --- |
| 4.297 | FLIGHT/ATTITUDE_ARREST | pitch +0.050 rad；髋+8.17、膝−13.60 rad/s |
| 4.337 | TUCK开始 | pitch +0.049；髋+1.42、膝−4.84；原始角速已明显负向 |
| 4.382 | TUCK末段 | filtered pitch_rate约−0.872 rad/s；不是接触才产生后倾 |
| 4.397 | EXTEND开始 | pitch约−0.026 rad；腿速度已很小，但机体后倾角速度仍在 |
| 4.687 | BUFFER/CATCH首条，尚未真实轮接触 | pitch−0.140、rate−0.446；COM在轮轴后约0.111 m；cmd_x +0.560 m/s，capture目标−0.182 m/s，受斜率限制不能立即切到新目标 |
| 4.695 | 真实首次轮接触 | 原生后倾5.739°、控制方向角速+0.569 rad/s；带后倾撞地，但同帧冲击后角速已转正 |
| 4.751 | CATCH | pitch−0.039、rate+1.057；COM速度已转负−0.137 m/s；cmd_x +0.592 m/s |
| 4.914–4.983 | REVERSE_BRAKE→CATCH | 反向制动约69 ms，随后`capture_diverged`释放，并已消耗本跳一次性制动资格 |
| 5.202 | CATCH | pitch已+0.063 rad，但COM仍−0.441 m/s；cmd_x +0.609 m/s，后退与持续负pitch不是同一件事 |
| 5.744 / 6.009 | PREPARE / HOLD首条 | 约触地后1.05/1.31 s；COM倒退到约5.8 s才过零 |
| 7.923 / 9.228 | RECOVERY / BALANCE首条 | 不能把前面的HOLD当作已完成全部恢复 |

真实轮轴在首触后1.5 s内已后退约40.60 cm；原完整窗口最大40.68 cm、COM最大28.36 cm，与原生验收复算一致。原生pitch很快转正，控制器pitch约0.2 s后转正；“持续后倾”应拆为**空中形成后倾→后倾首触→轮轴后退捕获→恢复摆动**，不是整个1.5 s都维持负机身角。

### 逐项因果核查

1. **落地前是否已经带pitch/rate：是。**高速腿在ARREST/TUCK约前100 ms被减速，与原始角速度负向峰值−1.51 rad/s时序相近；随后EXTEND时仍后倾，空中轮速已限幅。浮基反作用是有根据的机制候选，但仅凭同步变化不能把髋、膝或轮各自的净贡献定量归因。FLIGHT不调用地面 `allocate_torso_hips`，不能解释成“地面torso分配预算被挤占”。
2. **平衡目标是否突变：有控制律切换，没有瞬时大轮命令跳变。**空中姿态参考从离地角平滑趋向 `balance_offset_+flight_landing_pitch_bias_`（本次约0.085 rad）；触地切换为地面重心捕获。首条BUFFER的目标从空中正命令转负，但实际命令保留并按8 m/s²限斜率。腿从实测角锁存交接，`landing_joint_handoff_duration=0.16 s`，不是直接一步跳到缓冲IK。
3. **落地后单向大wheel torque：当前证据不能称为torque。**观测到的是大段正向速度命令；正cmd对应支撑轮向物理后方运动。合负载有冲击峰，但不是速度执行器电机力矩；不能由其认定控制器力矩限幅。
4. **位置误差追回旧参考：可以排除为本次主要机制。**触地时 `touchdown_x_ref_`从旧约0.101变为0.165787 m，`target_x_`同步重置（源4860–4867）。有效 `landing_capture_target()`覆盖旧位置/速度/姿态fallback，RECOVERY同样覆盖（4901–4934、5114–5146）。窗口保存 `x`、`x_dot`、`x_error`、`target_x`、`touchdown_x_ref`；原始位置差可复算，但当capture有效不实际参与轮目标。
5. **腾空积分/状态残留：未见位置积分驱动后退的证据。**常规 `vel_integral_`不在FLIGHT/BUFFER分支继续累加；另一个真实积分量为 `capture_v_ref_`，仅捕获静稳门内学习，门外回零（4954–4998）。初触与早期后退时该量为0。窗口后期会开始学习，但不能解释最初负向运动。
6. **为什么后退继续：重心位于轮轴后，地面捕获仍要求轮轴后移。**核心为 `-1.25*capture_com_velocity - sqrt(g/h)*com_forward_from_axle - R*shank_rate + 0.25*v_ref`，见 `jump_phase_control.hpp:504–518`；这是轮相对腿速度目标，不可单看第一项就认定符号错误。4.751 s三项分别约+0.171、+0.481、+0.093 m/s，相加即目标+0.744；5.202 s约+0.551、+0.056、+0.002→+0.609。正命令让轮轴后移以捕获后方COM，故机身角恢复不等于已经止退。4.914 s的反向制动曾减小后退，但4.983 s捕获状态发散后释放；源码一次性`consumed`锁存使本跳之后不再触发同一制动。不能直接删除释放条件，否则可能用姿态恶化换短停止距离。

## 最小修改方案与本轮停止点

**第一候选只改落地轮轴布置参数 `landing_capture_gain: 1.60 → 1.00`，保持其余参数、腿动作时长、轮限幅与恢复控制不变；不采用TUCK增益候选。** 源位置：默认控制器3860–3903 `preview_landing_target_x/latch_landing_capture_plan`，2026-10-07 配对实验前纠正：主launch未转交同名ROS参数；独立参数入口必须显式注入并核实节点dump，不能仅追加同名launch配置。

证据：当前锁存前速约0.415 m/s、omega≈4.952，`raw_capture≈0.084 m`，`forward_lead≈0.134 m`，`back_bias=0.120 m`，最终 `landing_target_x≈−0.014 m`。源约定正target_x为轮子相对机身后移。只降低这一gain至1.00，在同一前速下目标约+0.036 m，即将预定轮轴相对机身后移约5 cm，方向上减少首触COM落在轮轴后方的缺口，预计减少落地捕获所需的后退。它**不保证消除空中后倾**，实际关节跟踪、机身反作用和首触状态须由动态跳跃验证；若没有改善即保留失败记录，不同时加别的参数。

另查本地历史 `tuck_kd_probe_20260928/RESULT.md`：0.45→0.90的A–B–B–A并未改善，四次均保护退出；源码版本较旧，不用其绝对值替代当前基线，但不再把“只加TUCK D”当作已有证据支持的优先方案。此前commentary提出该候选，现依据历史撤回。

本轮完成的是用户要求的**第一阶段分析**，控制源码、默认参数、默认程序均未改；没有修改后动态测试，也没有宣称性能已改善。后续第一轮只执行上述一个参数的独立实验跳跃，使用当前README参数基线及原生轮位姿，检查成功离地/落地、首触姿态是否恶化、后倾持续时间、实际轮轴/COM后退和恢复时间；不混用旧runner基线，不立即失败重跑。净空提高放在落地改善之后，暂不动推力或收腿参数。

本轮新增真实实跑/腾空/性能达标均 **0/0/0**；本报告主对照为历史1次完整跳跃，存在高度与落地性能问题。独立接地修复线的4/0/0及其准入结论另存，不能据此声称默认不能跳，也不能据默认能跳声称独立模型资格通过。无提交、无推送；按“做完本轮停下”在此停止。
