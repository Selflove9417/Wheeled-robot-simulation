# agents4.md — BBot 跳跃控制调试交接（更新：2026-09-08）

## 当前结论与待办

- 用户已确认 **v6.11 落地后可以稳定恢复**。不要再把“落地后必然失稳”作为当前基线。
- v6.12恢复了RECOVERY中的行驶、停车与重复跳跃入口；v6.13把THRUST质心竖直速度参考修正到可达到约1.98m/s峰值。当前主要问题已经从“跳不起来”转到**空中后半程与落地几何**。
- v6.14解决了原TUCK/EXTEND固定短时长导致的超速度、超加速度轨迹问题；完整收腿超预算时会进入`PROTECTIVE_DEPLOY`。
- v6.15在Effort模式`PROTECTIVE_DEPLOY`中加入姿态退出稳定区后的连续减速保护，明显降低了触地前机身后仰：失败日志中触地附近pitch从v6.14约-0.85rad改善到v6.15约-0.155rad。但该版本会让腿停在中间构型，触地时COM仍在轮轴后约0.195m。
- v6.16尝试在v6.15姿态保护基础上，搜索“连续制动 + 最大可行landing progress”端点。最新172列日志确认该逻辑运行：7.361s触发姿态保护，规划`T=0.125s`、`protective_landing_progress=0.652562`、`protective_landing_joint_alpha=0.48`，完整落地COM目标为`+0.040091m`，规划端点仅为`-0.019973m`。
- **v6.16仍然轮子偏前。最新日志已经定位出更根本的问题：保护展腿轨迹用关节坐标`q_hip`规划，但真实腿在世界中的大腿绝对角由`q_hip - pitch`决定。机身pitch在轨迹执行期间显著变化，使规划末端的wheel-first条件失效。**
- 最新日志在7.474s到7.479s之间出现明确目标跳变：`knee_pos_cmd_left`从`-0.889570rad`突变为`-1.128920rad`，单周期变化约`-0.239rad`；同时`knee_vel_cmd_left=-2.317610rad/s`，恰与当时`pitch_rate=-2.317610rad/s`一致。该跳变来自姿态制动段结束后`enforce_wheel_first_target()`重新逐帧介入。
- 7.484s首次负轮间隙附近：`wheel_clearance=-0.000585m`、`pitch=-0.244470rad`、`com_forward_from_axle=-0.183694m`。因此“轮子偏前”仍然是在**触地前**形成，不应先去调地面CATCH。
- **下一步为v6.17设计，不是继续调`landing_wheel_back_bias_`：把保护展腿的髋轨迹改为世界参考坐标`eta=q_hip-pitch`，并把wheel-first约束直接写成`phi2_0 + eta + q_knee`。v6.17尚未实现，不能宣称已解决。**

## 当前代码与控制架构

- 主文件：`/home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`。
- 当前已运行验证的版本为 **v6.16**；启动标识：`[protective-landing-progress-v6.16]`。姿态保护日志：`[PROTECTIVE_ATT_BRAKE v6.16]`。
- v6.16日志为172列；在v6.15的167列基础上新增：`landing_com_forward_target`、`landing_com_forward_target_valid`、`protective_landing_progress`、`protective_landing_joint_alpha`、`protective_landing_com_end`。
- v6.15新增的4列仍保留：`protective_attitude_seen_stable`、`protective_attitude_brake_active`、`protective_attitude_brake_plan_check`、`protective_attitude_brake_duration`。
- 核心辅助文件仍在`src/bbot_balance_controller/include/bbot_balance_controller/`：`centroidal_state.hpp`、`jump_phase_control.hpp`、`torso_pitch_control.hpp`、`ground_joint_pd.hpp`、`thrust_velocity_reference.hpp`、`flight_trajectory.hpp`。
- 环境为ROS 2 Iron；主控制200Hz。当前调试中所有5ms级目标跳变都必须按单个控制周期对待，不能当作普通轨迹变化。
- 首次起跳前可使用Position；跳后保持Effort RECOVERY。`kHoldRecoveryAfterJump=true`，不会因静稳自动进入旧BALANCE或Position。
- 临近触地、CATCH、BRAKE、HOLD、RECOVERY继续共用质心轮控；当前问题发生在FLIGHT/`PROTECTIVE_DEPLOY`，不要先修改地面捕获律。
- 地面质心状态使用`centroidal_balance_.forward = COM_x - axle_x`。正值表示COM在轮轴前，负值表示轮轴在COM前。
- 当前完整landing IK对应的`landing_com_forward_target`在最新跳跃中为`+0.040091m`，说明完整落地目标方向本身不是“把轮子放前面”。实际失败来自该目标未按世界几何持续兑现。
- `enforce_wheel_first_target()`当前使用`theta_shank = phi2_0 + q_hip + q_knee - pitch`，并在几何超限时直接修改`q_knee`；在边界上还会令`qd_knee = pitch_rate - qd_hip`。
- 当前保护制动段为了保持五次轨迹连续性，在执行期间暂缓逐帧wheel-first硬夹；段结束后重新启用`enforce_wheel_first_target()`。最新日志已证明这种“段内不夹、段末突然恢复”的方式会在pitch变化较大时产生关节参考跳变。
- 现有仿真选项只在THRUST放宽到模型已有150N·m，其余阶段髋75/膝60；本轮问题不能通过继续提高轮速或关节力矩上限代替坐标修正。

## 用户操作与边界

- 出现`[READY v6.12]`后：W/S前后、A/D转向、空格停车；`/cmd_vel`使用同一目标通道。
- 行驶/转向参考上限沿用0.50m/s、0.60rad/s，参考变化率1.0/s。
- 地面质心轮命令请求仍可到5m/s，但实际轮关节URDF限速30rad/s、轮半径0.07m，轮面速度实际至多约2.1m/s。此前日志已出现30rad/s饱和；不要把5m/s请求当作实际轮速能力。
- 再次跳跃：先停车，再按J。RECOVERY入口要求已就绪、Effort激活、质心观测有效，并检查速度/姿态。
- Q/E高度调节仍只接原BALANCE入口，未扩展到RECOVERY。
- `vRef=0`仍使用v6.11以来的质心停车保持律；不要为了修空中落地重新切回旧Position/BALANCE接管。
- 当前调试原则：**一次只修一个因果链。** v6.17优先改空中保护展腿的坐标定义和wheel-first连续性；在该问题验证前，不同时修改THRUST、CATCH、`air_wheel_sign_`、轮速上限或固定后置偏移。

## 最新日志与下一轮检查

- 最新CSV：`jump_velocity_control_log.csv`，**v6.16，172列，1784行，时间0.012s到10.081s**。
- 关键时序：
  1. `7.361s`：`PROTECTIVE_ATT_BRAKE`触发；`pitch=+0.060893rad`、`pitch_rate=-0.671010rad/s`、`com_forward_from_axle=-0.132784m`。
  2. 同一时刻规划结果：`protective_attitude_brake_plan_check=1`、`protective_attitude_brake_duration=0.125s`、`landing_com_forward_target=+0.040091m`、`protective_landing_progress=0.652562`、`protective_landing_joint_alpha=0.48`、`protective_landing_com_end=-0.019973m`。
  3. `7.474s`：仍在制动段末，`pitch=-0.222154rad`、`pitch_rate=-2.317610rad/s`；左髋/膝目标约`+0.692089/-0.889570rad`，目标速度约`-0.029/+0.023rad/s`。
  4. `7.479s`：制动段结束后wheel-first硬保护重新介入，左膝目标从`-0.889570`跳到`-1.128920rad`；`knee_vel_cmd_left=-2.317610rad/s`。
  5. `7.484s`：首次负间隙附近，`wheel_clearance=-0.000585m`、`pitch=-0.244470rad`、`pitch_rate=-2.249020rad/s`、`com_forward_from_axle=-0.183694m`。
- 最新日志还给出一个重要事实：7.474s实际`hip_vel_left=-2.169450rad/s`、`pitch_rate=-2.317610rad/s`，因此`hip_vel_left - pitch_rate ≈ +0.14816rad/s`。即大腿在世界坐标中的角速度已经很小；但当前关节轨迹却趋向`hip_vel_cmd≈0`，等效要求世界大腿角速度约`-pitch_rate`，会随着机身旋转重新制造腿的世界角运动。
- 下一轮v6.17应重点验证：
  1. 保护展腿髋状态是否改为`eta = q_hip - pitch`，速度是否为`eta_dot = qd_hip - pitch_rate`。
  2. 输出关节参考时是否严格恢复为`q_hip_des = eta_des + pitch`、`qd_hip_des = eta_dot_des + pitch_rate`。
  3. wheel-first是否改为在世界参考轨迹内直接限制`phi2_0 + eta + q_knee`，而不是轨迹结束后再次硬改`q_knee`。
  4. 触地前是否消除类似`-0.239rad/5ms`的膝参考跳变。
  5. 在不恶化pitch/pitch_rate的前提下，`com_forward_from_axle`是否从当前约`-0.184m`明显向0或正值回归。
  6. 只有上述空中几何连续性通过后，再评估是否需要进一步做COM-relative landing placement或临地轮控调整。
- FLIGHT中的机身角速度判断继续使用`pitch_rate`；不要使用旧地面诊断量`torso_rate`代替。
- 离地确认速度不是精确接触分离时刻速度；`apex_world_z_delta`仍包含从下蹲开始的伸腿/站高，不等于净腾空高度。

## 已完成验证与约束

- v6.14历史版本已执行`colcon build --packages-select bbot_balance_controller --symlink-install`成功，`ctest` 12/12通过，`git diff --check`通过。
- v6.13历史版本控制器编译成功，`ctest` 11项通过；其THRUST质心速度参考修正已在后续Gazebo日志中实际运行。
- v6.15已有167列Gazebo日志，证明姿态保护逻辑实际运行；其主要收益是显著降低触地前机身后仰，但代价是腿停在中间构型。
- v6.16已有172列Gazebo日志，证明`protective_landing_progress`搜索逻辑实际运行；**不能因为规划值`protective_landing_progress=0.652562`就认为真实COM完成了65%落地点修正**，因为该评价使用了固定`landing_pitch_ref`，而实际pitch在轨迹期间从正值转为明显负值。
- 当前没有记录v6.15/v6.16完整`ctest`结果；不要补写“测试全部通过”。
- 最新日志已直接证明当前v6.16存在参考不连续：保护轨迹结束后`enforce_wheel_first_target()`可在一个5ms周期内大幅修改膝目标。v6.17必须优先消除这一结构性问题。
- 保留v6.15已经验证有效的姿态保护思想，不回退到v6.14强行高速完成landing IK。
- 不盲目翻转`air_wheel_sign_`，不通过提高轮速/力矩上限掩盖空中坐标问题。
- 不修改旧跳跃控制器；新版本仅围绕`bbot_velocity_jump_controller.cpp`当前分支继续迭代。
- 完成代码、编译及必要离线检查后仍由用户运行Gazebo；不要覆盖已有实验日志。

## 历史：v6.9机箱回正

v6.8日志曾在6.136s出现pitch=1.102rad、角速度+5.361rad/s，但髋输出仍为+0.434N·m，回正被承重/腿型项抵消。v6.9把髋共同力矩交给独立机箱回正环，使用10ms IMU滤波和比力补偿；差动髋及膝反馈在降维后离散求解。该版编译、9项离线测试通过，但后续整机实验仍失稳，原因继续在v6.10/v6.11定位。以下章节按当时版本记录历史，不能覆盖上方当前状态。

## 2026-09-07 本轮修正：com-capture-v6.10

当时的20:03日志已经是 v6.9（145 列），不能再按旧 v6.8 的髋力矩抵消解释：

- 13.452 s：pitch=0.187、torso_rate=1.676，实际左髋=-5.503 Nm，与命令一致。
- 13.776 s：pitch=0.298、torso_rate=-0.065，箱体旋转已暂时制止；COM 倾角仍约 0.421 rad。
- 13.939 s：机身世界 Y 速度=1.927 m/s，轮命令仍被旧 CATCH 卡在 -1.5 m/s，髋角已到 -0.871 rad。
- 14.100 s：髋角触及 -1.57 rad 限位，随后箱体角速度迅速增大。上述世界 Y 速度是机身原点速度，不等同于 COM 速度。

本轮只改临近触地及 CATCH 轮控，保留 v6.9 髋部回正、腿部反馈、起跳/空中姿态增益、制动/恢复状态机。

1. 增加世界 COM 平移观测：将关节插值到 odom 时间，使用完整机身旋转矩阵得到世界 COM 位置，再差分；不把轮速或机身原点速度当 COM 速度。
2. CATCH 请求轮轴速度 `1.25*vCOM + sqrt(g/h)*(COM_x-axle_x)`。1.25 来自线性倒立摆的临界阻尼捕获/停车推导，不是已验证整机增益；纯 `vCOM+omega*r` 会留下恒定行走速度。
3. 按当前关节轴方向补偿轮相对小腿转动：`cmd=-vAxle-R*(hip_rate+knee_rate-pitch_rate)`。
4. 使用当前 YAML 已有的 5 m/s 轮速上限，取消新 CATCH 路径里的 0.9/1.5 m/s 内层上限；接地命令斜率仍为 8 m/s²。新律也用于临近触地交接，避免带着反向轮速落地。
5. COM/IMU/关节数据超过 80 ms 或无有效几何时退回原 CATCH。未改变 air_wheel_sign。
6. 日志追加四列（共149列）：capture_com_velocity、capture_world_valid、capture_world_active、capture_world_target。启动标签 `[com-capture-v6.10]`。

已执行 ROS 2 Iron 控制器编译。新增测试覆盖世界 COM 观测、前后方向、轮轴/小腿运动补偿和带采样延迟/轮速限幅的线性倒立摆捕获；旧反向入地轮速在更大延迟下仍有不可恢复边界，因此不宣称任何状态均可救回。离线模型没有完整腿部接触耦合，下一步仍由用户运行 Gazebo，核对新捕获路径是否生效、髋角是否继续向限位漂移及停车后能否进入恢复。未启动或覆盖仿真日志。

## 2026-09-07 后续反馈：稳定后再次失衡，com-hold-v6.11

当时的15:20日志为 v6.10（149 列）：
- 9.023 s CATCH → PREPARE，capture_world_active 从 1 变为 0；随后 BRAKE/HOLD 多次往返。
- 9.633 s 进入 RECOVERY，z≈0.348、COM lean≈0、轮速≈0，确实曾达到近静止。
- 10.430 s 站起完成，始终未切入 BALANCE/Position，不能把本次故障归因于 Position 交接。
- 10.887 s 箱体 pitch=-0.023，但 COM lean=-0.059、vCOM=-0.235；11.827 s vCOM=-1.217、cmd=+1.078。恢复轮控仍是旧 HOLD，之后卡在 +1.1，持续后退并倾倒。capture_world_valid 在发散前一直为 1。

修正范围：持续地面轮控的接管连续性。
- 临近触地、CATCH、PREPARE、BRAKE、HOLD、RECOVERY 共用 v6.10 COM 捕获/停车目标，且有效 COM 控制统一沿用 8 m/s² 命令斜率；相位切换不再关闭 COM 轮控。旧轮控仅保留为观测无效时的回退。
- 恢复静稳判据增加 fresh COM、|lean|≤0.03、|lean_rate|≤0.15、|vCOM|≤0.08。阈值沿用/组合现有稳态与捕获阈值，不将箱体水平等同于整机静稳。
- 按 agents.md 的长期 RECOVERY 基线，稳定后继续保持同一 Effort 控制，不自动进入 BALANCE 或 Position。保留调试交接代码，但正常路径由 kHoldRecoveryAfterJump=true 禁用；enable_position_handoff 不再使静稳自动触发交接。
- 起跳、空中姿态、髋膝力矩与支撑参数保持不变。启动标签 [com-hold-v6.11]，稳定保持标签 [COM_HOLD v6.11]；日志仍149列，恢复阶段 capture_world_active 应为1（观测有效时）。

新增30秒离线持续保持检查：含0.8秒站高轨迹、竖直加速度、50/100Hz采样及一帧执行延迟，并在5/12秒加入前后扰动。它是独立倒立摆模型，不含完整轮腿接触动力学；仍须由用户运行Gazebo验证长期站立。

## 2026-09-07 稳态可操作与跳高改善：com-drive-v6.12

用户确认 v6.11 可以稳定站立，但不能行驶/再次跳跃。这是上一轮长期 RECOVERY 的功能缺口：轮控只接受零速度参考，trigger_jump 只接受 BALANCE。不能用旧 BALANCE/Position 控制重新接管来修这个入口。

v6.12：
- 连续静稳0.5秒后锁存 recovery_ready，打印 [READY v6.12]。仍在原 Effort RECOVERY 中运行；W/S前后（沿用0.50m/s），A/D转向（沿用0.60rad/s），空格停车。/cmd_vel 使用同一目标通道；参考按既有1秒速度斜坡限速。
- 地面控制从零速度推广为 vAxle=1.25*vCOM-0.25*vRef+omega*r。vRef=0时与v6.11保持律完全相同；速度指令不绕开平衡控制。
- J可从就绪的RECOVERY再次进入PRE_JUMP；仍需先停车到|vCOM|<0.30且COM姿态安全。准备、下蹲继续用已有髋共同回正/差动/膝阻尼与质心轮控，进入THRUST前不切Position，也不对已激活Effort再注入80/8的位置预充。Effort准备超时退回RECOVERY。每轮重置ready/行驶参考/跳跃统计。
- 目前高度调节Q/E仍是原BALANCE入口；此轮恢复的是行驶、转向、停车与再次跳跃。

跳低日志（v6.11，末尾22.845s）：目标质心离地速度1.98091，THRUST峰值1.44379。8.154s膝实际-0.398485、位置参考-0.0879087、速度-7.97836、速度参考-2.82941；P+D限幅为+22Nm，反向抵消伸腿力矩。之后行程保护卸力，离地确认时速度0.603421。注意这是确认时刻速度，不是精确接触分离时刻速度；汇总里的apex_world_z_delta从下蹲起点算起，包含站高/伸腿，不能当成净腾空高度。

本轮推地仅改变膝位置P：在观测有效、尚未达95%速度、未刹腿/未姿态阻塞/未行程保护时，位置参考不反向拉回已超前伸展的膝。落后于轨迹仍保留伸展P，原Kd=3和速度参考限幅完全保留；保护/卸力条件出现立即恢复双向P。上述8.154s样本的反向PD由22降到15.44685Nm，仍保留全部速度阻尼。力矩、推力增益与行程上限均未提高。实际跳高改善量仍需仿真，不能据此宣称达到0.20m。

日志追加5列（154列）：recovery_ready、recovery_drive_ref、recovery_yaw_ref、effort_jump_cycle、thrust_knee_position_yield。启动 [com-drive-v6.12]。新增离线检查覆盖行驶稳态、前进→后退→停车、准备期变高度、输入斜坡，以及日志膝P/D分解与保护时恢复P。仍未运行完整Gazebo重复跳跃；下一轮需验证第一次跳高、就绪后的行驶停车、第二次PRE_JUMP/SQUAT/THRUST及落地。


## 2026-09-08 跳高仍不足：com-thrust-v6.13

v6.12的单向位置P确实生效，但两次跳跃的速度仍约1.44m/s封顶。5.271s仍接地、COM速度1.43844/1.98091m/s；实际膝速度-10.0014，旧参考-2.93299rad/s，故单独D=+21.2052Nm。叠加前馈-25.835Nm后只剩-4.62957Nm伸腿力矩。11.523s第二跳同样出现D=+21.2966Nm、净膝力矩-8.60112Nm。随后行程保护卸力，确认离地速度分别0.651826、0.844658m/s。这次不是P改动未生效，也不是达到电机150Nm上限。

修正仅针对THRUST膝速度参考：
- 在双轮支撑假设下，`zCOM=R+world_z·(COM(q)-axle(q))`；使用质量加权COM/轮轴Jacobian、当前构型，以及原有期望髋/机箱角速度，反求共同膝速度，使其对应与推力反馈一致的目标COM上升速度。沿用原60ms反馈启动斜坡。
- 参考不依赖实测膝速度，不通过把目标设成实际速度消掉D。Kd仍为3，PD仍限幅±22Nm；超速时仍制动。
- `thrust_knee_velocity_limit`默认30rad/s，来自当前URDF四个腿关节的速度限制，可向下配置；此前名义IK±6rad/s限制只保留在回退路径。30是上限，不是固定指令，实际参考由目标COM速度及几何决定。
- 新参考继续乘原有行程保护和terminal brake系数；行程scale=0时停止伸展。姿态阻塞、COM/IMU/关节状态过期、奇异或反向构型时退回旧参考。
- 保持原髋反作用补偿使用原名义速度，避免本轮隐式调大髋补偿。推力增益、力矩预算/上限、腿行程上限、离地检测、空中/落地/行驶/重复跳跃逻辑均未更换。

对固定历史状态回放，5.271s新参考=-14.1103rad/s、D=-12.3268Nm，若保持该行前馈不变，净膝力矩=-38.1618Nm；11.523s对应新参考=-13.0091、D=-8.96469Nm。回放只能证明参考/力矩方向修正，不能当作新闭环仿真或实际跳高预测。

已执行控制器colcon编译成功，ctest全部11项通过；未启动Gazebo、未覆盖用户原CSV。下一轮由用户确认`[com-thrust-v6.13]`后运行，重点观察是否在行程卸力前达到所需竖直动量，以及更强伸展后的机箱姿态和落地是否仍稳定。


## 2026-09-08 最新后倒反馈：flight-plan-v6.14

本轮读取的新日志是v6.13（159列，到14.644s），不再使用上一轮“尚未仿真”的结论。首轮准备失败后恢复；第二轮12.306s进入THRUST，峰值COM速度1.9802m/s。Git HEAD保存的v6.12日志有3次腾空并恢复，三次均ARREST→PROTECTIVE_DEPLOY，从未实际验证TUCK；不能因空中代码未改就认定新推地后的空中分支已验证。

关键时序：
- 12.581s首次进入TUCK，髋/膝实际为0.988561/-1.27599rad；旧代码固定0.10s收至0.30m高度，绕过已有完整轨迹预算。
- 12.630s髋/膝参考速度=-24.2899/+29.2775rad/s，实际=-6.5684/+8.0372rad/s。12.655s髋实际速度已到-12.3676，机箱角速度=-2.1111rad/s。
- 12.665s进入EXTEND时髋参考=-0.2024、实际=+0.5899，参考已领先0.7923rad；展腿仍接着未兑现的参考运行。
- 12.783s轮距地0.208m，机箱已后仰0.449rad；12.921s首次负间隙时已后仰0.931rad。12.946s进入CATCH时pitch=-1.0177、COM在轮轴后0.242m。轮过前是触地前就形成的相对几何失衡。

v6.14只修收展腿轨迹规划这个主要因素：
- 从与实际收腿入口完全一致的当前关节位置/速度规划，保留0.30m收腿、0.50m落地目标；不通过裁掉关节阻尼或翻转飞轮符号处理后倒。
- 采用原代码已有的髋/膝速度预算7.5/10rad/s、加速度预算240/320rad/s²、位置边界1.45rad。这些是已有规划预算，不是整机动力学安全证明。
- 原0.09s展腿自身也超预算：本次真实IK从(-0.2524,+0.3288)到(+0.1778,-0.3991)，零端速五次轨迹峰值约8.96/15.16rad/s。仅加固定时间门控会禁掉全部完整收腿，因此按五次曲线的峰速/峰加速度计算展腿所需时长，再按200Hz的5ms步长寻找可行收腿时长；名义0.10/0.09秒是最短值。
- 两段总时长必须留出原landing_deploy_ready_margin和20ms的触地裕量，超出当前剩余飞行时间时走原保护展腿。最新失败起点不应再强行执行完整收腿；可能只看到保护收展腿，不能以“每跳都必须收至0.30m”绕过预算。
- TUCK→EXTEND时重新检查当前连续参考及原0.12rad跟踪误差。拒绝时从同一旧轨迹边界进入原保护展腿，保留位置/速度/加速度连续性。
- 推地、落地点偏置、空中/地面反馈增益、力矩/轮速上限、临地轮控混合与控制器切换均未改变。旧控制器未改，现有CSV未覆盖。

仍有独立限制需后续用新日志评估：原临地轮控只在80→20mm混合，本次轮仍带着反向转速入地；URDF实际轮速上限2.1m/s与控制器5m/s请求不一致。这两者发生在明显空中后仰之后，本轮不同时修改，不能宣称仅本补丁已解决所有落地问题。剩余飞行时间仍沿用原机身高度/速度估计；腿部运动会影响该估计，展腿前的二次检查和保护分支继续保留。

## 2026-09-08 空中姿态保护：protective-attitude-v6.15

v6.14的新日志证明完整TUCK已被正确拒绝，但Effort模式`PROTECTIVE_DEPLOY`仍会在空中持续注入后仰角动量。反作用轮最终到达空中轮速命令软件限幅，而机身角速度继续恶化。

v6.15只处理这一条因果链：
- 仅在Effort模式`PROTECTIVE_DEPLOY`中启用姿态保护；首次Position跳跃路径不主动改写。
- 复用已有`attitude_stable`判据，不新增姿态阈值。
- 姿态曾稳定、随后退出稳定区时锁存`[PROTECTIVE_ATT_BRAKE v6.15]`。
- 不再继续高增益追完整landing IK，而是从当前实测`q/qdot`生成连续五次减速段，基本端点为`q_stop = q + 0.5*qdot*T`，终端速度和加速度为0。
- 继续使用已有`flight_segment_admissible()`检查速度、加速度、位置预算。
- 姿态保护触发后，本次飞行不再由原`PROTECTIVE_REPLAN`重新强拉完整落地IK；关节反馈退回较低的空中姿态保护增益。

v6.15的167列Gazebo日志显示：
- 姿态保护在约12.024s触发。
- 首次接触附近约12.181s：`pitch≈-0.1553rad`、`pitch_rate≈-1.5463rad/s`，相比v6.14触地接近`pitch≈-0.85rad`有明显改善。
- 但触地时`com_forward_from_axle≈-0.1946m`，说明轮轴仍在COM前方很远。
- 结论：**v6.15保住了机身姿态，但“姿态保护=停止继续实现落地腿型”留下了严重的落地几何误差。**

## 2026-09-08 最大可行落地进度：protective-landing-progress-v6.16

v6.16在v6.15基础上，不再只停在`q_stop`：
- 先计算完整landing IK及其`landing_com_forward_target`。
- 对每个允许的制动时长，从连续停车端点`stop_end`向完整`landing_end`搜索`alpha∈[0,1]`。
- 每个端点继续经过`enforce_wheel_first_target()`和`flight_segment_admissible()`。
- 用`centroidal_balance_state()`计算规划端点COM位置，优先选择最接近完整落地COM目标的可行端点。
- 新增5列：`landing_com_forward_target`、`landing_com_forward_target_valid`、`protective_landing_progress`、`protective_landing_joint_alpha`、`protective_landing_com_end`。

最新172列日志中：
- 7.361s触发保护，规划`T=0.125s`、`alpha=0.48`、`progress=0.652562`。
- 完整landing IK对应`landing_com_forward_target=+0.040091m`，说明完整目标本身会把COM放到轮轴前方；因此不能再把问题归结为固定`landing_wheel_back_bias_`方向错误。
- 规划端点`protective_landing_com_end=-0.019973m`，即即使按规划端点理想到达，轮轴仍略在COM前。
- 更严重的是，规划COM评价使用固定`landing_pitch_ref`，而真实机身在0.125s执行期间持续后仰；因此该`progress`并不代表实际世界落点进度。

日志暴露出的决定性问题：
- 7.474s：`pitch=-0.222154`、`pitch_rate=-2.317610`，左膝目标`-0.889570`。
- 7.479s：左膝目标突然变成`-1.128920`，单个5ms周期变化约`-0.239rad`；`knee_vel_cmd_left=-2.317610`。
- 当前`enforce_wheel_first_target()`使用`theta_shank = phi2_0 + q_hip + q_knee - pitch`。保护轨迹执行期间为了连续性暂时不逐帧硬夹，结束后再次启用时，当前pitch已与规划时显著不同，于是一次性把积累的世界几何误差补到膝目标。
- 7.484s首次负间隙附近，`com_forward_from_axle=-0.183694m`，轮子仍明显在COM前。

因此v6.16的主要结论不是“landing progress还不够大”，而是：
**保护展腿使用关节相对坐标规划，而落地安全约束本质属于世界坐标。pitch变化使固定`q_hip`轨迹失去原来的世界腿姿态意义。**

## 下一步设计：world-referenced landing leg trajectory v6.17（尚未实现）

v6.17应先修坐标定义，不继续调固定落地偏置。

定义世界参考髋状态：
`eta = q_hip - pitch`

对应速度：
`eta_dot = qd_hip - pitch_rate`

根据当前运动学：
- 大腿绝对角：`theta_thigh = phi1_0 + eta`
- 小腿绝对角：`theta_shank = phi2_0 + eta + q_knee`

因此保护展腿阶段应：
1. 用`eta / eta_dot`与`q_knee / qd_knee`作为规划状态，而不是直接用`q_hip / qd_hip`。
2. 实际输出时恢复：`q_hip_des = eta_des + pitch`、`qd_hip_des = eta_dot_des + pitch_rate`。
3. wheel-first直接在轨迹空间约束`abs(phi2_0 + eta + q_knee) <= landing_shank_limit()`，使约束不再显式依赖瞬时pitch。
4. 不允许在轨迹结束后再通过一次性修改`q_knee`补偿整个pitch变化；必须保证整个保护段内位置、速度至少连续。
5. COM落点评价应使用与实际世界姿态一致的状态，不再用固定`landing_pitch_ref`高估`protective_landing_progress`。
6. v6.17先验证世界腿姿态连续、wheel-first连续和触地前COM位置；通过后再决定是否还需要修改COM-relative landing target或临地捕获。

最新日志中7.474s实际`hip_vel_left=-2.169450rad/s`、`pitch_rate=-2.317610rad/s`，故`eta_dot≈+0.14816rad/s`，说明当时大腿在世界中已经接近静止。当前控制若简单要求`qd_hip_des≈0`，反而等效为要求世界大腿角速度`eta_dot_des≈-pitch_rate`，这正是v6.17需要消除的坐标耦合。

