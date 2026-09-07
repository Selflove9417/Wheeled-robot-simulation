# agents4.md — BBot 跳跃控制调试交接（更新：2026-09-07）

## 当前结论与待办

- 用户已确认 **v6.11 落地后可以稳住**。不要再把“落地后必然前倾失控”作为当前基线。
- 用户随后反馈：跳得低；稳定后不能前进、后退或再次跳跃。
- 当前代码为 **v6.12**，已修正稳态操作入口，并减少正常推地时膝位置P反向抵消推力的问题。
- **v6.12 已编译、10项离线测试全部通过；尚无用户的新仿真验证。** 不能宣称操作、重复跳跃或目标0.20m跳高已在整机上验证成功。
- 当前优先验证：保留原有稳定性 → 就绪后的行驶/停车 → 第二次跳跃 → 实际跳高变化。不要只凭机箱水平判断整机质心稳定。

## 当前代码与控制架构

- 主文件：`/home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`。
- 核心辅助文件在 `src/bbot_balance_controller/include/bbot_balance_controller/`：`centroidal_state.hpp`、`jump_phase_control.hpp`、`torso_pitch_control.hpp`、`ground_joint_pd.hpp`。
- 启动标识：`[com-drive-v6.12]`；静稳后操作开放标识：`[READY v6.12]`；保持标识：`[COM_HOLD v6.12]`。
- 环境为 ROS 2 Iron；主控制200Hz，controller_manager为100Hz。
- 首次起跳前可使用Position；跳后保持Effort RECOVERY。`kHoldRecoveryAfterJump=true`，不会因静稳自动进入旧BALANCE或Position；`enable_position_handoff`不会绕过这个保持分支。
- 保持RECOVERY不再等于锁死操作：静稳0.5秒后锁存`recovery_ready_`，在同一质心轮控中接受用户速度目标。
- 临近触地、捕获、制动、保持、恢复共用质心轮控；重复跳跃的PRE_JUMP/SQUAT也保持Effort与质心轮控，准备超时退回RECOVERY。
- 髋共同力矩继续负责独立机箱回正，保留左右髋差动和膝关节阻尼。首次起跳/空中姿态参数未随本轮操作入口修正而更换。
- `gazebo_world_z / gazebo_world_z_dot`是机身原点状态，`com_world_z / com_world_vz`是整机质心状态。
- 现有仿真选项只在THRUST放宽到模型已有150N·m，其余阶段髋75/膝60；`sim_relax_thrust_limits:=false`可恢复原推地上限，非仿真时钟不放宽。v6.12未再提高这些上限。

## 用户操作与边界

- 出现`[READY v6.12]`后：W/S前后、A/D转向、空格停车；`/cmd_vel`使用同一目标通道。
- 行驶/转向参考上限沿用0.50m/s、0.60rad/s，参考变化率为1.0/s；地面质心轮命令仍受5m/s上限与8m/s²变化率约束。
- 再次跳跃：先停车，再按J。RECOVERY入口要求已就绪、Effort激活、质心观测有效，并检查速度/姿态；|vCOM|必须小于0.30m/s，COM倾角小于0.06rad、变化率小于0.35rad/s，同时保留机箱姿态检查。
- Q/E高度调节仍只接原BALANCE入口，此轮未扩展到RECOVERY。
- vRef=0时仍是v6.11的停车保持律；不能为了恢复按键而改回已知存在支撑/轮控接管问题的旧路径。

## 最新日志与下一轮检查

- 最近已分析的日志属于v6.11，149列，末尾时间22.845s；不要把它误当作v6.12验证结果。
- v6.12运行后应为154列，新增：`recovery_ready`、`recovery_drive_ref`、`recovery_yaw_ref`、`effort_jump_cycle`、`thrust_knee_position_yield`。
- 日志路径：`src/bbot_balance_controller/src/data_logs/jump_velocity_control_log.csv`及`jump_velocity_summary.csv`。
- 先核对启动标识及列数，再检查：
  1. 就绪后W/S是否改变`recovery_drive_ref`，空格是否回零；`capture_world_active`在观测有效的地面阶段应保持1。
  2. 行驶/停车时的`com_lean / com_lean_rate / capture_com_velocity`及机箱姿态是否持续收敛。
  3. 第二次J是否进入PRE_JUMP→SQUAT→THRUST，`effort_jump_cycle=1`，是否保持Effort且不出现预充力矩跳变。
  4. 推地`com_world_vz`、`thrust_knee_pd_left`、`thrust_knee_position_yield`与行程保护；比较离地前冲量和实际腾空高度。
- 离地确认速度不是精确接触分离时刻速度；`apex_world_z_delta`包含从下蹲开始的伸腿/站高，不等同净腾空高度。

## 已完成验证与约束

- 已执行`colcon build --packages-select bbot_balance_controller --symlink-install`，成功。
- 已执行`ctest --test-dir build/bbot_balance_controller --output-on-failure`，10/10通过；`git diff --check`通过。
- 离线检查包含30秒保持、受扰恢复、前进→后退→停车、变高度、传感器延迟及膝P/D样本回归；不等同Gazebo完整轮腿接触与重复跳跃验证。
- 保留机箱回正阻尼、髋差动/膝阻尼和空中轨迹连续性，不盲目翻转`air_wheel_sign=+1`。
- 不修改旧跳跃控制器；基于新日志区分原因，避免同时盲调多个控制环。不要用提高力矩上限代替定位跳低原因。
- 完成代码、编译及必要离线检查后，由用户运行仿真；不要覆盖已有实验日志。

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
