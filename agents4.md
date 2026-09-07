# agents4.md — 跳跃控制调试交接（2026-09-06）

## 当前问题

- 用户最新反馈：落地后机箱不能保持水平，向前倾倒，随后整机失去平衡；问题仍未解决。
- 此前下蹲、伸腿正常，空中姿态曾基本稳定；用户确认跳跃高度仍不足。
- 当前优先解决落地机箱姿态与整机平衡，不将重心捕获成功等同于机箱回正。

## 当前代码

- 主文件：`/home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`。
- 最新标识：`torso-ground-v6.9`；已编译，9 项离线测试通过，尚未运行新仿真。v6.8 仿真仍落地前倾失控。
- 起跳采用整机 COM 世界竖直速度反馈；落地轮控采用 COM 相对轮轴的倾角及变化率；髋部仍负责机箱姿态。
- `gazebo_world_z / gazebo_world_z_dot` 是机身原点状态；`com_world_z / com_world_vz` 才是整机质心状态。
- 最新日志在 6.136 s 时箱体前倾 1.102 rad、角速度 +5.361 rad/s，髋输出仍为 +0.434 N·m：机箱回正被承重/腿型项及耦合求解抵消。
- v6.9 将两个髋的共同力矩交给独立机箱回正环；左右髋差动和膝关节保留原 PD，在降维后离散求解，避免腿型目标经惯量耦合干扰回正。
- 回正环按箱体惯量独立离散化，使用专用 10 ms IMU 滤波及箱体质心比力补偿接触力矩；IMU 过期退回静态重力补偿。覆盖触地、恢复及落地后 Effort 支撑阶段。
- 起跳与空中控制保持本轮前参数。此前有符号推力预算和腿/轮重力补偿仍保留。
- 用户允许临时放宽力矩：Gazebo 启动默认仅 THRUST 使用模型已有的 150 N·m 上限，其余阶段仍为髋75/膝60；`sim_relax_thrust_limits:=false` 可恢复原上限。非仿真时钟下不放宽。

## 下一步排查

1. 用户运行仿真后读取 `src/bbot_balance_controller/src/data_logs/jump_velocity_control_log.csv`；v6.9 为 145 列，重点分析触地前后约 0.2 s。
2. 对照 `pitch / torso_rate`、`torso_force_ff / torso_feedback / torso_hip_command` 及实际左右髋输出，验证回正方向、IMU 新鲜度与机箱响应；原 `tau_ff_hip` 不再代表最终髋共同力矩。
3. 再检查 `com_lean / com_lean_rate` 与轮速，区分机箱回正和整机平衡。跳跃高度不足仍待验证，不能用增加轮控增益或力矩上限代替定位原因。

## 验证范围

- 本轮新增力矩分配、腿部阻尼、IMU 时序检查，以及 90 组含接触脉冲、采样延迟和惯量误差的独立箱体模型检查；全部通过。
- 上述模型不是 Gazebo 整机仿真，不能据此宣称落地问题已解决。

## 约束

- 保留机箱回正阻尼、左右髋差动及膝关节阻尼、空中轨迹位置/速度连续性；不盲目翻转已验证的 `air_wheel_sign=+1`。
- 不修改旧跳跃控制器，不为绕过故障强行切回 Position/BALANCE；一次聚焦一个主要原因。
- 改完代码并完成必要编译/离线检查后停止，由用户运行仿真。编译通过不代表跳跃或落地成功。

## 2026-09-07 本轮修正：com-capture-v6.10

最新 20:03 日志已经是 v6.9（145 列），不能再按旧 v6.8 的髋力矩抵消解释：

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

最新 15:20 日志为 v6.10（149 列）：
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
