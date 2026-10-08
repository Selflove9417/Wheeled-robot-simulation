# THRUST 早期差异源码审查（只读）

范围：`src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp` 与 `include/bbot_balance_controller/{jump_phase_control.hpp,torso_pitch_control.hpp}`。未改源码、参数或运行仿真。以下是源码控制流判断，不是物理运行结论。

## SQUAT→THRUST：入口和可残留量

- PRE_JUMP→SQUAT 在 `bbot_velocity_jump_controller.cpp:2518-2529` 锚定 `squat_start_height=clamp(current_z_, L_SQUAT_, L_MAX_)`，以当前高度、零起终速度初始化下蹲五次轨迹；长度为普通跳 `T_SQUAT_`，Effort 二跳为 `1.30*T_SQUAT_`。SQUAT 用 `quintic_traj_.evaluate(now_sec,...)`（2576-2585附近）生成高度/速度，IK 后发布位置腿控或 Effort 高度控。
- SQUAT 末段角参考不是静态初值：`active_jump_pitch_ref_ = rolling_reference_blend(squat_pitch_start_ref_, jump_pitch_ref_, progress)`；俯仰速率参考由最后 35% smoothstep 建到 `jump_takeoff_pitch_rate_`（约 2600-2630）。Effort 二跳若轨迹结束后仍等待，速率参考在 100 ms 内平滑回零。
- SQUAT→THRUST 的条件在 2671-2699：`traj_done && settled && motion_ready` 或 `forced_ready`。`motion_ready` 要求 `|x_dot-jump_forward_speed|≤0.12`、俯仰角误差≤0.08（Effort路径0.03）和角速度误差在允许带内（Effort路径上限 min(tolerance,0.10)）。普通路径轨迹后超时0.35 s，Effort路径1.20 s（2693-2697），但超时本身不是转 THRUST 的条件。
- 转换当帧（2704-2738）将状态设为 THRUST，`state_start_time_=now_sec`、清 `thrust_trajectory_initialized_`、清 seed flag、清空 `thrust_velocity_reference_`，将每腿推力设为半机重，清释放状态与运动时钟、关 gate；角速率参考立即设为起跳目标（约0.45 rad/s 默认参数在声明区）。最后轮速命令有意保留。若不是 Effort 连续路径，先写重力支撑并异步请求 Effort 控制器；因此“进入 THRUST”不等于已经执行推地控制律。
- `run_state_thrust`（2760起）首先有 `!effort_mode_active_` 分支（2764-2787）：重复请求 Effort 切换、腿保持蹲姿，但轮子继续以角度/角速度反馈追踪且朝目标前速加速，然后提前 return。故首次可见 THRUST 状态/轮命令变化，可能早于推力轨迹和膝力矩命令。
- 真正初始化在 Effort 激活之后且 `aligned_takeoff_geometry()` 成功时（2790-2835）。未对齐就保留切换前支撑并等待，超过200 ms abort（2793-2799）。成功时重新锚定 state time、起跳高度，标定地面高度偏置，重置离地/速度观察器，初始化 `quintic_traj_(t0=0, duration=thrust_duration, z0=current_z_, vz0=0, az0=0, zf=H_TAKEOFF_, vf=0.8, af=0)`（2814-2815）。然后用**当前**高度做首次IK seed，写入 `prev_q_*_des_` 并置 `thrust_reference_seed_pending_=true`（2816-2823）。这正是防止跨 SQUAT/THRUST 有限差分形成伪关节速度的保护。
- `sampled_joint_reference_rate` 在 `jump_phase_control.hpp:51-59` 对 seed sample 或非法/过小dt返回0，否则 `(current-previous)/dt` 再限幅。因此首次有效轨迹IK差分不应带入SQUAT末值；但首次膝/髋参考可能仍来自THRUST期的支撑状态，且关节**实测**位置/速度不会被seed重置。
- 每次轨迹初始化把 `thrust_motion_elapsed_=0`、初始 gate 关闭、阻塞态启用、力设回半机重（2823-2834）。启动时的 `thrust_force_per_leg_` 与 SQUAT 已写出的控制器命令不等价：后续推力须经过参考、门控、反馈、力矩预算、斜率限制才下发。实现没有全量重置所有传感器滤波器；IMU observer/filter是持续更新的进程状态，不能假设 THRUST 首帧“重新起滤”。

## THRUST 初始参考、时钟和反馈

- 初始化后每帧用 `thrust_pitch_reference(jump_pitch_ref_, jump_takeoff_pitch_rate_, lead_time, thrust_motion_elapsed_)` 生成角/角速率（2838-2849）。公式见 `jump_phase_control.hpp:31-39`：`u=clamp(elapsed/lead,0,1)`，`rate=rate0*(1-3u²+2u³)`，`angle=initial+rate0*lead*(u-u³+0.5u⁴)`。默认 lead time 0.07 s（参数声明约265-268，clamp到0.12s）。gate仍在首次打开前时角速率参考仍为曲线的rate（只有“gate已打开且后续又硬阻塞”才强制速率目标0，2844-2845）。
- 初始姿态 gate 要 pitch误差≤0.08 rad、速率误差≤起跳速率容差，并连续稳定20 ms（2852-2862）；THRUST初始 gate 最多等0.20 s，否则恢复（约2878-2900）。运动时钟只在 gate 已打开且当前无阻塞时增加 dt（约2930-2950）；gate 等待期间，真实 state elapsed 前进，但 `thrust_motion_elapsed_`、伸腿轨迹和速度反馈 ramp 可保持在0。因此不能只用 THRUST state elapsed 推断推力脉冲已开始。
- 竖直闭环（约2960-3025）：质量加权 COM vz有效时，`velocity_error=target_takeoff_velocity-vz`；`feedback_force=mass_per_leg*thrust_velocity_kp*error`，并乘前60 ms ramp；附加 Bezier 脉冲 `base_force*(peak_ratio−1)*shape`。姿态惩罚、gate/软后仰、离地速度释放、膝行程和力矩预算会继续改变请求。最终每腿力矩预算后斜率受 `1600*dt` 限制（3300附近）。所以“第一次THRUST输入”需区分 gate、motion clock、无限制请求力、预算后请求力、滤波/斜率后的力、髋/膝最终 effort 命令。
- 名义腿参考轨迹在运动时间上 evaluate（约3110-3120），首个IK参考为 `des_z` 经限幅得到的高度。推力期间可在新鲜 COM/IMU/关节几何数据上用 `thrust_velocity_reference(...)` 反解膝目标速度（约3170-3202）；有效条件要求 COM速度有效、关节样本≤80ms、IMU observer fresh。轨迹末段若仍接地会按当前构型保持并继续速度反馈（约3120-3130），并非持续伸至名义 H_TAKEOFF。

## IMU raw / legacy filtered / fast observer

- `/imu` 回调在1728-1758：姿态 `pitch_=-roll`；原始角速率 `pitch_rate_raw_=-angular_velocity.x`。同一raw角速率及 IMU 加速度y/z写给 `torso_imu_.update(header_stamp, raw_rate, ay, az)`；之后 raw rate 进入 legacy `pitch_rate_filt_`，首样本直接赋值，后续固定alpha=0.15 的低通，`pitch_rate_` 是该滤波结果。alpha在成员默认值771行；按 100Hz 使用时它相当于每个采样点约0.15权重，但实际时间常数取决于真实消息率，源码未按消息dt调整。
- `TorsoImuObserver` 定义在 `torso_pitch_control.hpp:12-35`：独立按消息timestamp更新；正常dt时 `alpha=1-exp(-dt/0.010)`，即10 ms连续时间常数；首次、非正dt或间隔>80ms时直接接收raw rate/加速度并把period重置10 ms。`fresh(now)` 容许样本年龄≤80ms。observer没有在阶段切换时reset，故THRUST首帧数据可包含最近IMU样本历史，但按header stamp可计算年龄。
- 常规THRUST姿态门控、主髋D反馈、wheel attitude项仍用legacy `pitch_rate_`（如2846-2849、3188-3200、约3440后）；fast observer只在满足开关、Effort确认、IMU新鲜和特定phase blend时进入附加的rate correction（约3290-3355）。代码提供raw、legacy filter、observer三条不同信号，日志若只记录`pitch_rate`会掩盖它们之间的时延/差值。
- gate范围的fast补偿默认开关参数可由 `thrust_fast_rate_correction_from_gate_` 控制（参数约232-235）；门控后20ms smoothstep blend见 `jump_phase_control.hpp:382-392`，状态资格要求 THRUST、Effort已激活、无切换pending、gate打开、fast IMU fresh、motion elapsed≥0。路径将同一 active rate reference 从两种误差同时扣除，再由`thrust_release_fast_rate_correction`计算：`0.5*kd*(legacy_rate_error-fast_rate_error)`，最大8 Nm（helper约395-413；调用约3335-3355）。如果默认 gate-scope=true，旧 release-only 条件分支被选择性替代；fast不是整条姿态D反馈的替代传感器。
- 原生/legacy rate与fast rate的时延差异来自两个滤波器的实现差别：legacy固定每消息0.15权重且依赖真实IMU发布频率；fast是10 ms时间常数并跟随header时间戳。对于角速度突变，legacy一般较慢（约100Hz时有效时间常数约62ms），fast约10ms；消息率偏离100Hz、header延迟/抖动或IMU年龄接近80ms会改变实际关系。不能仅以fast滤波更快推断端到端反馈一定更快：它仍经过 ROS订阅、5ms wall timer控制循环（timer创建约729行）、Effort状态/切换/gate eligibility、20ms blend和最终扭矩限幅。`pitch_rate_raw_` 对比项可揭示sensor/filter滞后，`torso_imu_.stamp()` 与控制时刻可揭示观测年龄，但源码日志是否对同一仿真时钟完成配对仍需检查数据管线。

## 用首次输入延迟拆分故障位置

若日志能同时提供下列时刻/值，延迟可分解为：

1. **SQUAT→THRUST**：状态转换日志/状态列时间；若异常，问题在 readiness 或SQUAT参考/样本。
2. **控制器切换完成**：`effort_mode_active`、`leg_mode_switch_pending`；若状态已THRUST但它滞后，则是异步切换等待，期间只运行轮速等待分支。
3. **几何对齐和轨迹初始化**：`thrust_trajectory_initialized`（若现有日志没有此列，用 state_start_time 跳变、轨迹seed/开始时间及对齐有效诊断代替）；若等待，则是odom+关节样本对齐/新鲜度，不是推力算法。
4. **gate打开 / `thrust_motion_elapsed`首次增加**：若初始化完但没有推进，检查pitch/legacy rate参考误差和20ms稳定计时；初始gate的阻塞诊断、abort计时也是独立门。
5. **请求与输出**：比较无限制F请求、force limit、`thrust_force_per_leg_`及最终膝/髋effort命令。F请求出现但力矩预算或斜率结果迟滞，说明限幅/命令平滑链；F尚无则检查gate、COM vz有效性、速度误差/phase shape。
6. **测量回馈**：同步记录 IMU header stamp、raw rate、legacy filtered rate、fast rate及fast age；再与state/motion elapsed、命令时刻配对。raw及时而legacy滞后、fast较近raw，支持“legacy滤波延迟”；两路都晚于物理事件，可能是采样/传输/记录配对；fast新鲜但corrected torque迟滞，则查eligibility/blend/effort发布。仅有首次电机输入/输出时间差，无法区分传感器滞后、state gate、异步切换和力矩预算。

## 证据限制

源码证明了状态机和参考生成顺序，但不能证明实际插件何时消费了命令、IMU的真实发布率/端到端延迟、Effort切换响应时长、backend servo/力矩效果或物理推地起始。需要同一时基的运行日志（`header.stamp`、`this->now()`、joint sample stamp、controller switch状态、各阶段参考/命令/输出）作区分；本审查未运行仿真，也未读取新的THRUST物理样本。日志里`state_start_time_`会在几何对齐成功时再次重置（2801），所以应单独区分“state进入时刻”与“轨迹初始化时刻”，否则计算出来的elapsed会隐藏等待时间。
