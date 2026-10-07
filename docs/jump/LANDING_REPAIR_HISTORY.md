# 用户实际入口复现与判断纠正

2026-10-05 已按 README 的 `controller_type:=jump_velocity jump_height:=0.25` 实际运行一次正常速度、带 GUI 的完整跳跃。**该 velocity 版本左右腿同步，也能减速；前述 −76/+73 Nm 对打属于独立分配器诊断，不能归因到用户正在运行的版本。**

本次完成正常跳跃并恢复，继续观察 12 s；首触附近后倾约 12°，落地后明显后退，修复目标仍未满足。COM 上升约 25.66 cm；CAD 对齐重建轮底净空约 18.18 cm、轮轴后退约 81.60 cm，均为采样几何估计而非同物理步原生轮位姿验收。默认程序、入口、世界与 MPC 保留。

- 本次真实运行、版本纠正及量测限制（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/readme_velocity_probe_20261005_223120/REVIEW.md`）
- 左右腿同步与减速曲线（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/readme_velocity_probe_20261005_223120/plots/leg_synchrony_and_braking.png`）

以下完整保留历史实验记录；各条异常应结合其入口和控制器解释。

# 落地修复独立实验

2026-10-05 本轮新增实际发布时刻与同物理步原生状态通道，发现并离线修正支撑分配器的左右髋反向力矩分配缺陷。**跳跃仍未修复；本轮 1 次正常速度诊断、0 次新收腿候选、0 次成功，未晋级默认/MPC。**

实际诊断未进入 FLIGHT，双轮轮底最高几何间隙约 0.98 cm，推地期间腿长反向缩短而失败。实际六输入的九维前向响应按原门槛 FAIL，因此新轨迹及新约束没有接入运行控制。腿命令实际生效延迟核对为 4–6 ms；有界 P 轮伺服的 5 ms 采样保持和状态龄仍须纳入预测。

固定真实状态回放中，加入髋相对加速度与平地对称法向载荷硬约束后，两髋由 −76/+73 Nm 变为约 −1.57/−1.57 Nm；这是离线结果，尚未实际运行验证。自适应刹车选项默认关闭，条件模型的 21.46 cm 净空也不是实际跳高。

- 本轮改动、真实结果、刹车解释和下一步条件（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_capture_plan_20261005_204118/REVIEW.md`）
- 髋膝控制命令、仿真输入、关节合负载曲线（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_capture_plan_20261005_204118/plots/actual_timing_probe_torques.png`）
- 诊断入口 `run_capture_command_probe.sh`；默认 `run_complete_jump_demo.sh`、DEFAULT_FLAT_JUMP.md、MPC、原有未提交工作与历史数据保留。

以下完整保留此前记录，各轮“最新”仅对应其历史状态。

## 此前记录（保留）

# 落地修复独立实验

2026-10-05 本轮已让髋参与推地机身姿态控制：支撑分配器新增 pitch 加速度任务，独立正常速度候选已实际离地；**跳跃仍未修复，未晋级默认/MPC。** 本轮实际候选 1 次、成功 0 次。

实际六命令的 200 个双支撑物理步响应核对按原门槛通过。双轮轮底峰值净空 14.74 cm（目标至少 20 cm），首触前倾 6.66°、角速度 −0.878 rad/s，轮轴最大后退 60.29 cm，物理验收 FAIL。COM 上升 23.31 cm 不是轮底净空。

推地前倾保持约 5.24–6.26°，但离地关节速度仍高：髋 +7.16、膝 −12.84 rad/s，H=+0.2584，未达到目标 −0.08。真实 q/v/H 初始化的 60 ms 收腿曲线因速度/加速度越界被拒绝，未执行新 TUCK/EXTEND。固定收腿目标的理想最短时间下界至少 132 ms，不能靠延长动作掩盖当前末端状态不可收腿的问题。下一步联合规划正常速度推地竖直冲量及离地 q/v/H，并纳入实际输入生效时序。

- 本轮改动、真实物理结果和剩余问题（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_pitch_allocator_20261005_195559/REVIEW.md`）
- 髋膝控制命令、仿真输入与关节合负载曲线（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_pitch_allocator_20261005_195559/plots/normal_speed_candidate_torques.png`）
- 独立入口 `run_allocator_jump_candidate.sh`，构建 `build_native_allocator` / `build_native_command_observer`。默认完整跳跃版本、世界、MPC、既有未提交改动和历史数据保留。

以下保留此前记录，其中“最新”仅对应各历史轮次。

## 此前记录（保留）

# 落地修复独立实验

2026-10-05 本轮完成只读物理步 `JointForceCmd` 与时序观测，真实输入的九维模型响应核对通过后，已将有界 P 轮模式分配器接入独立实验控制器。**跳跃仍未修复，默认完整跳跃版本与 MPC 未晋级、未改动。**

本轮实际运行一次输入诊断（双轮底净空 10.23 cm、首触后倾 8.86°、轮轴后退 1.885 m，FAIL）和唯一一次分配器候选（推地姿态保护中止，没有进入 FLIGHT，FAIL）。成功次数为 0，不以离线计算次数代替跳跃成功。

已确认并修正：接触纳秒相同却因浮点比较被误拒绝；保护分支零轮速目标导致约 7.2 Nm 制动和打滑。当前代码改用整数纳秒接触历史及明确零轮力矩保护，保持接触/新鲜度/实际限制。真实离地 q/v/H 飞行规划器已加入正常 60 ms 收腿、屈腿首触与真实轮底净空检查，但候选未腾空，飞行轨迹没有实际验证。候选之后的修正仅构建/定向测试通过，未再跑第二组或放宽验收。

- 本轮输入响应、实际候选、后续修正与剩余问题（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/native_command_response_20261005_183421/REVIEW.md`）
- 候选髋膝命令、仿真输入、关节合负载三类曲线（本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/native_command_response_20261005_183421/plots/allocator_candidate_torques.png`）
- 实验入口：`run_allocator_jump_candidate.sh`；独立构建：`build_native_allocator` / `build_native_command_observer`。

下面保留上轮记录，便于继续工作；“尚未接入”等描述仅对应上轮状态。

## 上轮记录（保留）

# 落地修复独立实验

默认入口仍为 run_complete_jump_demo.sh（本地资料：`/home/xy/bbot_ws_new/run_complete_jump_demo.sh`） 的完整跳跃保留版。当前落地后退问题尚未修复；新候选仅保存在独立实验控制器，未晋级默认。

2026-10-05 最新一轮已完成离线支撑任务/力矩分配器，连接实际关节边界与有界 P 轮速力矩伺服，并通过 156 个旧状态 × 三种轮模式的 468 次独立数值核对。分配器尚未接入运行控制器，本轮没有新增仿真。旧命令的条件前向响应仍有较大加速度偏差，下一步先只读记录物理步中的关节力命令并核对生效时序，再接入实验控制。Luna 两次启动返回模型容量已满，本轮由 root 完成。数值测试通过不代表落地修复；上一轮空中候选仍因关节越界、后倾且尚未首触而淘汰。

最近一次实际仿真仍是上一轮：加入推地髋膝联合速度参考与每侧 ±4 N·m 的有界髋反馈，重心前向目标为相对轮轴 +10 mm。`--thrust-support-coordination` 开关默认关闭，与旧 momentum/handoff/pulse 实验互斥。主控制器、旧 reference、模型、物理限制、动作时长和空中/落地策略保留。

只完成一次正常速度单跳：双轮净空 10.94 cm，首触后倾 5.83°、俯仰角速度 −0.433 rad/s，最大轮轴回退 78.40 cm，物理验收 **FAIL**。构建/针对性测试通过不代表跳跃合格。没有追加组。

最近一次实测确定了两个问题：推地髋膝参考未被实际速度充分跟踪，重心没有到达前向目标；整机 COM 已上升约 19.54 cm，但空中 COM 到轮轴竖直距离增加约 8.51 cm，抵消了轮净空。后续先完成并离线验证支撑阶段竖直/姿态/相对重心任务的力矩分配，再以真实离地 q/v/H 重做可执行的收腿与屈腿首触轨迹。

- 最新分配器与独立核对复盘（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_allocator_20261005_170509/REVIEW.md`）
- 上一轮离线模型与可行性复盘（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_task_budget_20261005_093838/REVIEW.md`）
- 最近一次实测复盘与复现配置（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_impulse_control_20261004_235754/REVIEW.md`）
- 髋膝命令与原生关节合负载图（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_impulse_control_20261004_235754/native_review/native_joint_load_review.png`）
- 重心与髋速度跟踪图（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_impulse_control_20261004_235754/native_review/support_tracking_review.png`）
- Luna 的作用范围与数据配对审核（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/support_impulse_control_20261004_235754/candidate_audit/REPORT.md`）
- 上轮原生记录复盘（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/flight_pitch_momentum_20261004_214740/REVIEW.md`）

图中明确区分控制命令与 Gazebo `JointTransmittedWrench` 关节合负载。合负载包含驱动、阻尼、摩擦和约束，不能当作实机电机扭矩；有接触但未提供 contact wrench 的帧也不能解释为零接触力。完整数据、测量限制与本轮校验均见最新复盘。
