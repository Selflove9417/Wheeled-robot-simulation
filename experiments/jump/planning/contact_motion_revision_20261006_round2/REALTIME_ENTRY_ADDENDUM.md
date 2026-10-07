# Round-2 实时入口与故障恢复补充审查

本补充记录新加的连续性保护和尚未解决的运行时入口风险；原始 [`REALTIME_ACCESS_REVIEW.md`](REALTIME_ACCESS_REVIEW.md) 保持不变。本次只读控制路径并新增本报告，没有启动 ROS/Gazebo，也没有修改 controller 或既有记录。

## 已实现的两项 guard

`GroundEngineStateHistory::push()` 现在允许第一条完整 ECS1 样本使用任意 iteration；后续每条必须正好 iteration +1、sim stamp +1 ms，且 dt=1 ms。重复、回退或缺步都会拒收并清空历史。controller 订阅使用 reliable QoS，与桥的默认 reliable publisher 匹配。`ground_engine_state.hpp` 的离线单测覆盖缺帧、stamp 不匹配、重复清空及重新建立历史；root 已确认 controller 独立构建成功，CTest `ground_engine_state` 1/1 通过，未进行运行时故障注入。

新增的 `GroundContactContinuityGuard` 已由 `record_reference_ground_pulse_contact_frame()` 对每个物理速率 contact callback 调用。开始 Effort 接管后，坏帧、非双轮接触、step gap/rollback 触发 sticky failure；后续接触恢复不能清除此故障。测试覆盖三帧“有效→丢失→恢复”，并验证恢复后仍失败；等待接管期间的非双支撑只重置基线，不预先锁死实验。失败沿用已有 failed-stage 输出锁，不绕开 mode-pending 的腿输出保护，也没有接入新的运动 law。

这些是状态源解析/回调路径和纯 C++ guard 的构建、fixture 证据，不是 DDS 丢包、controller manager 状态变化或硬件输出效果的 ROS 运行证据。

## Controller mode ownership 现状

controller 内没有 `ListControllers` 客户端或运行中的 controller-state 订阅。`effort_mode_active_` 在 Position→Effort `SwitchController` 成功回调后设置，在本地反向切换回调后清零；这是对一次服务响应的本地记忆，不会发现之后 manager 外部停用 `leg_effort_controller`、启用 `leg_position_controller`，或状态报告失效。参见 [`bbot_landing_repair_controller.cpp`](../../../../src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:835) 的 ECS1 callback、[entry stage](../../../../src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:2864) 和 [切换服务回调](../../../../src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:8880)。

独立 `run_ground_input_trial.py` 在启动和进入固定 hold 前调用 `ros2 control list_controllers`，并检查 wheel-effort active、diff-drive inactive、leg-effort/position 的预期状态；5 秒 hold 与运动循环仅读 controller log 的本地 `effort_mode_active` / `leg_mode_switch_pending`，没有持续查询 manager。因此它证明两个静态采样点的状态，不证明整段运行期间 controller 独占。

建议的最小实时门是在 controller 内增加非阻塞异步 `ListControllers` monitor（仅 contact-motion opt-in）。每次成功响应保存 steady receipt；要求响应新鲜度有硬上限，并按阶段校验：等待/预充时 leg Position active、leg Effort inactive；切换 pending 时停止任何腿输出；接管后 leg Effort active、leg Position inactive、wheel-effort active、diff-drive inactive。缺服务、超时、未知 controller、错误 active 状态或接口 claim 冲突都 sticky-fail。检查 controller 状态和 `claimed_interfaces`；必要时将 `/diff_drive_controller/cmd_vel` 意外 publisher 作为附加诊断，而不是把 publisher 数量当作消费证据。runner 应独立低频重复采样 manager 状态并在失配时请求暂停，形成第二道监视；该暂停用于阻止继续运动，不能计作安全卸力恢复。

controller manager 状态只能证明接口所有权，不能单独证明这条命令被 Gazebo Physics 消费。仍应保留 direct Physics 的六路 `JointForceCmd`、ECS1 key/gap、wheel servo 实际 torque 和命令/API bounds；输出审计要将 manager active 状态与实际 Physics before-input 同时列出。

## 故障输出与恢复缺口

当前 `fail_ground_motion()` 将阶段锁为 failed；下一控制循环进入 `run_state_balance()` failed-stage 分支，调用 `lock_owned_outputs()`。该函数请求轮输入归零；在本地 `effort_mode_active_` 为真且没有切换 pending 时，会反复重发最后一组有限、按髋 75 Nm / 膝 60 Nm 限幅的腿力矩。没有时间上限、减载轨迹或自动交接，所以这是 bounded last-command hold，不是卸力恢复，也无法保证外部已停 Effort controller 时命令被接收。

仓库已有 `STATE_RECOVERY` 的 `RECOVERY_FAIL_CROUCH` / `RECOVERY_FAIL_STABILIZE` 与 Effort→Position handoff；但当前 contact-motion failure 不会转入该状态。`trigger_handoff_fallback()` 只由 jump recovery 中 Position handoff 超时/拒绝调用，并假定那套 jump recovery 子阶段、轨迹和状态已建立。不能仅把 `ground_motion_failed_` 接到它或调用 `request_position_controller()`，就声称复用了安全恢复：接触失效、direct q/v 失效或 mode ownership 不明时，可能恰好缺少生成姿态预充命令所需的可靠状态。

下一轮最小实现应显式区分可恢复与不可恢复故障：

1. 当 controller manager 新鲜且确认唯一 Effort owner，独立关节/机身源新鲜，双轮接触有效，direct engine q/v/u 有效且限制内时，冻结错误原因并切入**有限时长**、有明确终止条件的协调恢复子阶段。该阶段须使用经离线验证的状态/支撑路径，保留必要支撑，并限制总时间、关节位移/速度、pitch、输入幅度；成功条件与 Position handoff 也须有状态连续和 manager readback。不能无限重复最后 torque，也不能直接把腿输出置零称为制动。
2. 如果 manager 状态不新鲜/失配、支撑丢失，或可用状态/实际输入无效，就不再对未知 actuator owner 发布假定有效的恢复命令；由独立 runner/监督进程请求暂停 world 并以明确 fault 终止。暂停是超出控制器可证明能力时的 fail-stop 兜底，不等于用户要求的卸力恢复 PASS。下一次物理准入前必须有这两个分支的可审查输出语义。

尚未实现上述 mode monitor 或协调卸力状态，也没有 manager fault、ECS/contact 丢帧、solver timeout 或停止包络越界的 ROS fault injection。正常 stop 的 0.25 s quiet dwell / 2 s timeout 已在控制器路径中；审计中的 excursion/rebound/axle 包络目前仍是事后离线判据，不是对应的实时超限 abort。helper 当前仍受 pending admission 拦截，本轮没有资格做接地动作。

## 最小复用测试与准入建议

- **纯 guard 测试**：manager 样本状态机验证各阶段允许组合；拒绝 manager stale/timeout、leg Position 与 Effort 同时 active、Effort inactive、diff-drive 与 wheel-effort 同时 active、缺 controller / claim 冲突，并保证 sticky fault 不被恢复响应清除。
- **恢复 reducer 测试**：新鲜 manager + fresh state + bilateral + actual input 有效时进入有限恢复、输出每步保持在既有限制内并按时结束；同样故障但缺任一 prerequisite 时只产生 pause/abort 请求，不能输出零腿力矩或持续旧 torque。测试需要同时检查“最后输出持有最长时长”和恢复分支终止条件。
- **ROS 集成 fault injection（独立短测试，先在暂停/无运动状态）**：逐项模拟 manager 服务超时、Effort deactivate、Position 意外 activate、diff-drive activate、contact 1 ms drop 后恢复、ECS1 gap。检查故障锁存、恢复/暂停分支、输出 owner 和事件时间戳；在该测试通过前不消耗低速运动 campaign。
- **唯一物理 campaign 前**：确认独立 build binary/source SHA、runtime manager freshness、初始及连续独占、native bridge QoS 与 10 ms receipt TTL、contact stream continuity、stop excursion 实时保护、bounded recovery prerequisite/readback 全部具备。runner 的暂停兜底及 controller 侧 mode monitor 必须分别有证据。

源检查、unit/CTest、启动时两个 controller-manager 查询、实际 ROS 故障注入和物理 campaign 是不同证据层；当前只满足前两类以及新 runner 在指定时刻的模式快照。没有实时故障安全恢复或低速 motion qualification 结论。
