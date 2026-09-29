# 平地跳跃：Gazebo 原生接触与落地 CATCH 单变量试验（2026-09-29）

范围：无载荷、默认平地滚动跳跃。不修改自适应控制参数；不清理已有进程或数据。

## 原生接触消息核对

在默认试验 `native_contact_probe_20260929/trial_01_20260929_180706` 的仿真运行中，直接执行：

```bash
ign topic -e -n 1 -t /world/flat_jump_world/model/bbot/link/link_004/sensor/left_wheel_contact/contact
```

Gazebo 原生 `ignition.msgs.Contacts` 消息的时间戳为 **7.842 s**，含 `bbot::link_004::link_004_collision_collision` 与 `ground_plane::link::collision`、两个 `position`（地面 z 约 −1.48e−6 m），**不含 `wrench`、`normal`、`depth` 字段**。这是桥接之前的消息，足以判定本次接触力缺失已发生在仿真原生输出层；无需继续排查 ROS 桥接是否丢了该字段。配套 ROS 独立记录器 54,968 行全部 `num_wrenches=0`、`force_status=WRENCH_MISSING`，和原生截面一致，但不能反过来把占位 `0.000 N` 当成测得的零力。

限制：原生只抽取一次左轮样本；它判定此消息的字段缺失，而不代表穷尽所有时刻或右轮。因此现有接触链路仅用于碰撞消息有无，不能计算轮地冲量，不能用力阈值确定真实脱地时刻。本轮到此停止环境力通道深挖。

## 基线与单变量控制试验

基线 `native_contact_probe_20260929`：FLIGHT 首帧 pitch `+0.071 rad`、gyro `+0.412 rad/s`；TOUCHDOWN_BUFFER 首帧 `−0.389 rad`、`−1.365 rad/s`、COM lean `−0.640 rad`。触地后 0.10 s，世界系轮速目标 3.695 m/s，实际命令 1.071 m/s，受 CATCH 的 8 m/s² 命令变化率限制。随后触地后约 0.36 s 触发 `FAIL_landing_unrecoverable_pitch`。

只把 `capture_world_active` 时 CATCH 的轮速命令变化率从 **8 提至 16 m/s²**；目标律、腿控制、空中轮控和自适应控制不变。首轮 `catch_slew16_20260929` 在 THRUST 初始姿态门控超时，未进入 FLIGHT，不能用于评价此改动。补跑 `catch_slew16_retry_20260929` 的 FLIGHT 首帧 `+0.070 rad`、`+0.399 rad/s`；触地首帧 `−0.405 rad`、`−1.382 rad/s`、COM lean `−0.653 rad`，与基线接近。

补跑在触地后 0.10 s 的目标仍为 3.693 m/s，实际命令升至 1.665 m/s，证明改动作用于预期通道；同拍车体世界前速从基线 −0.674 变成 −0.890 m/s，COM lean 分别 −0.770/−0.780 rad。补跑约 0.45 s 后同样 `FAIL_landing_unrecoverable_pitch`，恢复自平衡失败。两次的起跳高度增量均约 0.256 m，高于验收上限 0.23 m，这是另一个未解决问题。单次配对只能说明此候选未修复落地，不能量化变化率的因果效果。

由于没有达到修复目标，已撤回 16 m/s² 候选并重新编译，控制器保持原 8 m/s²。原始 CSV 保留在上述三个目录。

## 可复核结论与下一步

`colcon build --packages-select bbot_balance_controller` 成功；`rolling_jump`、`jump_phase_control`、`centroidal_capture`、`jump_regression` 四项测试通过。修改后的默认平地试验未通过；恢复默认后的编译也通过。

当前应以 IMU、轮速、髋膝关节和世界位姿为观测：两次相近的飞行入段都从约 `+0.07 rad/+0.4 rad/s` 演变为触地约 `−0.4 rad/−1.37 rad/s`，触地时 COM 已明显落在轮轴后。CATCH 单纯加快轮命令仍未恢复平衡。下一轮应先在这些可靠通道中量化 **THRUST 结束到 TUCK/EXTEND 的姿态与关节速度时序**，再挑一个能改变触地入段姿态且不破坏起跳高度的变量做配对试验；不得把髋 PD 分支或不可得的接触冲量直接当根因。
