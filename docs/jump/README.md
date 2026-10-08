# 跳跃入口与状态

[文档目录](../README.md) · [项目首页](../../README.md) · [实验入口](../../experiments/jump/README.md) · [历史实验索引](../../experiments/jump/RECORDS.md)

## 当前状态

第二轮最新决定为 **NO_PHYSICAL_ADMISSION**：阶段二未完成，本轮新增实跑 0 次、腾空 0 次、达标 0 次。完整 81 种摩擦状态的静摩擦交接和逆解兼容取得离线模型进展；快速求解仅覆盖有限状态，实时模式独占监视、有限故障卸力恢复和停止包络实时保护尚未完成。候选控制律尚未接入输出，控制器内部拦截仍在。详情以[最新阶段二验收报告](../../experiments/jump/planning/contact_motion_revision_20261006_round2/REVIEW.md)为准。

这不改变已有 velocity 入口，也不代表跳跃验收完成。完整验收要求在正常速度、实时倍率 1.0、物理步长 1 ms 下，同一会话连续完成三跳，且每跳都达到全部指标：真实双轮轮底净空至少 20 cm；首触前倾角 0–5° 且俯仰角速度绝对值不超过 0.20 rad/s；实际屈腿压缩至少 8 cm；首触后轮轴和质心后退都不超过 1 cm，并完成原有 12 s 保持检查。质心升高、动作流程成功和离线模型结果不能替代这些实测指标。

## 默认 velocity 跳跃

- 控制器：[bbot_velocity_jump_controller.cpp](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp)。
- 标准跳跃试验运行器：[run_flat_ground_jump_trial.py](../../src/bbot_balance_controller/scripts/run_flat_ground_jump_trial.py)。
- 启动：[bbot_gazebo.launch.py](../../src/bbot_bringup/launch/bbot_gazebo.launch.py)，`controller_type:=jump_velocity`。
- 示例：`ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump_velocity jump_height:=0.25`。
- 自动演示：[run_complete_jump_demo.sh](../../run_complete_jump_demo.sh)。
- 默认参数：[jump_profile.py](../../src/bbot_balance_controller/scripts/jump_profile.py)。
- 仿真世界：[flat_jump_world.sdf](../../src/bbot_bringup/worlds/flat_jump_world.sdf)。
- 经典跳跃控制器：[bbot_jump_controller.cpp](../../src/bbot_balance_controller/src/bbot_jump_controller.cpp)，`controller_type:=jump`。

站立稳定后按 **J**，或在另一个已加载环境的终端运行：

```bash
ros2 topic pub --once /jump_cmd std_msgs/msg/String '{data: jump}'
```

`jump_height` 生成离地速度目标，不是实际轮底净空。默认入口状态和验收进展见[LANDING_REPAIR.md](../../LANDING_REPAIR.md)、[修复历史](LANDING_REPAIR_HISTORY.md)、[2026-10-06 可行性复核](FEASIBILITY_REVIEW_20261006.md)和[reference 对照说明](../../experiments/jump/REFERENCE_FLAT_JUMP.md)。

## 审计与记录

- 试验入口、构建路径和当前资格状态：[experiments/jump/README.md](../../experiments/jump/README.md)。
- 原始实验与离线审计报告索引：[experiments/jump/RECORDS.md](../../experiments/jump/RECORDS.md)。
- 接触和净空审计：[audit_ground_contact_frames.py](../../src/bbot_balance_controller/scripts/audit_ground_contact_frames.py)、[audit_landing_geometry.py](../../src/bbot_balance_controller/scripts/audit_landing_geometry.py)。
- 扭矩与原生只读输入记录：[record_joint_torque.py](../../src/bbot_balance_controller/scripts/record_joint_torque.py)、[landing_repair_wrench_recorder.cc](../../src/bbot_bringup/src/landing_repair_wrench_recorder.cc)。发布命令、JointForceCmd 仿真输入和关节合负载是不同量。
- 所有原始历史运行文件保留在本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/` 原路径。

## 第一代实现冻结与资料整理

2026-10-08：[第一代velocity跳跃技术总结](../jump_controller_v1/README.md)，含分层状态机图、模式切换、实际公式、默认与实验参数区别，以及[33项重点实验和结果](../jump_controller_v1/EXPERIMENTS.md)。默认能完整跳跃与独立候选NO_PHYSICAL_ADMISSION分开记录；未采用候选不视为默认改进。后续文件整理先审阅[迁移方案](../jump_controller_v1/MIGRATION_PLAN.md)，本轮不移动原始资料。
