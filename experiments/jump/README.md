# 跳跃与接地实验入口

[项目首页](../../README.md) · [实验历史索引](RECORDS.md) · [跳跃文档](../../docs/jump/README.md)

## 当前准入

第二轮最新决定：**NO_PHYSICAL_ADMISSION**；阶段二未完成，本轮新增实跑 0 次、腾空 0 次、达标 0 次。完整 81 种摩擦状态的静摩擦交接和逆解兼容取得离线模型进展；快速求解仅覆盖有限状态，实时模式独占监视、有限故障卸力恢复和停止包络实时保护尚未完成。候选控制律尚未接入输出，控制器内部拦截保留。不要据模型停止曲线或单元测试启动接地运动/跳跃。权威结论：[第二轮 REVIEW](planning/contact_motion_revision_20261006_round2/REVIEW.md) · [入口缺口补充审查](planning/contact_motion_revision_20261006_round2/REALTIME_ENTRY_ADDENDUM.md)。机器验收副本 `planning/contact_motion_revision_20261006_round2/acceptance.json` 为本地记录。

当前开发分支为 `work/jump-repair`，尚未验收；`main` 仍是旧基线。GitHub 源码分支不会带上被忽略的原始运行数据或本地冻结副本；需要引用时请使用 [记录索引](RECORDS.md)，其中本地资料使用代码路径标注，不生成失效链接。当前阶段必需的 Markdown 验收报告保留为可上传文档。

## 入口

| 脚本 | 用途 | 当前范围 / 结果 |
| --- | --- | --- |
| [run_ground_input_trial.sh](run_ground_input_trial.sh) | 分配器关闭、有界 P 轮速、完整六路实际输入与站立交接 | 固定五秒输入门通过；不包含模型响应、刹停或跳跃资格 |
| [run_ground_motion_trial.sh](run_ground_motion_trial.sh) | 独立四腿低速接地运动与正常停止 | 实跑 1 次；响应门和当次停止通过，双髋未达激发门，第 2–4 试次及独立复验未执行，整体未通过 |
| [run_ground_contact_motion_trial.sh](run_ground_contact_motion_trial.sh) | 直接 Physics 状态、接地逆解和静摩擦交接候选 | 离线模型有进展但快速求解覆盖有限；实时监视、恢复和停止保护未完成，准入 NO，本轮未实跑 |
| [run_velocity_capture_candidate.sh](run_velocity_capture_candidate.sh) | velocity 髋膝协调与接管保护候选 | 最近记录实跑 1 次、未腾空；不是修复完成版，不能替换默认入口 |
| [run_allocator_jump_candidate.sh](run_allocator_jump_candidate.sh) | 独立支撑分配器跳跃候选 | 尚未通过跳跃验收 |
| [run_capture_command_probe.sh](run_capture_command_probe.sh) | 分配器命令/状态时序诊断 | 诊断入口，不是修复版 |
| [run_native_command_probe.sh](run_native_command_probe.sh) | 原生实际输入观测诊断 | 独立诊断入口 |
| [run_reference_jump_demo.sh](run_reference_jump_demo.sh) | 早期 reference 对照 | 历史实验 |

这些候选均不改变首页的默认 velocity 跳跃入口。独立实验控制器和 launch 仍位于 ROS 功能包中；同名顶层实验脚本集中在本目录。比如从工作区根运行：

```bash
./experiments/jump/run_allocator_jump_candidate.sh --help
```

## 构建与数据

已有独立构建目录均保留原位：`build_native_allocator/`、`build_native_command_observer/`、`build_support_allocator/`、`build_ground_input/`、`build_ground_motion/`、`build_ground_motion_contact/`、`build_ground_native_physics/`。原始历史 CSV、freeze snapshots 和报告不移动、不覆盖；按阶段从[记录索引](RECORDS.md)打开权威结果。

原生 Physics 插件构建及选取说明见 [native_physics/README.md](native_physics/README.md)。直接引擎状态审计脚本为 [audit_native_engine_state.py](audit_native_engine_state.py)；GetForce getter 仅作旁路诊断，不能替代物理步前的 JointForceCmd 输入。

## 固定约束

接地协议关闭分配器，P 轮伺服增益固定 1.0，轮速目标限幅 ±30 rad/s、轮力矩 ±10 Nm，髋/膝力矩上限 75/60 Nm；Stage 1 和各实验协议的具体差异以相应 `protocol.json` 与原审计报告为准。缺失输入保持 invalid/NaN，不用参考加速度、getter 值或同值命令匹配补数据。
