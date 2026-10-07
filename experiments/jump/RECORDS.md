# 跳跃与接地实验记录索引

[实验入口与当前准入](README.md) · [跳跃文档](../../docs/jump/README.md) · [项目首页](../../README.md)

本页只标注记录的类型与结论并链接到原文件；数据、原始报告和冻结快照保持原路径，不在此复制改写。`实跑`、`离线模型/审计`、`源码/测试`是不同证据类型。

标注为“本地资料”的记录不随源码分支上传。原始数据和冻结记录以代码路径标注，不做 Markdown 链接，避免 GitHub 页面出现失效链接。当前结论所需的小型验收 Markdown 报告单独保留为可上传文档，见 Round 2 条目。

## 近期物理运行

| 日期 / 记录 | 证据类型 | 结论与范围 | 原始报告 |
| --- | --- | --- | --- |
| 2026-10-06 velocity 接管与卸力保护 | 实跑 1 次 | 0 次腾空、0 次修复成功；不能满足跳跃验收 | 本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/velocity_capture_handoff_20261006_082545/REVIEW.md` |
| 2026-10-06 第一阶段站立输入门 | 实跑 1 次 | 五秒完整六路物理输入及位置→力矩模式站立交接通过；模型响应、刹停和跳跃未测 | 本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_input_stage1_20261006_091717/REVIEW.md` |
| 2026-10-06 原生引擎站立探针 | 实跑 1 次 | 固定五秒窗的原生状态与静止站立九维模型门通过；仅限该站立工况，不代表运动或停止资格 | 本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/native_engine_standing_probe_20261006_105035/REVIEW.md` |
| 2026-10-06 首档接地运动 | 实跑 1 次 | 三个运动窗口的九维响应和当次正常停止通过；双髋激发失败，第 2–4 试次及独立复验未执行，整体未通过 | 本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_motion_probe_20261006_134820/REVIEW.md` |

## 当前接地候选与计算记录

| 记录 | 证据类型 | 决定 | 权威文件 |
| --- | --- | --- | --- |
| 接地候选资格，Round 1 | 离线资格审查 | 静止交接失败，取消实跑；没有接地刹停资格 | 本地冻结记录 `planning/contact_motion_qualification_20261006/REVIEW.md`、`acceptance.json` |
| 接地候选，Round 2 | 离线模型、有限数值对照、源码/CTest | 完整 81 模式静摩擦交接与逆解兼容有进展；快速原型覆盖窄，实时模式监视、有限故障恢复和在线停止包络未通过，**NO_PHYSICAL_ADMISSION**；本轮实跑 0 次、腾空 0 次、达标 0 次 | [验收报告](planning/contact_motion_revision_20261006_round2/REVIEW.md) · [入口补充审查](planning/contact_motion_revision_20261006_round2/REALTIME_ENTRY_ADDENDUM.md) · [实时入口审查](planning/contact_motion_revision_20261006_round2/REALTIME_ACCESS_REVIEW.md)；其他验收产物与数据为本地冻结记录 `planning/contact_motion_revision_20261006_round2/` |
| Round 2 静止交接与完整逆解 | 离线模型 fixture | 原生旧锚点静摩擦交接和兼容 fixture 通过；不是物理运动或响应验收 | 本地冻结记录 `planning/contact_motion_revision_20261006_round2/static_blend_witness.csv`、`static_witness_native_crosscheck.json`；结论见[验收报告](planning/contact_motion_revision_20261006_round2/REVIEW.md) |
| Round 2 v4 往返停止轨迹 | 离线模型回放 | 模型轨迹、停止摘要与 sidecar 核对完成；`MODEL_ONLY`，不是实测停止距离 | 本地冻结记录 `planning/contact_motion_revision_20261006_round2/v4_stop_metrics/v4_model_review.md`、`root_v4_stop_crosscheck.json` |
| Round 2 快速求解 certificate | 离线数值测试 | 300 个扰动样本中 47 接受、253 截止拒绝；接受子集与完整模型一致，但覆盖窄、未做 ROS/物理验证 | 本地冻结记录 `planning/contact_motion_revision_20261006_round2/fast_certificate_final/FAST_CERTIFICATE.md`、`FAST_CERTIFICATE.json` 和独立结果 `root_fast_independent/result.txt` |

## 目标、默认入口与历史材料

- 当前跳跃目标和阶段判断：[LANDING_REPAIR.md](../../LANDING_REPAIR.md)；阶段历史：[LANDING_REPAIR_HISTORY.md](../../docs/jump/LANDING_REPAIR_HISTORY.md)；[2026-10-06 可行性复核](../../docs/jump/FEASIBILITY_REVIEW_20261006.md)。
- 默认 velocity 和经典跳跃入口：[默认版本说明](../../DEFAULT_FLAT_JUMP.md)；reference 对照说明：[REFERENCE_FLAT_JUMP.md](REFERENCE_FLAT_JUMP.md)。
- 旧版项目操作与方法背景：[PROJECT_REFERENCE.md](../../docs/PROJECT_REFERENCE.md)；仿真调试：[BBOT_SIMULATION_DEBUG_MANUAL.md](../../docs/BBOT_SIMULATION_DEBUG_MANUAL.md)。历史说明中的版本、配置和结论应按其原日期理解。
- 所有原始运行文件仍在本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/`；接触、几何、速度与输入审计工具见 [scripts/](../../src/bbot_balance_controller/scripts/)。
