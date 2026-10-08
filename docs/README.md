# 文档目录

[回到项目首页](../README.md)

`docs/` 放项目说明、控制器使用参考与跳跃验收文档；本目录只提供导航，不复制实验原始结果。

源码分支不包含被忽略的运行 CSV，也不自动包含本地冻结副本。对此类内容，文档使用代码路径并标注“本地资料”，不创建会在 GitHub 断开的链接。支撑当前阶段结论的验收 Markdown 报告作为必要文档保留并链接；详见[实验记录索引](../experiments/jump/RECORDS.md)。

| 路径 | 内容 |
| --- | --- |
| [balance/](balance/README.md) | 自平衡控制方式、launch 类型、对应源码与配置 |
| [jump/](jump/README.md) | 默认跳跃入口、当前修复状态与实验记录导航 |
| [PROJECT_REFERENCE.md](PROJECT_REFERENCE.md) | 迁入 docs 的历史综合说明；其中参数和实验结论可能较旧 |
| [BBOT_SIMULATION_DEBUG_MANUAL.md](BBOT_SIMULATION_DEBUG_MANUAL.md) | 仿真环境调试与运行说明 |

## 后续文件归类

- 工作区环境、通用仿真使用说明放在 `docs/`。
- 平衡方法、控制器启动和配置说明放在 `docs/balance/`。
- 跳跃入口、固定验收目标和当前修复结论放在 `docs/jump/`。
- 带日期的实验计划、审计报告、准入决定、原始数据和冻结快照继续保存在 `experiments/` 或既有 `src/.../data_logs/` 路径；导航页只链接权威文件，不重写或搬动记录。

首页：[README.md](../README.md) · 实验索引：[experiments/jump/RECORDS.md](../experiments/jump/RECORDS.md)

## 第一代跳跃控制器总结与整理规划（2026-10-08）

[完整技术总结](jump_controller_v1/README.md) · [参数与源码](jump_controller_v1/PARAMETERS.md) · [实验与性能](jump_controller_v1/EXPERIMENTS.md) · [文件盘点](jump_controller_v1/INVENTORY.md) · [待审核迁移方案](jump_controller_v1/MIGRATION_PLAN.md)。盘点与迁移规划保留其历史快照；后续已完成12个大型实验本地归档及两个通用工具兼容迁移，见[归档记录](jump_controller_v1/ARCHIVE_INDEX.md)和[发布准备清单](RELEASE_PREPARATION.md)。

## 公开资料与本地证据边界

公开核心导航与V1总结链接应在拟提交树内可达。历史冻结报告保留原貌，其中JSON、CSV、日志及图像的原始相对链接可能仅在本机资料恢复后有效，不代表GitHub提供完整证据包。少量必要的历史REVIEW.md通过精确忽略例外公开；不整目录放行数据。未修复的历史深层链接与限制列于[发布准备清单](RELEASE_PREPARATION.md)。
