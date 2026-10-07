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
