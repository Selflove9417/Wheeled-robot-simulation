# 工作区协作约定

## 目录与入口

- `src/bbot_balance_controller/`：平衡、跳跃控制器及配置、脚本和测试。
- `src/bbot_bringup/`：ROS launch、控制器配置、Gazebo 世界与插件。
- `src/bbot_description/`、`src/bbot_kinematics/`：机器人模型与运动学。
- `docs/`：面向使用者的说明；从 [docs/README.md](docs/README.md) 导航。
- `experiments/jump/`：候选运行器、离线审计和实验记录索引；从 [实验入口](experiments/jump/README.md) 开始。
- `src/bbot_balance_controller/src/data_logs/`：已有本地运行数据，保留原路径。

日常环境与启动命令见 [README.md](README.md)。跳跃修复目标及当前判断见 [LANDING_REPAIR.md](LANDING_REPAIR.md)；最新阶段二审查见 [Round 2 REVIEW](experiments/jump/planning/contact_motion_revision_20261006_round2/REVIEW.md) 和 [入口补充审查](experiments/jump/planning/contact_motion_revision_20261006_round2/REALTIME_ENTRY_ADDENDUM.md)。

## 修改与实验边界

- 先检查工作区状态和相关报告。保留用户已有修改、历史数据、冻结源码副本、构建目录及原始报告；除非用户明确授权清理，不要清理、覆盖、移动或重命名它们。
- 候选控制器和实验应保持独立，不擅自替换默认入口。用户未要求时，不修改默认控制、其他控制方式、控制限制或验收门。
- 已定义的力矩/速度限制、时钟配对规则、完整性条件和阶段门不得放宽；缺失或无效数据保持 invalid/NaN，不以补零、筛帧或误差筛选制造通过结果。
- 报告中分开说明真实物理运行、离线模型/回放、源码检查和单元测试；单元测试或离线结果不能表述为物理验收。
- 不启动仿真或改变实验次数，除非任务明确授权；保留每轮完整输入、故障和失败样本。
- 编辑范围按当前任务要求执行。不要为了导航更新而改动源代码、参数、原始数据或冻结记录。
- 原始数据和本地冻结记录不随源码分支上传时，在文档中用代码路径标注“本地资料”，不要创建指向不存在 GitHub 文件的链接；需要支撑当前结论的验收 Markdown 报告可以保留为可上传文件并正常链接。

## 当前状态

跳跃修复仍未完成。最新第二阶段结论为 `NO_PHYSICAL_ADMISSION`：虽然静摩擦交接、逆解兼容和部分离线检查有进展，实时独占模式监视、有限故障恢复、停止包络保护及完整运动独立复验尚未通过；候选控制律仍未接入正式输出。当前阶段二未通过，跳跃不得标记为已修复。以链接的原始验收报告为准，不从源码存在或测试通过推导物理资格。
