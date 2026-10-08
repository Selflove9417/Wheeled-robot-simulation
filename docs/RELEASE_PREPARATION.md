# 第一代收尾与整个BBot工程发布准备

日期：2026-10-08。状态：**文档及拟提交清单已准备，等待用户审核；未stage、commit、push或创建tag**。项目是完整机器人工作区，不是单独跳跃项目。本次不开始第二代设计或开发。

[项目首页](../README.md) · [逐文件提交清单](RELEASE_MANIFEST.tsv) · [静态检查与限制](RELEASE_CHECKS.json)

## 目标仓库、分支和版本

- origin：`git@github.com:Selflove9417/Wheeled-robot-simulation.git`，对应 [GitHub仓库](https://github.com/Selflove9417/Wheeled-robot-simulation)。fetch/push配置相同。
- 当前及建议目标分支：`work/jump-repair`。保持当前分支，不自动合入或覆盖 `main`。
- 本地HEAD：`e32bfb5`。本地已有remote-tracking信息不能代表远端当前状态。
- 建议后续版本标识：`jump-controller-v1`。本地没有tag；远端只读查询因DNS解析失败，无法确认远端tag重名、分支最新提交、访问权限或保护规则，正式发布前必须重新核对，不覆盖同名tag。

## 本轮实际编辑

1. 根README改为整个BBot工程介绍，核实7个功能包、平衡/高度/运动/跳跃/硬件模块、实际软件版本、构建与启动入口。
2. 补充源码实际区别：`lqr`也插值高度增益；固定中点是自适应节点的独立配置方式；正式launch入口不等于统一物理验收。
3. 复用已跟踪的 `figures/fig4_pid_gslqr_height.png`；不生成新结果或复制本地实验图。
4. 文档导航更新，盘点与迁移规划注明历史快照和后续完成状态；机密性较低但不适合公开的机器全量清单保留为本地资料。
5. `.gitignore`补充嵌套build/install/log、本地归档拷贝目录、一个已有隔离运行输出目录及5份机器清单；只对6份必要历史REVIEW.md精确放行，其他CSV/JSON/日志与冻结内容仍忽略。
6. 修正 `src/bbot_description/README.md` 中正式launch的相对链接；不修改模型。
7. 新增本发布说明、逐文件清单和静态验证记录。

没有编辑控制算法、模型、配置、launch、CMake、其他分析脚本、原始实验或冻结报告；两个工具的先前迁移成果直接复用。

## 拟提交范围

精确路径、完整文件长度及SHA256见 [RELEASE_MANIFEST.tsv](RELEASE_MANIFEST.tsv)，其中 `new`/`modified` 是本批拟提交项，`deferred_*` 是保留在工作区、不纳入本批的已有候选工作。自引用清单文件不记录自哈希。

| 类别 | 范围 |
|---|---|
| 新增 | V1六份总结/索引文档及10份轻量索引记录；两个通用工具及README/验证记录；107份现有、精确放行的planning报告/协议/离线分析/源码diff；6份历史REVIEW；本发布说明及清单/检查记录 |
| 修改 | README、.gitignore、文档/实验导航和状态说明、模型包README链接、两个工具旧路径兼容模块 |
| 删除 | **0** |
| 已跟踪且无需重复加入 | 正式7个功能包源码、模型网格/URDF、launch/config、构建声明、通用运行器与已公开图像；它们继续属于拟发布Git树 |

拟提交新增 **139** 项、修改 **10** 项、删除 **0** 项。新增和修改的完整文件体积约 **2.2 MB**（不是Git压缩传输量）；现有源码加本批后的完整拟发布树约 **15.6 MB**，不含Git历史。清单不替代后续逐路径stage操作，禁止直接 `git add .`。

## 用户原有工作区修改

进入本轮时已有 `.gitignore`、LANDING_REPAIR、docs与experiments导航、两个兼容入口，以及landing-repair代码/测试/构建/候选运行器改动和未跟踪文件。本轮保留全部原工作，不回滚、不覆盖。

下列7项已跟踪控制/集成改动**不纳入此次文档与V1工具收尾提交**：

- `experiments/jump/ground_contact_motion.launch.py`
- `experiments/jump/ground_engine_state_bridge.py`
- `experiments/jump/run_ground_contact_motion_trial.py`
- `src/bbot_balance_controller/CMakeLists.txt`
- `src/bbot_balance_controller/include/bbot_balance_controller/ground_contact_motion_fast_certificate.hpp`
- `src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp`
- `src/bbot_balance_controller/test/offline/test_ground_contact_motion_fast_certificate.cpp`

未跟踪supervisor、recovery/safety头文件、相应测试和integration测试同样保留并延期。源码改动与文档里历史候选的描述不应混同为本次验收；该候选仍是 `NO_PHYSICAL_ADMISSION`。若希望发布包含这些候选改动的整个工作树，需要另行审查和授权，不能沿用本清单直接全量提交。

## 默认不上传

- `~/bbot_jump_archives/`：12个大型tar.gz与原验证记录在仓库外，本轮不操作；没有安排外部备份。
- 原始CSV、高频物理帧、运行JSON/JSONL、冻结源码数据副本、PNG/PDF等实验派生产物（已有Git跟踪展示图例外）。
- build/install/log、ROS/Gazebo缓存、可选本地 `opt_ros/`、独立实验二进制和隔离运行输出。
- 本地 `inventory/{FILES.csv,IDENTICAL_TEXT.json,SUMMARY.json,PATH_REFERENCES.csv,git_status_before.txt}`：不删除，只从拟上传范围排除；文档以本地资料代码路径标注。
- 密钥、令牌、私人配置；本轮仅做常见高风险字面模式检查，不能视为全面隐私认证。

`.gitignore`不会取消已跟踪文件；已检查拟发布树，没有已跟踪tar.gz、原始CSV、JSONL、.so或.o。对历史报告只放行指定文件，不开放整个数据目录。

## 完整性和验证

- velocity核心C++、centroidal_state.hpp、正式Gazebo launch和控制器YAML与已有 `inventory/SOURCE_SHA256.json` 四项一致。
- 机器人模型、运动学和其正式运行文件相对HEAD无修改；没有可用的完整模型冻结哈希清单，本轮未解压归档，因此不宣称完成逐文件冻结模型证明。
- plot旧入口因已批准迁移不再匹配原实现哈希；按已记录的默认目录替换逆变换后，新实现与原冻结实现哈希一致。复用先前9张PNG字节一致验证，不重新绘图。
- momentum新实现与38项验证记录中的原实现哈希一致，复用3份日志17,188行的精确结果；未改公式或模型。
- 4个Python新旧入口通过AST语法检查；7份package.xml可解析；`git diff --check`通过。未运行新增测试、构建、Gazebo或硬件。
- 拟提交内容的常见私钥/GitHub令牌/AWS密钥/字面密码模式检查无命中；没有超过1 MB的单文件。已跟踪的平衡展示图随GitHub正常提供。
- 核心首页、导航、V1总结及其直接入口的文件路径检查通过。检查不证明所有Markdown锚点或在线外链可访问。

## 仍需公开说明的风险

1. **干净机器重建尚未验证**：工作机使用Ubuntu22.04、Iron和Fortress组合，且有未发布的本地插件补充目录。README明确依赖，不以现有install成功代替公开安装可复现。CRSF包依赖/许可证仍有TODO声明，根目录也没有独立LICENSE；公开授权范围应在正式版本前由维护者确认，本轮不擅自变更许可证或package.xml。
2. **远端未核实**：网络恢复后需要确认目标分支最新状态和tag是否重名；不据本地remote-tracking信息直接推送。
3. **历史深层链接不是完整在线数据包**：冻结报告保留原貌，许多JSON/CSV/图和源码`:行号`伪链接只适于本地阅读；另有旧的inverse-compatibility报告及文本证据缺失。逐项列表见RELEASE_CHECKS.json。没有为消除这些历史链接而上传大型数据或虚构缺失文件。当前核心导航已可达。
4. **历史性能不是所有方法通用保证**：默认V1仍有净空不足、落地后退；独立修复未通过，实机CAN/CRSF代码未据此获得跳跃验收。
5. **展示资源选择受限**：现有仓库内可公开平衡图已复用。若要补Gazebo截图或跳跃图，应先从本地记录选一个明确试次、建议单张≤1 MB的现有文件，建议公开位置 `docs/media/`，记录原路径和试次并精确放行；本轮未复制任何文件，不为美化而上传整个目录。

下一步仅等待用户审核提交范围、目标分支和上述风险。本轮不创建 `jump-controller-v1`，不自动上传，不开始第二代开发。

## 本次发布的获批格式例外

用户已批准继续提交。仅在检查命令中设置 `core.whitespace=cr-at-eol`，保留CSV/TSV的原CRLF，不修改Git全局配置。完整暂存检查仍有10个历史文件、11条告警；逐文件数量、历史依据与SHA256见 [RELEASE_CHECKS.json](RELEASE_CHECKS.json) 的 `approved_publication_whitespace_exceptions`。这些CSV快照、历史campaign脚本/报告及patch/diff证据字节保持不变；没有对目录批量格式化。以逐文件literal排除pathspec检查其他139个暂存文件，通过。此结论是“排除已审核例外后通过”，不是完整检查零告警。本节仅补充发布检查决定，不重写历史实验结论。
