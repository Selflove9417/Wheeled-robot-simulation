# 文件整理方案（待审核，未执行迁移或删除）

本轮原始文件保留原路径，正式ROS功能包、控制配置、源码和构建产物不动。先采用索引形成逻辑分类，日后获批再逐批处理。此方案不含第二代算法设计。

## 1. 建议目录结构

```text
bbot_ws_new/
├── src/                         # 七个正式功能包，结构不变
│   └── bbot_balance_controller/
│       ├── src/                 # 已安装/被launch选择的源码保留
│       │   └── data_logs/       # 现有资料暂保留；旧路径兼容区
│       ├── include/ config/ scripts/ test/
├── docs/
│   ├── jump_controller_v1/      # 本轮：综合说明、参数、实验、盘点与迁移计划
│   │   └── inventory/           # 写文档前清单与依赖/重复线索
│   ├── jump/                   # 入口与既有报告导航，不复制全部报告
│   └── balance/                # 平衡/高度/论文资料入口
├── experiments/jump/
│   ├── planning/               # 原报告及历史设计，保持日期目录
│   ├── tools/                  # 将来统一只读分析/绘图工具，需兼容包装与独立授权
│   │   ├── plotting/ analysis/ audit/
│   ├── candidates/             # 将来仅放明确非正式、独立构建的入口，当前不搬
│   └── archive/                # 将来本地数据整包归档（不是现在创建/搬运）
│       ├── baseline/ thrust/ airborne/ landing/
│       ├── dynamics/ contact_candidate/ legacy_unclassified/
│       └── <campaign>/         # 原相对结构、raw/config/source/diff/provenance完整保留
├── build*/ install*/ log*/     # 全部原位保留
└── README.md / LANDING_REPAIR.md / DEFAULT_FLAT_JUMP.md
```

数据按campaign完整单元归档，不把CSV、图、源码diff分别拆散。类别是索引属性，跨类别实验只给一个主归属和交叉索引；失败状态也是属性，不单独迁出导致配对B/T分离。旧原始目录不迁移是当前最安全方案，体积不靠改路径自动减少。

## 2. 精确迁移决策表

旧路径均相对工作区根。表中“建议”是后续方案，当前没有任何移动指令被执行。

| 旧路径 | 建议新路径 / 本轮决策 | 原因、依赖与批准要求 |
|---|---|---|
| `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`及其include头文件 | 原位保留 | CMake目标、launch、冻结哈希与实验引用；正式核心源码不可搬 |
| `src/bbot_balance_controller/src/bbot_jump_controller.cpp` | 原位保留 | `controller_type=jump`仍有launch/CMake入口，非默认不等于废弃 |
| `src/bbot_balance_controller/src/bbot_reference_jump_controller.cpp`、`bbot_landing_repair_controller.cpp` | 原位保留，索引标注独立候选 | 仍有可执行目标与候选入口；迁到candidates会涉及CMake/package，需单独授权 |
| `src/bbot_bringup/{launch,config,worlds,src}`、`src/bbot_description/`、`src/bbot_kinematics/` | 原位保留 | 正式运行与插件/模型依赖，不移动ROS包 |
| `src/bbot_balance_controller/scripts/{run_flat_ground_jump_trial,jump_profile,jump_startup_gate}.py` | 原位保留，在tools中只做导航 | CMake安装及相邻import、launch从已安装lib导入；移动会破坏启动/测试与根目录推断 |
| `src/bbot_balance_controller/scripts/{record_wheel_contacts,record_wheel_joint_state,wheel_effort_velocity_servo}.py` | 原位保留 | 已安装且runner/候选入口有依赖；不是纯离线脚本 |
| `src/bbot_balance_controller/src/data_logs/plot_jump_log.py` | 将来可提取至 `experiments/jump/tools/plotting/plot_jump_log.py`，旧路径保留薄包装 | 当前默认输入/输出基于`__file__`目录，移动改变行为；需批准代码/路径兼容修改，本轮仅规划 |
| `src/bbot_balance_controller/scripts/analyze_flat_jump_repeatability.py` | 将来 `experiments/jump/tools/analysis/analyze_flat_jump_repeatability.py`，安装入口暂留 | CMake安装，不能只搬文件；后续接口迁移需独立授权 |
| `src/bbot_balance_controller/scripts/analyze_jump_handoff.py` | 将来 `experiments/jump/tools/analysis/analyze_jump_handoff.py`，安装入口暂留 | 同上，先明确输入输出CLI，再评估兼容包装 |
| `src/bbot_balance_controller/scripts/analyze_command_transport.py` | 将来 `experiments/jump/tools/analysis/analyze_command_transport.py`，安装入口暂留 | 同上；运输诊断与动力学结果不是同一种验收 |
| `src/bbot_balance_controller/scripts/audit_{ground_contact_frames,landing_geometry}.py` | 原位保留；tools/audit只添加导航 | runner、安装规则、测试有引用，不拆审计阶段门 |
| `experiments/jump/ground_contact_motion.launch.py`及ground runner/bridge/supervisor | 原位保留 | 用户已有未提交修改；固定路径、哈希与独立candidate链仍依赖，当前不移动 |
| `src/.../data_logs/flat_jump_trials/clearance_apex_ab_20261008` | 将来 `experiments/jump/archive/baseline/clearance_apex_ab_20261008` | 整包包含配对失败/候选数据，基线B1/B2/B3不可单独抽走；先路径兼容与哈希核验 |
| `src/.../data_logs/flat_jump_trials/landing_capture_gain_ab_20261007` | 将来 `experiments/jump/archive/landing/landing_capture_gain_ab_20261007` | 保留全部基线/候选及被否决1.00；gain实际参数注入证据必须同包 |
| `src/.../data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008` | 将来 `experiments/jump/archive/landing/catch_no_reverse_brake_single_20261008` | 包含R1启动失败、唯一物理T1、源码差异/冻结证据；不可删除失败 |
| `src/.../data_logs/flat_jump_trials/catch_world_geometry_single_20261008` | 将来 `experiments/jump/archive/landing/catch_world_geometry_single_20261008` | 原生数据、aligned/current两种几何口径、候选日志和patch整体保留 |
| `src/.../data_logs/flat_jump_trials/{hip_momentum_ab_20261007,arrest_early_decel_ab_20261007}` | 将来 `experiments/jump/archive/airborne/<原campaign名>` | 每项独立旧→新映射见下述CSV；B2/T3失稳记录不能去掉 |
| `src/.../data_logs/flat_jump_trials/thrust_height_032_ab_20261008` | 将来 `experiments/jump/archive/thrust/thrust_height_032_ab_20261008` | 23.957cm失败工况整包保留，不标成稳定高跳基线 |
| `src/.../data_logs/flat_jump_trials/{thrust_release_073_single_20261008,thrust_terminal_forward_single_20261008}` | 将来 `experiments/jump/archive/thrust/<原campaign名>` | 保留EMERGENCY/未联合通过的输入与所有原始帧 |
| `experiments/jump/planning/*/{REVIEW,PROTOCOL,SOURCE,DESIGN}.md` | 原位保留，统一索引 | 已有历史链接与相对脚本/JSON引用；不复制成另一份结论 |
| `docs/jump/reviews/`、`archive/organization_20261005_230854/` | 原位冻结；仅加索引 | 既有历史组织和before快照，不能再当当前文档重复删除 |
| `src/.../data_logs/flat_jump_trails` | 原位待核实 | 搜索范围内无字面引用不等于没有依赖；拼写差异不是内容重复证据 |
| `height_campaign`、chapter42、PID/GS-LQR等data_logs目录 | 原位保留，归平衡/高度导航 | 不属于本轮V1跳跃迁移范围，勿批量归到jump |
| `build*`、`install*`、`log*`、`.ros`、`ros_log`及rc_receiver内嵌构建目录 | 原位保留 | 用户禁止主动清理；部分安装符号链接依赖源码，部分构建绑定实验二进制 |

精确到200个现有campaign的建议映射见 [CAMPAIGN_MIGRATION.csv](inventory/CAMPAIGN_MIGRATION.csv)。该表每行只有一个完整旧路径和一个完整新路径；未审定历史目录放`legacy_unclassified`，不按名称推断废弃或采用。执行表是候选计划，不能直接当批量搬运脚本。

## 3. 重复、冗余与疑似闲置清单

内容重复与可以删除必须分开。小型文本SHA256重复431组，见 IDENTICAL_TEXT.json（本地资料 `inventory/IDENTICAL_TEXT.json`）。没有得到“无需保留、无依赖且可安全删除”的原始资料清单，本轮建议批准删除数量为 **0**。

| 文件/组 | 证据与可能冗余 | 引用核对 / 删除理由是否成立 | 建议 |
|---|---|---|---|
| 当前velocity cpp与13份同内容历史副本（共14份） | SHA256完全相同；分布于A/B source_baseline、before等 | 实验patch/来源哈希/运行器需要该副本；只看重复内容不足以删除 | 保留。以后若要物理去重，须保留每个逻辑路径和不可变来源身份，并单独批准 |
| `src/bbot_rc_receiver/bbot_rc_receiver/rc_node.py`与包内build/install对应文件；CRSF模块同类复制 | 完整内容相同的构建安装副本 | 安装入口及Python运行可能引用，不能据重复认定弃用；属于用户明确禁止本轮清理的构建产物 | 原位保留 |
| `experiments/jump/integration/20261007_realtime_supervisor_final/summary.json`与`...retry5/summary.json` | SHA256一致 | 两个运行上下文不等价；输入/故障记录与报告引用可能不同，字面搜索不能解除provenance依赖 | 保留，索引交叉指明重复结果 |
| integration多个`supervisor_evidence.json` | 相同失败证据模板重复 | 重试/故障分支是完整性记录；删除会隐藏失败或改变场景清单 | 保留 |
| 多campaign冻结配置、头文件、脚本 | 内容相同组数量多 | 冻结清单/哈希/相对import和构建路径有引用；不能改历史source使其“共享最新工具” | 不移除、不统一覆盖 |
| `flat_jump_trails`与`flat_jump_trials` | 名称近似；实际全量清单显示前者164项、约43MiB | 不是已证明重复；选定活动文本字面扫描未命中前者，未覆盖动态组装/外部调用 | 待人工确定来源，当前无删除理由 |
| `docs/jump/reviews`与planning及当前索引 | 主题重叠、含历史产物 | 正文版本/验收范围不同，索引重叠不是内容冗余；有旧报告导航 | 不合并覆盖，建立历史入口 |
| `docs/PROJECT_REFERENCE.md`与根README、各子入口 | 使用说明重叠 | 历史综合说明还被README引用；内容可能过时但不废弃 | 标明历史范围，新用户走新索引；逐章节合并要另批 |
| 原始CSV/JSON/物理帧/惯量/配置/provenance | 体积大、同主题多次 | 是复验和失败证据，不存在删除理由 | 完整保留 |
| 可重生成PNG/派生CSV | 理论上可由脚本再生成 | 本轮没有逐一证明相同输入/工具版本可重现；有报告图引用，不给出假“可删除”名单 | 将来只在输入、生成器、哈希、图链接全部核验后单独批准，不在本轮删 |

原始清单是逐文件依据；相同组覆盖有限类型和大小，不能声称34GiB数据已有完整去重。主题重复的报告以增加索引解决，避免再写一套平行历史。

## 4. 已确认路径依赖与风险

PATH_REFERENCES.csv（本地资料 `inventory/PATH_REFERENCES.csv`）列出选定路径的586条字面命中（含部分planning冻结文本）；不是完整运行依赖图，动态拼接、安装链接、外部终端历史和原始目录内脚本还需逐项复核。

1. `bbot_balance_controller/CMakeLists.txt:227`安装21个脚本，含runner/profile/startup/audit；同文件测试也通过`${CMAKE_CURRENT_SOURCE_DIR}`找脚本与源码。移动它们须修改安装/测试规则，超出本轮权限。
2. `run_flat_ground_jump_trial.py:23–34`按脚本相邻目录导入jump_startup_gate/jump_profile，用`Path(__file__).resolve().parents[3]`推断工作区；顶层初始化会加载安装环境、创建日志目录。此次只读文本，没有import执行。移动会改变根推断和副作用目标。
3. `bbot_gazebo.launch.py:32–45`从package_prefix/lib导入已安装jump_profile，也按parents[3]推根；模型xacro、world和插件来源有源码/安装两条路径。符号链接安装不意味着任意搬目录仍可运行。
4. `run_complete_jump_demo.sh:11`把输出写到当前data_logs/flat_jump_trials；runner默认output-dir也写这里。移动老数据不能直接让新数据自动归档；本轮不改默认输出。
5. `plot_jump_log.py`默认输入/输出相对于脚本目录；物理位移依赖显式同次geometry和metrics。搬脚本后必须保持CLI与默认行为，不得以积分轮速替换缺失真位移。
6. independent ground-contact runner直接引用`experiments/jump/ground_contact_motion.launch.py`，并冻结源文件/模型/二进制；移动会导致资格哈希、证书或模型路径不匹配。禁止为整理而放宽完整性门。
7. 原始provenance、report、run_one及分析脚本中既有绝对`/home/xy/bbot_ws_new/...`、相对campaign、source_baseline/source_test路径。历史文件不重写为新路径，否则原来源记录被改变；迁移后需**外置路径映射/兼容解析**并保留原记录，不能悄悄覆盖哈希。
8. 安装符号链接、独立构建CMakeCache与可执行文件依赖源/库；不要用迁移间接清理build/install。现有链接目标在全量CSV可查。

## 5. Git与文档治理

当前`.gitignore`已忽略build*/install*/log*、data_logs大量数据、docs/jump/reviews及planning大部分内容，并对可上传报告/脚本逐条开白名单。Git当前有用户修改和148项未跟踪，不能用`git add .`把新试验全部收入。

后续建议单独批准：

- 对两次最新CATCH目录仅增加`REVIEW.md`及必要轻量分析源码的窄例外；CSV/PNG/模型/二进制/冻结副本仍为本地资料。本轮报告已在索引中以代码路径标明，不制造不存在GitHub文件链接。
- 若批准建立`experiments/jump/archive/`，先增加该**具体目录**的忽略规则及仅manifest/README例外；不能泛化忽略整个experiments，避免隐藏必要控制与审计代码。
- 当前inventory CSV/JSON是总结快照；可审核后决定哪些清单随Git保存，哪些较大逐文件清单留本地。不把Git状态字段当今后永久状态。
- docs导航→V1总览→参数/实验/迁移→原报告；旧README、DEFAULT_FLAT_JUMP与LANDING_REPAIR保留边界。若将来合并正文，先逐条转移有效信息与旧链接锚点，不覆盖冻结原报告。

本轮未修改`.gitignore`、包规则或默认参数，也未提交/推送。

## 6. 分批实施与需要批准的事项

| 顺序 | 后续动作 | 验证/回退条件 | 是否本轮完成 |
|---|---|---|---|
| 0 | 盘点、文档、虚拟分类、迁移表与导航 | 只改新文档/导航；既有源码配置哈希不变 | 是 |
| 1 | 用户审核分类、保留列表、最新报告Git范围 | 明确哪些可上传；不批量移动 | 待批准 |
| 2 | 批准一个纯离线工具提取试点及旧入口兼容修改 | 先记录输入/输出/依赖；比较既有文件生成结果，保留旧入口；运行验证需另批范围 | 待批准，不自动执行 |
| 3 | 批准一个campaign整包归档及外置路径映射 | 复制到临时目标→数量/逐文件哈希/来源与链接完整性核对→审核后才切换；旧数据保留至另批删除 | 待批准，涉及额外磁盘空间 |
| 4 | 其余campaign按精确映射逐批归档 | 禁止跨盘失败留下半包、禁止覆盖同名试次，失败可回原路径 | 待批准 |
| 5 | 若确有可再生重复产物，单独提交逐文件删除清单 | 每项生成输入、引用者、重建哈希与回退副本齐全；原始失败数据永不随“去重”删除 | 当前无可批准删除项 |

目录更整齐不能作为性能改善或物理资格通过的证据。审核完本轮方案前，停止文件迁移和控制器工作。

## 发布准备补充（2026-10-08）

本文件保留原迁移方案，不代表全部已执行。后续已完成12个大型实验本地归档，以及plot_jump_log和analyze_flight_momentum兼容迁移；其余工具不迁移。机器路径引用与重复组快照仍是本地资料，不上传；现状见[发布准备清单](../RELEASE_PREPARATION.md)。
