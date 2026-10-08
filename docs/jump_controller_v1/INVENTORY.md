# 工作区盘点（2026-10-08，写文档前快照）

本轮只新增总结文档与导航。完整路径、Git归属、逻辑长度、分配空间及符号链接目标见 FILES.csv（本地资料 `inventory/FILES.csv`），聚合与功能包声明见 SUMMARY.json（本地资料 `inventory/SUMMARY.json`），原有工作区改动见 Git状态快照（本地资料 `inventory/git_status_before.txt`）。统计使用lstat，不沿符号链接重复计数，不计目录自身元数据；GiB=2³⁰字节，MiB=2²⁰字节，硬链接不作去重。分类按路径/扩展名作初筛，不能据此判定运行职责或废弃。

共 **45,192项**（普通文件44,952、符号链接240），逻辑空间 **38.029 GiB**，分配空间 **38.134 GiB**。无读取错误。Git跟踪660项、未跟踪148项；余下44,384项统计为忽略/内部资料，其中1,768项是`.git`内部文件，不能称作待清理的忽略数据。已有修改与未跟踪文件属于用户工作，不能用本轮结果覆盖。

| 初筛类别 | 项数 | GiB | 跟踪/未跟踪/忽略或内部 |
|---|---:|---:|---|
| 其他根目录及工具 | 13,657 | 0.060 | 12/0/13645 |
| Git元数据 | 1,768 | 1.017 | 0/0/1768 |
| 项目文档 | 250 | 0.310 | 16/0/234 |
| 构建安装运行日志目录 | 12,436 | 1.686 | 0/0/12436 |
| 实验报告及设计文档 | 86 | 0.001 | 24/41/21 |
| 实验模型审计产物及配置 | 1,788 | 0.610 | 6/38/1744 |
| 实验工具及探针源码 | 220 | 0.006 | 32/65/123 |
| 数据目录内冻结源码及快照 | 2,936 | 0.127 | 0/0/2936 |
| ROS功能包文件 | 842 | 0.012 | 562/4/276 |
| 运行数据及派生资料 | 9,710 | 34.179 | 0/0/9710 |
| 数据目录内实验及分析脚本 | 1,499 | 0.022 | 8/0/1491 |

## 实际顶层目录

| 路径 | 项数 | MiB |
|---|---:|---:|
| `src` | 14,986 | 35163.60 |
| `.git` | 1,768 | 1041.58 |
| `build` | 1,623 | 694.90 |
| `experiments` | 2,095 | 631.83 |
| `docs` | 246 | 317.51 |
| `log` | 3,056 | 220.92 |
| `build_ground_motion` | 1,014 | 90.69 |
| `build_arrest_early_decel` | 172 | 66.06 |
| `build_thrust_terminal_forward_single` | 171 | 66.04 |
| `build_arrest_early_decel_baseline` | 171 | 66.04 |
| `build_clearance_apex_baseline` | 171 | 66.04 |
| `build_hip_momentum_baseline` | 171 | 66.04 |
| `build_clearance_apex_test` | 171 | 66.04 |
| `build_hip_momentum_test` | 171 | 66.03 |
| `.ros` | 8,830 | 56.71 |
| `build_native_command_observer` | 128 | 43.52 |
| `install_ground_motion` | 116 | 31.92 |
| `build_ground_finite_recovery` | 776 | 28.32 |
| `install` | 283 | 27.31 |
| `build_ground_input` | 834 | 19.82 |
| `build_support_allocator` | 682 | 16.66 |
| `build_native_allocator` | 702 | 16.47 |
| `build_ground_realtime_admission` | 754 | 15.49 |
| `build_ground_motion_contact` | 817 | 14.15 |
| `build_catch_world_geometry_single` | 172 | 9.89 |
| `build_catch_no_reverse_brake_single` | 171 | 9.68 |
| `build_ground_native_physics` | 28 | 7.68 |
| `build_thrust_contact_diagnostic` | 27 | 7.52 |
| `build_angular_momentum_physics` | 27 | 7.50 |
| `ros_log` | 4,674 | 1.62 |
| `opt_ros` | 117 | 1.42 |
| `log_ground_motion_contact` | 10 | 1.40 |
| `figures` | 9 | 1.11 |
| `debs` | 5 | 0.38 |
| `archive` | 15 | 0.16 |
| `install_ground_motion_contact` | 18 | 0.06 |
| `LANDING_REPAIR.md` | 1 | 0.03 |
| `.gitignore` | 1 | 0.01 |
| `README.md` | 1 | 0.01 |
| `DEFAULT_FLAT_JUMP.md` | 1 | 0.00 |
| `AGENTS.md` | 1 | 0.00 |
| `setup_env.sh` | 1 | 0.00 |
| `.vscode` | 2 | 0.00 |
| `run_complete_jump_demo.sh` | 1 | 0.00 |
| `.claude` | 2 | 0.00 |

## 功能包与正式依赖

| 功能包 | 从实现/构建入口核实的职责 | V1默认仿真依赖 |
|---|---|---|
| `bbot_balance_controller` | 多种平衡节点、velocity/classic/reference/landing-repair跳跃目标、公共头文件、运行器和审计脚本 | 是；默认velocity目标，不等于包内所有候选均启用 |
| `bbot_bringup` | launch、ros2_control配置、世界、原生只读记录插件 | 是；诊断插件是否启用以实际SDF/试次快照为准 |
| `bbot_description` | xacro/URDF、CAD网格、惯量、关节及接口定义 | 是 |
| `bbot_kinematics` | Kinematics、IK/Jacobian/重力与几何模型 | 是，控制器链接使用 |
| `bbot_motor_driver` | CMake构建CAN接口与motor_driver_node | 硬件路径；默认Gazebo launch不启动 |
| `bbot_rc_receiver` | Python CRSF接收、rc_node安装入口 | 遥控基础功能；默认跳跃不依赖其运行 |
| `bbot_torque_control` | torque_monitor、height_controller及独立launch | 辅助控制/监测，不是velocity跳跃腿命令发布节点 |

`src`里并不只有源码：约35,164 MiB中，`src/bbot_balance_controller/src/data_logs`占绝大部分。`flat_jump_trials`约33,978 MiB，冻结源码、重复独立构建、原生高频帧与派生图混放，是主要体积和导航复杂性来源。另有高度/平衡研究资料，不能全归入跳跃归档。

`experiments/jump/planning`混合协议、未实施设计、离线数值产物及物理试验总结；实时准入、round2和pitch-chain资料体积较大。`docs/jump/reviews`约317 MiB并被忽略，含历史资料，不能把“docs”全视为轻量可上传文本。`.ros`和`ros_log`数量大；多套build/install是独立试次的既有产物，全部保留。`src/bbot_rc_receiver`还含嵌套build/install，统计归于src；不能仅清理根目录来理解重复。

## 完整辅助索引

- [全部跳跃campaign](inventory/CAMPAIGNS.csv)：逐目录列出数量、体积、报告、分析脚本、图、参数/来源/diff线索；历史目录未逐跳重新认证，不猜测采用状态。
- [planning下全部Markdown](inventory/REPORTS.csv)：含被Git忽略的最后两次CATCH报告。
- 小型文本内容相同组（本地资料 `inventory/IDENTICAL_TEXT.json`）：SHA256相同的431组；只覆盖≤2MB的指定文本类型，排除顶层build/install/log/.git及原始数据JSON，不是全盘重复检测。冻结副本/独立失败记录多数必须保留。
- 现有路径引用（本地资料 `inventory/PATH_REFERENCES.csv`）：正式源码、工具及文档中的选定路径字面引用；不执行脚本、排除data_logs和顶层构建目录；部分planning冻结文本仍在搜索范围，未覆盖全部历史动态依赖。不命中字面不能证明无动态依赖。
- [参数声明与实际launch转交](inventory/PARAMETERS.csv)及[关键源码SHA256](inventory/SOURCE_SHA256.json)。

进一步分类、候选清理与精确迁移约束见 [迁移计划](MIGRATION_PLAN.md)。

## 发布范围补充（2026-10-08）

上述统计保留清理前口径；本机全量目录、重复组、状态和路径引用快照不上传GitHub，文件原样保留。公开参数、campaign、报告及迁移索引继续保留；归档与工具迁移后的状态见[发布准备清单](../RELEASE_PREPARATION.md)。
