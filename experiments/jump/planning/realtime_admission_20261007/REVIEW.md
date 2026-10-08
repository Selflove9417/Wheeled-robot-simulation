# 首轮实时准入验收（2026-10-07）

**结论：NO_PHYSICAL_ADMISSION；阶段二未完成，不能进入受限接地标定或试跳。** 本轮实现和测试有推进，但可信状态下的必要支撑协调减速没有可执行恢复路径，实际接管后的完整故障链及真实 Gazebo 暂停未验收。候选控制律仍受 `contact_motion_offline_qualification_pending` 拦截，未接到腿输出。按本轮要求，验收后停止。

本轮新增 **0 实跑／0 腾空／0 达标**；接地修复线累计 **4／0／0**。跳跃与落地指标保持无效。历史跳跃对照另列：真实净空 15.34 cm、首触后倾 5.74°、轮轴后退 40.68 cm，本轮没有产生新的跳跃改善证据。

## 问题、范围和通过条件

问题与原定通过条件见 [PLANNING.md](PLANNING.md)。以 [上一轮入口补充审查](../contact_motion_revision_20261006_round2/REALTIME_ENTRY_ADDENDUM.md) 和实际源码为接续基线。初始分支 `work/jump-repair`，工作树干净。只改独立 contact-motion opt-in 路径、证书、观测桥、监督器及针对性测试；默认 `jump_velocity`、完整跳跃、MPC、世界、物理限幅和验收门不变，无提交或推送。

| 本轮准入项 | 独立检查结果 | 尚缺证据 |
| --- | --- | --- |
| 持续真实模式、接口所有权、新鲜度 | 已实现；单测与候选等待分支 ROS 检查通过 | 实际接管后的全链 ROS／物理验证 |
| 可信条件下有限必要支撑协调减速 | **FAIL，未实现可执行路径** | 恢复控制、终点、退出交接及故障覆盖 |
| 不可信时独立暂停终止 | 独立 ROS 控制流及暂停请求 stub 通过 | 真实 Gazebo 暂停确认、终止时序与物理效果 |
| 停止行程、回弹、轮轴位移实时保护 | 源码与单测通过，逐原生步接入 | 接管后注入、真实响应和完整实测包络 |
| 完整固定协议、1 ms 计算期限 | **16 格模型检查通过** | 实际闭环调度与端到端延迟上界 |
| 唯一腿输出与连续交接 | 硬拦截保留；离线最终命令证书一致 | 新控制律未接输出，实际唯一输出交接未验收 |

机器结论为本地资料：`experiments/jump/planning/realtime_admission_20261007/acceptance.json`。runner 必须检查八项 `runtime_checks` 均为字面值 `PASS` 并核对冻结输入；本轮 FAIL 文件已验证在启动前拒绝，返回 2，没有创建 launch 命令或启动物理过程。不能将 `PASS_MODEL_ONLY` 当作 `PASS`。

## 已落实的运行时改动

- 控制器每 20 ms 墙钟异步查询真实 `ListControllers`；检查 active/inactive、完整接口集合、重复控制器／接口及任何外来控制器对六个执行器的 command claim，包括非预期的 velocity/position 接口。单次回复 RTT 上限 100 ms，状态年龄从请求发起算、上限 150 ms，未完成请求另有 250 ms 硬界限。启动先等合法 Position＋轮 Effort 所有权，10 s 无确认则故障。切换 ACK 后必须取得请求发起时间晚于 ACK 的新 readback；切换待完成时不发布腿轮命令。
- 故障锁存后阻断新腿轮输出，立即并每 100 ms 重发 `PAUSE_ABORT`，不无限重发最后腿力矩；manager 查询和诊断在故障后继续。**执行器可能继续保持前一命令直到暂停，这尚无已验证的有限支撑恢复能力。** 现有 `GroundFailureAction` 仅允许 PauseAbort，即使观测条件均可信也不能伪称恢复通过。
- 独立监督进程直接查询 manager，监视 runner 阶段心跳和控制器故障。使用墙钟 100 ms 查询、600 ms manager 期限、1 s 阶段心跳期限、3 s 切换期限；这些是第二道保护，不能称为 1 ms 故障反应保证。故障时请求 world pause 并终止；暂停单列，不能计为恢复。
- ECS2 在原有 ECS1 状态链外增加同键、直接引擎 after_step 的左右轮轴 world Y（模型前进轴）位置；缺失保持 invalid/NaN。复用已存在的连续帧检查和接触故障锁存，没有重写旧 guard。停止检查逐原生步配对同 iteration/time 的双轮接触；起点用正常停止命令时连续前一步 after_state 对应的 pre_state，参考时钟不再从老状态起算。按真实初速方向跟踪峰值和回弹，沿用 .04 rad 行程、.02 rad 回弹、.05 m 轮轴及 2 s 期限。这些仅为候选跳闸边界，未形成制动能力证明。
- launch/runner 可显式选本轮独立程序，增加监督 READY 门及源码、程序、模型、参数、协议和观测库的冻结绑定。新控制律准入拦截仍在，未并行叠加旧 PD、前馈或分配器。

## 求解器与完整模型协议

原逐摩擦模式动态矩阵＋完整 SVD 是主要开销；改为固定 17×17 系统，满秩模式 pivoted LU、秩亏模式仍用 SVD。保留边界闭包、歧义拒绝、原残差／摩擦／力矩／速度限制，完整 helper 仍守 **1000 μs** 硬截止，包括建模、逆解、PD、blend 和最终命令证书；没有放宽期限、减速动作或改变仿真倍率。

root 独立复核原 300 个扰动：**296 接受、4 拒绝**，四个拒绝也被完整 81 模式参考判定为 `no_consistent_friction_mode`；零错误接受、零模式／加速度不一致，最大加速度差 `8.41055e−12`。边界测试通过。早期冻结诊断曾将填充系统 rank 记为 17/15，当前源码已扣除滑动固定力矩维数，物理 rank 诊断修正；早期输出保留。

完整原固定协议包含 2 s 交接和四个同档正反向／重复试次；每次加速、巡航、减速均 0.5 s，并验证真实模型停稳和驻留。状态年龄 **1/3/4/10 ms × 腿命令延迟 1/5 ms × 轮命令延迟 1/5 ms**，16 格合计 **148,400 模型步、29,664 次完整 helper 调用**，零 helper 拒绝／超时，最大 **544.014 μs**。root 独立核对所有控制行的源状态、时钟和最终 blend 算术，最大差均为零。

没有只展示接受样本：正式首次 16 格中 age10 的四格在调用 helper 前因缺少起跑前历史而失败，全部保留。旧锚点 q/v 实际对应 18.566 s，却被旧文件标为 18.5635 s；root 从原生数据导出 18.556–18.566 s 的 11 条连续 q/v/实际轮输入，校正 source 时间、数值差为零。仅补做这四格，没有重跑前 12 格。补测最大 531.533 μs。更早计时／列顺序不完整的预跑弃用：root 数到 6 个完整结果和 1 个部分结果，原状态文件记 5 的差异另作 correction sidecar，未改旧文件。

**范围限制：**这是 full81 模型 plant 的闭环重放，不是 Gazebo 实跑、ROS 端到端资格或独立物理复验。COM 观察器起始差分仍使用 `q_previous=q−0.020v` 的模型种子，不是完整原生 COM 状态链；离散 16 格和当前机器耗时不证明所有状态／连续延迟范围或最坏执行时间。无 command ID 的旧 ForceCmd 匹配不能提供传播延迟保证；旧 13.834877 ms 腿／9.734014 ms 轮墙钟保持年龄也不能当作命令传播延迟。

本地资料（均不上传原数据）：

- `experiments/jump/planning/realtime_admission_20261007/root/fast_independent/`：300 扰动、完整参考、边界核对。
- `.../solver/fast_grid_formal/`、`.../solver/age10_fixture_correction/`：所有正式通过、首次四个失败及补测原始 CSV／逐调用计时／manifest。
- `.../root/fast_grid_with_prehistory_crosscheck.json`、`.../root/exact_clock_fixture/`：root 独立时钟与源数据审计。
- `.../solver/grid_replay_1/`、`.../root/discarded_prerun_count_correction.json`：弃用预跑及数量纠正。

## 单测和无运动 ROS 检查

独立构建 `build_ground_realtime_admission/bbot_balance_controller/` 成功；7 项针对性 CTest 均通过：motion_probe、engine_state、runtime_safety、fast_certificate、runtime_axle_bridge、motion_supervisor、contact_motion_control。覆盖源配对／缺帧／回退／陈旧、短暂接触丢失后恢复仍锁存、模式与 claim 冲突、候选停止越界、逆解拒绝、计算截止及交接 blend；这些不等于完整 ROS 故障注入。

实际候选程序＋fake manager、没有 Gazebo／clock／运动状态：先等五个健康真实服务回复，再分别注入双腿控制器 active、外来同接口 claim、外来 velocity claim、轮 Effort 丢失、延迟回复五场景，均发出 PAUSE_ABORT，零腿／轮目标发布。最终五场景观察故障通知延迟约 **13.5–151.8 ms**，只是该次观测，不是保证上界。故障后继续读 manager 已复核。最后仅改 ECS2 日志重复列后，最终程序补查一个场景：5 个合成 ECS2 包均有效、日志 47 列且无重复、故障后 6 个 readback、零输出。合成包仅检查 ROS 日志格式，不能当作物理输入。

独立监督器的每个场景使用独立 worker／ROS domain／fake manager，健康 READY 后才注入：pending→Effort→故障、所有权不符、claim 缺失、请求超时、状态陈旧、显式故障，**6/6** 命中预期原因、各发出一次暂停请求并退出。pending 场景观察到 Effort 实际服务快照，不能推导候选控制器已完成 Effort 接管。

**暂停服务均为 stub，没有真实世界。** 早先 fixture 共用回调／发现不足导致错误原因或未退出：root 的 `supervisor_reviewed` FAIL、隔离首轮 `supervisor_isolated_recheck1` FAIL 均保留；隔离与健康前置条件修正后的 `supervisor_isolated_recheck2` 为当前有效 ROS 控制流结果。没有通过放宽生产期限获得通过。早期候选缺参数、DDS sandbox 失败、延迟场景夹具阻塞等尝试也保留，未计为有效通过。

仍缺：**候选完整接管后**对配对失效、时钟回退、模式丢失／切换待完成、接触丢失后恢复、逆解拒绝、求解超时、停止距离越界和交接连续性的联合 ROS 注入，以及真实暂停确认。不能把辅助函数单测或独立监督器的阶段消息当作上述全部通过。

本地资料：`.../root/ctest_frozen_candidate.log`、`.../root/controller_ros_final/`、`.../root/controller_ros_schema_final/`、`.../root/supervisor_isolated_recheck2/` 及其保留的失败目录。`.../root/runner_refusal_audit.json` 证明本轮准入失败在启动前拒绝。

## 实际物理数据独立复核

只读复核已有 `ground_motion_probe_20261006_134820` 的加速／巡航／停止 **1752 步**：10,512 条六路实际输入、31,536 条直接引擎实体状态，q/v、ForceCmd、轮轴逐项差零、无无效步；ECS2 桥回放 1752 包的轮轴差零。双髋 **0 个有效激发步**，左右膝各 484 步，整体协议仍 FAIL。停止 751 个含端点状态、0.75 s，膝行程 0.007517642 rad、轮轴 1.198903 mm；不能外推髋制动、其他构型／速度或完整包络。

旧加速／巡航／停止命令的值因果延迟区间分别 3–4／1–2／2–3 ms；故障处 −1–0 ms 是歧义匹配，保留并判无效，不作为延迟数据。没有 command ID，不能将这些区间当保证。

实际三类髋膝扭矩曲线继续使用原记录中的发布命令／物理步实际输入／关节合负载，均为本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_motion_probe_20261006_134820/plots/three_torque_types.png`。本轮另交模型 inverse FF／implicit PD／final command 图：`experiments/jump/planning/realtime_admission_20261007/root/model_torque_components.png`，图面明确 **MODEL ONLY**，不是实际三类扭矩或新物理证据。

## 冻结、保留和复现

本地冻结与 manifest：`experiments/jump/planning/realtime_admission_20261007/frozen_final/`、`final_manifest.json`。程序、源码、参数、模型与测试数据绑定；模型矩阵另有各自运行 manifest，不声称不同日期二进制相同。独立验收代码、失败日志及原始 CSV 全部留在本轮目录。

正式模型 runner 在补 age10 前没有单独保存源码，收尾按新增历史 loader 的增量逆向重建了独立副本；SHA 精确匹配正式 manifest 的 `88e9a9dd…`。`solver/fast_grid_formal/SOURCE_RECONSTRUCTION.json` 明确记为执行后重建，不能冒称执行前归档。当前 age10 源码 `96e73a7c…` 及正式／补测程序均保留，未为此重跑模型。

本轮前后检查默认／完整跳跃／MPC 等 36 项哈希不变；旧实际轮次 225 文件、前轮 qualification 108 文件、round2 184 文件均未改且无新增。README 和 DEFAULT_FLAT_JUMP 在初始干净 checkout 已与旧 round2 manifest 不同；保留初始会话版本，没有回滚历史文档纠正。证据为本地 `root/preservation_before.json`／`root/preservation_after.json`。

从工作区根复现构建与针对性检查：

```bash
source setup_env.sh
cmake -S src/bbot_balance_controller -B build_ground_realtime_admission/bbot_balance_controller -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build_ground_realtime_admission/bbot_balance_controller --target bbot_landing_repair_controller test_ground_runtime_safety test_ground_engine_state test_ground_contact_motion_fast_certificate test_ground_contact_motion_control test_ground_motion_probe -j2
ctest --test-dir build_ground_realtime_admission/bbot_balance_controller --output-on-failure -R 'ground_motion_probe|ground_engine_state|ground_runtime_safety|ground_contact_motion_fast_certificate|ground_runtime_axle_bridge|ground_motion_supervisor|ground_contact_motion_control'
```

无运动 ROS 监督检查使用 `experiments/jump/integration/test_ground_motion_supervisor_ros_fake.py <新的输出目录>`；候选等待分支、实际数据只读审计、模型源时钟审计的脚本及输入在本地 `root/`／`solver/`。每次使用新输出目录，保留所有失败；不得把复现测试升级为物理运动。

下一轮应先实现并审查**有界、必要支撑协调减速＋终点＋唯一输出交接**，再补完整接管故障链和实际暂停验证；全部通过后才讨论一次固定协议的受限接地标定。当前不得开展跳跃调参、提高限制或降低标准。
