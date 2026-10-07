# 低速接地候选：实时接入与一次实跑准入审查

审查范围是当前 ground_contact_motion_experiment 入口、ECS1 到控制器的实时路径和 run_ground_contact_motion_trial.py。这是只读源码／现有记录审查；没有编译、启动 ROS/Gazebo 或修改现有数据。这里不把源代码分支、离线 fixture 或历史旧控制律的物理记录当作新候选的实跑证据。

## 结论

当前候选还不能做一次低速接地运动实跑。最直接的阻断是控制器仍在 publish_ground_motion_height_effort() 对 ground_contact_motion_experiment 无条件 fail_ground_motion("contact_motion_offline_qualification_pending") 并锁输出；运行时 ground_contact_motion_control 尚未接到输出路径。runner 虽要求 offline_admission.json PASS，但这个 admission 文件只放行 runner，不解除控制器内部阻断。相应的 round-1 接受记录仍是 FAIL、静摩擦交接 alpha=0/.25/.5 被 helper 拒绝；当前 round-2 计划也明确先解静摩擦交接与真实状态相容性。必须先有同一冻结源码／profile 上的离线 PASS 和受审的实时接入改动，才能考虑消耗那唯一一次 campaign。

另一个重要实时缺口是 ECS1 消费端允许单调但跳步的 DDS 消息进入历史。CSV 桥在发布前能检测其源文件的步缺失，但桥到控制器使用 best-effort DDS；如果队列拥塞丢了一个或多个 ECS1 包，NativeCommandStateHistory::push() 只要求 stamp/iteration 递增，并未要求恰好相邻 1 ms／iteration。这样收件端无法区分正常相邻消息和 transport 丢包。必须在消费端对消息键连续性硬拒绝或有等效、经测试的缺帧锁存。

## 接入时钟、样本与唯一输出

| 环节 | 当前实现与范围 | 一次实跑需要留证的项目 |
| --- | --- | --- |
| 直接引擎输入 | ground_engine_state_bridge.py 将 direct Physics before/after 状态与 native before_physics_joint_force_cmd_sim_input 组成 ECS1。帧 key 是同一 sim_ns/iteration/dt=1ms；q/v 是 after-step，before_v 与六输入是 before-step。bridge 先等两源越过 frame key，再发布，增加至少一个物理步加文件刷新／poll 延时。 | 日志必须记录控制 tick 实际选中的 ECS1 key、sim-age、steady receipt age、q/v/before-v/u 六输入及同 key contact；当前 .engine_source.csv 记录每个回调收到的原始包与 steady receipt，但 motion trace 没有记录每 tick 实际选中的 ECS1 key/数值。仅有原始输入文件不证明控制器在那次发布前已消费了哪一包。 |
| 1 kHz 到 200 Hz | bridge publisher 使用深度 10 的默认可靠 QoS；controller 的 ECS1 subscriber 是 best-effort、深度 128。控制循环 5 ms 一次。C++ 以仿真 key 不超过当前 control sim time、sim age ≤10 ms，并要求所选包的 steady receipt age ≤10 ms；history 容量 64。 | 这给出拒绝上限，不保证每个 physics sample 均送达或被控制回调观察。记录 DDS 丢包／step gap 计数、最大 sim 与 steady age、每周期所选 key，验证真实机器上不会靠旧包维持控制。 |
| transport continuity | GroundEngineStateHistory 验证 receipt steady 不回退，并通过底层 history 拒绝重复/回退；底层 NativeCommandStateHistory 未拒绝正向跳步。bridge 源文件连续性检查不能证明 best-effort topic 的交付连续性。 | 加消费者端严格 iteration+1, stamp+1ms 连续门（或等价的不可恢复 gap latch）并用 ROS 层注入丢包验证；只测 CSV assembler 不覆盖 DDS 队列行为。 |
| contact 配对 | motion tick 用所选 ECS1 的 stamp_ns/iteration 查 contact history，要求同 key、双轮 mask 0x3；运动 guard 还要求 contact snapshot 连续。contact history freshness 为 20 ms，ECS1 sim/receipt 门为 10 ms。 | contact callback 当前仅 push history；motion tick 只查看最新可选样本。若某个 1 ms frame 单轮／零接触后，在 5 ms 控制 tick 前又恢复，控制 tick 可能只看见恢复后的最新双支撑。对承诺“任何支撑丢失锁存”，需要在 1 kHz contact callback 对 motion-active 期间的坏 mask/无效 frame/sequence gap 立即 sticky-fail；并以 callback 事件和原始 contact frame 验证。 |
| 引擎实际输入 vs 发布 | ECS1 的六路 u 是 Physics 前观察到的 JointForceCmd 输入值；它不是电机实测。命令日志含 command_id、仿真 publish/control bounds 和 steady publish bounds，但 ForceCmd 没有 command ID，native CSV 也没有每次物理消费的 steady wall stamp。重复扭矩值不能唯一证明是哪条发布生效。 | 以 iteration/仿真 key 和 wall 因果范围比对 leg pub、wheel-servo pub 与 actual ForceCmd；报告候选区间、wall 差和重复值歧义。保持实际 ForceCmd 为门控输入，不能用命令目标替代或把相同值命中称作零延迟／精确 source provenance。 |
| 控制器独占 | launch 开启 allocator-off、ground input/motion/contact flags；runner 在进入 Effort 后查询一次 list_controllers 并要求 wheel effort active、diff drive inactive、leg effort active、position inactive。wheel servo 是独立进程，gain=1、目标限幅 30 rad/s、torque ±10 Nm，更新 200 Hz；其自身 command/joint stale 限期是 50 ms。 | runner 的 motion 循环依赖 controller 自报 effort_mode_active 和 switch_pending，没有在整段运动期间轮询 controller manager；如果 controller 被外部停用而内部 flag 未同步，不能仅靠此字段证明 ForceCmd 消费者仍 active。记录／监视整个 motion interval 的 manager 状态和真实唯一 publisher，并审查 wheel servo 的 invalid/stale/zero 行及每步实际轮 ForceCmd。 |
| 唯一腿输出 | motion publisher 走 controller 内现有 leg_effort_pub_，publish_recorded_command() 为腿输出分配单调 command id 和 publish wall/sim bounds；effort/pending gate 防止 pending 时发腿命令。候选 helper 尚未接入，故“helper 是最终唯一腿力矩源”目前并未实际成立。 | 接入后证明 helper 的最终 torque 走唯一 active leg Effort controller、旧 ground-PD/allocator 不并行接管；记录 helper 输入 key、solver result/reason、最终限幅命令 ID、Physics 实际 u。运动中检查 manager 独占而非仅入场时检查。 |

ECS1 里 q/v 与 u 共用 step key，但有不同物理相位：q/v 是该步 after 状态，u 是 before-Physics 读到的该步输入。若审计“该输入造成的该步响应”，需要使用 direct before q/v（engine 原始 CSV 有 before q；ECS1 当前仅有 before_v）或经完整相邻步连续性证明的 prior post q，不能把同键 post q/v 和该步 before-u 直接说成同相位状态转移。若控制器把 post q/v 作为下一条 command 的当前状态，则要在 motion trace 记录该确切 sample 与 command publication 的关系。

## 故障场景：代码检查与运行证据的区分

| 场景 | 当前证据 | 实际尚未证明的部分 |
| --- | --- | --- |
| 模式 pending / Effort 丢失 | 模式 guard、controller manager 启停路径、pending 阻止腿发布可在源码中检查；runner 入场与 Effort hold 查询 controller manager。C++ 测试覆盖 helper 输入 pending 时拒绝。 | 没有实际 ROS switch rejection、ack timeout、motion 中外部 deactivate 的端到端注入。当前内部 effort_mode_active_ 由 switch service callback 更新，不是 manager 状态流。 |
| ECS1 缺帧、NaN、回退 | parser／history unit test 有错误 magic、mask、NaN、重复及 rollback fixture；bridge 有源 CSV gap 检测；controller callback 对 parse/push 失败清 history，并在 motion active 时 fail。 | 没有实时 DDS 乱序／丢包／队列拥塞／steady TTL 超时的 ROS 注入；正向 transport gap 当前还会被 history 接受。 |
| 接触同 key 无效 / 失去双支撑 | 源码选相同 sim stamp 和 iteration、要求 wheel_mask==3、motion guard 要 continuous；协议离线 auditor 可逐物理步看 contact。 | 没有实时 contact topic 中断、损坏 frame、单轮／零接触与恢复的 callback 注入；当前 200 Hz latest-snapshot 可能漏看 1 ms 短暂失支撑。 |
| 逆解拒绝／物理模型拒绝 | offline C++ helper 单点、约束残差、力矩限值、静摩擦模式与特定拒绝条件可通过固定 fixture 检查。已有 round-1 全精度真实状态反例显示静摩擦交接 alpha=0/.25/.5 被拒，且实际真实状态不可投影为理想对称值。 | helper 仍未接实时输出，因此没有控制状态下的拒绝→hold 实测。现 helper 对 q/v/reference 有严格对称分支要求；须用原始完整精度 anchor 验证，不能把左右状态平均后称可达。 |
| solver timeout / control overrun | 可从 source 看同步固定规模线性代数求解；现有控制 log 有通用 control timing 字段。 | 没有本 helper 的单次／最坏耗时、deadline gate、timeout 故障输出测试，也没有证明 5 ms tick 内的最坏预算。候选应记录 solver wall time 和失败 reason，并设置独立于 physics/profile 的 hard deadline。 |
| IK refusal | 初始 BALANCE 路径调用站立 IK；运动参考本身是四关节空间参考，不是每周期 body IK。 | 没有新候选的运行 IK success/valid/soft-limit admission。需要在发起 motion 前验证初始姿态与所用模型构型合法；不要把关节空间模型通过描述为任意高度／pitch 可达。 |
| 停止距离／故障保护 | motion guard 有全程单步安全边界：joint anchor ±0.08 rad、leg rate ±0.10 rad/s、pitch anchor ±0.10 rad、body rate ±0.5 rad/s、soft-limit margin ≥0.30 rad；正常 stop 要四腿 ≤0.005 rad/s 连续 0.25 s，超 2 s fail。auditor 离线检查全程 joint excursion ≤0.04 rad、rebound ≤0.02 rad、axle ≤0.05 m。 | 0.04/0.02/0.05 的停止包络目前是事后审计门，不是实时超包络中止门；online 0.08 rad anchor 比停止 displacement allowance 大。故障锁输出将轮 command 置零、在 Effort active 时保持最后有限腿 effort 并限幅至 75/60 Nm；这属于 bounded hold，不是卸载或自动恢复。当前没有实测证明该故障策略在接触丢失时安全，也不能称 unload-recover。 |
| 交接连续性 | 固定参考/phase 事件和 2 s support-law blend 有源码与离线边界测试；测试包含固定 trajectory C2 边界。协议/runner 写入 event、command id 和发布前后时间。 | 没有新 helper 从 alpha=0 到 1 的实时腿输出连续性／ForceCmd 生效实测。已有冻结全精度锚点表明静摩擦反力非零，round-1 helper 在 alpha<1 拒绝；需 round-2 离线 PASS 覆盖交接全域和真实不对称 anchor 后再考虑实跑。 |

lock_owned_outputs() 故障时确实轮速请求零；如果腿 Effort 仍 active 则重发最后 finite 的有界支撑力矩，若 pending 或不 active 则不发腿 Effort。这避免无效传感器导致无限增长的控制输出，但会保留旧支撑命令；它只证明源码中的 fail-closed/bounded-hold 意图，不能等同于卸力、站回安全构型或现场恢复。本次没有运行时 fault injection 证明。

## offline admission、哈希与隔离构建

runner 当前要求输出目录内 offline_admission.json，并核对 gate == PASS、profile SHA 相等及其 frozen_inputs 每个条目的当前 SHA。必须列入的路径目前只有 controller binary、controller .cpp、ground_contact_motion_control.hpp 和 ECS1 bridge。runner 随后另建 tested_manifest.json／frozen_run_files，覆盖更广源码、模型/xacro、observer libraries、physics plugin、world、servo、protocol 和 runner 自身；该 snapshot 只证明这次启动快照未损坏，不能反向证明这些文件就是离线 candidate 用过的版本。

允许一次实跑前，应让 admission 的 required bindings 与 tested_manifest 的实际动力学依赖一致：至少包含规范化 protocol/profile 文件及其可信 SHA、controller binary、对应所有源码/头文件、model/URDF/xacro/kinematics、native Physics plugin、contact/native observer libraries、world/config/launch、servo、bridge、runner 和 audit/model gate 程序。拒绝未知/漏绑定文件，profile hash 应由 runner 或独立审查工具从规范化配置重算；独立构建须保持 build_ground_motion_contact/，observer 继续固定 build_ground_input/bbot_bringup/，native Physics plugin 显式指定并一起哈希。任何候选／头文件／plugin 改动都应重建并重新生成 admission，不能复用先前 gate。

## 一次 low-speed campaign 的最小前置与验收

1. **不要现在启动 movement。** 当前源码 pending gate 与旧 qualification FAIL 均阻止接入。先由 offline candidate 覆盖静止交接各 alpha、真实原始不对称 q/v、正反运动/停止全 9 DOF、接触锥与输入 caps；禁止投影状态或更改低速参考/原限值。
2. 解除 pending gate 时只允许将审过的 helper 接到唯一 Effort output 路径；对 invalid、contact/model rejection、nonfinite、solver 超时或 mode lost 一律 sticky fail，不能退回旧 PD／allocator。确定 bounded-hold 的安全适用范围，不能把它写成 unload-recover。
3. 冻结新的独立 Release build 和 exact offline_admission.json。必须比对 binary/profile/model/plugin/controller/bridge/runner/auditor 的 SHA；保留独立目录，不能替换 default/MPC 入口。
4. 做无运动的实时入场检查：Physics=1 ms、RTF=1；确认全帧 ECS1 q/v/before-v/u 满 mask、sim age 与 steady receipt age 均 ≤10 ms，contact 对同 iteration、同 sim time 且双轮连续，实际 manager 独占，wheel P=1 target/torque cap 生效。测出 callback/send/receipt/control/publish 到 Physics before-input 的时序与最大值；将 publisher/input 数值重复歧义分开报告。
5. 在同一次 runner campaign 中只做 protocol 固定的 standing-neighborhood 四段 trial（trial1/2 与 independent replay trial3/4，每段 acceleration/cruise/normal stop），不增速度、不加主动 wheel pulse、不换构型。motion 中记录并锁存 mode/contact/ECS1 gap/clock/solver failure；控制器每 tick 保护与 controller manager 监视全程开启。无需为证明代码分支而故意在唯一物理运动中注入危险故障。
6. run 完后先运行全帧审计，不把 runner 返回 MOTION_RECORDED_AWAITING_OFFLINE_MODEL_AND_STOP_AUDIT 当 PASS：所有输入/状态/接触帧连续有效、原 9 维每工况绝对+相对响应 gate 通过、每段至少 30 个完整 physics step、四腿达到激发步数且 stop dwell/timeout/位移/回弹/axle 包络全部通过。任何失败记录保留，campaign 总体 FAIL，不立刻重跑；本轮资格只限 protocol 所述 standing-neighborhood 低速域，不外推刹停速度、其他构型、主动轮速或跳跃。

## 运行证据分类

当前 round acceptance 记录 physical_runs_this_round=0、new_entry_standing_or_mode_handoff_ROS_validation=NOT_RUN、end_to_end_ROS_fault_injection=NOT_TESTED、four_trial_physical_campaign=NOT_RUN。已经存在的 bridge/helper/model/guard 测试是人工 fixture、离线数值或 source-path checks；round-1 的实际接地动作属于旧 helper／旧输出 law，不能为尚未连接的新候选背书。现阶段没有新候选的 standing handoff、9 维实时闭环、停止包络或故障恢复实证。

## 可复查位置

- ECS1 callback、steady receipt、parse/history reject：src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:826-850；wire parser / receipt history：src/bbot_balance_controller/include/bbot_balance_controller/ground_engine_state.hpp:10-55；underlying history的 10 ms sim TTL 与仅单调 push：src/bbot_balance_controller/include/bbot_balance_controller/native_command_state.hpp:89-120。
- 同 iteration contact selection、freshness/contact continuous和实时运动 guard：src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:2457-2478,2532-2574；history 的连续性定义及 20 ms freshness：src/bbot_balance_controller/include/bbot_balance_controller/allocator_contact_history.hpp:24-83。
- 2 s support blend、正常停止/0.25 s dwell/2 s timeout/激发门：src/bbot_balance_controller/src/bbot_landing_repair_controller.cpp:2577-2695；触发之后仍未接 helper 的 fail-closed输出：同文件:2800-2808、3460-3475。
- 故障锁定、保持 last-finite leg torque、轮速零：同文件:2932-2968；motion trace 当前记录的 contact/joint/imu/odom/command time：同文件:2736-2797。
- launcher QoS/action ordering：experiments/jump/ground_contact_motion.launch.py；one-shot runner admission/hash/build/source snapshot：experiments/jump/run_ground_contact_motion_trial.py:129-220；运动流程与 per-sample保护：同文件:285-409。
- wheel servo caps与50 ms source-age：src/bbot_balance_controller/scripts/wheel_effort_velocity_servo.py:20-39,127-175；round-1 FAIL、当前实时ROS/fault状态NOT_TESTED：experiments/jump/planning/contact_motion_qualification_20261006/acceptance.json 和 REVIEW.md。
