# 有限支撑恢复轮独立验收（2026-10-07）

本轮结论：**NO_PHYSICAL_ADMISSION，仍不允许受限接地标定或试跳。** 已实现可信前提下的有限支撑恢复路径；静止腿模型 seed 能在 0.75 s 后停止并终止故障试次，不能据此声称运动制动通过。正常接地候选控制律仍受准入拦截，默认 README velocity 跳跃入口未改。按用户追加要求，本轮验收后停止。

问题与通过条件见 [PLANNING.md](PLANNING.md)。本轮实现只针对独立接地候选的所有权、故障恢复和监督，不修改动作速度、仿真倍率、1 ms 物理步、力矩限制或验收门。

## 实现与独立检查

- 持续核对 manager 模式及接口 claims；切换 ACK 后必须取得请求起点晚于 ACK 的新 Effort 独占读回。修复了 switching 阶段被自己的输出保护永久拦住、无法推进到 effort_hold 的缺口。等待、pending、故障仍封锁不合法输出。
- 仅逆解拒绝、求解拒绝／超时等明确故障可申请恢复。每个新原生步重新核对直接 q/v/u、双时钟年龄、同键双轮支撑、接触故障锁存和真实 manager 读回；不可信就请求独立暂停。接触恢复不能清除锁存。
- 恢复首命令由同键实际六输入构造，之后腿逆解与原 COM 捕获轮支撑组成唯一恢复路径。正常 PD／前馈路径在恢复期间封锁。保留 gain=1、轮目标 ±30 rad/s、轮力矩 ±10 Nm、髋／膝 75／60 Nm 及原 8 m/s² 轮目标变化限制。
- 0.5 s 是减速参考时长，另需连续 0.25 s 安静确认，2 s steady 总期限不可重置。安静停止也只表示故障结束，trial 始终 FAIL；不自动恢复动作，也不把暂停算恢复通过。
- 运行时执行 0.04 rad 行程、从方向峰值计的 0.02 rad 回弹、0.05 m 轮轴位移候选边界；这些边界没有变成已验证制动能力。完整输入门、helper 与命令映射的发布前预算超过 1 ms 就禁止该步发布。发布调用及实际传递延迟另记，helper 快不代表完整链实时资格。
- 独立监督按请求起点而非响应收讫计 manager 年龄，拒绝迟到响应；收到恢复终态或故障立即 FAIL_STOP 并请求暂停。runner 收到 FAIL 后停止发动作，并等待监督完成暂停与证据写入，避免清理提前截断暂停。

编译期 ROS 测试变体复用控制器源码，但所有腿／轮输出强制进入测试专用 sink，故障注入仅在该编译宏存在。它不安装、不作物理入口，生产二进制没有该故障注入口。正常候选的 `contact_motion_offline_qualification_pending` 仍直接暂停退出。

## 证据分层

| 层级 | 本轮结果 | 有效范围 |
| --- | --- | --- |
| 独立构建／纯测试 | 两控制器目标构建；8 项针对性 CTest 全通过，其中含 11 项监督器纯单测 | guard、有限状态机、候选模型计算；不等于实际恢复 |
| 固定模型 B | 单个静止腿 seed 在 0.75 s 结束，全部 749 helper 小于 1 ms | 腿几乎静止，不覆盖运动故障协议或其他时延格点 |
| 监督器 fake-manager ROS | attempt3 六场景通过；pause 为 stub，未逐场景冻结当时监督器 SHA | 运行时控制流程记录；随后修复版本未重跑此套，不能用于最终源码整链准入 |
| 同源控制器 ROS | attempt1 缺二进制 BLOCKED；attempt2 fixture 解析错误；修复后的 attempt3 因 manager 新鲜度退出 | 未完成 Effort 接管，未注入恢复故障；不计恢复通过 |
| 真实 Gazebo 空世界 | 最新监督器复验 PASS：服务返回成功，3 个后续 stats 的 iteration 固定为 880 | 没有机器人或执行器，不计接地实跑，也不计支撑恢复 |

同源 controller attempt2 将 ROS sequence 错判为 list/tuple，导致 fixture 回调异常、状态发布停止；原失败保留。修复解析后 attempt3 的 fixture 无回调错误，但控制器仍在请求接管后、切换发起前因新鲜度退出。root 独立核对 manager CSV：关键 request 间隔 **460.009 ms**、receipt 间隔 **457.181 ms**；故障时距前一 receipt 已 **167.941 ms**，超过原 150 ms 新鲜度门。两端读回均是合法 Position claims，也不能消除中间陈旧区间。本轮没有将其误判为模式错误，更没有放宽门制造通过；尚缺调度间断的完整原因和满足限期的接管证据。

另一个独立源码缺口是 ACK 后等待新读回的分支位于立即判 stale 的门之后。最终版本把“尚无请求起点晚于 ACK 的读回”作为**无输出等待**，仍受原 manager 与 switch 限期约束。该修复不解释 attempt3 的切换前失败；最终程序重新构建，但本轮没有再进行 ROS 复验，不能称此改动已集成通过。

最新监督器 SHA 为 `b59974b97e53faea61cbcb78efefbedf65f0ad24bf3d0e483c2b32336a308925`。真实空世界复验首次收到 paused stats 的观测延迟为 272.182 ms；这是一次观测值，不是实际机器人受力终止的保证。世界配置目标倍率 1.0、物理步 1 ms，未调慢仿真。旧版空世界检查 PASS 也保留，不冒充最新版结果。

本地证据：最终 `root/ctest_postack_final.log`、`root/build_variant_audit_postack_final.json`（修复前版本另存，不能混作最终程序）；`supervisor/manager_watch_ros_attempt3/`、`supervisor/late_response_attempt1/`（未覆盖目标 late-done 分支，失败保留）；`supervisor/controller_ros_attempt2/`、`supervisor/controller_ros_attempt3/analysis_note.json`；空世界暂停的两个版本分别保留在 `root/gazebo_empty_pause/` 与 `root/gazebo_empty_pause_final/`。这些本地目录不上 GitHub。

| 故障／连续性项目 | 已有本轮有效检查 | 接管后完整 ROS／物理证据 |
| --- | --- | --- |
| 配对失效、时钟回退 | 复用引擎帧 guard 单测；恢复 steady 回退拒绝 | 未覆盖 |
| mode／claims 丢失、switch pending | 纯 guard 及独立监督 fake-manager ROS | 同源控制器尚未接管 |
| 短暂接触丢失后恢复 | 复用 sticky guard，恢复终态不能清除 | 未覆盖 |
| 逆解拒绝 | 真实 helper 的严格保护拒绝后不返回旧命令单测 | 未注入生产故障链 |
| 求解超时 | fast certificate 极短预算拒绝、恢复总期限单测；B 保留全样本计时 | 未完成整链限期／传递验证 |
| 停止行程、峰值回弹、轴位移 | 原候选界限的越界拒绝单测 | 没有实际制动包络 |
| 交接连续性 | 首六输入代数 Δu=0；B 同键输入／延迟审计 | 没有实际 Physics 接管连续性证据 |

## 模型失败保留与 B 的有效范围

A 首试在故障后 1.645 s 触发 `predicted_pitch_guard`，未达到安静停止。停止轮速趋零不能替代倒立摆的 COM 支撑。root 同时发现 A fixture 使用了错误的轮延迟队列、不同源键实际输入和 5 ms 恢复周期；因此 A 只作失败线索，不能作为完整固定协议证据。原始 CSV、源码、程序和重建头文件的来源说明全部保留，没有重跑 A。其头文件是明确标注的运行后重建副本，不能冒充运行前冻结。

B 在运行前纠正同键 post q/v＋该完成步 before 输入、1 ms 恢复调用、独立 5 ms 轮伺服采样和腿／轮 1 ms 命令队列；COM 速度直接由原生 q/v 与几何雅可比求出，没有外推前序状态。另修高度率旋转符号、首命令后的轮目标变化连续性及方向峰值回弹漏检。只运行一次 B seed，原始 CSV 未修改。

root 对 B 的全部 752 行独立核对：750 次候选命令、749 次完整 helper，零拒绝／超时；helper 最大 407.762 μs、整 step 最大 416.900 μs，故障后 0.750 s 进入 `StoppedAndTerminateFailure`，trial 没有被标 PASS。状态／输入键、1 ms 连续帧、独立延迟及 5 ms 轮力矩保持一致；轮轴最大模型位移 1.796 mm。

**B 四腿初速及全程峰速只有约 4.9×10⁻¹³ rad/s。** 它验证的是该静止腿支撑模型 seed，不是首档低速四腿运动的协调制动；没有覆盖完整运动故障、不同状态年龄与命令延迟范围。模型 contact／ownership 也是假设，不能替代实际 ROS 或物理读回。

原 CSV 的 `published_leg*` 表示最后生成命令，终止行不表示又发布一次；终止行的 `applied_u*` 只是保留的旧输入，后面没有再积分物理步。派生图必须遮掉终态命令并标注 MODEL ONLY。不能拿该行声称无限保持输出。

root 从冻结 B CSV 另算 751 个已执行模型步的摩擦／黏性项，得到 `command + model friction − model viscous` 三类扭矩图；没有重跑候选控制律。文件为本地资料 `root/B_model_aggregate_derived.csv`、`root/B_model_torque_three_series.png`，图已逐项标注 MODEL ONLY。模型合负载是计算量，不能冒充真实传递力矩；本轮没有新增真实髋膝物理曲线，原实际三类曲线保留原位置。

模型资料（本地资料）：`solver/A_README.md`、`solver/A_MINIMAL_MODEL_RESULT.json`、`solver/A_MANIFEST.json`；`solver/B_README.md`、`solver/B_MINIMAL_MODEL_RESULT.json`、`solver/B_MANIFEST.json`；root 的 `B_model_independent_audit.json`。路径均相对本目录；原始数据及冻结文件不上 GitHub。

## 仍缺的准入证据

1. 运动中恢复与退出的完整固定协议、状态年龄／命令延迟范围及独立复验；静止 seed 或测试注入成功不能替代。恢复 helper 目前以故障瞬间 q[2] 为 pitch anchor，还须核对它与原 trial 姿态锚点的共同限制，不能由重新冻结锚点扩大原姿态包络。
2. 接管后的配对失效、时钟回退、模式丢失／pending、接触丢失后恢复、逆解拒绝、超时、停止越界和交接连续性的完整同源 ROS 故障链。
3. 真实六输入在轮伺服 5 ms 调度及 ROS 传递后的交接／连续性证据；本轮首命令 Δu=0 是代数与模型检查，不是实际 Physics 输入连续性结论。
4. 站立、下蹲、伸展实际运动的九自由度响应、双髋有效激励、停止包络及独立复验。上一条实际输入／直接引擎状态与静止模型 PASS 保留，首档运动全协议仍 FAIL。

本轮新增机器人／接地实跑 **0**、腾空 **0**、跳跃达标 **0**；空世界暂停检查单列，不算接地实跑。接地修复线累计仍 **4／0／0**，跳跃／首触／落地后退指标保持无效，可靠历史跳跃对照另列。本轮未建立实际制动包络，没有接正常候选输出，没有提交或推送。

连续两轮没有取得实际运动阶段晋级，本轮停止参数修改与后续运行。恢复前须先复核本轮 manager 调度间断及运动状态下的控制／物理残差，不能直接再调跳跃增益或反复实跑。

## 复现与冻结

独立构建目录为本地资料 `build_ground_finite_recovery/bbot_balance_controller/`。源码检查、编译、单测、模型、假传感器 ROS 和真实空世界暂停分别留档，不能合并宣称物理恢复通过。最终二进制、模型与参数、数据哈希和保留核对在本地资料 `final_manifest.json`、`acceptance.json` 中记录。

最终 controller cpp SHA 为 `eb6d9e84d03ff82e88f9b1a6e71ef760020663cf322d04db710ddf91c44c2ea1`，生产候选程序 SHA 为 `8eb81a0cf36f52953b25ad42c674cab24edefe386371d112c53dd36c1f96ef67`，测试变体 SHA 为 `96c1fcb63ab9ac08e676e9f51fe083099ca86b93548cd15f01fff82d0bbbe4c7`。ROS attempt3 使用的是修复 ACK 等待之前的程序，绑定记录及旧源码／程序另存 `root/pre_postack_wait_fix/`，不能把最终构建当成已经重新通过 ROS。

root 收尾哈希核对：默认／完整跳跃／MPC 36 项、上一实时轮 808 项、旧实际运动 225 项、资格审计 108 项及 round2 184 项均未改变。原始报告与失败全部保留；最终源码、配置、模型、程序及证据副本冻结到 `final_freeze/`，不安装、不覆盖默认程序。

针对性测试复现：从工作区环境进入独立构建目录，以 CTest 匹配 `ground_motion_probe|ground_engine_state|ground_runtime_safety|ground_contact_motion_fast_certificate|ground_runtime_axle_bridge|ground_motion_supervisor|ground_contact_motion_control|ground_contact_finite_recovery`。模型与 ROS 运行必须使用新的证据目录，不能覆盖原记录。ROS 测试入口为 [同源控制器无运动 fixture](../../integration/test_ground_recovery_controller_ros.py) 和 [监督器 fake-manager fixture](../../integration/test_ground_motion_supervisor_ros_fake.py)；它们明确不代表机器人物理验收。
