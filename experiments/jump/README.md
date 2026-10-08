# 跳跃与接地实验入口

2026-10-08 [THRUST输入权限验证：匹配前置条件不足后停止](planning/thrust_input_authority_20261008/REVIEW.md)：既有B1/B2/B3/T1的72个同戳样本，传感器七刚体H重建最大误差5.57e-5kg·m²/s；2–5ms状态年龄使误当当前H的最大差值达0.115，在线完整性未合格。现有启动/站稳机制无完整物理及控制器状态匹配能力，因此不运行扰动分支，B、竖直零空间及响应有效支撑域均未验证，停止二输入协同方案、不进入主动原型。新增实跑0、未改控制/参数、155项冻结一致，默认0.25/0/0.78/1.60保持，无提交/推送。

2026-10-08 [THRUST竖直速度/离地H最小协同设计](planning/thrust_vz_momentum_design_20261008/DESIGN.md)：设计候选保留原轮速度路径，用原髋姿态补偿及膝Fz请求做有界二输入解析协同，保留原限幅/释放/保护及唯一腿输出。控制目标为系统总H，不用机身rate代替；接触真值仅验证。已有闭环数据未识别接触约束响应矩阵B，也无合格终端H集合，因此不可无条件激活；仅估计/解析分配部分可编码，B无可靠支持则停止。新增实跑0、未改控制/参数、155项冻结一致；默认0.25/0/0.78/1.60保持，无提交/推送，已停止等待批准。

2026-10-08 [THRUST构型/接触作用线有限可行性](planning/thrust_configuration_feasibility_20261008/REVIEW.md)：仅复用B1/B2/B3/T1，新增实跑0、155项冻结一致。外部力矩确有构型力臂通道，但末20ms平滑补偿需约26–33cm，条件髋增量速度75–94rad/s，不构成最小可执行方案。提前80–100ms条件几何约3–5cm、髋−0.10～−0.16rad虽位置可容纳，实际髋构型PD已被截为−3/0，跟踪与竖直冲量均无保证；姿态参考又耦合轮控/膝逆解/保护。不支持已有目标或增益单点修改，停止此路线、不扩诊断、不仿真；默认0.25/0/0.78/1.60未改，无提交/推送。

2026-10-08 [释放段局部轮速参考唯一一次物理实验](planning/thrust_terminal_forward_single_20261008/REVIEW.md)：独立代码仅将原释放段前向参考0.45→0.43，保持高跳0.32/0/释放0.78/gain1.60及其余控制。新增1实跑/1腾空/0联合通过；末20ms水平冲量1.140→0.540N·s、外部角冲量−0.531→−0.259，离地总H−1.633→−1.450，但双方干预前已差0.157，不能把改善全部归因于修改。正常ARREST/TUCK/EXTEND、最终BALANCE，无异常接触/二次腾空/EMERGENCY；净空19.608cm未达20cm，首触−9.661°/−1.357rad/s，后退87.588cm，安静恢复3.660s。候选不采用、不追加，默认155项及独立74项冻结一致；默认实现始终未替换，保持0.25/0/0.78/1.60。完整原始数据、diff及曲线保留，无提交/推送，已停止。

2026-10-08 [THRUST水平接触力控制可行性审查](planning/thrust_horizontal_control_review_20261008/REVIEW.md)：仅复用B1/B2/B3/T1，新增实跑0、155项冻结一致。轮端为速度输出、腿部为力矩输出；不存在已验证的独立水平力单参数。候选最小结构仅在原释放段局部降低轮端前向参考，保留膝竖直推力及全部保护；不指定未经识别的幅度，不实施。T1释放后发布轮速已跟上目标，不能据此放宽slew或关闭运动学补偿。实际轮电机力矩及力增益仍未知，保住20cm与落地均待验证。默认0.25/0/0.78/1.60不变，停止等待批准，无提交/推送。

最新单次物理结论：[释放比例0.73试验失败后停止](planning/thrust_release_073_single_20261008/REVIEW.md)。1实跑/1腾空/0联合通过，诊断净空24.478cm但保护展腿后失稳/EMERGENCY，首触−17.364°、后退247.420cm；离地总负H未减小。本次反馈同时跨越0.73/0.78阈值，不能认定实现旧预测20ms提前。配置不采用，不追加；默认0.25/0/0.78/1.60未改。

最新只读决策：[高跳THRUST外部角动量收支](planning/thrust_momentum_budget_20261008/REVIEW.md)。T1额外负离地H来自推地期间接触力矩积分，主要为切向项；最后20ms角动量代价大而竖直速度收益小。只提出既有速度释放比例0.78→0.73的单点候选，未实施，保留速度/接触稳定性尚未证明。新增实跑0，默认0.25/0/1.60不变，已停止。

最新只读结论：[T1首触前额外后倾形成阶段](planning/height_032_pitch_formation_20261008/REVIEW.md)。差距主要在晚TUCK/EXTEND前半段扩大；离地总H更负、髋膝交换及更长飞行共同解释，不能只归单帧切换或首触冲击。0新增实跑、未改控制；原0.25/0/1.60保持，完成后停止。

最新物理结论：[THRUST目标0.32首测后停止](planning/thrust_height_032_ab_20261008/REVIEW.md)。1实跑/1腾空，真实双轮同帧净空23.957cm，但首触后倾14.603°、后退153.458cm，落地明显退化，按协议不执行余下2次。0.32不采用，默认及gain1.60保持；原始数据/失败记录保留，已停止。

最新工程评估：[20cm双轮同帧净空可行性](planning/clearance_20cm_feasibility_20261008/REVIEW.md)。优先建议单因素提高THRUST目标jump_height=0.25→0.32，尚未实施；纯收腿缺少可靠20cm执行方案。模型预测不等于物理通过，落地能量与原后退问题仍需验证。本轮0实跑、控制/参数未改，已停止等待批准。

最新设计：[空中轨迹时间与角动量可行性](planning/airborne_reallocation_design_20261008/REVIEW.md)。仅设计计算、无控制修改或新实跑；有限保持的实际收敛时间未闭合，完成后停止等待批准。

最新性能研究：[TUCK目标可行性审查](planning/tuck_target_feasibility_20261008/REVIEW.md)。仅目标修改被现有反向保护阻断，本轮未改控制、未新增实跑，不进入A/B；默认及gain1.60保持。

2026-10-08 [双轮净空延后EXTEND的3+3 A/B](planning/clearance_apex_ab_20261008/REVIEW.md)：6次完整跳跃，净空均值+0.76cm但不一致，后退均值增加8.23cm，候选不采用。独立实验未替换默认/gain1.60，完成后停止。

2026-10-08 [THRUST早期差异六次冻结复验](planning/thrust_early_repeat_20261008/REVIEW.md)：6次原控制诊断均回BALANCE，未复现离地约−2rad/s；接触矩积分未闭合1.02～1.25%，输入延迟不是充分解释。默认/gain1.60未变，已停止。

2026-10-08 [THRUST入口分叉与接触物理诊断](planning/thrust_origin_diagnostic_20261007/REVIEW.md)：六次旧试验全THRUST追踪，新增1次默认控制只读接触力诊断，未调参；接触角动量收支未闭合约1.22%，轮伺服实际力矩仍未知，未认定唯一分叉原因。

2026-10-07 默认velocity单因素落地对照完成：[gain 1.60/1.00 REVIEW](planning/landing_capture_gain_ab_20261007/REVIEW.md)。1.00三次最大后退61.42–71.07 cm，未证明改善；默认保持1.60并停止调参。原始本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/`。

2026-10-07 默认 velocity 性能任务已转为落地后倾/后退及高度优化：[第一阶段分析](planning/velocity_performance_analysis_20261007/REVIEW.md)。默认已能完整跳跃；下列接地候选准入结论只适用于该独立实验线。本轮仅分析旧原生实跑，未改控制/参数、未新增实跑。

[项目首页](../../README.md) · [实验历史索引](RECORDS.md) · [跳跃文档](../../docs/jump/README.md)

## 当前准入

2026-10-07 有限支撑恢复轮决定仍为 **NO_PHYSICAL_ADMISSION**，阶段二未完成。独立候选已增加逐步可信前提检查、唯一恢复输出、有限协调减速／停止终止及发布前完整 1 ms 预算；8 项针对性 CTest 通过。静止腿模型 seed 仅证明该支撑路径，不能算运动制动通过。同源控制器无运动 ROS 检查在切换前因 manager 读回间断 457.181 ms 被原 150 ms 门拒绝，尚未取得 Effort 接管及完整恢复故障链证据。真实 Gazebo 空世界暂停另列，不算机器人物理恢复。正常候选控制律仍未接入输出，不允许受限接地标定或试跳；按用户要求本轮验收后停止。

本轮新增机器人／接地实跑／腾空／达标均 **0**，接地修复线累计 **4／0／0**。权威结论：[本轮有限恢复 REVIEW](planning/finite_recovery_20261007/REVIEW.md) · [本轮问题与通过条件](planning/finite_recovery_20261007/PLANNING.md)。机器验收为本地资料：`planning/finite_recovery_20261007/acceptance.json`。前轮依据保留：[首轮实时准入 REVIEW](planning/realtime_admission_20261007/REVIEW.md) · [第二轮 REVIEW](planning/contact_motion_revision_20261006_round2/REVIEW.md) · [入口补充审查](planning/contact_motion_revision_20261006_round2/REALTIME_ENTRY_ADDENDUM.md)。

当前开发分支为 `work/jump-repair`，尚未验收；`main` 仍是旧基线。GitHub 源码分支不会带上被忽略的原始运行数据或本地冻结副本；需要引用时请使用 [记录索引](RECORDS.md)，其中本地资料使用代码路径标注，不生成失效链接。当前阶段必需的 Markdown 验收报告保留为可上传文档。

## 入口

| 脚本 | 用途 | 当前范围 / 结果 |
| --- | --- | --- |
| [run_ground_input_trial.sh](run_ground_input_trial.sh) | 分配器关闭、有界 P 轮速、完整六路实际输入与站立交接 | 固定五秒输入门通过；不包含模型响应、刹停或跳跃资格 |
| [run_ground_motion_trial.sh](run_ground_motion_trial.sh) | 独立四腿低速接地运动与正常停止 | 实跑 1 次；响应门和当次停止通过，双髋未达激发门，第 2–4 试次及独立复验未执行，整体未通过 |
| [run_ground_contact_motion_trial.sh](run_ground_contact_motion_trial.sh) | 直接 Physics 状态、接地逆解和静摩擦交接候选 | 有限恢复已实现但尚未通过运动／ROS 接管验收，准入 NO；未接正常新控制律、本轮未实跑 |
| [run_velocity_capture_candidate.sh](run_velocity_capture_candidate.sh) | velocity 髋膝协调与接管保护候选 | 最近记录实跑 1 次、未腾空；不是修复完成版，不能替换默认入口 |
| [run_allocator_jump_candidate.sh](run_allocator_jump_candidate.sh) | 独立支撑分配器跳跃候选 | 尚未通过跳跃验收 |
| [run_capture_command_probe.sh](run_capture_command_probe.sh) | 分配器命令/状态时序诊断 | 诊断入口，不是修复版 |
| [run_native_command_probe.sh](run_native_command_probe.sh) | 原生实际输入观测诊断 | 独立诊断入口 |
| [run_reference_jump_demo.sh](run_reference_jump_demo.sh) | 早期 reference 对照 | 历史实验 |

这些候选均不改变首页的默认 velocity 跳跃入口。独立实验控制器和 launch 仍位于 ROS 功能包中；同名顶层实验脚本集中在本目录。比如从工作区根运行：

```bash
./experiments/jump/run_allocator_jump_candidate.sh --help
```

## 构建与数据

已有独立构建目录均保留原位：`build_native_allocator/`、`build_native_command_observer/`、`build_support_allocator/`、`build_ground_input/`、`build_ground_motion/`、`build_ground_motion_contact/`、`build_ground_native_physics/`、`build_ground_realtime_admission/`。本轮独立构建为 `build_ground_finite_recovery/bbot_balance_controller/`（本地资料），未安装或替换默认程序；编译期 ROS 测试变体只向测试 sink 发布，不作物理入口。原始历史 CSV、freeze snapshots 和报告不移动、不覆盖；按阶段从[记录索引](RECORDS.md)打开权威结果。

原生 Physics 插件构建及选取说明见 [native_physics/README.md](native_physics/README.md)。直接引擎状态审计脚本为 [audit_native_engine_state.py](audit_native_engine_state.py)；GetForce getter 仅作旁路诊断，不能替代物理步前的 JointForceCmd 输入。

## 固定约束

接地协议关闭分配器，P 轮伺服增益固定 1.0，轮速目标限幅 ±30 rad/s、轮力矩 ±10 Nm，髋/膝力矩上限 75/60 Nm；Stage 1 和各实验协议的具体差异以相应 `protocol.json` 与原审计报告为准。缺失输入保持 invalid/NaN，不用参考加速度、getter 值或同值命令匹配补数据。

2026-10-07：[首触前后倾与capture控制链只读审查](planning/landing_capture_pitch_chain_20261007/REVIEW.md)。既有七次完整物理记录对齐真实首触；未调参、未新增实跑。

2026-10-07：[TUCK→EXTEND轨迹连续性审查](planning/tuck_extend_continuity_20261007/REVIEW.md)。七次既有记录，零新增仿真；区分旧参考跳变与新轨迹贴合实测状态。

2026-10-07：[负向pitch_rate持续形成时刻审查](planning/negative_pitch_onset_20261007/REVIEW.md)。只读七次已有记录，定位THRUST末段/ARREST初段，未调参或新增仿真。

2026-10-07：[最后接触与空中后转时序审查](planning/contact_vs_airborne_onset_20261007/REVIEW.md)。七次同轴事件与原生腿速变化；5次离地后、2次末接触阶段，未新增仿真或控制修改。

2026-10-07：[ARREST早期制动单逻辑A/B物理验证](planning/arrest_early_decel_ab_20261007/REVIEW.md)。固定6次完成；前30ms制动后移，未确认一致后转改善，不采用候选，正式入口不变。

2026-10-07：[默认控制空中角动量诊断](planning/angular_momentum_20261007/REVIEW.md)。1次只读诊断跳跃，运行时七刚体完整自转/轨道分账，控制参数未改；后转重建误差最大0.0074rad/s，已停止。

2026-10-07：[仅髋早期轨迹角动量A/B](planning/hip_momentum_ab_20261007/REVIEW.md)。固定六次完成；髋项均值减弱但跨次不一致，B2/T3完整失稳样本保留，不采用候选，默认保持并停止。

2026-10-07：[六次A/B离地初态差异只读审查](planning/takeoff_variability_20261007/REVIEW.md)。零实跑；量化THRUST末段后转、实际输入与反馈相位，保留窗口左端已分叉及接触冲量缺失的限制，未改控制。

## 第一代总结入口（2026-10-08）

[第一代控制器技术总结](../../docs/jump_controller_v1/README.md) · [历史实验结果总表](../../docs/jump_controller_v1/EXPERIMENTS.md) · [文件与空间盘点](../../docs/jump_controller_v1/INVENTORY.md) · [待审核整理计划](../../docs/jump_controller_v1/MIGRATION_PLAN.md)。本轮暂停性能优化，仅建索引与整理规划；最新禁用REVERSE_BRAKE、早期COM几何一致化两次物理候选均未改善后退、均不采用，原始数据及报告原位保留。
