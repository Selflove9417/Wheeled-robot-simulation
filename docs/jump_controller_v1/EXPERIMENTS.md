# 第一代历史实验、性能与资料索引

本轮只读取既有报告、metrics.json和已生成flight.csv，没有运行仿真、动力学回放或参数扫描。物理次数按各原报告范围，不能把复用同一B1/B2/B3的多份分析重复累计成新实跑。不同campaign中的B1同名不代表同一试次；本文稳定B组专指`clearance_apex_ab_20261008`原配置组。

## 稳定基线的实际性能

来源：[延后EXTEND A/B原报告](../../experiments/jump/planning/clearance_apex_ab_20261008/REVIEW.md)；本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/{physical,analysis}`。参数 `.25 / 0 / .78 / 1.60`。下表是已有原生metrics/flight结果，不是以jump_height代替实测。

| 指标 | B1 | B2 | B3 |
|---|---:|---:|---:|
| 离地原生质量COM竖直速度 (m/s) | 2.158 | 2.200 | 2.155 |
| COM相对真实离地最大上升 (cm) | 23.510 | 24.402 | 23.425 |
| COM相对PRE_JUMP前参考上升 (cm) | 28.315 | 30.106 | 28.242 |
| 最大COM世界高度 (m) | 0.6773 | 0.6952 | 0.6765 |
| 双轮同帧最大轮底净空 (cm) | 15.034 | 16.564 | 15.059 |
| 原生离地至首触 (ms) | 408 | 423 | 408 |
| 空中pitch最小 (°) | -3.870 | -5.005 | -5.489 |
| 空中pitch最大 (°) | 5.731 | 5.946 | 5.968 |
| 空中最大绝对pitch_rate (rad/s) | 1.520 | 1.725 | 1.352 |
| 真实首触pitch (°) | -3.938 | -5.033 | -5.498 |
| 真实首触pitch_rate (rad/s) | -1.183 | -0.539 | -0.379 |
| 落地最大后倾幅度 (°) | 4.379 | 5.080 | 5.518 |
| 首触后最大后退 (cm) | 42.760 | 47.669 | 42.925 |
| 20s轮轴相对位置 (cm) | -2.699 | -6.598 | -3.544 |
| 连续安静区间起点 (s) | 3.447 | 3.435 | 3.407 |
| 回到BALANCE (s) | 4.692 | 4.685 | 4.652 |

首触定义为原生真实轮接触帧；位移是左右轮轴世界位置均值投影到试次前向轴，零点为首触，负值为后退，**不积分轮速**。净空指标是`max_t min(clearance_L(t),clearance_R(t))`，不是左右独立最大值的较小值。COM上升分离地/准备两种零点；原生质量模型与控制器近似七体模型另有IMU小质量差异。

安静恢复沿用现有门：控制pitch距.03rad<.04、滤波rate<.15、COM前速<.08、world_z_dot<.03并连续1s；报告满足区间起点，确认时刻要再加1s。BALANCE重入是状态指标。B组三次无报告判据的二次腾空、非轮碰地或EMERGENCY，均有首触后20s观察；不代表异常安全分支全覆盖。

## 最新两次落地候选（均未采用）

| 指标 | 原B组范围 | 禁用REVERSE_BRAKE | COM几何基准一致化 |
|---|---:|---:|---:|
| 最大后退cm | 42.76–47.67 | 50.73 | 50.80 |
| 20s相对位置cm | −6.60～−2.70 | −8.98 | −9.88 |
| 首触pitch° | −5.498～−3.938 | −7.077 | −4.983 |
| 首触rate rad/s | −1.183～−.379 | −1.322 | +.831 |
| 最大后倾° | 4.379–5.518 | 7.543 | 4.983 |
| 安静区间起点s | 3.407–3.447 | 3.242 | 3.423 |
| BALANCE重入s | 4.652–4.692 | 4.502 | 4.683 |
| 同帧双轮净空cm | 15.034–16.564 | 18.057 | 17.864 |

两次均正常恢复，却没有减少后退。禁用候选首触初态更重，不能据单次轨迹宣称策略导致恶化；几何候选首次生效早于真实首触12ms，净空峰值发生在修改前，不能把其17.864cm归功于CATCH修改。几何forward误差从17.087降至3.579mm（同控制时刻口径），但最大后退与最终偏差未改善；采样时间戳误差与当前帧误差分列在原报告。原默认未被替换，试次结束冻结哈希一致。

最新报告目前被Git忽略，保留为本地资料：`experiments/jump/planning/catch_no_reverse_brake_single_20261008/REVIEW.md`、`experiments/jump/planning/catch_world_geometry_single_20261008/REVIEW.md`。迁移计划列出后续可批准的窄范围报告纳入Git方案，本轮不改.gitignore。

## 历史实验分类和结果总表

下表“采用”指默认控制采用；诊断结论是资料成果，不是新控制资格。各配置仅列已确认的关键差异，完整输入以原报告/provenance/参数快照为准，未读到的配置不补默认值。物理报告有脚本与来源证据也不保证新机器确定复现；未实施设计没有物理可复现成绩。

| 实验 / 日期 | 分类与目的 | 证据类型 | 关键配置/单一差异 | 结果 | 状态 |
|---|---|---|---|---|---|
| [clearance_apex_ab_20261008](../../experiments/jump/planning/clearance_apex_ab_20261008/REVIEW.md) / 20261008 | 正式基线 / 空中A/B | 6次物理，B1/T1/B2/T2/B3/T3 | .25/0/.78/1.60；候选只延后普通EXTEND | B1–B3作为本轮稳定对照；候选平均净空+0.76cm但后退均值+8.23cm，T2后退68.78cm | 基线复用；候选不采用 |
| [landing_capture_gain_ab_20261007](../../experiments/jump/planning/landing_capture_gain_ab_20261007/REVIEW.md) / 20261007 | 落地捕获A/B | 已有物理A/B | .25基线；gain1.60→1.00，独立注入 | 有效test后退61.42–71.07cm、首触后倾9.20–12.29°，baseline波动不能删除 | 不采用1.00 |
| [angular_momentum_20261007](../../experiments/jump/planning/angular_momentum_20261007/REVIEW.md) / 20261007 | 角动量诊断 | 1次物理只读诊断 | 默认gain1.60；控制不变 | 0–80ms总H最大漂移约1.22%，实测关节重建body rate RMS .00560rad/s；支持内部交换解释 | 诊断证据，不是候选 |
| [arrest_early_decel_ab_20261007](../../experiments/jump/planning/arrest_early_decel_ab_20261007/REVIEW.md) / 20261007 | 空中ARREST A/B | 6次物理，全部BALANCE | 默认参数；单处早期腿减速逻辑 | 减速后移，未确认跨次一致后转改善；全部净空<20cm | 不采用 |
| [hip_momentum_ab_20261007](../../experiments/jump/planning/hip_momentum_ab_20261007/REVIEW.md) / 20261007 | 空中髋动量A/B / 失败样本 | 6次物理，4 BALANCE、2 EMERGENCY | gain1.60；仅ARREST中间髋端速公式变化 | 组均值髋贡献减弱但逐次不一致；B2/T3离地前差异、夹限、保护、非轮触地等保留 | 不采用 |
| `catch_no_reverse_brake_single_20261008`（本地报告） / 20261008 | 落地CATCH单次候选 | 1次物理；启动失败另存 | .25/0/.78/1.60；仅禁用临时反向激活 | 正常BALANCE；后退50.73cm、20s偏差−8.98cm，未改善且干预前初态不同 | 不采用 |
| `catch_world_geometry_single_20261008`（本地报告） / 20261008 | 早期CATCH坐标单次候选 | 1次物理 | .25/0/.78/1.60；普通CATCH前250ms几何基准对齐 | 几何forward同控制帧MAE17.087→3.579mm；命令改变，后退50.80cm、偏差−9.88cm，未改善 | 不采用为默认 |
| [contact_vs_airborne_onset_20261007](../../experiments/jump/planning/contact_vs_airborne_onset_20261007/REVIEW.md) / 20261007 | 动力学时间审查 | 已有7次数据离线 | gain1.60及既有1.00试次分列 | 5/7后转形成在完全离地后26–37ms，2/7在最后接触前；保留反例 | 分析结论 |
| [negative_pitch_onset_20261007](../../experiments/jump/planning/negative_pitch_onset_20261007/REVIEW.md) / 20261007 | 后转形成审查 | 已有数据离线 | 按原记录 | 量化持续负pitch_rate起点，不能以最小单点替代持续事件 | 分析结论 |
| [landing_capture_pitch_chain_20261007](../../experiments/jump/planning/landing_capture_pitch_chain_20261007/REVIEW.md) / 20261007 | 落地姿态链审查 | 已有数据离线 | 默认与既有gain候选分列 | 源码/物理链追踪，未取得新控制改进物理资格 | 分析结论 |
| [velocity_performance_analysis_20261007](../../experiments/jump/planning/velocity_performance_analysis_20261007/REVIEW.md) / 20261007 | 默认全流程历史分析 | 已有1次完整跳跃离线 | 历史默认gain1.60 | 净空15.34cm、最大后退40.68cm；最初gain建议后来已由A/B否决 | 分析复用；旧建议不再待执行 |
| [tuck_extend_continuity_20261007](../../experiments/jump/planning/tuck_extend_continuity_20261007/REVIEW.md) / 20261007 | 轨迹连续性审查 | 已有7次数据离线 | gain1.60与既有1.00 | 新段按实测q/v初始化，旧参考与实测误差造成参考跳变；后转早于EXTEND | 分析结论 |
| [clearance_20cm_feasibility_20261008](../../experiments/jump/planning/clearance_20cm_feasibility_20261008/REVIEW.md) / 20261008 | 20cm设计可行性 | 离线/条件式模型 | 原.25；建议.32 | 不是物理通过；其后.32物理试验落地退化，建议不能当当前采用项 | 历史方案，后续试验否决 |
| [airborne_reallocation_design_20261008](../../experiments/jump/planning/airborne_reallocation_design_20261008/REVIEW.md) / 20261008 | 空中重新分配设计 | 离线/未实施 | THRUST和限制不变的条件式轨迹 | 条件式参考轨迹可行不等于执行器能够及时有效收腿；未接控制输出 | 未采用/未实跑 |
| [height_032_pitch_formation_20261008](../../experiments/jump/planning/height_032_pitch_formation_20261008/REVIEW.md) / 20261008 | 高跳姿态形成诊断 | 已有B1/B2/B3/T1离线 | .25 vs .32 | 离地后110–250ms后倾差扩张明显；不能只归因早期ARREST峰值 | 分析结论 |
| [takeoff_variability_20261007](../../experiments/jump/planning/takeoff_variability_20261007/REVIEW.md) / 20261007 | 离地初态差异 | 已有数据离线 | 冻结原试次配置 | 初态/离地rate差异需与候选激活时刻分开 | 分析结论 |
| [contact_motion_review_20261006](../../experiments/jump/planning/contact_motion_review_20261006/REVIEW.md) / 20261006 | 独立接地候选接口 | 源码/离线审查 | 双侧滚动接触模型 | 腿/机身任务不可任意独立指定，存在可达性与逆接口约束 | 未取得物理资格 |
| [contact_motion_qualification_20261006](../../experiments/jump/planning/contact_motion_qualification_20261006/REVIEW.md) / 20261006 | 独立接地候选资格 | 本轮0物理 | 原限制和固定协议 | 静摩擦交接漏解，FAIL取消准入；累计4接地实跑不计默认跳跃 | 不准入 |
| [contact_motion_revision_20261006_round2](../../experiments/jump/planning/contact_motion_revision_20261006_round2/REVIEW.md) / 6_round2 | 独立接地候选round2 | 本轮0物理 | 完整81模式离线与兼容审查 | 计算修正有推进，实时适用/恢复未通过；NO_PHYSICAL_ADMISSION | 不准入，候选未接正式输出 |
| [realtime_admission_20261007](../../experiments/jump/planning/realtime_admission_20261007/REVIEW.md) / 20261007 | 独立候选实时准入 | 本轮0物理 | 不放宽任何门/限制 | 监视/监督有实现，实际接管、故障链、端到端与停止包络未验收 | NO_PHYSICAL_ADMISSION |
| [finite_recovery_20261007](../../experiments/jump/planning/finite_recovery_20261007/REVIEW.md) / 20261007 | 独立候选有限恢复 | 本轮0物理 | 冻结限制/速度 | 静止模型seed停止不代表运动制动物理资格 | NO_PHYSICAL_ADMISSION |
| [thrust_early_repeat_20261008](../../experiments/jump/planning/thrust_early_repeat_20261008/REVIEW.md) / 20261008 | THRUST差异诊断 | 6次冻结默认物理 | 原控制与gain1.60不变 | 全部BALANCE，未复现旧−2rad/s异常，接触矩积分未闭合1.02–1.25%，按上限停止 | 只读诊断，不是控制改进 |
| [thrust_height_032_ab_20261008](../../experiments/jump/planning/thrust_height_032_ab_20261008/REVIEW.md) / 20261008 | 高跳目标单次失败 | 1次物理；T2/T3停止 | jump_height .25→.32，其他不变 | 净空23.957cm；首触后倾14.603°、后退153.458cm；净空+落地联合接受0 | 不采用 |
| [thrust_release_073_single_20261008](../../experiments/jump/planning/thrust_release_073_single_20261008/REVIEW.md) / 20261008 | 高跳提前卸力单次失败 | 1次物理 | .32高跳；release .78→.73 | 净空24.478cm，首触后倾17.364°、后退247.420cm且EMERGENCY | 不采用，立即停止 |
| [thrust_terminal_forward_single_20261008](../../experiments/jump/planning/thrust_terminal_forward_single_20261008/REVIEW.md) / 20261008 | 高跳末段轮速单次候选 | 1次物理 | .32背景；release后局部前速目标.45→.43 | 净空19.608cm；首触−9.661°/−1.357rad/s、后退87.588cm；未联合通过 | 不采用 |
| [thrust_origin_diagnostic_20261007](../../experiments/jump/planning/thrust_origin_diagnostic_20261007/REVIEW.md) / 20261007 | 推地输入与角动量诊断 | 按原报告的已有/诊断记录 | 默认冻结 | 追踪输入相位和接触/姿态形成；不视为改进验收 | 诊断结论 |
| [thrust_momentum_budget_20261008](../../experiments/jump/planning/thrust_momentum_budget_20261008/REVIEW.md) / 20261008 | 推地角动量预算 | 已有B1/B2/B3/T1离线 | 使用各次实际惯量/物理帧 | 接触外矩、总H与输入预算分开核对；不推导未知控制权限 | 分析结论 |
| [thrust_horizontal_control_review_20261008](../../experiments/jump/planning/thrust_horizontal_control_review_20261008/REVIEW.md) / 20261008 | 推地轮水平控制 | 已有数据/源码离线 | 原控制 | 区分轮参考、关节运动学修正与实际接触作用 | 分析结论 |
| [thrust_configuration_feasibility_20261008](../../experiments/jump/planning/thrust_configuration_feasibility_20261008/REVIEW.md) / 20261008 | 推地构型可行性 | 已有数据/离线 | 原限制 | 条件式可行性不能替代接触执行器物理验证 | 未采用新控制 |
| [thrust_input_authority_20261008](../../experiments/jump/planning/thrust_input_authority_20261008/REVIEW.md) / 20261008 | 二输入权限审查 | 0新增物理 | 默认.25/0/.78/1.60冻结 | H重建同传感器stamp可信；当前帧资格/匹配初态不足，未进入扰动分支 | 停止协同原型，不采用 |
| `thrust_momentum_budget_logs_20261008`（本地报告） / 20261008 | 推地记录附属资料 | 已有数据 | 归属momentum budget | 辅助日志目录，不单独记一次物理实验 | 非独立候选 |
| [thrust_vz_momentum_design_20261008](../../experiments/jump/planning/thrust_vz_momentum_design_20261008/DESIGN.md) / 20261008 | 竖直速度/角动量设计 | DESIGN文档/未实施 | 以设计假设为准 | 设计不是控制器已实现功能；后续权限审查未准入 | 未采用 |
| [tuck_target_feasibility_20261008](../../experiments/jump/planning/tuck_target_feasibility_20261008/REVIEW.md) / 20261008 | 有效TUCK可行性 | 仅B1/B2/B3离线 | 原L_RETRACT .66与60ms、原加速度限制 | 反向保护退回入口；直接60ms膝加速度664–781超500；只改目标不成立 | 不进入A/B |

## 逐实验资料定位与复现边界

以下目录均为本地资料，完整文件清单含全部失败运行和配置线索见[200个campaign索引](inventory/CAMPAIGNS.csv)；每项的重要报告、脚本、图与来源记录保留原路径。这里只索引既有产物，不执行运行器。对未找到同名campaign的planning报告，不虚构原始目录，改读报告中列明的跨campaign输入。

### clearance_apex_ab_20261008

- 报告：`experiments/jump/planning/clearance_apex_ab_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008`。
- 分析/运行脚本：`experiments/jump/planning/clearance_apex_ab_20261008/analyze.py`；`experiments/jump/planning/clearance_apex_ab_20261008/report.py`；`experiments/jump/planning/clearance_apex_ab_20261008/summarize.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/comparison_flight.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/comparison_landing.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/analysis/B1/flight.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/baseline_configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/raw_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/test_configure.log`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### landing_capture_gain_ab_20261007

- 报告：`experiments/jump/planning/landing_capture_gain_ab_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007`。
- 分析/运行脚本：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/analyze_results.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/analyze_results_prejump_reference.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/campaign.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/comparison_20s.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/comparison_first_2s.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/PARAMETER_ENTRY_CORRECTION.md`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/final_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/frozen_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### angular_momentum_20261007

- 报告：`experiments/jump/planning/angular_momentum_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007`。
- 分析/运行脚本：`experiments/jump/planning/angular_momentum_20261007/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/campaign.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/momentum.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/momentum_overview.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/diagnostic_only.patch`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/momentum_configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/raw_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### arrest_early_decel_ab_20261007

- 报告：`experiments/jump/planning/arrest_early_decel_ab_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007`。
- 分析/运行脚本：`experiments/jump/planning/arrest_early_decel_ab_20261007/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/baseline.launch.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/early_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/physical/B1/early_torques.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/physical/B1/early_window.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/arrest_baseline_configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/arrest_test_configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007/raw_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### hip_momentum_ab_20261007

- 报告：`experiments/jump/planning/hip_momentum_ab_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007`。
- 分析/运行脚本：`experiments/jump/planning/hip_momentum_ab_20261007/analyze_momentum.py`；`experiments/jump/planning/hip_momentum_ab_20261007/analyze_performance.py`；`experiments/jump/planning/hip_momentum_ab_20261007/audit_reference.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/early_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/physical/B1/early_torques.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/baseline_configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/raw_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007/supplemental_source_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### catch_no_reverse_brake_single_20261008

- 报告：`experiments/jump/planning/catch_no_reverse_brake_single_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008`。
- 分析/运行脚本：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/compare.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/run_one.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/comparison/landing_0_1p5s.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/comparison/landing_0_20s.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/comparison/landing_0_5s.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/baseline_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_no_reverse_brake_single_20261008/experiment_frozen_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### catch_world_geometry_single_20261008

- 报告：`experiments/jump/planning/catch_world_geometry_single_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008`。
- 分析/运行脚本：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/analyze.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/compare.py`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/geometry_audit.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/comparison/landing_0_1p5s.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/comparison/landing_0_20s.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/comparison/landing_0_5s.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/baseline_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/catch_world_geometry_single_20261008/experiment_frozen_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### contact_vs_airborne_onset_20261007

- 报告：`experiments/jump/planning/contact_vs_airborne_onset_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/contact_vs_airborne_onset_20261007/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### negative_pitch_onset_20261007

- 报告：`experiments/jump/planning/negative_pitch_onset_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/negative_pitch_onset_20261007/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### landing_capture_pitch_chain_20261007

- 报告：`experiments/jump/planning/landing_capture_pitch_chain_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/landing_capture_pitch_chain_20261007/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### velocity_performance_analysis_20261007

- 报告：`experiments/jump/planning/velocity_performance_analysis_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/velocity_performance_analysis_20261007/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### tuck_extend_continuity_20261007

- 报告：`experiments/jump/planning/tuck_extend_continuity_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/tuck_extend_continuity_20261007/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### clearance_20cm_feasibility_20261008

- 报告：`experiments/jump/planning/clearance_20cm_feasibility_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008`。
- 分析/运行脚本：`experiments/jump/planning/clearance_20cm_feasibility_20261008/analyze.py`；`experiments/jump/planning/clearance_20cm_feasibility_20261008/report.py`；`experiments/jump/planning/clearance_20cm_feasibility_20261008/tuck_bound.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008/ballistic_and_thrust.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008/three_torque_types.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_20cm_feasibility_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### airborne_reallocation_design_20261008

- 报告：`experiments/jump/planning/airborne_reallocation_design_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/airborne_reallocation_design_20261008`。
- 分析/运行脚本：`experiments/jump/planning/airborne_reallocation_design_20261008/calculate.py`；`experiments/jump/planning/airborne_reallocation_design_20261008/continuous_brake.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/airborne_reallocation_design_20261008/conditional_model.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/airborne_reallocation_design_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### height_032_pitch_formation_20261008

- 报告：`experiments/jump/planning/height_032_pitch_formation_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/height_032_pitch_formation_20261008`。
- 分析/运行脚本：`experiments/jump/planning/height_032_pitch_formation_20261008/compare.py`；`experiments/jump/planning/height_032_pitch_formation_20261008/momentum.py`；`experiments/jump/planning/height_032_pitch_formation_20261008/report.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/height_032_pitch_formation_20261008/angle_budget.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/height_032_pitch_formation_20261008/phase_alignment.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/height_032_pitch_formation_20261008/input_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/height_032_pitch_formation_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### takeoff_variability_20261007

- 报告：`experiments/jump/planning/takeoff_variability_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/takeoff_variability_20261007`。
- 分析/运行脚本：`experiments/jump/planning/takeoff_variability_20261007/analyze.py`；`experiments/jump/planning/takeoff_variability_20261007/extract_inputs.py`；`experiments/jump/planning/takeoff_variability_20261007/summarize.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/takeoff_variability_20261007/comparison.png`。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### contact_motion_review_20261006

- 报告：`experiments/jump/planning/contact_motion_review_20261006`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：未发现独立同名产物；查原报告的跨目录依赖。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### contact_motion_qualification_20261006

- 报告：`experiments/jump/planning/contact_motion_qualification_20261006`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/contact_motion_qualification_20261006/offline_replay.sh`；`experiments/jump/planning/contact_motion_qualification_20261006/prepare_contact_motion_replay.py`；`experiments/jump/planning/contact_motion_qualification_20261006/frozen_files/setup_env.sh`；其余见资产CSV。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### contact_motion_revision_20261006_round2

- 报告：`experiments/jump/planning/contact_motion_revision_20261006_round2`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/contact_motion_revision_20261006_round2/plot_static_blend_round2.py`；`experiments/jump/planning/contact_motion_revision_20261006_round2/root_audit_v4_stop.py`；`experiments/jump/planning/contact_motion_revision_20261006_round2/v4_stop_metrics/normalize_round4_clock_writer_order.py`；其余见资产CSV。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### realtime_admission_20261007

- 报告：`experiments/jump/planning/realtime_admission_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/realtime_admission_20261007/frozen_final/setup_env.sh`；`experiments/jump/planning/realtime_admission_20261007/root/audit_fast_grid.py`；`experiments/jump/planning/realtime_admission_20261007/root/audit_previous_physics.py`；其余见资产CSV。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### finite_recovery_20261007

- 报告：`experiments/jump/planning/finite_recovery_20261007`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：`experiments/jump/planning/finite_recovery_20261007/final_freeze/setup_env.sh`；`experiments/jump/planning/finite_recovery_20261007/root/check_empty_world_pause.py`；`experiments/jump/planning/finite_recovery_20261007/root/check_empty_world_pause_final.py`；其余见资产CSV。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_early_repeat_20261008

- 报告：`experiments/jump/planning/thrust_early_repeat_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_early_repeat_20261008/analyze_diagnostic.py`；`experiments/jump/planning/thrust_early_repeat_20261008/audit_contact.py`；`experiments/jump/planning/thrust_early_repeat_20261008/compare.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/contact_aligned.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/old_aligned.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/old_reference_wheel.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/raw_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/physical/test_after_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_early_repeat_20261008/physical/test_frozen_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### thrust_height_032_ab_20261008

- 报告：`experiments/jump/planning/thrust_height_032_ab_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_height_032_ab_20261008/analyze.py`；`experiments/jump/planning/thrust_height_032_ab_20261008/details.py`；`experiments/jump/planning/thrust_height_032_ab_20261008/report.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/analysis/comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/analysis/three_torque_types.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/analysis/T1/flight.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/baseline_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/provenance.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008/raw_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### thrust_release_073_single_20261008

- 报告：`experiments/jump/planning/thrust_release_073_single_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_release_073_single_20261008/compare.py`；`experiments/jump/planning/thrust_release_073_single_20261008/details.py`；`experiments/jump/planning/thrust_release_073_single_20261008/momentum.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/analysis/contact_momentum_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/analysis/flight_landing_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/analysis/release_contact_detail.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/baseline_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/provenance.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008/raw_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### thrust_terminal_forward_single_20261008

- 报告：`experiments/jump/planning/thrust_terminal_forward_single_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_terminal_forward_single_20261008/compare.py`；`experiments/jump/planning/thrust_terminal_forward_single_20261008/control_effect.py`；`experiments/jump/planning/thrust_terminal_forward_single_20261008/details.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/analysis/contact_momentum_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/analysis/flight_landing_comparison.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/analysis/release_contact_detail.png`；其余见资产CSV。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/baseline_frozen_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008/experiment_frozen_sha256.json`；其余见资产CSV。
- 可复现性：有既有物理记录；脚本、模型、参数和二进制/源码来源须按原provenance绑定，不承诺确定性重现。

### thrust_origin_diagnostic_20261007

- 报告：`experiments/jump/planning/thrust_origin_diagnostic_20261007`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007`。
- 分析/运行脚本：`experiments/jump/planning/thrust_origin_diagnostic_20261007/analyze_diagnostic.py`；`experiments/jump/planning/thrust_origin_diagnostic_20261007/analyze_existing.py`；`experiments/jump/planning/thrust_origin_diagnostic_20261007/audit_contact.py`；其余见资产CSV。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/contact_closure.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/full_thrust_comparison.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/configure.log`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/raw_sha256.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007/physical/test_after_sha256.json`；其余见资产CSV。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_momentum_budget_20261008

- 报告：`experiments/jump/planning/thrust_momentum_budget_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_momentum_budget_20261008/analyze.py`；`experiments/jump/planning/thrust_momentum_budget_20261008/compare.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_20261008/joint_control.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_20261008/primary_budget.png`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_20261008/thrust_budget.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_horizontal_control_review_20261008

- 报告：`experiments/jump/planning/thrust_horizontal_control_review_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_horizontal_control_review_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_horizontal_control_review_20261008/analyze.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_horizontal_control_review_20261008/terminal_control_paths.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_horizontal_control_review_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_configuration_feasibility_20261008

- 报告：`experiments/jump/planning/thrust_configuration_feasibility_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_configuration_feasibility_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_configuration_feasibility_20261008/analyze.py`。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_configuration_feasibility_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_input_authority_20261008

- 报告：`experiments/jump/planning/thrust_input_authority_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_input_authority_20261008`。
- 分析/运行脚本：`experiments/jump/planning/thrust_input_authority_20261008/observe.py`；`experiments/jump/planning/thrust_input_authority_20261008/plot.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_input_authority_20261008/H_observation_validation.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_input_authority_20261008/provenance.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_momentum_budget_logs_20261008

- 报告：`experiments/jump/planning/thrust_momentum_budget_logs_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_momentum_budget_logs_20261008`。
- 分析/运行脚本：未发现独立同名产物；查原报告的跨目录依赖。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### thrust_vz_momentum_design_20261008

- 报告：`experiments/jump/planning/thrust_vz_momentum_design_20261008`。
- 本地同名资料目录未发现；输入来自原报告所引历史试次，不把此审查计为独立物理运行。
- 分析/运行脚本：未发现独立同名产物；查原报告的跨目录依赖。
- 结果图：未发现独立同名产物；查原报告的跨目录依赖。
- 来源、配置与差异：未发现独立同名产物；查原报告的跨目录依赖。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

### tuck_target_feasibility_20261008

- 报告：`experiments/jump/planning/tuck_target_feasibility_20261008`。
- 原始/派生资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/tuck_target_feasibility_20261008`。
- 分析/运行脚本：`experiments/jump/planning/tuck_target_feasibility_20261008/analyze.py`；`experiments/jump/planning/tuck_target_feasibility_20261008/report.py`。
- 结果图：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/tuck_target_feasibility_20261008/distance_comparison.png`。
- 来源、配置与差异：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/tuck_target_feasibility_20261008/provenance.json`；`src/bbot_balance_controller/src/data_logs/flat_jump_trials/tuck_target_feasibility_20261008/source_sha256.json`。
- 可复现性：这是既有数据分析/设计或未准入分支；只有材料/流程可追溯，没有新的物理通过结果。

## 较早版本与基础功能资料

- 本地 `restore_complete_jump_20261003_102513`：默认恢复的两跳审计、before_restore与restored_snapshot，见根目录[DEFAULT_FLAT_JUMP](../../DEFAULT_FLAT_JUMP.md)。恢复说明承认主文件未逐字匹配旧历史SHA256，不能把它写成完全同一历史版本。
- 2026-09-26～10-01的baseline/release/flight/touchdown/recovery/continuous_session等campaign全部留在CAMPAIGNS.csv；名称含smoke/retry不是删除依据。部分已成为默认历史，部分失败/被替代，采用状态需原RESULT/REVIEW/provenance确认。
- classic `bbot_jump_controller`、reference与landing-repair均有构建/入口引用，不因当前非默认即废弃。
- `height_campaign`、chapter42、PID/GS-LQR/MPC与hardware资料属于基础平衡/高度研究，继续由[平衡导航](../balance/README.md)管理，不并入跳跃性能样本。
- `flat_jump_trails`与`flat_jump_trials`拼写相近但内容不等同，暂保留；不能合并覆盖或凭名称删除。

## 信号与绘图语义

通用入口仍是`src/bbot_balance_controller/src/data_logs/plot_jump_log.py`。第三图为髋膝Effort命令；只在`effort_mode_active=1`且`leg_mode_switch_pending=0`绘制，Position/未知以NaN断线。髋膝参考/实测角可以观察首次BALANCE/PRE_JUMP/SQUAT的Position效果，但模式仍应读日志，不按状态猜。COM世界高度与IK站高、双轮净空、轮速等效速度与轮轴真位移分别命名；触地相对位移必须提供同次geometry与真实first_touch指标。

`controller_mode`是旧模式标签，在实际Effort状态也可能显示POSITION；不可用它单独覆盖active/ACK信息。`hip_effort_left`等JointState effort可能为零或插件合负载，不是Position内部电机转矩。原生输入记录要检查command类型、valid位、阶段和Physics消费顺序；缺失时保持invalid。最新几何候选的额外日志字段只属于候选，不能在默认旧CSV中假造。
