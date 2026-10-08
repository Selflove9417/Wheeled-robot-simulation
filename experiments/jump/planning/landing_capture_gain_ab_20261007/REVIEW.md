# landing_capture_gain 单因素 Gazebo 实验：不支持降低到1.00

本轮唯一问题：`landing_capture_gain: 1.60 → 1.00` 能否显著减少约40.68 cm的落地后退，同时不恶化落地稳定性？**本轮没有证明改善，不采纳1.00。** 三次有效test最大后退61.42–71.07 cm，均高于历史40.68 cm；首触后倾9.20–12.29°。本轮baseline自身波动很大，不能用本小样本精确估计gain的独立因果效应，但现有结果足以拒绝把该候选标记为改善。

## 修改前公式确认

默认控制器 `bbot_velocity_jump_controller.cpp:3860–3903`：

```
v = abs(predicted_landing_forward_velocity(now)) >= landing_capture_speed_deadband
    ? predicted_landing_forward_velocity(now) : 0
omega = sqrt(9.81 / clamp(landing_capture_height, 0.30, 0.50))
landing_capture_raw_offset = v / omega
landing_capture_offset = landing_capture_gain * landing_capture_raw_offset
landing_target_x = clamp(landing_wheel_back_bias - landing_capture_offset,
                         -0.12, landing_target_x_max)
```

本次固定：deadband=0.08 m/s，height=0.40 m，back_bias=0.120 m，target_x_max=0.100 m，omega=4.9522722/s。`target_x>0`表示body位于wheel前方，即轮轴相对机身后移；它是传给落地IK的几何目标，不是世界位置目标，也不是实际轮轴位移保证。

| 代入状态 | gain=1.60 的 target_x | gain=1.00 的 target_x | 后者−前者 |
| --- | ---: | ---: | ---: |
| 历史完整跳跃实际规划时锁存 v=0.414680 m/s | −1.39765 cm | +3.62647 cm | +5.02412 cm |
| 同一跳真实首触物理步结束时原生机身前速0.360854357 m/s，反算 | +0.34137 cm | +4.71336 cm | +4.37199 cm |

两行均未截断。**“约5 cm”来自控制器实际锁存的规划输入，不是首触时重算。** 源码在展腿规划时锁存，首触不会重新计算该目标。真实首触反算另列，不能把滞后的odom差分速度或COM速度替代公式使用的body前速。数值精度受CSV有效位数限制。

同时纠正上一分析的首触角速度符号：历史首触 `post_base_world_wx=-0.569381`，所以前倾正方向 `pitch_rate=-wx=+0.569381 rad/s`。本轮角度和角速均从同一原生物理步取值，不与低通IMU日志混用。

## 实验实施、参数入口错误及保留范围

- 所有运行均为当前README完整velocity控制器、`jump_height=0.25`、平地、物理步1 ms、配置RTF=1.0、正常动作速度；两组统一headless。每次新建Gazebo世界，固定初始站立条件并连续满足已有稳定检查1 s后发一次J；不改收展腿时序、限制、控制状态机、轮控制模式或恢复逻辑。
- 实验增益仅通过独立launch作用域ROS参数选择，默认源码/参数文件/程序均未改。当前可执行文件安装路径解析到冻结的 `build/bbot_balance_controller/bbot_velocity_jump_controller`。源码、程序、主launch、模型、控制器配置、profile、observer及全部控制头文件前后哈希一致。
- **发现并纠正入口错误：**原主launch未声明/转交`landing_capture_gain`。最初直接追加`landing_capture_gain:=1.00`只产生未消费的launch配置；实际ROS参数仍1.60。因此B1/B2不能计作test，全部保留并按**实际gain=1.60**列为baseline；没有把它们隐藏或筛掉。A3在发现错误后停止，已经起跳、落地、返回BALANCE，但未满20 s观察，单独保留且不进入统一窗口统计。B3未执行。
- 最小修正是本地 `test_gain_100.launch.py`：加载同一主launch，仅加入 `SetParameter(landing_capture_gain=1.00)`。三个test在发J前读取实际节点参数，断言gain=1.00，且与A1参数差异只能为gain/日志路径；失败即不发J。三个test均通过该检查，计划日志中的目标约+3.4…+3.9 cm，证明实际作用。没有修改主launch或新增通用测试框架。
- 最终有效统计：baseline **4次**，test **3次**，全部观察到真实首次双轮接触后20 s。另保留A3中断记录1次。本轮实际运行/腾空/落地 **8/8/8**，7次完整统一窗口，1次中断观察；不是只发生了7次实跑。无自动调参、无自动失败重跑、无提交/推送。默认gain仍1.60。

## 指标口径

真实首触由完整1 ms原生接触帧确认，七次首触collision pair均为左右轮与地面；接触前连续零接触，非状态机的提前BUFFER触发。原生geometry逐1 ms完整，观测窗口及接触源均无缺帧、无invalid、dt均1 ms。源码/数据绑定及逐步检查见本地SHA/检查文件。

- `pitch=-roll(base quaternion)`，前倾为正；`pitch_rate=-post_base_world_wx`，取首触物理步结束状态。机身前倾参考约3.885°，与控制器相对IMU参考存在初始零点差，不能把两者混用。
- 最大后倾为首触至+20 s原生pitch的最小值对应的负角幅度。最大后退为同窗口原生双轮轴中点沿物理前向(+world-Y)的最小位置相对真实首触位置；最终位移在+20 s同一物理步取值，正为前、负为后。不是轮里程计位移。
- “姿态恢复”是原生pitch距正常平衡参考<0.04 rad且原生|rate|<0.15 rad/s，连续1 s的起始时刻。正常参考由源码balance_offset=0.030及原生/IMU零点差换算，各次约3.885°。
- “全部静稳”沿用现有runner诊断：|控制器pitch−0.030|<0.04、|filtered rate|<0.15、|COM水平速度|<0.08、|世界竖直速度|<0.03，连续1 s的起始时刻。是分析口径，没有修改控制门。恢复时间均相对真实首触；若要使用“确认完成时刻”，在表值上加1 s。

## 全部有效原始结果

B1/B2虽保留原文件名，实际gain为1.60；不要按字母B把它们当作test。

| 记录 | 实际gain | 首触pitch ° | 首触rate rad/s | 最大后倾 ° | 最大后退 cm | 最终轮轴位移 cm | 姿态恢复 s | 全部静稳 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A1 | 1.60 | -6.39 | +1.014 | 6.39 | 56.68 | -14.29 | 0.765 | 3.442 |
| B1 | 1.60 | -14.84 | -1.522 | 15.30 | 144.44 | -110.07 | 1.079 | 4.293 |
| A2 | 1.60 | -2.98 | -1.333 | 4.14 | 0.55 | +82.58 | 1.645 | 2.557 |
| B2 | 1.60 | +0.42 | -1.096 | 0.04 | 21.97 | +13.39 | 1.335 | 3.386 |
| T1 | 1.00 | -9.20 | -1.328 | 9.92 | 66.17 | -19.46 | 0.918 | 3.655 |
| T2 | 1.00 | -10.86 | -0.899 | 10.97 | 61.42 | -29.98 | 2.097 | 3.113 |
| T3 | 1.00 | -12.29 | -1.387 | 12.53 | 71.07 | -24.97 | 0.870 | 3.610 |

| 记录 | 振荡、二次离地与失稳 |
| --- | --- |
| A1 / 1.60 | 缓慢超调后恢复，无二次离地，无持续失稳 |
| B1 / 1.60 | 明显接触振荡；首触后0.162–0.196 s零接触34 ms，两轮净空峰值2.65 mm，属于小幅次生腾空；最终恢复 |
| A2 / 1.60 | 明显短时振荡，且最终向前移动82.58 cm；“少后退”不等于落地点保持良好；最终恢复 |
| B2 / 1.60 | 小幅接触扰动和缓慢超调，无二次离地，最终恢复 |
| T1 / 1.00 | 短时接触扰动及缓慢超调，无二次离地，最终恢复 |
| T2 / 1.00 | 首触后约0.61–0.90 s明显往复振荡，无二次离地，最终恢复 |
| T3 / 1.00 | 小幅短时接触振荡及超调，无二次离地，最终恢复 |

七次均回到BALANCE，无EMERGENCY、无最终无法恢复；这不等于首触稳定或落地后退性能通过。T组没有观测到二次离地，不代表已经证明所有落地稳定性不恶化。

## 对比结论

- 最大后退：baseline均值 **55.91 cm**（范围0.55–144.44，median39.33），test均值 **66.22 cm**（61.42–71.07，median66.17）。均值增加10.31 cm、约18.4%，没有减少；test三次也全部超过历史40.68 cm。
- 首触后倾：baseline平均5.95°（含一次前倾0.42°），test平均10.78°；落地后最大后倾均值由6.47°变为11.14°。test未证明不恶化。
- 全部静稳时间均值：3.419 s→3.459 s，近似相同；三个test最终轮轴仍落在真实首触点后19.46–29.98 cm。姿态能恢复，位置后退仍很明显。
- baseline变化很大：本轮落点计划输入前速约0.033–0.511 m/s，其目标从+0.100 m截断到−0.045 m；test规划前速约0.403–0.428、目标+0.034…+0.039 m。首触/离地状态波动未被这个单参数修改消除。本轮只描述全部样本，不用小样本均值宣称统计显著或把差异全部归因于gain。

**本轮结论：降低到1.00不能据此认定显著减少后退，同时不恶化稳定性；候选不采纳。** 默认保持1.60，实验已结束。不处理高度，不调整收展腿，不继续自动调参。

## 原始资料与复现

本地资料根目录：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/`。

- 每次目录保留 `velocity_log.csv`、`velocity_summary.csv`、`geometry.csv`、`ground_frames.csv`、`native_wrench.csv`、`wheel_joints.csv`、`runtime_parameters.yaml`、`launch.log`、`launch_command.json`、`events.json`、`run_result.json`；三个T目录另有发J前 `verified_parameter_diff.json`。
- 汇总：`metrics.csv`、`metrics.json`、`group_summary.json`、`runtime_parameter_diff.json`；时钟/帧检查 `native_step_checks.json`。
- 绑定：`frozen_sha256.json`、`test_frozen_sha256.json`、`test_after_sha256.json`、`final_frozen_sha256.json`、`raw_sha256.json`。
- 曲线：`comparison_first_2s.png`、`comparison_20s.png`，均为实际Gazebo数据，非模型。源码命令和原生物理观测分开；无效轮力矩输入不补零。
- 复算（不启动仿真）：`MPLCONFIGDIR=/tmp/bbot_velocity_matplotlib python3 src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/analyze_results.py`。最初以起跳前姿态为恢复参考的派生结果另保留为`metrics_prejump_reference.json`及配套脚本；本报告使用正常平衡目标，源脚本与定义一致。

以上原始数据/脚本/图片为本地资料，不生成缺失GitHub链接。本Markdown作为当前验收报告保留。前轮分析报告已纠正首触角速符号和launch参数入口说明，历史原始运行记录未改。
