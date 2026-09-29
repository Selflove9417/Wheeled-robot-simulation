# MPC 长窗口位置验证（R = 8，编码器 x 对 Gazebo 真值）

日期：2026-09-29　目的：为 `MPC_POSITION_RESIDUAL_ANALYSIS.md` 补上当时缺的两项证据 —— **≥40 s 的观测窗** 与 **Gazebo 位姿真值**。
本轮**只加日志、不改控制**：R=8、H_hip=0.40、Q=[100,5000,3000,1200]、N=20、±10/±20 Nm、姿态带 0.20 rad、硬门 0.50 rad、
滤波 α=0.10/0.05、`theta_eq_source=table`、`startup_hip_axle=0.36`、`startup_hold_time=2.0`、`leg_transition_speed=0.05`、
`mpc_spawn_z=0.50`、`torque_pid_initial_roll=0.0` 全部保持冻结。未做参数扫描，未加积分器。

## 1. 真值来源（全部从现有 launch/bridge 与源码读出，未猜测）

| 项 | 取值 | 出处 |
|---|---|---|
| topic | `/model/bbot/odometry` | `bbot_bringup/launch/bbot_gazebo.launch.py:546`（`imu_clock_bridge` 内，无条件启动，故 `controller_type:=mpc` 下同样有效） |
| 消息 / 字段 | `nav_msgs/msg/Odometry` → `msg->pose.pose.position.{x,y,z}` | 与 `bbot_velocity_jump_controller.cpp:560-570` 现有订阅完全一致 |
| 世界系约定 | `WorldPoseVelocity` 头注释："Odometry.pose is in the world/odom frame"，且 **twist 不一定在世界系** | `include/bbot_balance_controller/world_pose_velocity.hpp:6-10` |
| 前向轴 | **由数据判定为 `pose.position.y`**（与编码器 x 的相关系数 0.9971/0.9990/0.9996，行程比 0.9923；`pose.position.x` 全程 ≈ 0） | 日志实测，见 §3 |

正因为不预设前向轴，节点把 x、y 两个分量都写进日志（`gt_pose_x,gt_pose_y,gt_valid`，追加在原有 41 列之后 ⇒ 现有按列名读取的脚本不受影响）。
真值**只记录不进控制律**，控制路径零改动。

## 2. 三次 rep 逐次结果（每次 +0.30 m 阶跃后观测 54.3 s ≈ 8.7 τ）

| rep | 首个受控拍 | 阶跃时刻 | 阶跃后观测 | 撞安全门 | 真值有效率 | 前向轴 corr | gt/x_enc 斜率 | 截距 | 残差 RMS |
|---|---|---|---|---|---|---|---|---|---|
| rep1 | 0.031 s | 8.672 s | 54.30 s | 否 | 97.1% | 0.9996 | **1.00017** | 0.0202 m | 1.65 mm |
| rep2 | 0.011 s | 7.949 s | 54.31 s | 否 | 97.2% | 0.9971 | **1.00103** | 0.0611 m | 2.12 mm |
| rep3 | 0.013 s | 8.115 s | 54.30 s | 否 | 95.8% | 0.9990 | **1.00031** | 0.0294 m | 1.79 mm |

编码器行程 vs 真值行程：0.3921 / 0.3891，0.3860 / 0.3805，0.3966 / 0.3934 m。

## 3. 收敛判据（模型无关 + 拟合，两口径互校）

| rep | e_x 均值 @step+25…35 s | e_x 均值 @>step+45 s | 末段 std | 末段斜率 | 同窗**真值**误差均值 | 自由渐近线单指数拟合 c₀ | τ_fit | 拟合 RMS |
|---|---|---|---|---|---|---|---|---|
| rep1 | **−0.36 mm** (std 1.47) | **+2.75 mm** (std 0.06) | 0.06 mm | +2.2e-5 m/s | +2.83 mm | **+2.858 ± 0.002 mm** | 6.261 s | 0.14 mm |
| rep2 | −0.32 mm | +2.75 mm | 0.06 mm | +2.1e-5 m/s | +2.80 mm | +2.859 ± 0.002 mm | 6.261 s | 0.13 mm |
| rep3 | −0.39 mm | +2.75 mm | 0.06 mm | +2.2e-5 m/s | +2.82 mm | +2.857 ± 0.002 mm | 6.260 s | 0.14 mm |

- 阶跃后误差从 −0.30 m 起单调衰减，**25–35 s 已经到 −0.36 mm（<0.12% of step）**，之后越过零点到 +2.75 mm 并平住（std 0.06 mm、斜率 2.2e-5 m/s）。
- **编码器口径与真值口径在最后窗口内相差 ≤0.08 mm** ⇒ 收敛不是轮里程假象。
- 三次 rep 的 τ_fit 与渐近线一致到小数点后 3 位 ⇒ 确定性行为，可复现。

## 4. 编码器 vs 真值：没有比例误差，也没有累积漂移

`gt_disp = a·x_enc + b`，在受控窗上拟合：**a = 1.0002 / 1.0010 / 1.0003（偏离 1 ≤ 0.10%）**，残差 RMS 1.65–2.12 mm（0.39 m 行程内）。

⇒ 先前记在 §7-E3 的疑虑（gz 实测机身高度比"髋-轴+0.14"高 8.7 mm ⇒ 有效滚动半径可能 ≈0.0753）**在水平位移通道上不成立**：若真是 0.0753/0.07，斜率应为 ≈1.076，实测 1.0003。截距 b 是三 rep 各不相同的常数（20/61/29 mm），来自两套原点对齐方式，不是随时间累积的漂移。E3 的高度方向疑点仍未结，但已与位置调节解耦。

## 5. 残余 ~2.9 mm 的机理（不是"缺积分器"）

末段真实状态（三次 rep 相同）：`u_mean = +0.0001 Nm`（0.1 mNm）、`|u|max = 0.0001`、`θ_err = −0.0045° = −7.9e-5 rad`、`x_dot = 2e-5 m/s`、饱和 0、姿态带绑定 0、H=0.40、θ_eq=0.052626。

- 该平衡态上**控制器几乎不出力**，因此这 2.9 mm 不是"力矩不够/被限幅卡住"。
- 它恰好落在本项目已建立的位置偏置定律上：`e_x ≈ (k_θ/k_x)·Δθ`。R=8 的 DARE 增益 k_θ/k_x = 109.408/2.9419 = **37.2**；
  实测 e_x/Δθ = 2.86e-3 / 7.9e-5 = **36.2** ⇒ 与定律一致（差 3%）。
  即：表列 θ_eq 与植物真实悬垂角相差约 8e-5 rad（0.0045°）时，纯误差调节器就会留 ≈3 mm 位置偏差 —— **要消掉它是 θ_eq 标定问题（8e-5 rad 量级），不是缺积分状态**。

## 6. 对"是否实现 integral / offset-free MPC"的结论

按给定判据（长期误差趋近 0 就不做）执行：**不实现，也不改 Q/R/N。**

- 阶跃后 ~25 s 误差已 <0.4 mm（0.12% of step），54 s 内平在 +2.8 mm（0.95%），std 0.06 mm；
- 不存在需要持续力矩抵消的常值扰动（末段 u ≈ 0.1 mNm）；
- 剩余 ~3 mm 有明确且更便宜的因果解释（θ_eq 与实际悬垂角差 8e-5 rad），积分作用只是把这条残差用持续力矩换掉，不是补上"缺失的平衡点"。

**证据边界（不过度解读）**：
1. 单指数自由渐近线拟合的 c₀ 会把任何更慢的分量吸收进常数 ⇒ "+2.86 mm" 应读作"**稳态上界约 3 mm**"，而不是"已证实存在 2.86 mm 真实偏置"。要区分需 >100 s（>15 τ）的窗口。
2. τ_fit = 6.26 s 与模型最慢极点 τ = 7.07 s 不同：真实响应是多模态之和，单指数拟合给出的是混合等效值；两者不矛盾，也不应用拟合 τ 去反推极点。
3. 若将来要把位置精度做到亚毫米级，候选顺序：先核对/标定 θ_eq（8e-5 rad 量级）→ 才谈 offset-free/增广扰动状态。**"加长 N"不作为候选**：带 DARE terminal P 的 N=20 已≈无限时域 LQR（首动增益与 DARE 差 ≤2.6%，ctest 记 N=40 为 1.09%），现有证据不支持有限时域是慢收敛的原因。若目标是**沉降速度**，第一步只做 `Q_x` 对闭环慢极点的**离线谱分析**，不直接改控制器。

## 7. 复现

```bash
cd /home/admin/bbot_ws_new && source /opt/ros/iron/setup.bash && source install/setup.bash
# 采数（3 次重复，每次阶跃后 54 s；配置全冻结）
ROS_DISABLE_LAUNCH_SHUTDOWN_KILL=1 python3 \
  src/bbot_balance_controller/scripts/run_mpc_balance_trials.py \
  --trials position --repeats 3 --duration 60 --out-dir /tmp/mpc_longhorizon_pos
# 分析（只读）
python3 src/bbot_balance_controller/scripts/analyze_mpc_long_horizon_position.py \
        --dir /tmp/mpc_longhorizon_pos
```
产物：`/tmp/mpc_longhorizon_pos/{position_rep{1,2,3}.csv,summary.json,long_horizon_analysis.json}`（每个 CSV 约 4.6 MB / 12.7 k 行，含新列 `gt_pose_x,gt_pose_y,gt_valid`）。
分析脚本：`src/bbot_balance_controller/scripts/analyze_mpc_long_horizon_position.py`。
节点侧改动仅：订阅 `/model/bbot/odometry` + 三个日志列（`linear_mpc_balance_controller.cpp`），ctest 20/20 仍通过。
