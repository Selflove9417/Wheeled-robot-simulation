# 高度调度线性 MPC 平衡控制器说明

[`linear_mpc_balance_controller.cpp`](./linear_mpc_balance_controller.cpp) 实现了 **Phase-2 高度调度线性 MPC 控制器**（Height-Scheduled Linear MPC）。算法核心在与节点解耦的 [`linear_mpc.hpp`](../include/bbot_balance_controller/linear_mpc.hpp) 中，QP 求解器在 [`dense_active_set_qp.hpp`](../include/bbot_balance_controller/dense_active_set_qp.hpp) 中，均便于单元测试。

该控制器使用与已部署 LQR **完全相同的设计模型**（见 [`lqr_plant_model.hpp`](../include/bbot_balance_controller/lqr_plant_model.hpp)），在每个 5 ms 控制周期求解一个带约束的滚动时域二次规划（QP）。与第 3 章各 LQR 控制器的本质区别在于：LQR 是无约束线性反馈律 + 事后限幅，而 MPC 在求解时**显式处理转矩饱和与俯仰姿态带约束**。

**高度调度（第二阶段新增）**：$A_d$、$B_d$、$\theta_{\mathrm{eq}}$、$P$ 不再是单一设计高度的快照，而是每个控制周期按当前髋-轴高度 `current_height_` 重建；预测域内 $H$ 视为常量（**frozen scheduling**），不做未来高度轨迹预测，也不是 LTV-MPC。代价权重 $Q/R$、时域 $N$ 与 QP 结构只由参数决定，构造时确定一次。

> 所有与已部署控制器共用的定义（高度语义、状态定义、符号约定、5 ms 内环、平衡角换算）均以 `adaptive_lqr_balance_controller.cpp` 为参考实现，逐条溯源见 `config/mpc_balance_params.yaml` 头部注释。

---

## 一、被控模型与状态

状态向量与已部署 LQR 完全一致：

$$
\boldsymbol e=
\begin{bmatrix}
x-x_{\mathrm r} &
\dot{x} &
\theta-\theta_{\mathrm{eq}} &
\dot{\theta}
\end{bmatrix}^{\mathrm T},
$$

离散化模型（零阶保持，$T_s=0.005\ \mathrm s$）：

$$
\boldsymbol e(k+1)=A_d\,\boldsymbol e(k)+B_d\,u(k),
$$

其中 $u$ 为**两轮总驱动转矩**（模型单位，$+u$ 使机器人前进）。匀速参考被吸收进误差状态，预测方程保持齐次，无需单独参考模型。

执行器映射与转矩输入控制器相同：

$$
\tau_{\mathrm{each}}=\operatorname{sat}\left(-\frac{u}{2},-10,10\right)\ \mathrm{N\,m},
$$

负号是 Gazebo 轮关节的执行方向变换，不是反馈方向错误。两个执行器限制（总转矩 $\pm20\ \mathrm{N\,m}$、单轮 $\pm10\ \mathrm{N\,m}$）作用于同一标量输入，有效界取 $\min(20,\ 2\times10)=20\ \mathrm{N\,m}$。

### 1. 高度调度机制（第二阶段）

每个控制周期的流程：

1. `update_height()` 以 `leg_transition_speed` 把 `current_height_` 平滑逼近目标高度；
2. 在 `current_height_` 处重建连续模型并离散化，得到该高度的 $A_d, B_d$；
3. Riccati 迭代的种子取**部署增益表在该高度插值行的增益**（与固定高度版本同一来源，不引入新增益），求解该高度的 DARE 得到 $P, K$；
4. 用新的 $A_d, B_d, P, K$ 重新 `configure()` MPC 预测矩阵与 QP；
5. $\theta_{\mathrm{eq}}$ 同步跟随该高度（`table` 源时按"先插 $y_c/z_c$ 再取 $-\operatorname{atan2}$"规则）。

关键设计约束：

- **代价与 QP 结构不随高度变化**：$Q/R/N$/限幅/松弛权重只在构造时由参数确定一次，高度调度只更换 $A_d/B_d/P/K$；
- **重建失败是安全的**：Riccati 迭代被拒（种子不稳定等）时**沿用上一拍已配置好的模型**并告警，控制律继续有定义，绝不输出未定义控制量，也绝不因腿部运动清零轮转矩；日志的 `model_height` 与 `current_height_`（`height` 列）可直接核对两者的短暂不一致；
- **frozen scheduling**：本拍读到的 $H$ 在整个 $N=20$ 预测域内视为常量，域内不预测未来腿高；下一拍再取新高度重新调度。

---

## 二、滚动时域优化问题

每个控制周期求解：

$$
\min_{u_0\cdots u_{N-1},\,s^+,s^-}\ \sum_{i=1}^{N}\boldsymbol e_i' Q\,\boldsymbol e_i+\boldsymbol e_N' P\,\boldsymbol e_N+\sum_{j=0}^{N-1}R\,u_j^2+W(s^++s^-)+w_2(s^{+2}+s^{-2})
$$

约束：

- $\boldsymbol e_i=A_d^i\boldsymbol e_0+\sum_{j<i}A_d^{i-1-j}B_d u_j$（预测方程）；
- $|u_j|\le u_{\mathrm{bound}}$（**硬约束**，转矩箱）；
- $e_\theta(i)-s^+\le\theta_{\mathrm{limit}}$、$-e_\theta(i)-s^-\le\theta_{\mathrm{limit}}$（**软约束**，共享松弛对）；
- $s^+\ge0,\ s^-\ge0$。

### 1. 终端代价 P

$P$ 取同一 $(Q,R)$ 的**离散代数 Riccati 解**（Kleinman 策略迭代，见 `lqr_plant_model.hpp` 的 `solve_dare`）。带上它，有限时域控制律复现无穷时域 LQR（单元测试验证）；不带它，闭环谱半径实测 $1.014>1$，即不稳定。因此 Riccati 迭代失败时节点拒绝驱动，输出零转矩。

### 2. 为什么俯仰带用软约束

倒立摆一旦越出俯仰带，全时域硬约束通常不可行，而不可行的 QP 没有定义的输出。采用大权重 $W$ 的共享松弛对后：只要带宽可达，松弛严格为零，解与硬约束情形完全一致（测试断言了这一点）；松弛非零时会被如实上报，实际跌倒由节点的硬安全门限（$0.50\ \mathrm{rad}$）处理。

### 3. 残差间隙

有限时域对无穷时域的残差间隙为 $\rho_{\mathrm{cl}}^{2N}$。$\rho_{\mathrm{cl}}=0.9993$ 时 $N=20$ 为 $2.8\%$、$N=40$ 为 $1.1\%$，测试要求 $N=40$ 比 $N=20$ 更贴近 DARE 增益。

---

## 三、QP 求解器与降级链

QP 由自研的**稠密严格凸 active-set 求解器**求解（Friedland / Lawrence-Keel 形式）：Hessian 正定时，配合阻断约束规则工作集只变化有限次，方法在有限次 KKT 分解内终止——这正是它能不加第三方求解器就跑进 200 Hz 定时器的原因。求解器以可行点起步（上一步计划限幅后抬升松弛），支持热启动，并对每拍上报迭代数、KKT 残差（原始/对偶/平稳性/互补性）与求解耗时。

每拍的降级链（`MpcFallbackStage`）：

| 阶段 | 含义 | 行为 |
| --- | --- | --- |
| `solved` | 带俯仰带的 QP 收敛 | 输出 QP 第一步 |
| `band_dropped` | 去掉俯仰带约束行后收敛 | 输出仅带转矩箱的解 |
| `lqr_gain` | QP 完全不可用 | 退化为单步 LQR $u=-K\boldsymbol e$，仍做限幅 |
| `zero_torque` | 状态非有限或未配置 | 输出零转矩 |

降级为 `band_dropped` 时剩余问题必有解（$u=0$ 在箱内部），该失败只可能是数值故障。所有阶段均记录进 CSV 的 `stage` 列。

---

## 四、默认参数

所有参数均为 ROS 2 参数，默认值见 [`config/mpc_balance_params.yaml`](../config/mpc_balance_params.yaml)，每个默认值都可溯源（节点、`RobotParams` 或实测）。

| 参数名 | 默认值 | 含义 |
| --- | ---: | --- |
| `mpc.horizon` | 20 | 预测时域步数 $N$（预览 $0.1\ \mathrm s$） |
| `mpc.q` | [100, 5000, 3000, 1200] | 状态权重 $Q$ 对角线（第 2 章基线 Q0） |
| `mpc.r` | 8.0 | 控制权重 $R$（2026-09-28 粗扫描标称值，见第十一节） |
| `mpc.wheel_torque_max` | 10.0 | 单轮转矩界（Nm） |
| `mpc.total_torque_max` | 20.0 | 总转矩界（Nm） |
| `mpc.theta_error_limit` | 0.20 | 俯仰误差带半宽（rad） |
| `mpc.use_theta_band` | true | 是否启用俯仰软约束带 |
| `mpc.theta_slack_weight` | 1.0e5 | 松弛线性罚系数 $W$ |
| `mpc.max_qp_iterations` | 300 | QP 每拍迭代上限 |
| `mpc.theta_eq_source` | table | `table`（adaptive 增益表质心换算）/ `geometric` / `override` |
| `control.period_s` | 0.005 | 控制周期（s）；模型按同值离散化 |
| `target_height` | 0.40 | 目标髋-轴高度（m） |
| `height.hip_axle_min/max` | 0.30 / 0.50 | 高度范围（m） |
| `height.startup_hip_axle` | 0.36 | 启动安全初始高度（m） |
| `height.startup_hold_time` | 2.0 | 启动保持时间（s）；只延后伸腿，不拦轮转矩 |
| `leg_transition_speed` | 0.05 | 高度斜坡速度（m/s） |
| `filter.pitch_rate_alpha` | 0.10 | 俯仰角速度低通系数 |
| `filter.x_dot_alpha` | 0.05 | 轮速低通系数 |
| `max_pitch_error` | 0.50 | 倾倒保护硬门限（rad） |
| `joystick.walk_speed` | 0.3 | 键盘 W/S 速度指令（m/s） |
| `joystick.speed_ramp_time` | 1.0 | 速度参考斜坡时间（s） |

标称平衡角 $\theta_{\mathrm{eq}}$ 的换算与 adaptive 节点严格一致：取 legacy_safe 表插值后的悬挂体质心，按 $-\operatorname{atan2}(y_c,z_c)$ 计算，并随指令高度每拍更新。$H=0.40\ \mathrm m$ 时为 $0.052626\ \mathrm{rad}$（模型一致性核查见根目录 `MPC_MODEL_CONSISTENCY_REPORT.md`）。

---

## 五、ROS 2 接口

### 1. 订阅话题

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 俯仰角与俯仰角速度 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 轮位置与轮速（编码器推算 $x,\dot x$） |
| `/model/bbot/odometry` | `nav_msgs/msg/Odometry` | Gazebo 真值位姿，**仅记录日志，不进控制律** |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 为速度参考；`angular.z` 被忽略（无偏航控制） |
| `/target_height` | `std_msgs/msg/Float64` | 髋-轴目标高度，按 $[0.30,0.50]\ \mathrm m$ 限幅；与 adaptive 节点同一话题同一约定（`teleop_keyboard` 的 `Q`/`E` 键每按一次 $\pm0.01\ \mathrm m$） |
| `/robot_mode` | `std_msgs/msg/String` | 急停与恢复 |
| `/linear_mpc/command` | `std_msgs/msg/String` | MPC 专用命令 |

### 2. 发布话题

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/wheel_effort_controller/commands` | `std_msgs/msg/Float64MultiArray` | 左右轮力矩指令 $[\tau_0,\tau_1]$ |
| `/leg_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 四个腿关节位置指令 |

### 3. 命令

`/linear_mpc/command` 与 `/robot_mode` 支持：

| 字符串 | 功能 |
| --- | --- |
| `emergency` / `x` | 关闭平衡控制并发布零轮力矩 |
| `balance` / `b` | 重设位置参考后恢复平衡控制 |
| `reset_position` | 将当前位置设为 $x_{\mathrm r}$ |
| `step_position:<offset>` | 位置阶跃测试：参考位置移动带符号偏移量 |
| `stop_moving` | 速度参考清零并重设位置参考 |

---

## 六、键盘操作

节点内建 `KeyboardReader`，在控制循环内直接读取键盘：

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 速度参考 $+/-`walk\_speed`$ |
| `Space` | 停止移动并重设位置参考 |
| `B` | 恢复平衡控制 |
| `X` | 紧急停机 |

变高度由 `teleop_keyboard`（终端 2）的按键完成：

| 按键 | 功能 |
| --- | --- |
| `Q` / `E` | 目标高度增加/减少 0.01 m（发布 `/target_height`，限幅 $0.30\sim0.50\ \mathrm m$） |

---

## 七、编译与启动

### 1. 编译

```bash
cd /home/admin/bbot_ws_new
colcon build --packages-select bbot_balance_controller bbot_bringup --symlink-install
source install/setup.bash
```

### 2. 启动仿真与控制器

```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=mpc
```

`controller_type:=mpc` 会激活 `wheel_effort_controller` 并启动 `linear_mpc_balance_controller`。参数按"配置文件 → 命令行覆盖"的次序加载，常用启动参数：

| 启动参数 | 默认值 | 对应节点参数 |
| --- | --- | --- |
| `mpc_config_file` | `config/mpc_balance_params.yaml` | 整个 YAML |
| `mpc_target_height` | 0.40 | `target_height` |
| `mpc_horizon` | 20 | `mpc.horizon` |
| `mpc_r` | 8.0 | `mpc.r` |
| `mpc_theta_error_limit` | 0.20 | `mpc.theta_error_limit` |
| `mpc_use_theta_band` | true | `mpc.use_theta_band` |
| `mpc_theta_eq_source` | table | `mpc.theta_eq_source` |
| `mpc_spawn_z` | 0.403 | base_link 出生高度；该默认值沿用其他控制器，MPC 运行应显式设为 **0.50** 以匹配髋-轴 0.36 m 启动姿态 |
| `mpc_log_path` | `data_logs/mpc/mpc_balance_log.csv` | `log_path` |

> 启动姿态为髋-轴 0.36 m，即 base_link 0.50 m；启动 MPC 时建议显式传 `mpc_spawn_z:=0.50`（launch 默认 0.403 是沿用其他控制器的出生高度）。配合 `torque_pid_initial_roll:=0.0` 可消除出生预倾斜带来的初值伪影（见 `MPC_MODEL_CONSISTENCY_REPORT.md` §5.3）。

### 3. 变高度运行

运行中通过 `/target_height` 话题在线改变目标高度（模型与平衡角随 `current_height_` 每拍调度）：

```bash
# 终端 2 启动键盘终端后，Q / E 键每按一次目标高度 ±0.01 m
ros2 run bbot_balance_controller teleop_keyboard
```

### 4. 位置阶跃测试

```bash
# 运行中向 /linear_mpc/command 发布阶跃命令
ros2 topic pub --once /linear_mpc/command std_msgs/msg/String "data: 'step_position:0.30'"
```

批量实验直接用脚本（见第十节）。

---

## 八、日志说明

默认日志文件为 `src/bbot_balance_controller/src/data_logs/mpc/mpc_balance_log.csv`，共 48 列，主要字段：

| 字段 | 含义 |
| --- | --- |
| `time`, `dt`, `stage`, `control_enabled` | 时间戳、实际步长、降级阶段、控制使能 |
| `height`, `base_link_height` | 当前髋-轴高度与 base_link 离地高度 |
| `x`, `x_ref`, `x_error`, `x_dot`, `v_ref`, `v_error` | 位置与速度状态 |
| `pitch`, `pitch_rate`, `theta_eq`, `theta_error` | 姿态状态与平衡角 |
| `u_mpc`, `u_lqr_fallback`, `tau_each` | MPC 输出、单步 LQR 参考与单轮指令 |
| `total_torque_saturated`, `wheel_torque_saturated` | 饱和标志 |
| `theta_band_enabled/limit/binding` | 俯仰带配置与是否贴边 |
| `theta_pred_max/min`, `theta_slack_upper/lower` | 预测俯仰极值与松弛量 |
| `solver_status/iterations/time_us`, `working_set_size`, `warm_start` | QP 求解诊断 |
| `kkt_primal/dual/stationarity/complementarity`, `objective` | KKT 残差与目标值 |
| `gt_pose_x`, `gt_pose_y`, `gt_pose_z`, `gt_valid` | Gazebo 真值位姿（仅记录；z 用于独立核对腿部实际高度） |
| `target_height`, `model_height` | 指令目标高度与模型实际所在高度（重建失败时两者短暂不一致） |
| `model_rebuild_us`, `model_rebuild_failed` | 本拍模型重建耗时（μs）与是否失败 |

---

## 九、单元测试

[`test/test_linear_mpc.cpp`](../test/test_linear_mpc.cpp) 覆盖 9 组行为检查，全部在真实模型上运行：

1. **DARE 对等**：收敛时域必须复现无穷时域 LQR 控制律，$N=40$ 比 $N=20$ 更贴近 DARE 增益；
2. **转矩箱精确执行**：饱和时输入恰好贴在界上，更紧的单轮限制收紧同一输入；
3. **俯仰带三种情形**：宽带不改变答案、紧带精确满足或松弛如实上报、跌出带宽仍输出有界解；
4. **非有限状态**：NaN 状态与未配置实例均输出零转矩；
5. **降级链**：迭代上限为 1 时两级 QP 失败，LQR 步接管且仍限幅；
6. **闭环滚动**：2000 拍俯仰通道收敛、带宽保持、求解耗时低于半个控制周期；
7. **俯仰扰动**：$0.12\ \mathrm{rad}$ 扰动被抓回，输入不超权限；
8. **与 LQR 基线对等**：同模型同初态下 MPC 紧跟同权重 LQR（验收标准是"跟随 LQR"，不是"优于 LQR"）；
9. **高度调度适定性**（第二阶段新增）：$H$ 从 0.30 到 0.50 m 共 41 个点上逐点检查——插值表行是稳定化种子、DARE 有解、闭环渐近稳定、MPC 无回退求解且在执行器权限内；$\theta_{\mathrm{eq}}$ 遵守"先插 $y_c/z_c$ 再取 $-\operatorname{atan2}$"规则（且该规则在中点处与"先算角度再插值"可区分）；每拍重建（冷启动 QP）与只配置一次（热启动 QP）在同一条状态序列上给出一致的第一步输入（$<10^{-9}\ \mathrm{N\,m}$）。

```bash
cd /home/admin/bbot_ws_new
colcon test --packages-select bbot_balance_controller
colcon test-result --verbose
```

> 阈值按未优化编译（空 `CMAKE_BUILD_TYPE`）设定：平均求解耗时要求 $<2500\ \mu s$（半个周期），$-\mathrm{O}2$ 下实测约 $20\ \mu s$。

---

## 十、实验脚本（scripts/）

| 脚本 | 功能 |
| --- | --- |
| `run_mpc_balance_trials.py` | 批量平衡试验：静止/位置阶跃/匀速/推扰四工况，自动启动 Gazebo 并落盘 |
| `run_mpc_r_sweep.py` | R 权重扫描，每格自动重试，产出 `cells/` 原始结果 |
| `aggregate_mpc_r_sweep.py` | 聚合扫描结果为 primary cohort 报表（`aggregate.md`、`sweep_table.md`） |
| `calibrate_mpc_theta_eq.py` | 标定/校验平衡角 |
| `analyze_mpc_position_residual.py` | 位置残差时序与模型检查 |
| `analyze_mpc_long_horizon_position.py` | 长窗口位置验证（编码器 x 对 Gazebo 真值） |
| `snapshot_mpc_experiment_config.py` | 实验配置审计快照 |

`test_mpc_sweep_harness.py` 为扫描工具链的冒烟测试。

---

## 十一、实测结果

> 以下结果在 Phase-1 定高度版本（$H=0.40\ \mathrm m$ 一次性建模）上实测；Phase-2 的每拍重建已在单元测试第 9 组中验证与一次性建模输出一致（$<10^{-9}\ \mathrm{N\,m}$），但这些工况级的扫描数据尚未在调度版本上重跑。

### 1. R 权重扫描（2026-09-28，`data_logs/mpc_r_sweep_20260928/`）

$R\in\{0.5,1,2,4,6,8,12\}$，每格 3 个主队列控制器结果，4 种工况：

| R | 结论 |
| ---: | --- |
| 0.5 / 1 | 不可用：位置与推扰工况 0/3 过稳定门 |
| 2 / 4 | 顶着 $\pm20\ \mathrm{N\,m}$ 限幅才稳住（位置工况饱和 47–59%） |
| 6 / 8 / 12 | 干净：0% 饱和，各工况 3/3 过门 |

$R=8$ 是唯一在所有工况、所有重复中均安静（无抖动）且 p95/p99 最低的值，故取为标称值。**这不是最优性结论**：$R$ 从 $10^{-6}$ 到 $10^{2}$，最慢闭环极点钉在 $|\lambda|=0.999293$（$\tau=7.07\ \mathrm s$），$R$ 对位置收敛速度没有杠杆（详见根目录 `MPC_POSITION_RESIDUAL_ANALYSIS.md`）。

### 2. 位置"稳态残差"的根因（2026-09-29）

position 工况聚合表中的 `x_mean ≈ -84 mm` 不是稳态误差，而是观测窗（约 2.3 个时间常数）内的窗口平均假象叠加慢闭环模态 $\tau=7.07\ \mathrm s$：原始数据中 $e_x$ 全程单调衰减，日志末尾仍以 $8.7\ \mathrm{mm/s}$ 收敛。该慢模态是已部署增益表的属性，不是 MPC 引入的问题。

### 3. 长窗口真值验证（2026-09-29，`data_logs/mpc_longhorizon_20260929/`）

$+0.30\ \mathrm m$ 阶跃后观测 $54.3\ \mathrm s\approx8.7\tau$，3 次重复全部：

- 无跌倒、无安全门触发，全程 `solved`（无降级）；
- $e_x$ 在阶跃后 25–35 s 已收敛至 $-0.36\ \mathrm{mm}$（$<0.12\%$ 阶跃幅值），最终平在 $+2.75\ \mathrm{mm}$（std $0.06\ \mathrm{mm}$）；
- 编码器口径与 Gazebo 真值口径在最后窗口内相差 $\le0.08\ \mathrm{mm}$（斜率 $1.0002\sim1.0010$），收敛不是轮里程假象；
- 三次重复的单指数拟合 $\tau_{\mathrm{fit}}$ 一致到小数点后 3 位，行为确定性可复现。

详见根目录 `MPC_LONG_HORIZON_POSITION_VALIDATION.md`。

---

## 十二、安全机制

- 未收到 IMU 或轮编码器数据时输出零轮力矩，并持续发布启动腿姿使腿先触地（省略此步会使首拍髋部在闭环下跳变，实测饱和 67% 周期并跌倒）；
- 俯仰误差超过 $0.50\ \mathrm{rad}$ 时关闭控制并输出零轮力矩；
- 仿真时钟回滚检测：时间基线重置，绝不用虚构的 $\mathrm{d}t$ 步进离散模型；
- 任何降级阶段（含 LQR 兜底）输出都经过转矩限幅；
- `X` 键或 `/robot_mode` 的 `emergency` 触发急停；`B` 键恢复时重设位置参考。

---

## 十三、适用边界

1. **Phase-2 高度调度 = frozen scheduling**：$A_d/B_d/P/K$ 每拍按 `current_height_` 重建，但预测域内 $H$ 视为常量，不预测未来高度轨迹，也不是 LTV-MPC；高度快速变化时模型滞后一拍，重建耗时已记录在 `model_rebuild_us` 列，需关注其与 5 ms 周期的余量；
2. **无偏航控制**：两轮等转矩，`/cmd_vel` 的 `angular.z` 被忽略；
3. **位置收敛由慢模态决定**：$\tau=7.07\ \mathrm s$ 是增益表属性，调 $R$ 或 $N$ 均无法加快，需要位置积分器或 offset-free 机制才能改变（当前未加）；
4. **$R=8$ 是标称工作点**：不是最优值；进一步的性能提升应从改变慢模态（权重结构/模型）入手，而不是继续扫 $R$；
5. **软约束不替代安全门**：俯仰带松弛非零只表示"带宽不可达"，跌倒判定仍由 $0.50\ \mathrm{rad}$ 硬门限负责；
6. **默认参数仅为仿真起点**：实机使用前必须重新评估传感器噪声、延迟与力矩限幅。
