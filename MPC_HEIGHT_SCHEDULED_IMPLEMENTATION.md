# Height-Scheduled Linear MPC（第二阶段）实施报告

日期：2026-09-29。基线：第一阶段定高度 Linear MPC v1（冻结，Q=diag(100,5000,3000,1200)、R=8、N=20、Ts=5 ms）。
本阶段目标：在**不调 Q/R/N**、不做 NMPC、不做完整 LTV-MPC 的前提下，让 MPC 的模型随
`current_height`（髋-轴↔轮-轴距离 H，允许 0.30–0.50 m）逐拍更新，并验证 0.40→0.30→0.50→0.40
动态升降时轮式平衡不中断。

本阶段要回答的问题（第 11 条）：**在保持同一组 Q/R/N 的情况下，当前 Linear MPC 能否在
H=0.30–0.50 m 范围内随高度变化实时更新模型并保持平衡？**
→ **能。** 依据是下面的 §6（离线全程可解且稳定）、§9（五个固定高度 10/10 站稳）、§10（三段升降
6456 个受控拍全部 `solved`、0 次回退、0 拍饱和、过渡中 |pitch 误差| 峰值 ≤ 2.95 mrad）。
唯一需要付出的代价是编译口径：默认 -O0 构建下每拍重建会超出 5 ms 周期，因此本目标单独开 `-O2`（§8）。

区分口径：**实测**＝在无头 Gazebo 里真跑出来的 CSV；**离线**＝直接调用仓库 MPC 核心算出来的数；
**推断**＝只有代码/物理理由、没有跑过的。下面逐处标注。

---

## 1. 修改文件

只动了 MPC 相关的 6 个文件。`adaptive_lqr_balance_controller`、`lqr_gain_scheduled_controller`、
`bbot_kinematics` 一字未改（已用 `git status` 核对）。

| 文件 | 改动 |
|---|---|
| `src/bbot_balance_controller/src/linear_mpc_balance_controller.cpp` | 新增 `/target_height` 订阅；`build_model()` → `rebuild_model(H)` 并在每个控制周期调用；`theta_eq` 随重建成功才更新；日志追加 5 列；`print_summary`/节流日志反映调度语义；节流行的速度标签改为 `x_err / x_dot / v_ref`（见下方注） |
| `src/bbot_balance_controller/CMakeLists.txt` | 仅对 `linear_mpc_balance_controller` 目标加 `target_compile_options(... PRIVATE -O2)`（依据 §8 的实测） |
| `src/bbot_balance_controller/test/test_linear_mpc.cpp` | 新增用例 9：0.30–0.50 网格适定性 + theta_eq 规则 + 每拍重建与只配置一次的等价性 |
| `src/bbot_balance_controller/scripts/bench_mpc_height_scheduled.cpp` | 新增：§6/§8 全部离线数字的来源（可解性网格、分段耗时、冷/热启动等价、准 LTV 仿真），文件头给了独立编译命令 |
| `src/bbot_balance_controller/scripts/analyze_mpc_height.py` | 新增：从控制器 CSV 核对 model/target/current 三个高度、theta_eq 规则、gz 真值腿高、稳态与耗时 |
| `src/bbot_balance_controller/scripts/run_mpc_height_transition_trial.py` | 新增：`/target_height` 驱动的动态升降试跑与分段统计 |

`include/bbot_balance_controller/linear_mpc.hpp` 与 `lqr_plant_model.hpp` **未修改**：`configure(Ad,Bd,Q,R,P,K,config)`
本来就是"给定模型就重建全部预测矩阵/Hessian/约束"的接口，高度调度只是在调用它时换一套输入。
`config/mpc_balance_params.yaml` 与 `bbot_gazebo.launch.py` 也**未修改**（高度相关参数与转发在第一阶段就已照 adaptive 建好）。

> **注：终端节流行的速度标签（第一阶段起就有的写法，非本阶段引入）**
> 旧行把状态向量的第 1、2 个分量打成 `x=` 和 `xdot=`，而它们其实是 `x_error` 与
> `x_dot − v_ref`（速度误差）。第一阶段归档日志里一个巡航样本：`v_ref=−0.455`、实测 `x_dot=+0.112`、
> `v_error=0.567` ⇒ 旧终端会打 `xdot=0.567`，看着像速度 0.567。现在改成 `x_err / x_dot / v_ref` 三个独立量。
> **实测验证**（`--trials speed`，指令 0.15 m/s，留档
> `v2_speed_print_check_summary.json`）：终端里 `x_dot` 沿 −0.039 → 0.119 → 0.17 爬升、
> `v_ref=0.150`，同一时刻 CSV 为 `x_dot=0.175 / v_error=0.025`；巡航段 CSV 汇总 `v_ref 0.150`、
> `实测 0.161`。只改打印，控制路径与 CSV 列名/列序都没动；`grep` 过全仓库，没有任何脚本解析这行终端输出。

留档：`src/bbot_balance_controller/src/data_logs/mpc_height_scheduled_20260929/`
（v1 基线 summary+md5、v2 固定 0.40 summary、五个高度 summary+md5、动态 report.json、
速度标签验证 `v2_speed_print_check_summary.json`）。原始 CSV 在
`/tmp/mpc_v1_baseline_20260929`、`/tmp/mpc_v2_fixed040`、`/tmp/mpc_v2_fixed_heights/H*`、
`/tmp/mpc_v2_dynamic`、`/tmp/mpc_v2_speed_print_check`。

## 2. 高度数据流（语义全部取自 adaptive_lqr_balance_controller.cpp）

```
/target_height  (std_msgs/msg/Float64)                    ← 与 adaptive:247-252 同一 topic/类型
   └─ target_height_ = clamp(msg->data, height.hip_axle_min, height.hip_axle_max)   [0.30, 0.50]
        │  （启动时 target_height 走 ROS 参数，同样 clamp，:139-140）
        ▼
control_loop 每 5 ms：
   balance_ready_elapsed_ += dt
   if (balance_ready_elapsed_ >= height.startup_hold_time)   ← 默认 2.0 s，只延后伸腿
        update_height(dt)   ← current_height_ 以 leg_transition_speed（0.05 m/s）限速逼近 target
   publish_leg_pose()       ← 每拍都发，与过渡无关
   base_link_height() = current_height_ + height.base_to_hip(0.07) + wheel_radius(0.07)
   kinematics_.inverse_kinematics(base_link_height, 0.0)
        ▼
/leg_position_controller/commands = [q_hip, q_knee, q_hip, q_knee]
```

- 高度限幅、参数名、`base_link` 换算、IK 调用方式逐条对齐 adaptive（`:180-209`、`:618-641`、`:859-867`）。
- `startup_hold_time` 语义不变：**只**决定什么时候开始腿部过渡，**不**拦轮转矩（adaptive `:713-715`）。
- **实测（动态跑）**：三段过渡共 1651 个受控拍（占受控拍 25.6%）里 `stage` 全为 `solved`、
  `fallback_rows=0`、`total_torque_saturated` 全程 0 → 腿在动的时候轮控一直在算、没被清零。
- **实测**：`dh/dt` 在上升/下降段均值都是 ±50.000 mm/s、p95 也是 50.000 mm/s → 限速 0.05 m/s 精确兑现。
- **实测**：`/target_height` 的 topic 图迟到仍在（发布到日志 `target_height` 列变化约 1.0–1.3 s），
  所以脚本先确认节点收到新目标、再等腿部到位，并把"目标确认"与"到达"分开记录（§10）。

## 3. 模型调度数据流

```
control_loop 每拍（frozen scheduling）
  1. H_model = current_height_                       ← 不是 target_height_
  2. rebuild_model(H_model):
       continuous_matrices(H_model) -> A, B          （悬挂体几何在 H 上重解）
       zoh_discretize(period_s_=0.005, A, B) -> Ad, Bd
       interpolate_row(H_model) -> 表列插值行（只当 Riccati 种子）
       solve_dare(Ad, Bd, Q, R=8, seed) -> P, K      （实测 6 次 policy pass 收敛）
       LinearMpc::configure(Ad, Bd, Q, R, P, K, config)  ← 幂矩阵/Psi/Theta/Hessian/约束行全重建
     成功 -> model_height_ = H_model，并更新 theta_eq_
     失败 -> 保留上一拍的模型与 theta_eq（两者仍是配套一对），model_rebuild_failed=1，WARN 节流
  3. 状态 X = [x-x_ref, x_dot-v_ref, pitch-theta_eq(H_model), pitch_rate]
  4. mpc_.solve(X) -> U=[u0..u19]，只下发 u0（tau_each = -0.5*u0）
  5. 下一拍回到 1，用新的 current_height_
```

Q、R、N、`theta_error_limit=0.20`、转矩限幅、硬安全门 0.50 rad 全部保持第一阶段的值，未重调。
`prepare_cost_and_qp_config()` 在构造时把 Q 与 `LinearMpcConfig` 定下来一次，之后每拍只换模型输入。

**失败路径**（第 10 条"某个高度求解失败不许输出未定义控制量"）：
- DARE 不收敛 / Ad,Bd,P 非有限 → **不动模型**，沿用上拍已配置好的问题，继续正常求解；日志
  `model_rebuild_failed=1`。
- `configure()` 被拒（只可能是固定配置本身有问题）→ `model_ready_=false`，走既有的 disabled 分支
  （发腿部位姿 + 零轮转矩 + `stage=disabled`）。
- **实测**：全部 27 条日志（v1 6 条 + v2 固定 0.40 6 条 + 五高度 10 条 + 动态 1 条）里
  `model_rebuild_failed` 之和 = 0，`fallback_rows` = 0，`stages` 只有 `solved`。

## 4. A / B / theta_eq / P 如何随 H 更新

| 量 | 更新方式 | 实测验证 |
|---|---|---|
| `A(H)`,`B(H)` | `lqr_plant::continuous_matrices(H)`：由 H 解腿部两连杆角 → 悬挂体 COM/惯量/`m2`,`m3`,`gravitational`,`determinant` → 线性化矩阵 | 27 条日志、H 覆盖 0.300–0.500 m |
| `Ad`,`Bd` | 同一个 `zoh_discretize(period_s_=0.005)`，expm 缩放平方 | 同上 |
| `theta_eq(H)` | **先**按 H 线性插值表列 `y_com/z_com`，**再**取 `-atan2(y_com, z_com)`（adaptive `:603/:615/:717-719` 的规则，不是"先算各节点角度再插角度"） | 每条日志 `max|theta_eq(列) − 规则值| ≤ 5.0e-8 rad`；动态跑里 theta_eq 实际走过 `[0.030969, 0.076757]` rad，跨度 0.0458 rad |
| `P(H)` | 每个 H 重新做 DARE，种子 = 该高度的表列插值行（与第一阶段一次性建模用的同一来源） | 41 点网格（0.30→0.50，步长 0.005）全部收敛，最多 6 次 policy pass；最差种子谱半径 0.999299 < 1，最差闭环谱半径 0.999293 |

单元测试用例 9 把上面两条锁住（ctest `linear_mpc` 通过，20 项 ctest 中除一个因 shell 未 source
ROS 的 python 用例 `wheel_effort_velocity_servo` 报 `ModuleNotFoundError` 外全绿，与 MPC 无关）。
中点交叉验证：H=0.375 处"先插 y/z 再 atan2"= 0.0581067，"先算角度再插值"= 0.0583961（差 2.89e-4 rad），
几何真值 = 0.0582898 ⇒ 采用规则既与另一规则可区分，又贴住几何。

## 5. frozen scheduling 的准确含义

本阶段的"预测域内高度不变"是指：**在一个 5 ms 控制周期内**，
`Ad(H_k)/Bd(H_k)/theta_eq(H_k)/P(H_k)` 用第 k 拍的 `current_height` 建好，随后 N=20 步（0.1 s）
的预测、Hessian、俯仰带约束行都用这**同一套**矩阵；不预测未来腿高轨迹，`A_k/B_k` 不随预测步变化，
因此它**不是** LTV-MPC，更不是 NMPC。到第 k+1 拍，重新读 `current_height`、重新调度整套模型。

因此域内的模型失配量级是"这段域里 H 会走多远"：0.05 m/s × 0.1 s = **5 mm**（一拍是 0.25 mm）。

日志上如何核对（第 6 条"不许用 target_height 提前切换模型"）：
- `model_height` 列记录的就是下发 u0 所依据的那套模型的 H。
- `model_height == current_height` 是构造性的相等（`rebuild_model()` 的入参就是 `current_height_`），
  所以这条只证明代码路径对；真正能证伪"提前切模型"的是它**不等于目标高度**：
  **实测（动态跑）** `max|model_height − current_height| = 0.000e+00`，
  而 `max|model_height − target_height| = 0.1997 m`、`max|current_height − target_height| = 0.1997 m`。
  ⇒ 0.30→0.50 的半路上，target 已经是 0.50，模型仍在 0.30–0.50 之间逐拍行走。

## 6. 离线适定性（回答"同一组 Q/R/N 能不能全程调度"）

**离线**（直接调用 `lqr_plant_model.hpp` + `linear_mpc.hpp`，非 Gazebo），Q=diag(100,5000,3000,1200)、R=8、N=20：

- H 从 0.30 到 0.50 步长 0.005 共 41 点：DARE 全部收敛（6 pass），最差种子 `|λ|=0.999299`、
  最差闭环 `|λ|=0.999293`，慢极点 τ = 7.07 s 在全区间一字不变（与第一阶段"R 对位置沉降没有杠杆"一致）。
- 五个名义高度的 MPC 首动增益：`[2.933 22.675 101.249 23.503]`(0.30) → `[3.001 23.374 120.078 30.997]`(0.50)，
  即调度只让增益动 2–31%，且都是稳定增益。
- `du/dH`：同一状态、H 从 0.40000 → 0.40025（一拍的高度变化）时 u 从 17.713 → 17.718 Nm，
  即 **0.92 Nm/s 的连续漂移**，没有跳变 ⇒ 逐拍重调度不会产生执行器阶跃。
- 准 LTV 闭环仿真（frozen scheduling，把 `theta_eq(H)` 的移动建模为 pitch 误差的瞬时参考跳变，
  植物按 `H+0/0.01/0.02` 偏置演化，0.40→0.30→0.50→0.40）：峰值 `|theta_err| = 3.3 mrad`、
  峰值 `|u| = 0.05 Nm`、饱和 0%、非 `solved` 拍 0，且模型偏置 20 mm 下数值几乎不变。
  ⚠️ 这只是**线性模型层面**的适定性检查（控制器与植物同源），不等于整机；它与 §10 的 Gazebo 实测
  对照才有意义（实测过渡峰值 2.5–2.95 mrad，与该预测同量级）。

## 7. 固定 H=0.40 回归（第 7 条）

改码前先构建当前树（ctest 20/20，含 MPC 三项），跑 v1 基线 6 条；改码 + `-O2` 后用同一 harness、
同一协议（`torque_pid_initial_roll:=0.0`、`spawn_z = startup_hip_axle + 0.14 = 0.50`、R=8、N=20、
θ 带 0.20 rad、22 s）跑 6 条。

| 工况 | 版本 | pitch RMS(°) | pitch 峰值(°) | x_err RMS(m) | x 末段均值(m) | u RMS | u 峰值 | 饱和% | 回退 | it_max | solver mean |
|---|---|---|---|---|---|---|---|---|---|---|---|
| static | v1 | 0.281 / 0.231 | 3.53 / 2.87 | 0.0255 / 0.0331 | 0.0006 / 0.0010 | 1.39 / 0.55 | 10.82 / 9.08 | 0 | 0 | 2 | 1399 / 1402 µs |
| static | v2 | 0.265 / 0.193 | 3.09 / 2.87 | 0.0301 / 0.0276 | 0.0009 / 0.0006 | 0.50 / 0.58 | 9.57 / 10.16 | 0 | 0 | 2 | 43.4 / 44.4 µs |
| position | v1 | 0.280 / 0.195 | 3.53 / 2.87 | 0.1235 / 0.1247 | 0.0366 / 0.0374 | 1.37 / 0.66 | 10.82 / 12.11 | 0 | 0 | 2 | 1381 / 1392 µs |
| position | v2 | 0.318 / 0.303 | 3.53 / 3.85 | 0.1274 / 0.1278 | 0.0379 / 0.0376 | 1.17 / 0.61 | 13.36 / 12.39 | 0 | 0 | 2 | 44.4 / 46.0 µs |
| push | v1 | 0.438 / 0.448 | 2.90 / 3.53 | 0.0677 / 0.0681 | 0.0174 / 0.0171 | 0.58 / 0.42 | 11.32 / 7.33 | 0 | 0 | 2 | 1373 / 1399 µs |
| push | v2 | 0.446 / 0.437 | 3.53 / 3.53 | 0.0674 / 0.0673 | 0.0175 / 0.0175 | 0.47 / 0.37 | 7.09 / 7.09 | 0 | 0 | 2 | 45.3 / 44.4 µs |

- 6/6 两版都没摔、`stages` 都只有 `solved`、0 次回退、0 拍饱和、迭代上限都是 2。
- **theta_eq**：两版末段都精确 `0.052626 rad`；末 6 s pitch RMS 都是 0.005°。
- **A/B 的间接验证**：`u_mpc` 与同权重单步 LQR `u_lqr_fallback` 的相对偏差（只在未饱和拍上算）
  v1 中位 14.3–16.2%、v2 中位 15.1–16.2%，分布同档 ⇒ N=20 首动增益相对 DARE 增益的固有差没变。
- **每拍重建 vs 只配置一次**：单元测试 300 拍闭环 `max|Δu| = 1.2e-14 Nm`、stage 差异 0；离线 400 拍
  `max|Δu| = 7.1e-14 Nm`。冷启动（`configure()` 会重置 warm start）不改变解，只改求解耗时。
- 唯一明显变化是 **solver 时间 1.38 ms → 44 µs**，这是 `-O2` 的直接后果（§8），属于改善。
- 掉拍情况（`dt>5.4 ms` 占比）：v1 1.4 / 0.5 / 3.1 %，v2 6.0 / 0.1 / 0.1 %。样本太小，不能判定谁更好；
  方向上 `-O2` 只会减少每拍工作量。**未观察到固定 H=0.40 的行为被明显改变**，故按第 7 条继续。

## 8. rebuild 耗时与编译口径（第 6 条的实测依据）

同一份 MPC 核心、H=0.40、3000 次重复（**离线**，单位 µs）：

| 阶段 | -O0（仓库默认，`CMAKE_BUILD_TYPE` 为空） | -O2（本次对 MPC 目标） |
|---|---|---|
| `continuous_matrices` A/B | 1.6 | 0.1 |
| `zoh_discretize`（5×5 expm） | 48.1 | 0.9 |
| `solve_dare`（Kleinman，6 pass，含 16×16 LU） | 1102.7 | 14.9 |
| 预测矩阵 + Hessian + 约束行 + QP 对象（`configure`） | 2010.7 | 44.8 |
| **每拍重建合计** | **3163.1**（p99 3342，max 4262） | **60.7**（p99 97.7，max 193.1） |
| QP `solve` | 2045.4 | 36.3 |
| 重建 + solve 对 5 ms 预算 | **104.2%** ← 超预算 | **1.9%** |

在线（节点 `model_rebuild_us` 列，27 条日志）：**mean 86–104 µs、p95 ≈150 µs、max 278 µs**；
`solver_time_us` mean 40.6–53.1 µs（v1 同列 1372–1402 µs）。

⇒ 结论：**不做高度阈值、不做缓存步长、不新增插值网格**，直接每拍按 `current_height` 精确重建，
代价是把 `-O2` 加到这个目标上。理由：-O0 下 3.16 ms 重建 + 1.47 ms 求解 ≥ 5 ms 周期，而现网日志的
`dt` 已经是 mean 5.12 ms 并有掉拍，再加 3.2 ms 会让有效控制周期恶化而模型仍按 5 ms 离散。
"预计算高度表 + 插值"因此没有必要（它会让模型不再等于 `current_height`）；两条备选路线在实测面前被淘汰。

口径影响：`solver_time_us` / `model_rebuild_us` 两列与冻结 v1 不可直接比（v1 是 -O0），其余列可比。
其他控制器（adaptive / gs_lqr / jump / PID）的编译口径未变。

## 9. 五个固定高度覆盖（第 8 条）

**实测**：`--trials static --repeats 2`，每个高度令 `height.startup_hip_axle = 目标`（故 `spawn_z = H+0.14`），
即**不带过渡**的真·定高，22 s。全部 10 条：没摔、`stages` 只有 `solved`、`fallback_events=0`、
`model_rebuild_failed=0`。

| H (m) | 到达误差 | model−current | theta_eq 规则差 (rad) | 末 6 s pitch RMS(°) | 末 6 s x_err (mm) | 全程 u RMS / 峰值 (Nm) | 饱和% | it_max | rebuild mean/max (µs) |
|---|---|---|---|---|---|---|---|---|---|
| 0.30 | 0.0000 | 0.0 | 2.1e-08 | 0.0071 / 0.0070 | −1.8 / −1.5 | 0.31 / 7.74 | 0.00 | 2 | 86 / 185 |
| 0.35 | 0.0000 | 0.0 | 2.9e-09 | 0.0062 / 0.0062 | −2.2 / −2.2 | 0.42 / 6.81 | 0.00 | 2 | 94 / 278 |
| 0.40 | 0.0000 | 0.0 | 3.7e-08 | 0.0046 / 0.0046 | +1.9 / +1.9 | 0.48 / 7.90 | 0.00 | 2 | 94 / 208 |
| 0.45 | 0.0000 | 0.0 | 2.9e-08 | 0.0047 / 0.0046 | −3.0 / −2.5 | 2.57 / 20.00 | 0.80 / 0.50 | 71 / 14 | 97 / 202 |
| 0.50 | 0.0000 | 0.0 | 2.8e-08 | 0.0040 / 0.0040 | −2.9 / −2.9 | 2.30 / 20.00 | 0.80 / 0.80 | 118 / 87 | 96 / 174 |

逐项对第 8 条：
- **实际腿高达到目标**：`current_height − target_height = 0.0000`；gz 真值 `base_link z − 0.14 − H`
  在末 4 s 稳定为 +3.2 ~ +7.6 mm（与第一阶段记录的有效滚动半径口径 ~8.7 mm 一致，不是腿没到位）。
- **theta_eq(H) 正确**：≤ 5e-8 rad。
- **模型用的 H 与 current_height 一致**：全部受控拍逐位为 0。
- **稳定平衡**：末 6 s pitch RMS ≤ 0.007°、|theta_err| 峰值 ≤ 1.3e-4 rad、x_err 末段 |均值| ≤ 3.0 mm。
- **QP 正常收敛 / 无异常 fallback**：`stages` 只有 `solved`，`fallback_events = 0`，全 27 条日志 0 拍回退。
- **力矩没有长期异常饱和**：0.45/0.50 出现的 `u=20 Nm`（总上界）**全部集中在接管后 0.13–0.52 s 的
  出生/接触瞬态**（40 / 49 拍，占全程 0.7–0.8%），1.5 s 之后饱和拍数为 0，末 6 s `|u|` 峰值 = 0.00 Nm。

需要如实记录的一点：0.45/0.50 的**出生瞬态明显更大**（H=0.50 时 `|theta_err|` 峰值 0.268 rad、
pitch 掉到 −0.237 rad、软俯仰带用了 0.080 rad 松弛但仍是 `solved`；H=0.30 峰值只有 0.073 rad）。
这是协议初值/腿部开环位置接口在长腿几何下的着地冲击，**不是**稳态问题（末 6 s 收敛到 4e-5 rad），
本阶段按第 11 条"只验证不调参"处理，未针对性调整 `spawn_z`、初倾角或 Q/R。

## 10. 动态升降 0.40→0.30→0.50→0.40（第 9 条）

**实测**：一条 33.8 s 日志，6456 个受控拍，`leg_transition_speed = 0.05 m/s`。
记录列齐备：`time, target_height, height(current), model_height, theta_eq, x_error, x_dot, pitch,
theta_error, pitch_rate, u_mpc, tau_each, total/wheel_torque_saturated, solver_status,
solver_iterations, solver_time_us, model_rebuild_us, model_rebuild_failed, gt_pose_z`（其余列不变，
新列一律追加在表尾；所有消费者按 `csv.DictReader` 列名读取，故列数变化安全）。

| 段 | 命令(sim) | 目标确认 | 到达 | 域内时长 | 期望 | model−current | 过渡中 |theta_err| 峰值 | x_err 峰值 | u 峰值 | 饱和 | 非 solved | 重建失败 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.40→0.30 | 6.349 | 7.369 | 9.429 | 2.06 s | 2.00 s | 0.0 | 2.79 mrad | 32.8 mm | 0.050 Nm | 0 | 0 | 0 |
| 0.30→0.50 | 13.946 | 14.971 | 19.231 | 4.26 s | 4.00 s | 0.0 | 2.51 mrad | 31.5 mm | 0.078 Nm | 0 | 0 | 0 |
| 0.50→0.40 | 23.758 | 25.086 | 27.222 | 2.14 s | 2.00 s | 0.0 | 2.95 mrad | 17.2 mm | 0.070 Nm | 0 | 0 | 0 |

- `model_height` 全程等于 `current_height`（差 0.000e+00），而它与 `target_height` 最大差 **0.1997 m**
  ⇒ 模型没有按 target 提前切换。
- 过渡期间 |pitch 误差| 峰值 **2.5–3.0 mrad（0.14–0.17°）**，与 §6 离线 frozen-scheduling 仿真预测的
  3.3 mrad 同量级；u 峰值 ≤ 0.08 Nm（转矩几乎没用），0 拍饱和、0 拍回退。
- x 方向：过渡段 x 位移 15.5 / 43.6 / 26.5 mm，x_error 在 ±33 mm 内；这是位置慢模态（τ=7.07 s，
  第一阶段已定性）对参考移动的响应，不是失稳；整条日志 x_err RMS 0.027 m、末段回到 +2.5 mm。
- 腿部真值滞后（`gt_pose_z − 0.14 − H`）：静稳 +5.3 mm；下降段 +7.8~+11.1 mm（机身还在高处、腿在往下追），
  上升段 −0.2~+4.8 mm（机身略低于指令） ⇒ 开环位置接口在 0.05 m/s 下约 ±4 mm 跟随滞后。
  全局最大 34.6 mm 出现在 t=0.122 s（接管前的着地沉降），不属于升降段。
- 全程 `model_rebuild_us` mean 101.9 / p95 150 / max 196.6 µs，`solver_time_us` mean 45.8 µs、
  迭代上限 2 ⇒ 重建 + 求解 ≈ 148 µs，占 5 ms 的 3.0%。
- `dt` mean 5.226 ms、max 11 ms：仍有掉拍（与 v1 同现象，非本阶段引入）。

## 11. 当前限制

1. **frozen scheduling 的域内失配**：域长 0.1 s 内 H 最多走 5 mm，模型按第 k 拍的 H 冻结。本阶段
   没有把这一失配写进约束（俯仰带 0.20 rad 是定值），也没有做鲁棒/ tube 处理。
2. **Q/R/N 未随高度重调**（按第 11 条禁止项执行）。因此不同高度上"同一组权重"表现并不等价：
   实测 0.45/0.50 的出生瞬态与迭代数（it_max 14–118）明显大于 0.30–0.40（it_max 2），
   稳态则都收敛到 <0.01°。
3. **位置沉降仍是 7.07 s 慢模态**：升降期间 x_error 摆到 ±3 cm 后缓慢回来；加高度调度不改变这一点，
   因为它与 R/权重有关而不是模型高度。
4. **可调度范围被两件事界定**：`/target_height` 限幅 [0.30, 0.50]（表列与 adaptive 一致），
   以及 `bbot_kinematics` 内部 `dZ_down` 的 [0.10, 0.60] 限位。区间内 IK 未被夹紧（0.50 m 时
   两连杆 0.343+0.300 仍有裕度）。
5. **腿部是开环位置接口**：真实腿高有 ±4 mm 跟随滞后与 +5 mm 接触口径偏差；MPC 不知道腿部自己的
   动力学（没有把 `H_dot` 作为状态或前馈）。
6. **没有进入 QP 的高度相关约束**：例如 `dH/dt` 上限、腿关节力矩/行程、高度与轮子接触的组合约束，
   本阶段都只靠节点侧的限幅和保护。
7. **`solver_time_us`/`model_rebuild_us` 与冻结 v1 不同口径**（-O0 vs -O2）；掉拍率样本不足，未定论。
8. 高个（0.45/0.50）的出生瞬态峰值 0.13–0.27 rad 已接近但未触及 0.50 rad 硬门限；若以后把
   `startup_hold_time`、`spawn_z` 或初倾角改掉，这个量需要重测。

## 12. 有没有必要下一阶段升级成 LTV-MPC？

**按目前的证据：不必要。** 理由（实测 + 离线）：

- frozen scheduling 已经把重调度放在"下一拍用新模型"上：一拍的 ΔH 只有 0.25 mm，实测过渡期间
  pitch 跟踪误差 2.5–3.0 mrad、u 峰值 ≤ 0.08 Nm、0 拍饱和/回退 —— 域内恒定假设在这套几何与
  0.05 m/s 下**根本没有成为瓶颈**，LTV 没有可兑现的收益目标。
- 该 2.5–3.0 mrad 与用同一 frozen 假设做的线性准 LTV 仿真预测值（3.3 mrad）对得上，说明"过渡误差"
  目前由 `theta_eq(H)` 参考移动主导，而不是由域内 `Ad/Bd` 变化主导 —— 换 LTV 预测的正是后者。

**什么时候需要重新评估**（触发条件，写清楚以免变成凭感觉升级）：
1. `leg_transition_speed` 显著大于 0.05 m/s（域内 ΔH 不再是 mm 级）；
2. 想要**主动**利用预测腿高（例如提前预压/预放来换取更好的捕获或跟踪），那需要
   `H(k+i)` 轨迹进入模型，属于真正的 LTV/时变参考；
3. 把俯仰带或接触约束做成随高度收紧（约束不可行性开始由域内模型失配引起）。

比 LTV 更靠前的两件事（都属于"另批走调参→冻结→正式对比"，本阶段没做）：
- 高个（0.45/0.50）出生/接管瞬态的协议处理（`spawn_z`/初倾角/带松弛的取值）；
- 若要提位置沉降速度，先按第一阶段定的顺序做 `Q_x` 的离线谱分析，而不是加长时域或加积分器。
