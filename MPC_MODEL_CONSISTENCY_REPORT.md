# MPC 模型一致性核查报告（H / base_link / hip-axle / COM / IK / LQR 模型高度 / theta_eq）

日期：2026-09-28　范围：只做定义与数值溯源，**未修改任何代码**。
所有行号：`src/bbot_description/urdf/bbot.urdf.xacro`（简称 URDF）、`src/bbot_kinematics/`（简称 KIN）、`src/bbot_balance_controller/`（简称 BC）、`src/bbot_bringup/launch/bbot_gazebo.launch.py`（简称 LAUNCH）。

---

## 0. 结论摘要：三个 theta_eq 的来源已全部对上

| 数值 | 它到底是什么 | 证据强度 |
|---|---|---|
| **0.052626 rad** | 几何式 `-atan2(y_com, z_com)` 在 **hip-axle 高度 = 0.40 m** 处的值。**这才是 H=0.40 的物理零力矩平衡角** | 推导 + 实测确认（见 §5） |
| **0.087823 rad** | 同一个几何式在 **hip-axle = 0.40 − 0.14 = 0.26 m** 处的值。GS-LQR 表列 θ_eq(H) 恒等于 `geo(H − 0.14)`，5 行全部到 3e-7 | 恒等式证明（§5.1） |
| **0.0145 rad** | ≈ `0.052626 − 0.0381`。是**仿真初值预倾角** `torque_pid_initial_roll = −0.038`（LAUNCH:333-334）造成的静止姿态，**不是物理平衡点** | 判别实验（§5.3） |

也就是说：**0.0526 与 0.0878 的差是同一个正确公式被喂了两个不同的高度（差 0.14 = base_to_hip 0.07 + wheel_radius 0.07）；0.0145 与前两者的差是初始姿态污染，不是建模公式的差。**

对 MPC 的直接含义：我之前把 0.0145 当"标定值"写进 `config/mpc_balance_params.yaml` 与 launch 默认，属于**对初值/非理想项的补偿**，应当改回几何值 0.052626 并把预倾角置 0 —— 这条已在 2026-09-28 执行，见 §0.1 与 §8.1。

### 0.1 对齐基准的最终选定（2026-09-28 执行）

用户指定 **`adaptive_lqr_balance_controller` 为对齐基准**（不是 GS-LQR）。核查结果：adaptive 的 H 语义自洽——`adaptive_lqr_balance_controller.cpp:180-186` 明确写"height means the vertical distance from the wheel axle to the hip joint"，`base_link_height()=H+base_to_hip+wheel_radius`（:859-867）后才送 IK（:635），θ_eq 用 `-atan2(y_com,z_com)`（:603,:615）并按 `current_height_` 每拍重算（:717-719）。H=0.40 处该式 = **0.052626 rad**，H=0.36 处 = **0.061667 rad**（与本报告 §0 的几何值一致；GS-LQR 的 0.0878 列属于 §10 那个"H−0.14"语义，与 adaptive 无关）。

MPC 侧据此改了四处（只动 MPC，未动任何既有控制器）：增益表列换成 adaptive 的 y_com/z_com 并按它的插值规则取 θ_eq、θ_eq 随指令高度每拍更新、高度参数名/启动语义（`height.hip_axle_min/max`、`height.startup_hip_axle=0.36`、`leg_transition_speed=0.05`、`height.startup_hold_time=2.0` 只延后伸腿而不拦力矩）、以及 adaptive 的仿真时钟 tick 门限（:668-691）。默认 `theta_eq_source` 从 `override 0.0145` 改为 `table`。

实测（无头 Gazebo，H=0.40，`torque_pid_initial_roll:=0.0`，spawn base_link 0.50 m = 启动髋-轴 0.36 + 0.14，launch 的就绪门接管；每条的首个受控拍 0.012–0.064 s）：

| 工况 | 结果 | θ_err rms | θ_err 峰值 | x_err 尾部 | u rms | 饱和 | 备注 |
|---|---|---|---|---|---|---|---|
| 静止平衡 | 站住 | 0.201° | 2.87° | +0.6 mm | 0.85 Nm | 0.0% | solver mean 1.39 ms（非优化编译），无 fallback |
| 位置回位（+0.3 m 阶跃） | 站住 | 0.279° | 2.87° | +36.8 mm | 1.96 Nm | 0.0% | 阶跃 9.78 s，误差峰值 0.3165 m，之后 |e| 均值 86.1 mm；**一阶段无积分项 ⇒ 留厘米级残差** |
| 给定速度 0.15 m/s | 站住 | 0.416° | 3.53° | +20.8 mm | 1.53 Nm | 0.0% | 巡航窗 7.28–14.13 s：实测 0.1612 m/s（偏差 3.3 cm/s ≈2.2%），行驶 1.125 m，平均俯仰 0.0028 rad |
| 外力冲击 20 N×0.2 s | 站住 | 0.438° | 2.87° | +17.0 mm | 0.56 Nm | 0.0% | `pulse_valid=True`；姿态峰 2.84°、x_err 峰 0.186 m、力矩峰仅 2.33 Nm |
| 力矩饱和（轮 ±0.5 Nm） | 按预期摔 | 14.97° | 29.36° | — | 0.99 Nm | 98.8% | **u_max 恰好 1.0 Nm ⇒ 约束真的生效**；随后 0.5 rad 安全门断开，fallback 链只触发 1 次 |

稳态位置偏置从旧 θ_eq=0.0145 下的 1.3–2.5 m 变成毫米级（静止 0.6 mm），直接印证 §5.2 的偏置定律。**已知代价**：仓库默认非优化编译下求解 mean ≈1.4 ms、常规工况 max ≈2.3 ms（5 ms 周期内），但饱和工况出现过一拍 **37 ms**（工作集 45 次迭代），超限后 tick 门限会丢弃一拍。

**三条只有实跑才暴露的坑（第一版归因错过一次，已更正）：**
1. **spawn 高度必须等于节点在拿到 `/imu` 之前指令的姿态高度**。控制器的 disabled 分支每拍发布腿部位置，而腿部 `position` 接口是开环的。沿用仓库通用 spawn 0.403 m（= 关节零位的地面静止高度，髋-轴 0.2577 m）时，节点一上来就把髋-轴指令到 0.36 ⇒ 机身比 spawn 高 0.10 m，实测接管第一拍 pitch 已 −0.573 rad、随后翻到 −2.07 rad、CSV 里 **0 个受控行**。`run_mpc_balance_trials.py` 因此默认 `mpc_spawn_z = height.startup_hip_axle + 0.14`。反过来试着"接管前不发腿部指令"更糟：接管那一拍要在闭环里瞬间跨 0.10 m，实测 67% 的拍子顶在 ±20 Nm 限幅上、0.56 s 就摔。
2. **`/imu` 迟到是启动时序竞态，不是控制器的结果；仓库里已有解药**。同一份配置 9 条里 4 条未接管就摔：站稳的那些首个受控拍在 sim **t ≈ 0.01–0.07 s**，摔的那几条拖到 **2.7 s / 9.99 s**，此时机器人早已向后倒在背上（pitch −2.0455 rad、**0 个受控行**）。根因：launch 的 `auto_unpause` 是**固定 2.8 s 计时器**（`bbot_gazebo.launch.py` 的 `auto_unpause_action`），与控制器/传感器是否就绪无关；而 spawner 激活又需要仿真步进。**解法（已接）**：把 `mpc` 加进 `gs_lqr_ready_unpause` 的条件（该脚本会单步暂停的世界直到三个控制器 active 且 `/imu`、`/joint_states` 真在发才 unpause），并从固定 2.8 s unpause 的适用集合里排除 `mpc`；trial 脚本改用 `gazebo_start_paused:=true auto_unpause:=true`。接好后 5/5 条的首个受控拍 0.012–0.064 s。
3. **批次必须从 source 过的 shell 起**。从没 source 的 shell 起时 `ros2 topic pub` 全部静默失败（`which ros2` 能命中，但 ros2cli 要 setup.bash 设的 PYTHONPATH/AMENT_PREFIX_PATH），于是位置阶跃/给定速度/外力三个工况的指令根本没发出去，CSV 却照样漂亮（`step_time_s=None`、`cruise_window_s=None`、`pulse_valid=False`）。`run_mpc_balance_trials.py` 现在起跑前用 `ros2 node list` 自检、`publish()` 检查返回码并抛错。


### 0.2 追加实测（§10）：GS-LQR 在 H=0.40 时真实髋-轴 ↔ 轮-轴距离

问题"GS-LQR 设 H=0.40 m 时，Gazebo 中髋关节轴到轮轴的距离是不是 0.40 m"的答案是 **不是，实测 0.260000 m**（GS-LQR 自己发布的指令关节角经 URDF 链正算）；gz 位姿真值给出 0.268683 m，两者相差 8.7 mm，属于 §7-E3 的有效滚动半径口径问题，不影响结论。也就是说 GS-LQR 该设定下的真实摆长是 `H − 0.14`，这使 §5.1 的恒等式 `θ_eq_table(H) = geo(H − 0.14)` 由"公式吻合"升级为"实测支撑"。**GS-LQR 未修改；MPC 的对齐基准已定为 adaptive（§0.1、§8.2），本节只作 GS-LQR 侧的事实留档。**

---

## 1. 坐标系、正方向、角度定义（逐条给出处）

### 1.1 URDF / gz 世界
* 机体三轴：**X = 轮轴方向（左右）**，六个运动关节 axis 全部是 `1 0 0`（URDF:2415/2423/2431/2439/2447/2455）；**Y = 前后（前为正）**；**Z = 上为正**。
* 俯仰（机器人学意义的 pitch，前后倒摆）= **绕 X 轴旋转**。IMU 与 base_link 之间 `rpy="0 0 0"`（URDF:2632），所以 IMU 的 x 轴与 base_link 的 x 轴平行 → 读出的 roll 就是绕机体 X 的转角，**无安装偏置**：
  ```
  pitch_controller = -roll_imu            // BC/src/lqr_gain_scheduled_controller.cpp:539
  pitch_rate       = -omega_x_imu         // 同文件 :541
  ```
  即"物理前倾 = 正 pitch"。IMU 发布 100 Hz（URDF:2635-2641）。
* 轮子正负方向：轮关节反转 = 机器人前进。位移/速度估计（GS-LQR 与 MPC 完全相同）：
  ```
  x_dot = -R * (q̇_004 + q̇_007) / 2      // BC/src/lqr_gain_scheduled_controller.cpp:638-639
  x     = -R * (Δq_004 + Δq_007) / 2     // 同文件 :666-667
  ```
  力矩指令符号：`tau_each = -0.5 * u_model`（同文件 :1039-1052），即"模型里 +u = 前进"，而 gz 里"负 effort = 前进"。

### 1.2 两个高度的定义（关键）
| 名称 | 定义 | 出处 |
|---|---|---|
| `H_hip`（hip-axle 高度） | **髋关节中心在轮轴上方的高度**；= IK 内部的 `dZ_down` | KIN/src/kinematics.cpp:77 |
| `z_base`（base_link 离地高度） | base_link 原点离地高度 = `H_hip + 0.07 + wheel_radius` | KIN/src/kinematics.cpp:74-77；BC/src/adaptive_lqr_balance_controller.cpp:60,859-867 |

0.07 这个"髋→base_link 原点"的竖直偏移来自 URDF 髋关节原点 `z = -0.07`（URDF:2414、:2438，左右两条腿都是 −0.07）。
`wheel_radius = 0.07` 来自轮碰撞体 `<cylinder radius="0.07">`（URDF:2295、:2407）与 `KIN/include/bbot_kinematics/robot_params.hpp:14`。

### 1.3 `inverse_kinematics(target_z, body_pitch)` 的入参语义
`target_z` **就是 z_base（机身离地高度）**，函数内部先减 `(0.07 + wheel_radius)` 得到 `dZ_down = H_hip`，再解两连杆余弦定理（KIN/src/kinematics.cpp:70-116）。夹取范围 `dZ_down ∈ [0.10, 0.60]`（:78-81）。
平面内两段长度与 URDF 关节原点严格一致：
* 大腿 `‖(-0.29348091, -0.06220095)‖ = 0.30000` = `RobotParams.l2`（URDF:2422/:2443 的 y,z；KIN robot_params.hpp:16）
* 小腿 `‖(0.28210870, -0.19553796)‖ = 0.34325` = `RobotParams.l1`（URDF:2430/:2451；robot_params.hpp:15）
* 髋→轮轴的水平偏置 `target_dY = -0.01137221`（KIN/src/kinematics.cpp:83，注释写明 = 0.11363 − 0.1250），与 URDF 推得的 `y_wheel − y_hip = 0.11362779 − 0.125 = -0.01137221` **完全相等**。

---

## 2. 每个调用点实际用的 H 语义（这就是分歧的根源）

| 代码位置 | 传给 IK 的值 | 该值语义 | 表/模型索引用的 H | 两者是否同一量 |
|---|---|---|---|---|
| BC/src/adaptive_lqr_balance_controller.cpp:635 | `base_link_height()` = `H + 0.14` | z_base | 增益表/几何表的 `H`（=H_hip） | **一致 ✅** |
| BC/src/balance_controller_keyboard.cpp:525 | `base_link_height()` | z_base | 同上 | **一致 ✅** |
| BC/src/lqr_gain_scheduled_controller.cpp:812 | `current_height_` 裸值 | 被当成 z_base | 表行 `H`（=H_hip） | **不一致 ❌（差 0.14）** |
| BC/src/lqr_balance_controller_yaokong.cpp:566 | `current_height_` 裸值 | 同上 | 其内置 2 行增益表 | 同样不一致 ❌ |
| MPC（新增，BC/src/linear_mpc_balance_controller.cpp） | `base_link_height()` = `H + 0.14` | z_base | `mpc_target_height` = H_hip | 一致 ✅（跟随 adaptive 约定） |

结论：**adaptive/PID 一系的 H 是自洽的；GS-LQR 一系的"高度"变量在"表索引"与"IK 入参"之间错位 0.14 m。**

---

## 3. H_hip = 0.40 m 时所有几何量的实际数值

计算链：URDF 关节/惯质原点 → 与 q=0 帧联立的平面链 → 质心/惯量 → A、B（脚本 `BC/scripts/verify_lqr_model_and_sweep.py:design_matrices()`，与 C++ 移植 `BC/include/bbot_balance_controller/lqr_plant_model.hpp` 数值一致）。

| 量 | 数值 | 来源 |
|---|---|---|
| 机身质量 m0(body) | 9.50 kg | URDF:26 `$(arg body_mass)` 默认 9.5（:6） |
| 大腿 1.20 / 小腿 0.80 / 轮 2.00 kg（各×2） | 1.20/0.80/2.00 | URDF:2190/:2213/:2284 |
| 悬挂质量 `M_B` = body+2×thigh+2×shank | **13.5000 kg** | = robot_params m1+m2+m3 = 1.6+2.4+9.5 |
| 整机质量（不含 IMU 0.01） | 17.50 kg | 求和；IMU 0.01 在 URDF:2623 |
| `M1` 等效质量 = M_B+2·m_wheel+2·J_wheel/R² | **20.145306 kg** | R=0.07, J_wheel=ixx=0.006481（URDF:2285） |
| 质心（轴系，+y 前、+z 上） | `y_com = −0.0234056 m`，`z_com = +0.4443432 m` | 与部署表 `BC/src/adaptive_lqr_balance_controller.cpp:299-303` 的 y_com/z_com 列一致；C++ 移植复现到 2.5e-8 |
| 摆长 `l = \|com\|` | **0.4449592 m** | |
| 悬挂惯量 `I_com` | 0.4689014 kg·m² | 各连杆 ixx + 平行轴：J_BODY=0.159013×(9.5/14)=0.107902（URDF:8,:27），J_thigh=0.017921（:2191），J_shank=0.013130（:2214） |
| `m2 = I + M_B l²` / `m3 = M_B l` / `Δ = M1·m2 − m3²` | 3.1417490 / 6.0069495 / 27.20805 | |
| `mgl` | 58.928175 N·m | g=9.81 |
| `A(1,2) = −m3·mgl/Δ` | **−13.010067** | |
| `A(3,2) = M1·mgl/Δ` | **+43.631426** | 开环 ω_n = √A(3,2) = 6.6054 rad/s = 1.051 Hz，1/ω_n = 0.1514 s |
| `B(1) = (m2/R+m3)/Δ` / `B(3) = −(M1+m3/R)/Δ` | **+1.870368 / −3.894394** | |
| ZOH(Ts=0.005) `Ad, Bd` | Bd = [2.338092e-05, 9.352897e-03, −4.868435e-05, −1.947551e-02] | 与 scipy 一致到 1e-11（BC/test/test_lqr_plant_model.cpp） |
| 关节指令（H_hip=0.40, pitch=0） | `q_hip = +0.34636423`，`q_knee = −0.52215638` | 与 `KIN::inverse_kinematics(0.54, 0)` 完全一致（单元测试断言 1e-9） |

### 3.1 base_link COM 的换系（−0.125 / +0.07 不是拟合，是关节原点）
URDF 里机身质心在 base_link 系为 `(0.20001846, 0.13261282, 0.05396677)`（:25），左髋在 `(0.3032, 0.125, −0.07)`（:2414）。把机身质心表达到**髋**系：
```
y: 0.13261282 − 0.125 = +0.00761282      z: 0.05396677 − (−0.07) = +0.12396677
‖·‖ = 0.12420  ≈ RobotParams.l3 = 0.124   （x 分量 0.20001846 是横向，平面模型丢弃）
```
同理 `C_THIGH = (−0.13690699, −0.02116697)` = URDF:2191 的 (y,z)，其模 0.13853 ≈ `x2c = 0.1385`；`C_SHANK = (0.11538205, −0.08532288)` = URDF:2213 的 (y,z)，其模 0.14355 ≈ `x1c = 0.1435`。
→ **Python 精确链、URDF、RobotParams 三者互相自洽**，KIN 的雅可比/重力矩用的是同一套距离（只丢掉了横向分量与连杆偏置角）。

---

## 4. theta_eq 的理论推导（不依赖任何拟合）

设机体（含双腿，关节角固定）相对竖直的倾角为 θ（前倾为正），悬挂质量对轮轴的位矢为 `(y_com, z_com)`，随机体一起转动：

```
x_com(θ) = y_com·cosθ + z_com·sinθ      （质心相对轮轴的水平坐标）
```

轮子只能提供 (i) 接触处接触力、(ii) 绕轮轴的驱动力矩。**若驱动力矩为 0**，整机对触地点的重力矩为
```
τ_g(θ) = −M·g·x_com(θ)          （触点在轮轴正下方，水平坐标相同）
```
平衡（τ_g = 0）要求 `x_com(θ) = 0`：
```
θ_eq = −arctan(y_com / z_com)                            …… 定义①（轮轴为参考）
```
用 H_hip=0.40 的数值：`−arctan(−0.0234056 / 0.4443432) = +0.052626 rad`。

线性化后 `ẍ₃ = (M1·mgl/Δ)·(θ − θ_eq) + …` 即 A(3,2) 作用在 **θ_err = θ − θ_eq** 上，这就是状态定义 `X = [x_err, ẋ, θ−θ_eq, θ̇]`（BC/src/lqr_gain_scheduled_controller.cpp:298-318；README_LQR_gain_scheduled.md 第 27 行）。

### 4.1 三种"参考点/质量口径"的差别（都在 H_hip=0.40 处算）

| 定义 | 公式 | H=0.40 的值 |
|---|---|---|
| ① 仓库/模型采用 | `-atan2(y_com, z_com)`，悬挂质量（不含轮） | **+0.052626** |
| ② 触地点为参考（不含轮） | `-atan2(y_com, z_com + R)` | +0.045474 |
| ③ 整机（含 2×2 kg 轮，轮质心在轴上） | `-atan2(y_tot, z_tot)` | **+0.052626**（轮在轴上 → 方向不变，与①相同） |
| ④ 整机 + 触点 | `-atan2(y_tot, z_tot + R)` | +0.043714 |

因为轮质心恰好在轮轴上（URDF:2283 inertial origin 的 y=z=0），①≡③；②/④ 与 ① 差 0.007–0.009 rad。仓库用的是 ①，报告里把它当作 H 的标准定义；②④ 的量级只有 0.5°，**不足以解释 0.038–0.088 rad 的分歧**，所以分歧必须另找原因（§5）。

---

## 5. 三个不一致数值的定量归因

### 5.1 GS-LQR 的 0.087823 = 定义①在 H−0.14 处（恒等，5/5 行）

| 表标签 H | 表列 θ_eq | ①在 H−0.14 | 差 | ①在 H | 差 |
|---|---|---|---|---|---|
| 0.30 | 0.118440 | 0.118440 | +8.8e-08 | 0.076757 | −0.0417 |
| 0.35 | 0.102967 | 0.102967 | −3.1e-07 | 0.064166 | −0.0388 |
| 0.40 | 0.087823 | 0.087823 | −3.0e-07 | 0.052626 | −0.0352 |
| 0.45 | 0.074136 | 0.074136 | +6.7e-08 | 0.041721 | −0.0324 |
| 0.50 | 0.061787 | 0.061787 | −2.0e-07 | 0.030969 | −0.0308 |

0.14 = base_to_hip(0.07) + wheel_radius(0.07)。这正是 §2 里 GS-LQR 把 `current_height_` 裸喂给 IK 造成的错位：**它的 θ_eq 列与"它真正飞的那个姿态"（H_hip = 表标签−0.14）自洽，而它的 K 行和 A/B 模型是按表标签 H_hip = 表标签设计的。**
表头注释（BC/src/lqr_gain_scheduled_controller.cpp:313-318 "theta_eq = -atan2(y_COM, z_COM)"）与 README_LQR_gain_scheduled.md:52 只写了公式、没写"喂哪个高度"，所以这个错位从注释上看不出来。

### 5.2 位置偏置定律（用于核对，不是用于猜定义）
闭环静态（u→0）下 `x_err = (k_θ/k_x)·(θ_eq − p0)`；比值实测/理论：

| 增益来源 | k_θ/k_x |
|---|---|
| 部署表 H=0.40 行 | 30.68 |
| DARE(Q0,R=1) | 34.42 |
| DARE(Q0,R=8) | 37.19 |
| 由两次无头运行反推（θ_eq=0.0878 vs 0.0526） | 32.55 / 33.76 |

对应实测：θ_eq=0.0878 → x_err 2.47 m；0.0526 → 1.33 m；0.0145 → 0.003 m。与定律一致（±3%）。

### 5.3 0.0145 的归因：预倾角 −0.038（判别实验，非拟合）
LAUNCH:333-334 `torque_pid_initial_roll` 默认 **−0.038**，在 :517 作为 spawn 的 `-R`（绕 X = 俯仰）。
把预倾角设 0、θ_eq 设成定义①在 H_hip=0.40 的值（0.052626）、其余为 MPC 默认（R=8, N=20, spawn z = H+0.14 = 0.54），实测（`/tmp/consistency/probe_roll0.csv`，t∈[20.8, 28.8] s 中位数）：

| 量 | 实测 |
|---|---|
| 控制器日志 pitch | **+0.052543** |
| gz 位姿真值 `/model/bbot/odometry` → roll(X)=−0.0262641 → pitch=−roll | **+0.052528** |
| 几何定义① | **+0.052626** |
| theta_error 中位数 | −8.3e-05 rad |
| x_error 中位数 | **+0.0012 m** |
| u 中位数 | +0.0007 Nm（≈零力矩） |
| 关节实测 vs 指令 | link_002 0.34636422622775664 vs 0.34636423（误差 4e-9），003/005/006 同；**无 droop** |
| 轮位姿 | 28 s 内轮仅累计 0.034 rad ≈ 2.4 mm 行程 |

三点结论：
1. **零力矩静止角 = 几何定义①**，三个独立来源（控制器日志、gz 真值、URDF 推导）互相符合到 1e-4 rad → 0.052626 是真正的物理平衡点。
2. 之前的 0.0145 = 0.052626 − 0.0381，偏移量的绝对值与默认预倾角 0.038 相同（且在 H=0.36 与 H=0.40 两次运行里都是 −0.0382/−0.0381，与几何无关）→ 它是**带预倾角起跳的静止姿态**，被 θ_eq=0.0145 "标定"进控制器 = 对初始条件的补偿。
3. **未解释清楚的部分（如实记录）**：预倾角只是初值，理想模型里 0.038 rad 的重力矩（A(3,2)·0.038 ⇒ θ̈≈1.66 rad/s²）应在 ~0.2 s 内把它拉回 0.0526，而那次运行却以 u≈0.01 Nm 稳定停在 0.0145（并付出 1.33 m 位置误差）。说明 sim 里存在一个模型未包含的、足以支撑 ~0.038 rad 姿态偏差的静阻力矩通路（候选：接触/摩擦参数 mu1=mu2=1.0（URDF:2570-2571）带来的等效滚阻、或 §5.4 的高度/半径不一致）。**这条不影响 θ_eq 的判定**（无预倾角时三条证据一致），但影响"稳态位置精度"，属未决项。

### 5.4 顺带查出的高度/半径问题
probe 中 gz 真值 base_link z = **0.5453180 m**，而指令链（H_hip=0.40 + 0.07 + 0.07）与 spawn 都应是 0.540 → **高 5.3 mm**；关节角实测完全等于指令（§5.3），故差异不在腿链，只能来自轮-地接触：若视为有效滚动半径，则 R_eff ≈ 0.0753 m，而代码/RobotParams 一律用 0.07。
影响量化：若 R_eff=0.0753，`M1` 19.786、A(3,2) +2.5%、B(1) −2.1%、B(3) −2.0%；且轮里程 `x = −R·Δq`、`x_dot` 会系统性偏小约 **7%**（x、x_dot 是状态量，直接进 QP）。
旁证：`src/bbot_bringup/config/bbot_controllers_effort.yaml:41` 用的正是 `wheel_radius: 0.075`，而主用的 `bbot_controllers.yaml:26` 是 0.07，URDF 碰撞体是 0.07，网格量出的轮胎半径 ≈0.0696-0.0698。**到底哪个是有效半径，我没有证据下结论**，需要专门量一次（量已知距离滚行的轮角，或直接量轮-地接触面）。

---

## 6. 判定：MPC 现在用的 theta_eq = 0.0145

**不是物理平衡点，是对其它建模/初值误差的补偿。** 具体补偿的是：
1. 首要成分（0.0381 rad）：`torque_pid_initial_roll = −0.038` 的初始姿态 + sim 里未建模的静阻能力（§5.3.3）；
2. 它**没有**补偿任何几何式错误——几何式①在 H_hip=0.40 就是对的，且被 gz 真值独立确认。

正确的 θ_eq 取法：`θ_eq = -atan2(y_com(H_hip), z_com(H_hip))`，H=0.40 → **0.052626**，同时验收时必须 `torque_pid_initial_roll:=0.0`（正式 campaign 早就是这么做的：BC/scripts/run_formal_pid_gslqr_campaign.py:414）。**2026-09-28 已按 adaptive_lqr 的同一规则落实到 MPC（§0.1），本节结论随之关闭。**

---

## 7. 定义与参数不一致清单（只记录差异与测量值；按指示不判定 GS-LQR 对错，未改任何代码）

| # | 差异 | 位置 | 影响 |
|---|---|---|---|
| E1 | **GS-LQR 送入 IK 的高度变量与表索引变量是同一个数**，而 IK 的入参语义是 base_link 离地高度（`kinematics.cpp:74-77` 先减 `0.07 + wheel_radius`）。实测：GS-LQR 在 H=0.40 时真实髋-轴距离 = **0.260000 m**（§10） | BC/src/lqr_gain_scheduled_controller.cpp:812（对照 :635 的 adaptive 约定） | 该节点飞的姿态对应 hip-axle=标签−0.14；其 θ_eq 列与这个真实姿态自洽（=geo(H−0.14)），其 K 行/A/B 按标签 H 设计（即摆长按标签 H 建模）。**基准已定为 adaptive（§8.2），GS-LQR 不修改** |
| E2 | 同一张 y_com/z_com 表在两节点得到的 θ_eq 不同：GS-LQR 存 0.0878（H=0.40），adaptive 现算 `-atan2(y_c,z_c)`=0.0526 | BC/src/lqr_gain_scheduled_controller.cpp:336-341 vs adaptive_lqr_balance_controller.cpp:603,732 | 两节点在"同一名义高度"下平衡点差 2.0°，稳态位置差 ≈34×Δθ（§5.2 实测符合） |
| E3 | **wheel_radius 口径**：URDF 0.07、RobotParams 0.07、`bbot_controllers_effort.yaml:41` 0.075；实测 gz base_link 高度比"髋-轴 + 0.14"高出 8.7 mm（§10），提示有效滚动半径 > 0.07 | URDF:2295/:2407；KIN robot_params.hpp:14；bringup config | 轮里程 x、x_dot 尺度与 B 矩阵受影响（§5.4 量化 ≈ 每 1 mm 半径 → B 约 −0.4%/mm）；有效半径未最终确定 |
| E4 | `KIN::forward_kinematics` 里 **`x_com = 0.0` 硬编码**，`z_com = z_hip + 0.07 + wheel_radius` | KIN/src/kinematics.cpp:63-65 | KIN 的 FK 算不出前后质心偏置 → θ_eq 只能来自 Python 精确链（或 IK 后的 URDF 链组合，如 §10） |
| E5 | 腿部 `position` 接口无重力前馈，唯一增益 `position_proportional_gain = 0.3`（URDF:5, :2464），但实测指令角与达成角一致（MPC 探针 4e-9 rad） | URDF:5/:2464 | 未见 droop；该增益的生效机制未在本轮查清，列为未决 |
| E6 | 力矩上限三套口径：URDF `<limit effort="150">`、ros2_control 髋 ±75/膝 ±60、轮 ±10；控制器/模型用轮 ±10 | URDF:2416/:2508-2511/:2524-2527/:2474-2477 | 与既有记忆一致，本轮复核仍成立 |
| E7 | 默认 spawn 预倾角 `torque_pid_initial_roll = -0.038`（正式 campaign 显式传 0.0） | LAUNCH:333-334, :517 | 定高度实验若不显式传 0.0，静止姿态会带 0.038 rad 初值污染（§5.3） |
| E8 | 仓库内无 `.m` 文件；`dare_gain(Q0,R0)` = [-6.231,-47.540,-214.496,-54.771] 与表列 [-5.932,-45.087,-181.973,-43.390] 差 5–18% | BC/scripts/verify_lqr_model_and_sweep.py:61-63, 321-355 | 可复现的是 A/B/几何（到 1e-11），K 的加权来源在仓库内不可复现 |

---

## 8. 处置状态（2026-09-28 更新）

1. **已执行**：MPC 的 θ_eq 默认改为 adaptive 规则（`theta_eq_source: table`，H=0.40 → 0.052626，按指令高度每拍更新），`config/mpc_balance_params.yaml` 与 launch 默认里的 `0.0145` override 已删除，`run_mpc_balance_trials.py` 显式传 `torque_pid_initial_roll:=0.0` 并把 spawn 高度对齐到启动姿态。**R 已在对齐配置下重扫并锁定**（见 §8.1）：旧扫描（−0.038 预倾角 + 错误 θ_eq + 有缺陷的 harness）的结论已全部作废。
2. **已决策**：对齐基准 = `adaptive_lqr_balance_controller`（用户指定），GS-LQR 不修改；本报告 §10 的 H−0.14 实测只作为 GS-LQR 侧的事实留档，不再作为 MPC 的输入。
3. **未做**：有效滚动半径测量（消 E3/§5.4）。
4. **未做**：消 E8（把 `LQR_K_new.m` 或其 Q/R、离散方式纳入仓库）。
5. **已完成**：MPC 五个工况（静止/回位/给定速度/外力/饱和）在对齐配置下重测，数值见 §0.1 的表；顺带修好了两条启动/发布链路的坑（§0.1 末）。
6. **已解决（原"仍未决"第 2 项）**：位置阶跃后厘米级"稳态残差"经查证**不是稳态误差**，而是 τ=7.07 s 的结构性慢模态在 16 s 窗口内的不完整收敛 + 聚合窗口平均的读数放大，定量闭合（解析 0.0889 m vs 实测 0.0855 m）；见 `MPC_POSITION_RESIDUAL_ANALYSIS.md`。§5.3 那个"带 −0.038 预倾角能以 u≈0 停住"的残余问题仍未决。

### 8.1 R 粗扫结论（2026-09-28，配置冻结，primary cohort n=3/格，28 格全部可评估）

| 区间 | 结论 |
|---|---|
| R = 0.5、1 | **不可用区**：四工况 primary cohort 全部 0/3 或关键工况 0/3（R=0.5 四格全 0/3），无可评估性能 |
| R = 2、4 | **高约束/高饱和区**：能站住但 position baseline 饱和 47–59%、push response 21–43%，且 static 有 hunting rep |
| R = 6、8、12 | **可用区**：四工况 3/3、饱和率 0%；**R=6 在 static 有 2/3 rep 处于 hunting regime**（θ_rms 0.25–0.30°、u_max 顶 20 Nm） |
| **R = 8** | **后续 nominal R**（唯一每个工况每一 rep 都安静、p95/p99 最低）。**不声明为全局最优**：最慢闭环极点 \|λ\|=0.999293（τ=7.07 s）在 R∈[1e-6,1e2] 上不变，故 R 对位置沉降无杠杆 |

产物：`/tmp/mpc_r_sweep_full/`（248 个原始 CSV 632 MB、`aggregate.md/json` 含 per-attempt ledger、`config_snapshot.md/json`、driver 的 `sweep_table.md`）；仓库内留档 `src/bbot_balance_controller/src/data_logs/mpc_r_sweep_20260928/`（小体积产物）。分析见 `MPC_POSITION_RESIDUAL_ANALYSIS.md`。

---

## 9. 复现方式

```bash
# 几何/模型数值（Python 权威链）
cd /home/admin/bbot_ws_new/src/bbot_balance_controller/scripts && python3 -c \
 "import verify_lqr_model_and_sweep as v,numpy as np; print(v.com_at(0.40)[0], v.design_matrices(0.40)[:2])"
# C++ 移植的黄金值一致性
ctest --test-dir build/bbot_balance_controller -R "lqr_plant_model|linear_mpc|dense_active_set_qp" --output-on-failure
# §5.3 的判别实验（预倾角 0 + θ_eq = 几何值，同时抓 /joint_states 与 gz 真值）
python3 /tmp/mpc_consistency_probe.py roll0
# 表列恒等式（§5.1）
python3 -c "import numpy as np,verify_lqr_model_and_sweep as v; [print(H, t, -np.arctan2(*reversed(v.com_at(H-0.14)[0][::-1]))-t) for H,t in zip([.3,.35,.4,.45,.5],[0.118440,0.102967,0.087823,0.074136,0.061787])]"
```

数据落盘：`/tmp/consistency/probe_roll0.{csv,json}`、`joint_states_roll0.yaml`、`odom_roll0.yaml`；`/tmp/mpc_trials_{a,b,c}/summary.json`（五个工况）。
本报告中所有"实测"都区分了：控制器日志 / gz 真值 / URDF 推导 三类来源。§10 的实测复现命令见 §10.4。

---

## 10. 实测复核：GS-LQR 在 H=0.40 时的真实髋-轴 ↔ 轮-轴距离

问题：**GS-LQR 设置 H=0.40 m 时，Gazebo 里"髋关节轴到轮轴"的距离是不是 0.40 m？**
答案：**不是，实测 0.260000 m。**（未修改 GS-LQR 任何代码；GS-LQR 的 CSV 路径在其源码里硬编码为 `$HOME/bbot_ws_new/...`（BC/src/lqr_gain_scheduled_controller.cpp:238-244），故把 HOME 重定向到 `/tmp/gslqr_home`，仓库归档 `gs_lqr_torque_log.csv` md5 复核前后均为 `a835bd6c7a3d60a5a4b5c8beb52962a9`。）

### 10.1 逐条核查要求的答案

| 待核查项 | 结论（含出处） |
|---|---|
| `current_height_` 在 GS-LQR 中的精确定义与赋值来源 | 声明 BC/src/lqr_gain_scheduled_controller.cpp:744；初值 `target_height_ = L_MAX_`、`current_height_ = target_height_`（:131-132），`L_MAX_ = robot_params.L_MAX = 0.50`（:107）；唯一其他写入点是 `/target_height` 订阅 `target_height_ = clamp(msg->data, L_MIN_, L_MAX_)`（:114-118）；每拍 `update_leg_height(dt)` 以 `leg_transition_speed_ = (L_MAX-L_MIN)/4 = 0.05 m/s`（:135）把 `current_height_` 移向 `target_height_`（:770-807）。**代码内没有把 `current_height_` 与 base_link 高度或髋-轴高度相互转换的任何语句**，它只被当作 IK 的第一个实参（:812）和增益表插值键（:946）。 |
| `inverse_kinematics()` 每个入参的精确定义 | 第 1 参 `target_z`：函数体 `dZ_down = target_z - (0.07 + p.wheel_radius)`（KIN/src/kinematics.cpp:77），注释 :74-76 明确写"target_z: 机身离地高度 / 髋关节到轮轴距离在竖直方向的投影 dZ_down"；随后夹取 [0.10,0.60]（:78-81）、`target_dY = -0.01137221`（:83，= URDF 的 0.11362779 − 0.125）。第 2 参 `body_pitch`：只以 `q_hip = phi1 - phi1_0 + body_pitch` 进入（:107），即机身俯仰角，前倾为正。 |
| IK 内部 `0.07 + wheel_radius` 的物理意义 | 0.07 = 髋关节原点在 base_link 系中的 `\|z\|`（URDF:2414 `origin xyz="0.3032 0.125 -0.07"`）；wheel_radius 0.07 = 轮碰撞圆柱半径（URDF:2295）。两者相加 = **base_link 原点 → 髋关节（0.07）＋ 轮轴 → 地面（0.07）**，因此 `target_z` 是 base_link 离地高度，`dZ_down` 是髋-轴 → 轮-轴的竖直距离。 |
| GS-LQR 在 H=0.40 实际得到的髋-轴距离 | **0.260000 m**。它的 `inverse_kinematics(0.40, 0.0)` 输出为 q_hip=+0.0058857576、q_knee=−0.0077823597。 |
| Gazebo 中由关节位置直接计算的实际髋-轴距离 | 见 10.2 的三路实测。 |

### 10.2 实测（Gazebo，headless，`/target_height = 0.40`，GS-LQR 自己日志确认 height 列 = 0.40000，5661 行）

髋-轴距离由 URDF 链直接组合而得（只用到 :2422、:2430 两个字面量向量与两个关节角）：
`hip_above_axle = −[ R(q_hip)·knee + R(q_hip+q_knee)·axle ]_z`

| 路径 | 输入 | 结果 |
|---|---|---|
| A：GS-LQR 发布的指令关节角（`/leg_position_controller/commands`） | q_hip=+0.0058858, q_knee=−0.0077824 | **0.260000 m**（髋在轮轴之前 +0.011372 m） |
| C：gz 位姿真值 `/model/bbot/odometry` 的 base_link z | z_base = 0.408683 m | 0.408683 − 0.07 − 0.07 = **0.268683 m** |
| D：GS-LQR 自身日志 height 列 | 0.40000 | 证明测量发生在 H=0.40 设定下 |
| 参照：关节零位 | q=0 | 0.257739 m（即 URDF 装配零位本身就是 0.2577 的近似伸直姿态） |
| 参照：MPC 探针（IK 入参 = H+0.14 = 0.54） | q_hip=+0.3463642, q_knee=−0.5221564 | **0.400000 m** |

A 与 C 相差 8.7 mm，方向与 §5.4 里 MPC 探针的 +5.3 mm 一致，来源指向接触/有效滚动半径（E3），不影响"0.26 而非 0.40"的结论。

### 10.3 对既有结论的影响
- 本报告 §5.1 的恒等式 `θ_eq_table(H) = geo(H − 0.14)`（5/5 行 ≤3e-7）**由实测支撑**：GS-LQR 的真实髋-轴距离就是 H−0.14 = 0.26，而 0.26 处的几何悬垂角正是表列 0.087823。
- 因此 §5.1 的"0.0878 vs 0.0526"分歧解释成立；按指示，本报告不再把该现象判为 GS-LQR 的定义错误，只在 E1 记录差异与测量值。
- 之前 MPC 的 0.0145 判定（§5.3、§6）**不受影响**：那是在 MPC 自身姿态（髋-轴 0.400，见 10.2 参照行）下测得，几何值 0.052626 与实测静止角 0.05253 相符。

### 10.4 复现
```bash
g++ -std=c++17 -O2 -I src/bbot_kinematics/include /tmp/audit_ik.cpp \
    src/bbot_kinematics/src/kinematics.cpp -o /tmp/audit_ik && /tmp/audit_ik 0.30 0.40 0.50
python3 /tmp/measure_gslqr_geometry.py        # Gazebo 实测，输出 /tmp/gslqr_geom/report.json
```
