# BBot 轮腿复合自平衡机器人 (Wheeled-Legged Robot) 控制系统

本项目为双轮足/轮腿式自平衡机器人（BBot）在 ROS 2 与 Gazebo 仿真环境下的动力学建模、运动学解算与高级运动控制平台。工作空间包含从底层执行器驱动、关节逆运动学（IK），到串级 PID、增益调度 LQR（GS-LQR）、自适应平衡点 LQR（Adaptive GS-LQR）以及多阶段爆发跳跃控制（Velocity Jump）的完整控制算法栈。

---

## 目录
- [1. 机器人与工作空间概览](#1-机器人与工作空间概览)
- [2. 编译构建与环境准备](#2-编译构建与环境准备)
- [3. 控制系统架构与算法全景](#3-控制系统架构与算法全景)
  - [3.1 控制算法选型对比](#31-控制算法选型对比)
  - [3.2 跳跃控制器族 (Jump Controllers)](#32-跳跃控制器族-jump-controllers)
  - [3.3 LQR 平衡控制器族 (LQR Family)](#33-lqr-平衡控制器族-lqr-family)
  - [3.4 增益调度 (Gain Scheduling) 与固定增益 (Fixed Gain) 消融机制](#34-增益调度-gain-scheduling-与固定增益-fixed-gain-消融机制)
  - [3.5 串级 PID 控制器族 (Cascade PID Family)](#35-串级-pid-控制器族-cascade-pid-family)
  - [3.6 运动学与动力学模块 (Kinematics)](#36-运动学与动力学模块-kinematics)
- [4. 仿真启动与参数配置](#4-仿真启动与参数配置)
  - [4.1 核心 Launch 参数表](#41-核心-launch-参数表)
  - [4.2 常用仿真启动指令](#42-常用仿真启动指令)
- [5. 键盘遥控终端与交互说明](#5-键盘遥控终端与交互说明)
  - [5.1 快捷键功能映射表](#51-快捷键功能映射表)
  - [5.2 核心 ROS 2 通信话题接口](#52-核心-ros-2-通信话题接口)
- [6. 仿真测试场与地形测试指南](#6-仿真测试场与地形测试指南)
- [7. 实验数据记录与分析脚本](#7-实验数据记录与分析脚本)
- [8. 核心源码架构索引](#8-核心源码架构索引)

---

## 1. 机器人与工作空间概览

BBot 为五自由度双轮腿机器人（每腿具备髋关节、膝关节，底盘包含两个独立驱动轮轴）。

### 关键物理与几何参数
| 物理量 | 符号 / 变量名 | 数值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| **小腿连杆长** | $l_1$ / `l1` | `0.30` | $\text{m}$ | 轮轴中心到膝关节轴心距离 |
| **大腿连杆长** | $l_2$ / `l2` | `0.30` | $\text{m}$ | 膝关节轴心到髋关节轴心距离 |
| **髋部至机身质心** | $l_3$ / `l3` | `0.10` | $\text{m}$ | 髋关节轴心至机身基准连杆偏置 |
| **驱动轮标称半径** | $R_w$ / `WHEEL_RADIUS` | `0.07` | $\text{m}$ | 驱动轮碰撞柱体半径 |
| **轮间距 (轮距)** | $B$ / `wheel_separation` | `0.364` | $\text{m}$ | 左右驱动轮旋转中心横向间距 |
| **整机总质量** | $M_{\text{total}}$ | $\approx 19.63$ | $\text{kg}$ | 机身 $15.60\text{kg}$ + 腿部 $3.80\text{kg}$ + 轮 $2.13\text{kg}$ |
| **高度行程区间** | $H_{\text{hip-axle}}$ | `0.30 ~ 0.50` | $\text{m}$ | 轮轴中心至髋关节垂直高度（实车定义） |

### 工作空间功能包结构
- [bbot_balance_controller](file:///home/admin/bbot_ws_new/src/bbot_balance_controller)：控制算法核心包，包含所有平衡控制器、跳跃状态机、自适应估计器及测试脚本。
- [bbot_bringup](file:///home/admin/bbot_ws_new/src/bbot_bringup)：系统启动入口，包含仿真启动 Launch 文件、Gazebo 世界模型与 `ros2_control` 控制器配置文件。
- [bbot_kinematics](file:///home/admin/bbot_ws_new/src/bbot_kinematics)：正逆运动学解析求解器（FK / IK）及雅可比矩阵运算库。
- [bbot_description](file:///home/admin/bbot_ws_new/src/bbot_description)：机器人 URDF / Xacro 模型定义、3D 网格模型文件（Meshes）与 Gazebo 传感器插件。
- [bbot_torque_control](file:///home/admin/bbot_ws_new/src/bbot_torque_control)：独立力矩控制器节点与关节扭矩实时监控节点。
- [bbot_motor_driver](file:///home/admin/bbot_ws_new/src/bbot_motor_driver)：底层关节电机通信与驱动接口。
- [bbot_rc_receiver](file:///home/admin/bbot_ws_new/src/bbot_rc_receiver)：无线遥控接收机解析驱动。

---

## 2. 编译构建与环境准备

工作空间基于 ROS 2 Iron 与 Gazebo Sim 构建。在运行前必须加载完整环境变量：

```bash
# 1. 切换至工作空间根目录
cd ~/bbot_ws_new

# 2. 编译所有功能包
source /opt/ros/iron/setup.bash
colcon build --symlink-install

# 3. 激活工作空间与仿真插件环境变量（必须执行，配置了仿真模型库与动态库路径）
source ~/bbot_ws_new/setup_env.sh
```

> [!NOTE]
> [setup_env.sh](file:///home/admin/bbot_ws_new/setup_env.sh) 已经预设了 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`、`GZ_SIM_RESOURCE_PATH` 与 `ROS_LOG_DIR`，确保 Gazebo 能正常定位 ROS 2 control 插件与自定义 mesh 网格。

---

## 3. 控制系统架构与算法全景

### 3.1 控制算法选型对比

| 控制器标识 (`controller_type`) | 对应可执行文件 | 控制模式 / 输出接口 | 底层 ROS 2 控制器 | 特性与适用场景 |
| :--- | :--- | :--- | :--- | :--- |
| **`jump`** | [`bbot_jump_controller`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_jump_controller.cpp) | 混合状态机 / 轮速 + 腿力矩 | `diff_drive_controller` + 动态切换 `leg_effort_controller` | 经典 5 阶段多项式跳跃，兼顾地面 LQR 平衡与空中姿态控制 |
| **`jump_velocity`** | [`bbot_velocity_jump_controller`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp) | 闭环动力学 / 轮速 + 腿力矩 | `diff_drive_controller` + 动态切换 `leg_effort_controller` | 目标离地速度闭环跳跃，含质心动量估计、冲量整形与滚动跳跃支持 |
| **`adaptive_lqr`** | [`adaptive_lqr_balance_controller`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp) | 力矩级 LQR + 慢速偏置自适应 | `wheel_effort_controller` + `leg_position_controller` | **推荐**。两阶段自适应外环消除载荷与质心漂移，直接力矩驱动 |
| **`gs_lqr`** | [`lqr_gain_scheduled_controller`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_gain_scheduled_controller.cpp) | 力矩级 LQR + 离线增益调度 | `wheel_effort_controller` + `leg_position_controller` | 5 节点连续增益插值，针对不同高度补偿名义平衡角与惯量变化 |
| **`lqr`** | [`lqr_balance_controller_yaokong`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) | 速度级 LQR (前馈+反馈) | `diff_drive_controller` + `leg_position_controller` | 速度外环 PI + 姿态内环全状态 LQR，静止锁位与平稳巡航解耦 |
| **`pid`** | [`balance_controller_keyboard`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp) | 速度级串级 PID | `diff_drive_controller` + `leg_position_controller` | 三层串级（速度环 $\to$ 角度环 $\to$ 角速度环），易于快速调优与速度基线对比 |
| **`torque_cascade_pid`** (别名 `torque_pid`) | [`torque_cascade_pid_controller`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/torque_cascade_pid_controller.cpp) | 带高度参数调度的三环串级 PID | `wheel_effort_controller` + `leg_position_controller` | 力矩控制基线：速度外环 → 姿态角环 → 角速度内环 → 轮端力矩；位置仅相对固定 $p_0$ 观测，验收结论以重新生成的日志为准 |

---

### 3.2 跳跃控制器族 (Jump Controllers)

#### 1. 经典多阶段跳跃控制器 (`bbot_jump_controller`)
采用有限状态机（FSM）管理跳跃生命周期：
```text
BALANCE (LQR平衡) ──> SQUAT (下蹲蓄力) ──> THRUST (爆发推地) ──> FLIGHT (腾空收展腿) ──> TOUCHDOWN_BUFFER (着陆阻抗缓冲) ──> RECOVERY (恢复站立)
```
- **SQUAT**：使用五次多项式将机身高度平滑下压至 $L_{\text{squat}} = 0.30\text{ m}$，底盘维持平衡。
- **THRUST**：在 $T_{\text{thrust}} \approx 0.16\text{s}$ 内爆发伸腿至 $0.475\text{ m}$，计算竖直冲量 $F_z = \frac{M}{2}(g + \ddot{z}_d)$，通过雅可比矩阵映射为关节力矩：
  $$\boldsymbol{\tau} = \boldsymbol{J}_z^T F_z$$
- **FLIGHT**：空中首先快速收腿（$0.30\text{ m}$），随后适时展腿（$0.45\text{ m}$）准备触地；轮毂施加反力矩调整机身俯仰姿态：
  $$u_{\text{wheel}} = -\left(K_{p,\text{air}}(\theta - \theta_0) + K_{d,\text{air}}\dot{\theta}\right)$$
- **TOUCHDOWN_BUFFER**：虚拟弹簧阻抗控制吸收着陆能量：
  $$F_z = \frac{M}{2}g + K_z(z_d - z) - D_z\dot{z}$$

#### 2. 目标离地速度闭环跳跃控制器 (`bbot_velocity_jump_controller`)
位于 [`bbot_velocity_jump_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp)，针对精准跳跃高度与行进间跳跃深度优化：
- **解耦模块支持**：集成离地检测 [`takeoff_detection.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/takeoff_detection.hpp)、推力速度轨迹规划 [`thrust_velocity_reference.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/thrust_velocity_reference.hpp)、空中俯仰控制 [`torso_pitch_control.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/torso_pitch_control.hpp) 与滚动跳跃协调 [`rolling_jump_control.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/rolling_jump_control.hpp)。
- **质心动量闭环**：根据指定 `jump_height` 实时反算理论目标离地速度 $V_{\text{takeoff}} = \sqrt{2g \Delta h}$，通过前馈+反馈调节推地冲量，避免因负载或电机关节限制产生高度欠冲。

---

### 3.3 LQR 平衡控制器族 (LQR Family)

#### 1. 自适应平衡点增益调度 LQR (`adaptive_lqr_balance_controller`)
针对真实机器人质心偏置（如搭载传感器、外挂电池或未知负载）引起的机身漂移与稳态静差设计，包含双时间尺度闭环：
- **200 Hz 快速内环**：基于轮式倒立摆状态向量 $\boldsymbol{e} = [x - x_r, \dot{x}, \theta - \hat{\theta}_{\text{eq}}, \dot{\theta}]^T$ 执行 LQR 力矩控制。
- **慢速两阶段自适应外环**（[`adaptive_equilibrium_estimator.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/adaptive_equilibrium_estimator.hpp)）：
  ```text
  0: WAIT_COARSE ──> 1: APPLY_COARSE ──> 2: WAIT_FINE ──> 3: APPLY_FINE ──> 4: VERIFY ──> 5: HOLD
  ```
  - **粗捕阶段 (Coarse)**：早期快速响应，遏制位置发散，估计并吸收约 $94\%$ 的偏置误差。
  - **精修阶段 (Fine)**：系统回拉准静态区间后严格门控，消除残余毫米级漂移并锁定收敛。

#### 2. 增益调度力矩 LQR (`lqr_gain_scheduled_controller`)
- 状态向量：$X = [x - x_{\text{ref}}, \dot{x}, \theta - \theta_{\text{eq}}(H), \dot{\theta}]^T$。
- 内置 $H \in [0.30, 0.50]\text{m}$ 的 5 节点增益调度表，动态线性插值求解当前刚度与名义平衡角。
- 支持在运行时动态切换 `GainMode::SCHEDULED`（增益调度）与 `GainMode::FIXED`（固定增益消融）。
- 控制输出映射到物理驱动轮力矩：$\tau_{\text{wheel}} = -0.5 \cdot u_{\text{model}}$。

#### 3. 速度级标准 LQR (`lqr_balance_controller_yaokong`)
- 将底层驱动适配为 `diff_drive_controller` 速度指令。
- **静止状态**：启用完整状态反馈 $[x, \dot{x}, \theta, \dot{\theta}]^T$，消除位置漂移。
- **移动状态**：将位置与速度误差解耦剥离至外环 PI 速度控制器，内环 LQR 仅跟踪动态目标倾角 $\theta_{\text{dynamic\_target}}$，避免速度闭环相互干扰。

---

### 3.4 增益调度 (Gain Scheduling) 与固定增益 (Fixed Gain) 消融机制

#### 1. 物理机理与增益调度的必要性
双轮腿机器人在改变高度 $H \in [0.30, 0.50]\text{ m}$ 时，腿部连杆的折叠与伸展导致全系统动力学特性发生显著的非线性改变：
- **等效质心高度与转动惯量变化**：高位（$0.50\text{ m}$）相比低位（$0.30\text{ m}$），机身等效摆长更长、重力倾覆力矩与俯仰惯量剧增，要求更强的俯仰恢复刚度与阻尼（$|k_{\theta}|$ 和 $|k_{\dot{\theta}}|$ 必须随高度单调增大）。
- **非对称静态偏角 $\theta_{\text{eq}}(H)$ 漂移**：机构非严格前后对称，质心在矢状面存在几何偏置（$y_{\text{CoM}}, z_{\text{CoM}}$）。在重力矩平衡点 $\theta_{\text{eq}}(H) = -\arctan(y_{\text{CoM}} / z_{\text{CoM}})$ 处，机身必须保持特定名义前倾角。随着高度 $H$ 增加，$z_{\text{CoM}}$ 变大，该静态偏角逐渐从 $+0.1184\text{ rad}$ 减小至 $+0.0618\text{ rad}$。
- **离散系统谱半径 $\rho(A_d - B_d K)$ 分析**：若在全高度范围内强行采用固定的单点增益，在远离标称高度时，闭环离散极点谱半径会恶化，引发低位高频震颤或高位响应迟缓甚至失稳。

#### 2. 5 节点增益调度表（DARE 离线求解）
控制器内置在 MATLAB / Python 中通过离散代数 Riccati 方程（DARE，基于状态权重 $Q=\text{diag}([20, 500, 2500, 150])$，控制权重 $R=1$）求解的标称参数表，在运行时以 200 Hz 按当前高度分段线性插值（lerp）：

| 节点高度 $H$ (m) | 水平位移 $k_x$ | 线速度 $k_{\dot{x}}$ | 俯仰倾角 $k_{\theta}$ | 俯仰角速度 $k_{\dot{\theta}}$ | 名义平衡角 $\theta_{\text{eq}}$ (rad) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0.30** | -5.6227 | -42.6665 | -156.5090 | -35.8467 | +0.1184 |
| **0.35** | -5.7913 | -43.9767 | -169.5441 | -39.5632 | +0.1030 |
| **0.40** (标称中点) | **-5.9316** | **-45.0871** | **-181.9730** | **-43.3902** | **+0.0878** |
| **0.45** | -6.0524 | -46.0612 | -193.9487 | -47.3497 | +0.0741 |
| **0.50** | -6.1572 | -46.9226 | -205.5182 | -51.4306 | +0.0618 |

#### 3. 固定增益 (Fixed Gain) 的定义与消融实验价值
为了严谨验证“增益调度矩阵 $K(H)$ 的动态调节”所带来的实际控制收益，代码中设计了严格的消融实验基线（Ablation Baseline）：
- **固定增益模式 (`fixed_midpoint` / `GainMode::FIXED`)**：
  - 将状态反馈矩阵 $K$ **严格锁定在 $H = 0.40\text{ m}$ 标称中位处的固定值**：
    $$K_{\text{fixed}} = [-5.9316, -45.0871, -181.9730, -43.3902]$$
  - **关键控制变量解耦**：在此模式下，名义平衡角 $\theta_{\text{eq}}(H)$、质心几何参数与腿部逆运动学依然跟随瞬时高度 $H$ 实时解算，**唯一冻结的是反馈控制增益矩阵 $K$**。
  - **消融目的**：排除几何偏角补偿的干扰，纯粹检验“增益调度（$K(H)$）”相较于“单一固定增益（$K_{\text{fixed}}$）”在全行程升降、推扰抗冲击、收敛时间上的性能差异。

#### 4. 控制器中的实现与切换方式
1. **增益调度力矩控制器 ([`lqr_gain_scheduled_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_gain_scheduled_controller.cpp))**：
   - 内部维护 `GainMode::SCHEDULED` 与 `GainMode::FIXED` 状态枚举。
   - 键盘实时切换：在终端中按下 <kbd>F</kbd> 立即切入 Fixed-LQR（锁定 $K(0.40\text{m})$），按下 <kbd>G</kbd> 切回 Gain-Scheduled LQR。
2. **自适应平衡点控制器 ([`adaptive_lqr_balance_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp))**：
   - 通过 ROS 2 Launch 参数指定：
     ```bash
     # 启用 5 节点增益调度（默认）
     ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=adaptive_lqr adaptive_gain_mode:=scheduled

     # 启用消融固定中点增益 (Fixed-Midpoint)
     ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=adaptive_lqr adaptive_gain_mode:=fixed_midpoint
     ```
3. **串级 PID 控制器 ([`balance_controller_keyboard.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp))**：
   - 同步采用增益调度设计：速度环、角度环、角速度环的 PID 增益在站立（$0.50\text{m}$）与下蹲（$0.30\text{m}$）两组参数之间按当前髋轴高度线性插值：
     - **速度环**：站立 `{0.55, 0.005, 0.025, 0.0, 0.50}` $\longleftrightarrow$ 蹲下 `{0.45, 0.004, 0.020, 0.0, 0.45}`
     - **角度环**：站立 `{7.0, 0.0, 0.12, 0.0, 2.5}` $\longleftrightarrow$ 蹲下 `{6.0, 0.0, 0.10, 0.0, 2.2}`
     - **角速度环**：站立 `{4.7, 0.0, 0.012, 10.0, 5.0}` $\longleftrightarrow$ 蹲下 `{4.3, 0.0, 0.010, 10.0, 4.5}`

#### 5. 消融对比实验数据（高度扫描与推扰）
根据 [`run_height_campaign.py`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/run_height_campaign.py) 在全高度与 $\pm 20\text{ N}$ 脉冲推扰工况下的批处理实测（见 [`table3_table4_draft.md`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/height_campaign/figures/table3_table4_draft.md)）：
- **全行程升降位置漂移**：`scheduled`（$23.30\pm0.18\text{ mm}$）优于 `fixed_midpoint`（$24.09\pm0.29\text{ mm}$）。
- **极值高度静差抑制**：在 $H = 0.30\text{ m}$ 极端蹲姿下，增益调度将稳态位置误差从 $3.23\text{ mm}$ 压低至 $2.93\text{ mm}$，RMS 总力矩由 $0.25\text{ Nm}$ 下降至 $0.22\text{ Nm}$。

---

### 3.5 串级 PID 控制器族 (Cascade PID Family)

工作空间内包含两套针对不同驱动接口与控制层级的串级 PID 控制器：底层基于 `diff_drive_controller` 的**速度级串级 PID**，以及底层直接写入 `wheel_effort_controller` 的**力矩级三环串级 PID**。

#### 1. 速度级串级 PID 控制器 (`balance_controller_keyboard`)
位于 [`balance_controller_keyboard.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp)，采用工业标准三环串级结构：
```text
车速误差 e_v ──> [速度外环 PI] ──> 目标倾角 θ_cmd ──> [角度中环 PD] ──> 目标角速度 ω_cmd ──> [角速度内环 PID] ──> 底盘轮速输出
```
- **高度调度机制**：根据腿部实时高度，在“站立（$0.50\text{ m}$）”与“蹲下（$0.30\text{ m}$）”两组预设参数之间线性插值调度。
- **输出接口**：发布至 `/diff_drive_controller/cmd_vel_unstamped`，由 Gazebo 差速驱动插件执行底层轮速闭环。

#### 2. 带高度调度的三环力矩串级 PID 控制器 (`torque_cascade_pid_controller`)
位于 [`torque_cascade_pid_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/torque_cascade_pid_controller.cpp)。统一定义为：**速度外环 → 姿态角环 → 角速度内环 → 轮端力矩；位置仅作为观测量，不参与反馈。静止实验固定 $v_ref=0$、$k_x=0$；控制器首次进入有效平衡后只锁存一次 $p_0$，位置漂移记录为 $\Delta p=p-p_0$。**

```text
期望线速度 v_ref = 0 ──> [速度外环 PID + 滤波限幅] ──> 姿态补偿角 Δθ_ref
                             └──> + θ_eq(H) ──> 期望倾角 θ_ref
                                      └──> [姿态中环 PD (陀螺直接阻尼)] ──> 期望角速度 θ_dot_ref
                                               └──> [角速度内环 PD] ──> 合成控制力矩 u ──> 饱和限幅 ──> 驱动轮分配 tau = +0.5 * u

位置观测量：固定原点 p_0 在首次进入平衡时锁存，残余漂移 Δp = p - p_0 仅作状态观测与记录，闭环位置增益恒定为 k_x = 0
```

- **核心工程与控制设计机制**：
  1. **物理力矩符号约定 (Sign Convention)**：Gazebo 仿真中轮轴负力矩驱动车轮向前滚动；当机身前倾（$\theta > \theta_{\text{eq}}$）时，误差 $e_\theta = \theta_{\text{ref}} - \theta < 0 \implies \dot{\theta}_{\text{ref}} < 0 \implies u < 0$。为加速向前赶上倾覆，双轮需施加负力矩，因此各轮力矩分配为 $\tau = \text{clamp}(+0.5 \cdot u, -10, 10)\text{ N}\cdot\text{m}$。
  2. **姿态补偿角低通滤波与硬限幅**：速度外环输出的 $\Delta\theta_{\text{ref}}$ 经一阶低通滤波（$\alpha = 0.15$）并严格限制在 $[-0.06, +0.06]\text{ rad}$（约 $\pm 3.4^\circ$）内，彻底隔绝车轮瞬时打滑与编码器高频差分抖动对姿态环的扰动。
  3. **陀螺仪直接微分阻尼 (Direct Gyro Damping)**：姿态环微分项直接采用 IMU 高频测得的角速度 $\dot{\theta}$（$D_\theta = -K_{d,\theta}\dot{\theta}$），无需对离散姿态角进行差分，根除了微分高频噪声放大。
  4. **条件积分抗饱和 (Conditional Anti-Windup)**：速度环积分器仅在输出未饱和或误差反向拉回时累加，避免启动与大扰动时的积分过冲与相位滞后。
  5. **高度调度与执行约束**：低位与高位端点参数由分层整定脚本分别从 $H=0.30$ m、$H=0.50$ m 的有效候选中得到，并在 $H=0.40$ m 验证线性插值；总力矩限制为 $\pm20$ N·m，单轮限制为 $\pm10$ N·m，额外压摆率默认关闭。
  6. **固定原点记录**：目标高度变化、推扰和升降换向均不得改变 $p_0$。`reset_position` 仅保留为手动调试命令，自动验收脚本不调用它。

#### 3. 实验验证与 GS-LQR 对照结果 (Publication Baseline Comparison)

基于严格独立重复实验（每工况有效运行 $N=3$），通过 [`run_pid_publication_campaign.py`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/run_pid_publication_campaign.py) 与 [`plot_pid_gslqr_publication.py`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/plot_pid_gslqr_publication.py) 完成了全套闭环仿真评估（原始日志详见 [`pid_publication_campaign`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/pid_publication_campaign/)，度量汇总见 [`pulse_metrics.json`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/paper42_pid_gslqr/pulse_metrics.json)）：

##### (1) 定高静态平衡与往复升降
- **定高静态平衡（$H=0.30, 0.40, 0.50\text{ m}$，各 $N=3$）**：
  - 三环 PID 在 3 个高度下末 8 s 俯仰误差 RMS 均稳定在 $0.007^\circ\sim0.025^\circ \le 0.05^\circ$，车速 RMS 在 $0.002\sim0.010\text{ m/s}$，力矩饱和率为 0，体现出基本的稳态平衡能力；由于 $k_x=0$ 无位置回拉，终段相对固定原点存在 $0.20\sim0.85\text{ m}$ 的缓速漂移。
  - GS-LQR 末 15 s 平均位置误差稳定在 $1.82\sim2.94\text{ mm}$，均收敛于 $\pm 5\text{ mm}$ 定点死区内。
- **连续升降扫高（$0.30 \leftrightarrow 0.50\text{ m}$，升降速度 $0.05\text{ m/s}$，各 $N=3$）**：
  - PID 最大俯仰误差为 $0.128 \pm 0.001^\circ$（GS-LQR 为 $0.166 \pm 0.001^\circ$）；
  - 最大相对位移波动：PID 为 $138.39 \pm 42.54\text{ mm}$，GS-LQR 为 $27.53 \pm 0.24\text{ mm}$。两者均平稳过渡未出现姿态发散。
  - 对应时域响应矢量图：[`figures/fig4_pid_gslqr_height.svg`](file:///home/admin/bbot_ws_new/figures/fig4_pid_gslqr_height.svg)。

##### (2) 多高度双向脉冲推扰响应（$\pm 20\text{ N} \times 0.20\text{ s}$，冲量 $4.0\text{ N}\cdot\text{s}$，各组 $N=3$ 均值 ± 样本标准差）
推扰起点以仿真时间戳对齐，位移峰值相对推扰前 2 s 均值计算；恢复时间 $T_{\theta v}$ 从脉冲结束起算，定义为俯仰角偏差 $\le 0.5^\circ$ 且车速 $|v| \le 0.02\text{ m/s}$ 并持续维持 2 s 的耗时：

| 高度 $H$ (m) | 脉冲推力 (N) | 控制器架构 | 最大相对位移 $\Delta p_{\max}$ (mm) | 俯仰角峰值 (°) | 姿态-速度恢复时间 $T_{\theta v}$ (s) | 峰值合成力矩 (N·m) |
| :---: | :---: | :--- | :---: | :---: | :---: | :---: |
| **0.30** | **+20** | **GS-LQR** | **143.1 ± 0.4** | 3.35 ± 0.004 | **1.71 ± 0.01** | 2.41 ± 0.12 |
| 0.30 | +20 | 三环 PID ($k_x=0$) | 484.2 ± 1.3 | **0.94 ± 0.004** | 14.05 ± 0.01 | 1.81 ± 0.03 |
| **0.30** | **−20** | **GS-LQR** | **143.1 ± 0.6** | 3.33 ± 0.012 | **1.70 ± 0.00** | 2.48 ± 0.12 |
| 0.30 | −20 | 三环 PID ($k_x=0$) | 466.2 ± 2.5 | **0.92 ± 0.004** | 14.01 ± 0.03 | 1.85 ± 0.03 |
| **0.40** | **+20** | **GS-LQR** | **152.7 ± 0.1** | 3.14 ± 0.005 | **1.80 ± 0.01** | 2.63 ± 0.16 |
| 0.40 | +20 | 三环 PID ($k_x=0$) | 454.6 ± 5.0 | **1.08 ± 0.011** | 13.09 ± 0.05 | 1.97 ± 0.04 |
| **0.40** | **−20** | **GS-LQR** | **152.9 ± 0.6** | 3.13 ± 0.016 | **1.80 ± 0.00** | 2.45 ± 0.15 |
| 0.40 | −20 | 三环 PID ($k_x=0$) | 425.8 ± 3.9 | **1.06 ± 0.010** | 13.22 ± 0.06 | 1.93 ± 0.01 |
| **0.50** | **+20** | **GS-LQR** | **161.8 ± 0.6** | 2.96 ± 0.010 | **1.93 ± 0.00** | 2.72 ± 0.05 |
| 0.50 | +20 | 三环 PID ($k_x=0$) | 419.6 ± 7.5 | **1.20 ± 0.022** | 11.89 ± 0.14 | 1.97 ± 0.07 |
| **0.50** | **−20** | **GS-LQR** | **165.2 ± 3.9** | 3.01 ± 0.070 | **1.94 ± 0.04** | 2.79 ± 0.19 |
| 0.50 | −20 | 三环 PID ($k_x=0$) | 384.6 ± 2.2 | **1.18 ± 0.006** | 12.30 ± 0.05 | 1.99 ± 0.04 |

- **动力学权衡机制分析**：
  - GS-LQR 的最大相对位移较 PID 降低 **57.0% ~ 70.4%**，姿态速度恢复时间缩短至 PID 的 **12% ~ 16%**（$< 2.0\text{ s}$），展现出全状态位置反馈消除扰动位移的优越性；
  - 三环 PID 偏向抑制瞬态倾角峰值（$0.92^\circ \sim 1.20^\circ$）并具有更小的执行器力矩峰值（$1.81 \sim 1.99\text{ Nm}$），但因无位置反馈通道，调节时间较长（$11.9 \sim 14.1\text{ s}$），且最终在新静止位置驻留（相对位移偏差约 $380\sim480\text{ mm}$）；
  - 对应低位推扰相对位移响应矢量图：[`figures/fig5_pid_gslqr_push_h030.svg`](file:///home/admin/bbot_ws_new/figures/fig5_pid_gslqr_push_h030.svg)。

---

### 3.6 运动学与动力学模块 (Kinematics)

由 [`bbot_kinematics`](file:///home/admin/bbot_ws_new/src/bbot_kinematics/src/kinematics.cpp) 提供解析支持：
- **正运动学 (FK)**：从机身倾角及各关节编码器角（$\theta_{\text{hip}}, \theta_{\text{knee}}$）递归推导膝关节、髋关节及全机等效质心（CoM）位置。
- **逆运动学 (IK)**：根据目标机身离地高度 $H$ 及俯仰角解算双腿关节对称目标位置：
  $$H \xrightarrow{\text{IK}} (\theta_{\text{hip}}, \theta_{\text{knee}})$$
- **运动链雅可比矩阵**：实时提供力矩与末端足底作用力的映射通道。

---

## 4. 仿真启动与参数配置

系统统一通过 [`bbot_gazebo.launch.py`](file:///home/admin/bbot_ws_new/src/bbot_bringup/launch/bbot_gazebo.launch.py) 拉起 Gazebo 物理仿真环境、ROS 2 Bridge 与目标控制器。

### 4.1 核心 Launch 参数表

| 参数名 | 默认值 | 可选值 / 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| `controller_type` | `jump` | `jump`, `jump_velocity`, `adaptive_lqr`, `gs_lqr`, `lqr`, `pid`, `torque_cascade_pid`, `none` | 平衡/运动控制算法类型（`torque_pid` 为 `torque_cascade_pid` 别名） |
| `world` | `balance_test_world.sdf` | `balance_test_world.sdf`, `empty.sdf` | Gazebo 仿真场景文件 |
| `gui` | `true` | `bool` | 是否启动 Gazebo 3D 渲染图形窗口（默认为 `true` 开启画面；设为 `false` 关闭画面） |
| `headless` | `false` | `bool` | 是否以无图形服务器模式运行（设为 `true` 或 `gui:=false` 适合后台批量实验） |
| `jump_height` | `0.20` | `float` (m) | `jump_velocity` 目标跳跃净空高度 |
| `takeoff_velocity` | `0.0` | `float` (m/s) | 强制覆盖离地速度（非零时优先于 `jump_height`） |
| `thrust_duration` | `0.24` | `float` (s) | 跳跃推蹬加速时间窗口 |
| `thrust_velocity_kp` | `8.0` | `float` | 推蹬阶段垂直速度反馈增益 |
| `adaptive_experiment_mode` | `adaptive` | `adaptive`, `nominal`, `oracle` | 自适应 LQR 实验模式（真实自适应 / 名义标称 / 真实真值注入） |
| `adaptive_gain_profile` | `legacy_safe` | `legacy_safe`, `optimized_v2` | LQR 反馈增益矩阵配置文件版本 |
| `adaptive_target_height` | `0.50` | `float` (m) | 轮轴至髋部稳定目标高度 |
| `payload_mass` | `0.0` | `float` (kg) | 仿真中在机身动态挂载物理配重质量 |
| `payload_x` / `payload_z` | `0.200` / `0.170` | `float` (m) | 物理外挂载荷在 `base_link` 下的坐标位置 |
| `torque_pid_target_height` | `0.50` | `float` (0.30~0.50m) | 力矩串级 PID 目标平衡高度 |
| `torque_pid_startup_height` | `0.36` | `float` (m) | 力矩串级 PID 起始触地高度（与生成原点几何匹配，消除触地冲击） |
| `torque_pid_stage_mode` | `normal` | `normal`, `stage_a`, `stage_b`, `stage_c` | 调谐阶段模式（`stage_a`角速度环 / `stage_b`姿态环 / `stage_c`或`normal`全串级） |
| `torque_pid_k_x` | `0.0` | `float` | 力矩串级 PID 位置增益（经典三环基线中恒为 0.0，位置仅被动观测） |
| `torque_pid_log_path` | `.../torque_pid_log.csv` | `string` | 力矩串级 PID 运行数据日志输出绝对路径 |

---

### 4.2 常用仿真启动指令

#### 1. 启动目标速度跳跃控制器（推荐跳跃验证模式）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump_velocity jump_height:=0.25
```

#### 2. 启动经典复合跳跃控制器（默认空场景/测试场）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump world:=balance_test_world.sdf
```

#### 3. 启动自适应平衡点 LQR（测试抗载荷与偏置性能）
```bash
# 挂载 1.5 kg 前向载荷，验证自适应两阶段状态机收敛
ros2 launch bbot_bringup bbot_gazebo.launch.py \
  controller_type:=adaptive_lqr \
  adaptive_experiment_mode:=adaptive \
  payload_mass:=1.5
```

#### 4. 启动力矩级增益调度 LQR（GS-LQR）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=gs_lqr
```

#### 5. 启动标准速度式 LQR 控制器
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=lqr
```

#### 6. 启动速度级串级 PID 控制器
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=pid
```

#### 7. 启动带高度调度的三环力矩串级 PID 控制器（经典力矩控制基线）
```bash
# 1. 默认 0.50m 目标高度站立平衡
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=torque_cascade_pid

# 2. 指定目标高度（例如 0.40m 中位或 0.30m 蹲姿）
ros2 launch bbot_bringup bbot_gazebo.launch.py \
  controller_type:=torque_cascade_pid \
  torque_pid_target_height:=0.40
```

#### 8. 执行力矩串级 PID 6 项全指标基准验收测试
```bash
# 自动依次拉起仿真执行：0.30m/0.40m/0.50m 定高静态、0.30m->0.50m->0.30m 连续扫高、±20N 脉冲推扰，并自动统计各测试项通过状态与量化指标
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/run_torque_pid_acceptance.py
```

#### 9. 运行力矩串级 PID 多阶段自动增益调谐
```bash
# 执行角速度内环 -> 姿态角中环 -> 速度外环分层自动寻优 (k_x = 0 strictly)
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/tune_torque_cascade_pid.py
```

#### 10. 运行论文全套对比实验批处理与矢量图生成
```bash
# 1. 批量自动执行 GS-LQR 与三环 PID 全套对比实验（定高、连续升降、多高度双向推扰各 N=3）
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/run_pid_publication_campaign.py

# 2. 汇总提取实验度量指标并重绘论文级高清矢量图 (输出至 figures/ 目录)
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/plot_pid_gslqr_publication.py
```

---

## 5. 键盘遥控终端与交互说明

控制节点推荐使用原生 C++ 编写的 [`teleop_keyboard`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/teleop_keyboard.cpp)，杜绝 Python 虚拟环境与标准输入冲突。

打开新终端并运行：
```bash
source ~/bbot_ws_new/setup_env.sh
ros2 run bbot_balance_controller teleop_keyboard
```

### 5.1 快捷键功能映射表

| 控制分组 | 按键 | 功能说明 | 控制输出细节 |
| :--- | :--- | :--- | :--- |
| **底盘移动** | <kbd>W</kbd> / <kbd>↑</kbd> | 前进移动 | 线速度 $+0.50\text{ m/s}$ |
| | <kbd>S</kbd> / <kbd>↓</kbd> | 后退移动 | 线速度 $-0.50\text{ m/s}$ |
| | <kbd>A</kbd> / <kbd>←</kbd> | 左转移动 | 角速度 $+0.60\text{ rad/s}$ |
| | <kbd>D</kbd> / <kbd>→</kbd> | 右转移动 | 角速度 $-0.60\text{ rad/s}$ |
| | <kbd>Space</kbd> | **主动刹车** | 线速度/角速度归零，并重置自适应 LQR 目标平衡位置 |
| **高度调节** | <kbd>Q</kbd> | **升高机身** | 轮轴-髋部高度 $+1\text{cm}$（行程限制 $0.30\sim0.50\text{m}$） |
| | <kbd>E</kbd> | **降低机身** | 轮轴-髋部高度 $-1\text{cm}$（行程限制 $0.30\sim0.50\text{m}$） |
| **自适应 LQR 状态机** | <kbd>T</kbd> | 开启 / 关闭偏置自适应 | 发送 `toggle_adaptation` 切换自适应状态 |
| | <kbd>H</kbd> | 锁定 / 恢复估计更新 | 发送 `toggle_adaptation_hold` 保持当前补偿量并暂停更新 |
| | <kbd>C</kbd> | 清零估计偏置 | 发送 `reset_adaptation` 重置质心估计 |
| **增益调度与固定增益** | <kbd>F</kbd> | 切换为固定增益 (Fixed-LQR) | 强制锁定使用 $H=0.40\text{m}$ 标称中位增益矩阵（消融测试） |
| | <kbd>G</kbd> | 切换为增益调度 (Gain-Scheduled) | 恢复按当前高度实时插值状态增益矩阵 $K(H)$ |
| **动作与状态模式** | <kbd>J</kbd> | **触发跳跃** | 向 `/jump_cmd` 发布 `"jump"`，激活跳跃状态机 |
| | <kbd>R</kbd> | **倒地起立自恢复** | 发布 `standup` 模式，触发大范围摆臂自复位冲量 |
| | <kbd>B</kbd> | 恢复平衡控制 | 发布 `balance` 模式 |
| | <kbd>X</kbd> | **紧急停机** | 发布 `emergency` 模式，切断所有控制力矩输出 |

---

### 5.2 核心 ROS 2 通信话题接口

| 话题名称 | 消息类型 | 传输方向 | 对应功能 |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 遥控 $\to$ 平衡控制器 | 遥控器平移线速度与偏航转向角速度指令 |
| `/target_height` | `std_msgs/msg/Float64` | 遥控 $\to$ 控制器 | 轮轴至髋关节目标竖直高度指令（单位：$\text{m}$） |
| `/jump_cmd` | `std_msgs/msg/String` | 遥控 $\to$ 跳跃控制器 | 跳跃触发指令（`"jump"`） |
| `/adaptive_lqr/command` | `std_msgs/msg/String` | 遥控 $\to$ 自适应控制器 | 自适应外环模式控制指令（`toggle_adaptation` 等） |
| `/torque_cascade_pid/command` | `std_msgs/msg/String` | 遥控/脚本 $\to$ 力矩 PID 控制器 | 控制指令（`reset_position` 仅手动重置积分状态且不改固定 $p_0$；`reset_pid` 重置积分器与内态） |
| `/robot_mode` | `std_msgs/msg/String` | 遥控 $\to$ 状态管理 | 机器人运行模式（`balance`, `standup`, `emergency`） |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo Bridge $\to$ 控制器 | 6 轴惯导高频姿态四元数与角速度数据 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Broadcaster $\to$ 控制器 | 所有驱动轮与腿部关节实时编码器位置与转速 |
| `/wheel_effort_controller/commands` | `std_msgs/msg/Float64MultiArray` | 控制器 $\to$ 驱动轮 | 直接轮毂驱动力矩（用于 `gs_lqr`, `adaptive_lqr`, `torque_cascade_pid`） |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | 控制器 $\to$ 驱动轮 | 底盘解耦速度指令（用于 `pid`, `lqr`, `jump`） |
| `/leg_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 控制器 $\to$ 腿部关节 | 左右腿四关节目标位置角（用于常规站立/巡航） |
| `/leg_effort_controller/commands` | `std_msgs/msg/Float64MultiArray` | 控制器 $\to$ 腿部关节 | 左右腿四关节雅可比驱动力矩（用于推蹬与落地缓冲） |

---

## 6. 仿真测试场与地形测试指南

默认场景文件 [`balance_test_world.sdf`](file:///home/admin/bbot_ws_new/src/bbot_bringup/worlds/balance_test_world.sdf) 是针对轮腿平衡性能定制的专业综合测试场。机器人初始位置生成于中心原点 `(0, 0, 0.403)`，机头朝向 $+Y$ 轴：

```text
                     [前方 +Y]
                测试区 1: 多级斜坡群
           (5° 绿 / 10° 黄 / 15° 红 / 8° 拱桥)
                       │
 [左侧 -X]             │             [右侧 +X]
测试区 2: 连续减速带群 ──┼── 测试区 3: 蛇形避障绕桩群
 (1.5cm/3.0cm/4.5cm)   │    (5 根立柱 + 狭窄静态箱体通道)
                       │
                测试区 4: 跳台与单边障碍
           (左右单侧 3~4cm 凸起 / 8~15cm 台阶跳台)
                     [后方 -Y]
```

- **测试区 1：多级斜坡**（$+Y > 2.0\text{m}$）：检验控制器在俯仰平衡斜坡分量下的抗倾覆性能、爬坡力矩裕度以及过渡衔接。
- **测试区 2：减速带与连续波浪路**（$X \approx -4.5 \sim -6.5\text{m}$）：考核轮地突发冲击扰动下的一阶微分阻尼响应与防飞轮保护。
- **测试区 3：绕桩与避障**（$X \approx +6.0 \sim +9.0\text{m}$）：考核高速打盘转向向心加速度下的侧倾补偿与横向稳定性。
- **测试区 4：单边高差与跳台**（$-Y < -2.0\text{m}$）：测试跳跃控制器落地冲击吸收、空中姿态自平稳及左右单侧过障能力。

---

## 7. 实验数据记录与分析脚本

控制算法在运行期间会在 [`src/bbot_balance_controller/src/data_logs/`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/) 自动生成结构化 CSV 格式时序日志，并在 [`scripts/`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/) 提供了自动化批处理与论文级绘图脚本：

```bash
# 1. 绘制跳跃测试时序曲线 (包含高度、离地速度、推地冲量、空中姿态)
python3 ~/bbot_ws_new/src/bbot_balance_controller/src/data_logs/plot_jump_log.py

# 2. 绘制自适应 LQR 偏置估计收敛曲线与漂移对比图
python3 ~/bbot_ws_new/src/bbot_balance_controller/src/data_logs/plot_adaptive_lqr.py

# 3. 运行不同高度全行程扫频批量测试 (0.30m ~ 0.50m)
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/run_height_campaign.py

# 4. 运行外挂载荷鲁棒性批量验证测试 (P0~P4 载荷对比)
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/run_payload_campaign.py

# 5. DARE 离散代数 Riccati 方程求解与 LQR 增益在线验证
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/verify_lqr_model_and_sweep.py

# 6. 执行力矩级串级 PID 6 项全指标基准验收测试 (0.30/0.40/0.50m 静态、连续变高扫频、±20N 脉冲推扰)
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/run_torque_pid_acceptance.py

# 7. 运行力矩级串级 PID 多阶段参数寻优自动调谐脚本
python3 ~/bbot_ws_new/src/bbot_balance_controller/scripts/tune_torque_cascade_pid.py
```

---

## 8. 核心源码架构索引

| 模块类别 | 关键源文件 | 功能职责与技术特征 |
| :--- | :--- | :--- |
| **跳跃控制** | [`bbot_velocity_jump_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp) | 闭环离地速度跳跃控制器主体，集成质心状态与冲量整形分配 |
| | [`bbot_jump_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/bbot_jump_controller.cpp) | 经典 5 阶段状态机跳跃复合控制器（五次多项式 + 雅可比力矩） |
| | [`jump_phase_control.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/jump_phase_control.hpp) | 跳跃阶段时序管理与平滑过渡逻辑 |
| | [`takeoff_detection.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/takeoff_detection.hpp) | 基于加速度与腿部反作用力的离地/触地综合检测器 |
| | [`torso_pitch_control.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/torso_pitch_control.hpp) | 空中飞行阶段机身俯仰稳定与落足角前馈解算 |
| **LQR 平衡** | [`adaptive_lqr_balance_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp) | 自适应平衡点 LQR 控制器，集成两阶段状态机与力矩分配 |
| | [`adaptive_equilibrium_estimator.hpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/include/bbot_balance_controller/adaptive_equilibrium_estimator.hpp) | 两阶段等效质心偏置估计器（粗捕/精修/死区/门控过滤） |
| | [`lqr_gain_scheduled_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_gain_scheduled_controller.cpp) | 增益调度 LQR 控制器，基于高度查表插值输出直接轮毂力矩 |
| | [`lqr_balance_controller_yaokong.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) | 速度式 LQR 控制器，速度 PI 外环与姿态 LQR 内环解耦 |
| **PID 平衡** | [`torque_cascade_pid_controller.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/torque_cascade_pid_controller.cpp) | 带高度调度的经典三环力矩串级 PID 控制器（速度-姿态-角速度闭环，位置仅作相对于固定原点 p_0 的观测量，输出直接轮毂力矩） |
| | [`balance_controller_keyboard.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp) | 速度级三层串级 PID 控制器实现与高低位增益线性插值 |
| | [`torque_cascade_pid_gains.yaml`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/config/torque_cascade_pid_gains.yaml) | 力矩级串级 PID 预设增益配置文件（高位 0.50m 与低位 0.30m 两组调谐增益） |
| | [`run_torque_pid_acceptance.py`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/run_torque_pid_acceptance.py) | 力矩级串级 PID 6 项全指标基准性能自动化批量验收测试脚本 |
| | [`tune_torque_cascade_pid.py`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/scripts/tune_torque_cascade_pid.py) | 力矩级串级 PID 分层多阶段参数网格搜索与自动调谐脚本 |
| **运动学** | [`kinematics.cpp`](file:///home/admin/bbot_ws_new/src/bbot_kinematics/src/kinematics.cpp) | 轮腿闭式运动学正逆解（FK / IK）与雅可比矩阵运算 |
| | [`robot_params.hpp`](file:///home/admin/bbot_ws_new/src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp) | 机器人全局连杆尺寸、质量、轮径标称参数常量定义 |
| **人机交互** | [`teleop_keyboard.cpp`](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/teleop_keyboard.cpp) | 纯 C++ 原生终端遥控节点，支持全套控制模式切换 |
| **配置与集成**| [`bbot_gazebo.launch.py`](file:///home/admin/bbot_ws_new/src/bbot_bringup/launch/bbot_gazebo.launch.py) | 仿真与全套控制器拉起入口，配置插件 Bridge 与控制器管理器 |
| | [`bbot_controllers.yaml`](file:///home/admin/bbot_ws_new/src/bbot_bringup/config/bbot_controllers.yaml) | `ros2_control` 控制器管理、关节分组接口与轮轴参数配置 |
