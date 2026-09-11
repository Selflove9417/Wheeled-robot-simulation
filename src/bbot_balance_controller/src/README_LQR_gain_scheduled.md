[`lqr_gain_scheduled_controller.cpp`]实现了**基于增益调度（Gain Scheduling）的力矩级 LQR 平衡与高度协调控制器**

该控制器的核心目标是：在机器人腿部高度 $H$（0.30 m ~ 0.50 m）动态变化的过程中，实时调整 LQR 增益与名义平衡点，保证轮式双足机器人的纵向平衡与位移锁死，同时驱动双腿关节追踪目标高度。

---

### 一、 整体控制架构

控制器的整体架构包含以下几个闭环环节：

```
[目标高度 target_height] ──> [平滑斜坡规划] ──> current_height (H) ──> [逆运动学 IK] ──> 发布腿关节角度 (髋/膝)
                                      │
                                      ├──> [增益插值表] ──> K(H), theta_eq(H)
                                                                 │
[IMU: 俯仰角/角速度]  ──> [低通滤波] ──> pitch, pitch_rate ─────┤
                                                                 ├──> [LQR 控制律 u = -K*X] ──> 双轮力矩分配 tau_each
[轮编码器: 位移/速度] ──> [低通滤波] ──> x, x_dot ──────────────┘
```

---

### 二、 核心算法与数学原理

#### 1. 状态空间与 LQR 控制律
控制器将机器人纵向动态抽象为轮式倒立摆，状态向量定义为：
$$X = \begin{bmatrix} x - x_{\text{ref}} \\ \dot{x} \\ \theta - \theta_{\text{eq}}(H) \\ \dot{\theta} \end{bmatrix}$$

其中：
- $x - x_{\text{ref}}$：轮轴在地面上的水平位移误差（虚拟弹簧项，用于原地锁死与抗漂移）。
- $\dot{x}$：底盘前进线速度（虚拟阻尼项）。
- $\theta - \theta_{\text{eq}}(H)$：机身俯仰角偏差。$\theta_{\text{eq}}(H)$ 为当前高度下的**静态质心等效平衡角**。
- $\dot{\theta}$：俯仰角速度。

LQR 反馈力矩输出（对应模型中的双轮总驱动力矩）：
$$u_{\text{model}} = - K(H) X = - (k_x e_x + k_{\dot{x}} \dot{x} + k_{\theta} e_{\theta} + k_{\dot{\theta}} \dot{\theta})$$

#### 2. 增益调度（Gain Scheduling）与静态平衡角 $\theta_{\text{eq}}$
当双腿弯曲使机身高度 $H$ 改变时，机器人的等效质心高度与转动惯量都会发生剧烈改变，固定参数的 LQR 在大范围升降时容易失稳或产生稳态静差。

代码中内置了一张在 MATLAB（`LQR_K_new.m`）中基于不同高度线性化计算出的增益表（第 321–355 行）：

| $H$ (m)  | $k_x$   | $k_{\dot{x}}$ | $k_{\theta}$ | $k_{\dot{\theta}}$ | $\theta_{\text{eq}}$ (rad) |
| :------- | :------ | :------------ | :----------- | :----------------- | :------------------------- |
| **0.30** | -5.6227 | -42.6665      | -156.5090    | -35.8467           | +0.1184                    |
| **0.35** | -5.7913 | -43.9767      | -169.5441    | -39.5632           | +0.1030                    |
| **0.40** | -5.9316 | -45.0871      | -181.9730    | -43.3902           | +0.0878                    |
| **0.45** | -6.0524 | -46.0612      | -193.9487    | -47.3497           | +0.0741                    |
| **0.50** | -6.1572 | -46.9226      | -205.5182    | -51.4306           | +0.0618                    |

- **增益随高度的趋势**：高度 $H$ 越高，重力倾覆力矩与摆动惯量越大，因此增益绝对值（尤其是 $k_{\theta}$ 和 $k_{\dot{\theta}}$）单调递增。
- **$\theta_{\text{eq}}$ 的物理含义**：由于机构非完全前后对称，质心在垂直方向存在偏置（$y_{\text{COM}}, z_{\text{COM}}$）。为了在某个高度下合力矩为 0 静止站立，机身必须保持 $\theta_{\text{eq}} = -\arctan(y_{\text{COM}} / z_{\text{COM}})$ 的前倾偏置。随着高度增加，该名义偏角逐渐减小。
- **插值算法**：在 `interpolate_schedule()` 函数中，当 $H$ 处于相邻两个高度点之间时，采用分段线性插值（lerp）计算当前的增益向量与名义平衡角。

---

### 三、 模块详细解析

#### 1. 传感器处理与符号约定转换
不同硬件与仿真环境（如 URDF/Gazebo）的坐标系定义不同，代码内部统一转换为标准物理定义：

* **IMU 姿态（`imu_callback`，第 511–564 行）**：
  * 从四元数转换出 RPY：
    $$\text{pitch} = -\text{roll}_{\text{imu}}$$
    $$\dot{\theta} = -\omega_{x,\text{imu}}$$
    将物理前倾映射为正角度（$\text{pitch} > 0$）。
  * 对角速度施加一阶低通滤波（$\alpha = 0.10$）抑制陀螺仪高频噪声。

* **驱动轮编码器（`joint_state_callback`，第 570–669 行）**：
  * 读取左/右轮关节位置与速度（`link_004_joint` 与 `link_007_joint`）。
  * 首次接收到数据时建立零位基准（`wheel_origin_set_`）。
  * 轮旋转方向与前进方向映射（轮反转为机器人向前）：
    $$\dot{x}_{\text{raw}} = -R_{\text{wheel}} \cdot \frac{v_{\text{004}} + v_{\text{007}}}{2}$$
    $$x = -R_{\text{wheel}} \cdot \frac{\Delta \theta_{\text{004}} + \Delta \theta_{\text{007}}}{2}$$
  * 对前进速度 $\dot{x}$ 施加一阶低通滤波（$\alpha = 0.05$）。

#### 2. 高度轨迹与逆运动学解算（第 770–834 行）
* **斜坡平滑器（`update_leg_height`）**：
  * 防止目标高度突变导致关节剧烈撞击或失稳。以固定速度（`leg_transition_speed_ = 0.05 m/s`，全行程 0.20 m 需耗时 4 秒）向目标高度过渡。
* **逆运动学求解（`publish_leg_pose`）**：
  * 调用 [`kinematics_.inverse_kinematics(current_height_, 0.0)`](file:///home/admin/bbot_ws_new/src/bbot_kinematics/src/kinematics.cpp) 计算左右对称的腿部目标姿态：
    $$\theta_{\text{hip}}, \theta_{\text{knee}}$$
  * 打包为 `Float64MultiArray` 发布至 `/leg_position_controller/commands`（左髋、左膝、右髋、右膝）。

#### 3. 控制律计算与驱动力矩分配（`control_loop`，第 883–1099 行）
在 200 Hz 主循环中：
1. **增益更新**：
   * 在 `GainMode::SCHEDULED` 模式下采用插值计算的增益；
   * 在 `GainMode::FIXED` 模式下强制使用固定在 $H=0.40$ m 的标称增益（便于对比验证增益调度的有效性）。
2. **计算原始模型力矩**：
   $$u_{\text{raw}} = - (k_x e_x + k_{\dot{x}} \dot{x} + k_{\theta} e_{\theta} + k_{\dot{\theta}} \dot{\theta})$$
3. **总力矩饱和截断**：
   限制在 $[-u_{\text{max}}, u_{\text{max}}]$（双轮总力矩上限 20 Nm）。
4. **模型力矩到底层执行机构力矩的映射**：
   * 仿真环境中：`-effort` 对应前进，且驱动为左右双轮独立电机，因此单轮指令分配为：
     $$\tau_{\text{each}} = -0.5 \cdot u_{\text{model}}$$
   * 进一步对单轮力矩限制在 $[-10.0, 10.0]$ Nm。
   * 发布至 `/wheel_effort_controller/commands`。

#### 4. 安全保护机制（Safety Mechanisms）
* **传感器就绪检查**：未收到有效 IMU 或轮原点未初始化前，强制输出零力矩（`publish_zero_torque()`）。
* **大角度倾覆保护**：
  $$\left|\theta - \theta_{\text{eq}}\right| > 0.50 \text{ rad} \ (\approx 28.6^\circ)$$
  一旦超出安全界限，立即切断控制使能（`control_enabled_ = false`）并输出零力矩，防止倒地后电机飞车。
* **急停功能**：支持键盘热键急停与切断。

#### 5. 人机交互与数据记录
* **键盘控制交互（`process_keyboard`）**：
  * `Q` / `E`：以 0.01 m 为步长调高/调低机身高度。
  * `Space`：将当前位置设为平衡原点（$x_{\text{ref}} = x$）。
  * `X` / `B`：急停切断 / 恢复平衡控制。
  * `F` / `G`：在固定增益（$H=0.40$ m）与增益调度间切换。
* **CSV 数据记录（`log_data`）**：
  * 自动将运行时间、模式、高度、状态量误差、增益参数及输出力矩实时记录到 `data_logs/gs_lqr_torque_log.csv`，供后续离线绘图与性能对比。