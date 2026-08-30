# BBOT 双轮足机器人 URDF 物理规范与关键避坑指南 (README)

本说明文档总结了本项目从 `myrobot_urdf` 模型迁移至 ROS 2 (Iron/Humble) + Gazebo Sim (Ignition Fortress) 过程中发现的**全部物理坐标系定义、关节轴向约定、运动学零位对齐、IMU 轴向映射、Gazebo 仿真参数与控制闭环注意事项**。

---

## 目录
1. [坐标系与右手定则物理定义 (至关重要)](#1-坐标系与右手定则物理定义)
2. [CAD 零位与解析运动学 (FK/IK) 偏置](#2-cad-零位与解析运动学-fkik-偏置)
3. [左右腿对称性与关节指令符号映射](#3-左右腿对称性与关节指令符号映射)
4. [轮子滚动方向与里程计速度计算](#4-轮子滚动方向与里程计速度计算)
5. [IMU 传感器轴向与俯仰角 (Pitch) 读取陷阱](#5-imu-传感器轴向与俯仰角-pitch-读取陷阱)
6. [Gazebo ros2_control 硬件接口与关节动力学参数](#6-gazebo-ros2_control-硬件接口与关节动力学参数)
7. [仿真时序与倒立摆同步启动机制 (Pause-until-Ready)](#7-仿真时序与倒立摆同步启动机制-pause-until-ready)
8. [核心参数快速速查表](#8-核心参数快速速查表)

---

## 1. 坐标系与右手定则物理定义

基于 CAD (`robot.urdf` / `bbot.urdf.xacro`) 导出的机身坐标系定义如下：

* **$+X$ 轴**：侧向向左（Lateral Left）
* **$+Y$ 轴**：前进/前向俯仰（Forward Longitudinal）
* **$+Z$ 轴**：竖直向上（Vertical Upward）

> [!WARNING]
> **关键物理规律**：
> 本机器人的腿部所有旋转关节（髋、膝、轮）的物理转轴**全部定义在 X 轴**（即 `[1, 0, 0]` 或 `[-1, 0, 0]`）。
> 因此，机器人的**前后俯仰（Sagittal Motion）在欧拉角 RPY 中属于绕 X 轴的旋转（Roll）**，而非传统飞行器定义中绕 Y 轴的 Pitch！

---

## 2. CAD 零位与解析运动学 (FK/IK) 偏置

### 2.1 CAD 零位物理构型
本模型在关节角全为零（$q_{\text{hip}} = 0, q_{\text{knee}} = 0$）时，**并非完全伸直的长腿，而是天然折弯的标准站立姿态**：
* 大腿初始倾角：$\alpha_{1,0} = \text{atan2}(-0.29348, 0.06220) = -78.03^\circ$
* 小腿初始倾角：$\alpha_{2,0} = \text{atan2}(0.28211, 0.19554) = +55.27^\circ$
* 此时整机离地高度天然为：
  $$H_0 = Z_{\text{hip}} + l_1 \cos(\alpha_{1,0}) + l_2 \cos(\alpha_{2,0}) + R_{\text{wheel}} = 0.070 + 0.0622 + 0.1955 + 0.075 = \mathbf{0.4027\text{ m}} \approx \mathbf{0.403\text{ m}}$$

### 2.2 髋关节与轮轴水平结构偏置（$\Delta Y$）
* 在 CAD 零位下，轮轴相对于髋关节在 Y 轴上存在结构设计偏置：
  $$\Delta Y = Y_{\text{wheel}} - Y_{\text{hip}} = 0.11362779 - 0.12500000 = \mathbf{-0.01137221\text{ m}}$$
* **避坑要点**：
  在 `bbot_kinematics` 的逆运动学解算中，**必须带入此偏置 `target_dY = -0.01137221`**。如果误将 $\Delta Y$ 设为 0，解算出的关节角会产生近 $70^\circ$ 的剧烈反常折叠，导致机器人生成后双腿猛烈后缩并瘫坐在地面上！

---

## 3. 左右腿统一转轴规范与运动学指令

在 ROS 2 URDF（`bbot.urdf.xacro`）中，为了杜绝因符号反转导致的左右腿非对称反向弯曲畸变：
* **所有左右侧关节（髋关节、膝关节、车轮）转轴全部严格统一设定为 `[1, 0, 0]`**：
  * 左腿：`link_002_joint` (`[1, 0, 0]`), `link_003_joint` (`[1, 0, 0]`), `link_004_joint` (`[1, 0, 0]`)
  * 右腿：`link_005_joint` (`[1, 0, 0]`), `link_006_joint` (`[1, 0, 0]`), `link_007_joint` (`[1, 0, 0]`)

### 指令映射关系
在统一坐标系下，左右腿具有完全一致的几何方程，控制器发送目标位置无需任何反号：
$$\text{leg\_cmd.data} = \{ q_{\text{hip}}, q_{\text{knee}}, q_{\text{hip}}, q_{\text{knee}} \}$$
从而从数学与底层物理上 100% 保证双腿严格平行动作与对称伸缩！

---

## 4. 轮子滚动方向与里程计速度计算

### 4.1 轮子旋转与地面驱动力的右手定则推导
对于转轴为 `[1, 0, 0]` 的车轮（角速度 $\vec{\omega} = [\omega_x, 0, 0]^T$），其接地触点相对于轮轴的矢量为 $\vec{r} = [0, 0, -R]^T$：
$$\vec{v}_{\text{contact}} = \vec{\omega} \times \vec{r} = (\omega_x \hat{i}) \times (-R \hat{k}) = +\omega_x R \hat{j}$$

* 当 $\omega_x > 0$ 时，接地点向 $+Y$（前方）搓动，地面摩擦力将机器人推向 **$-Y$（后退）**；
* 当 $\omega_x < 0$ 时，接地点向 $-Y$（后方）搓动，地面摩擦力将机器人推向 **$+Y$（前进）**。

### 4.2 速度与里程计公式
因此，前向运动速度 $\dot{x}$ 与位移 $x$ 需加上负号：
$$\dot{x} = - R \cdot \frac{\omega_{\text{left}} + \omega_{\text{right}}}{2}, \quad x = - R \cdot \frac{\Delta\theta_{\text{left}} + \Delta\theta_{\text{right}}}{2}$$

---

## 5. IMU 传感器轴向与俯仰角 (Pitch) 读取陷阱

### 5.1 致盲 Bug 解析
在标准 ROS `tf2::Matrix3x3(q).getRPY(roll, pitch, yaw)` 中：
* `roll`：绕 X 轴旋转
* `pitch`：绕 Y 轴旋转

由于机器人前后倾倒属于**绕 X 轴的俯仰运动**，若直接读取 `pitch` 和 `angular_velocity.y`，即使机器人倒在地上，读取到的数值也永远精确等于 $0.000$！这会导致平衡控制器完全感知不到倾倒而停止控制输出。

### 5.2 正确读取代码
```cpp
// 俯仰角属于绕 X 轴的旋转 (roll)
// 前倾 (+Y) 对应负 roll，取负号使前倾误差为正
pitch_ = -roll;
pitch_rate_raw_ = -msg->angular_velocity.x;
```

---

## 6. Gazebo ros2_control 硬件接口与关节动力学参数

在 Gazebo Sim (`gz_ros2_control`) 环境下，必须在 URDF 中显式配置以下参数，否则会导致双腿无力塌陷：

### 6.1 关节比例增益（`position_proportional_gain`）
`gz_ros2_control` 默认的位置增益仅为 $0.1\text{ Nm/rad}$。必须在 `<ros2_control><hardware>` 中配置：
```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  <param name="position_proportional_gain">150.0</param>
</hardware>
```

### 6.2 关节机械动力学阻尼（Dynamics Damping）
必须在每个腿部旋转关节的 `<joint>` 标签中添加阻尼与摩擦力：
```xml
<dynamics damping="3.0" friction="0.2"/>
```
驱动轮关节添加：
```xml
<dynamics damping="0.1" friction="0.05"/>
```

---

## 7. 仿真时序与倒立摆同步启动机制 (Pause-until-Ready)

双轮足机器人属于物理非稳定倒立摆。如果 Gazebo 物理引擎在 ROS 2 控制器加载的 2.5 秒空档期内即时运行，机器人会在无控制力矩下自由倒地，导致机身后方瘫坐在地面上，启动后无法自行恢复。

### 标准启动流程（[bbot_gazebo.launch.py](bbot_bringup/launch/bbot_gazebo.launch.py)）
1. **$t=0.0\text{s}$**：Gazebo 以暂停模式启动（`gz_args: 'empty.sdf'`，不加 `-r`），机器人以直立姿态（$-z\ 0.403$）生成；
2. **$t=1.0\text{s} \sim 2.0\text{s}$**：加载并激活 `joint_state_broadcaster`、`diff_drive_controller` 与 `leg_position_controller`；
3. **$t=2.5\text{s}$**：启动 `lqr_balance_controller`，接管双腿锁位与自平衡回路；
4. **$t=3.0\text{s}$**：自动调用 Gazebo `/world/empty/control` 服务解除仿真暂停，物理第一帧与平衡闭环无缝衔接。

---

## 8. 核心参数快速速查表

| 参数项 | 取值 | 说明 |
| :--- | :--- | :--- |
| **标准站立高度 $H_0$** | `0.403 m` | 对应 $q_{\text{hip}}=0, q_{\text{knee}}=0$ |
| **高度工作范围** | `[0.30, 0.445] m` | $0.445\text{ m}$ 预留 5mm 防奇异 |
| **轮半径 $R_{\text{wheel}}$** | `0.075 m` | 驱动轮物理半径 |
| **轮间距 $W$** | `0.364 m` | 双轮横向中心距 |
| **关节位置比例刚度 $K_p$** | `150.0` | `gz_ros2_control` 支撑刚度 |
| **平衡倾角参考零点 $\theta_{\text{eq}}$** | `0.000 rad` | 绝对水平直立平衡点 |
| **LQR 闭环全状态方程** | $\text{cmd\_x} = -(K_x x + K_v \dot{x} + K_\theta \theta + K_\omega \dot{\theta})$ | 四项负反馈协同，前倾/后倾强劲回正 |
