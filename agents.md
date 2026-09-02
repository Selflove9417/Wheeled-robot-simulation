# agents.md — BBot 双轮腿机器人跳跃控制项目

## 1. 项目范围

本仓库用于 BBot 双轮腿机器人在 ROS 2 + Gazebo 中的平衡、跳跃、落地缓冲与恢复控制。

当前跳跃控制 FSM：

`BALANCE → SQUAT → THRUST → FLIGHT → TOUCHDOWN_BUFFER → RECOVERY`

跳后 `RECOVERY → BALANCE` 当前暂时禁用：落地稳定后保持 RECOVERY，直到
BALANCE 接管能够单独验证稳定。

当前目标是在已经能够完成跳跃、落地并基本长期站立的基础上，降低跳后 Effort BALANCE 中的关节力矩稳态纹波。

## 2. 当前环境

- ROS 2：Iron
- `ros2_control`
- `gz_ros2_control/GazeboSimSystem`
- 主控制周期：5 ms / 200 Hz
- `controller_manager.update_rate`：100 Hz
- Position controller：`position_controllers/JointGroupPositionController`
- Effort controller：`effort_controllers/JointGroupEffortController`
- 轮毂：`diff_drive_controller/DiffDriveController`

不要把 Jazzy / Rolling 的 `gz_ros2_control` 实现细节直接当成 Iron 的确定行为。涉及底层接口行为时，先核对当前 Iron 安装版本或本机源码。

## 3. 腿关节顺序

控制器和 C++ 命令顺序必须严格一致：

1. `link_002_joint` — 左髋
2. `link_003_joint` — 左膝
3. `link_005_joint` — 右髋
4. `link_006_joint` — 右膝

Position：

```cpp
{q_hip_left, q_knee_left, q_hip_right, q_knee_right}
```

Effort：

```cpp
{tau_hip_left, tau_knee_left, tau_hip_right, tau_knee_right}
```

禁止在没有读取当前 YAML / 源码的情况下猜 joint 顺序。

## 4. 已验证的控制架构

### 4.1 起跳前

普通 BALANCE 使用 Position controller，可以正常站立。

### 4.2 跳跃开始后

SQUAT 完成后执行一次：

`Position → Effort`

之后保持 Effort：

`THRUST → FLIGHT → TOUCHDOWN_BUFFER → RECOVERY`

### 4.3 跳后禁止自动 Effort → Position

已经重复验证：

- controller manager 会报告 Position controller 激活成功；
- Position command 顺序正确且持续发布；
- 但 Effort controller deactivate 后，腿仍会快速塌软。

因此正常 FSM 中禁止 THRUST 之后自动执行 Effort → Position。

`request_position_controller()` 可以保留作调试，但正常跳跃流程不应调用。

## 5. 当前可靠基线：RECOVERY

RECOVERY 已经实际验证可以稳定承重：

```cpp
publish_effort_height_control(
    des_z, des_v,
    320.0, 50.0, 170.0,
    18.0, 4.0,
    30.0, 6.0,
    true);
```

参数：

```text
Kz = 320 N/m
Dz = 50 N·s/m
Fmax_per_leg = 170 N

Hip  Kp/Kd = 18 / 4
Knee Kp/Kd = 30 / 6

Body pitch PD = ON
```

RECOVERY → BALANCE 的原则是“控制律连续”。不要在状态切换瞬间同时更换多条反馈通道。

## 6. 已验证失败的调参方向

### 6.1 Joint D 不能撤掉

实验结果：

```text
Hip/Knee Kd = 4/6
→ 稳定，但有稳态纹波

Kd 逐步降到 0/0
→ 振荡快速放大，最终失稳

Kd 降到 2/3
→ 抖动没有减小，反而更大
```

当前结论：

**Joint D 是必要稳定阻尼，不是主要抖动源。**

后续不要再通过继续降低 Joint D 来消除纹波。

### 6.2 不要一次性删除多个阻尼环

曾经同时删除高度 D、Joint D、Body pitch PD，并改变 wheel gyro / soft-start，导致系统严重失稳和箱体绕髋轴旋转。

每次实验只允许改变一个主要控制因素。

### 6.3 不要恢复旧 Position preload 解释

过去曾用 `hip +0.008 rad / knee -0.040 rad` 解释 Position 下的静态力矩预载。这个解释不再作为当前设计依据。

### 6.4 `/joint_states.effort` 不是 Position 模式实际 actuator torque 的充分证据

Position 模式时 measured effort 曾长期接近 0，因此不能单独用该字段判断底层 Position servo 的真实输出。

## 7. 当前 v6 实验

v6 只改变 Joint P。

RECOVERY：

```text
Hip  Kp/Kd = 18 / 4
Knee Kp/Kd = 30 / 6
```

进入 BALANCE 后 1.0 s 平滑变为：

```text
Hip  Kp/Kd = 14 / 4
Knee Kp/Kd = 24 / 6
```

之后长期保持。

保持不变：

```text
Kz = 320
Dz = 50
Fmax_per_leg = 170 N
Body pitch PD = ON
post_landing gyro scale = 0.50
BALANCE soft-start = ON
Effort interface = ON
```

v6 的唯一问题是：

> 在完整保留 Joint D 的情况下，降低 Joint P 是否能够减小稳态力矩纹波，同时保持长期稳定？

## 8. 任务空间承重控制

核心形式：

```text
Fz_target =
    m_leg * g
  + Kz * (z_des - z)
  + Dz * (zdot_des - zdot)
```

然后使用左右腿实际构型的几何 Jacobian：

```text
tau_ff = Jz(q)^T * Fz
```

再叠加：

- Joint PD
- Body pitch PD

`current_z_dot_` 由 FK 高度差分得到，并经过：

```cpp
current_z_dot_ =
    low_pass_filter(current_z_dot_raw_, current_z_dot_, 0.22);
```

如果 v6 不能改善纹波，下一优先级是分析 `Dz / z_dot` 通道，而不是继续削弱 Joint D。

## 9. 调参纪律

1. 以当前最新可运行版本和最新日志为准。
2. 一次只修改一个主要控制因素。
3. 保持 RECOVERY 的已验证稳定参数作为基线。
4. 状态切换时保证控制律连续。
5. 不要同时修改 LQR、腿部阻抗、Joint PD、状态机。
6. 不要仅凭图像下结论；优先结合 CSV、终端日志和源码。
7. 已稳定的阻尼或保护逻辑不要为了“简化代码”而删除。
8. 所有新参数都要说明来源：已有验证值、推导值或明确的实验值。
9. 不要声称“已解决”，除非最新仿真实验已经验证。
10. 不要声称“编译成功”，除非确实执行了编译。

## 10. 建议保留的日志

至少记录：

```text
timestamp
state
state_name

z
z_dot
pitch
pitch_rate

cmd_x
x
x_dot

hip_pos_left
knee_pos_left
hip_pos_right
knee_pos_right

hip_cmd_left
knee_cmd_left
hip_cmd_right
knee_cmd_right

F_z
```

如果继续定位稳态纹波，建议新增：

```text
current_z_dot_raw
current_z_dot
force_target
height_force_per_leg

hip_vel_left
knee_vel_left
hip_vel_right
knee_vel_right

tau_ff_hip_left
tau_ff_knee_left
tau_ff_hip_right
tau_ff_knee_right

tau_body_per_hip
```

这样可以区分：

- Joint P 引起的真实机械振荡
- Joint D 输入纹波
- 高度 Dz 输入纹波
- Jacobian 随构型变化产生的前馈纹波
- Body pitch PD 与 wheel LQR 的耦合

## 11. v6 运行时确认

进入跳后 BALANCE 后应看到：

```text
[Effort连续接管 v6]
```

以及：

```text
[BALANCE Joint-P退坡 v6]
```

日志应显示：

```text
HipKp: 18 → 14
HipKd: 固定 4

KneeKp: 30 → 24
KneeKd: 固定 6
```

如果看不到这些标签，说明运行的不是当前 v6。

## 12. 编译

只修改 `bbot_balance_controller` 时：

```bash
cd ~/bbot_ws_new
colcon build --packages-select bbot_balance_controller --symlink-install
source install/setup.bash
```

如果修改了 `bbot_description`、`bbot_bringup`、controller YAML 或 URDF/xacro，则需要同时重新编译相关 package，并确认当前 shell 使用最新 install overlay。

## 13. 对后续代码代理的要求

- 先读取当前源码、YAML、URDF、最新日志，再修改。
- 用户环境是 ROS 2 Iron，不要默认 Jazzy。
- 不要从旧文件推断当前实现。
- 不要猜 controller 名称、joint 名称、topic、参数路径。
- 当前文件与历史记录冲突时，以最新运行代码和最新实验为准。
- 用户要求完整代码时，给完整可替换源文件。
- 修改后至少做结构检查：
  - 花括号平衡
  - 修改目标函数唯一命中
  - 关键参数确实写入
  - 日志版本标签正确

## 14. 最新实验结论（2026-09）

- RECOVERY 已反复验证可稳定承重：典型日志 `z≈0.430 m`、`vz≈0`，俯仰和角速度较小。
- 一旦切到 BALANCE，会在数百毫秒内发散；因此当前代码中
  `kHoldRecoveryAfterJump = true`，稳定后持续 RECOVERY。
- 最新日志发现跳后 BALANCE 中曾出现 `cmd_x=10`。原因是
  `post_landing_translation_feedback_` 被设为 `false`，导致保护限幅跳过；
  已修正为未来重新启用 BALANCE 时设为 `true`。
- 已尝试：降低/移除 Joint P、移除 Joint D、纯 `J^T mg` 支撑、零空间构型保持。
  均尚未证明可稳定替代 RECOVERY 参数；不要将这些实验方案称为可靠基线。
- 当前优先目标是先确认“长期 RECOVERY 保持”稳定，再以单变量实验重新设计
  RECOVERY → BALANCE 的连续接管。
