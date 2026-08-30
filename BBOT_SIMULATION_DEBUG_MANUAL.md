# BBOT 轮腿平衡机器人 Gazebo 仿真调试与问题排查总结文档

本文档全面梳理了在将真实 CAD 模型的 BBOT 轮腿机器人移植至 ROS 2 Iron + Ignition Gazebo (Fortress) 仿真环境过程中所遇到的所有核心问题、根本原因分析、系统性解决方案以及最终的使用与开发规范。

---

## 目录
1. [系统整体架构与环境说明](#一系统整体架构与环境说明)
2. [问题一：模型抽搐、剧烈抖动与大小腿折叠](#问题一模型抽搐剧烈抖动与大小腿折叠)
3. [问题二：IMU 传感器与 ROS-Gazebo 桥接断裂](#问题二imu-传感器与-ros-gazebo-桥接断裂)
4. [问题三：控制程序倒地一动不动、控制器未激活](#问题三控制程序倒地一动不动控制器未激活)
5. [问题四：Launch 启动时键盘控制（WASDQE）失效](#问题四launch-启动时键盘控制wasdqe失效)
6. [问题五：Conda 环境抢占与 Python 依赖缺失](#问题五conda-环境抢占与-python-依赖缺失)
7. [问题六：LQR 控制器失稳、后冲倒地与力矩不足](#问题六lqr-控制器失稳后冲倒地与力矩不足)
8. [问题七：轮径与物理参数的全局统一](#问题七轮径与物理参数的全局统一)
9. [系统运行与操作手册](#三系统运行与操作手册)

---

## 一、系统整体架构与环境说明

- **操作系统**：Ubuntu 22.04 LTS (Linux)
- **ROS 版本**：ROS 2 Iron Irwini
- **仿真引擎**：Ignition Gazebo 6.x (Fortress)
- **硬件控制中间件**：`gz_ros2_control`
- **机器人实体**：BBOT 轮腿自平衡机器人（整机质量约 22.4kg，CAD 结构，髋-膝-轮三连杆结构）
- **坐标约定**：
  - 机身前进方向：$+Y$（CAD 原始前向坐标）
  - 驱动轮转轴方向：$+X$
  - 竖直向上方向：$+Z$

---

## 二、问题与解决方案详解

### 问题一：模型抽搐、剧烈抖动与大小腿折叠

#### 1. 现象描述
- 单独启动 Gazebo 仿真或加载控制器时，机器人落地后关节剧烈抖动抽搐，甚至大小腿折叠在一起产生物理爆炸。

#### 2. 根本原因排查
1. **驱动轮复杂凹面碰撞网格（STL Mesh Collision）**：
   - URDF 中驱动轮使用了复杂三角面片 CAD STL 网格作为 `<collision>`。物理引擎在计算凹面/非凸网格与地面连续接触时，产生高频穿透和接触法向力奇点（Contact Jitter）。
2. **高频无阻尼位置刚度参数**：
   - 硬件接口中配置了 `position_proportional_gain: 600.0`。在未配置微分阻尼（$K_d$）的情况下，硬件接口形成了 30~50Hz 的高刚度弹簧振荡系统。

#### 3. 解决方案
- **修改 URDF 碰撞体为解析几何体**（[bbot.urdf.xacro](file:///home/admin/bbot_ws_new/src/bbot_description/urdf/bbot.urdf.xacro)）：
  将左右轮的碰撞网格替换为解析圆柱体：
  ```xml
  <collision name="link_004_collision">
    <origin xyz="0.03825 0 0" rpy="0 1.57079632679 0" />
    <geometry>
      <cylinder radius="0.07" length="0.04" />
    </geometry>
  </collision>
  ```
- **移除异常的高刚度参数**：
  在 `bbot_controllers.yaml` 与 URDF 中移除 `position_proportional_gain`，采用标准柔顺关节控制接口。

---

### 问题二：IMU 传感器与 ROS-Gazebo 桥接断裂

#### 1. 现象描述
- 启动仿真后，ROS 2 中 `/imu` 话题无任何数据发布，控制程序因为收不到姿态信息无法启动。

#### 2. 根本原因排查
1. **缺失 Gazebo 内部 IMU 系统插件**：Ignition Gazebo 必须显式声明 `ignition-gazebo-imu-system` 插件，否则 IMU 传感器不被物理引擎驱动更新。
2. **消息类型定义不匹配**：Ignition Fortress 底层发布的是 `ignition.msgs.IMU`，而桥接配置误写为 `gz.msgs.IMU`。

#### 3. 解决方案
- 在 URDF 中注入系统级 IMU 插件：
  ```xml
  <gazebo>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu" />
  </gazebo>
  ```
- 修正 `ros_gz_bridge` 桥接定义：
  ```python
  Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=[
          '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
          '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
      ]
  )
  ```

---

### 问题三：控制程序倒地一动不动、控制器未激活

#### 1. 现象描述
- 机器人不再抽搐，但在启动控制程序后，机身倒在地上完全没有动力输出。

#### 2. 根本原因排查
1. **控制器处于未激活（Inactive）状态**：
   - 启动脚本中添加了 `--stopped` 参数，使 `diff_drive_controller` 和 `leg_position_controller` 默认处于 Inactive 状态。
   - Inactive 状态下控制器管理器会直接丢弃所有发往 `/diff_drive_controller/cmd_vel` 和 `/leg_position_controller/commands` 的指令。
2. **启动时差导致倒地失衡**：
   - 手动分步启动节点时，机器人在控制器加载前已摔倒在地，超出倒立摆线性可控区间。

#### 3. 解决方案
- 移除 spawner 中的 `--stopped` 参数，以 Active 模式直接挂载控制器。
- 在 [bbot_gazebo.launch.py](file:///home/admin/bbot_ws_new/src/bbot_bringup/launch/bbot_gazebo.launch.py) 中，于第 2.0 秒自动接管自平衡控制，实现落地即稳。

---

### 问题四：Launch 启动时键盘控制（WASDQE）失效

#### 1. 现象描述
- 控制器启动后基本能够保持平衡，但在终端按下键盘按键 WASDQE 时没有任何反应。

#### 2. 根本原因排查
1. **Launch 进程的 TTY 隔离机制**：
   - `ros2 launch` 将节点作为后台子进程拉起，子进程的 `STDIN` 脱离了终端 TTY，C++ 内置的 `read(STDIN_FILENO)` 每次直接返回空。
2. **控制器缺少标准话题通信接口**：
   - 控制器仅依赖终端输入扫描，没有开通 `/cmd_vel` 与 `/target_height` 订阅通道。

#### 3. 解决方案
- 为 PID 和 LQR 控制器均增加标准 ROS 2 话题订阅：
  - `/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `/target_height` (`std_msgs/msg/Float64`)
  - `/robot_mode` (`std_msgs/msg/String`)
- 配套提供独立的前台遥控终端节点（在独立窗口读取真实 TTY 输入并发布话题）。

---

### 问题五：Conda 环境抢占与 Python 依赖缺失

#### 1. 现象描述
- 运行 `ros2 run bbot_balance_controller teleop_keyboard.py` 时抛出 `ModuleNotFoundError: No module named 'yaml'` 错误。

#### 2. 根本原因排查
- 用户的终端激活了 Conda `(base)` 环境，默认环境变量 `#!/usr/bin/env python3` 启动了 Conda 的 Python，缺失 ROS 2 相关包。

#### 3. 解决方案
- **提供纯 C++ 原生编译版本** [teleop_keyboard.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/teleop_keyboard.cpp)：
  编译为独立二进制文件，彻底摆脱 Python 解释器和 Conda 环境依赖。
- **将 Python 脚本 Shebang 强制绑定为系统 Python**：
  改为 `#!/usr/bin/python3`。

---

### 问题六：LQR 控制器失稳、后冲倒地与力矩不足

#### 1. 现象描述
- 切换到 LQR 控制器后，机器人无法维持平衡，一启动就猛烈向后冲，轮子想追赶重心但是力气不够直接滑倒。

#### 2. 根本原因排查
1. **控制指令符号相反**：
   - BBOT 的轮轴沿 $+X$ 轴，差速控制器中正线速度（`linear.x > 0`）对应轮子反转（驱动底座向 $-Y$ 移动以挽救后倒）。
   - 原 LQR 代码中由于缺少负号变换，导致机身后倾时发出负线速度（车轮向前加速跑掉），瞬间抽走底座。
2. **LQR 增益被硬编码且被严重低估**：
   - 代码中硬编码了 $K_\theta = 1.8, K_{\dot{\theta}} = 0.20$，仅为理论所需反馈力矩的 10%，轮子加速度严重不足。
3. **起立自恢复模式符号倒置**：
   - `MODE_STANDUP` 中后倒时错误地下发了 `-2.5 m/s`。

#### 3. 解决方案
- 在 [lqr_balance_controller_yaokong.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) 中修正 LQR 控制律与起立模式符号：
  ```cpp
  // 稳态 LQR 纠偏指令
  cmd_x = -u_pitch * cmd_scale_ - target_speed;
  ```
- 启用完整的四状态 LQR 反馈矩阵（$K_\theta = 227.19, K_{\dot{\theta}} = 57.64, K_v = 48.57, K_x = 6.365$），并配合 `cmd_scale_ = 0.05`，等效增益与 PID 对齐。

---

### 问题七：轮径与物理参数的全局统一

#### 1. 现状与核查
- 工程中部分文件混用了 `0.075 m` 与 `0.07 m` 两种轮径定义。

#### 2. 解决方案
- 全局将驱动轮半径统一修正为 **`R = 0.07 m`（70 mm）**，涉及：
  1. URDF 碰撞体柱体半径
  2. `bbot_controllers.yaml` 差速轮径参数
  3. `bbot_kinematics` 运动学正逆解与动力学力矩计算
  4. PID 与 LQR 控制器里程计与状态反馈计算

---

## 三、系统运行与操作手册

### 1. 编译与环境变量加载
```bash
cd ~/bbot_ws_new
source /opt/ros/iron/setup.bash
colcon build --symlink-install
source ~/bbot_ws_new/setup_env.sh
```

### 2. 启动仿真与控制器

#### 方式 A：启动 PID 自平衡控制器（默认）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=pid
```

#### 方式 B：启动 LQR 自平衡控制器
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=lqr
```

#### 方式 C：仅启动 Gazebo 仿真（便于单步物理调试）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=none
```

---

### 3. 启动键盘遥控控制终端

打开一个新的终端窗口：
```bash
source ~/bbot_ws_new/setup_env.sh
ros2 run bbot_balance_controller teleop_keyboard
```

#### 键盘快捷键说明：
| 按键 | 功能 | 说明 |
| :--- | :--- | :--- |
| <kbd>W</kbd> / <kbd>↑</kbd> | **前进** | 目标速度 $+0.35\text{ m/s}$ |
| <kbd>S</kbd> / <kbd>↓</kbd> | **后退** | 目标速度 $-0.35\text{ m/s}$ |
| <kbd>A</kbd> / <kbd>←</kbd> | **左转** | 原地转向速率 $+0.60\text{ rad/s}$ |
| <kbd>D</kbd> / <kbd>→</kbd> | **右转** | 原地转向速率 $-0.60\text{ rad/s}$ |
| <kbd>Space</kbd> | **刹车制动** | 速度归零，保持原地稳定自平衡 |
| <kbd>Q</kbd> | **升高机身** | 质心高度每次 $+1\text{cm}$（行程 $0.30\sim0.50\text{m}$） |
| <kbd>E</kbd> | **降低机身** | 质心高度每次 $-1\text{cm}$（行程 $0.30\sim0.50\text{m}$） |
| <kbd>R</kbd> | **倒地起立** | 触发自动起立冲量恢复状态机 |
| <kbd>X</kbd> | **紧急停机** | 复位所有积分器与控制输出 |

---

## 四、核心代码文件修改索引表

| 模块 / 文件 | 主要修改内容 |
| :--- | :--- |
| [bbot.urdf.xacro](file:///home/admin/bbot_ws_new/src/bbot_description/urdf/bbot.urdf.xacro) | 替换车轮碰撞体为解析柱体（$R=0.07\text{m}$）；注入 Gazebo IMU 系统插件；清理异常刚度配置 |
| [bbot_controllers.yaml](file:///home/admin/bbot_ws_new/src/bbot_bringup/config/bbot_controllers.yaml) | 修正轮径为 `0.07`；配置差速控制器与腿部位置控制接口 |
| [bbot_gazebo.launch.py](file:///home/admin/bbot_ws_new/src/bbot_bringup/launch/bbot_gazebo.launch.py) | 增加 `controller_type` 参数；移除 `--stopped`；配置 IMU/Clock Bridge 与平稳拉起机制 |
| [balance_controller_keyboard.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp) | 接入 `/cmd_vel`、`/target_height`、`/robot_mode` 话题；同步 $R=0.07\text{m}$ 与高度范围 |
| [lqr_balance_controller_yaokong.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) | 纠正 LQR 输出符号与起立恢复方向；启用完整状态反馈矩阵；同步物理参数 |
| [teleop_keyboard.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/teleop_keyboard.cpp) | 新增纯 C++ 原生遥控终端程序，杜绝 Conda/Python 环境冲突 |
| [robot_params.hpp](file:///home/admin/bbot_ws_new/src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp) | 同步动力学模型参数 $R=0.07\text{m}$ |
