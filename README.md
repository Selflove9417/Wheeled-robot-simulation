## 系统运行与操作手册

### 新版--可跳
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump

ros2 run bbot_balance_controller teleop_keyboard

python3 ~/bbot_ws_new/src/bbot_balance_controller/src/data_logs/plot_jump_log.py

### 1. 编译与环境变量加载
```bash
cd ~/bbot_ws_new
source /opt/ros/iron/setup.bash
colcon build --symlink-install
source ~/bbot_ws_new/setup_env.sh
```

### 2. 启动仿真与控制器

默认加载 **综合平衡性能测试场 (`balance_test_world.sdf`)**，包含斜坡、减速带、绕桩与单边障碍。

#### 方式 A：启动可跳跃平衡控制器（默认）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump
```

#### 方式 B：启动 PID 自平衡控制器
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=pid
```

#### 方式 C：启动 LQR 自平衡控制器
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=lqr
```

#### 方式 D：切换回纯空场景（empty.sdf）
```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump world:=empty.sdf
```

---

### 测试场区域与性能测试指南

机器人出生点位于中央 `(0, 0, 0.403)`，机身正前方为 $+Y$：

| 区域 | 相对出生点方位 | 地形特征 | 测试目的 |
| :--- | :--- | :--- | :--- |
| **测试区 1：多级斜坡** | 正前方 ($Y > 2.0\text{m}$) | 5° 缓坡 (绿)、10° 中坡 (黄)、15° 陡坡 (红)、8° 拱桥平台 (青) | 测试俯仰倾角平衡调节、爬坡力矩极限与上下坡过渡平稳性 |
| **测试区 2：减速带与台阶** | 左侧 ($X \approx -4.5 \sim -6.5\text{m}$) | 1.5cm / 3.0cm / 4.5cm 减速凸起条、连续波浪路面 (Rumble Strip) | 测试轮地突发冲击扰动抑制与微分阻尼响应 |
| **测试区 3：绕桩与避障** | 右侧 ($X \approx 6.0 \sim 9.0\text{m}$) | 5 根蛇形避障立柱 (橙)、静态箱体通道 (深灰) | 测试高速转向向心加速度下的偏航与侧倾动态平衡 |
| **测试区 4：单边高差与跳台** | 后方 ($Y < -2.0\text{m}$) | 左右单侧轮 3cm/4cm 凸起条 (紫)、8cm/15cm 阶梯跳跃台 | 测试单轮过障横滚抗扰、双腿差动高度及跳跃控制器着陆稳定性 |

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

## 核心代码文件修改索引表

| 模块 / 文件 | 主要修改内容 |
| :--- | :--- |
| [bbot.urdf.xacro](file:///home/admin/bbot_ws_new/src/bbot_description/urdf/bbot.urdf.xacro) | 替换车轮碰撞体为解析柱体（$R=0.07\text{m}$）；注入 Gazebo IMU 系统插件；清理异常刚度配置 |
| [bbot_controllers.yaml](file:///home/admin/bbot_ws_new/src/bbot_bringup/config/bbot_controllers.yaml) | 修正轮径为 `0.07`；配置差速控制器与腿部位置控制接口 |
| [bbot_gazebo.launch.py](file:///home/admin/bbot_ws_new/src/bbot_bringup/launch/bbot_gazebo.launch.py) | 增加 `controller_type` 参数；移除 `--stopped`；配置 IMU/Clock Bridge 与平稳拉起机制 |
| [balance_controller_keyboard.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/balance_controller_keyboard.cpp) | 接入 `/cmd_vel`、`/target_height`、`/robot_mode` 话题；同步 $R=0.07\text{m}$ 与高度范围 |
| [lqr_balance_controller_yaokong.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) | 纠正 LQR 输出符号与起立恢复方向；启用完整状态反馈矩阵；同步物理参数 |
| [teleop_keyboard.cpp](file:///home/admin/bbot_ws_new/src/bbot_balance_controller/src/teleop_keyboard.cpp) | 新增纯 C++ 原生遥控终端程序，杜绝 Conda/Python 环境冲突 |
| [robot_params.hpp](file:///home/admin/bbot_ws_new/src/bbot_kinematics/include/bbot_kinematics/robot_params.hpp) | 同步动力学模型参数 $R=0.07\text{m}$ |
