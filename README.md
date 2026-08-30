## 系统运行与操作手册

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
