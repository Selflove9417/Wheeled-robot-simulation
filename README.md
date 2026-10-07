# BBot 自平衡与跳跃工作区

工作区：`/home/xy/bbot_ws_new` · ROS 2 Iron / Gazebo Sim。

当前开发分支为 `work/jump-repair`，该分支与下述跳跃修复仍未验收。`main` 保持旧基线，不能把候选改动视为已合并或已通过。

## 环境与日常启动

```bash
cd /home/xy/bbot_ws_new
source setup_env.sh
```

启动自适应 LQR 平衡：

```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=adaptive_lqr
```

启动默认 velocity 跳跃控制器：

```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump_velocity jump_height:=0.25
```

站立稳定后按 **J**，或在另一个已加载环境的终端发送：

```bash
ros2 topic pub --once /jump_cmd std_msgs/msg/String '{data: jump}'
```

自动演示入口：

```bash
./run_complete_jump_demo.sh
```

`jump_height` 用于生成离地速度目标，不代表实际轮底净空。当前跳跃修复状态和目标验收见 [LANDING_REPAIR.md](LANDING_REPAIR.md)。

## 控制方式

`controller_type` 由 [bbot_gazebo.launch.py](src/bbot_bringup/launch/bbot_gazebo.launch.py) 选择。平衡方式与跳跃方式同级列出；名称对应 launch 参数，配置和源码仍是本地工作区内容。

| 类型 | 用途 | 主要参数 / 配置 | 源码与说明 |
| --- | --- | --- | --- |
| `pid` | 速度级串级 PID 平衡，通过差速控制器输出轮速 | 控制器内的速度、姿态、角速度级参数 | [实现与说明](src/bbot_balance_controller/src/README_PID.md) · [源码](src/bbot_balance_controller/src/balance_controller_keyboard.cpp) |
| `torque_cascade_pid` | 直接轮力矩的串级 PID 平衡；`torque_pid` 为兼容别名 | [增益配置](src/bbot_balance_controller/config/torque_cascade_pid_gains.yaml) | [源码](src/bbot_balance_controller/src/torque_cascade_pid_controller.cpp) |
| `position_torque_cascade_pid` | 位置误差参与轮端力矩串级控制 | [增益配置](src/bbot_balance_controller/config/position_torque_cascade_pid_gains.yaml) | [源码](src/bbot_balance_controller/src/position_torque_cascade_pid_controller.cpp) |
| `lqr` | 速度级 LQR 平衡 | 控制器内的状态反馈与速度环参数 | [实现与说明](src/bbot_balance_controller/src/README_LQR.md) · [源码](src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) |
| `gs_lqr` | 按腿部高度调度增益的 LQR 平衡 | 控制器内的高度节点与增益表 | [实现与说明](src/bbot_balance_controller/src/README_LQR_gain_scheduled.md) · [源码](src/bbot_balance_controller/src/lqr_gain_scheduled_controller.cpp) |
| `adaptive_lqr` | 自适应平衡点的 LQR 平衡 | `adaptive_gain_mode`，如 `scheduled`、`fixed_midpoint` | [实现与说明](src/bbot_balance_controller/src/README_adaptive_LQR.md) · [源码](src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp) |
| `mpc` | 带约束滚动优化的平衡控制 | [参数文件](src/bbot_balance_controller/config/mpc_balance_params.yaml) | [实现与说明](src/bbot_balance_controller/src/README_MPC.md) · [源码](src/bbot_balance_controller/src/linear_mpc_balance_controller.cpp) |
| `jump` | 经典多阶段跳跃与落地缓冲 | 由跳跃状态机配置控制 | [源码](src/bbot_balance_controller/src/bbot_jump_controller.cpp) · [入口说明](docs/jump/README.md) |
| `jump_velocity` | 按目标离地速度生成跳跃动作；当前 launch 默认入口 | `jump_height` 等跳跃配置 | [源码](src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp) · [入口说明](docs/jump/README.md) |

`gs_lqr_historical` 是历史对照入口：复用自适应 LQR 可执行文件并加载 [历史配置](src/bbot_balance_controller/config/gs_lqr_historical_experiment.yaml)，不作为日常默认方式。历史实验材料见[平衡报告索引](docs/balance/README.md)。

示例：

```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=torque_cascade_pid
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=mpc
```

## 文档导航

- [文档目录与文件归类](docs/README.md)
- [自平衡控制方法与配置](docs/balance/README.md)
- [跳跃入口、状态与记录](docs/jump/README.md)
- [独立实验入口与结果索引](experiments/jump/README.md) · [历史实验记录索引](experiments/jump/RECORDS.md)
- [默认 flat-jump 版本说明](DEFAULT_FLAT_JUMP.md) · [跳跃修复历史](docs/jump/LANDING_REPAIR_HISTORY.md)
- [历史综合说明](docs/PROJECT_REFERENCE.md) · [仿真调试手册](docs/BBOT_SIMULATION_DEBUG_MANUAL.md)

## 工作区目录

| 目录 | 用途 |
| --- | --- |
| `src/bbot_balance_controller/` | 平衡与跳跃控制器、脚本、测试和配置 |
| `src/bbot_bringup/` | launch、控制器配置与仿真世界 |
| `src/bbot_description/`、`src/bbot_kinematics/` | 机器人模型与运动学 |
| `docs/` | 面向使用者的项目、平衡与跳跃说明 |
| `experiments/jump/` | 独立候选入口、离线审计和实验导航，不作为默认版本 |
| `src/bbot_balance_controller/src/data_logs/` | 原始历史运行记录，保留原路径 |
| `build*/`、`install/`、`log/`、`ros_log/` | 已有构建与运行产物 |

需要重新构建时，在工作区根目录运行 `colcon build --symlink-install`。现有构建目录和安装产物各自保留原位。
