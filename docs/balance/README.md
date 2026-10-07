# 自平衡控制方式

[文档目录](../README.md) · [项目首页](../../README.md)

所有控制方式由 [`bbot_gazebo.launch.py`](../../src/bbot_bringup/launch/bbot_gazebo.launch.py) 的 `controller_type` 选择。下表按 launch 实际分支列出平衡控制器；`jump` 和 `jump_velocity` 跳跃入口见[跳跃文档](../jump/README.md)。

| 方法 / launch 参数 | 节点可执行文件 | 源码 | 简介与配置 |
| --- | --- | --- | --- |
| 自适应 LQR · `adaptive_lqr` | `adaptive_lqr_balance_controller` | [adaptive_lqr_balance_controller.cpp](../../src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp) | LQR 平衡并在线适配名义平衡点；[方法说明](../../src/bbot_balance_controller/src/README_adaptive_LQR.md) |
| 增益调度 LQR · `gs_lqr` | `lqr_gain_scheduled_controller` | [lqr_gain_scheduled_controller.cpp](../../src/bbot_balance_controller/src/lqr_gain_scheduled_controller.cpp) | 按腿高插值调度 LQR 增益；[方法说明](../../src/bbot_balance_controller/src/README_LQR_gain_scheduled.md) |
| 历史 GS-LQR 对照 · `gs_lqr_historical` | `adaptive_lqr_balance_controller` | [adaptive_lqr_balance_controller.cpp](../../src/bbot_balance_controller/src/adaptive_lqr_balance_controller.cpp) | 使用独立 nominal 配置运行历史对照；[配置](../../src/bbot_balance_controller/config/gs_lqr_historical_experiment.yaml) |
| 速度级 LQR · `lqr` | `lqr_balance_controller_yaokong` | [lqr_balance_controller_yaokong.cpp](../../src/bbot_balance_controller/src/lqr_balance_controller_yaokong.cpp) | 速度外环与姿态状态反馈；[方法说明](../../src/bbot_balance_controller/src/README_LQR.md) |
| 速度级串级 PID · `pid` | `balance_controller_keyboard` | [balance_controller_keyboard.cpp](../../src/bbot_balance_controller/src/balance_controller_keyboard.cpp)、[pid.cpp](../../src/bbot_balance_controller/src/pid.cpp) | 速度、姿态与角速度串级反馈；[方法说明](../../src/bbot_balance_controller/src/README_PID.md) · 键盘交互入口 |
| 力矩串级 PID · `torque_cascade_pid` / `torque_pid` | `torque_cascade_pid_controller` | [torque_cascade_pid_controller.cpp](../../src/bbot_balance_controller/src/torque_cascade_pid_controller.cpp) | 直接轮力矩的高度调度串级反馈；[增益配置](../../src/bbot_balance_controller/config/torque_cascade_pid_gains.yaml) |
| 位置误差力矩 PID · `position_torque_cascade_pid` | `position_torque_cascade_pid_controller` | [position_torque_cascade_pid_controller.cpp](../../src/bbot_balance_controller/src/position_torque_cascade_pid_controller.cpp) | 将位置误差纳入轮端力矩串级反馈；[增益配置](../../src/bbot_balance_controller/config/position_torque_cascade_pid_gains.yaml) |
| 模型预测控制 · `mpc` | `linear_mpc_balance_controller` | [linear_mpc_balance_controller.cpp](../../src/bbot_balance_controller/src/linear_mpc_balance_controller.cpp) | 以受约束滚动优化生成平衡控制量；[参数](../../src/bbot_balance_controller/config/mpc_balance_params.yaml) · [方法说明](../../src/bbot_balance_controller/src/README_MPC.md) |

启动示例：

```bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=adaptive_lqr
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=mpc
```

## 共享入口

- 仿真启动：[bbot_gazebo.launch.py](../../src/bbot_bringup/launch/bbot_gazebo.launch.py)
- ros2_control 配置：[bbot_controllers.yaml](../../src/bbot_bringup/config/bbot_controllers.yaml)
- 实验、诊断与运行脚本：[scripts/](../../src/bbot_balance_controller/scripts/)
- 单元测试：[test/](../../src/bbot_balance_controller/test/)
- 平衡控制原始历史数据：本地资料 `src/bbot_balance_controller/src/data_logs/`
- 跳跃实验原始数据：本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/`

上述数据目录不随 GitHub 源码分支自动上传；请通过[跳跃记录索引](../../experiments/jump/RECORDS.md)查看报告与已标注的本地记录路径。

## 历史报告

- 各方法的历史详细说明：[项目参考文档](../PROJECT_REFERENCE.md)。
