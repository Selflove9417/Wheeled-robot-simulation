# 平地跳跃：已恢复的完整跳跃基线

当前默认版本已经在正常速度（实时倍率 1.0、物理步长 1 ms）完成连续两次完整跳跃，通过原有接触、腾空流程和 12 秒落地保持审计。实测两跳的质心上升分别约 22.4 cm、20.2 cm。两次均经过正常 TUCK/EXTEND 并恢复站稳。

这是用户要求的“可以完整跳跃，但落地后会后退”的对照基线。低轮底净空、后倾和后退仍然存在；尚未达到后续高跳、屈膝且竖直/前倾落地、无倒退的目标。COM 上升量与轮底净空是不同指标。

## 启动查看

```bash
cd /home/xy/bbot_ws_new
./run_complete_jump_demo.sh
```

脚本打开 Gazebo，自动完成两次跳跃，在最后一跳站稳观察后退出；不必另发跳跃命令。记录保存至 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/restored_demo_时间戳`。如需无界面运行，在脚本后加 `--headless`。

手动启动（两个终端都需要设置相同 ROS_DOMAIN_ID 并加载环境）：

```bash
cd /home/xy/bbot_ws_new
export ROS_DOMAIN_ID=87
source /opt/ros/iron/setup.bash
source /home/xy/bbot_ws_new/install/setup.bash
ros2 launch bbot_bringup bbot_gazebo.launch.py controller_type:=jump_velocity real_time_factor:=1.0
```

待站立稳定后，在另一个终端请求跳跃：

```bash
ros2 topic pub --once /jump_cmd std_msgs/msg/String '{data: jump}'
```

## 恢复和验证说明

恢复依据为 2026-10-01 的 `default_profile_rtf050_latest_sensor_qos_campaign_v2_20261001` 连续六跳通过记录。该记录使用半速仿真，本次默认保持正常速度并重新完成两跳审计。运动参数恢复为 COM 上升目标 0.20 m、接近速度 0.35 m/s、离地前向速度 0.45 m/s、收腿/触地参考高度 0.66/0.69 m、缓冲高度 0.34 m。

历史 56 个文件中，55 个文件的内容与旧校验值完全匹配；主控制文件由补丁和旧源码读取记录重建，仍未匹配历史 SHA256，不能称为逐字一致的历史版本。上述通过结论针对本次实际构建和运行的程序。另有正常速度配置、帮助文字和命令行行为测试的明确调整。

首次单跳已完整腾空和恢复，但记录提前结束，缺少 12 秒保持覆盖，因此该次审计失败。随后的连续两跳补齐原有观察窗口并通过；没有放宽验收阈值。14 项跳跃相关检查通过。正式仿真前后 85 个源码/程序文件保持不变，所有自有仿真进程已退出；六个 MPC 文件保持原样。

当前恢复版本单独保存在 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/restore_complete_jump_20261003_102513/restored_snapshot`；恢复前的试验代码和程序保存在同目录的 `before_restore`。后续借鉴开源项目的改动应使用单独的试验入口，在通过验证前保留本版默认。

本次恢复记录（本地资料：`/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/restore_complete_jump_20261003_102513/REVIEW.md`）。髋膝扭矩日志目前仍主要反映控制命令，不能当作有效的电机轴实测扭矩。
