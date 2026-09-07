# 2026-09-05 跳跃边界修复（jump-boundary-v2）

本说明补充 agents3.md。修改已完成静态/数值验证；尚未运行新的 Gazebo 实验，不能据此认定跳跃或落地已经稳定。

## 本地已有日志证据

读取修改前的 jump_velocity_control_log.csv：

- 5.230 s 首个 FLIGHT 记录已经是 PROTECTIVE_DEPLOY，world_z=0.644 m，world_vz=1.620 m/s，pitch=-0.121 rad，pitch_rate=-0.606 rad/s。
- 5.318 s pitch_rate=-4.804 rad/s，随后实际下发力矩反复正负饱和。
- 5.588 s 首个 TOUCHDOWN_BUFFER 记录 world_z=0.289 m，pitch=-0.993 rad。

这比 agents3 第 7 节的实验更新，说明完整保护轨迹本身尚未消除振荡。上述是状态首个日志样本，不是精确物理接触时刻。

## 修复内容

1. THRUST 的 pd_h_l/r、pd_k_l/r 原本仅用于预算，实际发布函数却重新计算未限幅 PD。现在将已限幅 PD 显式加入各腿力矩，通用发布层不重复计算；预算收紧时推力不得因斜率限制而超过当前预算。
2. FLIGHT 保留左右四关节各自的解析目标速度，不再平均左右速度。TUCK、EXTEND 以及中途转保护均继承切换时刻旧轨迹的 q/qdot/qddot；正常收腿/展腿不再从 FK->IK 重置关节目标。离地保护仍保留原先 0.24 s 全段轨迹。
3. 触地窗口使用世界机身高度减去实际轮底到机身的几何距离，并在推地开始时标定高度偏差。计算包含 URDF 髋偏移 (y=0.125,z=-0.07) 的旋转，不能把 FK 本身当成世界飞行高度。
4. 保留 60 ms 下降历史，使冲击后速度回零仍能触发；窗口必须有 100 ms 内的有效世界里程计。仅膝力矩不能触发，另需压缩速度或 IMU 冲击。启动跳跃前要求世界里程计有效。
5. 初触地预充使用与缓冲阶段相同的受限俯仰补偿。
6. CSV 末尾追加实际/目标四关节速度、wheel_clearance 和 contact_window，保留原字段顺序。

主要增益、air_wheel_sign=+1、推地髋竖直分配=0、RECOVERY 承重路径保持原值。旧控制器、已有 CSV 没有修改。

## 验证

```bash
python3 src/bbot_balance_controller/scripts/test_jump_boundaries.py
source /opt/ros/iron/setup.bash
colcon build --packages-select bbot_balance_controller --symlink-install
git diff --check
```

回归脚本从实际 C++ 提取轨迹/几何/门控实现编译，验证不对称目标速度、中途切换 C2 连续性、URDF 独立变换的几何一致性、空中冲击/单独力矩不触发、触地停止下降后的捕获和过期里程计拒绝。不会启动 ROS 节点或覆盖实验日志。

## 下一轮 Gazebo 验收

启动输出应含 `[jump-boundary-v2]`。按 agents3 的离地/空中/触地标准重新运行，并特别检查新增四关节速度与目标速度、力矩饱和交替是否减轻。触地窗口必须对应轮子接近地面，结合仿真画面确认轮子先于膝盖接触。

本轮没有调整空中阻尼、力矩斜率或轮速增益。已有日志中的振荡原因尚未完全辨识；若新日志仍反复饱和，需利用新增速度字段分析采样、关节阻尼与力矩限速相位关系。轮底间隙基于当前模型的平面几何，不是接触传感器；强侧倾、不平地面及飞行中里程计中断需要额外验证。
