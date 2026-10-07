# 开源参考跳跃试验版（当前未通过落地验收）

本轮新文件：`src/bbot_balance_controller/src/bbot_reference_jump_controller.cpp`。它参照本机 `/home/admin/wheeled-bipedal-jumping/controllers/my_controller_python/motion.py::jump` 的有限推地脉冲、上升期收腿、顶点后展腿流程，沿用 BBOT 的 ROS 接口、平衡/下蹲/恢复、CAD 力矩分配和真实接触确认。没有照搬 Webots 的优化系数或35 Nm参数，也没有放宽关节范围。

**当前决定：不把这个候选设为默认。** 前一轮三次正常速度（real_time_factor=1.0）试跑均未完成站稳，其中最后一次完成了正常收腿/展腿，但触地确认时仍后仰约29°。本轮两次提前塑形试验也未通过，均未进入正常离地；该改动已默认关闭，保留显式试验开关。恢复版继续保留为“能跳完但会后退”的运行对照，其二连跳通过不代表满足最终落地要求。

## 已实现

- 独立可执行文件 `bbot_reference_jump_controller`；启动日志含 `[REFERENCE_JUMP]`，ROS 节点名及 `/jump_cmd` 保持兼容，便于复用启动检查。
- 90 ms平滑上升的有限推力脉冲替换旧的 Bezier/速度反馈叠加；同时间戳的新鲜 COM 速度达标后锁存卸力，320 ms后强制终止继续增冲量。速度达标不替代真实离地确认。
- 用 BBOT CAD 零位重新选定名义收腿高度0.52m、触地高度0.60m。名义收/展预算固定为0.14/0.12s，实际规划仍校验实测关节边界、速度、加速度和剩余时间。原版 `flight_tuck_nominal_duration` 参数不控制这份固定试验预算。
- 收腿参考完成后保持至新鲜 COM 顶点事件；剩余落地时间不足则提前展腿。
- 末端卸力加入样本延迟和停止距离预算。200rad/s²是本轮试验的制动估算，不是已辨识的物理保证。
- 原版控制器、默认参数文件、世界速度、MPC六个文件及原版二进制校验均未改变。

## 实跑结果

| 候选 | COM目标 | 正常收腿/展腿 | THRUST/FLIGHT膝角最小值 | 完整验收 |
| --- | --- | --- | --- | --- |
| 脉冲首版 | 0.30m | 未完成 | −1.57000rad，碰限位 | FAIL |
| 加延迟预瞄 | 0.30m | 完成 | −1.57009rad，仍碰限位 | FAIL |
| 加停止距离预算 | 0.20m | 完成 | −1.49322rad，避开硬限位 | FAIL：后仰、非轮碰撞、未站稳 |

名义高度不是实跑达到的高度。最后一次收腿结束时左髋目标0.497rad、实际0.780rad；左膝目标−0.640rad、实际−0.814rad。空中轨迹跟踪和机身反作用仍需解决。该候选尚无重复起跳通过记录，也不能宣称已经提高了可靠轮底净空。

构建通过，最终相关15项检查通过。所有试验保留严格接触/姿态/恢复审计，未放宽验收条件。

## 本轮接地构型试验

只改独立参考版的接地构型，尝试在下蹲期提前建立质心前置；推力脉冲、姿态增益、关节/命令上限、飞行策略、正常RTF1和动作时长保持原值。

| 前置目标 | 实际下蹲起始高度 | 结果 |
| --- | --- | --- |
| 30mm | 0.473823m | 推力门未开；触发腿长缩短保护，无正常FLIGHT |
| 10mm | 0.498794m | 初始推力门超时，无正常FLIGHT |

30mm轨迹在上一组约0.474m起点下需要髋参考峰值约2.28rad/s，超过既有2rad/s位置命令斜率。10mm在同起点的离线预算约1.90rad/s，但第二次实跑的起点更高，不能把这项离线通过当作该次真实运动的执行保证。两组都有机身继续前倾及非轮触地，没有起跳高度、触地缓冲或重复跳跃通过证据。

`reference_ground_launch_shaping` **默认false**，回到前一轮中立接地构型路径。失败候选代码和原始数据均保存，显式启用只供后续诊断；该开关不能作为已修好的功能推荐。

新增参考世界只加被动位置记录，不改变默认世界或物理设置。两次原生位置与接触数据分别有15283/21063帧，均以1ms仿真步同步；严格验收仍判失败。

[本轮复盘](/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/reference_ground_momentum_20261003_115551/REVIEW.md) · [指令扭矩、姿态与模型动量图](/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/reference_ground_momentum_20261003_115551/ground_launch_review.png)

## 查看效果

保留版完整跳跃对照：

```bash
cd /home/xy/bbot_ws_new
./run_complete_jump_demo.sh
```

观察新的试验版（当前会落地失败，用于核查动作）：

```bash
cd /home/xy/bbot_ws_new
./experiments/jump/run_reference_jump_demo.sh
```

两个入口分别选定各自的程序。新入口默认正常速度、一次跳跃、COM目标0.20m；后续参数可在命令末尾覆盖。

## 扭矩和证据

[关节指令扭矩/姿态总览](/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/reference_jump_20261003_111540/torque_analysis/jump_torque_overview.png)

[起跳指令扭矩细节](/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/reference_jump_20261003_111540/torque_analysis/takeoff_torque_detail.png)

[本轮复盘](/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/reference_jump_20261003_111540/REVIEW.md)

这些曲线是**下发的扭矩命令**。当前 JointState effort反馈全为0；`actual_tau`字段与命令逐行相同，不能当作独立实测力矩。图中撞地后的姿态回弹也不是恢复站稳。

后续诊断显示，空中总后翻角动量约1.70kg·m²/s，单看机身陀螺仪接近零会漏掉腿部运动。本轮提前调整接地构型失败；末段髋反馈已限幅，单改末段位置目标也没有控制余量。下一步必须对齐接地构型与平衡参考，在保持动作速度的前提下先稳定通过推力门，再验证真实离地总角动量。默认提升依据必须包括可靠轮底净空、屈膝轮先触地、竖直/前倾姿态、不后退和连续两跳；仅有控制器SUCCESS不足以通过。
