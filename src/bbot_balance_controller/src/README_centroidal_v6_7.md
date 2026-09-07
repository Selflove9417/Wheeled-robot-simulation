# v6.7：区分机身状态与整机重心状态

状态：代码及离线验证；未运行 Gazebo，不能据此宣称已跳高或稳定落地。
用户负责仿真。启动标识为 `[centroidal-v6.7]`。

## 本次日志证据

使用 2026-09-06 17:04 的 jump_velocity_control_log.csv：

- 10.633 s，THRUST 的 base_link 世界竖直速度约 1.94 m/s，轮子仍贴地，v6.6 已启动卸力；同时膝关节还在约 -7.69 rad/s 伸展。机身原点速度不能代表含双轮和腿部的整机竖直动量。
- 11.094 s，箱体 pitch=+0.363 rad，但按当前 CAD 质量及关节角重建，整机 COM 位于轮轴后方约 0.074 m，重心倾角约 -0.272 rad。旧箱体角度轮控仍向前追赶，方向与重心位置冲突。
- 此前图中 gazebo_world_z/gazebo_world_z_dot 是 base_link 原点的世界状态，不是质量加权整机 COM。旧字段保留原语义。

## 修改范围

1. 新增七刚体 COM 几何、关节雅可比及重心相对双轮平均轮轴的倾角/角速度。模型与当前 URDF 一致，质量参数跟随 body_mass；沿用控制器的 17.5 kg 模型（省略固定 IMU 的 0.01 kg）。
2. 起跳速度反馈及末段刹腿/卸力采用质量加权世界 COM 速度。插值关节位置到 odom 时间戳，使用完整 odom 姿态旋转 COM，再按真实时间差分。失效、过期或无法对齐的估计不能作为速度达标证据，也不启动速度不足追加推力。保留原力矩预算、行程和姿态保护、推地超时。
3. 临近触地的轮控交接及 CATCH/PREPARE/BRAKE/HOLD/RECOVERY 共用重心倾角及其变化率，CATCH 的重心参考为轮轴正上方。捕获释放仍同时要求箱体姿态/角速度和轮速稳定。
4. 箱体髋姿态控制、地面关节阻尼和离散求解、腾空关节轨迹和轮控符号、TUCK/APEX 时间参数保持原值。

本次主要改变反馈状态定义，未加大电机力矩或轮速上限。目标高度仍为 0.20 m，能否达到受实际行程、力矩和姿态约束，需新仿真数据判断。

## 新日志字段

- com_world_z / com_world_vz：整机 COM 世界高度/竖直速度。
- com_velocity_valid / com_sample_stamp：估计有效性及对应传感器时间戳。
- com_forward_from_axle / com_height_above_axle：COM 相对平均轮轴的前向位置/高度。
- com_lean / com_lean_rate / com_balance_valid：用于地面轮控的重心倾角/角速度及有效性。
- thrust_feedback_vz：最近一次 THRUST 实际采用的速度反馈。
- capture_state 在落地阶段现在基于 COM，相位日志明确区分 body_pitch 和 com_err/com_rate。

## 离线验证与仿真关注点

- 直接解析当前 URDF 独立计算变换，120 个左右不对称姿态、9.5/14 kg 两种机身质量，与新几何模型误差小于 1e-12 m。
- 单元测试覆盖重心角速度与数值导数、站姿重心零点、落地日志帧反馈方向、异步插值、重复/过期/无效样本、腿部运动不应污染整机速度估计。原有空中/地面离散反馈和相位测试继续保留。
- 这些测试不是闭环接触仿真，不能证明机器人实际稳定。
- 下次运行先检查 com_velocity_valid 是否正常，以及卸力时 com_world_vz，而非只看 gazebo_world_z_dot；接着对照 body_pitch 与 com_forward_from_axle，检查接触压缩阶段轮控是否及时追赶重心。保留关节力矩波形，检查是否出现新的振荡。
