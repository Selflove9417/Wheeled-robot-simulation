# agents3.md — BBot 速度跳跃控制当前基线与调试交接

## 1. 文档定位

本文记录 `/home/admin/bbot_ws_new` 中速度跳跃控制器截至 2026-09-04 的真实状态。

后续修改必须先区分：

- **已由 Gazebo 日志验证**：可以作为事实使用；
- **已实现且编译通过、尚未运行验证**：只能作为待验证方案；
- **历史失败方案**：不得在没有新证据时重复恢复。

不得因为代码能够编译就声称跳跃问题已经解决。

## 2. 主要文件

- 当前目标控制器：
  `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`
- 旧跳跃控制器：
  `src/bbot_balance_controller/src/bbot_jump_controller.cpp`
- 绘图脚本：
  `src/bbot_balance_controller/src/data_logs/plot_jump_log.py`
- 最新逐周期日志：
  `src/bbot_balance_controller/src/data_logs/jump_velocity_control_log.csv`
- 跳跃汇总：
  `src/bbot_balance_controller/src/data_logs/jump_velocity_summary.csv`
- 启动文件：
  `src/bbot_bringup/launch/bbot_gazebo.launch.py`
- 运动学实现：
  `src/bbot_kinematics/src/kinematics.cpp`
- 可参考开源项目：
  `/home/admin/wheeled-bipedal-jumping`

旧控制器是行为参考，不应在未获得用户明确授权时直接改动。

## 3. 环境与接口约定

- ROS 2 Iron；
- Gazebo / `gz_ros2_control`；
- 主控制循环约 200 Hz；
- Position 与 Effort 控制器必须互斥；
- 轮毂由差速速度控制器接收 `TwistStamped`；
- 四关节命令顺序固定为：左髋、左膝、右髋、右膝；
- 轮半径约 `0.07 m`；
- 当前控制器运行质量参数约 `17.5 kg`，不要继续引用早期文档中的 `22 kg` 作为当前代码事实；
- IMU 约定：`pitch > 0` 为前倾，`pitch < 0` 为后仰；
- 当前运动学速度符号：

  ```cpp
  x_dot_ = -wheel_radius_ * 0.5 *
           (left_wheel_vel_ + right_wheel_vel_);
  ```

- 当前空中轮速控制的已验证默认符号为：

  ```text
  air_wheel_sign = +1.0
  ```

  `AGENTS2.md` 中的 `-1.0` 已经过时，不得照搬。

## 4. 顶层状态机

必须保持：

```text
BALANCE → SQUAT → THRUST → FLIGHT → TOUCHDOWN_BUFFER → RECOVERY
```

FLIGHT 内部子状态：

```text
ATTITUDE_ARREST / TUCK / EXTEND / PROTECTIVE_DEPLOY
```

当前绝大多数实验因为离地姿态超标，走保护路径。

## 5. 已验证的总体进展

### 5.1 已经可以真实跳起

最近多次实验的世界系离地速度约为：

```text
1.30 ~ 1.40 m/s
```

世界系质心/机身高度有清晰抛物线，说明机器人已经真实离地，不是腿部 FK 高度造成的假跳跃。

目标离地速度约为 `1.98 m/s`，目前高度仍未完全达标，但它不是当前首要问题。

### 5.2 当前首要故障

机器人能够起跳，但推地末段和空中腿部角动量会使机身后仰。随后腿型失控，可能出现膝关节先于轮子接地，最终翻倒。

当前优先级：

1. 控制离地角速度；
2. 保持空中关节轨迹的位置和速度连续；
3. 保证轮子先触地；
4. 在此基础上再调 TOUCHDOWN 捕获；
5. 最后再提高跳跃高度。

## 6. 已由日志确认的关键结论

### 6.1 后仰不是单一轮速符号错误

空中轮速为负时，曾经成功把 `pitch_rate` 从约 `-1.20 rad/s` 拉回到 `+0.30 rad/s`。

因此当前 `air_wheel_sign=+1.0` 的物理方向已有正向证据。除非新日志做出相反的输入响应辨识，不要再次盲目翻转符号。

### 6.2 推地时髋部竖直力矩会抵消纠姿

完整 `J^T Fz` 的髋力矩方向会抵消后仰纠姿力矩。当前 THRUST 已改成：

```text
膝关节：承担主要竖直推力
髋关节：优先承担机身姿态控制
thrust_hip_force_share = 0.0
```

这属于当前结构性选择，不应仅为了恢复传统 `J^T Fz` 而改回去。

### 6.3 “保持离地构型”不能把关节目标速度瞬间设零

一轮实验中，离地瞬间估算关节速度约为：

```text
hip  qdot ≈ +3.6 rad/s
knee qdot ≈ -8.6 rad/s
```

若 FLIGHT 首帧直接令目标速度为零，膝关节会立即产生约 `+56 N·m` 制动力。该内部反力矩会直接传给机身，使 `pitch_rate` 从约 `-1.17` 恶化到 `-4.88 rad/s`。

结论：所有腾空关节轨迹必须同时保证位置和速度连续。

### 6.4 “先刹停腿，再反向展腿”仍然失败

采用 0.10 s 平滑减速后，FLIGHT 第一帧力矩尖峰明显消失，说明速度连续方案有效。

但旧子状态仍等待约 0.15 s 才进入保护展腿。等待期间关节运动到极端构型：

```text
hip  ≈ +0.92 rad
knee ≈ -1.31 rad
```

之后再用 0.20 s 强制反向恢复着陆构型，会产生新的大角动量交换，机身仍会翻倒。

结论：保护离地不能使用“刹停”和“展腿”两条独立轨迹，应该从离地初始状态直接生成一条完整着陆轨迹。

### 6.5 髋关节不能同时强力纠姿和固定腿型

髋关节力矩是机身与整条腿之间的内部力矩。用髋关节强力转动机身，必然把大腿向相反方向甩动。

曾出现：

```text
正常站立髋角约 0.25 rad
空中髋角被甩到 1.0 rad 左右
触地髋角仍约 0.63 rad
```

膝角已经接近目标，但大腿没有回到安全方向，最终膝盖先触地。

当前职责分配应为：

```text
空中轮毂：主要控制机身 pitch / pitch_rate
髋、膝关节：主要控制安全着陆腿型
```

保护展腿阶段不应再用大幅髋姿态力矩覆盖关节轨迹。

### 6.6 IK 的第二参数是 body pitch，不是水平位置

`Kinematics::inverse_kinematics(target_z, body_pitch)` 的第二参数是机身俯仰补偿。

此前 FLIGHT 始终传 `0.0`，机身后仰时整条腿会跟随机身倾斜，增加膝盖先触地风险。

当前空中与初触地阶段使用受限补偿：

```cpp
pitch_comp = clamp(pitch - balance_offset, -0.30, 0.30);
```

### 6.7 膝力矩尖峰不能单独作为触地证据

控制器自身的腿型跟踪也会产生大膝力矩。曾在世界高度约 `0.566 m`、世界竖直速度仅 `-0.135 m/s` 时，因为膝力矩尖峰错误进入 TOUCHDOWN，实际还在最高点附近。

当前触地力矩/IMU判据增加了窗口：

```text
gazebo_world_z_dot < -0.20 m/s
gazebo_world_z <= thrust_start_world_z + 0.035 m
```

只有正在下降且接近地面时，膝力矩尖峰、腿压缩或 IMU 冲击才可触发 TOUCHDOWN。

## 7. 最新一次已运行实验

最新已分析的运行日志关键点：

```text
离地时刻：约 21.057 s
world vertical velocity：1.396 m/s
pitch：-0.183 rad
pitch_rate：-1.045 rad/s

旧保护展腿开始：约 21.205 s
hip：0.917 rad
knee：-1.311 rad
pitch：-0.266 rad
pitch_rate：+0.138 rad/s

失控触地：约 21.412 s
pitch：-1.549 rad
pitch_rate：-9.256 rad/s
```

这次实验说明：

- 关节速度连续接管有效；
- 先等待再展腿的策略失败；
- 保护展腿反向运动是新的主要失稳源；
- 原始空中轮速控制在进入极端腿型后已经没有足够控制余量。

## 8. 当前代码状态：已实现、编译通过、尚未运行验证

最新代码已经将保护离地改为：

1. 在离地瞬间锁存四个关节位置；
2. 锁存并限幅四个关节实际速度；
3. 以当前 `q/qdot` 为五次轨迹初始边界；
4. 以带俯仰补偿的 `L_TOUCH_` IK 为终点；
5. 用一条 `0.24 s` 轨迹直接覆盖整个保护腾空段；
6. 不再先进入 0.15 s 的 ATTITUDE_ARREST，然后反向展腿；
7. 保护展腿期间髋姿态附加力矩为零，由轮毂控制机身俯仰；
8. 触地检测继续使用世界下降速度和接近地面高度门控。

期望新日志出现：

```text
[离地保护] ... 启动全腾空段连续着陆构型轨迹
```

不应再出现：

```text
[腾空保护] 姿态刹车超时 ... 放弃收腿转入展腿保护
```

该版本已执行并通过：

```bash
colcon build --packages-select bbot_balance_controller --symlink-install
```

但用户尚未提供这一最新版本的运行结果，因此不得标记为运行验证成功。

## 9. 当前主要试调参数

```text
L_SQUAT              = 0.34 m
T_SQUAT              = 0.50 s
T_THRUST             = 0.10 s
K_BODY_P_THRUST      = 70
K_BODY_D_THRUST      = 12
TAU_HIP_BODY_MAX     = 20 N·m

T_PROTECTIVE_DEPLOY  = 0.24 s
flight pitch comp    = ±0.30 rad

air wheel Kp         = 0.55
air wheel Kd         = 0.65
air wheel command    = ±1.40 m/s
air wheel slew       = 0.18 m/s per control cycle

landing Kz           = 450 N/m per leg
landing Dz           = 75 N·s/m per leg
landing Fmax         = 240 N per leg
```

这些都是当前仿真试调值，不是理论唯一值。

## 10. 下一轮运行必须检查的量

不要只看最终是否倒下。首先检查离地后前 0.25 s：

1. 是否直接进入 `PROTECTIVE_DEPLOY`；
2. `hip_pos_cmd`、`knee_pos_cmd` 在离地第一帧是否连续；
3. 目标速度是否继承离地关节速度；
4. 髋、膝目标是否单调或平滑趋向着陆构型；
5. 是否仍先运动到 `hip≈0.9 / knee≈-1.3` 再反向；
6. 关节力矩是否出现连续多个周期的饱和交替；
7. `pitch_rate` 是否从约 `-1 rad/s` 快速恶化到 `< -3 rad/s`；
8. 轮速命令是否长期饱和，实际轮速是否达到约 `±20 rad/s`；
9. TOUCHDOWN 是否发生在世界高度合理、且明显下降的时刻；
10. 触地前髋膝实际角是否已接近目标，轮子是否低于膝盖。

### 初步验收标准

```text
离地：|pitch| < 0.15 rad，|pitch_rate| < 1.0 rad/s（目标，当前尚未达到）
空中：不出现 |pitch_rate| > 3 rad/s 的持续发散
触地：轮子先接触，膝关节不接地
触地姿态：进入轮控可捕获域，而不是 pitch 接近 ±π
```

## 11. 后续修改优先级

如果最新全段连续轨迹仍然后仰，按以下顺序排查：

1. 用日志计算离地四关节真实速度，验证估计是否存在采样重复或尖峰；
2. 检查五次轨迹产生的 `q_des/qdot_des` 是否连续；
3. 计算腿部角动量变化与机身 `pitch_rate` 的相关性；
4. 必要时给空中轮速加入基于关节加速度的前馈补偿；
5. 再考虑调整 0.24 s 轨迹时长和终端腿型；
6. 最后才调整轮速 P/D。

若轮速已经达到物理速度上限，继续增加 P/D 不会增加持续反作用力矩，只会更早饱和。

## 12. 明确禁止重复的做法

- 不要再次把空中高速关节目标速度单周期清零；
- 不要先把腿刹到极端构型，再用短轨迹反向展腿；
- 不要让髋关节同时承担大幅机身纠姿和刚性腿型跟踪；
- 不要只凭膝力矩尖峰判断触地；
- 不要把 FK 腿长当作世界系高度；
- 不要盲目翻转 `air_wheel_sign`；
- 不要只增加空中轮速增益，而忽略轮速饱和；
- 不要一次同时修改 THRUST、FLIGHT、TOUCHDOWN 三套主要增益；
- 不要自动切回会导致腿部失去承重的 Position/BALANCE 路径；
- 不要修改旧控制器来掩盖速度跳跃控制器的问题。

## 13. 开源项目的正确使用方式

`/home/admin/wheeled-bipedal-jumping` 可借鉴：

- 推地力矩/轨迹参数化；
- 离地速度和姿态联合损失函数；
- 贝叶斯优化或批量仿真调参流程；
- 腿部运动与机身角动量联合规划的思想。

不能直接照搬：

- 原机器人力矩数值；
- 原质量、杆长、轮径参数；
- 原 Webots 接触传感器逻辑；
- 原关节符号和轮速符号。

在当前空中轨迹连续性尚未验证之前，不应立刻引入完整 BOTP 优化，否则优化器只会补偿状态机和轨迹边界错误。

## 14. 日志字段解释

- `gazebo_world_z / gazebo_world_z_dot`：世界系高度和竖直速度；
- `z / z_dot`：腿部 FK/几何状态，不等价于世界系飞行状态；
- `hip_pos_* / knee_pos_*`：实际关节角；
- `hip_pos_cmd_* / knee_pos_cmd_*`：当前关节位置目标；
- `hip_cmd_* / knee_cmd_*`：控制器输出的关节力矩命令；
- `actual_tau_*`：经过限幅和斜率限制后的实际下发力矩；
- `air_wheel_cmd_raw`：空中轮速控制未限幅原始量；
- `cmd_x`：实际发送给轮控制器的限幅线速度命令；
- `left_wheel_vel/right_wheel_vel`：实际轮角速度；
- `flight_subphase`：`0=ATTITUDE_ARREST`、`1=TUCK`、`2=EXTEND`、`3=PROTECTIVE_DEPLOY`。

分析时不得把 `air_wheel_cmd_raw` 的大数值误认为实际发送指令；实际指令以 `cmd_x` 为准。

## 15. 构建与静态验证

```bash
source /opt/ros/iron/setup.bash
colcon build --packages-select bbot_balance_controller --symlink-install
git diff --check
```

若修改绘图脚本：

```bash
python3 -m py_compile \
  src/bbot_balance_controller/src/data_logs/plot_jump_log.py
```

用户负责实际运行 Gazebo。没有新的运行日志时，只能报告“已实现并编译通过”。

