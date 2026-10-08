# TUCK → EXTEND 轨迹连续性审查

2026-10-07，仅源码与七次已有完整物理记录；新增实跑0次，未改控制、参数、限制、状态时序，gain默认1.60。T1/T2/T3是已有1.00记录，并非本轮重新调参。未提交/推送。

## 结论

**存在期望位置、速度不连续，但不是新EXTEND从静止生成，也不是后倾最初出现的时刻。** 新段已经以实测q/v初始化；旧TUCK参考与实测有跟踪误差，切换时把参考重置到实测，因而发生参考跳变。七次左右膝位置跳变约−0.108～−0.191rad，髋速度跳变最大约0.135rad/s、膝最大约0.461rad/s。新段首帧与控制器实测q/v之差在CSV精度内全部为0。

后转最初形成早于EXTEND入口81～158ms，早于TUCK目标反向35～45ms的时刻。A2最早后转为短暂推地摆动，其主要后转也早于EXTEND约75ms。因此不能把TUCK→EXTEND跳变称为最初后倾原因；是否加重后续后倾未做隔离物理验证。

## 源码精确边界

源文件 `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`。

### ATTITUDE_ARREST → TUCK

4235–4348调用 `begin_checked_tuck`，完整可行性预算通过后进入4110–4151的 `begin_tuck`。四关节数组索引为左hip、左knee、右hip、右knee，分别采用：

```text
q0 = {hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_}
v0 = {hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_}
tuck_comp = clamp(pitch_ - balance_offset_, -0.24, 0.24)
tuck_ik = inverse_kinematics_with_target_x(L_RETRACT_, tuck_comp, 0.0)
qf[i] = reaction_safe_configuration_step(q0[i],
        i为hip ? tuck_ik.theta_hip : tuck_ik.theta_knee,
        i为hip ? 0.14 : 0.04, v0[i])
normal_flight_joint_traj_[i].init(now_sec,
        flight_round_trip_plan_.tuck_duration,
        q0[i], v0[i], 0.0, qf[i], 0.0, 0.0)
```

`reaction_safe_configuration_step`在 `flight_trajectory.hpp:20–36`：先将位移限制为±maximum_delta；若abs(current_velocity)>0.50且限幅后位移与速度反向，返回current位置。**使用实测q/v重新初始化，不从旧ARREST参考取初值；终速明确为0，初末加速度0。** 本组四关节的qf全部等于q0，已用各次CSV的pitch、q/v及源码IK/保护公式逐关节重算确认，不仅依赖三位小数控制台日志。

### TUCK → EXTEND

4354–4425：`tuck_finished`或`landing_deadline_reached`后锁定落脚计划，以 `normal_landing_plan_feasible(candidate)` 搜索剩余预算内最长可行duration（5ms网格，下限 `T_FLIGHT_EXTEND_`）。4410附近显式设 `flight_trajectory_initialized_=false`，再调用 `plan_flight_joints(...)`。

3988–4043 `plan_flight_joints`：

```text
q = 当前四个实测关节位置
v = 当前四个实测关节速度
a = {0,0,0,0}
if (flight_trajectory_initialized_) sample_flight_joints(now_sec,q,v,a)
ik = inverse_kinematics_with_target_x(
        L_TOUCH_, clamp(balance_offset_+flight_landing_pitch_bias_,-0.30,0.30),
        landing_target_x_)
end = {ik.theta_hip, ik.theta_knee, ik.theta_hip, ik.theta_knee}
traj.init(now_sec,flight_round_trip_plan_.extend_duration,
          q[i],v[i],a[i],end[i],0.0,0.0)
```

由于调用前标志清零，**不会采样旧TUCK参考；实际采用实测位置/速度与零初加速度。** 终位置为落脚IK，终速/终加速度0。4563采样后4569–4577可能进行 `enforce_wheel_first_target` 投影；本组入口最终记录的q/v与实测完全一致，未观察到入口投影造成额外q/v差。

高度 `tuck_traj_` / `extend_traj_` 的零端速不是四关节初速度；不能据其初始化参数断言腿部新段初速度为0。四关节实际走 `normal_flight_joint_traj_`。

### 两段均使用的五次公式

`src/bbot_balance_controller/include/bbot_balance_controller/flight_trajectory.hpp:45–92`，令u=clamp((t−t0)/T,0,1)：

```text
q(u) = a0+a1*u+a2*u²+a3*u³+a4*u⁴+a5*u⁵
qdot(u) = (a1+2*a2*u+3*a3*u²+4*a4*u³+5*a5*u⁴)/T
a0=q0; a1=v0*T; a2=0.5*acc0*T²
Dz=qf-(a0+a1+a2)
Dv=vf*T-(a1+2*a2)
Da=accf*T²-2*a2
a3=10*Dz-4*Dv+0.5*Da
a4=-15*Dz+7*Dv-Da
a5=6*Dz-3*Dv+0.5*Da
```

时长取init传入的duration（系数计算最小1e-4s）；本组远高于该值。TUCK相同q起终点而非零入速意味着有先行位移再返回：当acc0=accf=vf=0、qf=q0时，q=q0+v0*T*(u−6u³+8u⁴−3u⁵)。因此存在中途速度反向，不能把零终速单独等同于切换时对实测速度强制归零。

## 每关节完整数值与数据来源

原始本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007/`。采用七次原CSV/launch日志；版本继承上轮冻结，源文件哈希另见本目录 `version_check.json`。

可复现分析：[analyze.py](analyze.py)。工作区根运行 `python3 experiments/jump/planning/tuck_extend_continuity_20261007/analyze.py`，无ROS/仿真调用。完整28行本地派生表：本目录 `boundary.csv`；含左右hip/knee的TUCK q0/v0、受保护qf/vf、两段时长、切换处旧参考q/v、控制器实测q/v、独立原生q/v、新EXTEND首帧q/v及差值。

旧参考未作为额外一行在切换时发布，故 `q(t−)/v(t−)` 用源码五次式重建，非伪装成额外物理观测；q0/v0源于TUCK首行，qf经源码公式重算。本组TUCK入口至EXTEND入口均≥规划TUCK时长，故边界旧参考为qf/0。CSV约6位有效数字，保留其精度；控制台时长精确到1ms，实际搜索为5ms网格。Native q/v是物理步状态，不强行等同于控制器接收到的测量值。

|试验|TUCK计划ms / 实际阶段ms|EXTEND计划ms|左髋旧q / 新q rad|左膝旧q / 新q rad|左髋速度跳变 rad/s|左膝速度跳变 rad/s|
|---|---|---:|---|---|---:|---:|
|A1|60 / 60|290|0.955361 / 0.987521|-1.422220 / -1.559820|-0.098905|0.157633|
|B1|60 / 60|270|0.873725 / 0.907089|-1.357420 / -1.465280|-0.062864|0.460383|
|A2|60 / 60|270|0.825471 / 0.811380|-1.283800 / -1.404140|-0.055373|-0.067207|
|B2|60 / 70|255|0.791896 / 0.825584|-1.209340 / -1.399840|-0.031453|0.112374|
|T1|60 / 60|280|0.890677 / 0.905428|-1.327280 / -1.474280|-0.082489|0.028567|
|T2|60 / 65|265|0.874247 / 0.899743|-1.427700 / -1.555990|0.135112|0.437114|
|T3|60 / 65|260|0.861421 / 0.877577|-1.249460 / -1.376370|-0.016799|0.377494|

左、右分开量化如下。每项为新参考减旧参考；新初速等于控制器实测初速，旧TUCK边界速度均0。

|试验|髋Δq 左/右 rad|膝Δq 左/右 rad|髋Δv 左/右 rad/s|膝Δv 左/右 rad/s|
|---|---|---|---|---|
|A1|0.032160/0.032142|-0.137600/-0.137610|-0.098905/-0.098954|0.157633/0.157409|
|B1|0.033364/0.033370|-0.107860/-0.107860|-0.062864/-0.062817|0.460383/0.460588|
|A2|-0.014091/-0.014113|-0.120340/-0.120350|-0.055373/-0.055163|-0.067207/-0.067816|
|B2|0.033688/0.033670|-0.190500/-0.190500|-0.031453/-0.031406|0.112374/0.112160|
|T1|0.014751/0.014737|-0.147000/-0.147010|-0.082489/-0.082343|0.028567/0.028180|
|T2|0.025496/0.025501|-0.128290/-0.128290|0.135112/0.135335|0.437114/0.437015|
|T3|0.016156/0.016154|-0.126910/-0.126920|-0.016799/-0.016661|0.377494/0.377258|

这表明主要不连续是膝参考位置回跳，而不是非常大的速度阶跃。不能沿用“EXTEND重新从零速度起步”的判断。

## 按EXTEND入口对齐的时间关系

本地派生图 `A1.png`～`T3.png` 分别覆盖入口前后约0.15s：左右髋膝原生实际速度与CSV目标速度、原生pitch/pitch_rate、轮发布cmd_x、左右原生轮速（图中乘轮半径0.07转换为m/s，与轮指令同轴）、jump state/subphase。没有把轮角速度直接与m/s混为同单位。各原始量仍可从上轮完整切片和本轮边界表复核。FLIGHT子状态0=ATTITUDE_ARREST、1=TUCK、2=EXTEND。

|试验|明显后转最早时刻ms|当时阶段|TUCK入口ms|TUCK四关节目标首次反向ms|EXTEND入口|
|---|---:|---|---:|---:|---:|
|A1|-81|ATTITUDE_ARREST|-60|-35|0|
|B1|-126|THRUST|-60|-35|0|
|A2|-158|THRUST|-60|-35|0|
|B2|-84|ATTITUDE_ARREST|-70|-45|0|
|T1|-84|ATTITUDE_ARREST|-60|-35|0|
|T2|-120|THRUST|-65|-40|0|
|T3|-91|ATTITUDE_ARREST|-65|-40|0|

后转定义沿用前轮只读报告：原生pitch_rate连续15ms低于−0.20rad/s且随后50ms下降至少1°，不是新的控制门。A2早期短暂回正，主要下降约−75ms；保留早期样本，未筛除以制造同步。目标反向的采样定位精度约5ms；实际腿速在ARREST/TUCK期间逐渐制动，不能用目标反向时刻代替实际反向时刻，二者在图中分列。

例如A1：明显后转−81ms；TUCK开始−60ms时原生髋约+1.76、膝约−5.15rad/s，目标约+1.905/−5.317；目标反向−35ms；EXTEND入口原生髋约−0.086、膝约+0.251，控制器实测−0.0989/+0.1576。**后转与腿部制动过程有时间重叠，但发生在目标反向和段间跳变之前。** 不能从时间相关性得出腿制动独自造成机体后转，更不能据此量化净因果贡献；轮反馈和先前角动量同时在作用。

## 一个最小后续方案（不实施）

用户优先建议的“使用切换瞬间实际q/v”在现代码中已经存在，再做一次不会修复参考连续性。在跟踪误差非零时，直接从实测重置与保持旧参考C1连续无法同时成立。

针对本轮确认的**参考断点**，最小候选是只改变TUCK→EXTEND的轨迹初值来源：保留旧TUCK轨迹，在同一 `now_sec` 调用 `sample_flight_joints` 取得旧参考q/v/a作为新段起点，不再在该调用前清除 `flight_trajectory_initialized_`。既有新段时长、IK终点、零终速与其他控制参数均保持。可行性检查须同步使用这同一初值，否则检查实测起点而生成旧参考起点不一致。若固定时长/限制下不可行，则拒绝方案，不改时序或限制。这样针对参考有q(t−)=q(t+)、qdot(t−)=qdot(t+)，但不能宣称新参考等于实测，也不能保证减少首触后倾。

该候选仍须核对 `enforce_wheel_first_target` 后最终发布参考连续性，不能只检查原始五次曲线。由于初期后转早于此切换，本方案最多用于消除确认的断点，不能宣称已解决后倾起因。本轮仅建议，未实施、未构建、未仿真，到此停止。
