# THRUST末段水平接触力：控制可行性审查

日期：2026-10-08。仅源码与既有B1/B2/B3、原高跳T1数据分析；新增物理实跑0。未修改控制器、参数或默认入口；未提交、推送。155项冻结文件哈希一致。

## 决策

存在可控的轮端共同速度参考，但**不存在已经证实可独立调节水平接触力、同时保证竖直速度不损失的现有单参数**。最小值得验证的结构调整，是仅在现有THRUST释放段局部降低轮端前向速度参考，保持原腿部竖直推力路径和所有限制。此处只提出设计，不实施，也不指定未经识别的增益或幅度。

默认保持jump_height=0.25、takeoff_velocity=0.0、thrust_release_velocity_ratio=0.78、landing_capture_gain=1.60。高跳0.32仍只是失败对照，不成为默认。

## 实际执行链

以下行号对应本轮核对的实际源码。

- `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp:714–721,6551–6561`：轮端发布`geometry_msgs::msg::TwistStamped`到`/diff_drive_controller/cmd_vel`，使用`twist.linear.x`，本阶段`angular.z=0`。不是轮电机力矩命令。髋膝经`/leg_effort_controller/commands`发布四路力矩。
- `src/bbot_bringup/config/bbot_controllers.yaml`：实际轮控制为`diff_drive_controller/DiffDriveController`，轮半径0.07m、轮距0.364m，stamped velocity输入；腿为`JointGroupEffortController`。Gazebo硬件为实际URDF中的`gz_ros2_control/GazeboSimSystem`，轮velocity接口、腿effort接口。轮effort控制器的存在不意味着当前正在使用。
- 本机Gazebo插件接口和动态符号可确认`GazeboSimSystem::write`及`JointVelocityCmd`/`JointForceCmd`，但没有下层速度执行源码与可信轮电机实际力矩日志，不能虚构其力矩反馈公式。旧`wheel_joints.csv`中全零effort不能据此声称电机没有出力；旧`native_wrench.csv`没有轮velocity command字段。

THRUST轮端公式（cpp:3427–3459；`include/bbot_balance_controller/jump_phase_control.hpp:357–368,439–461`）：

```
v_p = thrust_forward_speed_predictor_.predict(...).predicted_speed
v_* = jump_takeoff_forward_speed_ = 0.45
k = thrust_forward_velocity_kp_ = 1.20
m = clamp(v_* + k*(v_* - v_p), 0, 0.85)
a_raw = 0.037*(current_gain_.k_theta*pitch_err
              + current_gain_.k_theta_dot*pitch_rate_err)
a = apply_thrust_forward_attitude_taper(a_raw, alpha, enabled)
c = clamp(thrust_kinematics_applied_correction_diag_, 0, 0.35)
u_target = clamp(-m + a + c, -1.5, 1.5)
u = u_previous + clamp(u_target-u_previous, -8*dt_eff, +16*dt_eff)
dt_eff = max(dt, 0.001)
```

这里16来自实际`thrust_wheel_max_decel_`，不是helper默认值。姿态taper只削弱负向姿态项：`s=clamp((v_p-(v_*-0.10))/0.08,0,1)`，`alpha=1-s²*(3-2s)`；正向制动项保留。观测无效时不套用上述有效补偿；原有效性处理不改变。

运动学补偿来自`thrust_wheel_kinematics_observer.hpp`的对齐odom/关节状态：`c_raw = d/dt[forward·R*(COM_body-axle_body)] + 0.07*(omega_shank×up)·forward`，再限幅0–0.35m/s。轮关节速度是相对小腿的速度，不是轮在世界系的绝对角速度；不能仅凭u正负判断地面力方向。

腿部作用（cpp:2972–3116,3176–3349,3370–3422）：

- 原COM竖直速度闭环、力预算及姿态保护生成竖直推力，膝逆速度参考考虑实际几何、机身和髋速度。
- `thrust_hip_force_share=0.0`（3256）：髋竖直力前馈为0，膝前馈为`Fz*Jk`；不代表髋对地面力无影响。
- `tau_body_per_hip = clamp(-0.5*(70*pitch_err+12*pitch_rate_err)+reaction_ff, -20,20)`；另有髋PD、膝PD、膝制动及快速速度补偿，经现有预算/限幅发布。姿态补偿确实进入髋执行器。
- THRUST调用`publish_effort_leg_control_lr`时通用PD系数传0，不能把上述专用PD再叠加一次。日志`actual_tau_*`是最终发布力矩名称，不是物理传感力矩。

轮速和腿力矩通过小腿旋转、轮轴平移、接触摩擦、姿态反馈及膝力臂耦合；没有水平/竖直接触力独立分配器。现有数据不能证明两条输出发生确定的力矩抵消，更不能把改变机身角速度等同于改变系统总角动量。

## 末段数据对控制假设的约束

沿用既有直接物理接触数据、COM与俯仰轴符号：前向+世界Y，pitch轴−世界X；前向地面力作用于COM下方产生负pitch力矩。下表为离地前最后20个1ms物理步，冲量来自完整步积分，保留交替正负力帧。

|试次|前向水平冲量 N·s|切向力pitch角冲量 kg·m²/s|有符号等效垂向力臂 m|目标/发布值不同帧数 /20|
|---|---:|---:|---:|---:|
|B1|−0.395|+0.162|−0.410|14|
|B2|−0.821|+0.345|−0.420|20|
|B3|+0.314|−0.135|−0.431|4|
|T1|+1.140|−0.510|−0.448|3|

力臂列是角冲量/水平冲量的有符号积分比，不是单一接触点高度。法向力水平力臂也随时间变化，见曲线；不能只改写为水平力一个因素。

T1最末端：13.765s现有释放记录生效后，至离地13.781s前15个分析帧中，轮目标已等于发布值（无该层slew截断）。13.765–13.780s，`v_p=0.550–0.557m/s`，`m=0.331–0.321m/s`，姿态项被taper为0，补偿封顶0.35，目标/发布值约+0.019～+0.029m/s；水平正冲量仍积累。

因此：

1. 增大轮速变化率不是T1释放后问题的直接证据支持方案；这不排除更早slew影响后续状态。
2. 关闭运动学补偿会令目标少约+0.35、转向更负的前向滚动要求；关闭负姿态项taper也会恢复更多前向要求，均缺乏正确方向依据。
3. `jump_takeoff_forward_speed_`还用于SQUAT等待（cpp:2781）及THRUST姿态taper（3445）；直接调全局值会改变起跳入口状态。提高`thrust_forward_velocity_kp_`在超速末段可减小m，但在早期欠速时反而增强前向驱动；也不是隔离的末段实验。
4. 轮相对速度在离地帧与发布值对应：T1 `0.07*dq=0.028568m/s`，发布0.028568；世界小腿角速度−7.194rad/s、轮绝对−6.786rad/s，符合相对关节角速度叠加。它证明该速度路径可到达运动，不识别电机实际力矩或水平力增益。

瞬时原生运动学表达式与在线补偿分别是物理步后瞬时值和带时间对齐/差分的观测值，不能把二者差值直接称为跟踪误差；平面滚动速度残差也不是实际接触滑移传感器。曲线的控制记录按最新已发布行保持，与after_step物理帧并列，不能视作同相位实际输入。

## 唯一建议的最小结构范围（尚未批准实施）

只局部修改cpp:3432–3436的共同前向速度参考生成。在既有`thrust_release_.active()`且仍处THRUST、未完全离地的轮输出路径内，使用局部参考`v_T=v_*−δ`；其余时刻完全沿用v_*。δ仅为方案符号，**不是现有参数名，也没有建议数值**。

```
原：m = clamp(v_* + k*(v_*-v_p),0,0.85)
新：m_T = clamp(v_T + k*(v_T-v_p),0,0.85)
```

原姿态项及其全局参考、补偿、释放阈值0.78、腿部推力公式、轮/关节限制和所有安全保护均保留。只在原已存在释放阶段改变轮速要求，不新增接触判定、时间阈值、控制器或输出路径。局部未饱和时`Δu_target=(1+k)δ=2.2δ`，不是力矩或接触力增益。

物理假设：在相同腿部运动、有效接触下，减少前向滚动要求可能减少前向地面力及负pitch角冲量；现有证据尚不能确认这个方向在闭环下成立。它比提前撤销膝竖直推力更直接作用于水平运动要求，且没有直接改动Fz请求；但**不保证竖直冲量不下降**。轮速变化可能改变摩擦、接触序列、膝几何力臂和姿态保护，轮内部反作用也可能先恶化机身角速度。

按T1约0.448m有效力臂，减少此前额外0.48–0.60角冲量需要约1.1–1.34N·s水平冲量改变。这只是离线量级预算，不能换算成安全δ。已有数据没有识别δ到接触力的灵敏度，也不足给出保住20cm的保证。若需要真正直接控制Fx/Fz，则必须超出本候选，增加有约束的接触力/力矩分配及执行映射；本轮不建议直接开展这种扩展。

## 获批后最小一次验证及停止条件

先冻结并审查一个局部δ及其命令边界，再仅做一次独立高跳T1匹配条件实验，0.32/0/0.78/1.60及其余设置保持，不扫描。使用现有记录：

- 核对释放前初态和输入未被干预；释放后发布轮速及实际相对dq确实变化，再判断接触力变化。若被限幅/延迟遮蔽或初态明显不匹配，结果记为干预不足/不可归因，不自动补跑。
- 首要判据为真实接触水平冲量、绕COM外部角冲量及离地**总**H共同改善；仅机身rate改善不足以认定成功。
- 同时核对竖直冲量/vz、同帧双轮净空20cm、真实首触姿态/角速度、后退及恢复。不能用条件模型或只看净空替代落地。
- 原有保护触发即按原机制退出；非轮碰地、二次腾空、保护展腿、EMERGENCY或失稳则停止，不追加。净空不足20cm或落地明显恶化也结束本次、不得采用或自动调参。即使一次改善也只是初步验证。

## 复现和保留

运行本目录`analyze.py`只读四次已保存物理试验，不启动ROS/Gazebo。输出与原始输入哈希是本地资料：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_horizontal_control_review_20261008/`，含四份`*_terminal_control.csv`、`summary.json`、`provenance.json`、`terminal_control_paths.png`。上游完整物理输入路径和版本沿用`thrust_momentum_budget_20261008`、`clearance_apex_ab_20261008`、`thrust_height_032_ab_20261008`记录。曲线未裁除失败/负力帧，未把模型曲线当实测。

本轮审查完成后停止，等待批准；不修改控制或默认配置。
