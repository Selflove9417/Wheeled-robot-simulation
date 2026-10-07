# 接地实际输入前向响应：当前模型失败

结论：**当前模型响应门失败，不推进运动速度、正常刹停资格或跳跃。** 本轮离线核对没有新的物理实跑；累计仍是 1 次地面实跑、0 次腾空跳跃、0 次跳跃达标。

输入来自已通过阶段一输入门的固定原生五秒站立窗 [9.553,14.553) s。取同物理步 BeforePhysics q/v、实际六路 JointForceCmd，预测 (post_v-before_v)/1 ms。模型中未注入实际加速度；JointTransmittedWrench 只用于单独诊断，不替代输入。保持原CAD、重力、阻尼、摩擦参数和原响应门。没有拟合参数或按误差筛掉样本。

原始全窗帧、双轮接触、六路输入、时钟/身份与九自由度速度连续性检查通过，连续性最大差0。接触转换为0。**5000个完整连续支撑步全数参与**，固定前半2500步诊断、后半2500步留出复核。两窗均8/9自由度失败；只有前向平移通过。

## 后半窗的原门结果

| 自由度 | 加速度误差RMS | 原绝对RMS限值 | 相对RMS | 门 |
| --- | ---: | ---: | ---: | --- |
| base_forward | 0.07504 | 0.5 | 0.07504 | PASS |
| base_z | 0.37903 | 0.5 | 0.37903 | FAIL |
| roll_x | 1.94464 | 2 | 1.94464 | FAIL |
| hip_left | 2.98162 | 5 | 2.98162 | FAIL |
| knee_left | 0.47790 | 5 | 0.47790 | FAIL |
| hip_right | 2.98153 | 5 | 2.98153 | FAIL |
| knee_right | 0.47790 | 5 | 0.47790 | FAIL |
| wheel_left | 1.19795 | 5 | 1.19795 | FAIL |
| wheel_right | 1.19807 | 5 | 1.19807 | FAIL |

各维必须同时满足绝对门和相对≤0.20；相对分母沿用 `max(observed_RMS,1)`。本次相对门失败，不能因为绝对门都通过就称模型合格。静止站立即使通过，也不构成三个构型、速度范围或独立运动标定资格。

## 已确认的物理差异与未确认项

1. CAD主要质量/惯量、模型世界Y/Z与RollX坐标、六执行器排列与原生记录一致。模型未计固定 IMU 0.01 kg（真实总质量17.51 vs模型17.5kg）；世界g=9.8而模型g=9.81。这些小差异保留，后续分别检查，未用作随意调参。
2. [现摩擦实现](../../../../src/bbot_balance_controller/include/bbot_balance_controller/thrust_support_dynamics.hpp) 对任意正/负腿速度施加±0.1 Nm。真实髋速度约±10⁻¹³ rad/s，几乎静止，却因数值符号触发动摩擦翻转。三个预先固定时刻9.600/12.100/14.500s均见预测髋±约2–3rad/s²，而实际接近0。
3. 本机DART `JointAspect.hpp` 与 `ConstraintSolver.hpp` 明确提供并自动创建 Coulomb friction constraint；这种离散约束允许零速静摩擦，与 `sign(极小速度)`的显式力语义不同。缺少 importer源码，尚不能逐行证明URDF值的导入路径；因此这是有来源的修模方向，尚非单一原因定论。
4. 只读逆动力学残差诊断，在两个静止四腿时刻可得到摩擦未知项约髋0.0364/0.0094 Nm、膝0.0789/0.0145 Nm，均在±0.1内，同时还留有约0.0066/0.0062的广义平衡残差。9.6s有缓慢膝运动，残差约0.0551。接触力与摩擦的反演并不唯一，已处理对称平面数值冗余；这些数值只是相容性证据，**使用观测加速度的反演不是前向响应通过**。
5. 现观测器没有记录 `JointVelocityCmd`。controller_manager 已确认腿Position inactive、Effort active，但本机只装 gz_ros2_control头文件/二进制，无法由源码证明物理层遗留速度组件或内部位置设定值已清除。下一轮新增只读三相位速度命令观测；存在组件本身也不能证明其实际参与执行。

## 下一次改动与关口

只在独立离线模型中实现1ms有界静/动摩擦预测：腿摩擦±0.1 Nm，原粘性阻尼、原CAD与原接触模型；粘住/正滑/负滑按预测下一步速度一致性判断，不读实际加速度，不放宽任何门或执行器限制。先在相同5000步全数对比，失败则继续残差复核；通过后也必须取得新会话独立实录及受限运动验证。

另在新的独立observer构建中记录每joint Pre/BeforePhysics/Post `JointVelocityCmd` presence、valid、size、value，缺失仍NaN，不能主动清除command component。冻结阶段一观测器程序保留。

禁止提高髋膝/轮上限、降低20%门、把逆向仍加速样本删掉、填入未经标定的统一减速度或调跳跃增益。支撑分配器仍关闭，README默认未晋级。

## 复现文件

[固定问题/条件](protocol.json) · [原响应门](response_gate.json) · [源码/模型/程序版本](model_manifest.json) · [实际审计命令](audit_command.json) · [审计源/原始数据版本](audit_manifest.json)

[全部响应门与残差](current_model/summary.json) · [逐步预测/观测/残差](current_model/response_steps.csv) · [完整窗口原始状态](current_model/window_frames.csv) · [原41列bridge输入](current_model/bridge_input.txt)

新增审计fixture 8项通过，包含q/v/实际输入列协议、前后窗隔离、缺失/NaN/时钟错配、接触转换和wrench缺失的真实无效保留。未以测试次数替代物理实跑。

后续独立物理记录也复现原模型失败，证明不是旧记录偶发。最新纯摩擦模型失败记录见 [friction_candidate/REVIEW.md](friction_candidate/REVIEW.md)；修正显式sign(v)后已从原8/9失败减少到两轮失败，但62帧仍为明确拒绝。新只读速度组件记录及官方物理分支核对见 [第二次接地报告](../../../../src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_velocity_component_probe_20261006_100700/REVIEW.md)。正在核对URDF固定IMU与SDF重力，仍不推进跳跃。
