# 物理层速度命令核查 — 接地输入复验通过

本轮问题：已经验证的P轮与腿Effort站立中，是否保留物理层速度命令？仅更换只读记录器，控制器程序SHA256仍为 `1a7bd4a0ad50b27bb55fb5a706b4efd793ac1f4e72205a8611899d936e5fae0c`，分配器关闭、gain=1、正常RTF=1、步长1ms、原75/60/10Nm和30rad/s限制。

**1次接地实跑，0次腾空、0次跳跃达标。** 固定五秒窗 [8.876,13.876) s 内5000物理步、六路30000输入全部合法；双轮支撑、q/v与发布/响应配对、稳定站立通过完整原输入门。模式已通过实际controller_manager状态确认互斥。缺失输入保持无效；本报告不作刹停与跳跃合格结论。原运行结果仍为等待离线审计的初始FAIL，正式结论在 acceptance.json，完整分析在audit/summary.json。

六关节的Pre/BeforePhysics/Post各5000条速度组件都存在、维度1、值0；完整原样统计在 velocity_component_summary.json。Gazebo安装版本6.18.0的[官方Physics.cc](https://raw.githubusercontent.com/gazebosim/gz-sim/ignition-gazebo6_6.18.0/src/systems/physics/Physics.cc) 1814–1835行显示存在JointForceCmd时SetForce，速度命令仅在else分支处理。因本窗六路力组件完整，残留零值速度组件本身不会调用速度命令分支。这排除了该分支的同时输入；不能据此证明所有内部物理状态已核对通过。观测器未写入或清除任何组件。

发布/实际输入/关节合负载三类曲线：[signals.png](audit/signals.png)。合负载不是电机实际扭矩，缺失值不补零。新观测器2项ECS/消息测试、3项只读源码测试通过，控制器默认程序/MPC及上一轮观测器构建哈希不变。

重新运行须先复制protocol.json到全新输出目录：

```bash
experiments/jump/run_ground_input_trial.sh --output-dir NEW_DIR --observer-build-dir build_ground_input/bbot_bringup
```

版本绑定在tested_manifest.json、build_source_manifest.json、runtime_parameters.yaml和launch_command.json，所选程序/插件/源码已保存。下一步继续离线模型及残差原因审查；没有提高速度或试跳。

两份物理记录的离线复验：当前原CAD模型在本次独立窗口仍FAIL，两轮RMS前/后约1.006/1.339；纯摩擦候选同样FAIL，两轮约0.479/0.378且64帧预测拒绝，全部5000帧保留。见[原模型](original_model/summary.json)与[摩擦候选](friction_model/summary.json)。因此尚不能推进受限运动和刹停速度档。

原生九自由度消息独立复核：固定窗5000条消息全部q/v/before-v masks=511、六输入mask=63，44991组post→next-before分量最大差0，见[native9_crosscheck.json](native9_crosscheck.json)。第一个1ms自由落体物理步vz从0到-.0098m/s，诊断重力=-9.8m/s²，支持本地SDF默认值；该启动帧未纳入持续支撑响应样本。
