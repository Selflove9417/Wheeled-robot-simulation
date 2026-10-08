# 原始默认控制：离地后角动量交换定量核对

2026-10-07。默认控制保持原实现，gain=1.60；未修改THRUST、ARREST、TUCK/EXTEND、轮限速或其他控制参数。本轮仅增加只读诊断，补充**1次实跑、1次腾空、1次落地**，首触后观察20秒，最终回到BALANCE。未调参，未追加A/B，未替换默认Physics插件或默认控制器。

## 最终回答

**可以。在这次默认控制跳跃中，离地后机身负向pitch_rate的形成能够由腿与轮的角动量交换定量解释。** 0～80ms总pitch轴角动量最大漂移0.01230kg·m²/s，约初始值的1.22%；将总角动量固定为离地初值，以实测关节速度、构型和实际惯量重建机身角速度，RMS误差0.00560rad/s、最大0.00743rad/s，远小于快速后转段2.28090rad/s的变化。

主要后转项来自髋运动变化，膝运动变化抵消其一部分；轮轴自身旋转在此窗口也是抵消项。**轮刚体被腿带动的轨道角动量，与轮轴自身旋转必须分开，不能把轮刚体的全部角动量变化都归于wheel command。**

这是一轮实际物理状态的角动量分账，不是新的控制干预因果A/B，也不证明某个PD/前馈项是唯一根因。未分解的1.22%总量漂移保留，不声称严格守恒；仍足以解释本次后转的主体。单次诊断不外推为全部工况独立复验通过。

## 实际模型与数据来源

首先读取当前实际xacro及launch，默认body_mass=9.5、payload_mass=0.0、position_proportional_gain=0.3。物理运行前从 `/robot_state_publisher` 实际参数读取robot_description，保存 `actual_robot.urdf`，再由同机 `ign sdf -p`保存实际URDF对应的 `actual_robot.sdf`。同时直接从运行中Gazebo ECM的 `components::Inertial`读取每个实际Link的mass、完整惯量张量和inertial pose，保存 `engine_frames.csv.inertials.csv`；逐项与实际转换SDF比较到1e-12。

运行中物理Link清单严格等于SDF七个Link，没有遗漏payload或辅助刚体：固定imu_link的0.01kg及惯量已合并进base_link，故body质量为9.51kg，总质量17.51kg。不是手工增加IMU或重复计数。

|分组|实际Link|mass kg|inertial origin xyz（Link坐标）|
|---|---|---:|---|
|body（含合并IMU）|base_link|9.51|0.200018441, 0.132612807, 0.053966805|
|left thigh|link_002|1.20|0.061073570, -0.136906990, -0.021166970|
|left shank|link_003|0.80|-0.011043980, 0.115382050, -0.085322880|
|left wheel|link_004|2.00|0.038250010, 0.000000000, 0.000000000|
|right thigh|link_005|1.20|-0.060773570, -0.136906990, -0.021166970|
|right shank|link_006|0.80|0.006043990, 0.115382050, -0.085322880|
|right wheel|link_007|2.00|0.019250000, 0.000000000, 0.000000000|

映射来自模型关节树和控制器左右关节索引；运行时scoped name为 `flat_jump_world::bbot::{Link}`。所有运行时inertial quaternion均(1,0,0,0)。下表张量按惯性坐标轴给出，计算中使用完整精度，不丢交叉项。

|Link|Ixx / Iyy / Izz kg·m²|Ixy / Ixz / Iyz kg·m²|
|---|---|---|
|base_link|0.1079026786 / 0.1827185714 / 0.1587711071|5.632142621e-05 / 6.989286327e-05 / -0.006339214281|
|link_002|0.017921 / 0.003296 / 0.016016|0 / 0 / 0|
|link_003|0.01313 / 0.004587 / 0.008687|-4e-06 / 3e-06 / 0.001951|
|link_004|0.006481 / 0.004557 / 0.004535|0 / 0 / 0|
|link_005|0.017921 / 0.003296 / 0.016016|0 / 0 / 0|
|link_006|0.01313 / 0.004587 / 0.008687|4e-06 / -3e-06 / 0.001951|
|link_007|0.006481 / 0.004557 / 0.004535|0 / 0 / 0|

位置、四元数、线速度、角速度全部来自私有只读Physics诊断的 `FrameDataRelativeToWorld()`，绕过可能保持旧值的ECM WorldPose；六关节q/v来自同一物理引擎Joint getter。只分析 `after_step`，physical_state_time_ns=sim_time_ns，每步1ms。惯量来自运行中模型，运动状态来自实际物理引擎，均非运动学模型预测或控制器参考。

现有日志初算可接近守恒，但未保存运行时合并惯量、七Link直接引擎状态的完整链路，因此只补充1次原始默认控制诊断。Physics诊断副本相对已有只读Physics仅增加七Link列表和实际惯量getter日志；补丁：[diagnostic_only.patch](diagnostic_only.patch)。唯一Physics插件在独立world内按绝对路径选取，未与原Physics并行加载，不写力、速度、位姿或控制参数。

## 坐标系和计算公式

采用统一世界坐标分量，原点平移至系统瞬时质心，速度减去系统质心速度，坐标轴不旋转。定义pitch投影方向e=(-1,0,0)。源码 `bbot_velocity_jump_controller.cpp:1735–1736` 使用pitch=-roll、原始rate=-IMU angular_velocity.x；本平地近似平面运动中机身横轴与世界X最大偏离约1.79e-5，采用固定世界轴避免旋转投影造成虚假不守恒。计算始终保留三维向量及完整张量，再取pitch投影。

对每Link i，R_i为世界姿态，c_i为惯性原点相对Link原点的偏移，Q_i为惯性轴相对Link轴的旋转：

```text
p_ci = p_link_i + R_i*c_i
v_ci = v_link_i + omega_i × (R_i*c_i)
I_Wi = R_i*Q_i*I_inertial_i*Q_i^T*R_i^T
C = Σ(m_i*p_ci)/M
V_C = Σ(m_i*v_ci)/M
r_i = p_ci-C
H_spin_i = I_Wi*omega_i
H_orb_i = r_i × m_i*(v_ci-V_C)
H_i = H_spin_i+H_orb_i
H_total = Σ H_i
H_pitch_i = e·H_i
```

因此不存在把Link原点速度当质心速度、漏掉轨道项、直接用局部惯量乘世界角速度、忽略IMU合并质量等问题。按各组使用同一平移COM参考系分账，不能与另一组采用绝对世界线速度的部分角动量混算。

## 0～80ms真实物理窗口与守恒

真实最后轮地接触sim=16.746s；首个持续无接触帧t0=16.747s。全部81帧接触为0，时间、状态有效；ARREST入口为t0后13ms。前18ms角速度曾回正，随后18～54ms快速转负，不能把单帧负值当完整持续形成。

- 七物理Link完整，运行时惯量与实际SDF一致。
- 同一步直接物理引擎位姿/速度/关节速度，不混用before/after相位。
- 关节树原点位姿关系最大残差8.67e-16m；关节速度重建各Link速度最大残差8.88e-15。
- 完整三维关系 `H_total=I_locked*omega_body+h_relative` 最大残差1.10e-14kg·m²/s。
- 总pitch角动量t0为−1.010236、80ms为−0.998173kg·m²/s；最大漂移0.012300（1.2176%），三维漂移范数最大同为约0.012300。

这里的非零漂移尚未进一步归因为积分误差、约束求解或执行器实现，不假定其来源。清单、坐标、接触和相位已核查；其等效角速度影响远小于主要后转，故可以称本窗口近似守恒，不能称严格闭合。

|t ms|total H_pitch|body H_pitch|legs H_pitch|wheels H_pitch|body spin H_pitch|body orbital H_pitch|原生pitch_rate rad/s|
|---:|---:|---:|---:|---:|---:|---:|---:|
|0|-1.010236|-0.747596|-0.636740|0.374100|-0.011616|-0.735980|-0.107649|
|18|-1.007690|-0.601995|-0.568148|0.162453|0.066555|-0.668550|0.616803|
|54|-0.998074|-0.576401|-0.213672|-0.208002|-0.179560|-0.396840|-1.664096|
|80|-0.998173|-0.461255|-0.180610|-0.356308|-0.112411|-0.348845|-1.041777|

H单位均为kg·m²/s。body关于系统COM的总H在0～80ms增加+0.286341，但其自转H减少−0.100795；轨道H增加+0.387135。**机身后转与body总H增加并不矛盾，后转直接对应自转/角速度，不能忽略轨道项后强求body与legs总H一对一等量反向。** 三组完整变化为body +0.286341、legs +0.456130、wheels −0.730408，合计+0.012063，与总量末初变化一致。

### 七个刚体分别分账

|Link|H_spin t0→80ms|H_orb t0→80ms|H_total t0→80ms|
|---|---|---|---|
|base_link|-0.011616→-0.112411|-0.735980→-0.348845|-0.747596→-0.461255|
|link_002|-0.147045→-0.029110|0.013773→-0.000024|-0.133272→-0.029134|
|link_003|0.076875→0.012470|-0.261972→-0.073641|-0.185098→-0.061171|
|link_004|0.021829→-0.080012|0.165222→-0.098145|0.187050→-0.178157|
|link_005|-0.147045→-0.029106|0.013773→-0.000026|-0.133272→-0.029132|
|link_006|0.076875→0.012472|-0.261972→-0.073645|-0.185098→-0.061173|
|link_007|0.021829→-0.080010|0.165221→-0.098140|0.187050→-0.178151|

## 能否用腿与轮的运动重建机身后转

仅看各刚体总H的增减仍可能误判。额外使用原始URDF关节树、世界姿态及六关节实测qdot建立相对机身的运动速度；不使用当前机身角速度反算相对运动项。

```text
omega_rel_child = omega_rel_parent + axis_world*qdot
u_child = u_parent + omega_rel_parent × (p_child-p_parent)
v_rel_COM_i = u_i + omega_rel_i × (R_i*c_i)
h_relative = Σ[I_Wi*omega_rel_i + r_i × m_i*v_rel_COM_i]
I_locked = Σ[I_Wi + m_i*((r_i·r_i)*Identity-r_i*r_i^T)]
omega_body_pred(t) = inverse(I_locked(t))*(H_total(t0)-h_relative(t))
```

模型六个关节axis均为其关节坐标中的(1,0,0)，用实际姿态变换；关节原点与child Link原点关系从实际URDF读取并经物理姿态核对。对六个qdot逐项线性分解，可区分腿关节携带整个轮刚体的运动与轮轴自身旋转。

0～80ms重建误差RMS=0.005605rad/s、最大=0.007430rad/s；80ms实测−1.041777，固定初始总H预测−1.048906rad/s。此预测只用初始总H和之后的关节运动/构型，并非逐帧直接代入实测当前总H得到的恒等式。

快速形成段18～54ms：实测pitch_rate从+0.616803降至−1.664096，变化−2.280899rad/s。左/右髋实际速度约+7.696→+0.381rad/s，左/右膝约−11.799→−3.716rad/s；两轮轴实际速度+3.836→+11.010rad/s。

将末端惯量矩阵固定用于各相对运动差项，构型惯量变化单列，得到可加的精确分账：

|项|对18～54ms pitch_rate变化的分账 rad/s|
|---|---:|
|左右髋关节运动变化|-4.678974|
|左右膝关节运动变化|+2.418016|
|腿关节净项（髋+膝）|-2.260958|
|两轮轴自身旋转变化|+0.056432|
|构型/locked inertia变化|-0.082208|
|实测总H非守恒残差|+0.005835|
|净合计（腿净项+轮+构型+残差）|-2.280899|

轮轴自身旋转抵消约2.50%的腿关节净后转项；不是后转主导项。按刚体分组时“轮刚体相对角动量”还包括腿带动轮质心运动，因而该组可表现为较大的后转贡献，不能据此指控wheel controller向机身注入了全部后转。这里是当前构型和运动下的角动量机械分解，不能当作单独关闭髋/膝/轮控制后的独立因果试验。

## 记录、复现与边界

本地资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007/`。

- `physical/D1/actual_robot.urdf`、`actual_robot.sdf`及robot_state_publisher参数dump：实际模型；`engine_frames.csv.inertials.csv`：运行时惯量完整清单。
- `physical/D1/engine_frames.csv`：七Link与六关节的before_step/after_step实际引擎位姿、线/角速度与有效位；`ground_frames.csv`：真实碰撞；控制CSV、geometry、native_wrench、轮joint-state、参数、launch、事件和运行结果均保留。
- `engine_window.csv`保留对应原始物理窗口，`momentum_window.csv`含81步的各组H、pitch/rate、六关节实际速度、轮命令、状态与接触；`link_momentum_window.csv`保存每Link自转/轨道/合计H及COM状态；`model_inventory.json`保存完整惯量。
- `summary.json`保留三维守恒、运动学残差、固定初始H预测误差及两个窗口的分账；`momentum.png`、`momentum_overview.png`为实测状态计算结果，不是仿真外的模型响应预测。角速度重建曲线明确标为prediction。
- `raw_sha256.json`、前后冻结清单、`version_check.json`绑定模型、程序、参数和数据；原54个正式源码/程序/模型文件完全不变。诊断构建目录本地 `build_angular_momentum_physics/`，完整诊断源码副本与构建日志留在原始资料根。

[分析程序](analyze.py)，工作区根运行 `python3 experiments/jump/planning/angular_momentum_20261007/analyze.py`。依赖numpy/scipy/matplotlib，只读原始资料、写派生结果，无ROS控制或仿真动作。[单次诊断协议](PROTOCOL.md)。

1ms物理步、配置real_time_factor=1.0，未改变仿真倍率、动作速度或限制。诊断只读取状态与惯量，正式默认控制保持原样，运行已停止。未进一步调参，也未设计正式修复。
