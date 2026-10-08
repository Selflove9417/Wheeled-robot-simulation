# 双轮净空：延后普通EXTEND的单因素A/B

2026-10-08完成。默认控制及landing_capture_gain=1.60始终保留；独立候选仅改普通TUCK→EXTEND触发条件，源码差异一处，独立构建/launch。THRUST、反馈滤波、轮控制器、落地捕获算法及所有参数/安全限制均未改。固定B1/T1/B2/T2/B3/T3，6实跑/6腾空/6真实首触/6次最终BALANCE，正式性能达标不据此认定。每次真实首触后固定观察20s，已停止，无提交/推送/追加调参。

## 独立验收结论：不采用

**延迟展腿有实际作用，但净空收益不一致且落地后退恶化，未满足“提高净空且不明显恶化稳定性”。** 三对净空变化−0.63、+1.08、+1.83cm，平均仅+0.76cm（约4.9%），组间范围重叠。T2最大后退68.78cm，高于三个baseline的42.76–47.67cm；T2/T3首触负角速度明显较配对baseline增大。3/group为探索性实跑，不能宣称统计显著或排除入空差异。

|指标（组均值）|baseline|test|
|---|---:|---:|
|双轮同时最大真实轮底净空|15.552cm|16.312cm|
|COM离地后最大上升|23.779cm|23.895cm|
|首触pitch|-4.823°|-5.603°|
|首触pitch_rate|-0.700rad/s|-1.094rad/s|
|首触后最大后倾|4.992°|6.076°|
|首触点锚定最大后退|44.451cm|52.683cm|
|首触后20s最终轮轴位移|-4.280cm|-10.250cm|
|完整安静条件起点|3.430s|3.495s|

## 每次真实物理结果

|试次|COM离地后上升cm|双轮净空cm|首触pitch °|首触rate rad/s|落地最大后倾°|最大后退cm|20s最终位移cm|安静恢复s|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|B1|23.51|15.03|-3.94|-1.183|4.38|42.76|-2.70|3.447|
|T1|22.39|14.40|-6.29|-1.196|6.73|39.97|-1.77|3.344|
|B2|24.40|16.56|-5.03|-0.539|5.08|47.67|-6.60|3.435|
|T2|24.87|17.65|-6.04|-1.074|6.54|68.78|-21.66|3.626|
|B3|23.43|15.06|-5.50|-0.379|5.52|42.92|-3.54|3.407|
|T3|24.42|16.89|-4.48|-1.013|4.95|49.30|-7.32|3.516|

六次均无EMERGENCY、无非轮部件碰地、无符合既有≥10ms且净空>2mm判据的二次腾空。全部接触中断区间保留metrics.json，没有把短暂零接触帧删除。恢复到BALANCE约首触后4.58–4.89s；较早安静起点需连续满足1s，不等同恢复到起跳前位置，T2结束时仍后退21.66cm。没有用“最终没倒”抵消后退退化。

## 源码与改动前证据

[源码公式与唯一改动](SOURCE.md) · [冻结实验协议](PROTOCOL.md) · [源差异](test_source.patch)。默认cpp4354–4372原来TUCK结束或着陆deadline到达就立即EXTEND。名义TUCK0.060s、EXTEND最小0.090s、landing_deploy_ready_margin=0.055s；实际展腿时长由剩余弹道预算内最长可行搜索决定，未修改。旧六次EXTEND比COM顶点提前68–112ms。

TUCK用当前实测q0/v0初始化五次，终速/终加速度0。L_RETRACT_=0.66m，但reaction_safe_configuration_step的反向运动保护常返回当前q0，故“高度参考0.66”不代表四关节已经收至该构型。新六次24条髋膝参考由发布数据反解qf−q0最大约3.3e−6rad、重建误差最大9.0e−6rad，符合CSV舍入，均等效qf=q0。本轮没有绕过或放宽这一保护。

|试次|离地原生sim s|TUCK入口/EXTEND入口ms|COM顶点/双轮净空峰ms|TUCK计划/实际持续ms|
|---|---:|---|---|---|
|B1|13.807|50/110|219/238|60/60|
|T1|10.888|48/233|214/228|60/185|
|B2|11.046|53/113|223/243|60/60|
|T2|15.105|53/258|225/234|60/205|
|B3|11.367|50/110|219/239|60/60|
|T3|15.346|52/237|223/232|60/185|

所有时间统一以持续无接触首帧t=0。COM顶点由原生七刚体瞬时质心计算，非机身/FK最高点。原条件没有等顶点，是为低跳预留wheel-first部署时间；本候选只将常规tuck_finished分支同时要求有效COM差分速度≤0，deadline和保护仍可抢先。T1真实COM顶点214ms而EXTEND233ms，实际触发日志deadline=1；T2顶点225ms而EXTEND258ms，deadline=0。控制器COM差分速度和真实物理COM并不同步，不把候选叫作精确顶点检测。

下面列左关节TUCK初态及EXTEND入口实际位置；右侧四项全量见tuck_reference_audit.csv。qf等效q0、终速0，计划时长60ms；入速并非0。

|试次|髋q0/v0 rad,rad/s|膝q0/v0|EXTEND时实际髋/膝q|
|---|---|---|---|
|B1|0.821730/2.04687|-1.229680/-6.13612|0.855532/-1.413950|
|T1|0.805679/2.42004|-1.157370/-6.21840|0.914781/-1.270940|
|B2|0.895273/1.48507|-1.329600/-5.48369|0.911383/-1.475560|
|T2|0.899540/1.35794|-1.415650/-5.48449|1.015370/-1.472010|
|B3|0.803964/2.26431|-1.224550/-6.30217|0.849548/-1.407070|
|T3|0.874478/1.40014|-1.323620/-5.45809|0.969001/-1.398650|

候选保持TUCK更久，但并没有创造新的收腿目标；参考终点仍是进入TUCK时的位置。实际腿运动及跟踪误差在各次flight.csv和flight.png保留。延后展腿同时使剩余可行时长缩短、落脚捕获锁存较晚，并延长现有TUCK轮控分支作用时间；这些是单一状态时机变更的下游效应，未修改任何轮控或捕获公式/增益。不把净空变化全部归因于某个独立关节，也不重新扩大THRUST异常分析。

## 指标定义与有效性

- 净空使用实际URDF中的左右wheel cylinder（半径0.07m、长度0.04m、各自碰撞origin）及原生link位姿计算：center=p+R*collision_origin，axis=R*collision_axis，底部z=center_z−r*sqrt(1−axis_z²)−half_width*abs(axis_z)，减实际水平地面高度。world中ground_plane无额外pose且normal=(0,0,1)，地面z=0。主指标是max_t min(left,right)，不把两个不同时刻单轮峰值拼成双轮值。左右独立峰另存。比旧“轮link原点z−r”更完整，未改验收门。
- COM含实际七刚体mass、inertial origin与原生link旋转，不用机身z代替。主表上升量从真实离地COM起算；另存起跳指令前站立COM基准上升量，baseline均值约28.89cm、test约29.35cm，不能与轮底净空混称。
- 首触来自原生真实碰撞帧；pitch用同帧原生姿态，rate用同物理步原生角速度。轮轴后退使用左右实际轮轴原点均值，在冻结jump_forward_axis上投影，真实首触点为零，负向为后退；窗口首触后20s。最终位移是该窗口末值。
- 完整安静恢复沿用已有诊断条件：abs(controller pitch−.03)<.04、abs(filtered pitch_rate)<.15、abs(capture_com_velocity)<.08、abs(world z_dot)<.03连续1s；另存原生姿态恢复和BALANCE入口。仅用于比较，不新增控制/安全门。
- 发布effort、BeforePhysics实际输入、AfterPhysics关节合负载分别作图；轮速度伺服实际力矩仍unknown，未补零。所有失败/扰动保留；六次都没有保护退出，未用无效跳跃指标充数。

## 原始数据、曲线与复现

本地资料根：`src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/`。physical/B1,T1,B2,T2,B3,T3保存完整velocity/summary、engine/inertials、ground/contact_detail、geometry、native_wrench、wheel_joints、actual URDF/SDF、参数、launch及事件。source_baseline/source_test、独立launch、campaign.py、构建日志与build_clearance_apex_baseline/test保留；默认完整跳跃未覆盖。

analysis/每次含metrics.json、flight.csv、flight_inputs.csv、post_contact.csv、flight.png与torques.png；comparison_flight.png、comparison_landing.png、tuck_reference_audit.csv、group_summary.json包含关键对齐与全精度结果。曲线均为真实Gazebo记录或由其计算，无模型扭矩曲线冒充实测。

工作区运行[分析](analyze.py)并传试次名称，然后[汇总](summarize.py)、[报告](report.py)，均只读分析、不执行控制输出。version_check.json核验155项冻结文件前后及当前相同；源码树差异仅一cpp的一条布尔表达式，raw_sha256.json保存全量原始文件校验。源码检查、独立构建和6次物理结果分列，不以构建通过当性能通过。

**本轮候选不采用；默认原实现、gain1.60保持。分析和A/B均完成，停止，不继续下一项修改。**
