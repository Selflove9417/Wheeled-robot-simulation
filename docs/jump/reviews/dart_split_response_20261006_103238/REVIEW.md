# DART 分步响应核对：已有站立数据离线通过

待验证问题与改动见 [预写协议](protocol.json)：只把自由动力学与接触/静摩擦冲量分开处理；未改接触几何、限制、响应门、控制器或实验参数。

root 独立编译三配置及单元检查。每份实际记录固定五秒、5000步，前后各2500步单独同时核对原九维绝对误差及20%相对RMS门。六项全部PASS，每项完整5000输出、0预测拒绝；无删帧、无增益拟合。

|记录|配置|前窗轮相对RMS|后窗轮相对RMS|全部九维|
|---|---|---|---|---|
|1|原CAD|0.023163422|0.019927542|[PASS](record1_profile0/summary.json)|
|1|仅重力9.8|0.019586027|0.019706701|[PASS](record1_profile1/summary.json)|
|1|重力9.8＋固定IMU|9.756368e-06|4.0321074e-05|[PASS](record1_profile2/summary.json)|
|2|原CAD|0.025069184|0.019962863|[PASS](record2_profile0/summary.json)|
|2|仅重力9.8|0.019462647|0.019548427|[PASS](record2_profile1/summary.json)|
|2|重力9.8＋固定IMU|1.1209227e-06|4.7617843e-05|[PASS](record2_profile2/summary.json)|

预测只读取同物理步前 q/v 和六路实际 JointForceCmd，模型参数来自固定源码；观测加速度及关节合负载仅供比较。首先解 (M+dtD)a_free=τ−C−G−Dv，再以普通M处理接触/关节摩擦冲量；阻尼取v_free，不重新取接触约束后的v_next。接触仍为原连续滚动约束，本轮未同时更换该模型。

root 首次编译单元测试遇到旧边界 fixture 断言FAIL，日志保留于 root_initial_unit.log。新的物理顺序不再产生旧近零滑动分支；只将该fixture断言改为严格方向拒绝及唯一物理解，未放宽物理门。最终独立编译 [单元日志](root_unit.log) PASS，所有冻结源/程序前后 [版本核对](execution_version_check.json) PASS。

**验收范围：**这里只证明两份已记录的静止站立数据能够通过固定模型响应门。ECM基座速度的新鲜度尚未获得直接物理引擎证据；下蹲、伸展、非零腿速和停止距离包络均未验证，不能宣告阶段二通过或推进跳跃。下一轮先加入独立Physics只读before/post实际状态，再重新核对九维响应。

本轮新增实跑0、腾空0、达标0。累计本次执行2次接地实跑、0次腾空、0次达标；默认与MPC未修改/晋级。

[第二份记录全部5000步九维残差图](record2_profile2/residuals.png)。

实跑输入、三类扭矩曲线和原始数据：[记录1](../../../../src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_input_stage1_20261006_091717/REVIEW.md)、[记录2](../../../../src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_velocity_component_probe_20261006_100700/REVIEW.md)。
