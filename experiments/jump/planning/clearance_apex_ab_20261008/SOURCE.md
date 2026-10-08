# 修改前证据与唯一干预

默认源：`src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`，实验源码在本地 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008/source_baseline|source_test/`。默认源与程序未替换。

## TUCK参考

cpp396–424：L_RETRACT_=0.66m、L_TOUCH_=0.69m，flight_tuck_nominal_duration默认/实际0.06s，名义EXTEND下限0.090s。实际时长由round-trip可行性计算；髋/膝速度上限11/13rad/s、加速度上限450/500rad/s²、位置上限1.52/1.5708rad，均不变。

cpp3910–3942规划，与4110–4136生成均采用同一边界：

```
q0={hip_pos_left_, knee_pos_left_, hip_pos_right_, knee_pos_right_}
v0={hip_vel_left_, knee_vel_left_, hip_vel_right_, knee_vel_right_}
tuck_comp=clamp(pitch_-balance_offset_,-0.24,0.24)
tuck_ik=inverse_kinematics_with_target_x(L_RETRACT_,tuck_comp,0.0)
qf[i]=reaction_safe_configuration_step(q0[i],tuck_ik对应关节,
                                     hip为0.14/knee为0.04,v0[i])
normal_flight_joint_traj_[i].init(now_sec,flight_round_trip_plan_.tuck_duration,
                                 q0[i],v0[i],0.0,qf[i],0.0,0.0)
```

`flight_trajectory.hpp:20–36`：bounded=q0+clamp(request−q0,−maximum_delta,+maximum_delta)；若abs(v0)>0.50且(bounded−q0)*v0<0则返回q0，其他返回bounded。故名义L_RETRACT不保证关节真的到达该IK构型。旧有效试验已确认qf=q0，实际膝仍继续伸展，不能称为跟随有效收腿。

四关节走五次轨迹（flight_trajectory.hpp:45–92），初始q/v实测、初加速度0，终速/终加速度0。若qf=q0，u=clamp((t−t0)/T,0,1)，q=q0+v0*T*(u−6u³+8u⁴−3u⁵)，dq=v0*(1−18u²+32u³−15u⁴)。这是先随入速运动后返回原构型，不是明显弯腿收缩。高度tuck_traj_的0.66m参考不等于最终四关节到达0.66m。

## 时序证据与原因

前轮六次默认物理记录（本地 `thrust_early_repeat_20261008/physical/R1–R6`）按持续无接触首帧t=0：TUCK入口50–91ms，EXTEND入口111–155ms，七刚体COM顶点213–224ms，双轮同时净空峰230–244ms。EXTEND提前68–112ms；峰值净空13.53–17.76cm，历史15.34cm仅作旧对照，不冒充本轮baseline。

cpp4356–4372原式 `tuck_finished || landing_deadline_reached`。其中tuck_finished为(now−tuck_start)>=planned tuck duration；deadline为remaining_time<=planned extend duration+landing_deploy_ready_margin_+0.020。实际margin0.055s，round-trip初始extend0.090s。R1旧日志tuck_done=1/deadline=0，直接展腿是为了给低跳wheel-first部署预留时间，并非顶点检测触发。真实EXTEND规划从当时实测q/v初始化，搜索剩余时间内最长可行段（5ms网格，至少0.09s）；不从静止开始。

## 唯一改动

独立test源cpp4372：

```
// baseline
else if (tuck_finished || landing_deadline_reached)
// test
else if ((tuck_finished && centroidal_velocity_valid_ &&
          centroidal_height_.velocity() <= 0.0) || landing_deadline_reached)
```

只推迟普通TUCK→EXTEND触发，试验不改善收腿目标本身。保留deadline、姿态超限保护、原可行性检查、限制和全部控制参数。COM速度为有效高度差分样本，非原生COM真值；<=0单样本条件可能在真实顶点之后触发或被deadline抢先，本轮统计实际行为，不增加滤波/死区/新门。无效时仍由原deadline处理。

THRUST、轮控、落地捕获代码和参数均不改，gain1.60。相位持续时间改变会间接改变既有TUCK/EXTEND轮D分支作用时间、落脚计划锁存时刻、剩余时间规划及可行轨迹；不能声称所有其他实际输出保持相同。保护路径仍能提前部署。每组3次固定顺序交错，源码/程序/参数冻结，完成后不扫描或追加。
