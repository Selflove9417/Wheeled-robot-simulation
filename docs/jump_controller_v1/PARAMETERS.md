# 参数、限制与源码索引

统计对象为当前 `bbot_velocity_jump_controller.cpp` 中76个直接 `declare_parameter<T>`。声明默认值不等于运行值：随后clamp、profile、launch、ROS参数注入以及分支都会影响最终结果。完整默认表达式、launch转交表达式与双侧行号见 [PARAMETERS.csv](inventory/PARAMETERS.csv)。外部共用姿态组件参数、URDF/YAML约束另列，不冒充都在这76项内。

## 默认与实验基线

| 参数 | C++声明 | 当前jump_velocity launch | 历史B1/B2/B3 |
|---|---:|---:|---:|
| jump_height | .20 | .20 | .25 |
| takeoff_velocity | 0 | 0 | 0 |
| thrust_release_velocity_ratio | .78 | profile .78 | .78 |
| landing_capture_gain | 1.60 | 没有转交，用节点默认 | 1.60 |
| thrust_peak_ratio | 2.30 | 2.0 | 以各次参数快照为准 |
| thrust_shape_early | .95 | .75 | 以各次参数快照为准 |
| sim_relax_thrust_limits | false | true | 以各次参数快照为准 |

`jump_profile.py`的`JUMP_VELOCITY_PROFILE`为当前默认路径；`OBSERVE_REPEATABILITY_PROFILE`保留旧对照默认（如release .70、kp .80及若干关闭的开关）。不能把launch表达式`jump_default(...,.70)`的legacy fallback当作jump_velocity实际.70。运行已安装jump_profile与源码之间还可能存在版本差异，复现实验必须使用来源快照、真实参数dump和二进制哈希，不能只看现在源码。

`landing_capture_gain`没有在主launch的velocity Node参数字典转交，同名launch附加参数不能证明节点实际变化；gain A/B使用独立显式注入路径。launch还有reference候选用的`thrust_reference_handoff/momentum_reference/support_coordination`转交键，当前默认主文件未声明，不可据此认定默认算法已接入这些候选功能。

## 全部主文件ROS参数

单位：高度/位置m、时间s、速度m/s或关节rad/s、加速度rad/s²、角度rad、力矩N·m；开关无量纲。复合反馈系数按README公式与原实现单位解释，不统一写成无量纲增益。

| 参数 | C++声明默认 | 功能 / 模块 | launch默认转交 | 源码行 |
|---|---|---|---|---:|
| `jump_forward_speed` | `0.35` | 准备/下蹲接近速度 | `"0.35"` | [L218](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L218) |
| `jump_takeoff_forward_speed` | `0.45` | THRUST离地前向速度参考 | `"0.45"` | [L220](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L220) |
| `jump_log_path` | `""` | 控制CSV输出路径 | `""` | [L223](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L223) |
| `jump_summary_path` | `""` | 摘要输出路径 | `""` | [L224](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L224) |
| `auto_return_balance` | `true` | 静稳后保持Effort返回BALANCE；true优先于Position交接 | `"true"` | [L225](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L225) |
| `thrust_forward_velocity_kp` | `1.20` | 推地轮控前向速度误差系数 | `jump_default("thrust_forward_velocity_kp", 0.80)` | [L226](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L226) |
| `thrust_forward_attitude_taper` | `true` | 临近卸力时衰减轮姿态项 | `jump_default("thrust_forward_attitude_taper", False)` | [L230](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L230) |
| `thrust_release_fast_rate_correction` | `true` | 推地快速角速补偿开关 | `jump_default("thrust_release_fast_rate_correction", False)` | [L232](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L232) |
| `thrust_fast_rate_correction_from_gate` | `true` | 补偿作用域从gate开始，受profile表达式控制 | `PythonExpression(launch_gate_scope_expression(             controller_type, LaunchConfiguration("thrust_release_fast_rate_correction")))` | [L234](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L234) |
| `thrust_release_velocity_ratio` | `0.78` | 速度比例触发单调卸力，非离地判据 | `jump_default("thrust_release_velocity_ratio", 0.70)` | [L236](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L236) |
| `thrust_forward_speed_prediction` | `true` | 前向速度短时预测 | `jump_default("thrust_forward_speed_prediction", False)` | [L241](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L241) |
| `thrust_wheel_kinematics_compensation` | `true` | 推地滚动几何速度修正 | `jump_default("thrust_wheel_kinematics_compensation", False)` | [L243](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L243) |
| `arrest_dynamics_feedforward` | `true` | ARREST浮基约化动力学前馈 | `jump_default("arrest_dynamics_feedforward", False)` | [L245](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L245) |
| `complete_contact_takeoff_confirmation` | `true` | 完整接触帧离地确认 | `jump_default("complete_contact_takeoff_confirmation", False)` | [L247](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L247) |
| `thrust_wheel_max_decel` | `16.0` | 推地轮参考减速限制 | `jump_default("thrust_wheel_max_decel", 8.0)` | [L249](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L249) |
| `jump_pitch_offset` | `0.020` | 起跳工作姿态相对balance_offset偏置 | `"0.020"` | [L255](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L255) |
| `jump_takeoff_pitch_rate` | `0.05` | 离地角速度目标 | `"0.05"` | [L257](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L257) |
| `jump_takeoff_pitch_rate_tolerance` | `0.18` | 推地准入角速误差门，Effort续跳另取更严界 | `未转交` | [L261](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L261) |
| `thrust_pitch_rate_lead_time` | `0.07` | 预测姿态门的角速前瞻时间 | `未转交` | [L265](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L265) |
| `landing_wheel_back_bias` | `0.120` | 落地目标轮轴后移几何偏置 | `未转交` | [L275](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L275) |
| `landing_capture_gain` | `1.60` | 空中轮轴捕获布置比例，非地面速度捕获律增益 | `未转交` | [L283](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L283) |
| `landing_target_x_max` | `0.10` | 落地IK水平目标正向上界 | `未转交` | [L287](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L287) |
| `landing_capture_height` | `0.40` | 预测落地布置的omega高度 | `未转交` | [L291](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L291) |
| `landing_capture_speed_deadband` | `0.08` | 预测水平速度死区 | `未转交` | [L295](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L295) |
| `landing_shank_abs_max` | `0.75` | wheel-first小腿方向限制 | `未转交` | [L303](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L303) |
| `landing_knee_axis_clearance_min` | `0.20` | 膝轴最小离地几何要求 | `未转交` | [L308](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L308) |
| `landing_deploy_ready_margin` | `0.055` | 展开就绪裕量 | `未转交` | [L313](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L313) |
| `landing_protective_deploy_min` | `0.10` | 保护展开最小预算 | `未转交` | [L317](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L317) |
| `flight_landing_pitch_bias` | `0.055` | 空中着陆姿态相对平衡点偏置 | `未转交` | [L324](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L324) |
| `flight_pitch_transition_duration` | `0.28` | 空中姿态参考平滑时长 | `未转交` | [L328](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L328) |
| `flight_landing_pitch_rate_max` | `0.30` | 着陆姿态参考角速上限 | `未转交` | [L334](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L334) |
| `pre_jump_timeout` | `bbot_jump::kDefaultPreJumpTimeout` | 准备超时，rolling_jump_control.hpp常量为4.0 s | `未转交` | [L339](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L339) |
| `pre_jump_stable_duration` | `0.08` | 准备稳定累计时间 | `未转交` | [L341](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L341) |
| `pre_jump_speed_tolerance` | `0.06` | 准备速度误差门 | `未转交` | [L343](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L343) |
| `pre_jump_pitch_tolerance` | `0.035` | 准备姿态误差门 | `未转交` | [L345](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L345) |
| `pre_jump_rate_limit` | `0.30` | 准备角速门 | `未转交` | [L347](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L347) |
| `thrust_leg_reaction_ff_gain` | `0.15` | 膝伸展速度反作用前馈系数 | `未转交` | [L378](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L378) |
| `thrust_leg_reaction_ff_max` | `2.0` | 膝反作用前馈限幅 | `未转交` | [L382](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L382) |
| `flight_hip_speed_limit` | `11.0` | 整段髋参考速度限制 | `未转交` | [L398](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L398) |
| `flight_knee_speed_limit` | `13.0` | 整段膝参考速度限制 | `未转交` | [L400](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L400) |
| `flight_hip_acc_limit` | `450.0` | 整段髋参考加速度限制 | `未转交` | [L402](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L402) |
| `flight_knee_acc_limit` | `500.0` | 整段膝参考加速度限制 | `未转交` | [L404](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L404) |
| `flight_hip_pos_limit` | `1.52` | 整段髋参考位置限制 | `未转交` | [L406](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L406) |
| `flight_knee_pos_limit` | `1.5708` | 整段膝参考位置限制 | `未转交` | [L408](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L408) |
| `landing_buffer_height` | `0.34` | 缓冲IK高度；运行时裁剪下限L_SQUAT+.01，故声明.34实际至少.35 | `"0.34"` | [L410](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L410) |
| `landing_joint_handoff_duration` | `0.16` | 触地实测关节构型到缓冲IK的最小平滑时长 | `未转交` | [L415](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L415) |
| `flight_tuck_nominal_duration` | `0.06` | 名义收腿时长；实际受往返规划/预算约束 | `"0.06"` | [L421](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L421) |
| `body_mass` | `9.5` | body质量；总质量=body+8 kg，更新运动学模型 | `"9.5"` | [L434](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L434) |
| `position_proportional_gain` | `0.3` | Position执行路径/切换相关增益；launch也传给xacro | `"0.3"` | [L442](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L442) |
| `enable_position_handoff` | `true` | 允许兼容Position交接；默认auto_return=true时不走该路径 | `"true"` | [L443](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L443) |
| `handoff_joint_error_limit` | `0.12` | Position交接关节误差门 | `未转交` | [L444](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L444) |
| `handoff_height_drop_limit` | `0.04` | Position交接高度跌落门 | `未转交` | [L445](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L445) |
| `handoff_pitch_error_limit` | `0.12` | Position交接姿态误差门 | `未转交` | [L446](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L446) |
| `handoff_z_dot_limit` | `0.20` | Position交接高度速度门 | `未转交` | [L447](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L447) |
| `failed_thrust_crouch_height` | `0.38` | 接地失败回低位支撑高度 | `未转交` | [L452](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L452) |
| `failed_thrust_crouch_duration` | `0.32` | 失败缩腿时长 | `未转交` | [L456](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L456) |
| `failed_thrust_settle_duration` | `0.18` | 失败低位稳定时长 | `未转交` | [L460](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L460) |
| `thrust_hard_block_timeout` | `0.12` | 推地硬阻塞退出时间 | `未转交` | [L465](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L465) |
| `jump_height` | `0.20` | 换算COM离地速度的弹道目标，非轮净空 | `"0.20"` | [L471](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L471) |
| `takeoff_velocity` | `0.0` | >0时覆盖sqrt(2g*jump_height)，0表示自动换算 | `"0.0"` | [L472](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L472) |
| `thrust_duration` | `0.24` | 名义推地轨迹运动时长，不是无条件接触持续时长 | `"0.24"` | [L473](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L473) |
| `thrust_peak_ratio` | `2.30` | shape额外推力规模 | `"2.0"` | [L474](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L474) |
| `grounded_launch_boost_ratio` | `0.75` | 名义stroke后仍接地且未达速度时的受限boost | `未转交` | [L477](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L477) |
| `thrust_shape_early` | `0.95` | 前半段推力Bezier形状 | `"0.75"` | [L479](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L479) |
| `thrust_shape_late` | `0.75` | 后半段推力Bezier形状 | `"0.75"` | [L480](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L480) |
| `thrust_velocity_kp` | `8.0` | 竖直COM速度反馈系数 | `"8.0"` | [L481](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L481) |
| `thrust_knee_velocity_limit` | `15.0` | COM速度反解膝参考限幅 | `未转交` | [L490](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L490) |
| `thrust_torque_margin` | `0.95` | 关节力矩预算保留比例 | `未转交` | [L491](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L491) |
| `sim_relax_thrust_limits` | `false` | 选择仿真THRUST限值路径；不等同取消所有安全保护 | `"true"` | [L493](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L493) |
| `thrust_timeout` | `0.60` | 推地未完成超时 | `"0.60"` | [L496](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L496) |
| `air_wheel_sign` | `-1.0` | 空中轮姿态控制方向 | `"-1.0"` | [L509](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L509) |
| `air_wheel_linear_speed_limit` | `2.0` | 空中轮线速度等效参考限幅 | `未转交` | [L514](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L514) |
| `air_wheel_extend_kd` | `2.50` | EXTEND轮姿态阻尼 | `"2.50"` | [L517](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L517) |
| `air_wheel_tuck_kd` | `0.45` | TUCK轮姿态阻尼 | `"0.45"` | [L520](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L520) |
| `flight_arrest_freewheel_duration` | `0.0` | ARREST可选短暂自由轮窗口 | `"0.0"` | [L523](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L523) |
| `thrust_terminal_hip_floor` | `false` | 末段髋力矩floor候选开关 | `"false"` | [L525](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp#L525) |

## 非ROS声明常量与接口限制

| 量 / 模块 | 当前值或规则 | 来源 |
|---|---|---|
| balance_offset / LQR | .030 rad；low=[−6.1624,−45.8436,−179.6985,−42.8109]；high=[−6.365,−49.5719,−233.4004,−62.6391] | cpp 198～209 |
| 站高范围 / SQUAT | L_MIN=.30、L_MAX/L_STAND=.50、L_SQUAT=.34、T_SQUAT=.50；正常升降.05 m/s | cpp 351～364；Effort续跳分支可延长 |
| 名义推地终点 / 旧常量 | H_TAKEOFF=.475；V_TAKEOFF=2.30、T_THRUST=.10不是当前v*与thrust_duration的唯一来源 | cpp 366～368；实际用声明参数/换算 |
| THRUST姿态 | Kp70、Kd12、每髋20 N·m限幅；髋竖直分担0 | cpp 372～374及3210～3274 |
| THRUST构型PD | hip18/2.5，knee22/3.0；后级预算/修正仍作用 | cpp 3137～3140 |
| 空中名义几何 / 时间 | L_RETRACT=.66、L_TOUCH=.69；EXTEND最小.090、保护部署.24、timeout常量.60、pitch guard常量.45 | cpp 396～430；不是所有分支无条件用同一个timeout或直接关机 |
| 缓冲支撑 | Kz450、Dz75、每腿Fmax240；THRUST1600 N/s，BUFFER3500 N/s建力率 | cpp 431～433、3275、5259 |
| 捕获目标 / slew | 公式系数−1.25/.25、h裁剪.15～.55；目标±5；普通捕获8 m/s²，临时反向分支12 | jump_phase_control.hpp 504；cpp TD/RECOVERY |
| 轮控制YAML | 半径.07、轮距.364、线速±5、角速±2、cmd timeout .5、发布50 Hz | bbot_bringup/config/bbot_controllers.yaml |
| 模型机械限制 | 腿±1.57 rad、effort150 N·m、velocity30 rad/s；轮URDF effort50、velocity30；另有接口限制 | bbot_description/urdf/bbot.urdf.xacro 2416起；不能等同实际速度伺服电机能力 |
| 当前Effort预算 | 由current_effort_limits按THRUST/ground及sim_relax分支选择，而非URDF150全程通用 | cpp 6607与jump_phase_control.hpp |
| Pose/关节配对 | 不外推，最大年龄80 ms，世界位置速度差分1～80 ms；观测器valid与采样stamp必须一起使用 | centroidal_state.hpp、world_pose_velocity.hpp、takeoff_detection.hpp |

共用姿态路径见`torso_pitch_control.hpp`及`sim_imu_reference_roll`的launch输入；IMU安装参考不能与原生世界pitch混写。配置来源优先级：实际试次参数dump/模型快照/二进制→该试次冻结源码/profile/launch→当前仓库；历史报告不能用现在默认值补齐缺失参数。

## 维护用源码地图

以下主文件统一指 [bbot_velocity_jump_controller.cpp](../../src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp)，详细算法见 [技术总结](README.md)。

| 职责 | 函数 / 行号 |
|---|---|
| 用户请求与门 | trigger_jump 1371；handle_mode_command1620；keyboard_loop1647 |
| 状态输入与调度 | imu_callback1728；joint_state_callback1766；control_loop1910 |
| 顶层阶段 | run_state_balance2097；pre_jump2389；squat2576；thrust2760；flight4046；touchdown_buffer5150；recovery5767；standup6179 |
| 着陆几何和轨迹 | inverse_kinematics_with_target_x3717；wheel-first3783；aligned_takeoff_geometry3806；preview3857；latch3872；plan_tuck_round_trip3909；plan_flight3988 |
| 捕获与重锁存 | landing_capture_target4901；update_capture_velocity_reference4954；ground_balance5002；post_brake_hold5035；落地锁存4860；回BALANCE锁存5963 |
| 失败与保护 | handoff fallback5744；abort_jump_to_recovery6210；protective deploy6283 |
| 支撑及执行 | interpolate_lqr_gain6337；vertical Jacobian6347；effort_height6361；effort_balance6436；wheel6551；position6565；effort limits6607；effort publish6614；Effort切换6935；Position切换6984 |
| 控制日志 | open_log7088；summary7175；log_data7234；CSV表头7140起 |
| 基础运动学 | bbot_kinematics/src/kinematics.cpp：inverse_kinematics65；Jacobian120起 |
| 轨迹/动力学/估计 | include/bbot_balance_controller/{flight_trajectory,flight_joint_pd,centroidal_state,world_pose_velocity,jump_phase_control,takeoff_detection,rolling_jump_control}.hpp |
| launch与安装 | bbot_bringup/launch/bbot_gazebo.launch.py velocity Node1120起；bbot_balance_controller/CMakeLists.txt的目标和install(PROGRAMS) |
| 绘图 | src/data_logs/plot_jump_log.py：Effort有效性83起、CLI340起；读取同次geometry与metrics实现真实位移 |

部分旧枚举注释仍写旧收腿高度或“顶点展腿”，本文采用实际赋值、条件与调用；维护时不能按注释直接重建控制器。
