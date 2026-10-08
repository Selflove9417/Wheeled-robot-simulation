# 修改前源码和单因素定义

实际入口 `controller_type:=jump_velocity`；源码 `src/bbot_balance_controller/src/bbot_velocity_jump_controller.cpp`，未使用reference controller。

- 3559附近记录 `takeoff_q_hip_left_/right_` 和膝位置为实测位置。
- 3597–3626：`arrest_duration=.060`；`vh_l/r=hip_vel_left_/right_`、`vk_l/r=knee_vel_left_/right_`。各自 `.init(now_sec,T,q0,v0,0,qf,vf,0)`。
- `end_v_hip(v)=clamp(v,-2,2)`；`end_v_knee(v)=clamp(v,-4,4)`；`end_q=clamp(q+.5*(v0+vf)*T,-1.56,1.56)`。
- 3964–3987 `sample_flight_joints` 分别evaluate四条轨迹。4563得到 `q_des/qdot_des/qddot_des`；轨迹解析加速度存在，非差分猜测。
- 4602–4619 ARREST反馈根据实际膝速混合增益；4644–4647髋姿态辅助；4660–4689使用四参考加速度的耦合动力学前馈；6667–6700离散关节PD与前馈相加。因此髋参考改动也可能经机械耦合及既有前馈影响膝实际响应/力矩，必须实测膝补偿。

五次轨迹详见 `flight_trajectory.hpp:45–92`。u=clamp((t-t0)/T,0,1)，q=sum(a_i*u^i)，v=sum(i*a_i*u^(i-1))/T，acc=sum(i*(i-1)*a_i*u^(i-2))/T²。a0=q0,a1=v0T,a2=acc0T²/2；Dz=qf-a0-a1-a2,Dv=vfT-a1-2a2,Da=accfT²-2a2；a3=10Dz-4Dv+Da/2,a4=-15Dz+7Dv-Da,a5=6Dz-3Dv+Da/2。

未触端位夹限、初末acc为0时，可化简为 `q=q0+v0*T*u+(vf-v0)*T*(u³-.5u⁴)`，`v=v0+(vf-v0)*(3u²-2u³)`，`acc=(vf-v0)*6u*(1-u)/T`。默认髋将约8rad/s降至2rad/s，参考acc在30ms达到负峰；原D1的18–54ms相当于ARREST开始后5–41ms，落在这一参考减速主段。但实际减速不能只归于参考，需包括反馈及前馈。

候选仅一行：`end_v_hip(v)=v-.25*(v-clamp(v,-2,2))`。初态相同时未夹限的髋参考速度变化及加速度为原25%，膝轨迹公式不变。以上轮D1 `q0=.598479,v0=7.82218` 为例，原vf=2,qf=.893144；候选vf=6.366635,qf=1.024143，ARREST中间髋端位前移.131rad，不能宣称此段端点固定。最终TUCK/EXTEND/landing构型目标生成逻辑不变，后续实测初态及实际状态切换可能改变。

保持60ms同终点且全过程减速更小在积分上不相容，因此没有采用后半段反而更强的固定端点修形。本次是保留残余髋速的机械干预，不是正式安全轨迹设计。正式控制器/默认可执行文件不变；独立副本、独立构建及launch仅用于六次实验。
