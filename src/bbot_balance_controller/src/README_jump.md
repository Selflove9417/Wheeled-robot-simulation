# Wheeled Robot Jump Control

本文整理当前仿真项目中的跳跃控制流程，按状态机分为 5 个主要阶段：

```text
BALANCE
   ↓
SQUAT
   ↓
THRUST
   ↓
FLIGHT
   ↓
TOUCHDOWN_BUFFER
   ↓
RECOVERY
```

---

## 1. SQUAT：下蹲蓄力

机器人先从正常站立高度平滑下蹲到：

$$
L_{\text{squat}} = 0.30\text{ m}
$$

下蹲时间：

$$
T_{\text{squat}} = 0.45\text{ s}
$$

使用五次多项式生成高度轨迹：

$$
z_d(t),\quad \dot z_d(t),\quad \ddot z_d(t)
$$

边界条件：

$$
z(0) = z_0,\quad \dot z(0) = 0,\quad \ddot z(0) = 0
$$

$$
z(T) = L_{\text{squat}},\quad \dot z(T) = 0,\quad \ddot z(T) = 0
$$

期望高度通过逆运动学转换为髋、膝目标角：

$$
z_d \rightarrow \text{IK} \rightarrow (q_{h,d}, q_{k,d})
$$

下蹲过程中轮子仍使用 LQR 保持机身平衡。

---

## 2. THRUST：爆发推地

腿从下蹲高度快速伸展到：

$$
H_{\text{takeoff}} = 0.475\text{ m}
$$

规划时间：

$$
T_{\text{thrust}} = 0.160\text{ s}
$$

并要求轨迹末端速度：

$$
V_{\text{takeoff}} = 2.30\text{ m/s}
$$

五次轨迹给出期望加速度 $\ddot z_d$，再根据竖直方向动力学计算每条腿的推地力：

$$
\boxed{F_z = \frac{M}{2}(g + \ddot z_d)}
$$

然后通过腿部 Jacobian 转换为髋、膝关节力矩：

$$
\boxed{\tau = J_z^T F_z}
$$

即：

$$
\tau_h = J_{z,h} F_z,\qquad \tau_k = J_{z,k} F_z
$$

髋关节还叠加机身俯仰姿态补偿，膝关节使用较强的 PD 跟踪伸腿轨迹。

---

## 3. FLIGHT：腾空

检测到离地后进入腾空阶段。

此时腿不再产生地面推力，主要完成两件事：

- **收腿**：将腿长缩短到
$$
L_{\text{retract}} = 0.30\text{ m}
$$
收腿时间约 $0.12\text{ s}$。

- **展腿**：约在腾空 $0.25\text{ s}$ 后开始展腿，并伸到
$$
L_{\text{touch}} = 0.45\text{ m}
$$
用于准备落地。

腿部通过

$$
L_d \rightarrow \text{IK} \rightarrow (q_{h,d}, q_{k,d})
$$

并使用关节 PD 跟踪。

空中轮子用于辅助机身姿态控制：

$$
\boxed{u_{\text{wheel}} = -\left(K_{p,\text{air}}(\theta - \theta_0) + K_{d,\text{air}}\dot\theta\right)}
$$

利用轮子加减速产生反作用力矩，修正机身俯仰。

---

## 4. TOUCHDOWN_BUFFER：落地缓冲

检测到触地后进入缓冲阶段。

目标是通过腿部压缩吸收冲击，而不是立即用高刚度位置控制将腿顶回去。

采用竖直方向阻抗控制：

$$
\boxed{F_z = \frac{M}{2}g + K_z(z_d - z) + D_z(\dot z_d - \dot z)}
$$

落地阶段：

$$
z_d = L_{\text{touch}},\qquad \dot z_d = 0
$$

所以：

$$
\boxed{F_z = \frac{M}{2}g + K_z(L_{\text{touch}} - z) - D_z\dot z}
$$

当前参数：

$$
K_z = 450\text{ N/m},\qquad D_z = 45\text{ N}\cdot\text{s/m}
$$

再通过实际关节构型计算 Jacobian：

$$
\tau = J_z^T F_z
$$

当前 TOUCHDOWN_BUFFER 会分别计算左右腿 Jacobian，再取平均值生成一组共同的前馈力矩：

$$
\bar J_z = \frac{J_{z,L} + J_{z,R}}{2}
$$

同时轮子重新启用较弱的 LQR，用于恢复机身平衡。

---

## 5. RECOVERY：恢复站立

落地缓冲稳定后进入恢复阶段。

通过新的五次多项式，将机器人高度平滑恢复到：

$$
L_{\text{stand}} = 0.40\text{ m}
$$

恢复时间约：

$$
0.45\text{ s}
$$

腿部继续使用高度阻抗控制：

$$
\boxed{F_z = \frac{M}{2}g + K_z(z_d - z) + D_z(\dot z_d - \dot z)}
$$

当前参数：

$$
K_z = 320\text{ N/m},\qquad D_z = 50\text{ N}\cdot\text{s/m}
$$

左右腿分别根据实际关节构型计算 Jacobian：

$$
\tau_L = J_{z,L}^T F_z
$$

$$
\tau_R = J_{z,R}^T F_z
$$

同时轮子使用较弱的 LQR 逐渐恢复平衡。

当前代码中，跳跃完成后会继续保持在 RECOVERY 状态，不自动切回普通 BALANCE。

---

## 总结

```text
BALANCE
   ↓
SQUAT
五次轨迹下蹲
   ↓
THRUST
五次轨迹 + F=M(g+a) + JᵀF
   ↓
FLIGHT
收腿 / 展腿 + 空中姿态控制
   ↓
TOUCHDOWN_BUFFER
阻抗控制吸收落地冲击
   ↓
RECOVERY
阻抗控制 + 五次轨迹恢复站立
```

三个最核心的公式：

$$
\boxed{F_z = \frac{M}{2}(g + \ddot z_d)}
$$

$$
\boxed{\tau = J_z^T F_z}
$$

$$
\boxed{F_z = \frac{M}{2}g + K_z(z_d - z) + D_z(\dot z_d - \dot z)}
$$