# BBot — 两轮自平衡机器人

## 平衡控制

三层串级 PID，200 Hz：

```
IMU → pitch, pitch_rate
JointState → x_dot (车速)

速度环(外) → 目标倾角 → 角度环(中) → 目标角速度 → 角速度环(内) → 轮速指令
```

### PID 增益调度

随高度在站立/蹲下两组参数间线性插值：

| 环     | 站立                          | 蹲下                          |
| ------ | ----------------------------- | ----------------------------- |
| 速度   | {0.35, 0.08, 0.002, 0.0, 1.2} | {0.40, 0.10, 0.005, 0.0, 1.5} |
| 角度   | {7.0, 0.0, 0.06, 0.0, 1.5}    | {6.8, 0.0, 0.04, 0.0, 1.7}    |
| 角速度 | {1.4, 0.0, 0.01, 8.0, 4.0}    | {1.3, 0.0, 0.007, 8.0, 4.2}   |

## 高度控制

使用**逆运动学 (IK)** 将目标质心高度实时解算为关节角度：

```
Q/E 调节目标高度 h → IK(目标高度) → (θ_hip, θ_knee) → 发布关节位置指令
```

## 运动学模型

### 参数

| 符号      | 值           | 说明         |
| --------- | ------------ | ------------ |
| l₁        | 0.30 m       | 小腿长度     |
| l₂        | 0.30 m       | 大腿长度     |
| l₃        | 0.10 m       | 髋到机身质心 |
| R         | 0.075 m      | 轮半径       |
| m_body    | 15.6 kg      | 机身         |
| m_thigh×2 | 1.20 kg      | 双腿大腿     |
| m_shank×2 | 0.70 kg      | 双腿小腿     |
| m_wheel×2 | 2.13 kg      | 双轮         |
| M_total   | **19.63 kg** | 总质量       |

### 正运动学 (FK)

正运动学的核心任务是：**已知机身俯仰角和各关节的相对旋转角，推导计算出机器人各关节轴心的几何坐标，以及全系统的总质心位置。**

代码实现的坐标系基准为：**以轮轴轴心为原点 $(0,0)$，竖直向上为 $Z$ 轴正方向，水平向前为 $X$ 轴正方向。所有绝对角度均以竖直向上（$Z$ 轴正方向）为 $0$ 度基准，顺时针为正。**

---

#### 1. 关节相对角向连杆绝对角的转化

传感器直接测得的是关节电机的相对旋转角：`hip_angle`（髋关节角）和 `knee_angle`（膝关节角）。为了便于在笛卡尔坐标系下进行向量叠加，首先需要计算各连杆相对于竖直方向的绝对夹角：

- **机身绝对角** $\theta_{\text{body}}$：直接等于机身俯仰角 `body_pitch`。
- **大腿绝对角** $\theta_{\text{thigh}}$：机身受到髋关节电机驱动，大腿相对于机身向后转动，因此：

$$\theta_{\text{thigh}} = \theta_{\text{body}} - \text{hip\_angle}$$

- **小腿绝对角** $\theta_{\text{shank}}$：小腿受到膝关节电机驱动，相对于大腿进一步向后弯曲，因此：

$$\theta_{\text{shank}} = \theta_{\text{thigh}} - \text{knee\_angle}$$

---

#### 2. 关键关节轴心坐标的几何迭代

利用三角函数，顺着“轮轴 $\rightarrow$ 膝关节 $\rightarrow$ 髋关节 $\rightarrow$ 机身连杆端点”的运动链，逐级累加各个连杆的杆长向量。

#### (1) 膝关节位置 $(x_{\text{knee}}, z_{\text{knee}})$

小腿一端连接在轮轴（原点），长度为 $l_1$，绝对角为 $\theta_{\text{shank}}$。根据几何关系：

$$x_{\text{knee}} = l_1 \sin\theta_{\text{shank}}$$

$$z_{\text{knee}} = l_1 \cos\theta_{\text{shank}}$$

#### (2) 髋关节位置 $(x_{\text{hip}}, z_{\text{hip}})$

大腿一端连接在膝关节，长度为 $l_2$，绝对角为 $\theta_{\text{thigh}}$。在膝关节坐标的基础上累加：

$$x_{\text{hip}} = x_{\text{knee}} + l_2 \sin\theta_{\text{thigh}}$$

$$z_{\text{hip}} = z_{\text{knee}} + l_2 \cos\theta_{\text{thigh}}$$

#### (3) 机身参考点位置 $(x_{\text{body}}, z_{\text{body}})$

机身连杆一端连接在髋关节，长度为 $l_3$，绝对角为 $\theta_{\text{body}}$。在髋关节坐标的基础上累加：

$$x_{\text{body}} = x_{\text{hip}} + l_3 \sin\theta_{\text{body}}$$

$$z_{\text{body}} = z_{\text{hip}} + l_3 \cos\theta_{\text{body}}$$

---

#### 3. 各连杆分体质心坐标计算

各个连杆的物理质量并非集中在关节轴心，而是分布在各自的质心上。

- **分体 0（驱动轮 $m_0$）**：质心直接位于轮轴中心，即：

$$(x_{c0}, z_{c0}) = (0, 0)$$

- **分体 1（小腿 $m_1$）**：小腿质心到轮轴的距离为 $x_{1c}$，沿小腿绝对角方向：

$$x_{c1} = x_{1c} \sin\theta_{\text{shank}}$$

$$z_{c1} = x_{1c} \cos\theta_{\text{shank}}$$

- **分体 2（大腿 $m_2$）**：大腿质心到膝关节的距离为 $x_{2c}$，在膝关节坐标基础上累加：

$$x_{c2} = x_{\text{knee}} + x_{2c} \sin\theta_{\text{thigh}}$$

$$z_{c2} = z_{\text{knee}} + x_{2c} \cos\theta_{\text{thigh}}$$

- **分体 3（机身 $m_3$）**：在此段代码的建模中，直接将分体 3 的质心视为了机身参考点位置（即忽略了机身内部质心的微小偏移）：

$$(x_{c3}, z_{c3}) = (x_{\text{body}}, z_{\text{body}})$$

---

#### 4. 系统总质心的加权合成

最后，根据刚体系的质心定义，全系统的总质心坐标 $(x_{\text{com}}, z_{\text{com}})$ 是各个分体质心坐标关于其质量的加权平均值：

$$x_{\text{com}} = \frac{m_0 \cdot 0 + m_1 \cdot x_{c1} + m_2 \cdot x_{c2} + m_3 \cdot x_{\text{body}}}{M_{\text{total}}}$$

$$z_{\text{com}} = \frac{m_0 \cdot 0 + m_1 \cdot z_{c1} + m_2 \cdot z_{c2} + m_3 \cdot z_{\text{body}}}{M_{\text{total}}}$$

其中 $M_{\text{total}} = m_0 + m_1 + m_2 + m_3$。这两行最终的加权平均公式，完全对应了代码中 `forward_kinematics` 的结尾输出。

### 逆运动学 (IK)

逆运动学求解核心思想是：**将复杂的全机身多连杆质心问题，通过变量分离与常数合并，降维转化为一个经典的二维平面双关节臂几何求解问题**。

---

#### 一、 建立质心运动学方程

定义系统的总质量为 $M_{\text{total}}$。将小腿绝对角设为 $\theta_{\text{shank}}$，大腿绝对角设为 $\theta_{\text{thigh}}$，机身俯仰角设为 $\theta_{\text{body}}$（即 `body_pitch`）。全系统质心坐标 $(x_c, z_c)$ 满足：

$$M_{\text{total}} \cdot x_c = m_1 (x_{1c} \sin\theta_{\text{shank}}) + m_2 (l_1 \sin\theta_{\text{shank}} + x_{2c} \sin\theta_{\text{thigh}}) + m_3 (l_1 \sin\theta_{\text{shank}} + l_2 \sin\theta_{\text{thigh}} + l_3 \sin\theta_{\text{body}})$$

$$M_{\text{total}} \cdot z_c = m_1 (x_{1c} \cos\theta_{\text{shank}}) + m_2 (l_1 \cos\theta_{\text{shank}} + x_{2c} \cos\theta_{\text{thigh}}) + m_3 (l_1 \cos\theta_{\text{shank}} + l_2 \cos\theta_{\text{thigh}} + l_3 \cos\theta_{\text{body}})$$

提取同类项 $\sin\theta_{\text{shank}}$、$\sin\theta_{\text{thigh}}$ 等，可以将上式整理为：

$$M_{\text{total}} \cdot x_c = \underbrace{(m_1 x_{1c} + m_2 l_1 + m_3 l_1)}_{a_1} \sin\theta_{\text{shank}} + \underbrace{(m_2 x_{2c} + m_3 l_2)}_{b_1} \sin\theta_{\text{thigh}} + m_3 l_3 \sin\theta_{\text{body}}$$

$$M_{\text{total}} \cdot z_c = \underbrace{(m_1 x_{1c} + m_2 l_1 + m_3 l_1)}_{a_1} \cos\theta_{\text{shank}} + \underbrace{(m_2 x_{2c} + m_3 l_2)}_{b_1} \cos\theta_{\text{thigh}} + m_3 l_3 \cos\theta_{\text{body}}$$

---

#### 二、 变量分离与已知量消除

在逆运动学求解时，机身俯仰角 $\theta_{\text{body}}$（`body_pitch`）和目标质心位置 $(x_c, z_c)$ 均是**已知输入量**（代码中设定目标 $x_c = 0$，即质心在轮轴正上方）。

将包含已知量 $\theta_{\text{body}}$ 的项移到方程左边：

$$M_{\text{total}} \cdot x_c - m_3 l_3 \sin\theta_{\text{body}} = a_1 \sin\theta_{\text{shank}} + b_1 \sin\theta_{\text{thigh}}$$

$$M_{\text{total}} \cdot z_c - m_3 l_3 \cos\theta_{\text{body}} = a_1 \cos\theta_{\text{shank}} + b_1 \cos\theta_{\text{thigh}}$$

令左边整体为中间已知变量 $c_1$ 和 $d_1$：

$$\begin{cases} c_1 = M_{\text{total}} \cdot x_c - m_3 l_3 \sin\theta_{\text{body}} \\ d_1 = M_{\text{total}} \cdot z_c - m_3 l_3 \cos\theta_{\text{body}} \end{cases}$$

此时方程组简化为仅包含未知数 $\theta_{\text{shank}}$ 和 $\theta_{\text{thigh}}$ 的二级形式：

$$\begin{cases} c_1 = a_1 \sin\theta_{\text{shank}} + b_1 \sin\theta_{\text{thigh}} \quad \text{--- (1)} \\ d_1 = a_1 \cos\theta_{\text{shank}} + b_1 \cos\theta_{\text{thigh}} \quad \text{--- (2)} \end{cases}$$

---

#### 三、 求解膝关节角 $\theta_2$（`theta_knee`）

为了消除绝对角度项，将方程 (1) 和 (2) 两边分别平方并相加：

$$c_1^2 + d_1^2 = (a_1 \sin\theta_{\text{shank}} + b_1 \sin\theta_{\text{thigh}})^2 + (a_1 \cos\theta_{\text{shank}} + b_1 \cos\theta_{\text{thigh}})^2$$

展开后利用三角恒等式 $\sin^2\theta + \cos^2\theta = 1$ 进行化简：

$$c_1^2 + d_1^2 = a_1^2 + b_1^2 + 2a_1b_1(\sin\theta_{\text{shank}}\sin\theta_{\text{thigh}} + \cos\theta_{\text{shank}}\cos\theta_{\text{thigh}})$$

根据余弦差角公式 $\cos(\alpha - \beta) = \cos\alpha\cos\beta + \sin\alpha\sin\beta$：

$$c_1^2 + d_1^2 = a_1^2 + b_1^2 + 2a_1b_1 \cos(\theta_{\text{thigh}} - \theta_{\text{shank}})$$

而在机械臂构型中，大小腿的绝对角度差即为膝关节的相对弯曲角：$\theta_2 = \theta_{\text{thigh}} - \theta_{\text{shank}}$。代入上式得：

$$c_1^2 + d_1^2 = a_1^2 + b_1^2 + 2a_1b_1 \cos\theta_2$$

由此可直接反解出 $\theta_2$：

$$\cos\theta_2 = \frac{c_1^2 + d_1^2 - a_1^2 - b_1^2}{2a_1b_1}$$

$$\theta_2 = \arccos\left(\frac{c_1^2 + d_1^2 - a_1^2 - b_1^2}{2a_1b_1}\right)$$

---

#### 四、 求解小腿绝对角 $\theta_{\text{shank}}$ 与中间角 $\theta_{01}$

代码在求解基础连杆角度时，利用了平面几何的向量三角形关系。

设目标等效向量为 $\vec{P} = (c_1, d_1)$，其模长的平方为 $dist\_sq = c_1^2 + d_1^2$。

1. **计算目标向量的方位角 $\alpha$**：

$$\alpha = \text{atan2}(d_1, c_1)$$

2. **利用余弦定理计算三角形内角 $\beta$**（$\beta$ 为由 $a_1$ 和 $dist$ 组成的夹角，其对边为 $b_1$）：

$$b_1^2 = a_1^2 + dist^2 - 2a_1 \cdot dist \cdot \cos\beta \implies \cos\beta = \frac{a_1^2 + dist^2 - b_1^2}{2a_1 \cdot dist}$$

$$\beta = \arccos\left(\frac{a_1^2 + dist^2 - b_1^2}{2a_1 \cdot dist}\right)$$

3. **合成基准组合角 $\theta_{01}$**：
   根据代码中选定的后屈腿/前屈腿几何构型，连杆绝对极坐标角度 $\theta_{01}$ 表达式为：

$$\theta_{01} = \alpha - \beta$$

4. **转换至机器人本身的物理关节角**：
   由于机器人的 $\theta_{\text{shank}}$ 是以竖直方向为 0 度的相对编码器角，而几何推导中的 $\theta_{01}$ 包含了 `M_PI / 2.0` 的基准轴偏移量（$\theta_0$），因此需要减去该偏移量：

$$\theta_{\text{shank}} = \theta_{01} - \frac{\pi}{2}$$

---

#### 五、 闭环约束反推髋关节角 $\theta_{\text{hip}}$

根据关节链的串联定义，连杆绝对角度由上至下依次作差：

$$\theta_{\text{thigh}} = \theta_{\text{body}} - \theta_{\text{hip}}$$

$$\theta_{\text{shank}} = \theta_{\text{thigh}} - \theta_{\text{knee}}$$

将第一式代入第二式，消除中间变量 $\theta_{\text{thigh}}$：

$$\theta_{\text{shank}} = (\theta_{\text{body}} - \theta_{\text{hip}}) - \theta_{\text{knee}}$$

移项整理，即可由已求得的 $\theta_{\text{shank}}$ 和 $\theta_{\text{knee}}$（即 `theta2`）直接算出最终的髋关节控制角 $\theta_{\text{hip}}$：

$$\theta_{\text{hip}} = \theta_{\text{body}} - (\theta_{\text{shank}} + \theta_{\text{knee}})$$

---

### 关节重力矩

虚功原理：`τ = Jᵀ · [0, -M_total · g]ᵀ`

|             | 膝力矩 (总) | 膝力矩 (单腿) |
| ----------- | ----------- | ------------- |
| 站立 0.549m | 0 Nm        | 0 Nm          |
| 蹲下 0.407m | 34.5 Nm     | 17 Nm         |

平面模型算的是双腿总力矩，单腿值 ÷2，与仿真 effort 对应。

## 数据记录

运行控制器后自动生成到 `src/bbot_balance_controller/src/data_logs/`：

| 文件                    | 内容                                     |
| ----------------------- | ---------------------------------------- |
| `height_torque_log.csv` | 时间戳、高度、关节角、理论力矩、仿真力矩 |
| `*_data.txt`            | 速度/角度/角速度跟踪数据                 |

## 包结构

| 包                        | 用途                             |
| ------------------------- | -------------------------------- |
| `bbot_description`        | URDF 模型 + ros2_control         |
| `bbot_bringup`            | Gazebo 启动 + 控制器配置         |
| `bbot_balance_controller` | 串级 PID 平衡控制器              |
| `bbot_kinematics`         | 运动学库 (FK/IK/Jacobian/重力矩) |
| `bbot_torque_control`     | 力矩监控 + 高度力矩控制          |
