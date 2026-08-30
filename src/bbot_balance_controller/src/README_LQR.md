# BBot — 两轮自平衡机器人 (LQR 控制版)

## 平衡控制

系统采用**速度外环 PI + 姿态内环 LQR** 的串级控制架构，运行频率 200 Hz，系统在不同运动状态下能自动切换控制方法，保持稳定。

这个轮足机器人的 LQR控制，其核心思想是**将复杂的两轮足机器人简化为平面“轮式倒立摆”模型进行动力学稳态控制**。

无论机器人的两条腿怎么弯曲、离地高度是多少，只要双腿是对称动作的，它在纵向（X-Z平面）上的平衡问题就可以抽象为一个经典的四状态控制系统。

### 一、 状态空间建模

LQR 的前提是建立系统的状态空间方程。四个核心状态量组成状态向量 $X$：

$$X = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} \begin{matrix} \text{--- 轮轴水平位移} \\ \text{--- 轮轴水平速度} \\ \text{--- 机身俯仰角 (Pitch)} \\ \text{--- 机身俯仰角速度} \end{matrix}$$

系统的控制输入 $u$ 为**双轮的总驱动力矩**。在平衡点（机器人直立静止，所有状态量为 0）附近，通过拉格朗日方程建立非线性动力学模型，并进行线性化泰勒展开，得到状态空间矩阵：

$$\dot{X} = AX + Bu$$

- **$A$ 矩阵**：描述系统在没有外力矩时，状态量之间如何相互影响（例如：身体前倾 $\theta > 0$ 会导致俯仰角速度 $\dot{\theta}$ 进一步增大，这就是倒立摆的固有不稳定特性）。
- **$B$ 矩阵**：描述控制力矩 $u$ 作用到驱动轮上后，对系统水平加速度 $\ddot{x}$ 和机身角加速度 $\ddot{\theta}$ 的直接刚度贡献。

### 二、 能量最优化核心：二次型代价函数

LQR中的 **Q**，指的是它通过最小化一个“性能指标” $J$ 来寻找最优控制律：

$$J = \int_{0}^{\infty} (X^T Q X + u^T R u) \, dt$$

这个公式的物理本质是在做**控制效果**与**控制能量**之间的权衡：

- **$X^T Q X$ 项**： $Q = \text{diag}([20, 500, 2500, 150])$ 分别对应四个状态的惩罚。
- \*\*$u^T R u$ 项：$R = 1$ 限制了电机的最大输出。

MATLAB 的 `lqr(A, B, Q, R)` 函数求解，计算出一组能让总能量消耗 $J$ 达到理论上最小的反馈增益矩阵 $K$：

$$K = \begin{bmatrix} k_x & k_{\dot{x}} & k_{\theta} & k_{\dot{\theta}} \end{bmatrix}$$

### 三、 控制律在 C++ 控制器中的落地

#### 1. 静止状态

当机器人被命令静止时，LQR 发挥其完整的全状态反馈力：

$$u_{pitch} = - (k_x \cdot e_x + k_{\dot{x}} \cdot e_{\dot{x}} + k_{\theta} \cdot e_{\theta} + k_{\dot{\theta}} \cdot e_{\dot{\theta}})$$

- **位置项 $-k_x (x - x_{target})$**：充当了一个虚拟的“弹簧”，只要车子偏离了锁死点，它就会把车子拉回来，消除位移漂移。
- **速度项 $-k_{\dot{x}}(\dot{x} - 0)$**：充当一个“阻尼器”，压制车底盘的前后晃动。
- **角度与角速度项**：负责最根本的直立倒立摆控制。

#### 2. 运动巡航状态（前馈+反馈）

由于我们的底层接口是底盘速度接口，如果在运动时继续让 $k_x$ 和 $k_{\dot{x}}$ 作用，会由于两套速度反馈闭环打架而导致速度偏差和超调。（两个独立的控制器同时在对同一个状态量（车速 $\dot{x}$）进行闭环控制会导致冲突）

因此，在运动时将**线性运动从 LQR 内部剥离**，交给外环 PI 控制器。此时内环 LQR 退化为一个的**姿态追踪器**：

$$u_{pitch} = - \left[ k_{\theta}(\theta - \theta_{dynamic\_target}) + k_{\dot{\theta}}\dot{\theta} \right]$$

- 外环速度 PI 根据车速误差，计算当前速度下机器人应该前倾多少度（$\theta_{dynamic\_target}$）。
- 内环 LQR 接收到这个动态目标后，使用增益（$k_{\theta}$ 和 $k_{\dot{\theta}}$）让轮子去追踪这个倾角。

最终，通过 `cmd_x = u_pitch * cmd_scale_ + target_speed`，将 LQR 的直立纠偏速度与外环（PI）的目标前馈速度结合，然后发布速度。

## 高度控制

使用**逆运动学 (IK)** 将目标腿长实时解算为关节角度：

Q/E 调节目标腿长 L → IK(目标腿长) → (θ_hip, θ_knee) → 发布关节位置指令

## 运动学模型

### 参数

| 符号                 | 值                     | 说明                         |
| :------------------- | :--------------------- | :--------------------------- |
| $l_1$                | $0.30 \text{ m}$       | 小腿长度                     |
| $l_2$                | $0.30 \text{ m}$       | 大腿长度                     |
| $l_3$                | $0.10 \text{ m}$       | 髋关节轴心到机身连杆顶端距离 |
| $R$                  | $0.075 \text{ m}$      | 驱动轮半径                   |
| $m_{body}$           | $15.6 \text{ kg}$      | 机身质量                     |
| $m_{thigh} \times 2$ | $1.20 \text{ kg}$      | 左右大腿总质量               |
| $m_{shank} \times 2$ | $0.70 \text{ kg}$      | 左右小腿总质量               |
| $m_{wheel} \times 2$ | $2.13 \text{ kg}$      | 左右驱动轮总质量             |
| $M_{total}$          | **$19.63 \text{ kg}$** | 机器人全系统总质量           |

### 正运动学 (FK)

正运动学的核心任务是：**已知机身俯仰角和各关节的相对旋转角，推导计算出机器人各关节轴心的几何坐标，以及全系统的总质心位置。**

代码实现的坐标系基准为：**以轮轴轴心为原点 $(0,0)$，竖直向上为 $Z$ 轴正方向，水平向前为 $X$ 轴正方向。所有绝对角度均以竖直向上（$Z$ 轴正方向）为 $0$ 度基准，顺时针为正。**

#### 1. 关节相对角向连杆绝对角的转化

$$\theta_{\text{body}} = \text{body\_pitch}$$

$$\theta_{\text{thigh}} = \theta_{\text{body}} - \text{hip\_angle}$$

$$\theta_{\text{shank}} = \theta_{\text{thigh}} - \text{knee\_angle}$$

#### 2. 关键关节轴心坐标的几何迭代

- **膝关节位置 $(x_{\text{knee}}, z_{\text{knee}})$**：
  $$x_{\text{knee}} = l_1 \sin\theta_{\text{shank}}$$
  $$z_{\text{knee}} = l_1 \cos\theta_{\text{shank}}$$

- **髋关节位置 $(x_{\text{hip}}, z_{\text{hip}})$**：
  $$x_{\text{hip}} = x_{\text{knee}} + l_2 \sin\theta_{\text{thigh}}$$
  $$z_{\text{hip}} = z_{\text{knee}} + l_2 \cos\theta_{\text{thigh}}$$

- **机身参考点位置 $(x_{\text{body}}, z_{\text{body}})$**：
  $$x_{\text{body}} = x_{\text{hip}} + l_3 \sin\theta_{\text{body}}$$
  $$z_{\text{body}} = z_{\text{hip}} + l_3 \cos\theta_{\text{body}}$$

#### 3. 各连杆分体质心坐标计算

- **分体 0（驱动轮 $m_0$）**：
  $$(x_{c0}, z_{c0}) = (0, 0)$$

- **分体 1（小腿 $m_1$）**：
  $$x_{c1} = x_{1c} \sin\theta_{\text{shank}}, \quad z_{c1} = x_{1c} \cos\theta_{\text{shank}}$$

- **分体 2（大腿 $m_2$）**：
  $$x_{c2} = x_{\text{knee}} + x_{2c} \sin\theta_{\text{thigh}}, \quad z_{c2} = z_{\text{knee}} + x_{2c} \cos\theta_{\text{thigh}}$$

- **分体 3（机身 $m_3$）**：
  $$(x_{c3}, z_{c3}) = (x_{\text{body}}, z_{\text{body}})$$

#### 4. 系统总质心的加权合成

$$x_{\text{com}} = \frac{m_0 \cdot 0 + m_1 \cdot x_{c1} + m_2 \cdot x_{c2} + m_3 \cdot x_{\text{body}}}{M_{\text{total}}}$$

$$z_{\text{com}} = \frac{m_0 \cdot 0 + m_1 \cdot z_{c1} + m_2 \cdot z_{c2} + m_3 \cdot z_{\text{body}}}{M_{\text{total}}}$$

---

### 逆运动学 (IK)

逆运动学求解核心思想是：**将复杂的全机身多连杆质心问题，通过变量分离与常数合并，降维转化为一个经典的二维平面双关节臂几何求解问题。**

#### 1. 变量分离与已知量消除

提取运动学同类项归纳为常数系数 $a_1, b_1$，剥离出包含已知输入量 $\theta_{\text{body}}$（机身俯仰角）和目标质心位置 $(x_c, z_c)$ 的中间已知变量 $c_1$ 和 $d_1$：

$$\begin{cases} c_1 = M_{\text{total}} \cdot x_c - m_3 l_3 \sin\theta_{\text{body}} \\ d_1 = M_{\text{total}} \cdot z_c - m_3 l_3 \cos\theta_{\text{body}} \end{cases}$$

此时方程组简化为仅包含未知数 $\theta_{\text{shank}}$ 和 $\theta_{\text{thigh}}$ 的平面解析几何形式：

$$\begin{cases} c_1 = a_1 \sin\theta_{\text{shank}} + b_1 \sin\theta_{\text{thigh}} \\ d_1 = a_1 \cos\theta_{\text{shank}} + b_1 \cos\theta_{\text{thigh}} \end{cases}$$

#### 2. 求解膝关节角 $\theta_2$（`theta_knee`）

两式分别平方求和化简，利用余弦差角公式变换：

$$c_1^2 + d_1^2 = a_1^2 + b_1^2 + 2a_1b_1 \cos(\theta_{\text{thigh}} - \theta_{\text{shank}})$$

由此直接反解出膝关节相对弯曲角 $\theta_2$：

$$\theta_2 = \arccos\left(\frac{c_1^2 + d_1^2 - a_1^2 - b_1^2}{2a_1b_1}\right)$$

#### 3. 求解小腿绝对角与中间几何角

利用平面极坐标向量三角形关系，计算目标等效向量模长平方 $dist\_sq = c_1^2 + d_1^2$，反解方位角 $\alpha$ 与内角 $\beta$：

$$\alpha = \text{atan2}(d_1, c_1), \quad \beta = \arccos\left(\frac{a_1^2 + dist\_sq - b_1^2}{2a_1 \cdot dist}\right)$$

根据前屈腿机械几何构型，消除基准轴偏移量得到小腿物理关节角：

$$\theta_{\text{shank}} = \alpha - \beta - \frac{\pi}{2}$$

#### 4. 闭环约束反推髋关节角 $\theta_{\text{hip}}$

由串联几何链约束做差直接闭环反推：

$$\theta_{\text{hip}} = \theta_{\text{body}} - (\theta_{\text{shank}} + \theta_{\text{knee}})$$
