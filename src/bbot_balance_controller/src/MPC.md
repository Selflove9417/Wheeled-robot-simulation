# BBot Linear MPC 完整数学推导

这份代码实现的是一个定高度线性 MPC。

它使用的离散状态空间模型为：

$$
e_{k+1}=A_de_k+B_du_k
$$

为了简化后面的公式，下面记：

$$
A=A_d
$$

$$
B=B_d
$$

所以系统写成：

$$
\boxed{
e_{k+1}=Ae_k+Bu_k
}
$$

---

# 1. MPC 使用的状态

代码中的状态不是机器人的绝对状态，而是已经处理好的误差状态：

$$
e_0=
\begin{bmatrix}
x_{\mathrm{error}}\\
v_{\mathrm{error}}\\
\theta_{\mathrm{error}}\\
\dot{\theta}
\end{bmatrix}
$$

其中：

$$
x_{\mathrm{error}}
=
x-x_{\mathrm{ref}}
$$

$$
v_{\mathrm{error}}
=
\dot{x}-v_{\mathrm{ref}}
$$

$$
\theta_{\mathrm{error}}
=
\theta-\theta_{\mathrm{eq}}
$$

第四个状态是：

$$
\dot{\theta}
$$

因此：

$$
\boxed{
e_0\in\mathbb{R}^4
}
$$

这里的参考值已经被吸收到误差状态里面了。

所以 MPC 不需要额外构造参考轨迹：

$$
r_1,r_2,\cdots,r_N
$$

它的控制目标可以直接理解成：

$$
\boxed{
e_i\rightarrow0
}
$$

---

# 2. 控制输入

模型输入是两轮的总驱动转矩：

$$
u_k\in\mathbb{R}
$$

也就是说这是一个单输入系统。

系统模型：

$$
e_{k+1}=Ae_k+Bu_k
$$

其中：

$$
A\in\mathbb{R}^{4\times4}
$$

$$
B\in\mathbb{R}^{4\times1}
$$

---

# 3. 定义预测时域

源码默认：

$$
N=20
$$

也就是在当前时刻，MPC 一次预测未来 20 个状态：

$$
e_1,e_2,\cdots,e_N
$$

同时求未来 20 个控制输入：

$$
u_0,u_1,\cdots,u_{N-1}
$$

这里：

$$
e_0
$$

是当前已经测量到的状态，因此不属于“未来预测状态”。

---

# 4. 第一步预测

系统模型：

$$
e_{k+1}=Ae_k+Bu_k
$$

当前状态记为：

$$
e_0
$$

所以第一步预测：

$$
\boxed{
e_1=Ae_0+Bu_0
}
$$

---

# 5. 第二步预测

根据系统模型：

$$
e_2=Ae_1+Bu_1
$$

代入：

$$
e_1=Ae_0+Bu_0
$$

得到：

$$
e_2
=
A(Ae_0+Bu_0)+Bu_1
$$

展开：

$$
\boxed{
e_2
=
A^2e_0
+
ABu_0
+
Bu_1
}
$$

---

# 6. 第三步预测

继续：

$$
e_3=Ae_2+Bu_2
$$

代入：

$$
e_2
=
A^2e_0+ABu_0+Bu_1
$$

得到：

$$
e_3
=
A
\left(
A^2e_0+ABu_0+Bu_1
\right)
+
Bu_2
$$

展开：

$$
\boxed{
e_3
=
A^3e_0
+
A^2Bu_0
+
ABu_1
+
Bu_2
}
$$

---

# 7. 推广到第 i 步

因此一般形式为：

$$
\boxed{
e_i
=
A^ie_0
+
\sum_{j=0}^{i-1}
A^{i-1-j}Bu_j
}
$$

这就是这份 MPC 代码使用的多步预测公式。

它可以理解成：

$$
\boxed{
\text{未来状态}
=
\text{当前状态的自由演化}
+
\text{未来控制的影响}
}
$$

---

# 8. 把所有未来状态堆起来

定义未来状态向量：

$$
E=
\begin{bmatrix}
e_1\\
e_2\\
e_3\\
\vdots\\
e_N
\end{bmatrix}
$$

因为每一个：

$$
e_i\in\mathbb{R}^4
$$

所以：

$$
\boxed{
E\in\mathbb{R}^{4N}
}
$$

注意这里没有：

$$
e_0
$$

这是这份代码与一些教材写法的重要区别。

---

# 9. 把未来控制输入堆起来

定义：

$$
U=
\begin{bmatrix}
u_0\\
u_1\\
u_2\\
\vdots\\
u_{N-1}
\end{bmatrix}
$$

因为每一个输入都是标量：

$$
u_i\in\mathbb{R}
$$

所以：

$$
\boxed{
U\in\mathbb{R}^{N}
}
$$

默认：

$$
N=20
$$

因此：

$$
U\in\mathbb{R}^{20}
$$

---

# 10. 构造状态预测矩阵 Ψ

观察：

$$
e_1=Ae_0+Bu_0
$$

$$
e_2=A^2e_0+ABu_0+Bu_1
$$

$$
e_3=A^3e_0+A^2Bu_0+ABu_1+Bu_2
$$

所有与当前状态：

$$
e_0
$$

有关的部分分别为：

$$
Ae_0
$$

$$
A^2e_0
$$

$$
A^3e_0
$$

因此定义：

$$
\boxed{
\Psi=
\begin{bmatrix}
A\\
A^2\\
A^3\\
\vdots\\
A^N
\end{bmatrix}
}
$$

矩阵尺寸：

$$
\boxed{
\Psi\in\mathbb{R}^{4N\times4}
}
$$

于是如果未来全部不给控制输入：

$$
U=0
$$

那么未来状态就是：

$$
\boxed{
E_{\mathrm{free}}
=
\Psi e_0
}
$$

这就是代码里面的：

$$
\text{free\_response}
$$

也就是“自由响应”。

---

# 11. 构造控制预测矩阵 Θ

现在整理所有控制输入对未来状态的影响。

第一步：

$$
e_1=Ae_0+Bu_0
$$

控制部分：

$$
Bu_0
$$

第二步：

$$
e_2=A^2e_0+ABu_0+Bu_1
$$

控制部分：

$$
ABu_0+Bu_1
$$

第三步：

$$
e_3=A^3e_0+A^2Bu_0+ABu_1+Bu_2
$$

控制部分：

$$
A^2Bu_0+ABu_1+Bu_2
$$

因此：

$$
\boxed{
\Theta=
\begin{bmatrix}
B&0&0&\cdots&0\\
AB&B&0&\cdots&0\\
A^2B&AB&B&\cdots&0\\
A^3B&A^2B&AB&\ddots&0\\
\vdots&\vdots&\vdots&\ddots&B\\
A^{N-1}B&A^{N-2}B&A^{N-3}B&\cdots&B
\end{bmatrix}
}
$$

因为：

$$
E\in\mathbb{R}^{4N}
$$

$$
U\in\mathbb{R}^{N}
$$

所以：

$$
\boxed{
\Theta\in\mathbb{R}^{4N\times N}
}
$$

---

# 12. 得到完整预测模型

最终：

$$
\boxed{
E=\Psi e_0+\Theta U
}
$$

这是整份 MPC 最核心的预测公式。

可以把它分成：

$$
E_{\mathrm{free}}=\Psi e_0
$$

以及：

$$
E_{\mathrm{control}}=\Theta U
$$

因此：

$$
\boxed{
E
=
E_{\mathrm{free}}
+
E_{\mathrm{control}}
}
$$

也就是说：

当前状态决定机器人自己会怎么发展，而未来控制输入负责改变这个未来。

---

# 13. 接下来定义 MPC 的代价函数

这份代码实际使用的代价函数为：

$$
\boxed{
J=
\sum_{i=1}^{N}e_i^TQe_i
+
e_N^TPe_N
+
\sum_{j=0}^{N-1}Ru_j^2
+
W(s_++s_-)
+
w_2(s_+^2+s_-^2)
}
$$

注意：

这里严格按照源码。

状态：

$$
e_1,e_2,\cdots,e_N
$$

全部都会受到：

$$
Q
$$

的惩罚。

同时最后一个状态：

$$
e_N
$$

还会额外受到：

$$
P
$$

的终端惩罚。

因此最后一个状态实际权重是：

$$
\boxed{
Q+P
}
$$

而不是只有：

$$
P
$$

---

# 14. 状态代价

第一部分：

$$
\sum_{i=1}^{N}e_i^TQe_i
$$

假设：

$$
Q=
\begin{bmatrix}
q_x&0&0&0\\
0&q_v&0&0\\
0&0&q_\theta&0\\
0&0&0&q_\omega
\end{bmatrix}
$$

那么：

$$
e_i^TQe_i
=
q_xx_{e,i}^2
+
q_vv_{e,i}^2
+
q_\theta\theta_{e,i}^2
+
q_\omega\omega_i^2
$$

因此：

$$
Q
$$

决定 MPC 对：

$$
x_{\mathrm{error}}
$$

$$
v_{\mathrm{error}}
$$

$$
\theta_{\mathrm{error}}
$$

$$
\dot{\theta}
$$

分别有多在意。

---

# 15. 终端代价

除了普通状态代价以外，最后一步：

$$
e_N
$$

还有：

$$
e_N^TPe_N
$$

因此：

$$
\boxed{
J_{\mathrm{terminal}}
=
e_N^TPe_N
}
$$

源码中的：

$$
P
$$

由对应：

$$
Q,R
$$

的离散代数 Riccati 方程得到。

---

# 16. 输入代价

未来每个控制输入都要付出：

$$
Ru_j^2
$$

因此：

$$
\boxed{
J_u
=
\sum_{j=0}^{N-1}Ru_j^2
}
$$

由于：

$$
U=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_{N-1}
\end{bmatrix}
$$

所以：

$$
\boxed{
J_u
=
U^T(RI_N)U
}
$$

其中：

$$
I_N
$$

是：

$$
N\times N
$$

单位矩阵。

---

# 17. Slack 变量是什么？

代码额外定义了两个变量：

$$
s_+
$$

和：

$$
s_-
$$

其中：

$$
s_+\geq0
$$

$$
s_-\geq0
$$

它们负责在必要时放松 pitch 约束。

对应的代价是：

$$
\boxed{
J_s
=
W(s_++s_-)
+
w_2(s_+^2+s_-^2)
}
$$

其中源码默认：

$$
W=10^5
$$

因此 MPC 非常不愿意使用 slack。

只有当 pitch 约束确实无法满足时，才会允许：

$$
s_+>0
$$

或者：

$$
s_->0
$$

---

# 18. 将所有状态权重堆成矩阵

定义：

$$
\mathcal Q=
\begin{bmatrix}
Q&0&0&\cdots&0\\
0&Q&0&\cdots&0\\
0&0&Q&\cdots&0\\
\vdots&\vdots&\vdots&\ddots&0\\
0&0&0&0&Q+P
\end{bmatrix}
$$

也可以写成：

$$
\boxed{
\mathcal Q
=
\operatorname{diag}
(
Q,Q,\cdots,Q,Q+P
)
}
$$

矩阵尺寸：

$$
\mathcal Q
\in
\mathbb{R}^{4N\times4N}
$$

于是状态代价可以写成：

$$
\boxed{
J_x
=
E^T\mathcal QE
}
$$

---

# 19. 现在把预测模型代入状态代价

我们已经知道：

$$
E=\Psi e_0+\Theta U
$$

所以：

$$
J_x
=
(\Psi e_0+\Theta U)^T
\mathcal Q
(\Psi e_0+\Theta U)
$$

处理左边转置：

$$
(\Psi e_0+\Theta U)^T
=
e_0^T\Psi^T
+
U^T\Theta^T
$$

因此：

$$
J_x
=
(e_0^T\Psi^T+U^T\Theta^T)
\mathcal Q
(\Psi e_0+\Theta U)
$$

展开得到：

$$
\begin{aligned}
J_x
={}&
e_0^T\Psi^T\mathcal Q\Psi e_0\\
&+
e_0^T\Psi^T\mathcal Q\Theta U\\
&+
U^T\Theta^T\mathcal Q\Psi e_0\\
&+
U^T\Theta^T\mathcal Q\Theta U
\end{aligned}
$$

由于中间两项都是标量，而且：

$$
\mathcal Q^T=\mathcal Q
$$

所以：

$$
e_0^T\Psi^T\mathcal Q\Theta U
=
U^T\Theta^T\mathcal Q\Psi e_0
$$

因此：

$$
\boxed{
J_x
=
e_0^T\Psi^T\mathcal Q\Psi e_0
+
2e_0^T\Psi^T\mathcal Q\Theta U
+
U^T\Theta^T\mathcal Q\Theta U
}
$$

---

# 20. 加上控制输入代价

输入代价：

$$
J_u
=
U^T(RI_N)U
$$

因此：

$$
\begin{aligned}
J_x+J_u
={}&
e_0^T\Psi^T\mathcal Q\Psi e_0\\
&+
2e_0^T\Psi^T\mathcal Q\Theta U\\
&+
U^T
\left(
\Theta^T\mathcal Q\Theta
+
RI_N
\right)
U
\end{aligned}
$$

---

# 21. 去掉与优化变量无关的常数

当前控制周期：

$$
e_0
$$

已经测量得到。

所以：

$$
e_0^T\Psi^T\mathcal Q\Psi e_0
$$

对于优化变量：

$$
U
$$

来说是常数。

因此它不会影响最优：

$$
U^*
$$

所以优化时可以忽略。

于是剩下：

$$
\boxed{
J_U
=
U^T
\left(
\Theta^T\mathcal Q\Theta+RI_N
\right)
U
+
2e_0^T\Psi^T\mathcal Q\Theta U
}
$$

---

# 22. 把线性项改写一下

因为：

$$
2e_0^T\Psi^T\mathcal Q\Theta U
$$

是标量，所以：

$$
2e_0^T\Psi^T\mathcal Q\Theta U
=
\left(
2\Theta^T\mathcal Q\Psi e_0
\right)^TU
$$

定义：

$$
\boxed{
g_U
=
2\Theta^T\mathcal Q\Psi e_0
}
$$

这就是代码中的：

$$
\text{gradient.head}(N)
$$

---

# 23. 定义控制部分 Hessian

定义：

$$
\boxed{
H_U
=
2
\left(
\Theta^T\mathcal Q\Theta
+
RI_N
\right)
}
$$

那么：

$$
U^T
\left(
\Theta^T\mathcal Q\Theta
+
RI_N
\right)
U
$$

可以写成：

$$
\frac12U^TH_UU
$$

因此：

$$
\boxed{
J_U
=
\frac12U^TH_UU
+
g_U^TU
}
$$

---

# 24. 把 slack 变量也加入优化变量

定义完整优化变量：

$$
\boxed{
z=
\begin{bmatrix}
U\\
s_+\\
s_-
\end{bmatrix}
}
$$

展开：

$$
z=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_{N-1}\\
s_+\\
s_-
\end{bmatrix}
$$

因此：

$$
\boxed{
z\in\mathbb{R}^{N+2}
}
$$

默认：

$$
N=20
$$

因此：

$$
z\in\mathbb{R}^{22}
$$

---

# 25. 完整 Hessian

Slack 二次惩罚为：

$$
w_2s_+^2+w_2s_-^2
$$

所以完整 Hessian 为：

$$
\boxed{
H=
\begin{bmatrix}
H_U&0&0\\
0&2w_2&0\\
0&0&2w_2
\end{bmatrix}
}
$$

即：

$$
\boxed{
H=
\begin{bmatrix}
2(\Theta^T\mathcal Q\Theta+RI_N)&0&0\\
0&2w_2&0\\
0&0&2w_2
\end{bmatrix}
}
$$

---

# 26. 完整 gradient

控制部分：

$$
g_U
=
2\Theta^T\mathcal Q\Psi e_0
$$

Slack 线性代价：

$$
W(s_++s_-)
$$

因此：

$$
\boxed{
g=
\begin{bmatrix}
2\Theta^T\mathcal Q\Psi e_0\\
W\\
W
\end{bmatrix}
}
$$

---

# 27. 最终目标函数

因此 MPC 的优化目标整理成：

$$
\boxed{
\min_z
\frac12z^THz+g^Tz
}
$$

其中：

$$
z=
\begin{bmatrix}
U\\
s_+\\
s_-
\end{bmatrix}
$$

这就是代码传给 QP 求解器的核心目标函数。

---

# 28. 接下来推导输入硬约束

模型输入有上限：

$$
|u_j|
\leq
u_{\mathrm{bound}}
$$

其中：

$$
u_{\mathrm{bound}}
=
\min
\left(
u_{\mathrm{total,max}},
2\tau_{\mathrm{wheel,max}}
\right)
$$

当前默认：

$$
u_{\mathrm{total,max}}=20
$$

$$
\tau_{\mathrm{wheel,max}}=10
$$

因此：

$$
u_{\mathrm{bound}}
=
\min(20,20)
=
20
$$

即：

$$
\boxed{
|u_j|\leq20
}
$$

对于每一个：

$$
j=0,\cdots,N-1
$$

有：

$$
u_j\leq u_{\mathrm{bound}}
$$

以及：

$$
-u_j\leq u_{\mathrm{bound}}
$$

也就是：

$$
\boxed{
-u_{\mathrm{bound}}
\leq
u_j
\leq
u_{\mathrm{bound}}
}
$$

---

# 29. Pitch 是未来状态的第三个分量

状态顺序：

$$
e_i=
\begin{bmatrix}
x_{\mathrm{error}}\\
v_{\mathrm{error}}\\
\theta_{\mathrm{error}}\\
\dot{\theta}
\end{bmatrix}
$$

定义选择向量：

$$
c_\theta^T=
\begin{bmatrix}
0&0&1&0
\end{bmatrix}
$$

那么第 $i$ 步预测 pitch 误差就是：

$$
\theta_i
=
c_\theta^Te_i
$$

---

# 30. 将 pitch 预测拆成自由响应和控制响应

因为：

$$
E=\Psi e_0+\Theta U
$$

所以第 $i$ 个状态：

$$
e_i
=
\Psi_i e_0+\Theta_iU
$$

因此：

$$
\theta_i
=
c_\theta^T\Psi_i e_0
+
c_\theta^T\Theta_iU
$$

定义：

$$
f_i
=
c_\theta^T\Psi_i e_0
$$

它表示零控制输入时，第 $i$ 步的预测 pitch。

再定义：

$$
r_i
=
c_\theta^T\Theta_i
$$

则：

$$
\boxed{
\theta_i
=
f_i+r_iU
}
$$

这是 pitch 约束推导的核心公式。

---

# 31. 如果使用硬 Pitch 上界

理想要求：

$$
\theta_i
\leq
\theta_{\mathrm{limit}}
$$

代入：

$$
f_i+r_iU
\leq
\theta_{\mathrm{limit}}
$$

得到：

$$
r_iU
\leq
\theta_{\mathrm{limit}}-f_i
$$

---

# 32. 代码没有直接使用硬约束，而是加入上界 slack

代码实际使用：

$$
\boxed{
\theta_i-s_+
\leq
\theta_{\mathrm{limit}}
}
$$

代入：

$$
\theta_i=f_i+r_iU
$$

得到：

$$
f_i+r_iU-s_+
\leq
\theta_{\mathrm{limit}}
$$

整理：

$$
\boxed{
r_iU-s_+
\leq
\theta_{\mathrm{limit}}-f_i
}
$$

---

# 33. Pitch 下界

希望：

$$
\theta_i
\geq
-\theta_{\mathrm{limit}}
$$

等价于：

$$
-\theta_i
\leq
\theta_{\mathrm{limit}}
$$

代码加入另外一个 slack：

$$
-\theta_i-s_-
\leq
\theta_{\mathrm{limit}}
$$

代入：

$$
\theta_i=f_i+r_iU
$$

得到：

$$
-f_i-r_iU-s_-
\leq
\theta_{\mathrm{limit}}
$$

整理：

$$
\boxed{
-r_iU-s_-
\leq
\theta_{\mathrm{limit}}+f_i
}
$$

---

# 34. Slack 自身必须非负

要求：

$$
s_+\geq0
$$

写成标准“小于等于”形式：

$$
\boxed{
-s_+\leq0
}
$$

同理：

$$
s_-\geq0
$$

写成：

$$
\boxed{
-s_-\leq0
}
$$

---

# 35. 为什么是共享 Slack？

代码不是给每一个未来时刻一个：

$$
s_{+,i}
$$

而是整个预测时域共用一个：

$$
s_+
$$

所有上界都是：

$$
r_iU-s_+
\leq
\theta_{\mathrm{limit}}-f_i
$$

同理整个时域共用：

$$
s_-
$$

因此：

$$
s_+
$$

最终等于“为了让所有未来上界约束可行，至少需要放宽多少”。

而：

$$
s_-
$$

表示下界至少需要放宽多少。

---

# 36. Pitch 约束还进行了数值缩放

代码针对每个：

$$
r_i
$$

计算：

$$
\alpha_i
=
\frac{1}{\|r_i\|_1}
$$

当：

$$
\|r_i\|_1
$$

过小时则使用：

$$
\alpha_i=1
$$

然后把：

$$
r_iU-s_+
\leq
\theta_{\mathrm{limit}}-f_i
$$

整体乘以：

$$
\alpha_i>0
$$

得到：

$$
\boxed{
\alpha_ir_iU
-
\alpha_is_+
\leq
\alpha_i
(
\theta_{\mathrm{limit}}-f_i
)
}
$$

下界同样：

$$
\boxed{
-\alpha_ir_iU
-
\alpha_is_-
\leq
\alpha_i
(
\theta_{\mathrm{limit}}+f_i
)
}
$$

因为两边同时乘同一个正数，所以不会改变可行域。

这个操作只是为了改善 QP 的数值条件。

---

# 37. 最终约束形式

所有约束都可以统一写成：

$$
\boxed{
Az\leq b
}
$$

其中：

$$
z=
\begin{bmatrix}
U\\
s_+\\
s_-
\end{bmatrix}
$$

约束主要包括：

$$
-u_{\mathrm{bound}}
\leq
u_j
\leq
u_{\mathrm{bound}}
$$

以及：

$$
r_iU-s_+
\leq
\theta_{\mathrm{limit}}-f_i
$$

$$
-r_iU-s_-
\leq
\theta_{\mathrm{limit}}+f_i
$$

还有：

$$
s_+\geq0
$$

$$
s_-\geq0
$$

---

# 38. 因此完整 MPC 优化问题为

这份代码实际求解：

$$
\boxed{
\begin{aligned}
\min_{U,s_+,s_-}\quad
&
\sum_{i=1}^{N}e_i^TQe_i
+
e_N^TPe_N
+
\sum_{j=0}^{N-1}Ru_j^2\\
&
+
W(s_++s_-)
+
w_2(s_+^2+s_-^2)
\\[4pt]
\text{s.t.}\quad
&
e_i
=
A^ie_0
+
\sum_{j=0}^{i-1}
A^{i-1-j}Bu_j
\\
&
|u_j|\leq u_{\mathrm{bound}}
\\
&
\theta_i-s_+
\leq
\theta_{\mathrm{limit}}
\\
&
-\theta_i-s_-
\leq
\theta_{\mathrm{limit}}
\\
&
s_+\geq0
\\
&
s_-\geq0
\end{aligned}
}
$$

把状态消掉以后：

$$
\boxed{
\begin{aligned}
\min_z\quad&
\frac12z^THz+g^Tz\\
\text{s.t.}\quad&
Az\leq b
\end{aligned}
}
$$

其中：

$$
\boxed{
z=
\begin{bmatrix}
U\\
s_+\\
s_-
\end{bmatrix}
}
$$

$$
\boxed{
H=
\begin{bmatrix}
2(\Theta^T\mathcal Q\Theta+RI_N)&0&0\\
0&2w_2&0\\
0&0&2w_2
\end{bmatrix}
}
$$

$$
\boxed{
g=
\begin{bmatrix}
2\Theta^T\mathcal Q\Psi e_0\\
W\\
W
\end{bmatrix}
}
$$

而：

$$
\boxed{
\mathcal Q
=
\operatorname{diag}
(
Q,Q,\cdots,Q,Q+P
)
}
$$

---

# 39. QP 求出来的是什么？

求解器最终得到：

$$
z^*
=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*\\
\vdots\\
u_{N-1}^*\\
s_+^*\\
s_-^*
\end{bmatrix}
$$

但 MPC 不会把未来所有控制量一次执行完。

实际只执行：

$$
\boxed{
u_{\mathrm{applied}}
=
u_0^*
}
$$

然后下一个控制周期重新读取新的：

$$
e_0
$$

重新计算：

$$
\Psi e_0
$$

重新更新：

$$
g
$$

和 pitch 约束：

$$
b
$$

再重新求解 QP。

这就是滚动时域控制。

---

# 40. 模型总转矩如何转换成两个轮子的转矩？

MPC 得到的是：

$$
u_{\mathrm{applied}}
$$

它表示两轮总驱动转矩。

代码最后转换为单轮 effort：

$$
\boxed{
\tau_{\mathrm{each}}
=
-\frac12u_{\mathrm{applied}}
}
$$

因此：

$$
\tau_L
=
-\frac12u_{\mathrm{applied}}
$$

$$
\tau_R
=
-\frac12u_{\mathrm{applied}}
$$

负号来自 Gazebo 中轮 effort 与模型输入方向的符号约定不同。

---

# 41. Warm Start

假设这一周期求出了：

$$
U_k^*
=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*\\
\vdots\\
u_{N-1}^*
\end{bmatrix}
$$

下一周期不会从全零开始猜测。

而是把旧解向前移动：

$$
\boxed{
U_{\mathrm{warm}}
=
\begin{bmatrix}
u_1^*\\
u_2^*\\
u_3^*\\
\vdots\\
u_{N-1}^*\\
u_{N-1}^*
\end{bmatrix}
}
$$

因为相邻控制周期之间状态变化通常很小。

所以旧规划的后半部分往往很接近新规划。

这可以减少 active-set QP 的求解时间。

---

# 42. 无约束情况下 MPC 会变成什么？

暂时忽略：

$$
s_+
$$

$$
s_-
$$

以及所有约束。

目标：

$$
J_U
=
\frac12U^TH_UU
+
g_U^TU
$$

最优点满足：

$$
\frac{\partial J_U}{\partial U}
=
H_UU+g_U
=
0
$$

因此：

$$
U^*
=
-H_U^{-1}g_U
$$

代入：

$$
g_U
=
2\Theta^T\mathcal Q\Psi e_0
$$

得到：

$$
\boxed{
U^*
=
-H_U^{-1}
2\Theta^T\mathcal Q\Psi e_0
}
$$

又因为：

$$
H_U
=
2
(
\Theta^T\mathcal Q\Theta+RI_N
)
$$

所以也可以写成：

$$
\boxed{
U^*
=
-
\left(
\Theta^T\mathcal Q\Theta+RI_N
\right)^{-1}
\Theta^T\mathcal Q\Psi e_0
}
$$

因此：

$$
U^*
=
K_Ue_0
$$

其中：

$$
K_U
=
-
\left(
\Theta^T\mathcal Q\Theta+RI_N
\right)^{-1}
\Theta^T\mathcal Q\Psi
$$

---

# 43. 只取无约束 MPC 的第一步

定义：

$$
S=
\begin{bmatrix}
1&0&0&\cdots&0
\end{bmatrix}
$$

那么：

$$
u_0^*
=
SU^*
$$

因此：

$$
u_0^*
=
-
S
\left(
\Theta^T\mathcal Q\Theta+RI_N
\right)^{-1}
\Theta^T\mathcal Q\Psi e_0
$$

定义：

$$
\boxed{
K_{\mathrm{first}}
=
-
S
\left(
\Theta^T\mathcal Q\Theta+RI_N
\right)^{-1}
\Theta^T\mathcal Q\Psi
}
$$

于是：

$$
\boxed{
u_0^*
=
K_{\mathrm{first}}e_0
}
$$

这就是代码里面：

$$
\text{first\_move\_gain}
$$

的数学来源。

---

# 44. MPC 与 LQR 为什么这么像？

无约束情况下：

$$
u_0
=
K_{\mathrm{first}}e_0
$$

而 LQR：

$$
u_{\mathrm{LQR}}
=
-K_{\mathrm{LQR}}e_0
$$

二者本质上都是：

$$
\boxed{
u=Ke
}
$$

形式的状态反馈。

MPC 真正明显区别于普通 LQR 的地方，是它可以直接把：

$$
|u|\leq u_{\max}
$$

$$
|\theta|\lesssim\theta_{\max}
$$

这类约束放进优化问题。

---

# 45. 这份代码一个控制周期的完整流程

当前测量得到：

$$
e_0
$$

首先计算自由响应：

$$
\boxed{
E_{\mathrm{free}}
=
\Psi e_0
}
$$

然后更新 QP 的 gradient：

$$
\boxed{
g_U
=
2\Theta^T\mathcal QE_{\mathrm{free}}
}
$$

也就是：

$$
g_U
=
2\Theta^T\mathcal Q\Psi e_0
$$

然后根据：

$$
E_{\mathrm{free}}
$$

计算每一步 pitch 的：

$$
f_i
$$

从而更新 pitch 约束右端。

接着求：

$$
\boxed{
\begin{aligned}
\min_z\quad&
\frac12z^THz+g^Tz\\
\text{s.t.}\quad&
Az\leq b
\end{aligned}
}
$$

得到：

$$
z^*
$$

取第一项：

$$
u_0^*
$$

然后：

$$
\boxed{
\tau_L=\tau_R=-\frac12u_0^*
}
$$

机器人运行一个周期后重新测量状态，再重新求解。

---

# 46. 整个算法最终可以压缩成这一条数学链

离散模型：

$$
\boxed{
e_{k+1}=Ae_k+Bu_k
}
$$

↓

多步预测：

$$
\boxed{
E=\Psi e_0+\Theta U
}
$$

↓

代价函数：

$$
\boxed{
J=
E^T\mathcal QE
+
RU^TU
+
W(s_++s_-)
+
w_2(s_+^2+s_-^2)
}
$$

↓

代入预测模型：

$$
J
=
(\Psi e_0+\Theta U)^T
\mathcal Q
(\Psi e_0+\Theta U)
+
RU^TU
+
J_s
$$

↓

去掉常数项：

$$
\boxed{
J
=
U^T
(
\Theta^T\mathcal Q\Theta+RI
)
U
+
2e_0^T\Psi^T\mathcal Q\Theta U
+
J_s
}
$$

↓

定义：

$$
\boxed{
z=
\begin{bmatrix}
U\\
s_+\\
s_-
\end{bmatrix}
}
$$

↓

得到 QP：

$$
\boxed{
\begin{aligned}
\min_z\quad&
\frac12z^THz+g^Tz\\
\text{s.t.}\quad&
Az\leq b
\end{aligned}
}
$$

↓

QP 求解：

$$
z^*
=
\begin{bmatrix}
u_0^*\\
u_1^*\\
\vdots\\
u_{N-1}^*\\
s_+^*\\
s_-^*
\end{bmatrix}
$$

↓

只执行：

$$
\boxed{
u_{\mathrm{applied}}=u_0^*
}
$$

↓

转换为两个轮子：

$$
\boxed{
\tau_L=\tau_R=-\frac12u_0^*
}
$$

↓

下一控制周期重新求解。

这就是这份 LinearMpc 代码从动力学模型到实际轮子力矩输出的完整数学过程。