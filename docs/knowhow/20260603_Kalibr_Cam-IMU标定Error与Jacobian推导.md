# Kalibr Cam-IMU 标定 Error 与 Jacobian 推导

## 结论先行

Kalibr 的 cam-imu 标定不是在某个单一 C++ 残差类里手写完整 Jacobian，而是由 Python 组装残差表达式，再由 `aslam_backend` 的 expression graph 逐节点链式求导。核心链路是：

- 问题搭建：`aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py`
- 相机与 IMU 残差创建：`aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py`
- IMU 三维欧式残差：`aslam_offline_calibration/kalibr/src/EuclideanError.cpp`
- 视觉重投影残差：`aslam_cv/aslam_cv_error_terms/include/aslam/backend/implementation/ReprojectionError.hpp`
- 表达式节点 Jacobian：`aslam_optimizer/aslam_backend_expressions/src/*ExpressionNode.cpp`
- 位姿与 bias B-spline Jacobian：`aslam_nonparametric_estimation/aslam_splines/src/BSplineExpressions.cpp`

方向上有一个非常重要的差异：

- 视觉残差：
  $$
  e_{\text{cam}} = y - \hat y
  $$
- IMU 残差：
  $$
  e_{\text{imu}} = \hat z - z
  $$

所以同样的预测模型，视觉 Jacobian 比“预测减观测”的写法多一个负号；IMU Jacobian 没有这个负号。

## 1. 记号与代码约定

本文使用以下 frame：

- $w$：标定板坐标系。Kalibr 初始化 pose spline 时把 target frame 当作 world。
- $b$：body/reference IMU 坐标系。单 IMU 情况下就是 IMU0。
- $c$：相机坐标系。
- $i$：某个 IMU 坐标系。reference IMU 时 $C_{ib}=I,\ r_b=0$。
- $g$：scale-misalignment 模型里的 gyro 坐标系。

位姿样条是：

$$
T_{wb}(t)=
\begin{bmatrix}
C_{wb}(t) & t_{wb}(t)\\
0 & 1
\end{bmatrix},
$$

它把 body 点变到 world。相机残差里使用：

$$
T_{bw}(t)=T_{wb}(t)^{-1}, \qquad
T_{cw}(t)=T_{cb}\,T_{bw}(t).
$$

代码里 `crossMx(a)` 定义为：

$$
[a]_\times b = a \times b.
$$

### 1.1 Kalibr 的小扰动号约定

Micro Lie Theory 通常写右扰动：

$$
R^+ = R\operatorname{Exp}(\delta), \qquad
\operatorname{Exp}(\delta)\approx I + [\delta]_\times.
$$

但本仓库 `sm_kinematics::axisAngle2R/updateQuat` 使用的是被动旋转号约定，小扰动等价于：

$$
C(\delta)\approx I - [\delta]_\times,\qquad
C^+ = C(\delta)C.
$$

因此代码中最常见的旋转乘点节点

$$
y = Cx
$$

的线性化是：

$$
y^+ = (I-[\delta]_\times)Cx + C\,\delta x
     = y + [y]_\times\delta + C\,\delta x,
$$

也就是：

$$
\frac{\partial (Cx)}{\partial \delta} = [Cx]_\times,\qquad
\frac{\partial (Cx)}{\partial x}=C.
$$

这正对应 `EuclideanExpressionNodeMultiply::evaluateJacobiansImplementation()`：

```cpp
_lhs->evaluateJacobians(outJacobians, crossMx(_C_lhs * _p_rhs));
_rhs->evaluateJacobians(outJacobians, _C_lhs);
```

后面所有公式都按这个代码约定写。若和 Micro Lie Theory 的 $R\operatorname{Exp}(\delta)$ 主动右扰动公式比较，很多旋转 action Jacobian 会差一个符号或一个左右扰动转换。

## 2. 目标函数与加权

每个普通残差项在 backend 中贡献：

$$
E_k = e_k^\top R_k^{-1} e_k.
$$

`ErrorTermFs::setInvR()` 对 $R_k^{-1}$ 求平方根，构造正规方程时使用：

$$
\tilde e_k = L_k^\top e_k,\qquad
\tilde J_k = L_k^\top J_k,\qquad
L_kL_k^\top=R_k^{-1}.
$$

若启用 M-estimator，则再乘：

$$
\sqrt{w_k},\qquad w_k=\rho'(E_k)\ \text{对应代码里的 } getWeight(E_k).
$$

所以优化器实际线性化的是：

$$
\tilde e_k(x+\delta)
\approx
\sqrt{w_k}L_k^\top(e_k + J_k\delta).
$$

IMU 噪声从 YAML 读入后，每条 measurement 的协方差在 `IccImu.loadImuData()` 中设为：

$$
R_g = \sigma_{g,d}^2 I,\qquad
R_a = \sigma_{a,d}^2 I.
$$

创建 IMU 残差时，代码传入：

$$
R_g^{-1}\cdot \frac{1}{s_g},\qquad
R_a^{-1}\cdot \frac{1}{s_a},
$$

其中 $s_g=\texttt{gyroNoiseScale}$，$s_a=\texttt{accelNoiseScale}$。注意代码是乘 $1/s$，不是 $1/s^2$。

## 3. 状态变量

cam-imu 主问题的设计变量包括：

1. 位姿 B-spline 系数：

   $$
   c_j =
   \begin{bmatrix}
   p_j\\
   r_j
   \end{bmatrix}\in\mathbb R^6
   $$

   其中前三维是平移曲线参数，后三维是 rotation-vector 曲线参数。

2. 重力方向或重力向量：

   $$
   g_w
   $$

   `estimateGravityLength=False` 时用 `EuclideanDirection`，只优化方向；否则用 `EuclideanPointDv`，也优化模长。

3. 每个 IMU 的 bias B-spline：

   $$
   b_g(t)=\sum_j B_j(t)\beta^g_j,\qquad
   b_a(t)=\sum_j B_j(t)\beta^a_j.
   $$

4. 相机链外参。第一个相机的 $T_{cb}$ 是 cam-imu 标定主外参：

   $$
   T_{cb}=
   \begin{bmatrix}
   C_{cb}&t_{cb}\\0&1
   \end{bmatrix}.
   $$

   多相机 chain 里的后续相机 baseline 默认可固定，也可按参数打开。

5. 相机到 IMU 的时间偏移：

   $$
   t_{\text{eval}} = t_{\text{image}} + \Delta t_{\text{prior}} + \delta t_c.
   $$

6. 多 IMU / IMU intrinsic 模型中的变量：

   $$
   C_{ib},\ r_b,\ M_a,\ M_g,\ M_{ag},\ C_{gi},\ r_x,r_y,r_z.
   $$

单 IMU 常规 cam-imu 标定里，$C_{ib}=I,\ r_b=0$ 通常固定。

## 4. B-spline 的基本求值与 Jacobian

`BSpline::evalDAndJacobian(t, d)` 的数学形式是：

$$
s^{(d)}(t)=
\begin{bmatrix}
c_{j_0}&c_{j_0+1}&\cdots&c_{j_0+k-1}
\end{bmatrix}
B^\top u^{(d)}(t)
=\sum_{\ell=0}^{k-1}\lambda_\ell^{(d)}(t)c_{j_0+\ell}.
$$

因此对局部第 $\ell$ 个 spline coefficient：

$$
\frac{\partial s^{(d)}(t)}{\partial c_{j_0+\ell}}
=\lambda_\ell^{(d)}(t)I.
$$

这就是 `BSpline::evalDAndJacobian()` 里每个 block 设置成 `Identity * Bt_u[i]` 的原因。

pose spline 的曲线值为：

$$
s(t)=
\begin{bmatrix}
p(t)\\r(t)
\end{bmatrix}.
$$

它再通过 `BSplinePose::curveValueToTransformationAndJacobian()` 转成：

$$
T_{wb}(t)=F(s(t)).
$$

代码中的 pose spline 节点最终都形如：

$$
J_{\text{node},c_j}
=
J_{\text{node},s}\,
\lambda_j^{(d)}(t)I.
$$

### 4.1 位姿相关量

代码定义：

$$
C_{wb}(t)=R(r(t)),
\qquad
a_w(t)=\ddot p(t).
$$

body frame angular velocity 节点使用：

$$
\omega_b(t)
=-C_{wb}(t)^\top S(r(t))\dot r(t).
$$

body frame angular acceleration 节点在代码里对应：

$$
\dot\omega_b(t)
=-C_{wb}(t)^\top S(r(t))\ddot r(t),
$$

并通过 `angularVelocityAndJacobian(p, pdot)` 的同一套 $S(r)$ Jacobian 计算对 $r$ 和 $\dot r/\ddot r$ 的导数。

## 5. Expression graph 的 Jacobian 基元

下面这些是所有 cam-imu 残差 Jacobian 的基础。设进入当前节点的上游链式矩阵为 $A$，则传给子节点的是 $A$ 乘以下局部 Jacobian。

### 5.1 加减法

$$
y=x_1+x_2:
\qquad
J_{x_1}=I,\quad J_{x_2}=I.
$$

$$
y=x_1-x_2:
\qquad
J_{x_1}=I,\quad J_{x_2}=-I.
$$

### 5.2 叉乘

$$
y=a\times b=[a]_\times b.
$$

一阶变化：

$$
\delta y
=\delta a\times b+a\times\delta b
=-[b]_\times\delta a+[a]_\times\delta b.
$$

所以：

$$
J_a=-[b]_\times,\qquad
J_b=[a]_\times.
$$

这对应 `EuclideanExpressionNodeCrossEuclidean`。

### 5.3 旋转乘向量

$$
y=Cx.
$$

按 Kalibr 小扰动约定：

$$
J_C=[Cx]_\times,\qquad
J_x=C.
$$

### 5.4 矩阵乘向量

$$
y=Mx.
$$

若 `vec(M)` 按列展开，则：

$$
\frac{\partial y}{\partial \operatorname{vec}(M)}
=
\begin{bmatrix}
x_1I & x_2I & x_3I
\end{bmatrix}.
$$

代码中 `MatrixBasic` 还会乘 update pattern 矩阵 $B_M$：

$$
J_{\theta_M}
=
\begin{bmatrix}
x_1I & x_2I & x_3I
\end{bmatrix}B_M.
$$

同时：

$$
J_x=M.
$$

### 5.5 齐次变换乘点

令：

$$
p' = Tp,\qquad p,p'\in\mathbb R^4.
$$

`sm::kinematics::boxMinus(p')` 是：

$$
p'^\boxminus =
\begin{bmatrix}
p'_4 I_3 & [p'_{1:3}]_\times\\
0 & 0
\end{bmatrix}\in\mathbb R^{4\times 6}.
$$

所以：

$$
\frac{\partial (Tp)}{\partial \xi_T}=p'^\boxminus,\qquad
\frac{\partial (Tp)}{\partial p}=T.
$$

若 $p$ 是普通三维点且 $p_4=1$，前三行就是：

$$
\delta p'=\delta\rho + [p'_{1:3}]_\times\delta\phi.
$$

### 5.6 变换复合与求逆

对：

$$
T=T_1T_2
$$

代码中的局部 Jacobian 是：

$$
J_{T_1}=I_6,\qquad
J_{T_2}=T_1^\boxtimes,
$$

其中：

$$
T^\boxtimes=
\begin{bmatrix}
C & -[t]_\times C\\
0 & C
\end{bmatrix}.
$$

对：

$$
T^{-1}
$$

代码使用：

$$
J_{T^{-1},T}=-(T^{-1})^\boxtimes.
$$

这些来自 `TransformationExpressionNodeMultiply` 和 `TransformationExpressionNodeInverse`。

## 6. 视觉重投影残差

源码入口是 `IccCamera.addCameraErrorTerms()`。

每帧观测时间：

$$
t_k=t_{\text{image},k}+\Delta t_{\text{prior}}+\delta t_c.
$$

标定板角点在 world/target frame：

$$
P_w=
\begin{bmatrix}
X\\Y\\Z\\1
\end{bmatrix}.
$$

预测相机点：

$$
P_c(t_k)=T_{cw}(t_k)P_w
=T_{cb}T_{bw}(t_k)P_w.
$$

相机投影：

$$
\hat y = \pi(P_c;\theta_{\text{cam}}).
$$

视觉残差方向是：

$$
e_{\text{cam}}=y-\hat y.
$$

代码中角点协方差是：

$$
R_{\text{cam}}=2\sigma_{\text{corner}}^2I_2,
\qquad
R_{\text{cam}}^{-1}=\frac{1}{2\sigma_{\text{corner}}^2}I_2.
$$

### 6.1 对相机点的 Jacobian

相机模型提供：

$$
J_\pi=\frac{\partial \pi(P_c)}{\partial P_c}\in\mathbb R^{2\times 4}.
$$

因为残差是 $y-\hat y$：

$$
J_{e,P_c}=-J_\pi.
$$

这就是 `ReprojectionError::evaluateJacobiansImplementation()` 中：

```cpp
_point.evaluateJacobians(_jacobians, -J);
```

### 6.2 对 $T_{cw}$ 的 Jacobian

由齐次变换乘点：

$$
\frac{\partial P_c}{\partial \xi_{cw}}
=P_c^\boxminus.
$$

所以：

$$
J_{e,\xi_{cw}}
=
-J_\pi P_c^\boxminus.
$$

### 6.3 对相机外参 $T_{cb}$ 的 Jacobian

令：

$$
P_b=T_{bw}P_w,\qquad
P_c=T_{cb}P_b.
$$

若把 $T_{cb}$ 拆成 `RotationQuaternionDv + EuclideanPointDv`，即：

$$
p_c=C_{cb}p_b+t_{cb},
$$

则：

$$
\frac{\partial p_c}{\partial \delta t_{cb}}=I,
\qquad
\frac{\partial p_c}{\partial \delta\phi_{cb}}
=[C_{cb}p_b]_\times.
$$

因此：

$$
J_{e,t_{cb}}=-J_{\pi,3}I,
\qquad
J_{e,\phi_{cb}}=-J_{\pi,3}[C_{cb}p_b]_\times.
$$

其中 $J_{\pi,3}$ 是 $J_\pi$ 对前三个 Euclidean 坐标的部分。

若按 transformation expression 的 6 维扰动写，则：

$$
J_{e,T_{cb}}
=-J_\pi P_c^\boxminus.
$$

`TransformationBasic` 会再把这 6 维扰动拆回 rotation DV 和 translation DV；拆分后正是上面的两个式子。

### 6.4 对 pose spline 系数的 Jacobian

视觉点链路是：

$$
P_c
=
T_{cb}\,T_{wb}(t_k)^{-1}P_w.
$$

设：

$$
J_{T_{wb},c_j}
=
\frac{\partial T_{wb}(t_k)}{\partial c_j}
\in\mathbb R^{6\times 6}
$$

是 `BSplineTransformationExpressionNode` 从 spline 得到的局部块。按 expression graph：

$$
\frac{\partial T_{bw}}{\partial c_j}
=
-T_{bw}^\boxtimes J_{T_{wb},c_j},
$$

再过复合：

$$
\frac{\partial T_{cw}}{\partial c_j}
=
T_{cb}^\boxtimes
\left(-T_{bw}^\boxtimes J_{T_{wb},c_j}\right).
$$

最终：

$$
J_{e,c_j}
=
-J_\pi P_c^\boxminus
T_{cb}^\boxtimes
\left(-T_{bw}^\boxtimes J_{T_{wb},c_j}\right).
$$

这个公式就是代码中以下节点连续链式相乘的结果：

```text
ReprojectionError
  -> HomogeneousExpressionNodeMultiply(T_cw, P_w)
  -> TransformationExpressionNodeMultiply(T_cb, T_bw)
  -> TransformationExpressionNodeInverse(T_wb)
  -> TransformationTimeOffsetExpressionNode / BSplineTransformationExpressionNode
```

### 6.5 对时间偏移 $\delta t_c$ 的 Jacobian

时间偏移只通过：

$$
T_{wb}(t_k)
$$

进入视觉残差。`TransformationTimeOffsetExpressionNode` 中先计算：

$$
s(t)=\sum_j\lambda_j(t)c_j,\qquad
\dot s(t)=\sum_j\dot\lambda_j(t)c_j.
$$

再由：

$$
T_{wb}(t)=F(s(t))
$$

得到：

$$
J_{T_{wb},t}
=
\frac{\partial F}{\partial s}\dot s(t)
\in\mathbb R^{6\times 1}.
$$

因为：

$$
\frac{\partial t_k}{\partial \delta t_c}=1,
$$

所以视觉时间偏移 Jacobian 是：

$$
J_{e,\delta t_c}
=
-J_\pi P_c^\boxminus
T_{cb}^\boxtimes
\left(-T_{bw}^\boxtimes J_{T_{wb},t}\right).
$$

这就是 temporal calibration 的核心 Jacobian。它说明时间偏移可观测性依赖于位姿曲线的时间导数；如果相机看到的运动太慢，$\dot s(t)$ 小，时间偏移就弱可观测。

### 6.6 对相机内参的 Jacobian

`ReprojectionError` 支持相机内参/畸变/shutter 的设计变量，公式是：

$$
J_{e,\theta_{\text{proj}}}
=
-\frac{\partial \pi(P_c)}{\partial \theta_{\text{proj}}},
\qquad
J_{e,\theta_{\text{dist}}}
=
-\frac{\partial \pi(P_c)}{\partial \theta_{\text{dist}}}.
$$

不过 cam-imu 主流程通常读入已有 camchain，并不把 camera geometry design variable 加到这个优化问题里；所以这个接口存在，但默认不优化相机内参。

## 7. 陀螺仪残差

源码入口是 `IccImu.addGyroscopeErrorTerms()`。

IMU measurement 时间：

$$
t_k=t_{\text{imu},k}+\Delta t_i.
$$

对普通 calibrated IMU，代码预测量为：

$$
\hat\omega_i(t_k)=C_{ib}\omega_b(t_k)+b_g(t_k).
$$

残差方向：

$$
e_g=\hat\omega_i-z_g.
$$

其中 $z_g$ 是 IMU 文件里的 gyro measurement。

### 7.1 对各物理量的 Jacobian

令：

$$
x_\omega=C_{ib}\omega_b.
$$

则：

$$
\frac{\partial e_g}{\partial \omega_b}=C_{ib},
\qquad
\frac{\partial e_g}{\partial b_g}=I.
$$

对 IMU-to-body rotation $C_{ib}$：

$$
\frac{\partial e_g}{\partial \delta\phi_{ib}}
=[C_{ib}\omega_b]_\times.
$$

对 gyro bias spline 局部系数：

$$
\frac{\partial e_g}{\partial \beta^g_j}
=
B_j(t_k)I.
$$

对 pose spline 系数：

$$
J_{e_g,c_j}
=
C_{ib}\,
J_{\omega_b,c_j},
$$

其中 $J_{\omega_b,c_j}$ 由 `BSplineAngularVelocityBodyFrameExpressionNode` 给出。

### 7.2 $\omega_b$ 对 spline 的 Jacobian

代码中：

$$
\omega_b=-C_{wb}^\top S(r)\dot r.
$$

设：

$$
\omega_{\text{local}}=S(r)\dot r.
$$

令 $C_{bw}=C_{wb}^{\top}$，则：

$$
\omega_b=-C_{bw}\omega_{\text{local}}.
$$

若 $\eta_{bw}$ 表示 `poseSplineDv.orientation(t).inverse()` 这个 rotation expression 传出的局部旋转扰动，则：

$$
\delta\omega_b
=
\underbrace{[\omega_b]_\times\eta_{bw}}_{\text{来自 }C_{bw}}
\;+\;
\underbrace{(-C_{bw})\,\delta\omega_{\text{local}}}_{\text{来自 }r,\dot r}.
$$

展开到 spline coefficient：

$$
J_{\omega_b,c_j}
=
[\omega_b]_\times J_{C_{bw},c_j}
-C_{bw}
J_{\omega_{\text{local}},[r,\dot r]}
\begin{bmatrix}
J_{r,c_j}\\
J_{\dot r,c_j}
\end{bmatrix}.
$$

源码没有把这个公式写在 `GyroscopeError` 里，而是在：

- `BSplinePose::angularVelocityBodyFrameAndJacobian()`
- `RotationVector::angularVelocityAndJacobian()`
- `BSplineAngularVelocityBodyFrameExpressionNode::evaluateJacobiansImplementation()`

中组装。

## 8. 加速度计残差

源码入口是 `IccImu.addAccelerometerErrorTerms()`。

定义：

$$
s_w(t)=a_w(t)-g_w,
\qquad
u_b(t)=C_{bw}(t)s_w(t).
$$

若考虑 IMU lever arm $r_b$，body frame 下 IMU 原点的刚体加速度补偿项是：

$$
\ell_b(t)
=
\dot\omega_b(t)\times r_b
+\omega_b(t)\times(\omega_b(t)\times r_b).
$$

普通 calibrated IMU 的预测量：

$$
\hat a_i
=
C_{ib}\left(u_b+\ell_b\right)+b_a(t).
$$

残差方向：

$$
e_a=\hat a_i-z_a.
$$

reference IMU 常见情况 $C_{ib}=I,\ r_b=0$，此时简化为：

$$
\hat a_b=C_{bw}(a_w-g_w)+b_a.
$$

### 8.1 对基础量的 Jacobian

令：

$$
a_b=u_b+\ell_b.
$$

则：

$$
\frac{\partial e_a}{\partial a_w}
=C_{ib}C_{bw},
\qquad
\frac{\partial e_a}{\partial g_w}
=-C_{ib}C_{bw},
\qquad
\frac{\partial e_a}{\partial b_a}=I.
$$

对 $C_{ib}$：

$$
\frac{\partial e_a}{\partial \delta\phi_{ib}}
=[C_{ib}a_b]_\times.
$$

对 accel bias spline 局部系数：

$$
\frac{\partial e_a}{\partial \beta^a_j}
=B_j(t_k)I.
$$

### 8.2 lever arm 项的 Jacobian

令：

$$
\ell_b
=
\dot\omega\times r+\omega\times(\omega\times r).
$$

对 $r$：

$$
\frac{\partial \ell_b}{\partial r}
=
[\dot\omega]_\times+[\omega]_\times[\omega]_\times.
$$

对 $\dot\omega$：

$$
\frac{\partial \ell_b}{\partial \dot\omega}
=
-[r]_\times.
$$

对 $\omega$：

$$
\delta\left(\omega\times(\omega\times r)\right)
=
\delta\omega\times(\omega\times r)
+\omega\times(\delta\omega\times r),
$$

所以：

$$
\frac{\partial \ell_b}{\partial \omega}
=
-[\omega\times r]_\times
-[\omega]_\times[r]_\times.
$$

等价地，也可以写成：

$$
\frac{\partial \ell_b}{\partial \omega}
=
(\omega^\top r)I+\omega r^\top-2r\omega^\top.
$$

两种形式相同。

因此：

$$
J_{e_a,r_b}
=
C_{ib}
\left(
[\dot\omega_b]_\times+[\omega_b]_\times[\omega_b]_\times
\right).
$$

### 8.3 对 pose spline 系数的 Jacobian

把加速度残差写成：

$$
e_a
=
C_{ib}
\left(
C_{bw}(a_w-g_w)
+\ell_b(\omega_b,\dot\omega_b,r_b)
\right)
+b_a-z_a.
$$

对局部 pose coefficient $c_j$：

$$
\begin{aligned}
J_{e_a,c_j}
=C_{ib}\Big(
&[C_{bw}(a_w-g_w)]_\times J_{C_{bw},c_j}
+C_{bw}J_{a_w,c_j}\\
&+J_{\ell,\omega}J_{\omega_b,c_j}
+J_{\ell,\dot\omega}J_{\dot\omega_b,c_j}
\Big).
\end{aligned}
$$

其中：

$$
J_{\ell,\omega}
=
-[\omega_b\times r_b]_\times
-[\omega_b]_\times[r_b]_\times,
\qquad
J_{\ell,\dot\omega}
=
-[r_b]_\times.
$$

各项来源：

- $J_{C_{bw},c_j}$：`poseSplineDv.orientation(t).inverse()`
- $J_{a_w,c_j}$：`poseSplineDv.linearAcceleration(t)`
- $J_{\omega_b,c_j}$：`poseSplineDv.angularVelocityBodyFrame(t)`
- $J_{\dot\omega_b,c_j}$：`poseSplineDv.angularAccelerationBodyFrame(t)`

代码通过 expression graph 自动完成这个链式乘法；`AccelerometerError.cpp` 只负责把表达式拼出来。

## 9. Scale-Misalignment IMU 模型

`IccScaledMisalignedImu` 会额外估计 IMU intrinsic。加速度预测变为：

$$
\hat a
=
M_a\,C_{ib}a_b+b_a.
$$

陀螺预测变为：

$$
\hat\omega
=
M_g(C_{gi}C_{ib}\omega_b)
+M_{ag}(C_{gi}C_{ib}a_b)
+b_g.
$$

这里：

- $M_a$：accelerometer scale/misalignment
- $M_g$：gyro scale/misalignment
- $M_{ag}$：acceleration sensitivity of gyro
- $C_{gi}$：gyro frame relative to IMU frame

### 9.1 对矩阵 intrinsic 的 Jacobian

对任意：

$$
y=Mx
$$

若所有 9 个元素都可更新：

$$
\frac{\partial y}{\partial \operatorname{vec}(M)}
=
\begin{bmatrix}
x_1I & x_2I & x_3I
\end{bmatrix}.
$$

代码用 update pattern 限制可更新元素。例如 $M_a$ 和 $M_g$ 使用下三角 pattern，所以实际 Jacobian 是：

$$
J_{\theta_M}
=
\begin{bmatrix}
x_1I & x_2I & x_3I
\end{bmatrix}B_M.
$$

### 9.2 scaled accel residual

令：

$$
x_a=C_{ib}a_b.
$$

则：

$$
e_a=M_ax_a+b_a-z_a.
$$

主要 Jacobian：

$$
J_{e_a,M_a}
=
\begin{bmatrix}
x_{a1}I & x_{a2}I & x_{a3}I
\end{bmatrix}B_{M_a},
$$

$$
J_{e_a,x_a}=M_a,
\qquad
J_{e_a,\delta\phi_{ib}}
=
M_a[C_{ib}a_b]_\times.
$$

其余对 $a_w,g_w,\omega,\dot\omega,r,b_a,c_j$ 的 Jacobian 都是在普通 accel 公式外侧再乘 $M_a$。

### 9.3 scaled gyro residual

令：

$$
C_{gb}=C_{gi}C_{ib},
\qquad
x_\omega=C_{gb}\omega_b,
\qquad
x_a=C_{gb}a_b.
$$

残差：

$$
e_g=M_gx_\omega+M_{ag}x_a+b_g-z_g.
$$

对矩阵：

$$
J_{e_g,M_g}
=
\begin{bmatrix}
x_{\omega1}I & x_{\omega2}I & x_{\omega3}I
\end{bmatrix}B_{M_g},
$$

$$
J_{e_g,M_{ag}}
=
\begin{bmatrix}
x_{a1}I & x_{a2}I & x_{a3}I
\end{bmatrix}B_{M_{ag}}.
$$

对 $C_{gi}$：

$$
J_{e_g,\delta\phi_{gi}}
=
M_g[C_{gb}\omega_b]_\times
+M_{ag}[C_{gb}a_b]_\times.
$$

对 $\omega_b$：

$$
J_{e_g,\omega_b}
=
M_g C_{gb}
+M_{ag}C_{gb}J_{a_b,\omega_b}.
$$

对 $a_b$：

$$
J_{e_g,a_b}=M_{ag}C_{gb}.
$$

再接到 pose spline：

$$
J_{e_g,c_j}
=
M_gC_{gb}J_{\omega_b,c_j}
+M_{ag}C_{gb}J_{a_b,c_j}.
$$

## 10. Size-effect accelerometer 模型

`IccScaledMisalignedSizeEffectImu` 给三个 accelerometer sensing axes 设置不同 lever arm：

$$
r_x^b=r_b+C_{ib}^{-1}r_x^i,\quad
r_y^b=r_b+C_{ib}^{-1}r_y^i,\quad
r_z^b=r_b+C_{ib}^{-1}r_z^i.
$$

预测量：

$$
\hat a
=M_a\Big(
C_{ib}C_{bw}(a_w-g_w)
+I_x C_{ib}\ell(r_x^b)
+I_y C_{ib}\ell(r_y^b)
+I_z C_{ib}\ell(r_z^b)
\Big)+b_a,
$$

其中：

$$
\ell(r)=\dot\omega_b\times r+\omega_b\times(\omega_b\times r),
$$

并且：

$$
I_x=\operatorname{diag}(1,0,0),\quad
I_y=\operatorname{diag}(0,1,0),\quad
I_z=\operatorname{diag}(0,0,1).
$$

对每个 axis lever arm 的 Jacobian：

$$
\frac{\partial e_a}{\partial r_x^b}
=
M_a I_x C_{ib}
\left(
[\dot\omega_b]_\times+[\omega_b]_\times[\omega_b]_\times
\right),
$$

$r_y^b,r_z^b$ 同理，把 $I_x$ 换成 $I_y,I_z$。

如果对 IMU frame 中的 $r_x^i$ 求导，还要乘：

$$
\frac{\partial r_x^b}{\partial r_x^i}=C_{ib}^{-1}.
$$

对 $C_{ib}$ 的 Jacobian 比普通模型多两类项：

1. $C_{ib}$ 直接乘 $C_{bw}(a_w-g_w)$ 和 $\ell(r)$ 的旋转 action Jacobian。
2. $r_x^b=r_b+C_{ib}^{-1}r_x^i$ 中 $C_{ib}^{-1}$ 对 rotation 的 Jacobian。

代码没有手写这个总式，而是用表达式：

```python
rx_b = r_b + C_i_b.inverse() * rx_i
...
Ix * (C_i_b * (... rx_b ...))
```

让 `RotationExpressionNodeInverse`、`EuclideanExpressionNodeMultiply`、`CrossEuclidean` 自动链式求导。

## 11. Bias motion regularization

若 `doBiasMotionError=True`，每个 IMU 会添加：

```python
gyroBiasMotionErr = asp.BSplineEuclideanMotionError(self.gyroBiasDv, Wgyro, 1)
accelBiasMotionErr = asp.BSplineEuclideanMotionError(self.accelBiasDv, Waccel, 1)
```

权重：

$$
W_g=\frac{1}{\sigma_{bg}^2}I,\qquad
W_a=\frac{1}{\sigma_{ba}^2}I.
$$

该项不是普通 measurement residual，而是 spline coefficient 的二次型：

$$
E_{\text{bias}}
=
c^\top Q c,
$$

其中 $Q$ 是：

$$
Q=\int B^{(1)}(t)^\top W B^{(1)}(t)\,dt.
$$

源码 `BSplineMotionError::buildHessianImplementation()` 直接向 Hessian 加：

$$
H\leftarrow H+Q,
\qquad
b\leftarrow b-Qc.
$$

因此这里没有常规的 $e$ 和 $J$，但等价于对 bias 一阶导做白化平滑先验。

## 12. Pose motion regularization

若 `doPoseMotionError=True`，会调用：

```python
asp.addMotionErrorTerms(problem, self.poseDv, W, errorOrder)
```

这会按 segment 添加 `MarginalizationPriorErrorTerm`，本质仍是 spline coefficient 的积分二次型：

$$
E_{\text{pose}}
=
\int
\left\|
\frac{d^m}{dt^m}s(t)
\right\|_W^2dt.
$$

默认 cam-imu 标定常不开启这一项；开启后它主要抑制轨迹 spline 过度弯曲。

## 13. Error 与 Jacobian 的总装配

把所有 residual 堆叠：

$$
e(x)=
\begin{bmatrix}
e_{\text{cam},1}\\
\vdots\\
e_{g,1}\\
\vdots\\
e_{a,1}\\
\vdots
\end{bmatrix}.
$$

一次 Gauss-Newton/LM 线性化：

$$
e(x+\delta)\approx e(x)+J\delta.
$$

对每个普通残差：

$$
H_k=J_k^\top R_k^{-1}J_k,\qquad
b_k=-J_k^\top R_k^{-1}e_k.
$$

加 M-estimator 后：

$$
H_k=w_kJ_k^\top R_k^{-1}J_k,\qquad
b_k=-w_kJ_k^\top R_k^{-1}e_k.
$$

加二次型正则项后：

$$
H\leftarrow H+Q,\qquad b\leftarrow b-Qc.
$$

LM 解：

$$
(H+\lambda D)\delta=b.
$$

然后各设计变量按自己的 manifold update 更新：

- EuclideanPoint / Scalar / spline coefficient：直接加。
- RotationQuaternion：`q <- updateQuat(q, delta)`，即按 Kalibr 的小角度约定更新。
- MatrixBasic：按 update pattern 对矩阵元素加。

## 14. 最容易出错的符号点

1. 视觉 residual 是 $y-\hat y$，所以点和外参 Jacobian 前面有负号。

2. IMU residual 是 $\hat z-z$，所以没有视觉那个负号。

3. 本代码的旋转 action Jacobian 是：

   $$
   \frac{\partial Cx}{\partial \delta\phi}=[Cx]_\times
   $$

   不是常见主动右扰动写法里的 $-C[x]_\times$。这是 `sm_kinematics` 小角度号约定导致的。

4. temporal calibration 的 Jacobian 不是一个额外观测模型，而是视觉残差通过 $T_{wb}(t)$ 对时间求导：

   $$
   J_{e,\delta t_c}
   =
   J_{e,T_{wb}}\frac{\partial T_{wb}}{\partial t}.
   $$

5. bias motion error 是二次型正则，不走普通 `evaluateJacobiansImplementation()`。

6. `gyroNoiseScale/accelNoiseScale` 在代码里缩放的是 information matrix 的一次方：$R^{-1}\leftarrow R^{-1}/s$。

## 15. 源码对应表

| 内容 | 源码 |
|---|---|
| cam-imu problem build | `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py` |
| camera residual 创建 | `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py` |
| gyro/accel residual 创建 | `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py` |
| IMU Euclidean residual 方向 | `aslam_offline_calibration/kalibr/src/EuclideanError.cpp` |
| gyro/accel C++ residual 表达式 | `aslam_offline_calibration/kalibr/src/GyroscopeError.cpp`, `aslam_offline_calibration/kalibr/src/AccelerometerError.cpp` |
| reprojection residual 方向 | `aslam_cv/aslam_cv_error_terms/include/aslam/backend/implementation/ReprojectionError.hpp` |
| rotation/vector/cross/matrix expression Jacobian | `aslam_optimizer/aslam_backend_expressions/src/EuclideanExpressionNode.cpp` |
| transformation expression Jacobian | `aslam_optimizer/aslam_backend_expressions/src/TransformationExpressionNode.cpp` |
| homogeneous point transform Jacobian | `aslam_optimizer/aslam_backend_expressions/src/HomogeneousExpressionNode.cpp` |
| rotation quaternion update | `aslam_optimizer/aslam_backend_expressions/src/RotationQuaternion.cpp` |
| B-spline basis Jacobian | `aslam_nonparametric_estimation/bsplines/src/BSpline.cpp` |
| pose spline angular velocity / acceleration Jacobian | `aslam_nonparametric_estimation/bsplines/src/BSplinePose.cpp` |
| pose/time-offset expression | `aslam_nonparametric_estimation/aslam_splines/src/BSplineExpressions.cpp` |
| bias motion quadratic term | `aslam_nonparametric_estimation/aslam_splines/include/aslam/backend/implementation/BSplineMotionError.hpp` |
