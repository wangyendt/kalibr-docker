# Jacobian Cheatsheet：residual 对 design variable 总表

这份 cheatsheet 只列 **几何 Jacobian**：

$$
\mathbf J_i
=
\frac{\partial\mathbf e}{\partial\delta\boldsymbol\theta_i}.
$$

它默认：

1. 使用 Kalibr expression tangent。
2. 不包含 information matrix whitening。
3. 不包含 robust weight。
4. 不包含 LM damping。
5. 不包含 design-variable scaling，cam-imu 主路径里通常为 $1$。

真正进入线性系统时，第 3 章和第 11 章给出的规则是：

$$
\bar{\mathbf e}
=
\sqrt{w}\,\mathbf L^\top\mathbf e,
\qquad
\bar{\mathbf J}_i
=
s_i\sqrt{w}\,\mathbf L^\top\mathbf J_i.
$$

## 1. 公共符号

Camera residual：

$$
\mathbf e^\pi
=
\mathbf y-\hat{\mathbf y},
\qquad
\hat{\mathbf y}
=
\boldsymbol\pi(\mathbf p_c;\boldsymbol\eta).
$$

定义：

$$
\mathbf A_T
\triangleq
\frac{\partial\mathbf e^\pi}
{\partial\delta\boldsymbol\xi_{T_{c_nw},K}}
\in\mathbb R^{2\times6}.
$$

它包含 projection、residual 方向和 transform action 链路。第 4 章给出完整推导。

Camera transform：

$$
\mathbf T_{c_nw}
=
\mathbf T_{c_nb}\mathbf T_{bw},
\qquad
\mathbf T_{bw}=\mathbf T_{wb}^{-1}.
$$

Pose spline：

$$
\mathbf T_{wb}(t)=F(\mathbf s(t)),
\qquad
\mathbf J_{\mathbf T,\mathbf c_j}
=
\frac{\partial\delta\boldsymbol\xi_{T_{wb},K}}
{\partial\delta\mathbf c_j}.
$$

Accelerometer intermediate:

$$
\mathbf u_b
=
\mathbf R_{bw}(\mathbf a_w-\mathbf g_w)
+
\boldsymbol\alpha_b\times\mathbf r_b
+
\boldsymbol\omega_b\times
(\boldsymbol\omega_b\times\mathbf r_b).
$$

Lever-arm matrix:

$$
\mathbf A_r
\triangleq
[\boldsymbol\alpha_b]_\times
+
[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times.
$$

Bias spline weights:

$$
\mathbf b_g(t)=\sum_\ell\mu_\ell^{(0)}(t)\mathbf d^g_{j+\ell},
\qquad
\mathbf b_a(t)=\sum_\ell\nu_\ell^{(0)}(t)\mathbf d^a_{j+\ell}.
$$

若某控制点不在当前 active window 内，对应 block 为零。

## 2. Camera reprojection residual

Residual:

$$
\mathbf e^\pi_{n,k,\ell}
=
\mathbf y_{n,k,\ell}
-
\hat{\mathbf y}_{n,k,\ell}.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| complete camera 外参 $\mathbf T_{c_nb}$ | $\mathbf A_T$ | $2\times6$ |
| camera chain link $\mathbf T_m$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf P_m)$ | $2\times6$ |
| world-to-body pose $\mathbf T_{bw}$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf T_{c_nb})$ | $2\times6$ |
| body-to-world pose $\mathbf T_{wb}$ | $\mathbf A_T\mathrm{boxTimes}(\mathbf T_{c_nb})[-\mathrm{boxTimes}(\mathbf T_{bw})]$ | $2\times6$ |
| active pose control point $\mathbf c_j$ | $\mathbf J_{\mathbf e^\pi,\mathbf T_{wb}}\mathbf J_{\mathbf T,\mathbf c_j}$ | $2\times6$ |
| camera time shift $\Delta t_n$ | $-\mathbf J_{\pi,\mathbf p_c}\dot{\mathbf p}_{c_n}$ | $2\times1$ |
| camera intrinsics $\boldsymbol\eta_n$ | $-\mathbf J_{\hat{\mathbf y},\boldsymbol\eta_n}$ | $2\times d_\eta$ |
| distortion coefficients $\boldsymbol\kappa_n$ | $-\mathbf J_{\hat{\mathbf y},\boldsymbol\kappa_n}$ | $2\times d_\kappa$ |

其中：

$$
\mathbf P_m=\mathbf T_n\mathbf T_{n-1}\cdots\mathbf T_{m+1}
$$

是 camera chain 中 link $\mathbf T_m$ 左侧的外侧链。

Camera residual 对 gyro/accel bias、IMU 内参、IMU lever arm、gravity 的直接 Jacobian 为零。

## 3. Ordinary gyro residual

Residual:

$$
\mathbf e^\omega_k
=
\mathbf R_{ib}\boldsymbol\omega_b(t_k)
+
\mathbf b_g(t_k)
-
\mathbf z^\omega_k.
$$

令：

$$
\mathbf y_\omega=\mathbf R_{ib}\boldsymbol\omega_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}$ | $3\times6$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $[\mathbf y_\omega]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf 0$ | $3\times3$ |
| gyro bias control $\mathbf d^g_{j+\ell}$ | $\mu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| optional IMU time offset $\Delta t_i$ | $\mathbf R_{ib}\dot{\boldsymbol\omega}_b(t_k)+\dot{\mathbf b}_g(t_k)$ | $3\times1$ |

Ordinary gyro residual 对 camera 参数、accel bias、gravity、accelerometer scale/misalignment 的直接 Jacobian 为零。

## 4. Ordinary accelerometer residual

Residual:

$$
\mathbf e^a_k
=
\mathbf R_{ib}\mathbf u_b(t_k)
+
\mathbf b_a(t_k)
-
\mathbf z^a_k.
$$

令：

$$
\mathbf y_a=\mathbf R_{ib}\mathbf u_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| gravity $\mathbf g_w$ | $-\mathbf R_{ib}\mathbf R_{bw}$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $[\mathbf y_a]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| accel bias control $\mathbf d^a_{j+\ell}$ | $\nu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| optional IMU time offset $\Delta t_i$ | $\mathbf R_{ib}\dot{\mathbf u}_b(t_k)+\dot{\mathbf b}_a(t_k)$ | $3\times1$ |

Ordinary accelerometer residual 对 camera 参数、gyro bias、gyro scale/misalignment 的直接 Jacobian 为零。

## 5. Accelerometer scale/misalignment residual

Residual:

$$
\mathbf e^a_k
=
\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf u_b
+
\mathbf b_a
-
\mathbf z^a.
$$

令：

$$
\mathbf x_i=\mathbf R_{ib}\mathbf u_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| gravity $\mathbf g_w$ | $-\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf R_{bw}$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_{\mathrm{acc}}[\mathbf x_i]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| accel bias control $\mathbf d^a_{j+\ell}$ | $\nu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| accel matrix $\operatorname{vec}(\mathbf M_{\mathrm{acc}})$ | $[x_{i,1}\mathbf I_3\ x_{i,2}\mathbf I_3\ x_{i,3}\mathbf I_3]$ | $3\times9$ before mask |

实际 matrix 列数由 `MatrixBasicDv` mask 决定。

## 6. Extended gyro residual

Residual:

$$
\mathbf e^\omega_k
=
\mathbf M_g\boldsymbol\omega_g
+
\mathbf A_g\mathbf a_g
+
\mathbf b_g
-
\mathbf z^\omega,
$$

其中：

$$
\boldsymbol\omega_g=\mathbf R_{gi}\mathbf R_{ib}\boldsymbol\omega_b,
\qquad
\mathbf a_g=\mathbf R_{gi}\mathbf R_{ib}\mathbf u_b.
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| active pose control point $\mathbf c_j$ | $\mathbf M_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\boldsymbol\omega_b,\mathbf c_j}+\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf J_{\mathbf u_b,\mathbf c_j}$ | $3\times6$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_g\mathbf R_{gi}[\mathbf R_{ib}\boldsymbol\omega_b]_\times+\mathbf A_g\mathbf R_{gi}[\mathbf R_{ib}\mathbf u_b]_\times$ | $3\times3$ |
| gyro sensing rotation $\mathbf R_{gi}$ | $\mathbf M_g[\boldsymbol\omega_g]_\times+\mathbf A_g[\mathbf a_g]_\times$ | $3\times3$ |
| IMU lever arm $\mathbf r_b$ | $\mathbf A_g\mathbf R_{gi}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| gyro bias control $\mathbf d^g_{j+\ell}$ | $\mu_\ell^{(0)}(t_k)\mathbf I_3$ | $3\times3$ |
| gyro matrix $\operatorname{vec}(\mathbf M_g)$ | $[\omega_{g,1}\mathbf I_3\ \omega_{g,2}\mathbf I_3\ \omega_{g,3}\mathbf I_3]$ | $3\times9$ before mask |
| accel sensitivity $\operatorname{vec}(\mathbf A_g)$ | $[a_{g,1}\mathbf I_3\ a_{g,2}\mathbf I_3\ a_{g,3}\mathbf I_3]$ | $3\times9$ |

## 7. Accelerometer size-effect residual

Residual:

$$
\mathbf e^a
=
\mathbf M_{\mathrm{acc}}
\left[
\mathbf R_{ib}\mathbf h_b
+
\sum_{j\in\{x,y,z\}}
\mathbf I_j\mathbf R_{ib}\boldsymbol\ell_j
\right]
+
\mathbf b_a
-
\mathbf z^a,
$$

其中：

$$
\mathbf h_b=\mathbf R_{bw}(\mathbf a_w-\mathbf g_w),
$$

$$
\mathbf r_j^b=\mathbf r_b+\mathbf R_{ib}^{-1}\mathbf r_j^i,
\qquad
\boldsymbol\ell_j
=
\boldsymbol\alpha_b\times\mathbf r_j^b
+
\boldsymbol\omega_b\times(\boldsymbol\omega_b\times\mathbf r_j^b).
$$

| design variable | Jacobian | 维度 |
|---|---|---:|
| axis offset $\mathbf r_j^i$ | $\mathbf M_{\mathrm{acc}}\mathbf I_j\mathbf R_{ib}\mathbf A_r\mathbf R_{ib}^{-1}$ | $3\times3$ |
| base lever arm $\mathbf r_b$ | $\mathbf M_{\mathrm{acc}}\mathbf R_{ib}\mathbf A_r$ | $3\times3$ |
| IMU 外参旋转 $\mathbf R_{ib}$ | $\mathbf M_{\mathrm{acc}}\left([\mathbf R_{ib}\mathbf h_b]_\times+\sum_j\mathbf I_j\left([\mathbf R_{ib}\boldsymbol\ell_j]_\times-\mathbf R_{ib}\mathbf A_r\mathbf R_{ib}^{-1}[\mathbf r_j^i]_\times\right)\right)$ | $3\times3$ |

其他 pose、gravity、bias、matrix 分支沿用第 5 节，只是把 $\mathbf u_b$ 换成 size-effect 的 bracket 内部表达。

## 8. Bias motion prior

Bias motion prior 不是普通 finite-dimensional measurement residual。它的代价是：

$$
E_{b_g}
=
\mathbf d_g^\top\mathbf Q_g\mathbf d_g,
\qquad
E_{b_a}
=
\mathbf d_a^\top\mathbf Q_a\mathbf d_a.
$$

等价 residual/Jacobian 读法：

$$
\mathbf Q=\mathbf S^\top\mathbf S,
\qquad
\mathbf e=\mathbf S\mathbf d,
\qquad
\mathbf J=\mathbf S.
$$

Kalibr 实际装配：

$$
\boxed{
\mathbf H\mathrel{+}=\mathbf Q,
\qquad
\mathbf b_{\mathrm{rhs}}\mathrel{-}=\mathbf Q\mathbf d.
}
$$

## 9. Pose motion prior

Pose motion prior 同样是二次型：

$$
E_{\mathrm{pose}}
=
\mathbf c_s^\top\mathbf Q_s\mathbf c_s.
$$

其中 $\mathbf c_s$ 是 pose spline 控制点堆叠向量。若写成虚拟 residual：

$$
\mathbf Q_s=\mathbf S_s^\top\mathbf S_s,
\qquad
\mathbf e^m=\mathbf S_s\mathbf c_s,
\qquad
\mathbf J^m=\mathbf S_s.
$$

Kalibr 实际装配：

$$
\boxed{
\mathbf H\mathrel{+}=\mathbf Q_s,
\qquad
\mathbf b_{\mathrm{rhs}}\mathrel{-}=\mathbf Q_s\mathbf c_s.
}
$$

## 10. 零 block 总览

| residual | 典型零 Jacobian |
|---|---|
| camera reprojection | gyro/accel bias、IMU scale/misalignment、IMU lever arm、gravity |
| ordinary gyro | camera intrinsics/extrinsics/time shift、accel bias、gravity、lever arm |
| ordinary accelerometer | camera intrinsics/extrinsics/time shift、gyro bias、gyro scale/misalignment |
| bias motion prior | pose、camera、IMU extrinsics、gravity、measurement noise 参数 |
| pose motion prior | camera intrinsics、bias、IMU intrinsics、gravity |

“零”表示该 residual 的 forward expression 不直接依赖这个变量。通过优化耦合，一个变量当然仍然可能间接影响其他 residual 的最优解。
