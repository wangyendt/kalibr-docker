# Kalibr Cam-IMU 标定推导

这组文档把 Kalibr cam-imu 标定按“从问题出发”的方式重写。它不是 API 手册，也不是只列公式的备忘录；它要回答的是：为什么这个标定问题可以被构造成一个非线性最小二乘问题，每个 residual 为什么长成现在这样，Jacobian 又是怎样一步步由链式法则得到的。

理论主线采用 Micro Lie Theory 记号：$\mathbf T\in SE(3)$、$\mathbf R\in SO(3)$、$\mathbf t\in\mathbb R^3$、$\mathrm{Exp}$、$\mathrm{Log}$、$\oplus$、$\ominus$ 和 $\mathrm{Ad}$。Jacobian 推导默认使用 Kalibr 的左侧/global 扰动约定，记作 $\boxplus_K$；源码对照时再映射到 Kalibr 变量名，例如 `T_w_b`、`T_cN_b` 和 expression node 的局部 Jacobian。

## 全书层次

这本书把 cam-imu 标定拆成四层。读的时候先建立问题和符号，再进入优化公共框架，然后逐个 residual 推 Jacobian，最后回到 Kalibr 源码和可复现验证。

| 层次 | 覆盖章节 | 核心问题 |
|---|---|---|
| 地图层 | 第 0-2 章 | 符号、坐标系、状态量、观测链路是什么 |
| 优化公共层 | 第 3 章 | 一个 residual 的 error、weight、Jacobian 如何进入 Gauss-Newton |
| residual / Jacobian 层 | 第 4-10 章 | camera、gyro、accel、time shift、extrinsics、bias、motion prior 和 IMU 扩展模型的具体公式怎么推 |
| 实现与验证层 | 第 11 章、附录 B | Kalibr expression graph 怎么实现这些链式 Jacobian，如何用最小 Python/C++ 程序复现和做 finite-difference check |

第 3 章不负责推完所有具体 Jacobian。它回答的是：**一旦某个 residual 的 Jacobian 推出来，优化器如何用它构造 $H\delta\theta=b$**。具体公式从第 4 章开始逐项展开。LM conditioner、CHOLMOD / QR 等线性求解器后端细节不是第 3 章的阅读前提。

## 阅读顺序

| 章节 | 文件 | 状态 | 作用 |
|---|---|---|---|
| 第 0 章 | [00_符号表与约定.md](00_符号表与约定.md) | draft | 固定 Micro Lie Theory 主记号、Kalibr 桥接表、残差方向和 Jacobian 方向 |
| 第 1 章 | [01_问题从哪里来.md](01_问题从哪里来.md) | draft | 从相机、IMU、标定板和连续轨迹解释为什么要这样构建优化问题 |
| 第 2 章 | [02_坐标系与刚体变换.md](02_坐标系与刚体变换.md) | draft | 从点变换推到 camera residual 的几何链路 |
| 第 3 章 | [03_最小二乘与残差方向.md](03_最小二乘与残差方向.md) | draft | 优化公共层：covariance、information matrix、robust kernel、线性化、$H$ 和 RHS |
| 第 4 章 | `04_相机观测模型.md` | planned | 推导 reprojection residual、projection/distortion Jacobian、$\mathbf p_c$ 对 pose/extrinsic/time 的链式 Jacobian |
| 第 5 章 | `05_连续时间轨迹与B样条.md` | planned | 推导 pose spline、姿态/位置控制点、速度、角速度、加速度和 time derivative 的 Jacobian |
| 第 6 章 | `06_陀螺仪残差.md` | planned | 推导 gyro residual 对 pose spline、gyro bias、IMU 外参/内参的 Jacobian |
| 第 7 章 | `07_加速度计残差.md` | planned | 推导 accelerometer residual 对旋转、线加速度、重力、accel bias、lever arm 和 IMU 参数的 Jacobian |
| 第 8 章 | `08_时间偏移与外参.md` | planned | 汇总 time shift 和 sensor extrinsics 在 camera/IMU residual 中的共同 Jacobian 结构 |
| 第 9 章 | `09_bias与motion_prior.md` | planned | 推导 bias spline、bias motion prior、pose motion prior 和 regularization residual/Jacobian |
| 第 10 章 | `10_扩展IMU模型.md` | planned | 推导 scale、misalignment、size-effect、acceleration sensitivity 等扩展 IMU 参数的 Jacobian |
| 第 11 章 | `11_表达式图与源码对应.md` | planned | 把第 4-10 章公式映射到 Kalibr expression graph、error term、design variable 和源码调用链 |
| 附录 A | [appendix_A_SO3幂律与Rodrigues公式.md](appendix_A_SO3幂律与Rodrigues公式.md) | draft | 从 $[\boldsymbol\omega]_\times$ 的幂律推导 Rodrigues 公式，并连接第 0.10 节的 $SE(3)$ 指数映射 |
| 附录 B | `appendix_B_数值验证与最小复现.md` | planned | 用 Python/C++ 最小实现验证 SO(3)/SE(3)、projection、residual、analytic Jacobian 和 finite difference |

## 写作约定

每章按同一顺序展开：

1. 先讲直觉：这一章解决什么困惑。
2. 再讲模型：预测量怎么从状态量和观测时刻产生。
3. 再讲 residual：预测量和测量量按什么方向相减。
4. 再讲 Jacobian：先对中间量求导，再链到设计变量。
5. 最后给源码锚点：读代码时应该看哪里。

符号约定以第 0 章为准。后续章节如果必须引入新符号，先在本章局部定义，再回填到第 0 章。
