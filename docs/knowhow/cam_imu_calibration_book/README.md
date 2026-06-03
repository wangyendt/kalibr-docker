# Kalibr Cam-IMU 标定推导

这组文档把 Kalibr cam-imu 标定按“从问题出发”的方式重写。它不是 API 手册，也不是只列公式的备忘录；它要回答的是：为什么这个标定问题可以被构造成一个非线性最小二乘问题，每个 residual 为什么长成现在这样，Jacobian 又是怎样一步步由链式法则得到的。

理论主线采用 Micro Lie Theory 记号：$\mathbf T\in SE(3)$、$\mathbf R\in SO(3)$、$\mathbf t\in\mathbb R^3$、$\mathrm{Exp}$、$\mathrm{Log}$、$\oplus$、$\ominus$ 和 $\mathrm{Ad}$。Jacobian 推导默认使用 Kalibr 的左侧/global 扰动约定，记作 $\boxplus_K$；源码对照时再映射到 Kalibr 变量名，例如 `T_w_b`、`T_cN_b` 和 expression node 的局部 Jacobian。

## 阅读顺序

| 章节 | 文件 | 状态 | 作用 |
|---|---|---|---|
| 第 0 章 | [00_符号表与约定.md](00_符号表与约定.md) | draft | 固定 Micro Lie Theory 主记号、Kalibr 桥接表、残差方向和 Jacobian 方向 |
| 第 1 章 | [01_问题从哪里来.md](01_问题从哪里来.md) | draft | 从相机、IMU、标定板和连续轨迹解释为什么要这样构建优化问题 |
| 第 2 章 | `02_坐标系与刚体变换.md` | planned | 从点变换推到 camera residual 的几何链路 |
| 第 3 章 | `03_最小二乘与残差方向.md` | planned | 解释 covariance、information matrix、robust kernel 和线性化 |
| 第 4 章 | `04_相机观测模型.md` | planned | 推导 reprojection residual 和 Jacobian |
| 第 5 章 | `05_连续时间轨迹与B样条.md` | planned | 推导 pose spline、速度、加速度和时间偏移 Jacobian |
| 第 6 章 | `06_陀螺仪残差.md` | planned | 推导 gyro residual 和 Jacobian |
| 第 7 章 | `07_加速度计残差.md` | planned | 推导 accelerometer residual 和 Jacobian |
| 第 8 章 | `08_时间偏移与外参.md` | planned | 解释 temporal calibration 和 sensor extrinsics |
| 第 9 章 | `09_bias与motion_prior.md` | planned | 解释 bias spline 和 motion regularization |
| 第 10 章 | `10_扩展IMU模型.md` | planned | 解释 scale-misalignment 和 size-effect IMU |
| 第 11 章 | `11_表达式图与源码对应.md` | planned | 把公式映射到 Kalibr expression graph |
| 附录 A | [appendix_A_SO3幂律与Rodrigues公式.md](appendix_A_SO3幂律与Rodrigues公式.md) | draft | 从 $[\boldsymbol\omega]_\times$ 的幂律推导 Rodrigues 公式，并连接第 0.10 节的 $SE(3)$ 指数映射 |

## 写作约定

每章按同一顺序展开：

1. 先讲直觉：这一章解决什么困惑。
2. 再讲模型：预测量怎么从状态量和观测时刻产生。
3. 再讲 residual：预测量和测量量按什么方向相减。
4. 再讲 Jacobian：先对中间量求导，再链到设计变量。
5. 最后给源码锚点：读代码时应该看哪里。

符号约定以第 0 章为准。后续章节如果必须引入新符号，先在本章局部定义，再回填到第 0 章。
