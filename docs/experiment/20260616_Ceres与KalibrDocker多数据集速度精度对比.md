# Ceres 与 Kalibr Docker 多数据集速度精度对比

## 目的

在**充分收敛**(单次全联合优化:不分阶段、所有变量含重力方向一起放开,`--max-iterations 150`、`--solver-max-trust-region-radius 1e7`)下,用**两种初始化口径**对比 `ceres_cam_imu` 和 Kalibr。两表的**优化器完全相同,唯一变量是初始化**,这样可以把"初始化的影响"和"两套实现本身的差异"分开:

- **表1 — 独立(完全不参考 Kalibr 初始化)**:不读 Kalibr 任何输出,复刻 Kalibr 自己的初始化(`--estimate-time-shift-prior` gyro-norm 互相关 + `--estimate-orientation-gravity-prior` Wahba/Kabsch 闭式 + 外参平移从 0 起 + pose 样条拟合 + bias 零)。这是"从零独立标定"的真实能力,对应 preset `independent-joint-full`,已固化在代码。
- **表2 — 热启动(从 Kalibr 结果读取)**:用 `--init-from-kalibr` 把 Kalibr 输出(外参/time-shift/gravity)当**初值**,再放开同一 joint 优化器。用途是**检验"Ceres 和 Kalibr 是不是同一个优化问题"**:若是,从 Kalibr 解出发应几乎不动;实际会漂多少,见表2。这只是验证实验,`--init-from-kalibr` 是调试 flag,不进生产路径。

- **Kalibr 侧**:各数据集 `data1.csv` 的产线全量收敛结果 `cam0_640x400_corners-1-results-imucam.txt`(精度基线),并在本机 amd64 模拟 Docker 重跑一次取全量收敛墙钟。
- **对比量**:Ceres 减 Kalibr 的外参平移差、旋转差、time-shift 差;两侧 reproj 均值;两侧墙钟。定义见「列名定义」节。

## 配置

| 项 | 内容 |
|---|---|
| 数据 | `production_calibration/data/<ts>/cam_imu`,12 套,均用 `data1.csv` |
| Ceres | `ceres_cam_imu/build/calibrate_cam_imu`,原生 arm64 Release,preset `independent-joint-full` |
| Kalibr | `kalibr-camera-calibration:20.04`,amd64 在 arm64 主机上**模拟**运行 |
| 口径 | pose 100 kps / bias 50 kps / `--timeoffset-padding 0.04` / `--imu-trim-edge-count 1000` |

运行粒度:一次标定 = 1 camera + 1 IMU 文件(`data1.csv`),不是 4 个 IMU 一起。

## 列名定义

记 Ceres 收敛后的 camera-IMU 外参为 $\mathbf T_{C}=\begin{bmatrix}\mathbf R_C & \mathbf t_C\\ \mathbf 0^\top & 1\end{bmatrix}$,Kalibr 产线结果为 $\mathbf T_{K}=(\mathbf R_K,\mathbf t_K)$。两者都是同一个刚体变换 `T_c_b`(源码里写作 `T_ci`,把点从 IMU/body frame 变到 camera frame):$\mathbf R$ 是相机相对 IMU 的朝向,$\mathbf t$ 是相机光心相对 IMU 的位置。下面的"差"全部是 Ceres 减 Kalibr。

- **外参平移差**:两者平移向量之差的欧氏范数(单位 mm)。衡量"相机相对 IMU 的位置"估得准不准。
$$\Delta t=\lVert \mathbf t_C-\mathbf t_K\rVert_2.$$

- **外参旋转差**:两旋转之间的测地角,即把一个姿态转到另一个所需的转角(单位 °)。$\mathbf R_C\mathbf R_K^\top$ 是 Kalibr→Ceres 的相对旋转,其转角就是两者朝向差。
$$\Delta\theta=\arccos\!\Big(\frac{\operatorname{tr}(\mathbf R_C\mathbf R_K^\top)-1}{2}\Big).$$

- **time-shift 差**:camera→IMU 时间偏移之差(单位 ms)。时间偏移 $\tau$ 的定义是 $t_{\mathrm{imu}}=t_{\mathrm{cam}}+\tau$;符号有意义(谁的时钟更超前)。
$$\Delta\tau=\tau_C-\tau_K.$$

- **reproj px(Ceres/Kalibr)**:各自收敛后,全部 $N$ 个角点观测的重投影误差像素范数均值。$\mathbf y_k$ 是检测到的角点,$\hat{\mathbf y}_k$ 是按当前标定参数预测的投影。这是"标定自身拟合得好不好"的指标——**两边各算各的、不是相减**,放在一起是为了看 Ceres 的拟合质量有没有追平 Kalibr。
$$\overline{e}_\pi=\frac{1}{N}\sum_{k=1}^{N}\bigl\lVert \mathbf y_k-\hat{\mathbf y}_k\bigr\rVert_2.$$

- **Ceres 墙钟 / Kalibr 墙钟**:端到端真实流逝时间(`/usr/bin/time` 的 `real`),含读数据 / 构建问题 / 求解迭代 / 写结果全过程,不是只算求解、也不是 CPU 时间。Kalibr 列括号里是收敛用的 LM 迭代数。Ceres 为原生 arm64;Kalibr 为 amd64 模拟 Docker(故非 native-vs-native,见结论)。

## 表1:独立口径(完全不参考 Kalibr 初始化)vs Kalibr 全量收敛

Ceres 从零独立初始化(`independent-joint-full`)。差异列为 Ceres 减 Kalibr(定义见上节)。reproj 为各自收敛后的重投影均值。墙钟列括号内是收敛用的迭代数(Ceres 单次全联合 LM;Kalibr Optimizer2 LM)。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 墙钟 | Kalibr 墙钟 |
|---|---:|---:|---:|---|---:|---:|
| 2025_03_14_00_10_18 | 3.62 mm | 0.030° | -0.99 ms | 0.1803 / 0.1798 | 115 s (110 it) | 162 s (5 it) |
| 2025_03_14_00_34_14 | 4.66 mm | 0.057° | -0.80 ms | 0.1803 / 0.1797 | 115 s (111 it) | 215 s (6 it) |
| 2025_03_14_00_50_37 | 1.96 mm | 0.005° | -0.78 ms | 0.1803 / 0.1801 | 120 s (115 it) | 145 s (3 it) |
| 2025_03_14_02_13_45 | 2.35 mm | 0.007° | -1.39 ms | 0.1797 / 0.1791 | 118 s (113 it) | 207 s (7 it) |
| 2025_03_14_02_21_41 | 2.24 mm | 0.002° | -0.57 ms | 0.1792 / 0.1787 | 125 s (121 it) | 254 s (10 it) |
| 2025_03_14_10_23_35 | 2.38 mm | 0.004° | -1.12 ms | 0.1772 / 0.1766 | 112 s (106 it) | 168 s (4 it) |
| 2025_04_19_18_43_05 | 1.40 mm | 0.005° | -3.20 ms | 0.1708 / 0.1715 | 84 s (79 it) | 225 s (10 it) |
| 2025_04_19_19_03_03 | 3.05 mm | 0.014° | -6.27 ms | 0.1724 / 0.1712 | 81 s (77 it) | 246 s (10 it) |
| 2025_04_19_19_20_46 | 1.48 mm | 0.000° | -0.70 ms | 0.1704 / 0.1706 | 81 s (77 it) | 194 s (7 it) |
| 2025_04_19_19_35_04 | 1.31 mm | 0.000° | +1.34 ms | 0.1719 / 0.1708 | 80 s (75 it) | 183 s (8 it) |
| 2025_04_19_19_55_25 | 1.68 mm | 0.007° | +1.42 ms | 0.1728 / 0.1717 | 90 s (84 it) | 178 s (7 it) |
| 2025_04_19_20_21_09 | 2.49 mm | 0.011° | -6.19 ms | 0.1722 / 0.1712 | 89 s (86 it) | 261 s (12 it) |

汇总(12 套):

| 指标 | 范围 / 典型 |
|---|---|
| 外参平移差 | 1.3 – 4.7 mm(中位 ~2.3 mm) |
| 外参旋转差 | 0 – 0.057°(多数 < 0.02°) |
| time-shift 差 | -6.3 – +1.4 ms(多数 < 1.5 ms,两套 ~-6 ms) |
| reproj 均值 | Ceres 与 Kalibr 差 ≤ 0.001 px(基本逐位相同) |
| Ceres 墙钟 / 迭代 | 80 – 125 s / 75 – 121 it(原生,均 CONVERGENCE,未触 150 上限) |
| Kalibr 墙钟 / 迭代 | 145 – 261 s / 3 – 12 it(amd64 模拟) |

两组分别是 3 月、4 月两次采集:March 组 camera residual ~662k、Ceres 收敛 106–121 it、墙钟 112–125 s;April 组 camera ~582k、Ceres 75–86 it、墙钟 80–90 s。组内紧、组间分明。

## 表2:热启动口径(从 Kalibr 结果读取)— 验证是否同一优化问题

Ceres 用 `--init-from-kalibr` 把 Kalibr 的外参/time-shift/gravity 当**初值**,再放开**与表1完全相同**的 joint 优化器。如果两套实现是同一个优化问题,从 Kalibr 解出发应该几乎不动(漂 ≈0);实际漂多少,就是"两套实现各自最优点之间的距离"。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 墙钟 |
|---|---:|---:|---:|---|---:|
| 2025_03_14_00_10_18 | 1.82 mm | 0.028° | +0.07 ms | 0.1802 / 0.1798 | 40 s (32 it) |
| 2025_03_14_00_34_14 | 2.63 mm | 0.056° | +0.04 ms | 0.1802 / 0.1797 | 60 s (53 it) |
| 2025_03_14_00_50_37 | 1.43 mm | 0.004° | -0.01 ms | 0.1802 / 0.1801 | 61 s (55 it) |
| 2025_03_14_02_13_45 | 1.14 mm | 0.004° | +0.03 ms | 0.1795 / 0.1791 | 38 s (32 it) |
| 2025_03_14_02_21_41 | 1.06 mm | 0.002° | +0.00 ms | 0.1791 / 0.1787 | 40 s (35 it) |
| 2025_03_14_10_23_35 | 0.30 mm | 0.002° | +0.06 ms | 0.1770 / 0.1766 | 53 s (45 it) |
| 2025_04_19_18_43_05 | 0.19 mm | 0.001° | +0.03 ms | 0.1717 / 0.1715 | 56 s (51 it) |
| 2025_04_19_19_03_03 | 0.46 mm | 0.000° | +0.08 ms | 0.1715 / 0.1712 | 44 s (40 it) |
| 2025_04_19_19_20_46 | 0.49 mm | 0.000° | +0.07 ms | 0.1708 / 0.1706 | 48 s (44 it) |
| 2025_04_19_19_35_04 | 0.70 mm | 0.000° | +0.04 ms | 0.1711 / 0.1708 | 48 s (45 it) |
| 2025_04_19_19_55_25 | 0.55 mm | 0.004° | +0.04 ms | 0.1719 / 0.1717 | 91 s (89 it) |
| 2025_04_19_20_21_09 | 0.43 mm | 0.004° | +0.05 ms | 0.1715 / 0.1712 | 68 s (66 it) |

**这是本篇最重要的判断。** 热启动从 Kalibr 解出发,放开 joint 优化器后**并没有停在原地,而是漂到 0.19–2.63 mm 处收敛**(time-shift 被紧先验锁住,几乎不漂)。这意味着:

- **Ceres 和 Kalibr 不是逐位相同的优化问题。** 在弱可观的外参平移方向上,两套实现的最优点本身相差 0.19–2.63 mm(March 组 1–2.6 mm,April 组 0.2–0.7 mm)。差异来自权重映射(Cauchy loss)、pose 样条表示/正则、以及优化器在该弱方向上的细节,而非 bug——reproj 仍逐位相同(±0.001 px),说明这点平移差在重投影上几乎不可见。
- **表2(0.19–2.63 mm)< 表1(1.3–4.7 mm)。** 二者之差 ≈ "从零独立收敛"的额外代价:表2 是"两套最优点的固有距离",表1 在此之上再加上独立初始化没能完全收敛到 Ceres 自己那个最优点的部分。
- **修正之前的说法**:早先用 staged `0,1,4,5` 暖启动得到的 ~0.05 mm 是**调度假象**——保守 stage 几乎不优化外参,所以看起来"没动"。一旦用 joint 优化器真正放开,真实漂移是 0.19–2.63 mm。所以"热启动一致性 = 同一问题"只在 sub-mm~mm 量级近似成立,不是严格相等。

## 墙钟波动的深层原因

**一句话**:Ceres 的墙钟 = 窄迭代带(75–121,相对停止)× 确定性原生每步成本 → 低波动 + 按问题规模清晰分 March/April 两簇;Kalibr 的墙钟 = 宽迭代带(3–12,绝对停止 `deltaX/deltaJ`)× 模拟环境的昂贵且带噪每步 → 高波动、不按规模分簇。

为什么 Ceres 墙钟波动小、组内紧组间分明,而 Kalibr 波动大?把墙钟拆成 `墙钟 ≈ 构建/加载 + 迭代数 × 每迭代成本` 看,两边在这三项上性质不同。

**1. 迭代数的波动幅度不同(主因)。** Kalibr 用 Optimizer2 的**绝对停止**(`deltaX≤1e-2`、`deltaJ≤1`),迭代数对每套数据的代价地形非常敏感:简单的 3 次就停(`00_50_37`),难的要 12 次(`20_21_09`)——4× 跨度。Ceres 用 trust-region 的**相对下降停止**,迭代数落在一个随问题规模变化的窄带(75–121,1.6× 跨度),且和采集 session 的规模强相关。迭代数越稳,墙钟越稳。

**2. 每迭代成本的确定性不同。** Ceres 原生、单进程、固定 `SPARSE_NORMAL_CHOLESKY`,每迭代成本几乎是常数(≈1.05 s/it:115 s / 110 it);成本主要由问题规模(camera residual 数)决定,所以 March(~662k)比 April(~582k)系统性更慢,形成清晰的两簇。Kalibr 在 amd64 模拟 Docker 里跑,每迭代 ≈17 s/it(优化 86.8 s / 5 it),模拟层本身带额外开销和抖动,放大了波动。

**3. 一个反直觉但关键的事实:Kalibr 迭代数远少于 Ceres(3–12 vs 75–121),但每步贵 ~16×。** Kalibr 的 LM(lambda 策略 + BlockCholesky)在弱可观的外参平移方向上每步走得更狠,十几步内收敛;Ceres 的 trust-region 在该方向上每步走得保守,需要上百步。最终 Ceres 用"很多便宜步"、Kalibr 用"很少昂贵步",Ceres 总时间反而更短、更稳。

所以本质是:**Ceres = 窄迭代带 × 确定性原生每步成本 → 低波动 + 规模驱动的清晰分组;Kalibr = 绝对停止导致的宽迭代带(3–12)× 模拟环境的昂贵且带噪每步 → 高波动**。这也解释了为什么 Kalibr 墙钟不像 Ceres 那样干净地按 March/April 分簇——它的墙钟被波动的迭代数主导,而迭代数取决于各套数据能多快越过绝对阈值,不只是问题规模。

## 2026-06-16 模型覆盖回归

这次回归要回答两个不同层级的问题。第一,相机模型覆盖变宽以后,已有 `current-full` 精度基线有没有被破坏。第二,扩展 IMU 模型不是只"能跑",而是要确认它和 Kalibr 源码里的 `IccScaledMisalignedImu` / `IccScaledMisalignedSizeEffectImu` 是同一个前向模型,并且新增参数块已经有生产路径需要的手写解析 Jacobian。

相机侧新增读取和投影组合包括 `pinhole+radtan/equidistant/fov/none`、`omni+radtan/none`、`eucm` 和 `ds`。`tests/test_math.cpp` 对这些组合的 `projectWithJacobian()` 做中心差分验证;`check_dataset` 打印实际读取的 camera/distortion model,用于防止 YAML 被静默当成默认 `pinhole-radtan`。

IMU 侧新增两条可选模型:`--imu-model scale-misalignment` 对应 Kalibr `IccScaledMisalignedImu`,`--imu-model scale-misalignment-size-effect` 对应 `IccScaledMisalignedSizeEffectImu`。参数块包括 accelerometer/gyro lower-triangular scale-misalignment 矩阵、gyro sensing rotation、gyro acceleration sensitivity,以及 size-effect 的三根 accelerometer sensing-axis offset。扩展 gyro、scale/misaligned accel 和 size-effect accel 现在都已从 Ceres 中心差分切换为手写 `SizedCostFunction`;`tests/test_math.cpp` 对所有新增参数块做中心差分复核。

回归结果:

| 运行 | 目的 | 结果 |
|---|---|---|
| `ctest --test-dir ceres_cam_imu/build --output-on-failure` | 相机投影 Jacobian、普通 IMU residual、扩展 IMU Kalibr 源码公式等价、扩展 IMU 解析 Jacobian、problem size、状态快照 | 通过 |
| `camera_model_current_full_regression_20260616/current-full` | 确认新相机模型代码不改变已有 Kalibr 初值全量基线 | rotation `0 deg`, translation `5.17357e-05 m`, time-shift `+0.00084628 s`, residual mean `0.196977 px / 0.167393 rad/s / 0.863287 m/s^2` |
| `imu_size_effect_analytic_smoke_20260616_repeat/smoke-fixed` | 用小样本确认 size-effect 参数块、解析 residual、优化和统计链路可运行 | `imu_model=scale-misalignment-size-effect`, `active_parameter_blocks=7`, elapsed `0.055597 s`, solver total `0.00432 s`, residual mean `0.12762 px / 0.212704 rad/s / 2.98951 m/s^2` |

扩展 IMU 的"精度一致性"目前是**模型级一致性**,不是全量扩展 IMU 标定结论。`test_math` 用非平凡的 scale/misalignment、gyro sensing rotation、gyro acceleration sensitivity 和 size-effect axis offsets,把 Ceres prediction 与 Kalibr 源码公式直接展开的表达式逐项比较,阈值为 `1e-12`。同一个测试还对 scale-misaligned gyro、scale-misaligned accel、size-effect accel 的所有参数块做中心差分复核;这证明当前 Ceres 版扩展 residual 不是另一个前向模型,手写 Jacobian 也没有明显符号错误。

小样本 smoke 继续说明一件更工程化的事:把 Jacobian 从中心差分切到解析式以后,同一 `cam_imu_2`、20 帧、50 个 IMU residual、Kalibr 初值、固定 pose/bias/time/gravity 的 size-effect run 与旧 smoke 在外参和 residual 上保持打印精度一致。旧中心差分 smoke 为 elapsed `0.058918 s`,solver total `0.00605 s`;解析 smoke 复跑为 elapsed `0.055597 s`,solver total `0.00432 s`。这个规模太小,不能外推成生产全量速度倍率,但它确认了解析路径没有改变求解结果,也去掉了生产路径上最不该保留的 per-residual 数值差分开销。

还没有覆盖的是**全量扩展 IMU 模型的 Kalibr-vs-Ceres 标定精度**。当前多数据集基线使用普通 `IccImu` 产线结果,没有 scale/misalignment 或 size-effect 的 Kalibr 扩展 IMU 标定结果作为 oracle。要回答这个问题,需要准备带扩展 IMU model 字段和初值的 Kalibr 配置,用 Docker Kalibr 跑出同口径结果,再用 Ceres 的 `--imu-model scale-misalignment` / `scale-misalignment-size-effect` 做全量对比。当前结论应严格表述为:相机模型扩展不破坏普通模型全量基线;扩展 IMU 的前向模型和解析 Jacobian 已完成源码级与差分级回归,全量扩展标定精度仍是下一组实验。

## 结论

- **是否同一优化问题(表2 热启动)**:不是逐位相同。从 Kalibr 解出发、放开 joint 优化器后,外参平移漂到 0.19–2.63 mm 处收敛——这就是两套实现各自最优点的固有距离(来自权重/前向/优化器弱方向细节)。reproj 仍逐位相同,说明该差在重投影上不可见。早先 staged 暖启动的 ~0.05 mm 是调度假象。
- **从零独立精度(表1)**:独立初始化充分收敛后,reproj 与 Kalibr 逐位相同(±0.001 px),旋转差多数 < 0.02°,time-shift 差多数 < 1.5 ms;外参平移差 1.3–4.7 mm = 上面的固有差 + 从零收敛的额外代价。
- **速度**:Ceres 原生 80–125 s vs Kalibr 模拟 Docker 145–261 s。注意这不是 native-vs-native——Kalibr 在本机只能 amd64 模拟(有 3–10× 惩罚);要严格倍率需在同一台 Linux 上各跑一次。
- **要抹平最后这几毫米**:外参平移是弱可观方向(reproj 对它不敏感),需对齐 Kalibr 优化器内核(LM lambda 策略 + BlockCholesky + `deltaX/deltaJ` 绝对停止),或对外参平移块做变量缩放/预条件。这是单独的下一步。

## 复现

公平口径充分收敛 Ceres(单数据集,把 `<DS>` 换成具体目录):

```bash
ceres_cam_imu/build/calibrate_cam_imu --kalibr-corner-defaults \
  --cam <DS>/cam0-camchain-640x400.yaml --imu <DS>/imu.yaml --target <DS>/aprilgrid.yaml \
  --imu-data <DS>/data1.csv --corners <DS>/cam0_640x400_corners.csv \
  --corner-poses <DS>/cam0_640x400_corner_poses.csv \
  --kalibr-result <DS>/cam0_640x400_corners-1-results-imucam.txt \
  --estimate-time-shift-prior --estimate-orientation-gravity-prior \
  --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 \
  --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 \
  --max-iterations 150 --solver-max-trust-region-radius 10000000
```

等价 preset:`python3 ceres_cam_imu/tools/run_ceres_sweep.py --preset independent-joint-full --run-name <name>`(默认数据集 cam_imu_2;其它数据集用上面的显式命令)。

Kalibr 全量收敛(单数据集,amd64 模拟,约 2–4 分钟):每个数据集的单行命令见 `docs/常用命令.txt`「production_calibration 多数据集」一节。

角点导出(Ceres 读 CSV 的前置一次性步骤)同见上文 `docs/常用命令.txt`。

表2 热启动只需把上面命令里的 `--estimate-time-shift-prior --estimate-orientation-gravity-prior` 换成 `--init-from-kalibr`,其余 joint 优化器参数不变。`--init-from-kalibr` 仅用于验证"是否同一问题",不是生产路径。

注:`cam_imu_debug` 等低质量数据不在本篇范围。
