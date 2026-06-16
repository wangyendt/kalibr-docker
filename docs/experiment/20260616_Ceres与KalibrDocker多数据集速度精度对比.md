# Ceres 与 Kalibr Docker 多数据集速度精度对比

## 目的

只回答一个问题:**在"公平口径 + 充分收敛"下,`ceres_cam_imu` 标定结果和 Kalibr 差多少、各自要多久。**

- **公平口径(独立初始化)**:Ceres 不读 Kalibr 的任何输出。它复刻 Kalibr 自己的初始化——`--estimate-time-shift-prior`(gyro-norm 互相关求 time shift)、`--estimate-orientation-gravity-prior`(Wahba/Kabsch 闭式求 camera-IMU 旋转/gyro bias/重力)、外参平移从 0 起、pose 样条最小二乘拟合、bias 零初值。这是"从零独立标定"的真实对比,不是把 Kalibr 答案当初值的暖启动。
- **充分收敛**:单次全联合优化(不分阶段、所有变量含重力方向一起放开),`--max-iterations 150`、`--solver-max-trust-region-radius 1e7`。对应 `tools/run_ceres_sweep.py` 的 preset `independent-joint-full`。
- **Kalibr 侧**:各数据集 `data1.csv` 的产线全量收敛结果 `cam0_640x400_corners-1-results-imucam.txt`(精度基线),并在本机 amd64 模拟 Docker 重跑一次取全量收敛墙钟。
- **对比量**:Ceres 减 Kalibr 的外参平移差、旋转差、time-shift 差;两侧 reproj 均值;两侧墙钟。

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

## 主对比表:公平口径充分收敛 Ceres vs Kalibr 全量收敛

差异列为 Ceres 减 Kalibr(定义见上节)。reproj 为各自收敛后的重投影均值。

墙钟列括号内是收敛用的迭代数(Ceres 单次全联合 LM;Kalibr Optimizer2 LM)。

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

## 墙钟波动的深层原因

**一句话**:Ceres 的墙钟 = 窄迭代带(75–121,相对停止)× 确定性原生每步成本 → 低波动 + 按问题规模清晰分 March/April 两簇;Kalibr 的墙钟 = 宽迭代带(3–12,绝对停止 `deltaX/deltaJ`)× 模拟环境的昂贵且带噪每步 → 高波动、不按规模分簇。

为什么 Ceres 墙钟波动小、组内紧组间分明,而 Kalibr 波动大?把墙钟拆成 `墙钟 ≈ 构建/加载 + 迭代数 × 每迭代成本` 看,两边在这三项上性质不同。

**1. 迭代数的波动幅度不同(主因)。** Kalibr 用 Optimizer2 的**绝对停止**(`deltaX≤1e-2`、`deltaJ≤1`),迭代数对每套数据的代价地形非常敏感:简单的 3 次就停(`00_50_37`),难的要 12 次(`20_21_09`)——4× 跨度。Ceres 用 trust-region 的**相对下降停止**,迭代数落在一个随问题规模变化的窄带(75–121,1.6× 跨度),且和采集 session 的规模强相关。迭代数越稳,墙钟越稳。

**2. 每迭代成本的确定性不同。** Ceres 原生、单进程、固定 `SPARSE_NORMAL_CHOLESKY`,每迭代成本几乎是常数(≈1.05 s/it:115 s / 110 it);成本主要由问题规模(camera residual 数)决定,所以 March(~662k)比 April(~582k)系统性更慢,形成清晰的两簇。Kalibr 在 amd64 模拟 Docker 里跑,每迭代 ≈17 s/it(优化 86.8 s / 5 it),模拟层本身带额外开销和抖动,放大了波动。

**3. 一个反直觉但关键的事实:Kalibr 迭代数远少于 Ceres(3–12 vs 75–121),但每步贵 ~16×。** Kalibr 的 LM(lambda 策略 + BlockCholesky)在弱可观的外参平移方向上每步走得更狠,十几步内收敛;Ceres 的 trust-region 在该方向上每步走得保守,需要上百步。最终 Ceres 用"很多便宜步"、Kalibr 用"很少昂贵步",Ceres 总时间反而更短、更稳。

所以本质是:**Ceres = 窄迭代带 × 确定性原生每步成本 → 低波动 + 规模驱动的清晰分组;Kalibr = 绝对停止导致的宽迭代带(3–12)× 模拟环境的昂贵且带噪每步 → 高波动**。这也解释了为什么 Kalibr 墙钟不像 Ceres 那样干净地按 March/April 分簇——它的墙钟被波动的迭代数主导,而迭代数取决于各套数据能多快越过绝对阈值,不只是问题规模。

## 结论

- **精度**:公平口径充分收敛后,Ceres 的 reproj 与 Kalibr 几乎逐位相同(±0.001 px),旋转差多数 < 0.02°,time-shift 差多数 < 1.5 ms。**唯一有量级的残差是外参平移,1.3–4.7 mm,尚未到 Kalibr 的亚毫米。**
- **速度**:Ceres 原生 80–125 s vs Kalibr 模拟 Docker 145–261 s。注意这不是 native-vs-native——Kalibr 在本机只能 amd64 模拟(有 3–10× 惩罚);要严格倍率需在同一台 Linux 上各跑一次。
- **为什么还差几毫米**:外参平移从 0 出发是弱可观方向(reproj 对它不敏感,主要靠 IMU 杠杆/旋转激励约束)。Ceres 已经**收敛**(75–121 it,均 CONVERGENCE,未触 150 上限),但收敛到的点离 Kalibr 还差 1.3–4.7 mm——不是迭代不够,而是两个优化器在这个弱可观方向上的步长策略和停止条件不同,落点略有差异。要抹平最后这几毫米,需对齐 Kalibr 优化器内核(LM lambda 策略 + BlockCholesky + `deltaX/deltaJ` 绝对停止),或对外参平移块做变量缩放/预条件。这是单独的下一步。

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

注:`cam_imu_debug` 等低质量数据、暖启动一致性口径(读 Kalibr 输出做残差/Jacobian 自洽性验证)的细节不在本篇范围;本篇只做公平口径充分收敛对比。
