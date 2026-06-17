# Ceres 与 Kalibr Docker 多数据集速度精度对比

## 背景

`ceres-cam-imu` 的目标不是做一个依附 Kalibr 输出的后处理器，而是形成一条**可独立运行、可生产部署、可解释速度和精度边界**的 Ceres cam-IMU 标定链路。Kalibr 在这份文档里只承担两个角色：第一，作为已知强基线，帮助量化 Ceres 的精度差；第二，在部分输入格式还没有原生检测器时，提供角点导出/ROS bag 转换环境。

因此本文把三种口径明确拆开：

| 口径 | 用途 | 是否读取 Kalibr 结果参与求解 |
|---|---|---|
| **Ceres 独立标定** | 生产主路径，从 Ceres 自己的 time/gravity/pose 初始化开始求解 | **否** |
| **Ceres 热启动标定** | 实验诊断，检查两套优化问题从同一点出发会漂多远 | 是，且显式使用 `--init-from-kalibr` |
| **Kalibr Docker 标定** | 精度/速度基线与扩展 IMU oracle | Kalibr 自身求解 |

## 目标

这份实验文档要回答四个问题。

1. **独立精度**：不读 Kalibr 结果时，12 组生产数据能否全部收敛，外参、time-shift、reprojection 与 Kalibr 相差多少。
2. **优化问题一致性**：从 Kalibr 解热启动后，Ceres 会不会停在原地；若不会，固有差异有多大。
3. **模型覆盖**：新增相机模型、扩展 IMU 的 `M_a/M_g/A_g/C_g`、多 camera 链路是否有全量数据证据，而不只是单测。
4. **TUM IMU residual**：TUM 双目数据上，Ceres 能否把 gyro/accel residual 提升到 Kalibr 同量级。

## 数据与口径

| 项 | 内容 |
|---|---|
| 生产数据 | `production_calibration/data/<ts>/cam_imu`，12 组，均使用 `data1.csv` |
| TUM 数据 | `/Users/wayne/Documents/work/data/TUM` 下两组 `dataset-calib-imu*_512_16` 双目 cam-IMU |
| Ceres | `ceres_cam_imu/build/calibrate_cam_imu`，原生 arm64 Release |
| Kalibr | `kalibr-camera-calibration:20.04`，本机 arm64 上 amd64 Docker 模拟 |
| 默认 Ceres 口径 | `--corner-defaults`，显式覆盖参数优先级更高 |
| 评测指标 | 外参平移差、外参旋转差、time-shift 差、reprojection/gyro/accel residual mean、墙钟 |

差异均为 Ceres 减 Kalibr。`T_c_b` 表示把点从 IMU/body frame 变到 camera frame 的外参；平移差为 `||t_C - t_K||`，旋转差为 `R_C R_K^T` 的测地角，time-shift 定义为 `t_imu = t_cam + tau`。

### IMU 裁边口径

生产数据和扩展 IMU 表格使用的是 **Kalibr-compatible 有效 IMU 区间**，不是 raw `data1.csv` 的全量行数。原因是 Kalibr 的 `--imu_data_file` 路径默认 `trim_imu_edge_count=1000`，本仓库的 Kalibr Docker wrapper 也显式传入 `--trim-imu-edge-count 1000`；Ceres 的 `--corner-defaults` 对齐该口径，默认设置 `--imu-trim-edge-count 1000`。

具体规则是裁掉首尾边缘 IMU 样本，保留索引 `[1000, num_messages - 1000]`。例如 `2025_03_14_00_10_18` 的 raw IMU 是 `24859` 行，参与 Kalibr/Ceres 优化与 residual 统计的是 `22860` 行。这样做是为了避免 spline 边界处缺少稳定支撑的 IMU 点进入优化；若要评估 raw IMU 全量口径，需要 Kalibr 和 Ceres 都显式设置 `--trim-imu-edge-count 0` 并作为新的实验表格单独报告。

## 实验一：生产独立标定

这一组是生产主路径：Ceres 不读取 Kalibr 的外参、time-shift、gravity 或 IMU intrinsic 作为初值。初始化来自 Ceres 自己的 gyro-norm time-shift prior、orientation/gravity prior、pose spline fit、bias zero init 和 joint 优化。表格中的 delta 在评测阶段与 Kalibr 结果文件离线比较得到。

本表的 IMU 约束与 residual 统计均采用上一节的裁边口径。相机角点不做对应裁边，仍按 corner CSV 中的全部有效观测统计 reprojection。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 墙钟 | Kalibr 墙钟 |
|---|---:|---:|---:|---|---:|---:|
| 2025_03_14_00_10_18 | 3.62 mm | 0.0302° | -0.988 ms | 0.1803 / 0.1798 | 118 s (110 it) | 162 s (5 it) |
| 2025_03_14_00_34_14 | 4.66 mm | 0.0570° | -0.800 ms | 0.1803 / 0.1797 | 120 s (111 it) | 215 s (6 it) |
| 2025_03_14_00_50_37 | 1.96 mm | 0.0052° | -0.783 ms | 0.1803 / 0.1801 | 124 s (115 it) | 145 s (3 it) |
| 2025_03_14_02_13_45 | 2.35 mm | 0.0070° | -1.391 ms | 0.1797 / 0.1791 | 121 s (113 it) | 207 s (7 it) |
| 2025_03_14_02_21_41 | 2.24 mm | 0.0016° | -0.575 ms | 0.1792 / 0.1787 | 131 s (121 it) | 254 s (10 it) |
| 2025_03_14_10_23_35 | 2.38 mm | 0.0036° | -1.121 ms | 0.1772 / 0.1766 | 111 s (106 it) | 168 s (4 it) |
| 2025_04_19_18_43_05 | 1.40 mm | 0.0051° | -3.199 ms | 0.1708 / 0.1715 | 85 s (79 it) | 225 s (10 it) |
| 2025_04_19_19_03_03 | 3.05 mm | 0.0136° | -6.267 ms | 0.1724 / 0.1712 | 82 s (77 it) | 246 s (10 it) |
| 2025_04_19_19_20_46 | 1.48 mm | 0.0000° | -0.700 ms | 0.1704 / 0.1706 | 81 s (77 it) | 194 s (7 it) |
| 2025_04_19_19_35_04 | 1.31 mm | 0.0000° | +1.339 ms | 0.1719 / 0.1708 | 81 s (75 it) | 183 s (8 it) |
| 2025_04_19_19_55_25 | 1.68 mm | 0.0073° | +1.420 ms | 0.1728 / 0.1717 | 88 s (84 it) | 178 s (7 it) |
| 2025_04_19_20_21_09 | 2.49 mm | 0.0113° | -6.189 ms | 0.1722 / 0.1712 | 89 s (86 it) | 261 s (12 it) |

**结论**：12 组独立标定全部收敛；reprojection 与 Kalibr 基本逐位相同，旋转差多数小于 `0.02°`，time-shift 多数在 `1.5 ms` 内；外参平移差为 `1.3-4.7 mm`。剩余毫米级差异集中在弱可观平移方向，reprojection 对它不敏感。

## 实验二：热启动一致性

热启动不是生产路径。它只回答一个诊断问题：如果把 Kalibr 的外参、time-shift、gravity 当初值，再用与实验一相同的 Ceres joint 优化器放开求解，最终会离 Kalibr 多远。

| 数据集 | 外参平移差 | 外参旋转差 | time-shift 差 | reproj px (Ceres/Kalibr) | Ceres 墙钟 |
|---|---:|---:|---:|---|---:|
| 2025_03_14_00_10_18 | 1.82 mm | 0.0275° | +0.071 ms | 0.1802 / 0.1798 | 36 s (32 it) |
| 2025_03_14_00_34_14 | 2.63 mm | 0.0555° | +0.042 ms | 0.1802 / 0.1797 | 57 s (53 it) |
| 2025_03_14_00_50_37 | 1.43 mm | 0.0045° | -0.009 ms | 0.1802 / 0.1801 | 59 s (55 it) |
| 2025_03_14_02_13_45 | 1.14 mm | 0.0044° | +0.029 ms | 0.1795 / 0.1791 | 36 s (32 it) |
| 2025_03_14_02_21_41 | 1.06 mm | 0.0019° | +0.001 ms | 0.1791 / 0.1787 | 39 s (35 it) |
| 2025_03_14_10_23_35 | 0.30 mm | 0.0018° | +0.062 ms | 0.1770 / 0.1766 | 49 s (45 it) |
| 2025_04_19_18_43_05 | 0.19 mm | 0.0014° | +0.030 ms | 0.1717 / 0.1715 | 53 s (51 it) |
| 2025_04_19_19_03_03 | 0.46 mm | 0.0000° | +0.082 ms | 0.1715 / 0.1712 | 43 s (40 it) |
| 2025_04_19_19_20_46 | 0.49 mm | 0.0000° | +0.071 ms | 0.1708 / 0.1706 | 47 s (44 it) |
| 2025_04_19_19_35_04 | 0.70 mm | 0.0000° | +0.037 ms | 0.1711 / 0.1708 | 48 s (45 it) |
| 2025_04_19_19_55_25 | 0.55 mm | 0.0043° | +0.036 ms | 0.1719 / 0.1717 | 91 s (89 it) |
| 2025_04_19_20_21_09 | 0.43 mm | 0.0041° | +0.055 ms | 0.1715 / 0.1712 | 69 s (66 it) |

**结论**：Ceres 与 Kalibr 不是逐位相同的优化问题。从同一个 Kalibr 解出发，Ceres 仍会漂到 `0.19-2.63 mm` 后收敛；这就是两套实现的固有距离。实验一的 `1.3-4.7 mm` 可以理解为“固有距离 + 独立初始化/收敛路径的额外代价”。

## 实验三：速度结构

Ceres 墙钟为 `81-131 s / 75-121 it`，Kalibr Docker 为 `145-261 s / 3-12 it`。这个速度对比不能直接解释为算法原生倍率，因为 Kalibr 是 amd64 Docker 模拟，而 Ceres 是 arm64 原生。

更有价值的是结构差异：Ceres 单步便宜但迭代多，墙钟主要跟 problem size 走；Kalibr 单步贵但迭代少，墙钟主要受绝对停止条件和模拟环境波动影响。March 组 camera residual 约 `662k`，Ceres 用时 `111-131 s`；April 组约 `582k`，Ceres 用时 `81-89 s`，组内稳定、组间清晰。

### 停止条件口径

Kalibr 的 cam-IMU 优化使用三类停止条件：最大迭代数、`deltaX <= 1e-2`、`|deltaJ| <= 1`。后端循环的逻辑等价于三者满足其一即可退出；例如 `2025_03_14_00_10_18` 的扩展 IMU run 在第 8 次迭代 `dJ=0.303 < 1` 时停止，当时 `deltaX=0.027` 仍大于 `1e-2`。

Ceres 也补了三类停止入口，但只对齐**停止结构**，不逐值复用 Kalibr 的 `J/dJ/deltaX`。原因有三点：

- `J/dJ` 是优化器内部目标值。Kalibr 使用 aslam backend 的目标统计，Ceres 报告的是自身 loss convention 下的 cost；两边的 robust loss、先验项和归一化不逐项相同。
- `deltaX` 也不是同一个量。Kalibr 统计最小维度设计变量更新向量的 max coefficient；Ceres 当前 callback 统计 active parameter block 的实际最大系数变化，只是更接近 `deltaX` 的生产代理量。
- residual parity 不要求内部 `J` parity。实验四说明扩展 IMU 的 reprojection/gyro/accel residual 和 `M_a/M_g` 已到同量级，但这不能推出两边 `dJ=1` 代表相同收敛程度。

`2025_03_14_00_10_18` 扩展 IMU 的 smoke 结果说明，Ceres 若直接照搬 Kalibr 的 `dJ=1` 会提前退出：

| 口径 | 停止点 | 墙钟 | 结果判断 |
|---|---|---:|---|
| Kalibr 扩展 IMU | 8 it，`dJ=0.303 < 1`，`deltaX=0.027` | optimize `160 s` | 作为 residual 与扩展 IMU intrinsic 基线 |
| Ceres 直接用 `dJ=1` | 49 it，`cost_change=0.951` | `60 s` | 过早；外参旋转差 `0.31°`，平移差 `12.8 mm`，gravity 差 `0.092` |
| Ceres 用参数变化 `1e-2` | 107 it，`parameter_delta=0.00987` | `122 s` | residual 同量级，`M_a/M_g` 相对差 `1e-3` 量级 |
| Ceres 跑满 150 it | max-iteration reference | `168 s` | delta 最稳，但时间增加 |

因此当前 `--corner-defaults` 采用生产 preset：保留最大迭代数、绝对 cost change 和参数变化三类停止条件，但把 cost change 收紧到 `5e-2`，并保留已验证的参数变化阈值 `1e-2`。这解释了为什么 Ceres 和 Kalibr 的 residual 可以对齐，而日志里的 `J/dJ/deltaX` 不应该逐数值比较。

## 实验四：模型覆盖与扩展 IMU

相机侧已补齐 `pinhole+radtan/equidistant/fov/none`、`omni+radtan/none`、`eucm`、`ds` 的读取和投影 Jacobian；`tests/test_math.cpp` 对 `projectWithJacobian()` 做中心差分验证，`check_dataset` 会打印实际读取到的 camera/distortion model，避免 YAML 被静默当成默认模型。

IMU 侧新增两类模型：

| Ceres 参数 | 对应含义 |
|---|---|
| `--imu-model scale-misalignment` | `M_a/M_g/A_g`，对应 scale/misalignment 与 gyro sensing rotation |
| `--imu-model scale-misalignment-size-effect` | 在上面基础上增加 size-effect 的 accelerometer sensing-axis offset `C_g` |

扩展 gyro、scale/misaligned accel 和 size-effect accel 均已从数值差分切到手写 `SizedCostFunction`。`ctest --test-dir ceres_cam_imu/build --output-on-failure` 覆盖普通 IMU residual、扩展 IMU Kalibr 源码公式等价、扩展 IMU 解析 Jacobian 与中心差分复核。

三组生产数据的全量 `scale-misalignment` 对比：

这里的“全量”指相机角点全量与 Kalibr-compatible 有效 IMU 区间全量；IMU 不是 raw `data1.csv` 全量行数。

| 数据集 | residual mean Ceres/Kalibr | 外参差 | time-shift 差 | `M_a` rel | `M_g` rel | `A_g` fro | `C_g` fro |
|---|---|---:|---:|---:|---:|---:|---:|
| `2025_03_14_00_10_18` | `0.18006/0.18081 px`, `0.01669/0.01746 rad/s`, `0.11228/0.10409 m/s^2` | `0.082°`, `2.05 mm` | `-1.26 ms` | `8.54e-4` | `4.94e-4` | `4.71e-4` | `2.03e-3` |
| `2025_03_14_10_23_35` | `0.17663/0.17722 px`, `0.01596/0.01673 rad/s`, `0.11395/0.10608 m/s^2` | `0.069°`, `1.85 mm` | `-1.20 ms` | `7.19e-4` | `5.26e-4` | `4.62e-4` | `1.75e-3` |
| `2025_04_19_19_35_04` | `0.17208/0.18885 px`, `0.01385/0.01412 rad/s`, `0.07172/0.06555 m/s^2` | `0.053°`, `6.13 mm` | `-1.31 ms` | `6.09e-4` | `1.45e-3` | `9.22e-4` | `2.05e-3` |

**结论**：扩展 IMU 不再只是模型级 smoke。`M_a/M_g` 相对差在 `1e-3` 量级，`A_g/C_g` Frobenius 也在 `1e-3` 量级；reprojection 和 gyro 基本追平 Kalibr，accelerometer residual 仍略高，弱可观外参平移在毫米级漂移。

## 实验五：多 camera 与 TUM single-stage

Kalibr cam-IMU 支持多 camera：`kalibr_calibrate_imu_camera --cams <camchain.yaml>` 会通过 `IccCameraChain` 构建 camera chain，并为每个 camera 加重投影误差、time shift 和 camera-chain baseline。Ceres 当前也支持**一个共享 camchain + 多个 `--corners` CSV** 的非 staged joint 优化，结果 YAML 写出 `camera_chain`。当前限制是 staged multi-camera 还未支持，CLI 会拒绝该组合。

TUM 双目使用同一个 `--cam <camchain.yaml>`，并传两个 `--corners`。如果误传两个相同的 `--cam`，两路会都按 `cam0` section 读取，cam1 投影会错误；这属于 CLI 约定问题，不是多相机优化缺失。

早期二阶段诊断说明，20/10 kps 的低频 pose/bias 轨迹承载不了 TUM 高频 IMU；这不是数据读取、multi-camera、扩展 IMU 前向公式或 `M_a/M_g` 的问题。最终保留的实验口径是**不读 Kalibr result 的 100/50 kps single-stage joint 优化**。

| TUM 数据 | Kalibr residual mean | Ceres single-stage residual mean | Ceres solver | 判断 |
|---|---|---|---|---|
| `dataset-calib-imu1_512_16` | `0.10646-0.10737 px`, `0.00119649 rad/s`, `0.02146520 m/s^2` | `0.10352 px`, `0.00118534 rad/s`, `0.02121921 m/s^2` | 27 iter, 15.9 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |
| `dataset-calib-imu2_512_16` | `0.10708-0.10702 px`, `0.00116924 rad/s`, `0.02134775 m/s^2` | `0.10321 px`, `0.00121503 rad/s`, `0.02117489 m/s^2` | 28 iter, 17.3 s, `CONVERGENCE` | gyro/accel 同量级，reprojection 不差于 Kalibr |

**结论**：TUM 上最初差一个数量级的 IMU residual，根因是低 kps 轨迹频率不足。直接使用足够的 pose/bias knot rate 后，Ceres single-stage 已达到 Kalibr 同量级；旧 hotstart/二阶段结果只作为诊断背景，不再作为默认流程证据。

## 输入格式与独立性

`calibrate_cam_imu` 标定二进制本身读取统一中间格式：camchain/IMU/target YAML、IMU CSV、一个或多个角点 CSV、可选 corner poses CSV。它不需要 Kalibr Docker，也不需要先读取 Kalibr 标定结果。

当前外层转换入口是 `tools/prepare_ceres_inputs.py`，支持：

| 输入 | 当前方式 | 是否依赖 Kalibr Docker/ROS |
|---|---|---|
| `pkl` | `export_kalibr_corners.py` 转角点 CSV | 转换依赖，求解不依赖 |
| `bag` | `export_kalibr_bag_to_ceres.py` 转角点/时间戳/IMU CSV | 转换依赖，求解不依赖 |
| `euroc` / TUM `mav0` | 先 `kalibr_bagcreater` 成 bag，再走 bag 导出 | 转换依赖，求解不依赖 |

`--run-calibration` 可一键完成转换后单阶段标定，并默认补上 production single-stage preset；`--run-two-stage` 保留为 TUM/轨迹频率诊断入口。后续要完全摆脱 Kalibr Docker，需要补 Ceres 侧原生 AprilTag 检测和 bag/euroc parser。

## 结论

- **Ceres 独立标定已经可跑生产 12 组**：全部 `CONVERGENCE`，reprojection 与 Kalibr 基本一致，外参旋转多数 `<0.02°`，外参平移 `1.3-4.7 mm`。
- **生产表格采用 Kalibr-compatible IMU 裁边口径**：Kalibr `--imu_data_file` 与 Ceres `--corner-defaults` 均裁掉首尾 `1000` 个 IMU 样本；这些结论不能直接解释为 raw IMU 全量行数实验。
- **热启动证明两套优化问题不是逐位相同**：从 Kalibr 解出发仍会漂 `0.19-2.63 mm`，这解释了独立口径中剩余毫米级差异的下限。
- **停止条件对齐结构，不对齐内部数值**：Ceres 已有 max-iteration、absolute cost change、parameter delta 三类停止条件；`J/dJ/deltaX` 的定义与 Kalibr 不同，所以不能直接照搬 Kalibr 的 `dJ=1`。
- **扩展 IMU 有全量证据**：`M_a/M_g` 相对差约 `1e-3`，`A_g/C_g` Frobenius 约 `1e-3`，并且已切到手写解析 Jacobian。
- **多 camera 支持已覆盖非 staged joint 优化**：TUM 双目验证了 shared camchain + 多 `--corners` 的路径；staged multi-camera 是诊断增强，不再阻塞默认流程。
- **TUM residual 已闭环到 Kalibr 同量级**：single-stage 100/50 kps 后，gyro 达 `0.00119-0.00122 rad/s`，accel 达 `0.0212 m/s^2`。
- **速度结论需谨慎**：当前 Ceres 原生快于 Kalibr Docker 模拟，但严格倍率需要在同一 Linux native 环境里重跑。

## 复现命令

Ceres 生产独立标定单数据集示例，`<DS>` 替换为具体 `cam_imu` 目录：

```bash
ceres_cam_imu/build/calibrate_cam_imu --corner-defaults \
  --cam <DS>/cam0-camchain-640x400.yaml --imu <DS>/imu.yaml --target <DS>/aprilgrid.yaml \
  --imu-data <DS>/data1.csv --corners <DS>/cam0_640x400_corners.csv \
  --corner-poses <DS>/cam0_640x400_corner_poses.csv \
  --estimate-time-shift-prior --estimate-orientation-gravity-prior \
  --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 \
  --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 \
  --max-iterations 150 --solver-max-trust-region-radius 10000000 \
  --output-result /tmp/ceres_independent.yaml
```

`--corner-defaults` 会对齐当前 Kalibr production 口径，其中包含 `--imu-trim-edge-count 1000`。若要跑 raw IMU 全量对照实验，需要显式追加 `--imu-trim-edge-count 0`，并确保 Kalibr 也使用相同设置。

评测 delta 另跑：

```bash
ceres_cam_imu/build/compare_kalibr_result \
  --kalibr-result <DS>/cam0_640x400_corners-1-results-imucam.txt \
  --ceres-result /tmp/ceres_independent.yaml
```

热启动诊断只用于实验：

```bash
ceres_cam_imu/build/calibrate_cam_imu --corner-defaults \
  --cam <DS>/cam0-camchain-640x400.yaml --imu <DS>/imu.yaml --target <DS>/aprilgrid.yaml \
  --imu-data <DS>/data1.csv --corners <DS>/cam0_640x400_corners.csv \
  --corner-poses <DS>/cam0_640x400_corner_poses.csv \
  --kalibr-result <DS>/cam0_640x400_corners-1-results-imucam.txt --init-from-kalibr \
  --pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors --time-shift-prior-sigma 0.0001 \
  --pose-motion-prior --pose-motion-translation-variance 10 --pose-motion-rotation-variance 1 \
  --max-iterations 150 --solver-max-trust-region-radius 10000000
```

12 个 production session 的单行命令见 `docs/常用命令.txt` 的「production_calibration 多数据集」一节。
