# Ceres Cam-IMU 标定

这个目录是 Kalibr cam-IMU 优化链路的独立 C++/Ceres 重写路径。它刻意和原 catkin 包分开，方便逐层阅读、替换和验证。

当前支持范围是单 IMU + 单/多 camera chain、AprilGrid，以及统一 CSV 中间格式。相机模型已覆盖 Kalibr 常用组合：`pinhole` + `radtan/equidistant/fov/none`、`omni` + `radtan/none`、`eucm` 和 `ds`。IMU residual 默认使用普通 `calibrated` 模型，也可通过 CLI 切到 scale/misalignment 和 size-effect 扩展模型。Kalibr pkl、ROS bag 和 EuRoC/TUM `mav0` 目录通过 `tools/prepare_ceres_inputs.py` 转换为 Ceres CSV；多 IMU 仍属于后续范围。

## 目录结构

```text
include/ceres_cam_imu/
  camera/        Kalibr 相机投影/畸变模型
  core/          公共类型、SO(3)、SE(3) 工具
  initialization/pose spline 拟合与 time-shift prior 估计
  io/            yaml/csv/result 读写
  optimizer/     Ceres 状态、problem builder、状态快照、求解摘要
  processing/    数据裁剪、帧数限制、统计和时间范围工具
  residuals/     camera、gyro、accel、bias/pose/time prior residual
  target/        AprilGrid target 几何
  trajectory/    均匀 B-spline basis 与 pose/bias spline 元数据
  variables/     pose、bias、外参、重力和 time-shift 参数块
tools/           数据格式转换、Kalibr Docker 基线、Ceres sweep 工具
apps/            CLI 入口
tests/           轻量数值检查
```

## 构建

宿主机已有 Ceres 和 Eigen 时：

```bash
cmake -S ceres_cam_imu -B ceres_cam_imu/build -DCMAKE_BUILD_TYPE=Release
cmake --build ceres_cam_imu/build -j
ctest --test-dir ceres_cam_imu/build --output-on-failure
```

如果宿主机缺依赖，可用本地 Dockerfile：

```bash
docker build -f ceres_cam_imu/docker/Dockerfile -t ceres-cam-imu .
docker run --rm -it -v "$PWD":/work -v /Users/wayne/Documents/work/data:/data ceres-cam-imu bash
cmake -S /work/ceres_cam_imu -B /work/ceres_cam_imu/build -DCMAKE_BUILD_TYPE=Release
cmake --build /work/ceres_cam_imu/build -j
```

## 数据准备

当前快速验证数据集是：

```text
/Users/wayne/Documents/work/data/cam_imu_2
```

`calibrate_cam_imu` 主程序保持轻依赖,原生读取统一中间格式:

| 输入 | CLI | 格式 |
|---|---|---|
| camera / camchain | `--cam` | Kalibr camchain YAML,支持 `cam0/cam1/...` |
| IMU 噪声 | `--imu` | Kalibr IMU YAML |
| 标定板 | `--target` | Kalibr aprilgrid YAML |
| IMU 数据 | `--imu-data` | CSV: `timestamp_ns,gx,gy,gz,ax,ay,az` |
| 角点观测 | `--corners` | CSV: `timestamp_ns,corner_id,pixel_x,pixel_y,target_x,target_y,target_z` |
| target pose 初值 | `--corner-poses` | CSV: `timestamp_ns,T_t_c_00...T_t_c_33` |

外部格式通过 `tools/prepare_ceres_inputs.py` 转成上述 CSV。转换需要 `kalibr-camera-calibration:20.04` Docker 镜像,因为 Kalibr pkl 和 ROS bag target extraction 依赖 Kalibr 的 Python/C++ 扩展类型。转换后,Ceres 标定本身不依赖 ROS。

```bash
# Kalibr corner pickle -> cam0_corners.csv / cam0_corner_poses.csv
python3 ceres_cam_imu/tools/prepare_ceres_inputs.py \
  --source-type pkl --corner-pkl cam0_corners.pkl --out-dir out/ceres_inputs

# ROS bag -> cam*_corners.csv / cam0_corner_poses.csv / imu.csv
python3 ceres_cam_imu/tools/prepare_ceres_inputs.py \
  --source-type bag --bag data.bag --cams camchain.yaml --imu imu.yaml \
  --target aprilgrid.yaml --out-dir out/ceres_inputs

# EuRoC/TUM mav0 folder -> bag -> Ceres CSV
python3 ceres_cam_imu/tools/prepare_ceres_inputs.py \
  --source-type euroc --euroc-dir dataset-calib-imu2_512_16 \
  --cams camchain.yaml --imu imu.yaml --target aprilgrid.yaml \
  --out-dir out/ceres_inputs
```

也可以直接在 Kalibr Python 环境里调用底层导出脚本:

```bash
python3 ceres_cam_imu/tools/export_kalibr_corners.py \
  --corner-pkl /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.pkl \
  --corners-csv /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --poses-csv /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv
```

检查 reader 和数据规模：

```bash
ceres_cam_imu/build/check_dataset \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt
```

## Docker Kalibr 基线

用项目内 wrapper 复现官方零迭代基线：

```bash
python3 ceres_cam_imu/tools/run_kalibr_docker.py --run-name smoke_iter0_staged_verify
```

wrapper 会复制数据到 `ceres_cam_imu/out/kalibr_runs/<run-name>/`，用 `kalibr_calibrate_imu_camera` 运行 `--corner_file`、`--image_timestamp_file`、`--imu_data_file`、`--max-iter 0`、`--timeoffset-padding 0.04`、`--pose-knots-per-second 100`、`--bias-knots-per-second 50` 和 `--trim-imu-edge-count 1000`。

已验证的 `cam_imu_2` 零迭代基线是：`22929` 条 IMU readings，`22929 + 22929` 个 accel/gyro error terms，`9758` 个 design variables，`693126` 个 error terms，Jacobian `1432108x43893`，time shift `-0.5505243893928822 s`，原始 residual mean 为 `0.392372 px / 0.175486 rad/s / 1.06985 m/s^2`。

## Kalibr 对齐开关

`--kalibr-corner-defaults` 是当前 corner 文件基线的便捷 preset。它会在解析显式覆盖参数前设置：

```text
pose_kps=100
bias_kps=50
max_iterations=30
timeoffset_padding_s=0.04
imu_trim_edge_count=1000
camera/gyro/accel Cauchy width=10
```

问题规模 dry-run：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --kalibr-corner-defaults \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --init-from-kalibr \
  --dry-run
```

当前 Ceres dry-run 可对齐 Kalibr 的数量级：`camera=647266`，`gyro=22929`，`accel=22929`，`gyro_priors=2435`，`accel_priors=2435`，`parameter_blocks=9758`，`tangent_params=43893`，`kalibr_style_error_terms=693126`。这个结果证明 problem 组装规模对齐，不等价于最终收敛完全一致。

## 相机模型

`io/config_reader.cpp` 会按 Kalibr camchain YAML 读取 `camera_model`、`distortion_model`、`intrinsics` 和 `distortion_coeffs`，再交给 `camera/CameraModel` 做统一投影。当前支持的组合是：

| camera_model | distortion_model | 内参顺序 |
|---|---|---|
| `pinhole` | `radtan` / `equidistant` / `fov` / `none` | `[fu, fv, cu, cv]` |
| `omni` | `radtan` / `none` | `[xi, fu, fv, cu, cv]` |
| `eucm` | `none` | `[alpha, beta, fu, fv, cu, cv]` |
| `ds` / `double-sphere` | `none` | `[xi, alpha, fu, fv, cu, cv]` |

`equi` 会归一化为 `equidistant`，空畸变会按 `none` 处理。`check_dataset` 会打印实际读取到的模型，例如：

```text
camera: 640x400 model=pinhole distortion=radtan fx=...
```

当前相机内参仍作为固定配置参与投影，不作为 Ceres 参数块优化；优化变量仍是 camera-to-IMU 外参、time shift、pose spline、bias spline、gravity 和可选 IMU intrinsics。`tests/test_math.cpp` 对上表所有投影组合的 `projectWithJacobian()` 做中心差分验证。

## 多相机 Cam-IMU

Kalibr 的 `kalibr_calibrate_imu_camera --cams camchain.yaml` 支持 camera chain。`ceres_cam_imu` 现在也支持同一类输入:传一个包含 `cam0/cam1/...` 的共享 camchain YAML,再按 camera 顺序重复传 `--corners`。

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --cam camchain.yaml \
  --imu imu.yaml \
  --target aprilgrid.yaml \
  --imu-data imu.csv \
  --corners cam0_corners.csv \
  --corners cam1_corners.csv \
  --init-from-camchain \
  --max-iterations 100
```

也可以传多个 `--cam` 和多个 `--corners`,一一对应读取每个 YAML 的 `cam0`。多相机目前只支持单次 joint 优化;`--staged` + 多相机会被拒绝。`--init-from-kalibr` 可以和 `--init-from-camchain` 组合使用:Kalibr result 提供 gravity 和扩展 IMU intrinsic 初值,camchain 提供每个 camera 的 `T_cam_imu` 和 `timeshift_cam_imu` 初值。输出 result YAML 会包含 `camera_chain` 中每个 camera 的 `T_c_b/time_shift_s`。

## IMU 扩展模型

默认 `--imu-model calibrated` 对应 Kalibr 的普通 `IccImu`：gyro prediction 是 `R_i_b * omega_b + bias`，accelerometer prediction 是 `R_i_b * (h_b + lever) + bias`，两条 residual 都使用解析 `SizedCostFunction`。

可选模型：

| CLI | Kalibr 对应 | 新增参数块 |
|---|---|---|
| `--imu-model scale-misalignment` | `IccScaledMisalignedImu` | accelerometer lower-triangular `M_accel`、gyro lower-triangular `M_gyro`、gyro sensing rotation `R_gyro_i`、gyro acceleration sensitivity `A_g` |
| `--imu-model scale-misalignment-size-effect` | `IccScaledMisalignedSizeEffectImu` | 上一行全部参数，再加 accelerometer 三轴 sensing-axis offset `rx_i/ry_i/rz_i` |

`--fix-imu-intrinsics` 会把这些 IMU intrinsic 参数块全部固定，只保留扩展前向模型。size-effect 模型默认固定 `rx_i`，只释放 `ry_i/rz_i`，用于沿用 Kalibr 对 size-effect gauge 的保守处理。

实现入口是：

```text
include/ceres_cam_imu/variables/imu_intrinsics.h
include/ceres_cam_imu/residuals/imu_model.h
src/residuals/gyroscope_residual.cpp
src/residuals/accelerometer_residual.cpp
```

扩展模型已经接入 problem builder、residual statistics、stage state snapshot 和 sweep summary。默认 `calibrated` 路径仍是多数据集性能回归基线；新增 scale/misalignment 和 size-effect residual 使用同一套模块化前向模型和手写解析 `SizedCostFunction`。`tests/test_math.cpp` 会先检查这些前向模型与 Kalibr 源码公式等价，再用中心差分复核扩展 IMU 的每个参数块 Jacobian。

## Time Shift 初值

`--estimate-time-shift-prior` 复现 Kalibr 的粗 time-shift 初始化思路：从 camera target pose 拟合 pose spline，计算 camera 推出的 body angular-rate norm，再和 IMU gyro norm 做 full cross-correlation。输出满足：

```text
t_imu = t_cam + shift
```

当前默认已按 Kalibr 对齐：

```text
--time-shift-pose-kps 100
--time-shift-fit-lambda 1e-4
```

验证命令：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --kalibr-corner-defaults \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --init-from-kalibr \
  --estimate-time-shift-prior \
  --dry-run \
  --max-frames 20 \
  --imu-stride 1000 \
  --max-imu-residuals 5
```

关键输出：

```text
estimated time shift prior: shift_s=-0.55052438939288217 pose_kps=100 fit_lambda=0.0001 discrete_shift_samples=278 sample_dt_s=0.001980303558967202 samples=22929
```

这和 Docker Kalibr 零迭代的 `-0.5505243893928822 s` 对齐。显式旧参数 `--time-shift-pose-kps 20 --time-shift-fit-lambda 1e-9` 在同一数据上会得到 `-0.55448499651081662 s`，说明旧差距主要来自 time-shift 初值 spline 参数没对齐。

## Orientation / Gravity 初值

`--estimate-orientation-gravity-prior` 会在主优化 problem 构建前估计 camera-to-IMU 旋转、gyro bias prior 和 gravity。实现入口是：

```text
include/ceres_cam_imu/initialization/orientation_gravity_initializer.h
src/initialization/orientation_gravity_initializer.cpp
```

算法流程是：先用已知或刚估计的 time shift 把 camera target pose 拟合成 camera-frame pose spline。这里默认复制首尾 pose 作为 boundary anchors，对齐 Kalibr `initPoseSplineFromCamera()` 即使 `timeOffsetPadding=0.0` 也会把首尾观测再加入一次的行为。随后用 body angular velocity 和 IMU gyro 做 Wahba/Kabsch 闭式求解：

```text
gyro_imu ~= R_i_c * omega_camera + bias
```

闭式解之后会再构建一个只含 `r_i_c` 和 `b_g` 两个参数块的小 Ceres problem，使用手写 Jacobian 做最多 50 次 refinement，对齐 Kalibr `findOrientationPriorCameraToImu(...)` 里先建小 Optimizer2 再回写外参旋转和 gyro bias 的结构。当前 `cam_imu_2` 数据上闭式解已经是该最小二乘问题的最优点，refinement 只执行 1 次检查型迭代，数值不变。

然后用 `R_w_c * R_c_i * (-accel_imu)` 的均值方向初始化 gravity，并归一到 `9.80655 m/s^2`。可用 `--no-orientation-prior-boundary-anchors` 回退旧的无首尾锚点拟合，用 `--no-orientation-prior-ceres-refine` 跳过小 Ceres refinement。

真实数据 dry-run：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --kalibr-corner-defaults \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --estimate-time-shift-prior \
  --estimate-orientation-gravity-prior \
  --dry-run \
  --max-frames 20 \
  --imu-stride 1000 \
  --max-imu-residuals 5
```

关键输出是：`boundary_anchors=1`、`ceres_refine=1`、`refine_iterations=1`、`kalibr_rotation_delta_deg=0.175954`、`kalibr_gravity_delta_norm=0.0141821`，`gyro_bias_rad_s=-0.0011955 -0.0013163 0.0065238`。这说明该模块已经能独立给出可用的 orientation/gravity 初值；是否完全替代 `--init-from-kalibr`，还需要继续跑 staged solve 变体验证。

## 外参平移初值

Kalibr 的 cam0 camera-to-IMU 外参初值来自 `IccCamera.T_extrinsic = sm.Transformation()`，也就是单位旋转和零平移。它没有单独的 accelerometer-only 平移初值公式；外参平移是在主 cam-IMU 联合优化中和 pose spline、bias、time shift 一起被约束出来的。

`CameraExtrinsicBlock` 的默认值已经按这个逻辑设为：

```text
T_c_b translation = [0, 0, 0]
T_c_b rotation    = identity
```

这点很重要。旧默认值曾是 `z=0.1m`，独立初始化替代 `--init-from-kalibr` 时会把 staged 优化带到错误的外参平移盆地。改成零平移后，同一全量 staged 命令的 translation delta 从约 `0.101754 m` 降到约 `0.032903 m`；但外参平移仍没有达到 `--init-from-kalibr` 基线，说明剩余问题不再是 Kalibr 平移初值没迁移，而是当前 staged/约束路径还不能从零平移稳定恢复 Kalibr 最终外参。

如果 `--cam` 指向 Kalibr 或产线输出的 `*-camchain-imucam.yaml`，可以用 `--init-from-camchain` 直接读取其中的 `T_cam_imu` 和 `timeshift_cam_imu`：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --kalibr-corner-defaults \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-camchain-imucam.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --init-from-camchain \
  --dry-run \
  --max-frames 20 \
  --imu-stride 1000 \
  --max-imu-residuals 5
```

验证输出应包含 `kalibr_translation_delta_m` 接近 `0`、`kalibr_time_delta_s=0`。这个入口只读取外参和时间偏移；camchain YAML 不保存 gravity，因此完整复现实验基线仍需要 `--init-from-kalibr` 或 `--estimate-orientation-gravity-prior` 提供重力初值。

## 快速冒烟运行

先用小样本检查残差方向、结果写入和 compare 链路：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --init-from-kalibr \
  --estimate-time-shift-prior \
  --fix-poses \
  --fix-biases \
  --fix-time-shift \
  --fix-gravity \
  --max-frames 20 \
  --imu-stride 20 \
  --max-iterations 5
```

`--time-shift-prior-sigma` 会围绕当前初始化 time shift 添加一个 scalar soft prior。分阶段运行中如果 IMU residual 把 time shift 拉离粗估计，可用这个开关定位问题。

## Staged 优化

`--staged` 会重建四个 Ceres problem，并在同一个 state 上逐步放开变量：固定运动估外参、固定 pose/extrinsic 估 time/bias、固定外参估 pose/time/bias、最后固定 refined pose 再估 extrinsic/time/bias。`--max-iterations` 是每个 stage 的默认迭代上限。

常用诊断开关：

```text
--stage-iterations N0,N1,N2,N3
--stage-free MASK[,MASK...]
--stop-on-stage-failure
--stage-pose-translation-variances V0,V1,...
--stage-pose-rotation-variances V0,V1,...
--stage-pose-motion-orders N0,N1,...
--stage-time-shift-prior-sigmas S0,S1,...
--stage-solver-initial-trust-region-radii R0,R1,...
--stage-solver-max-trust-region-radii R0,R1,...
--stage-solver-min-trust-region-radii R0,R1,...
--stage-solver-min-relative-decreases D0,D1,...
--stage-solver-absolute-cost-change-tolerances J0,J1,...
--stage-solver-absolute-step-tolerances X0,X1,...
--stage-solver-absolute-parameter-tolerances X0,X1,...
```

`--stage-free` 的 mask 字母是：`p` pose，`b` bias，`e` extrinsic，`t` time shift，`g` gravity。用 `-` 或 `none` 表示 evaluation-only stage。

每阶段 solver 调度只覆盖显式给出的 Ceres solver 字段，未指定的字段继续使用全局 `--solver-*` 默认值。列表长度必须和 stage 数一致，适合做“前三阶段保持默认、只限制最后 pose/extrinsic 阶段”的实验。绝对停止列表里 `-1` 表示该 stage 关闭自定义停止条件。

每个 stage 开始前都会对 `CalibrationState` 的变量块拍一次快照。stage solve 结束后，如果 Ceres 报告 solver failure、stage 内 cost 变成非有限值，或者同一个 stage problem 的 final cost 高于 initial cost，staged optimizer 会恢复到 stage 前状态；正常下降或零迭代评估则保留更新。CLI 会打印：

```text
stage state [stage_name]: decision=accepted restored=0 initial_cost=... final_cost=... cost_change=... usable=1
```

这里的快照只包含 pose controls、gyro/accel bias controls、camera/IMU extrinsic、gravity、time shift 和可选 IMU intrinsics，不复制 spline 元数据。这样做的目的是把“参数更新是否可信”和“轨迹结构如何初始化”分开，后续如果某个 stage 因病态线性化失败，不会把坏状态带入下一阶段。

## Residual 与状态

camera、gyroscope、accelerometer 以及扩展 IMU residual 都是解析 `SizedCostFunction`。覆盖关系如下：

| residual | 参数块 |
|---|---|
| camera | `T_c_b`、camera time shift、6 个 pose spline control blocks |
| gyroscope | IMU extrinsic rotation、pose controls、gyro bias controls；扩展模型另含 `R_gyro_i`、`M_gyro`、`A_g` |
| accelerometer | lever arm、IMU extrinsic rotation、gravity、pose controls、accel bias controls；扩展模型另含 `M_accel` 和 size-effect axis offsets |
| bias motion prior | 6 个 bias control blocks |
| pose motion prior | 6 个 pose control blocks |
| time shift prior | camera time-shift scalar |

重力默认使用 Kalibr 风格固定范数方向约束：Ceres `SphereManifold<3>` 让三维 gravity block 的 tangent dimension 为 `2`。`--estimate-gravity-length` 会切回无约束三维向量。

`tests/test_math.cpp` 对主要手写 Jacobian 做中心差分验证。覆盖范围包括所有已支持相机模型的 reprojection Jacobian、普通 gyro/accelerometer、扩展 IMU 的 scale/misaligned gyro、scale/misaligned accel、size-effect accel、time shift prior、bias motion prior 和 pose motion prior；motion prior 还额外用数值积分检查二次型残差能量。扩展 IMU 另有 Kalibr 源码公式等价检查和小样本 size-effect smoke。修改 residual 或 SO(3) Jacobian 后，至少需要重新跑：

```bash
ctest --test-dir ceres_cam_imu/build --output-on-failure
```

## Pose 与 Bias 初始化

`--pose-fit-diagonal-lambda`、`--pose-fit-motion-lambda` 和 `--pose-fit-boundary-anchors` 只控制 camera-pose spline 初始化。motion lambda 属于 Kalibr `initPoseSplineSparse` 同族的 derivative-integral regularization；boundary anchors 会在 padded spline 边界复制首尾 camera pose。

Kalibr `initPoseSplineFromCamera()` 会把首尾 pose 各复制一次，再调用 `initPoseSplineSparse(..., 1e-4)`。因此 `independent-full` preset 已默认启用：

```text
--pose-fit-motion-lambda 0.0001
--pose-fit-boundary-anchors
```

这让独立初始化的 pose 二阶导更平滑。`cam_imu_2` 上的实测结果是：accelerometer mean 从旧 pose-fit 的 `0.898892 m/s^2` 降到 `0.880795 m/s^2`，translation delta 从 `0.032903 m` 微降到 `0.0328967 m`，但 reprojection mean 从 `0.197435 px` 升到 `0.201851 px`。这说明 pose spline 初始化差异已经迁移，但它不是剩余外参平移差的主因。旧行为保留在 `independent-legacy-posefit-full` sweep preset。

bias spline 是常值初始化。accel bias 默认是零；gyro bias 默认是零，但启用 `--estimate-orientation-gravity-prior` 后会由该模块估计并通过 `CalibrationOptions::initial_gyro_bias_rad_s` 注入所有 gyro bias control blocks。bias motion prior 已实现 Kalibr 风格一阶导积分。

## 结果写入与比较

`--output-result result.yaml` 会写出结构化结果，包含 `T_c_b`、`T_b_c`、camera-to-IMU time shift、gravity、spline 元数据、分组 residual statistics、top accelerometer outliers，以及提供 Kalibr result 时的 `kalibr_delta`。

比较命令：

```bash
ceres_cam_imu/build/compare_kalibr_result \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --ceres-result /tmp/ceres_cam_imu_compare_smoke.yaml
```

输出包含 Ceres/Kalibr rotation、translation、time shift、gravity 和 residual mean delta。

## Ceres Sweep Runner

批量比较多组 Ceres 参数时使用：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py --run-name smoke_verify
```

默认 `smoke-fixed` preset 是小样本：20 个 camera frames、50 个 IMU residuals、Kalibr 初始化，并固定 pose/bias/time/gravity。输出写到 `ceres_cam_imu/out/ceres_sweeps/<run-name>/`。

内置 preset 的定位如下：

| preset | 用途 | 说明 |
|---|---|---|
| `smoke-fixed` | 小样本构建冒烟 | 默认 preset，固定 pose/bias/time/gravity，用于快速确认 residual 构建和结果写入 |
| `kalibr-dry-run` | 读取 Kalibr txt 的初始化检查 | 不优化，只检查 `--init-from-kalibr`、数据规模和初始 delta |
| `current-full` | 当前 Ceres 对 Docker/Kalibr 的全量回归 | 复用 Kalibr txt 初值，使用当前推荐 staged optimizer 设置 |
| `independent-full` | 不依赖 Kalibr txt 的独立初始化回归 | 使用已迁移的 time-shift、orientation/gravity、gyro-bias 和 Kalibr 风格 pose-fit 初始化 |
| `independent-legacy-posefit-full` | 旧 pose-fit 独立初始化回归 | 不启用 `--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors`，用于和迁移后的 `independent-full` 对照 |
| `independent-final-pe-full` | 独立初始化后的最终 pose+extrinsic 联合释放诊断 | 前三阶段同独立初始化路径，最后一阶段只释放 pose 和 extrinsic，固定 bias/time/gravity，用于观察外参平移能否脱离零平移盆地 |
| `independent-capped-pe-full` | 最后 pose+extrinsic 阶段 trust-region 受限诊断 | 在 `independent-final-pe-full` 基础上只限制最后 `pe` 阶段的 `max_trust_region_radius`，复现当前独立初始化最接近 Kalibr 的路径 |
| `independent-joint-full` | 独立初始化 + 单次全联合优化（最接近 Kalibr 主优化） | 不分阶段，所有变量含重力方向一起放开，`--max-iterations 150 --solver-max-trust-region-radius 10000000`。生产 cam_imu 数据上把外参平移差从 ~14 cm（保守 staged）降到 ~2–4 mm，是目前公平口径最接近 Kalibr 的 preset；详见 `docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md` |
| `camchain-dry-run` | 读取 Kalibr/产线 camchain YAML 的初始化检查 | 自动把 `--cam` 切到 `cam0_640x400_corners-1-camchain-imucam.yaml`，读取 `T_cam_imu` 和 `timeshift_cam_imu` |
| `camchain-full` | camchain YAML 初值加当前 staged optimizer | 用 camchain 的外参/time shift，加独立 orientation/gravity prior 做全量优化 |

例如同时检查 camchain 读取层和独立初始化全量基线：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py \
  --preset camchain-dry-run \
  --preset independent-full \
  --run-name preset_probe_camchain_independent \
  --stop-on-failure
```

`summary.csv` 中的 `camchain_init_time_shift_s`、`camchain_init_translation_x_m`、`camchain_init_translation_y_m`、`camchain_init_translation_z_m`、`camchain_init_kalibr_translation_delta_m` 和 `camchain_init_kalibr_time_delta_s` 用于确认 camchain YAML 读取是否和 Kalibr 结果文件一致。`time_shift_init_*`、`orientation_init_*` 和 `pose_init_*` 字段用于确认独立初始化是否启用 Kalibr 风格参数，例如 `orientation_init_boundary_anchors`、`orientation_init_ceres_refine`、`orientation_init_refine_iterations`、`pose_init_fit_motion_lambda`、`pose_init_boundary_anchors`、`orientation_init_kalibr_rotation_delta_deg` 和 `orientation_init_kalibr_gravity_delta_norm`。

staged run 还会在 `summary.csv` 里生成每个 stage 的 `stage_<name>_state_decision`、`stage_<name>_state_restored`、`stage_<name>_state_initial_cost`、`stage_<name>_state_final_cost` 和 `stage_<name>_state_cost_change`。这些字段回答的是“该阶段的状态更新有没有被 optimizer 接受”，和最终几何误差是不同层级的诊断。

当前全量数据基线：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py \
  --preset current-full \
  --run-name current_full_verify_stage_fields \
  --stop-on-failure
```

已验证结果：`max_active_parameter_blocks=9755`，`max_tangent_params=43885`，最终阶段 `active_parameter_blocks=4882`，最终阶段 `tangent_params=14647`，residual mean 为 `0.196977 px / 0.167393 rad/s / 0.863287 m/s^2`。

外参平移可观性诊断：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py \
  --preset independent-final-pe-full \
  --run-name independent_final_pe_verify \
  --stop-on-failure
```

这个 preset 的最后阶段使用 `--stage-free e,bt,pbt,pe --stage-iterations 0,1,4,8`。`pe` 阶段只释放 pose 和 camera-to-IMU extrinsic，固定 bias、time shift 和 gravity。`cam_imu_2` 上它把独立初始化的外参平移差从 `0.0328967 m` 降到 `0.00606277 m`，rotation delta 从 `0.17413 deg` 降到 `0.0943398 deg`；代价是 residual mean 变为 `0.208920 px / 0.172327 rad/s / 0.885812 m/s^2`，略差于 conservative `independent-full`。继续增加 `pe` 迭代会让 translation 再次漂移，因此这个 preset 是诊断和折中基线，不是最终推荐替代 `current-full`。

如果需要看每个 Ceres accepted step 如何移动外参，可追加 iteration trace：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py \
  --preset independent-final-pe-full \
  --run-name independent_final_pe_trace \
  --extra-arg=--trace-iteration-state \
  --stop-on-failure
```

`--trace-iteration-state` 会打印 `iteration_state ... reference_translation_m=...`。当命令带有 `--kalibr-result` 时，trace 会同时输出相对 Kalibr 的 rotation、translation、time shift 和 gravity 差。`summary.csv` 会记录 `trace_custom_3_free_pe_min_reference_translation_m`、`trace_custom_3_free_pe_min_reference_translation_m_iter` 和 `trace_custom_3_free_pe_last_*`。当前 `cam_imu_2` 诊断中，最后 `pe` 阶段第 `8` 次迭代达到最小平移差 `0.0060627709213373904 m`，cost 为 `133127.52017312695`。

solver 控制面也可以从 CLI 显式配置：

```bash
--solver-linear-solver SPARSE_NORMAL_CHOLESKY
--solver-num-threads 4
--solver-initial-trust-region-radius 10000
--solver-max-trust-region-radius 10000000
--solver-min-trust-region-radius 1e-32
--solver-min-relative-decrease 0.001
--solver-absolute-cost-change-tolerance 1
--solver-absolute-step-tolerance 0.01
--solver-use-nonmonotonic-steps
--solver-max-consecutive-nonmonotonic-steps 5
```

这些参数默认保持 Ceres 原行为；命令行同时支持 `--name value` 和 `--name=value`。`run_ceres_sweep.py` 会把它们写入 `solver_*` summary 字段。Kalibr 主 cam-IMU 优化在 `IccCalibrator.optimize()` 中使用 `LevenbergMarquardtTrustRegionPolicy(1.0)`、`convergenceDeltaX=1e-2`、`convergenceDeltaJ=1` 和 `BlockCholeskyLinearSystemSolver()`；Ceres 的 trust region 半径与 LM 参数满足 `mu=1/radius`，所以这里先把半径和线性求解器暴露为实验入口，而不是把默认值硬改成某个未验证的 Kalibr 等价值。

`--solver-absolute-cost-change-tolerance`、`--solver-absolute-step-tolerance` 和 `--solver-absolute-parameter-tolerance` 是可选的 Kalibr-style absolute stopping callback，默认 `-1` 关闭。它在 successful iteration 后检查 Ceres `cost_change`、`step_norm` 和活跃参数块的实际最大系数变化；任一启用条件满足时打印 `absolute_stop ...` 并以 `USER_SUCCESS` 结束当前 solve。参数块变化由 `optimizer/parameter_delta_tracker` 统一追踪，`--trace-iteration-state` 也会逐轮打印 `parameter_delta`。注意 Kalibr 的 `deltaX` 是最小维度更新向量的最大绝对分量；Ceres 这里统计的是 active ambient parameter block 的最大实际变化，因此比 `step_norm` 更接近 Kalibr，但仍不是完全相同的 tangent-space max-coeff。

staged 模式还支持 `--stage-solver-initial-trust-region-radii`、`--stage-solver-max-trust-region-radii`、`--stage-solver-min-trust-region-radii`、`--stage-solver-min-relative-decreases`、`--stage-solver-absolute-cost-change-tolerances`、`--stage-solver-absolute-step-tolerances` 和 `--stage-solver-absolute-parameter-tolerances`。这些参数按 stage 覆盖 solver 选项；例如 `1e16,1e16,1e16,10000000` 表示只限制第 4 个 stage 的最大 trust region 半径，`-1,-1,1,-1` 表示只在第 3 个 stage 启用 `cost_change <= 1` 的 absolute stop。

新增的 `independent-capped-pe-full` preset 用来复现当前最接近 Kalibr 的独立初始化诊断：

```bash
python3 ceres_cam_imu/tools/run_ceres_sweep.py \
  --preset independent-capped-pe-full \
  --run-name independent_capped_pe_verify \
  --extra-arg=--trace-iteration-state \
  --stop-on-failure
```

该 preset 在 `independent-final-pe-full` 基础上把最后 `pe` 阶段改为 `10` 次，并通过 `--stage-solver-max-trust-region-radii 1e16,1e16,1e16,10000000` 只限制最后阶段。2026-06-16 复验输出在 `ceres_cam_imu/out/ceres_sweeps/abs_parameter_default_regression_20260616/`：`current-full` 仍为 rotation `0 deg`、translation `5.17357e-05 m`、time-shift `+0.00084628 s`；`independent-capped-pe-full` 为 translation `0.00243325 m`、rotation `0.0886821 deg`、time-shift `-0.00252163 s`、residual mean `0.209027 px / 0.172274 rad/s / 0.880630 m/s^2`。这说明限制最后 pose/extrinsic 阶段 trust region 半径可以改善独立初始化路径；它仍是诊断 preset，不替代 `current-full` 的 Kalibr 初值回归基线。

参数 delta trace 给出一个重要边界：在 `independent_final_pe_parameter_trace_20260616` 中，final `pe` 阶段外参平移差第 `10` 次最小，为 `0.00243325 m`；但 `parameter_delta` 的最小值出现在第 `20` 次，为 `0.0028641`，此时平移差已经漂到 `0.0424538 m`。因此 `--solver-absolute-parameter-tolerance` 是比 Ceres `step_norm` 更接近 Kalibr `deltaX` 的诊断工具，但不能单独替代当前固定 `pe=10` 的 best-so-far 调度。

## 当前全量数据结论

当前推荐全量数据诊断命令使用全部 camera observations、Kalibr count 对齐的 IMU 子集、Kalibr spline 频率、Kalibr 最终 time shift 作为紧 soft prior 的中心，以及 staged optimizer：

```bash
ceres_cam_imu/build/calibrate_cam_imu \
  --kalibr-corner-defaults \
  --cam /Users/wayne/Documents/work/data/cam_imu_2/cam0-camchain-640x400.yaml \
  --imu /Users/wayne/Documents/work/data/cam_imu_2/imu.yaml \
  --target /Users/wayne/Documents/work/data/cam_imu_2/aprilgrid.yaml \
  --imu-data /Users/wayne/Documents/work/data/cam_imu_2/data1.csv \
  --corners /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners.csv \
  --corner-poses /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corner_poses.csv \
  --kalibr-result /Users/wayne/Documents/work/data/cam_imu_2/cam0_640x400_corners-1-results-imucam.txt \
  --init-from-kalibr \
  --time-shift-prior-sigma 0.0001 \
  --fix-gravity \
  --pose-motion-prior \
  --pose-motion-translation-variance 10 \
  --pose-motion-rotation-variance 1 \
  --staged \
  --stage-iterations 0,1,4,5 \
  --top-residuals 8 \
  --inspect-times 368.848,376.071 \
  --inspect-window 0.01
```

当前验证点：外参平移差约 `0.052 mm`，rotation 在当前打印精度下为 `0 deg`，最终 time-shift delta 约 `+0.846 ms`，residual mean 为 `0.196977 px / 0.167393 rad/s / 0.863287 m/s^2`，accel max 约 `43.2 m/s^2`。

因此，初始化层的 time-shift prior 已经和 Docker Kalibr 零迭代对齐；orientation/gravity/gyro-bias prior、Kalibr 风格 pose spline 初始化和 camera-to-IMU 零平移初值也已经迁移。当前 `--init-from-kalibr` 全量基线和 Docker Kalibr 已经在外参平移上接近对齐；完全去掉 `--init-from-kalibr` 后，conservative 独立路径仍有约 `3.29 cm` 平移差，`independent-final-pe-full` 可把它压到约 `6.1 mm`，`independent-capped-pe-full` 在限制 final `pe` 阶段 trust region 后可压到约 `2.4 mm`，但 residual 有小幅退化。剩余差距更可能来自主优化路径、pose/bias refinement、动态段二阶平动项和外参平移可观性，而不是 time-shift 或单个初始化公式。
