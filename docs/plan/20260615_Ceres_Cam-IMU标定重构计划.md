# Ceres Cam-IMU 标定重构计划

## 背景

这次重构的目标是把 cam-IMU 标定从 Kalibr/ROS/catkin 的耦合实现中拆出来，形成一条 C++/Ceres 的独立求解链路。Kalibr 仍然是重要基线，但它在新链路里的角色应限制为**评测 oracle**和**历史输入转换环境**，不能成为生产独立标定的初始化依赖。

当前主线已经从“能不能跑通”推进到“能不能解释精度边界”：Ceres 主程序已经能读统一 CSV/YAML 输入、独立初始化、装配全量 problem、求解并写结构化结果；实验文档开始用 12 组 production 和 2 组 TUM 双目数据记录精度/速度/扩展 IMU 一致性。

## 目标状态

- **独立求解**：`calibrate_cam_imu` 不依赖 Kalibr Docker，不需要先转成 Ceres CSV 后再手工跑；外层脚本可一键完成转换与标定。
- **清晰口径**：生产独立命令不传 `--kalibr-result`；该参数只用于离线对比或热启动诊断。
- **模型覆盖**：相机模型覆盖 Kalibr 常见 pinhole/omni/eucm/ds 组合；IMU 覆盖 ordinary、scale-misalignment、scale-misalignment-size-effect。
- **生产速度**：核心 camera/gyro/accel/扩展 IMU residual 使用手写解析 Jacobian，避免生产路径上的 per-residual 数值差分。
- **可复现实验**：production、TUM、扩展 IMU、多 camera 的结果都写入实验文档，并能通过常用命令复跑。

## 当前架构

| 模块 | 职责 | 当前状态 |
|---|---|---|
| `io/` | camchain/IMU/target YAML、IMU CSV、corner CSV、Ceres result、Kalibr result parser | 已可用 |
| `processing/` | IMU edge trim、frame limit、corner count、time range | 已拆出第一版 |
| `camera/` | 多相机模型投影和 Jacobian | 已覆盖主要模型并有差分测试 |
| `trajectory/` | order-6 pose/bias B-spline、导数查询 | 已用于全量标定 |
| `initialization/` | pose fit、gyro-norm time shift prior、orientation/gravity prior | 已用于独立标定 |
| `residuals/` | camera、gyro、accel、bias prior、pose prior、time prior、扩展 IMU | 核心 residual 已解析 Jacobian |
| `optimizer/` | problem build、solver、staged/summary/statistics | 单相机 staged 已可用；多相机 staged 待补 |
| `tools/` | Kalibr Docker wrapper、Ceres sweep、输入转换、TUM two-stage | 已可复现实验；原生检测仍待补 |

## 进度

| 阶段 | 目标 | 当前判断 |
|---|---|---|
| P0 数据闭环 | 真实数据读取、corner 导出、dry-run problem build | 完成 |
| P1 数学基础 | SO3/SE3、B-spline、AprilGrid、camera projection | 完成 |
| P2 普通 IMU 标定 | camera/gyro/accel/bias/time/pose residual 与解析 Jacobian | 完成 |
| P3 Kalibr 口径对齐 | IMU trim、time padding、gravity manifold、Cauchy loss、problem-size summary | 完成 |
| P4 结构化结果 | `--output-result`、结果读回、residual statistics、top outlier | 完成 |
| P5 批量评测 | 12 组 production、热启动、TUM、扩展 IMU 文档化 | 完成第一轮 |
| P6 生产补强 | 原生输入转换、多 camera staged、严格 native speed benchmark | 进行中 |

## 关键决策

- `--corner-defaults` 是正式 preset 名称；`--kalibr-corner-defaults` 只作为旧脚本兼容别名。
- `--kalibr-result` 不属于独立标定主路径。只有 `--init-from-kalibr` 热启动或 `compare_kalibr_result` 评测时才读取 Kalibr 结果。
- pkl/bag/euroc 目前通过 `prepare_ceres_inputs.py` 转成统一 CSV/YAML；转换阶段仍复用 Kalibr Docker/ROS 环境，求解阶段不依赖。
- TUM 使用二阶段策略：第一阶段估全局块和扩展 IMU intrinsic；第二阶段从 Ceres result 初始化，固定全局块，用 100/50 kps refine pose/bias。
- 多 camera 当前支持 shared camchain + 多 `--corners` 的非 staged joint 优化；staged multi-camera 另列为后续任务。

## 当前证据

- 12 组 production 独立标定全部 `CONVERGENCE`，reprojection 与 Kalibr 基本一致，外参平移差 `1.3-4.7 mm`，旋转差多数 `<0.02°`。
- 热启动从 Kalibr 解出发仍会漂 `0.19-2.63 mm`，说明剩余毫米级差异包含两套优化问题的固有差。
- 扩展 IMU 三组 production 对比中，`M_a/M_g` 相对差约 `1e-3`，`A_g/C_g` Frobenius 约 `1e-3`。
- TUM 双目二阶段后，gyro residual 达 `0.00120-0.00121 rad/s`，accel residual 达 `0.0222 m/s^2`，与 Kalibr 同量级。
- time-shift 初始化同步 bug 已修复：`initial_camera_time_shift_s` 与 `initial_camera_time_shifts[0]` 保持一致，March 组不再 `NO_CONVERGENCE`。

## 剩余工作

| 优先级 | 工作 | 目标 |
|---|---|---|
| P0 | 原生 AprilTag/输入转换方案 | 让 pkl/bag/euroc 之外的生产输入不再依赖 Kalibr Docker/ROS 做角点导出 |
| P0 | 多 camera staged optimizer | 让 TUM/双目也能使用 staged 调度，而不是仅 joint |
| P1 | native-vs-native 速度评测 | 在同一 Linux native 环境里重跑 Ceres 与 Kalibr，给出可信速度倍率 |
| P1 | 外参弱方向毫米级差异分析 | 对齐 LM/停止条件/变量缩放，判断是否值得继续追 sub-mm |
| P1 | TUM 二阶段产品化 | 把 two-stage 参数收敛成稳定 preset，并补失败诊断 |
| P2 | 输入转换边界继续下沉 | 把 observation selection、运行配置描述从 CLI 里继续抽出 |

## 验收线

生产独立路径的验收标准是：不传 `--kalibr-result`，Ceres 自己初始化并收敛；输出结构化 result；必要时再用 `compare_kalibr_result` 做离线评测。实验路径可以读取 Kalibr result，但必须在文档和命令中标明“评测/热启动”，不能和生产命令混写。
