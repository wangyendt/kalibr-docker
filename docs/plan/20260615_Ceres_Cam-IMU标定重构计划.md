# Ceres Cam-IMU 标定重构计划

## 背景

当前仓库已经有 Kalibr 源码、工程 Docker、离线数据和 `docs/knowhow/cam_imu_calibration_book`。这次重构不是改 Kalibr 本体，而是在新目录里用 Ceres 重新搭一条 cam-imu 优化链路，目标是让数据读取、数据处理、设计变量、residual、优化调度和结果导出都能独立阅读、独立测试、独立替换。

可用快测数据优先选 `/Users/wayne/Documents/work/data/cam_imu_2`。该目录已经包含 `aprilgrid.yaml`、`cam0-camchain-640x400.yaml`、`imu.yaml`、`0_save_timestamp.txt`、`data*.csv`、`cam0_640x400_corners.pkl` 和 Kalibr 输出的 `*-results-imucam.txt`。这使第一阶段可以先复用角点文件和 IMU csv，对照 Kalibr 的 `T_ci`、time shift、gravity 和三类 residual 统计。

## 核心问题

Kalibr 的实现依赖 ROS/catkin、Python expression graph、aslam backend 和自定义 spline/error-term 类型。新实现要把这些隐式耦合拆开：

- 轨迹模型要是独立模块，先实现 Kalibr 同型的 6D pose B-spline 查询、速度、角速度、加速度接口。
- 相机、gyro、accel、bias prior、time shift、外参这些 residual 必须分文件，不能把所有链式法则堆在一个 cost functor 里。
- 设计变量要按语义拆分：pose 控制点、bias 控制点、外参、time shift、gravity、相机内参。
- 优化器只负责组装 Ceres problem、设定 loss/solver、运行、回写参数和导出结果。
- 角点 pickle 依赖 Kalibr Python 扩展，C++ 不能直接反序列化；需要一个中间格式导出工具作为数据读取层的一部分。

## 目标

第一版目标是建成可编译、可跑小数据、可继续扩展手写 Jacobian 的 C++/Ceres 子工程。

必须具备：

- 新目录保存所有新代码，不污染 Kalibr 原有 catkin 包。
- `io/` 读取 IMU csv、相机 yaml、IMU yaml、AprilGrid yaml、Kalibr 结果文本，以及中间角点 CSV。
- `calibration_target/` 生成 AprilGrid target corner 3D 坐标。
- `trajectory/` 提供 B-spline basis、pose spline evaluate、速度、角速度、加速度接口。
- `variables/` 定义各类设计变量和 Ceres parameter block 的局部参数化。
- `residuals/` 分离 camera reprojection、gyro、accelerometer、bias motion prior；第一版允许部分 residual 用 AutoDiff，但接口要给 analytic Jacobian 留位置。
- `optimizer/` 分离 problem 构建、solver 配置、结果回写、统计和导出。
- `tools/` 提供 Kalibr corner pickle 到轻量 CSV 的导出脚本，以及与 Kalibr 输出做结果对比的脚本。
- `README.md` 写清楚构建、数据准备、运行和对比命令。

## 范围

第一阶段只覆盖单相机、单 IMU、pinhole+radtan、AprilGrid、已提角点输入。多相机、多 IMU、IMU scale/misalignment、size effect、rolling shutter、直接从图像检测 AprilTag 暂不作为首个可跑闭环，但目录和接口保留扩展点。

第一阶段优先复现 ordinary model：

- camera residual：`e = observed_pixel - project(T_cb * T_bw(t + dt) * p_target)`
- gyro residual：`e = R_ib * omega_b(t) + b_g(t) - z_g`
- accel residual：`e = R_ib * (R_bw(t) * (a_w(t) - g_w) + alpha_b(t) x r_b + omega_b(t) x (omega_b(t) x r_b)) + b_a(t) - z_a`
- bias motion prior：按 bias spline 一阶导平滑约束，先用离散相邻控制点差分，后续替换为 Kalibr 同型积分二次型。

## 方案

新目录建议为 `ceres_cam_imu/`：

| 子目录 | 职责 | 关键产出 |
|---|---|---|
| `include/ceres_cam_imu/core` | 基础类型、SE(3)/SO(3)、配置结构 | `types.h`, `so3.h`, `se3.h` |
| `include/ceres_cam_imu/initialization` | 标定初值 | pose spline 最小二乘拟合、gyro norm 互相关 time shift prior |
| `include/ceres_cam_imu/io` | 数据读取与结果解析 | IMU csv、yaml、corner CSV、Kalibr result parser |
| `include/ceres_cam_imu/processing` | 数据处理与筛选 | Kalibr-style IMU edge trim、frame limit、corner count、time range |
| `include/ceres_cam_imu/target` | 标定板几何 | AprilGrid corner 生成 |
| `include/ceres_cam_imu/camera` | 相机投影模型 | pinhole+radtan projection 和 Jacobian |
| `include/ceres_cam_imu/trajectory` | 连续时间轨迹 | uniform B-spline basis、pose/bias spline |
| `include/ceres_cam_imu/variables` | Ceres 参数块封装 | pose control、bias control、extrinsic、gravity、time shift |
| `include/ceres_cam_imu/residuals` | 每类 residual 独立文件 | camera、gyro、accel、prior |
| `include/ceres_cam_imu/optimizer` | problem 组装与求解 | builder、options、summary、export |
| `src/` | 模块实现 | 与 include 一一对应 |
| `tools/` | 数据转换与对比脚本 | corner 导出、Kalibr 对比 |
| `apps/` | CLI 程序 | `calibrate_cam_imu`, `compare_kalibr_result` |
| `tests/` | 数值验证 | projection、SO3、B-spline、residual finite difference |

效率原则：

- 数据读取阶段一次性转成紧凑结构，residual 构造阶段不做字符串解析和动态查找。
- B-spline 查询返回 active 控制点索引和 basis 权重，residual 只连接 active window。
- residual 内避免堆分配，固定维度用 Eigen stack matrix。
- 大量 residual 的构造做 sampling/stride 参数，快测先限制帧数和 IMU 数，正式跑再放开。
- 手写 Jacobian 分阶段落地：先 camera projection 与 SO(3) 公共 Jacobian，再 gyro/accel，最后 pose spline 对控制点的完整链式 Jacobian。

## 执行路径

| 阶段 | 目标 | 产出 | 判断标准 |
|---|---|---|---|
| P0 | 固定计划和目录 | 本计划、`ceres_cam_imu/` 骨架 | 目录清晰，README 能说明范围和命令 |
| P1 | 数据层跑通 | YAML/CSV/Kalibr result reader、corner export 脚本 | 能读取 `cam_imu_2` 的配置、IMU 和角点中间格式 |
| P2 | 数学基础层 | SO3/SE3、camera projection、AprilGrid、B-spline basis | 单元测试或 CLI check 能做基本数值验证 |
| P3 | residual 层 | camera/gyro/accel/bias prior cost functions | 每类 residual 独立文件，可用 finite difference 验证 |
| P4 | 优化闭环 | problem builder、solver、参数更新、结果导出 | 小样本能完成 Ceres solve 并输出 T_ci/time shift/gravity/residual |
| P5 | Kalibr 对比 | Docker/Kalibr 结果解析与差异报告 | 与 `*-results-imucam.txt` 给出矩阵、角度、平移、time shift 和 residual 差异 |
| P6 | 手写 Jacobian 增强 | analytic cost function 替换核心 AutoDiff | 数值差分通过，速度优于纯 AutoDiff |

## 风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| 角点 pickle 绑定 Kalibr Python 类型 | C++ 不能直接读取现有角点 | 在 Kalibr Docker/环境里导出 CSV，中间格式作为新工程正式输入 |
| Kalibr pose spline 初始化复杂 | 初值差会导致优化无法收敛 | 已支持 pose CSV 最小二乘初始化和 gyro norm 互相关 time shift prior，后续补 Kalibr 同型 spline/bias prior |
| 完整手写 Jacobian 工作量大 | 影响首个闭环速度 | 模块接口先支持 `AutoDiff` 和 `SizedCostFunction` 双实现，按 residual 分批替换 |
| 本机缺少 Ceres/Eigen/yaml-cpp | 本地无法立即编译 | 提供 Dockerfile 或在现有 camera-calibration Docker 中安装 `libceres-dev` 进行验证 |
| 与 Kalibr residual 权重不一致 | 对比结果偏差大 | 明确复用 Kalibr 的离散噪声、robust loss、IMU edge trim 和 bias prior 设定 |

## 检查点

- `cmake -S ceres_cam_imu -B ceres_cam_imu/build` 能给出明确依赖检查。
- `calibrate_cam_imu --help` 展示输入路径、采样、solver 和输出参数。
- `tools/export_kalibr_corners.py` 能在 Kalibr Python 环境中把 `cam0_640x400_corners.pkl` 导出为 CSV。
- `calibrate_cam_imu` 能在 `cam_imu_2` 的小采样上构建 problem，不出现空 residual。
- `compare_kalibr_result` 能解析 Kalibr 的 `results-imucam.txt` 并输出差异表。

## 当前进展

截至 2026-06-16，P0-P5 的第一版闭环已经完成到可验证状态：`ceres_cam_imu/` 可本机 CMake build/ctest；Docker 镜像 `kalibr-camera-calibration:20.04` 能把真实 Kalibr corner pickle 导出为 CSV；`check_dataset` 能读取真实 `cam_imu_2` 配置、IMU、corner、pose 和 Kalibr result。

优化链路已经能构建真实 camera/gyro/accel/bias prior problem。pose 控制点初始化已从最近邻控制点填充改为按 B-spline basis 的稀疏最小二乘拟合，真实 1455 帧 pose 的拟合 RMS 为 `0.000652881 m` / `0.000936701 rad`。固定 pose、bias、time shift 和 gravity 时，20 帧与 200 帧样本的外参结果都能贴近 Kalibr，200 帧平移差约 `8.56926e-05 m`。

camera time shift prior 已按 Kalibr 的 gyro norm 互相关思路接入。`cam_imu_2` 默认估计为 `-0.552334 s`，距离 Kalibr 最终结果 `-0.546504 s` 约 `5.83 ms`；固定该估计值做 200 帧受限优化时，外参平移差约 `7.94528e-05 m`。

bias motion prior 已从相邻差分替换为每 segment 的 B-spline 一阶导数积分等价 residual。实现用 `sqrt(Q_s)` 作为固定 Jacobian，Ceres residual 平方和等价于 `integral(dot(b)^T W dot(b) dt)`；`ctest` 覆盖常值 bias 零 prior 和随机控制点数值积分一致性。

camera reprojection residual 已从 AutoDiff 替换为 analytic `SizedCostFunction`。手写 Jacobian 覆盖 `T_c_b`、camera time shift 和 6 个 active pose control blocks，并用 finite difference 在 `ctest` 中逐块校验。真实 200 帧受限优化保持 `7.94528e-05 m` 的 Kalibr 平移差。

gyroscope residual 已从 AutoDiff 替换为 analytic `SizedCostFunction`。实现使用 SO(3) left Jacobian：`omega_b = -J_l(r) * r_dot`，手写覆盖 IMU 外参旋转、6 个 pose control blocks 和 6 个 gyro bias control blocks；`ctest` 对全部 13 个参数块做 finite-difference 校验。真实 200 帧、500 个 IMU residual、staged、`--time-shift-prior-sigma 0.0001` 的 smoke 结果保持 `0.071 mm` / `0 deg`。

accelerometer residual 也已从 AutoDiff 替换为 analytic `SizedCostFunction`。实现覆盖 IMU lever arm、IMU 外参旋转、gravity、6 个 pose control blocks 和 6 个 accel bias control blocks；对 pose control 的旋转分支显式展开 `R_bw(a_w-g_w)`、`alpha_b x r_b` 和 `omega_b x (omega_b x r_b)` 三条链。`ctest` 对全部 14 个参数块做 finite-difference 校验，真实 staged smoke 结果与替换前保持一致。

gravity design variable 已改成更接近 Kalibr 默认 `EuclideanDirection` 的形式：Ceres 默认在三维 gravity block 上挂 `SphereManifold<3>`，固定重力模长、只优化 2 维方向；`--estimate-gravity-length` 才退回普通三维欧式向量。`buildCalibrationProblem` 会在 summary 中打印 `gravity_tangent`，单测覆盖默认 `2`、估计模长 `3`、固定 gravity `0` 三种情况。

pose motion prior 已实现为通用 Euclidean spline derivative integral residual 的 6 维版本，默认关闭，通过 `--pose-motion-prior` 显式启用。200 帧、`--imu-stride 10`、`--max-imu-residuals 100` 的 dry-run 中，problem 会添加 `134` 个 active pose prior，同时保留全段 bias prior `500 + 500`。固定 camera extrinsic 和 gravity 的受控验证中，默认 pose prior 不改变 Kalibr 外参，time shift 差约 `1.36e-05 s`。

自由变量阶梯实验显示，固定 pose/gravity、放开 bias/time shift 时，200 帧结果仍稳定；但固定 gravity、放开 pose/bias/time shift 时，外参仍会漂移。新 pose motion prior 在默认 Kalibr 风格大方差下不会解决这个退化，200 帧全自由外参小样本仍可偏到约 `5.9 mm` / `3.58 deg`。这解释了为什么不能直接跑全量自由优化，而需要分阶段解冻、外参冻结 warm-start 或更接近 Kalibr 的初始化/约束调度。

conservative staged optimizer 已接入 `optimizer/staged_optimizer.*` 和 CLI `--staged`。当前 preset 每阶段重建 problem 并复用同一份 state，顺序为：固定 motion 求外参、固定 pose/extrinsic 求 time+bias、固定 extrinsic 放开 pose/time/bias、最后固定 refined pose 求外参/time/bias。真实 200 帧、`--imu-stride 10`、`--max-imu-residuals 100`、`--pose-motion-prior`、每阶段 5 次迭代时，直接全自由结果为 `1.85 mm` / `1.94 deg`，staged 结果为 `0.090 mm` / 当前打印精度 `0 deg`，time shift 差约 `-2.32 ms`。

把 IMU residual 放大到 500 后，staged 外参仍稳定，但 time shift 会暴露新的约束问题。不固定 time shift 时，外参约为 `0.346 mm` / `0.0128 deg`，但 time shift 偏到 `+39.6 ms`；显式传入 `--fix-time-shift` 后，staged 会尊重用户固定标志，外参为 `0.071 mm` / 当前打印精度 `0 deg`，time shift 保持互相关初值，差 Kalibr 约 `-5.83 ms`。

time-shift prior residual 已作为独立文件接入，通过 `--time-shift-prior-sigma` 启用，prior 值默认取当前初始化或互相关估计值。500 个 IMU residual、`sigma=0.0001 s` 时，time shift 被稳定在互相关初值附近，最终差 Kalibr 约 `-5.07 ms`，外参仍为 `0.071 mm` / 当前打印精度 `0 deg`。

Docker 环境已重新确认可用：`docker ps` 可连接 daemon，镜像 `kalibr-camera-calibration:20.04` 存在，容器可以从 `cam0_640x400_corners.pkl` 复现导出 `1455` 帧 observations。进一步用容器直接跑通了 Kalibr `--corner_file/--image_timestamp_file/--imu_data_file` 的官方入口；只读挂载会在最后写 `*-camchain-imucam.yaml` 时失败，但 problem build 和 residual statistics 已完整输出。该 zero-iteration run 确认官方链路默认使用 `pose-knots-per-second=100`、`bias-knots-per-second=50`，在 `--timeoffset-padding 0.04` 下构建出 `9758` 个 design variables、`693126` 个 error terms，Jacobian 规模为 `1432108 x 43893`；初始 time shift 为 `-0.550524 s`，raw residual mean 为 `0.392372 px` / `0.175486 rad/s` / `1.06985 m/s²`。源码复核进一步确认 Kalibr 会把 `--timeoffset-padding S` 作为首尾各 `2*S` 的 pose spline 时间扩展；Ceres 已按该规则修正，并且 CLI 同时接受 `--time-padding` 和 Kalibr 原名 `--timeoffset-padding`。Ceres dry-run 为 `4874` 个 order-6 pose controls 和 `2435 + 2435` 个 bias priors，并已在 build summary 中输出同口径规模：`parameter_blocks=9758`、`tangent_params=43893`、`kalibr_style_error_terms=693126`，与 Docker Kalibr 的 design variable、Jacobian column 和 error term 基准一致。由于镜像是 amd64 而主机是 arm64，完整 Kalibr 重跑可行但会走模拟，当前更高效的策略是继续复用已有 Kalibr result 文本作为基线，把 Docker 主要用于 pickle 导出、Kalibr 参数确认和必要时的官方环境复验。

随后新增 `ceres_cam_imu/tools/run_kalibr_docker.py`，把上述官方入口固化为可复跑 wrapper。脚本默认把 `cam_imu_2` 输入只读挂载到 `/data`，再复制到 `/out/input` 后运行 Kalibr，避免原始 raw 命令因 `/data:ro` 写回结果而退出 `1`；同时保存 `command.txt`、`kalibr_raw.log`、`kalibr_clean.log` 和 `summary.txt`。`smoke_iter0_staged_verify` 复验零退出，summary 为 `22929` IMU readings、`22929 + 22929` accel/gyro error terms、`9758` design variables、`693126` error terms、Jacobian `1432108x43893`、time shift `-0.5505243893928822 s`、raw mean `0.392372 px` / `0.175486 rad/s` / `1.06985 m/s²`。

staged/time-shift prior 验证已扩大到 500 帧和 1000 个 IMU residual。该 run 包含 `234585` 个 camera residual、`1000 + 1000` 个 IMU residual、`500 + 500` 个 bias prior、`408` 个 pose prior 和 `1` 个 time prior。四阶段优化最终 cost 为 `41986.85`，外参与 Kalibr 的平移差为 `0.000124314 m`，旋转差在当前打印精度下为 `0 deg`，time shift 差为 `-0.00370631 s`。这说明核心 analytic residual 全部落地后，中等扩展规模仍能稳定外参；但 time shift 仍受强 prior 锚定，尚未等价于 Kalibr 的最终 refinement。

全数据 staged 已跑通。未裁剪 IMU 的完整 problem 包含 `647266` 个 camera residual、`24928 + 24928` 个 IMU residual、`500 + 500` 个 bias prior、`1000` 个 pose prior 和 `1` 个 time prior。四阶段、每阶段最多 10 次迭代时，最终 cost 为 `489682.3`，外参与 Kalibr 的平移差为 `0.000220967 m`，旋转差为 `0.00726292 deg`，time shift 差为 `0.00264592 s`。这证明新实现已经可以装配并求解完整 `cam_imu_2` 数据面；但 pose stage 仍因迭代上限结束，不能把该结果视为最终收敛的 Kalibr 等价标定。

final residual statistics 已接入 `optimizer/residual_statistics`。CLI 会在 solve 后输出 camera reprojection、gyro、accel 的 raw 物理单位分布和 Ceres normalized 分布。20/10 kps 全数据 staged 10-iter 的 mean residual 为 `0.318336 px`、`0.163418 rad/s`、`0.797414 m/s²`；与 Kalibr report 的 `0.203681 px`、`0.170054 rad/s`、`0.816666 m/s²` 相比，gyro 和 accel 已同量级，camera residual 偏高。Docker/Kalibr smoke run 确认 Kalibr 默认 pose/bias knot 频率是 `100/50`，因此 20/10 kps 的相机残差问题主要来自样条频率不匹配。

结构化结果导出已接入 `io/calibration_result_writer.*` 和 CLI `--output-result`。写出的 YAML-like 文件包含 `T_c_b`、`T_b_c`、camera-to-IMU time shift、gravity、pose/bias spline metadata、分组 residual statistics、top accel outlier 和可选 `kalibr_delta`。20 帧受限 smoke run 已验证 `/tmp/ceres_cam_imu_result_smoke.yaml` 能落盘并读回关键字段。这补齐了 P4 中“参数更新和结果导出”的基本闭环，后续全数据 sweep 可以用结构化结果文件做对比，而不是只解析终端日志。

P5 的差异报告也从单向解析 Kalibr 文本推进到双向文件对比。新增 `io/calibration_result_reader.*` 读取 Ceres 输出的 YAML-like 文件，`compare_kalibr_result` 现在支持 `--ceres-result result.yaml`，会复算 Ceres/Kalibr 的 rotation、translation、time shift、gravity 差异，并输出三类 residual mean 的 `ceres - kalibr` 差值。20 帧 smoke 结果文件 `/tmp/ceres_cam_imu_compare_smoke.yaml` 的对比输出为：rotation `0.00276155 deg`、translation `0.000141644 m`、time shift `0 s`、gravity `0`，residual mean delta 为 `-0.0760614 px` / `+0.0578465 rad/s` / `+2.34804 m/s²`；复算 delta 与结果文件内嵌 `kalibr_delta` 一致。

为减少后续手工日志整理，新增 `tools/run_ceres_sweep.py`。它能按 preset 或自定义 variant 生成命令、自动写 `--output-result`、调用 `compare_kalibr_result`，并把 problem size、几何差异、residual mean 和输出路径聚合到 `summary.csv`。`smoke_verify` 默认 preset 已验证通过，结果复现 20 帧结构化 compare：rotation `0.00276155 deg`、translation `0.000141644 m`、time shift `0 s`，residual mean delta 为 `-0.0760614 px` / `+0.0578465 rad/s` / `+2.34804 m/s²`。这个结果只证明 sweep/compare 管线可复现，不改变“尚未完全精度对齐”的判断。

`current_full_verify_stage_fields` 又用同一个 runner 复验了当前全量基线。结果为 `0.196977 px` / `0.167393 rad/s` / `0.863287 m/s²`，外参平移差 `5.17357e-05 m`，time shift 差 `0.00084628 s`；相对 Kalibr report 的 residual delta 为 `-0.00670377 px` / `-0.00266096 rad/s` / `+0.0466207 m/s²`。脚本已补充 staged 维度字段：通用 `active_parameter_blocks/tangent_params` 代表最后一个 stage，`max_active_parameter_blocks=9755`、`max_tangent_params=43885` 和 `stage_*` 字段用于检查全 staged 过程或具体 stage。

Ceres 侧也新增了 `--kalibr-corner-defaults`，把 Kalibr `--corner_file` 官方入口的主要默认值固化为一个可复跑 preset：`pose/bias kps=100/50`、`max_iterations=30`、`timeoffset_padding_s=0.04`、`imu_trim_edge_count=1000`、camera/gyro/accel Cauchy width `10`。显式 CLI 参数仍可覆盖 preset。真实 `cam_imu_2` dry-run 输出 `camera=647266`、`gyro=22929`、`accel=22929`、`gyro_priors=2435`、`accel_priors=2435`、`parameter_blocks=9758`、`tangent_params=43893`、`kalibr_style_error_terms=693126`，与 Docker Kalibr zero-iteration problem-size 基准同口径。solver 停止条件也已显式暴露为 `--solver-function-tolerance`、`--solver-gradient-tolerance`、`--solver-parameter-tolerance` 和 `--solver-initial-trust-region-radius`，后续 sweep 可以把停止条件差异和 residual/调度差异分开记录。

Ceres robust loss 映射已修正。Kalibr `CauchyMEstimator(10)` 中的 `10` 是 squared-error 分母 `sigma²`，而 Ceres `CauchyLoss(a)` 的等价权重为 `1/(1+s/a²)`，所以 Ceres 需要传 `sqrt(10)`。修正后，在 Kalibr 默认 `--pose-kps 100 --bias-kps 50`、固定 Kalibr time shift/gravity、全数据 staged 10-iter 下，外参与 Kalibr 的差异为 `0.000106814 m` / 当前打印精度 `0 deg`，time shift 差为 `0 s`；reprojection mean 为 `0.206192 px`，gyro mean 为 `0.168073 rad/s`，已经贴近 Kalibr report。

100/50 kps 下的主要缺口一度是 accelerometer 离群值。默认大方差 pose prior 的 accel median 为 `0.166931 m/s²`，但 mean 为 `4.70951 m/s²`、max 为 `7881.52 m/s²`，说明少量高频 pose spline 尖峰主导了均值。对 pose motion prior 方差做粗扫后，当前最佳点是 `--pose-motion-translation-variance 10 --pose-motion-rotation-variance 1`：外参差 `0.000105844 m` / 当前打印精度 `0 deg`，reprojection mean `0.210360 px`，gyro mean `0.164879 rad/s`，accel mean `0.933974 m/s²`，accel max `98.8465 m/s²`。继续收紧到 `1 / 0.1` 会让 camera mean 增到 `0.213746 px`，accel mean 也回升到 `0.970222 m/s²`，因此 `10 / 1` 是当前已测折中点。

在 `10 / 1` 上放开 time shift 后，外参仍稳定，平移差为 `9.80143e-05 m`，三类 mean residual 为 `0.211943 px`、`0.165775 rad/s`、`1.04515 m/s²`，但 time shift 偏到比 Kalibr 小 `6.361 ms`。加入 `--time-shift-prior-sigma 0.0001` 后，time shift 差收敛到 `-0.711 ms`，外参差 `9.98477e-05 m`，三类 mean residual 为 `0.211103 px`、`0.165107 rad/s`、`0.963145 m/s²`。这说明 `10 / 1` 已经把运动正则调到可用范围，剩余问题主要是 time-shift refinement 的等价调度，而不是外参或 residual 前向模型。

为定位剩余 accel max，`residual_statistics` 已增加 top-k IMU outlier 输出。当前 `0.1 ms` time prior run 的最坏 5 个 accel residual 全部集中在 `411.577 s` 到 `411.585 s`，rank 1 残差为 `137.943 m/s²`。分解量显示 pose world acceleration `137.962 m/s²`、gravity-corrected body acceleration `131.669 m/s²`，而 angular-accel lever 和 centripetal lever 均为 `0`。这把问题定位到局部 pose spline 平动二阶导数尖峰，而不是 IMU lever arm 或角速度项。

`--inspect-time 411.581` 进一步确认该尖峰位于 `coeff_start=4855` 的 pose spline 段内，segment dt 约 `0.0100009 s`，二阶 basis 权重为 `5.82, 2510.58, 1414.33, -9366.35, 4418.31, 1017.31`。最近 camera query pose 在 `411.559 s` 和 `411.592 s`，outlier 位于最后几帧相机观测之间；`411.562-411.599 s` 的 IMU 窗口 measured accel norm 稳定在 `9.77-9.78 m/s²`。因此下一阶段应验证尾段/局部 pose control 平滑策略，而不是继续扩大全局 prior 或怀疑 IMU 数据。

局部 pose-motion scaling 已验证为负例：在 `411.581 s` 附近 `0.05 s` 半窗口内把 translation variance 缩小到 `0.01x` 可以把该点 acceleration 压到 `0.076 m/s²`，但尖峰会转移到 `411.633 s` 和 `411.812 s`，accel max 升到 `431.112 m/s²`。因此单点局部 smoothing 只保留为诊断工具，不作为默认收敛策略。

更关键的 Kalibr 对齐点是 IMU sample 选取和 spline padding。Docker Kalibr zero-iteration run 实际读取 `22929` 个 IMU readings，并加入 `22929 + 22929` 个 accel/gyro residual；Ceres 未裁剪时使用完整 `24928` 个 IMU samples。源码复核确认 Kalibr 在 `--imu_data_file` 下默认 `trimImuEdgeCount=1000`，skip 条件是 `index < trim` 或 `index > numMessages - trim`，因此保留右边界样本。Ceres reader 已改成同一 inclusive upper-bound 语义，`--imu-trim-edge-count 1000` 可构建 `22929 + 22929` 个 IMU residual。随后 Ceres `makeSplineForTimes` 按 Kalibr `--timeoffset-padding S` 首尾各扩 `2*S` 的规则修正，dry-run 构建出 `4874` 个 pose controls、`2435 + 2435` 个 bias priors 和 `4636` 个 pose priors；这与 Kalibr zero-iter 报告的 `4869` pose spline segments 和 `2435` bias segments 对齐。该设置下 `0,1,4,5` 全数据 result 为：外参差 `0.0000517357 m` / 当前打印精度 `0 deg`，time shift 差 `+0.846 ms`，reprojection/gyro/accel mean `0.196977 px` / `0.167393 rad/s` / `0.863287 m/s²`，accel max `43.1613 m/s²`。

数据处理层已从 reader/CLI 中抽出第一版。新增 `include/ceres_cam_imu/processing` 和 `src/processing`，把 Kalibr-compatible IMU edge trim、frame limit、corner count 和 time range helpers 放到独立模块；`readImuCsv` 保留兼容参数，但内部委托给 processing，`check_dataset` 直接打印 trim summary。真实 `cam_imu_2` 复验显示 `24928 -> 22929`、`first_kept_index=1000`、`last_kept_index=23928`，全数据 dry-run 仍为 `9758` parameter blocks、`43893` tangent params、`693126` Kalibr-style error terms。这一改动提升模块边界，不改变数值基线。

剩余重点转为 P6 和 Kalibr 等价性收敛：核心 camera/gyro/accel residual 已全部 analytic，100/50 kps、正确 robust loss、`10 / 1` pose prior、`0.1 ms` time prior、Kalibr-count IMU subset 和 Kalibr-style time padding 已让三类 mean residual 都接近 Kalibr report。Kalibr 的 IMU sample 选取规则和 `--timeoffset-padding` 边界规则已经源码级确认并复刻到 Ceres；裁剪后 `367-374 s` 动态段 outlier 也已用 `--inspect-times` 做过多点局部诊断，确认仍由 pose spline 平动二阶导主导。直接把 accelerometer residual 切到 Huber 是负例：accel max 会显著下降，但 camera residual、time shift 和外参都会变差。stage-specific iteration cap 已接入 `--stage-iterations N0,N1,N2,N3`，第一轮全量 sweep 表明 `0,1,4,5` 是当前最佳折中：`0.197 px` / `0.167 rad/s` / `0.863 m/s²`，accel max `43.2 m/s²`，外参差 `0.052 mm`。随后 `--stage-free` 支持用 `p/b/e/t/g` 自定义每阶段自由变量；`e,tb,pbt,eb` 和 `e,tb,pbt,e` 说明 final stage 是否放开 time shift 影响很小，固定全局 time shift 的 `e,b,pb,e` 则略降 camera mean 和 accel max、略升 accel mean。`--stage-pose-translation-variances` 和 `--stage-pose-rotation-variances` 进一步支持每阶段 pose prior 权重；把第三阶段从 `10 / 1` 收紧到 `5 / 0.5` 可把 accel mean/max 小幅降到 `0.858 m/s²` / `42.4 m/s²`，放松到 `20 / 2` 则 camera/gyro 稍好但 accel 变差。新的 `--pose-motion-all-segments` 可把 pose prior 从 active-only 扩到 Kalibr 同型的全 spline scope，dry-run 中 `pose_priors` 从 `4636` 变为 `4869`，但全量结果基本中性。`--stage-time-shift-prior-sigmas` 则把 time-shift prior 宽度也纳入每阶段调度；`0.0001,0.0001,0.0002,0` 可把 reprojection mean 降到 `0.1957 px`、外参差降到 `0.039 mm`，但 time shift 漂到 `+1.75 ms`、accel mean 升到 `0.865 m/s²`。`--stage-pose-motion-orders` 又补上了每阶段 pose prior 导数阶数控制；第三阶段改成一阶的 `2,2,1,2` 是明确负例，camera mean 降到 `0.1801 px` 但 accel mean/max 爆到 `242 m/s²` / `14070 m/s²`。Kalibr-style pose 初始化正则也已补成可选项：`--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors` 会加入二阶导数积分初始化正则和首尾边界 pose anchor，但全量结果为 `0.201 px` / `0.169 rad/s` / `0.846 m/s²`、accel max `43.7 m/s²`，不优于当前默认基线。这些结果说明 joint pose/time/bias stage 是有效调度点，但还不是完整解法；velocity-level smoothing 和初始化阶段 `1e-4` motion smoothing 都不能作为默认解法。

## 下一步

下一步以 `tools/run_ceres_sweep.py` 为记录入口，以 `--stage-free e,tb,pbt,ebt`、`--stage-iterations 0,1,4,5`、per-stage pose prior 方差、per-stage time prior sigma 和固定二阶 pose motion order 为基线，继续扫 pose/time/bias 联合阶段的权重、迭代预算和更接近 Kalibr 的 pose smoothing 形式；一阶 smoothing、final extrinsic stage time 解冻和 `1e-4` pose-fit 初始化正则都不是当前主要解法。
