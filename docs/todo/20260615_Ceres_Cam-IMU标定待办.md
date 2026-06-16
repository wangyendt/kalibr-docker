# Ceres Cam-IMU 标定待办

## 背景

`ceres_cam_imu/` 已经完成第一版可编译骨架：数据读取、AprilGrid、pinhole+radtan、order-6 uniform B-spline、camera/gyro/accel/bias prior residual、Ceres problem builder、Kalibr-style gravity direction manifold、结构化结果导出和 CLI 都能通过本机编译。真实 `cam_imu_2` 的 yaml、IMU csv、Kalibr result 文本、Kalibr corner pickle 导出的 CSV 都已读通。

Docker Desktop 启动后，已有镜像 `kalibr-camera-calibration:20.04` 可用于反序列化 Kalibr Python pickle，也能直接复跑 Kalibr `--corner_file/--image_timestamp_file/--imu_data_file` 官方入口。2026-06-16 复验确认 `docker ps` 可连接 daemon，镜像存在，临时导出仍能从原始 pickle 得到 `1455` 帧 observations；官方 zero-iteration run 在 `--timeoffset-padding 0.04` 下可构建 `9758` 个 design variables / `693126` 个 error terms，Jacobian 为 `1432108 x 43893`，time shift 初始化为 `-0.550524 s`，raw residual mean 为 `0.392 px` / `0.175 rad/s` / `1.070 m/s²`。当前真实数据链路已经走到全数据 staged 验证，Ceres problem 可装配 `647266` 个 camera residual 和 Kalibr 裁剪后的 `22929 + 22929` 个 IMU residual；build summary 已输出 `parameter_blocks=9758`、`tangent_params=43893`、`kalibr_style_error_terms=693126`，与 Docker Kalibr 的 problem-size 基准同口径对齐。Kalibr-style gyro norm 互相关 time shift prior 也已接入，`cam_imu_2` 默认估计为 `-0.552334 s`，距离 Kalibr 最终值约 `5.83 ms`。bias motion prior 已从相邻差分替换为 B-spline 一阶导数积分的等价 residual。camera、gyroscope 和 accelerometer residual 已全部切到手写 Jacobian，并用 finite difference 覆盖各自参数块。pose motion prior 和 time-shift prior 已实现为可选 residual；分阶段 optimizer 已把 200 帧自由变量小样本从直接全自由的 `1.85 mm` / `1.94 deg` 压到 `0.090 mm` / 当前打印精度 `0 deg`。

Docker 对照已经从手工命令收敛为 `ceres_cam_imu/tools/run_kalibr_docker.py`。脚本只读挂载原始数据，复制输入到 `/out/input` 后运行官方 Kalibr，避免 raw 命令因为结果写回 `/data:ro` 而退出 `1`；`smoke_iter0_staged_verify` 复验零退出，并保存 `command.txt`、`kalibr_raw.log`、`kalibr_clean.log` 和 `summary.txt`。当前 summary 仍为 `22929` IMU readings、`22929 + 22929` accel/gyro terms、`9758` design variables、`693126` error terms、Jacobian `1432108x43893`、time shift `-0.5505243893928822 s`、raw mean `0.392372 px` / `0.175486 rad/s` / `1.06985 m/s²`。

最新定位是：20/10 kps 全数据结果的 camera residual 偏高主要来自与 Kalibr 默认 `100/50` knot 频率不匹配；Ceres Cauchy loss 参数也已从 `10` 修正为 `sqrt(10)` 以匹配 Kalibr `CauchyMEstimator(10)` 的 `sigma²` 语义。修正后，100/50 kps、固定 Kalibr time shift/gravity、全数据 staged 10-iter 的外参差为 `0.107 mm` / 当前打印精度 `0 deg`，reprojection/gyro mean 为 `0.206 px` / `0.168 rad/s`，已接近 Kalibr report。

对 100/50 下的 pose motion prior 方差做粗扫后，当前最佳点是 translation/rotation variance `10 / 1`：外参差 `0.106 mm`，reprojection/gyro/accel mean 为 `0.210 px` / `0.165 rad/s` / `0.934 m/s²`，已经接近 Kalibr report 的 `0.204 px` / `0.170 rad/s` / `0.817 m/s²`。继续收紧到 `1 / 0.1` 会让 camera 和 accel 都变差，暂不作为默认候选。

同一 `10 / 1` 设置下，完全放开 time shift 会让它偏回互相关初值附近，差 Kalibr `-6.36 ms`；加入 `--time-shift-prior-sigma 0.0001` 后，time shift 差收敛到 `-0.711 ms`，外参差约 `0.100 mm`，reprojection/gyro/accel mean 为 `0.211 px` / `0.165 rad/s` / `0.963 m/s²`。

新增 `--top-residuals` 后，当前最坏 5 个 accel residual 全部集中在 `411.577 s` 到 `411.585 s`。rank 1 的 accel residual 为 `137.943 m/s²`，pose world acceleration 为 `137.962 m/s²`，gravity-corrected body acceleration 为 `131.669 m/s²`，angular-accel lever 和 centripetal lever 均为 `0`。剩余尖峰来自 pose spline 平动二阶导数，不是 IMU lever arm 或角速度分支。

`--inspect-time 411.581 --inspect-window 0.02` 已把这个尖峰定位到局部 pose spline 段：`coeff_start=4855`、`dt=0.0100009 s`、`u=0.848321`，二阶 basis 权重为 `5.82, 2510.58, 1414.33, -9366.35, 4418.31, 1017.31`。translation controls 只在厘米级变化，但 100 Hz knot interval 会把二阶导放大；最近 camera query pose 在 `411.559 s` 和 `411.592 s`，IMU 窗口 `411.562-411.599 s` 的测量 accel/gyro 都平稳。因此下一步是验证局部或尾段 pose smoothing，而不是继续怀疑 IMU 数据或 lever-arm 项。

进一步对齐 Docker Kalibr smoke 后，发现 Kalibr 实际只加入 `22929` 个 IMU readings，而 Ceres 之前使用完整 `24928` 个 IMU samples。源码复核确认 Kalibr 在 `--imu_data_file` 下默认 `trimImuEdgeCount=1000`，skip 条件是 `index < trim` 或 `index > numMessages - trim`，因此会保留右边界样本。Ceres reader 已改成同一 inclusive upper-bound 语义，`--imu-trim-edge-count 1000` 可把 Ceres problem 调成 `22929 + 22929` 个 IMU residual。继续复核 `initPoseSplineFromCamera` 后确认 Kalibr `--timeoffset-padding S` 会在 pose spline 首尾各扩 `2*S`；Ceres 已同步该规则，dry-run 为 `4874` 个 order-6 pose controls、`2435 + 2435` 个 bias priors 和 `4636` 个 pose priors。当前 `0,1,4,5` 全数据 residual mean 为 `0.197 px` / `0.167 rad/s` / `0.863 m/s²`，accel max `43.2 m/s²`，外参差 `0.052 mm`，time shift 差 `+0.846 ms`。

`--inspect-times` 已支持一次 run 检查多个 outlier。按 Kalibr-style padding 复跑后，top accel outlier 仍在 `367-374 s` 动态段；`368.848 s` 和 `376.071 s` 的局部诊断仍指向 pose spline 平动二阶导，pose acceleration norm 分别约 `36.1 m/s²` 和 `12.6 m/s²`，lever-arm 分支仍为 `0`。剩余问题已经从“确认数据窗口和定位 outlier”转为“调整 pose/time-shift refinement 调度或 accelerometer 鲁棒化”。

accelerometer 鲁棒化已做首轮负例验证：新增独立的 `--camera-loss/--gyro-loss/--accel-loss` 后，直接把 accel 切到 Huber 可把 accel max 压到 `6-7 m/s²`，但 camera mean、time shift 和外参都会明显变差；固定 time shift 后仍不够好。默认 Cauchy 下调整 staged 每阶段迭代数更有效。第一轮全量 stage schedule sweep 后，`--stage-iterations 0,1,4,5` 目前最均衡；按 Kalibr-style padding 复跑后为 `0.197 px` / `0.167 rad/s` / `0.863 m/s²`，accel max `43.2 m/s²`，外参差 `0.052 mm`。新增 `--stage-free` 后，final stage 固定 time 或只放开外参基本不改变指标；固定全局 time shift 能小幅降低 camera mean 和 accel max，但 accel mean 略升。per-stage pose prior 方差也已接入，第三阶段收紧到 `5 / 0.5` 可把 accel mean/max 小幅降到 `0.858 m/s²` / `42.4 m/s²`，但 camera/gyro 稍差；放松到 `20 / 2` 则相反。最新新增 `--pose-motion-all-segments`、`--stage-time-shift-prior-sigmas`、`--stage-pose-motion-orders` 和 Kalibr-style pose-fit 初始化正则：全段 pose prior scope 基本中性；第三阶段放松 time prior 能改善 camera/extrinsic 但带来 time drift；第三阶段改一阶 pose prior 是负例，camera mean 降到 `0.180 px`，但 accel mean/max 爆到 `242 m/s²` / `14070 m/s²`；`--pose-fit-motion-lambda 0.0001 --pose-fit-boundary-anchors` 不优于默认初值，最终为 `0.201 px` / `0.169 rad/s` / `0.846 m/s²`，accel max `43.7 m/s²`。

模块边界继续向原始目标收敛：新增 `processing/dataset_processing.*` 后，文件解析和数据处理已经分开。`readImuCsv` 保持兼容入口，但 Kalibr edge trim 的实现和 summary 在 processing 层；`check_dataset` 复验 `24928 -> 22929`、保留索引 `1000..23928`，全数据 dry-run 的 problem-size 基准不变。

复现入口也继续收敛：`--kalibr-corner-defaults` 已把 Kalibr `--corner_file` 官方入口的 `100/50` knot、`max-iter 30`、`timeoffset-padding 0.04`、`trim-imu-edge-count 1000` 和 Cauchy width `10` 固化成 Ceres preset。真实 dry-run 仍为 `647266 / 22929 / 22929` residual 和 `9758 / 43893 / 693126` problem-size 基准。solver tolerance/trust-region 参数也已暴露，后续 sweep 可明确区分停止条件和 residual/调度差异。

## 当前待办

| 状态 | 类型 | 优先级 | 内容 | 来源 | 下一步 |
|---|---|---|---|---|---|
| done | action | P0 | 在 Kalibr Python/Docker 环境中运行 `ceres_cam_imu/tools/export_kalibr_corners.py`，导出 `cam0_640x400_corners.csv` 和 per-frame pose csv | `cam_imu_2` 角点 pickle 依赖 `aslam_cv` | 已导出 1455 帧 observations |
| done | action | P0 | 用真实 corners CSV 跑 `calibrate_cam_imu --dry-run` 和小样本 solve | 当前只用临时角点 CSV 验证 problem builder | 已记录到 `docs/experiment/20260615_Ceres_Cam-IMU真实数据小样本验证.md` |
| done | action | P1 | 验证 per-frame pose csv 初始化在真实数据上的收敛效果 | 已接入 `--corner-poses` 读取 | 已改为按 B-spline basis 的稀疏最小二乘拟合；真实数据使用 1455 帧 pose，RMS 为 `0.000652881 m` / `0.000936701 rad` |
| done | action | P1 | 实现 camera time shift prior 的互相关初始化 | Kalibr 会先估计 `timeshiftCamToImuPrior` | 已新增 `initialization/time_shift_initializer` 和 `--estimate-time-shift-prior` |
| done | action | P2 | 将 camera residual 改成 analytic `SizedCostFunction` | 目标要求核心 residual 尽量手写 Jacobian | 已替换为手写 Jacobian；`ctest` 做 finite-difference 校验 |
| done | action | P2 | 将 gyro residual 的 pose spline Jacobian 手写化 | 目标要求尽量手写 Jacobian | 已替换为 analytic `SizedCostFunction`；`ctest` 覆盖 IMU 外参、pose controls 和 gyro bias controls |
| done | action | P2 | 将 accelerometer residual 的 pose spline Jacobian 手写化 | 目标要求尽量手写 Jacobian | 已替换为 analytic `SizedCostFunction`；`ctest` 覆盖 IMU 外参、gravity、pose controls 和 accel bias controls |
| done | action | P2 | 对齐 Kalibr gravity design variable 默认语义 | Kalibr 默认用 `EuclideanDirection` 固定 gravity 模长、只优化方向 | Ceres 默认对 gravity block 使用 `SphereManifold<3>`；`gravity_tangent=2`，`--estimate-gravity-length` 切到 3D，`--fix-gravity` 为 0；`ctest` 覆盖 |
| done | action | P3 | 扩大 staged/time-shift prior 实验规模 | 核心 residual 已手写，但还不是全量对比 | 已完成 500 帧、1000 个 IMU residual 验证；后续转全量或接近全量 |
| done | action | P2 | 添加 pose motion prior 原型 | 200 帧放开 pose/bias/time 后外参偏离 Kalibr | 已实现 6 维 spline derivative integral residual 和 `--pose-motion-prior`；受控固定外参验证稳定 |
| done | action | P2 | 制定分阶段 pose/extrinsic 解冻策略 | 默认 pose prior 不能单独解决全自由小样本退化 | 已新增 `--staged` conservative preset，200 帧中等规模结果回到 Kalibr 附近 |
| done | action | P2 | 扩大 staged 中等规模验证到 500 个 IMU residual | 200 帧 staged 已稳定，但还不是全量 | 外参稳定；不固定 time shift 会漂移，固定 time shift 后外参为 `0.071 mm` / `0 deg` |
| done | action | P2 | 添加 time shift prior | 500 个 IMU residual 下自由 time shift 会漂到 `+39.6 ms` | 已实现 `--time-shift-prior-sigma`；`sigma=0.0001 s` 时外参与 fixed-time 情况接近 |
| open | action | P2 | 调优 time shift prior 或专门解冻阶段 | 软 prior 可防漂移，但仍把结果锚在互相关初值附近 | 设计更接近 Kalibr 的 time shift refinement，而不是单纯强 prior |
| done | action | P2 | 添加 final residual statistics 输出 | 全量对比不能只看外参/time shift，需要对齐 Kalibr report 的 residual 统计 | 已新增 `optimizer/residual_statistics`，CLI 输出 camera/gyro/accel raw 和 normalized 分布 |
| done | action | P2 | 把 bias motion prior 从相邻差分替换为 Kalibr 同型 spline derivative integral | 原相邻差分 prior 是可用近似，不是 Kalibr 完全一致 | 已用每 segment `sqrt(Q_s)` residual 等价实现；单测对齐数值积分 |
| done | action | P2 | 对齐 Kalibr 默认 knot 频率和 Cauchy loss 语义 | 20/10 kps 全数据 camera residual 高于 Kalibr report | Docker Kalibr smoke run 确认默认 `100/50`；Ceres Cauchy loss 改为 `sqrt(10)` 后，100/50 全数据 reprojection/gyro mean 为 `0.206 px` / `0.168 rad/s` |
| done | action | P3 | 粗扫 100/50 kps 下的 pose motion prior 方差 | camera/gyro 已贴近 Kalibr，但 accel mean 被少量高频尖峰主导 | `10 / 1` 是当前最佳点，accel mean 降到 `0.934 m/s²`，camera 仍为 `0.210 px` |
| done | action | P3 | 验证 `10 / 1` pose prior 在自由 time shift 下是否稳定 | 当前最佳结果固定了 Kalibr time shift，尚未证明 refinement 等价 | 自由 time shift 外参稳定但偏 `-6.36 ms`；加 `0.1 ms` prior 后偏 `-0.711 ms` |
| done | action | P3 | 定位剩余 accelerometer 离群点 | `10 / 1` 已把 mean 拉近 Kalibr，但 max 仍有 `98.85 m/s²` | 已新增 `--top-residuals`；最坏 5 个点集中在 `411.58 s` 附近，来源为平动二阶导尖峰 |
| done | action | P3 | 局部诊断 `411.58 s` pose spline 尖峰 | outlier 已定位，但还不知道是相邻 camera pose、控制点、边界效应还是局部观测异常 | 已输出 active pose control ids、basis 权重、相邻 camera pose 间隔和 IMU 样本窗口 |
| done | action | P3 | 验证 `411.58 s` 局部 pose smoothing 策略 | 局部诊断证明尖峰来自 pose 平动二阶导，不是 IMU 测量异常 | 局部 translation variance `0.01x` 会把尖峰转移到后续尾段，accel max 升到 `431 m/s²`，不作为默认 |
| done | action | P2 | 精确复核 Kalibr IMU sample 选取规则 | Docker Kalibr count 已对上，但需要源码级确认 | `IccSensors.py` 已确认 `trimImuEdgeCount=1000` 和 inclusive upper-bound 规则；Ceres reader 已更新并测试 |
| open | action | P3 | 调整裁剪后剩余动态段 accel outlier | 多点诊断确认 `367-374 s` outlier 仍由 pose spline 平动二阶导主导；直接 accel Huber、局部 smoothing、final stage time 解冻和一阶 pose prior 都是负例或弱作用 | 继续以二阶 pose motion prior 为基线，扫 pose/time/bias 联合阶段的权重、time prior 和迭代预算 |
| open | action | P3 | 设计更像 Kalibr 的 time-shift refinement 阶段 | `0.1 ms` soft prior 可控；自定义 stage 证明 final stage time 解冻不是主要差异 | 继续研究 pose/time/bias 联合阶段和 prior/边界设置，而不是只改 final stage |
| done | action | P3 | 复核 pose spline padding/knot 边界 | Docker zero-iter 在 `--timeoffset-padding 0.04` 下为 `4869` pose spline segments；Ceres 已从 `4866` pose coeffs 修正到 `4874` order-6 pose controls | 已按 Kalibr 首尾各 `2*S` padding 规则更新 `makeSplineForTimes` 并通过 dry-run/ctest |
| done | action | P3 | 兼容 Kalibr 原名 `--timeoffset-padding` | 文档和 Kalibr CLI 使用原名，Ceres 之前只解析 `--time-padding` | CLI 已支持两个名字；冲突值会返回错误；`--timeoffset-padding 0.01` dry-run 验证 pose coeffs 变为 `4802` |
| done | action | P3 | 添加每阶段 pose prior 方差控制 | 自定义 stage 证明 final stage 不是主要矛盾，需要直接扫 pose/time/bias 联合阶段权重 | 已新增 `--stage-pose-translation-variances` / `--stage-pose-rotation-variances`；`pbt=5/0.5` accel 稍好但 camera/gyro 稍差，`pbt=20/2` 相反 |
| done | action | P4 | 添加结构化结果导出 | 计划中 P4 需要参数更新和结果导出，原 CLI 只打印最终状态 | 已新增 `io/calibration_result_writer.*` 和 `--output-result`；20 帧 smoke 写出并读回 `/tmp/ceres_cam_imu_result_smoke.yaml` |
| done | action | P5 | 结构化读回 Ceres 结果并输出 Kalibr 差异报告 | `compare_kalibr_result` 之前只能打印 Kalibr 文本，不能把 Ceres 输出文件作为对比输入 | 新增 `io/calibration_result_reader.*`；`compare_kalibr_result --ceres-result` 复算 rotation/translation/time/gravity 和 residual mean delta |
| done | action | P3 | 增加 Kalibr 同口径 problem-size summary | 需要直接检查 Ceres 装配是否与 Docker Kalibr 的 design variable、Jacobian column 和 error term 数一致 | CLI 已打印 `parameter_blocks`、`tangent_params`、`kalibr_style_error_terms`；真实 dry-run 对齐 `9758 / 43893 / 693126` |
| done | action | P3 | 固化 Ceres 侧 Kalibr corner-file 默认入口 | 手写 `100/50`、trim、padding、max-iter 等参数容易混入口径 | 新增 `--kalibr-corner-defaults`；真实 dry-run 输出 `pose_kps=100`、`bias_kps=50`、`max_iterations=30`、`imu_trim_edge_count=1000`，problem-size 仍对齐 Docker Kalibr |
| done | action | P5 | 固化 Ceres sweep runner | 全数据对比需要批量记录命令、结构化结果、Kalibr compare 和 CSV 指标，不能继续手工抄终端日志 | 新增 `tools/run_ceres_sweep.py`；`smoke_verify` 和 `current_full_verify_stage_fields` 均 return_code `0`，summary 可复现当前全量基线 |
| done | action | P3 | 支持 Kalibr 同型全段 pose motion prior scope | Kalibr `BSplineMotionError` 作用于整个 pose spline，Ceres 原先只加 active segment prior | 新增 `--pose-motion-all-segments`；dry-run `pose_priors=4869`，全量结果与 active-only 基本相同 |
| done | action | P3 | 支持每阶段 time-shift prior 强度 | 全程同一个 `0.1 ms` prior 只能防漂移，不能细分早期防漂移和后期 refinement | 新增 `--stage-time-shift-prior-sigmas`；`0.0001,0.0001,0.0002,0` 得到 `0.1957 / 0.1673 / 0.8652`，time shift 差 `+1.75 ms` |
| done | action | P3 | 支持每阶段 pose motion derivative order | 需要确认 joint pose/time/bias 阶段是否可用一阶平滑降低 camera residual 且保住 accel | 新增 `--stage-pose-motion-orders`；`2,2,1,2` 得到 `0.1801 / 0.1739 / 242.4`，accel max `14070 m/s²`，一阶是负例 |
| done | action | P3 | 对齐并验证 Kalibr-style pose spline 初始化正则 | Kalibr `initPoseSplineSparse` 会加入二阶导数积分正则并扩展边界 pose | 新增 `--pose-fit-motion-lambda`、`--pose-fit-diagonal-lambda` 和 `--pose-fit-boundary-anchors`；`1e-4 + anchors` 全数据结果不优于默认初值 |
| done | action | P1 | 拆出数据处理层 | 原始目标要求数据读取层和数据处理层分开；IMU trim 规则原本在 reader/CLI 中混杂 | 新增 `processing/dataset_processing.*`；`ctest` 和真实 `check_dataset` 覆盖 Kalibr edge trim、frame limit、corner count 和 time range |
| done | action | P5 | 固化 Docker Kalibr zero-iter 对照 wrapper | 手工 `docker run` 可对齐 problem size，但只读挂载会在结果写回阶段失败且日志解析不可复用 | 新增 `tools/run_kalibr_docker.py`；staging 输入后 return_code `0`，summary 自动抽取 `22929 / 9758 / 693126 / 1432108x43893` |
| open | action | P3 | 跑充分迭代的全量 Ceres/Kalibr 对比 | 三类 mean residual 已接近 Kalibr，但 time-shift refinement 和 accel max 尚未等价 | 在离群点检查和 time refinement 调度后继续做全量收敛对比 |
| open | action | P1 | 继续收敛数据处理层边界 | processing 当前只覆盖基础筛选和统计，观测时间窗口选择、sweep 数据集描述仍散在 CLI/optimizer | 后续把可复用的数据选择和运行配置描述继续从 CLI 抽出 |

## 风险与观察点

| 状态 | 风险 | 影响 | 触发条件 | 缓解措施 | 下一步 |
|---|---|---|---|---|---|
| open | time shift 在更大 IMU 子问题中漂移 | 外参可能稳定但时间偏移不可信 | 200 帧、500 个 IMU residual、staged 且 time shift 自由时偏到 `+39.6 ms`；全数据 smoke 依赖强 prior 才稳定 | `--time-shift-prior-sigma 0.0001` 可防漂移，但仍偏向互相关初值 | 调优 time-shift refinement |
| done | IMU sample 时间窗口未源码级等价 Kalibr | residual count 和尾段行为会影响 accel mean/max，导致和 Kalibr report 对不齐 | Docker Kalibr smoke 使用 `22929` IMU readings；Ceres 未裁剪时使用 `24928` | 已新增 `--imu-trim-edge-count`，并按 Kalibr `trimImuEdgeCount=1000` / inclusive upper-bound 语义修正 reader | 已验证 `22929 + 22929` residual count；后续只监控其他 sample 条件差异 |
| open | 100/50 pose spline 仍有动态段平动二阶导尖峰 | accel mean 已接近 Kalibr，但 max 仍明显高于常规 IMU residual | IMU count 和 spline padding 对齐后 top outliers 位于 `367-374 s`，局部诊断仍指向 pose acceleration | `0,1,4,5` 将 accel max 降到 `43.2 m/s²`；固定全局 time shift 只小幅改变取舍；直接 accel Huber 会损伤 camera/time shift；`pbt=5/0.5` 只小幅改善 accel；第三阶段一阶 prior 会让 accel 爆到 `14070 m/s²`；`1e-4` pose-fit 初始化正则不优于默认 | 继续比较二阶 pose/time/bias 联合阶段的迭代预算和更接近 Kalibr 的 pose smoothing |
| open | pose spline 自由度吸收 residual | 外参/time shift 可能偏离 Kalibr | 直接全自由 200 帧小样本仍可退化；全数据不同 knot 设置下分别出现 camera residual 偏高或 accel 尖峰 | `--staged` 已能缓解 200 帧 case；100/50 下外参和 camera/gyro residual 已稳定 | 调整 pose regularization、time-shift 解冻和 pose/extrinsic final stage |
| open | 初值不足导致真实数据优化发散 | 无法与 Kalibr 数值比较 | 放开 time shift、gravity 或大量 pose 控制点后外参漂移 | 已完成 pose CSV 最小二乘初始化、gyro norm 互相关 time shift prior、B-spline derivative integral bias prior 和 staged optimizer | 在全数据 10-iter 基础上调 staged，不直接跑无调度全自由优化 |
| open | 当前 rotation-vector 约定必须持续对齐 Kalibr | Jacobian 符号可能反 | finite difference 或 Kalibr 对比出现系统性反号 | 所有 SO(3) 改动都用 `RotationVector.cpp` 和书的符号表复核 | 给 SO3/Jacobian 增加数值测试 |

## 已处理

| 日期 | 原事项 | 结果 | 去向 |
|---|---|---|---|
| 20260615 | 新建模块化 Ceres 子工程 | 已创建 `ceres_cam_imu/`，本机 CMake/build/ctest 通过 | 继续推进真实数据闭环 |
| 20260615 | 读取 `cam_imu_2` 配置和 Kalibr 结果 | `check_dataset` 和 `compare_kalibr_result` 已验证 | 后续用于差异报告 |
| 20260615 | 接入 per-frame pose CSV 读取 | `--corner-poses` 已接入 `check_dataset` 和 `calibrate_cam_imu`，可初始化 pose 控制点 | 已在真实导出的 pose CSV 上验证 |
| 20260615 | 导出真实角点并跑小样本验证 | Docker 镜像成功导出真实 CSV；固定 pose/bias/time/gravity 后，Ceres 与 Kalibr 外参差异为 `0.00213955 deg`、`0.000136175 m` | 实验文档已记录 |
| 20260615 | pose 控制点初始化从最近邻改为 B-spline 最小二乘拟合 | 本机 build/ctest 通过；真实数据 20 帧和 200 帧固定辅助变量对比均稳定，200 帧外参与 Kalibr 平移差约 `8.56926e-05 m` | 继续推进全量/自由变量对比 |
| 20260615 | camera time shift prior 互相关初始化 | 新增 `initialization/time_shift_initializer` 和 CLI `--estimate-time-shift-prior`；真实数据估计 `-0.552334 s`，与 Kalibr 最终值差 `-0.0058298 s`；固定该估计值的 200 帧优化外参平移差约 `7.94528e-05 m` | 已继续完成 bias motion prior，后续转 analytic/free-variable |
| 20260615 | bias motion prior 替换为 B-spline derivative integral | 新 prior 每个 bias segment 连接 6 个控制点，residual 平方和等价于 `integral(dot(b)^T W dot(b) dt)`；`ctest` 通过，真实 20 帧 bias 自由 smoke solve 可运行 | 继续 analytic Jacobian 和中规模自由变量验证 |
| 20260615 | camera reprojection residual 手写 Jacobian | 替换 AutoDiff camera residual；finite-difference 覆盖 `T_c_b`、time shift 和 6 个 pose control blocks；真实 200 帧受限优化仍保持 `7.94528e-05 m` 平移差 | 继续 gyro/accel analytic Jacobian |
| 20260615 | 中规模自由变量阶梯实验 | 固定 pose/gravity、放开 bias/time 在 200 帧上稳定；固定 gravity 且放开 pose/bias/time 在 200 帧上外参偏到 `4 mm` / `0.146 deg` | 转向 pose/extrinsic 分阶段解冻策略 |
| 20260615 | pose motion prior 原型 | 新增通用 spline motion prior、6 维 pose prior、CLI `--pose-motion-prior` 和数值积分测试；固定外参受控验证稳定，但全自由小样本仍退化 | 转向分阶段解冻和 gyro/accel analytic Jacobian |
| 20260615 | 分阶段 pose/extrinsic 解冻策略 | 新增 `optimizer/staged_optimizer.*` 和 CLI `--staged`；200 帧、100 个 IMU residual、`--pose-motion-prior` 下，staged 外参差约 `0.090 mm` / `0 deg`，明显优于直接全自由 `1.85 mm` / `1.94 deg` | 扩大 staged 规模，继续 gyro/accel analytic Jacobian |
| 20260615 | staged 扩大到 500 个 IMU residual | 不固定 time shift 时外参仍接近但 time shift 漂移；显式 `--fix-time-shift` 后外参差约 `0.071 mm` / `0 deg` | 补 time-shift prior 或专门解冻阶段 |
| 20260615 | time-shift prior residual | 新增 `residuals/time_shift_prior.*`、`--time-shift-prior-sigma` 和单测；500 个 IMU residual 下 `sigma=0.0001 s` 时外参约 `0.071 mm` / `0 deg`，time shift 差 Kalibr `-5.07 ms` | 后续调优 refinement，不只锚定互相关初值 |
| 20260615 | gyroscope residual 手写 Jacobian | 新增 analytic `SizedCostFunction`，基于 SO(3) left Jacobian 覆盖 IMU 外参、pose controls、gyro bias controls；finite difference 和真实 staged smoke 通过 | 继续 accelerometer analytic Jacobian |
| 20260615 | accelerometer residual 手写 Jacobian | 新增 analytic `SizedCostFunction`，覆盖 lever arm、IMU 外参旋转、gravity、pose controls、accel bias controls；finite difference 和真实 staged smoke 通过 | 扩大 staged/time-shift prior 真实数据规模 |
| 20260615 | staged/time-shift prior 扩大到 500 帧和 1000 个 IMU residual | problem 包含 `234585` 个 camera residual、`1000 + 1000` 个 IMU residual、`408` 个 pose prior；最终外参差约 `0.124 mm` / `0 deg`，time shift 差 Kalibr `-3.71 ms` | 下一步推进全量或接近全量对比 |
| 20260615 | 全数据 staged smoke | problem 包含 `647266` 个 camera residual、`24928 + 24928` 个 IMU residual、`1000` 个 pose prior；每阶段 2 次迭代后外参差约 `0.267 mm` / `0.0097 deg`，time shift 差 Kalibr `+0.817 ms` | 下一步做充分迭代和 residual 统计 |
| 20260615 | final residual statistics | 新增分组统计输出；全数据 staged smoke 的 mean residual 为 `0.316 px` / `0.163 rad/s` / `1.052 m/s²` | 下一步用统计结果对齐 Kalibr report |
| 20260615 | 全数据 staged 10-iter | cost 降到 `4.8968e5`，外参差约 `0.221 mm` / `0.0073 deg`，time shift 差 `+2.65 ms`；mean residual 为 `0.318 px` / `0.163 rad/s` / `0.797 m/s²` | camera residual 偏高已由 100/50 knot 和 Cauchy loss 对齐定位；仍需处理 time-shift refinement |
| 20260615 | Docker Kalibr 官方入口 smoke run | 容器内 `--corner_file/--image_timestamp_file/--imu_data_file` 可跑通；早期 1-iteration smoke 确认 Kalibr 默认 `pose/bias kps=100/50`，后续以 20260616 zero-iteration 复核的 `9758` design variables / Jacobian `1432108 x 43893` 作为问题规模基准 | 已记录到实验文档 |
| 20260615 | Kalibr knot 频率和 Cauchy loss 语义对齐 | Ceres `CauchyLoss` 改为 `sqrt(10)`；100/50 全数据 staged 10-iter 外参差约 `0.107 mm`，reprojection/gyro mean 为 `0.206 px` / `0.168 rad/s` | 已继续完成 pose prior 方差粗扫 |
| 20260615 | 100/50 pose prior 方差粗扫 | `1e4/1e3`、`1e3/1e2`、`1e2/1e1`、`10/1`、`1/0.1` 均已跑；`10/1` 得到 `0.210 px` / `0.165 rad/s` / `0.934 m/s²` | 已继续完成自由 time shift 验证；后续定位 accel max |
| 20260615 | `10/1` 自由 time shift 验证 | 不加 prior 时 time shift 差 `-6.36 ms`，加 `0.1 ms` prior 后差 `-0.711 ms`；三类 mean 保持在 Kalibr 量级 | 下一步做 time-shift refinement 调度和 accel 离群定位 |
| 20260616 | top accel residual 诊断 | 新增 `--top-residuals`；最坏 5 个 accel residual 位于 `411.577 s` 到 `411.585 s`，rank 1 残差 `137.943 m/s²`，来源为平动二阶导数尖峰 | 下一步做 `411.58 s` 局部 spline/观测窗口诊断 |
| 20260616 | Docker 重新打开后的 Kalibr 复验 | `docker ps`、镜像查询、临时 pickle 导出和官方 zero-iteration run 均通过到 problem build；problem 为 `9758` design variables / `693126` error terms，Jacobian `1432108 x 43893`，residual mean 为 `0.392 px` / `0.175 rad/s` / `1.070 m/s²`；只读挂载只在结果写回阶段失败，退出后无残留容器 | Docker 可继续作为 Kalibr 基准环境 |
| 20260616 | `411.58 s` 局部 spline 诊断 | `--inspect-time 411.581` 输出 active controls `4855-4860`、二阶 basis 权重、最近 camera query pose 和 IMU 窗口；IMU 测量平稳，尖峰来自局部平动二阶导 | 下一步验证局部/尾段 smoothing 策略 |
| 20260616 | 局部 pose-motion scaling 诊断 | 新增局部方差缩放 CLI；`411.581 s` 附近 `0.01x` translation variance 可压低该点 acceleration，但会把尖峰转移到 `411.63 s` 和 `411.81 s`，max 升到 `431 m/s²` | 作为诊断工具保留，不作为默认策略 |
| 20260616 | IMU residual count 对齐 Kalibr | 新增 `--imu-trim-edge-count` 并按 Kalibr 源码修正为 `--imu-trim-edge-count 1000` 精确对齐；构建出 `22929 + 22929` IMU residual | 已继续完成 spline padding 复核和裁剪后动态段 outlier 多点诊断 |
| 20260616 | pose spline padding/knot 边界复核 | Kalibr `--timeoffset-padding S` 在 pose spline 首尾各扩 `2*S`；Ceres `makeSplineForTimes` 已同步该规则，dry-run 为 `4874` pose controls 和 `2435 + 2435` bias priors | 后续继续 time-shift refinement 和 pose prior 调度 |
| 20260616 | 裁剪后动态段 outlier 多点诊断 | 新增 `--inspect-times`；Kalibr-style padding 后 top accel max 为 `43.2 m/s²`，`368.848 s` 和 `376.071 s` 的 pose acceleration norm 分别约 `36.1` / `12.6 m/s²`；lever-arm 分支仍为 `0` | 下一步做 pose/time-shift 调度和 accel 鲁棒化实验 |
| 20260616 | accelerometer 鲁棒核负例 | 新增独立 robust loss CLI；accel Huber 5-iter 将 accel max 降到 `6-7 m/s²`，但 camera mean 升到 `0.292-0.336 px`、外参差 `0.327-0.354 mm`，time shift 可偏到 `+7.07 ms` | 不作为默认；下一步转 stage-specific 调度 |
| 20260616 | stage-specific 迭代预算与失败停止 | 新增 `--stage-iterations N0,N1,N2,N3` 和 `--stop-on-stage-failure`；full dry-run 计数保持 `647266 / 22929 / 22929`，小规模 solve smoke 通过 | 已继续完成首轮全量 sweep |
| 20260616 | 首轮全量 stage schedule sweep | `0,1,1,8` 和 `0,1,2,8` 会制造或转移 accel 尖峰；`0,1,3,5` camera 偏高；Kalibr-style padding 后 `0,1,4,5` 得到 `0.197 px` / `0.167 rad/s` / `0.863 m/s²`，accel max `43.2 m/s²`，外参差 `0.052 mm` | 下一步以 `0,1,4,5` 为基线做 time-shift/pose-prior sweep |
| 20260616 | 自定义 stage free-variable mask | 新增 `--stage-free`，可用 `p/b/e/t/g` 指定每阶段自由变量；`e,tb,pbt,eb`、`e,tb,pbt,e` 和固定全局 time shift 对照均已跑 | 结论是 final stage time 解冻影响很小；下一步扫 pose/time/bias 联合阶段 |
| 20260616 | 每阶段 pose prior 方差控制 | 新增 `--stage-pose-translation-variances` 和 `--stage-pose-rotation-variances`；`e,tb,pbt,ebt` 下第三阶段 `5/0.5` 得到 `0.197648` / `0.167630` / `0.858257`，accel max `42.3882`；`20/2` 得到 `0.196171` / `0.167064` / `0.869481`，accel max `43.9022` | 权重是有效旋钮，但不能单独消除动态段 outlier |
| 20260616 | 结构化结果导出 | 新增 `io/calibration_result_writer.*` 和 `--output-result result.yaml`；文件包含 `T_c_b/T_b_c`、time shift、gravity、spline metadata、residual statistics、top accel outlier 和 `kalibr_delta`；`ctest` 和 20 帧 smoke 均通过 | 后续全数据 sweep 可落结构化结果文件，便于和 Kalibr/历史 run 对比 |
| 20260616 | 结构化 Ceres/Kalibr 差异报告 | 新增 `io/calibration_result_reader.*` 并扩展 `compare_kalibr_result --ceres-result`；20 帧 smoke 文件复算 delta 为 `0.00276155 deg` / `0.000141644 m` / `0 s` / gravity `0`，residual mean delta 为 `-0.0760614 px` / `+0.0578465 rad/s` / `+2.34804 m/s²` | 后续全数据 sweep 可直接用结果文件比较，不再依赖终端日志 |
| 20260616 | Kalibr-style gravity direction manifold | Ceres 默认对 gravity block 挂 `SphereManifold<3>`，匹配 Kalibr `EuclideanDirection` 的固定模长、2 维方向更新；`--estimate-gravity-length` 可切回 3D 欧式 gravity | 后续 gravity 自由实验可区分方向优化和模长优化 |
| 20260616 | `--timeoffset-padding` CLI 兼容别名 | `--time-padding` 与 Kalibr 原名 `--timeoffset-padding` 指向同一选项；冲突值返回错误；非默认 `0.01` dry-run 已验证 coeff 数变化 | 后续命令可直接复用 Kalibr 原参数名 |
| 20260616 | Kalibr 同口径 problem-size summary | `CalibrationBuildSummary` 新增 residual block、scalar residual、parameter block、active tangent 和 Kalibr-style error term 统计；固定参数块不再计入 tangent 维度 | 真实 dry-run 与 Docker Kalibr 对齐为 `9758` parameter blocks、`43893` tangent params、`693126` Kalibr-style error terms |
| 20260616 | 全段 pose motion prior 与每阶段 time prior sigma | 新增 `--pose-motion-all-segments` 和 `--stage-time-shift-prior-sigmas`；全段 prior 把 `pose_priors` 从 `4636` 提到 `4869` 但结果中性；第三阶段 time sigma 从 `0.1 ms` 放宽到 `0.2 ms` 可降 camera mean 到 `0.1957 px`，但 time shift 差增到 `+1.75 ms` | 保留为后续 stage sweep 工具，不替代当前默认基线 |
| 20260616 | 每阶段 pose motion order 与一阶负例 | 新增 `--stage-pose-motion-orders`，dry-run 会打印 `pose_order`；第三阶段改一阶 `2,2,1,2` 后 camera mean 为 `0.1801 px`，但 accel mean/max 为 `242 m/s²` / `14070 m/s²`，外参差 `0.147 mm` | 说明 velocity-level smoothing 不是当前动态段解法，继续用二阶 prior 做基线 |
| 20260616 | Kalibr-style pose spline 初始化正则 | 新增 `--pose-fit-motion-lambda`、`--pose-fit-diagonal-lambda` 和 `--pose-fit-boundary-anchors`；`0.0001 + anchors` 初始化 RMS 为 `0.001084 m` / `0.002040 rad`，全量结果为 `0.2010 px` / `0.1691 rad/s` / `0.8461 m/s²`，accel max `43.7 m/s²` | 作为诊断开关保留，默认仍使用无 motion 正则的 pose fit |
| 20260616 | Docker Kalibr zero-iter wrapper | 新增 `tools/run_kalibr_docker.py`，默认 staging `cam_imu_2` 输入到 `/out/input` 后运行官方 Kalibr；`smoke_iter0_staged_verify` return_code `0`，summary 为 `22929` IMU readings、`9758` design variables、`693126` error terms、Jacobian `1432108x43893` | 后续 Kalibr problem-size 复验优先用 wrapper，不再依赖会写回只读 `/data` 的 raw 命令 |
| 20260616 | 数据处理层第一版 | 新增 `processing/dataset_processing.*`，把 Kalibr IMU edge trim、frame limit、corner count、time range 从 reader/CLI 中抽出；`check_dataset` 输出 `24928 -> 22929` 和保留索引 `1000..23928`，dry-run problem size 不变 | 后续继续抽象观测筛选和 sweep 运行配置 |
| 20260616 | Ceres Kalibr corner-file preset | 新增 `--kalibr-corner-defaults`，默认 `100/50` knot、`max-iter 30`、`timeoffset-padding 0.04`、`trim-imu-edge-count 1000`、Cauchy width `10`；dry-run 为 `9758 / 43893 / 693126` | 后续 Kalibr 对齐 dry-run 优先使用 preset，显式参数再覆盖 |
| 20260616 | Ceres sweep runner | 新增 `tools/run_ceres_sweep.py`，支持内置 preset、自定义 variant、自动 `--output-result`、`compare_kalibr_result` 和 `summary.csv`；`smoke_verify` 输出 `0.00276155 deg` / `0.000141644 m` / `0 s` 几何差异，`current_full_verify_stage_fields` 输出 `0.196977` / `0.167393` / `0.863287` mean residual 和 `5.17357e-05 m` 外参平移差 | 后续全量对齐 sweep 优先用该 runner 记录，不再手工整理日志 |
