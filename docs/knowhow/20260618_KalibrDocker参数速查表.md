# Kalibr Docker 参数速查表

## 场景

这个仓库对外主入口是 Docker wrapper，用户只需要记住镜像 `kalibr-camera-calibration:20.04` 和两个子命令：`cam-cam`、`cam-imu`。高级用户仍可在镜像里直接调用原始 `rosrun kalibr ...`，用于低层调试或复刻官方命令。

## Docker 入口

| 命令 | 用途 | 影响 |
|---|---|---|
| `docker pull wang121ye/kalibr-camera-calibration:20.04` | 拉取公开镜像 | 推荐给外部用户，一键部署，不在本机编译 ROS/Kalibr |
| `docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .` | 构建本地镜像 | 适合开发、改 Dockerfile、私有依赖或无法访问镜像仓库时使用 |
| `docker push wang121ye/kalibr-camera-calibration:20.04` | 发布 DockerHub 镜像 | 公开交付最直接；GHCR 可用 `ghcr.io/<user>/kalibr-camera-calibration:20.04` 同步发布 |
| `docker run --rm kalibr-camera-calibration:20.04 --help` | 查看总入口 | 验证镜像可用 |
| `docker run --rm kalibr-camera-calibration:20.04 cam-cam --help` | 查看 cam-cam 参数 | 只覆盖相机/多相机标定 |
| `docker run --rm kalibr-camera-calibration:20.04 cam-imu --help` | 查看 cam-imu 参数 | 只覆盖 IMU-camera 标定 |

挂载约定：输入目录建议只读挂载为 `:ro`，输出目录挂载为可写。Apple Silicon 或非 amd64 机器可加 `--platform linux/amd64`。

## 支持的输入格式

| 入口 | 支持输入 | 不支持/限制 |
|---|---|---|
| `cam-cam` wrapper | 单相机图片目录、单张图片、单视频、`cam0/cam1/...` 多相机图片目录 | 多相机视频不同步；多相机图片要求每路数量一致并按自然排序配对 |
| `cam-imu` wrapper | 多相机 camchain、一个或多个 IMU YAML；`--bag`、corner-file 三件套或 H5 三件套 | 不直接接收图片目录/视频；多 IMU 的 H5/corner-file 模式要求每个 IMU 一份 CSV |
| 原始 `kalibr_calibrate_cameras` | ROS bag + 多个 image topics + 对应 camera models | 需要用户自己保证 topic、模型、target 匹配 |
| 原始 `kalibr_calibrate_imu_camera` | ROS bag、camchain、一个或多个 IMU YAML、target YAML | 多 IMU 要让 `--imu` 和 `--imu-models` 数量一致，第一个 IMU 是参考 IMU |
| Ceres 子模块 | `pkl`、`bag`、`euroc/mav0` 转换后运行单阶段 Ceres | 当前 Ceres 二进制是单 IMU 标定入口 |

## `cam-cam` 参数

| 参数 | 默认 | 背景 | 影响与限制 |
|---|---:|---|---|
| `--input` | 必填 | 图片目录、单个视频，或含 `cam0/cam1/...` 的目录 | 多相机目录按自然排序配对；多相机视频不支持 |
| `--target` | 必填 | AprilGrid YAML 文件或目录 | YAML 必须匹配真实标定板 |
| `--output` | 必填 | 输出目录 | 写入 work 目录、Kalibr 结果、报告 |
| `--models` | `pinhole-radtan` | Kalibr 相机模型列表 | 可传单个模型或逗号分隔模型；数量需匹配相机数 |
| `--resize` | 空 | 强制归一化尺寸，如 `1280x720` | 改变输入图像尺寸，需理解像素尺度影响 |
| `--preprocess` | `none` | `none/hist-eq/clahe` | CLAHE 可改善低对比度，但会改变图像纹理 |
| `--timestamp-fps` | `10.0` | 合成图片时间戳频率 | 影响生成 bag 的图像时间轴 |
| `--video-sample-fps` | `4.0` | 视频抽帧频率 | 太低会覆盖不足，太高会变慢 |
| `--max-video-frames` | `0` | 视频最多抽帧数 | `0` 表示不显式限制 |
| `--diagnostic-max-images` | `200` | 诊断最多检查图片数 | 只影响诊断速度和报告，不限制 Kalibr 输入 |
| `--focal-length-init` | 空 | 设置 Kalibr 焦距初值环境变量 | 焦距初始化失败时使用 |
| `--fast-extraction` | `auto` | `auto/always/never` | `auto` 先多进程，检测到 rosbag 读取异常后回退单线程 |
| `--lang` | `zh` | `zh/en` | 报告和 warning 语言 |
| `--verbose` | 关闭 | 保存更多中间文件并流式日志 | 便于排查，输出更多 |
| `--show-report` | 关闭 | 允许 Kalibr 打开图形报告 | Docker/无 GUI 环境不建议 |
| `--skip-kalibr` | 关闭 | 只做数据整理与诊断 | 不产生最终 Kalibr 标定结果 |

## `cam-imu` 参数

| 参数 | 默认 | 背景 | 影响与限制 |
|---|---:|---|---|
| `--target` | 必填 | AprilGrid YAML | 与 cam-cam 相同 |
| `--cam-chain` | 必填 | cam-cam 输出的 camchain YAML | 决定相机模型、内参、初始外参 |
| `--imu-yaml` | 必填 | 一个或多个 Kalibr IMU YAML；支持 flat 输入 YAML、nested 结果 YAML、一个聚合 YAML | 按展开后的顺序决定 reference IMU；没有人为 IMU 数量上限 |
| `--output` | 必填 | 输出目录 | 写入日志、结果、报告 |
| `--lang` | `zh` | `zh/en` | 报告语言 |
| `--bag` | 空 | ROS bag，topic 来自 camchain/IMU YAML | 支持多相机、多 IMU；适合 TUM/EuRoC bag |
| `--imu-models` | 每个 IMU 为 `calibrated` | 每个 IMU 的模型 | 可选 `calibrated`、`scale-misalignment`、`scale-misalignment-size-effect`；传 1 个会复制到全部展开后的 IMU，传多个时数量必须等于展开后的 IMU 数 |
| `--imu-delay-by-correlation` | 关闭 | 多 IMU 间延迟相关估计 | 多 IMU 时可打开；会增加时间延迟估计自由度 |
| `--corner-file` | 空 | 预提取角点 pkl | corner-file 模式三件套之一 |
| `--image-timestamp-file` | 空 | corner-file 对应图像时间戳 | corner-file 模式三件套之一 |
| `--imu-data-file` | 空 | corner-file 模式 IMU CSV/TXT，可传多个 | corner-file 模式三件套之一；多 IMU 时数量必须等于展开后的 IMU 数 |
| `--h5-file` | 空 | H5 图像观测文件 | H5 模式三件套之一 |
| `--h5-timestamp-file` | 空 | H5 对应图像时间戳 | H5 模式三件套之一 |
| `--imu-csv` | 空 | H5 模式 IMU CSV，可传多个 | H5 模式三件套之一；多 IMU 时数量必须等于展开后的 IMU 数 |
| `--fixture-id` | `fixture` | corner-file 输出命名后缀 | 影响 Kalibr 结果文件名 |
| `--trim-imu-edge-count` | corner-file 模式 `1000`，bag/H5 模式 `0` | 裁掉首尾 IMU 样本数 | 不传时由原生 Kalibr 按数据源决定；显式传入会覆盖默认值 |
| `--timeoffset-padding` | `0.03` | spline 时间边界 padding | BenchmarkCalibration 常用 `0.04` |
| `--max-iter` | `30` | Kalibr 最大迭代数 | 太小可能未收敛，太大耗时 |
| `--pose-knots-per-second` | `100` | pose spline knot rate | 越大轨迹更灵活、变量更多 |
| `--bias-knots-per-second` | `50` | bias spline knot rate | 越大 bias 更灵活、变量更多 |
| `--no-time-calibration` | 默认开启 | 禁用 time offset 标定 | 默认不估 time offset |
| `--estimate-time-offset` | 关闭 | 开启 time offset 标定 | 会释放时间偏移自由度 |
| `--export-poses` | 默认开启 | 导出优化后 pose | 输出更多结果文件 |
| `--no-export-poses` | 关闭 | 不导出 pose | 对齐旧 BenchmarkCalibration 时常用 |
| `--focal-length-init` | 空 | 设置 Kalibr 焦距初值 | 一般 cam-imu 不需要 |
| `--verbose` | 关闭 | 流式打印 Kalibr 输出 | 调试失败时打开 |

`cam-imu` 必须三选一输入模式：`--bag`、`--corner-file/--image-timestamp-file/--imu-data-file`、或 `--h5-file/--h5-timestamp-file/--imu-csv`，不能混用。wrapper 会先把 `--imu-yaml` 展开成 flat Kalibr 输入 YAML：可以传多个 flat YAML，也可以传多个 Kalibr 输出型 nested YAML，还可以传一个包含 `imu0/imu1/...` 或 `imus:` 的聚合 YAML。H5/corner-file 模式必须按展开后的 IMU 顺序传同等数量的 IMU CSV。

## wrapper 与原生命令差异

上面的 `cam-cam`、`cam-imu` 表覆盖的是 Docker wrapper 对外暴露的参数。Kalibr 原生命令还有一些调试/低层参数没有透出到 wrapper，典型缺口如下：

| 原生参数 | 所属命令 | wrapper 状态 | 何时直接用原生命令 |
|---|---|---|---|
| `--recompute-camera-chain-extrinsics` | `kalibr_calibrate_imu_camera` | 未暴露 | 怀疑 camchain 里的相机链外参有问题，需要让 cam-IMU 阶段重新估相机链外参 |
| `--reprojection-sigma` | `kalibr_calibrate_imu_camera` | 未暴露 | 调整角点重投影噪声权重 |
| `--recover-covariance` | `kalibr_calibrate_imu_camera` | 未暴露 | 需要输出设计变量协方差；会增加耗时/内存 |
| `--bag-from-to`、`--bag-freq` | `kalibr_calibrate_cameras`、`kalibr_calibrate_imu_camera` | 未暴露 | 只截取 bag/H5/corner 的一段数据，或降频提取图像特征 |
| `--perform-synchronization` | `kalibr_calibrate_imu_camera` | 未暴露 | 需要使用 Kalibr 内置 clock synchronization |
| `--show-extraction`、`--extraction-stepping`、`--plot`、`--plot-outliers` | 原生可视化/调试参数 | wrapper 只暴露 `cam-cam --show-report` 和 `--verbose` | 有 GUI 环境并要交互查看提取过程 |
| `--qr-tol`、`--mi-tol`、`--no-shuffle`、`--no-outliers-removal`、`--no-final-filtering`、`--min-views-outlier`、`--use-blakezisserman` | `kalibr_calibrate_cameras` | 未暴露 | 精细控制相机标定的视图筛选、离群点过滤和鲁棒核 |

注意原生命令和 wrapper 的参数拼写不完全一致：wrapper 用 `--cam-chain`、`--h5-file`、`--h5-timestamp-file`、`--imu-csv`、`--corner-file`、`--image-timestamp-file`、`--imu-data-file`、`--fixture-id`；原生 `kalibr_calibrate_imu_camera` 用 `--cams`、`--h5file`、`--h5timestampfile`、`--imufile`、`--corner_file`、`--image_timestamp_file`、`--imu_data_file`、`--fixture_id`。

支持的 IMU YAML 形态：

```yaml
rostopic: /imu0
update_rate: 500.0
accelerometer_noise_density: 0.01
accelerometer_random_walk: 0.0002
gyroscope_noise_density: 0.001
gyroscope_random_walk: 0.000004
```

```yaml
imu0:
  rostopic: /imu0
  update_rate: 500.0
  accelerometer_noise_density: 0.01
  accelerometer_random_walk: 0.0002
  gyroscope_noise_density: 0.001
  gyroscope_random_walk: 0.000004
imu1:
  rostopic: /imu1
  update_rate: 500.0
  accelerometer_noise_density: 0.01
  accelerometer_random_walk: 0.0002
  gyroscope_noise_density: 0.001
  gyroscope_random_walk: 0.000004
```

## 原始 Kalibr CLI 高级入口

| 场景 | 命令形态 | 影响 |
|---|---|---|
| bag 格式多相机 cam-cam | `rosrun kalibr kalibr_calibrate_cameras --bag data.bag --topics /cam0/image_raw /cam1/image_raw --models pinhole-equi pinhole-equi --target aprilgrid.yaml` | 绕过 wrapper，直接使用 bag 内时间戳和 topic |
| bag 格式单 IMU cam-imu | `rosrun kalibr kalibr_calibrate_imu_camera --bag data.bag --cams camchain.yaml --imu imu.yaml --imu-models calibrated --target aprilgrid.yaml` | 适合 TUM/EuRoC 转 bag 后运行；原生参数名是 `--cams` |
| bag 格式多 IMU cam-imu | `camera_calibration cam-imu --bag data.bag --cam-chain camchain.yaml --imu-yaml imu0.yaml imu1.yaml --imu-models calibrated --target aprilgrid.yaml --output out` | wrapper 一键入口；第一个展开后的 IMU 为参考 IMU，可加 `--imu-delay-by-correlation` |
| 聚合 YAML 多 IMU cam-imu | `camera_calibration cam-imu --bag data.bag --cam-chain camchain.yaml --imu-yaml imus.yaml --imu-models calibrated --target aprilgrid.yaml --output out` | `imus.yaml` 可包含 `imu0/imu1/...`，wrapper 会自动展开 |

### `kalibr_calibrate_cameras` 原生参数

| 参数 | 默认 | 背景 | 影响与限制 |
|---|---:|---|---|
| `--models` | 必填 | 每个 topic 一个相机模型 | 支持 `pinhole-radtan`、`pinhole-equi`、`pinhole-fov`、`omni-none`、`omni-radtan`、`eucm-none`、`ds-none`；数量必须等于 `--topics` |
| `--bag` | 空 | ROS bag | 原生命令直接从 bag 读图像 topic |
| `--topics` | 必填 | 一个或多个 image topic | 数量必须等于 `--models` |
| `--bag-from-to` | 空 | 起止时间 `[start end]`，单位秒 | 只使用 bag 的指定时间段 |
| `--bag-freq` | 空 | 图像特征提取频率，单位 Hz | 对 bag 图像降频，降低耗时 |
| `--target` | 必填 | AprilGrid/棋盘等 target YAML | 必须匹配真实标定板 |
| `--approx-sync` | `0.02` | 多相机近似同步容差，单位秒 | 多相机 topic 时间戳不同步时影响配对 |
| `--qr-tol` | `0.02` | QR 分解因子容差 | 影响新增视图的数值筛选 |
| `--mi-tol` | `0.2` | mutual information 入选阈值 | 越高加入的图像越少；`-1` 强制使用全部图像 |
| `--no-shuffle` | 关闭 | 不打乱数据处理顺序 | 便于复现顺序相关问题 |
| `--no-multithreading` | 关闭 | 禁用角点提取多进程 | 更慢但更稳；wrapper 的 `--fast-extraction never/auto fallback` 会用到 |
| `--no-outliers-removal` | 关闭 | 禁用角点离群点过滤 | 调试用，可能降低结果鲁棒性 |
| `--no-final-filtering` | 关闭 | 禁用最终过滤 | 调试用，保留更多观测 |
| `--min-views-outlier` | `20` | 初始化离群点统计所需 raw views 数 | 太小统计不稳，太大可能过滤启动较晚 |
| `--use-blakezisserman` | 关闭 | 启用 Blake-Zisserman m-estimator | 处理离群点较多的数据 |
| `--plot-outliers` | 关闭 | 绘制离群点 | 慢，适合 GUI 调试 |
| `--verbose` | 关闭 | 更详细日志 | 会禁用报告弹窗 |
| `--show-extraction` | 关闭 | 显示 target 提取过程 | 会禁用报告弹窗；Docker/无 GUI 不建议 |
| `--plot` | 关闭 | 标定过程中绘图 | 慢，适合 GUI 调试 |
| `--dont-show-report` | 关闭 | 不弹出报告窗口 | Docker 默认建议加上 |
| `--export-poses` | 关闭 | 导出优化后的 pose CSV | 额外输出轨迹结果 |

### `kalibr_calibrate_imu_camera` 原生参数

| 参数 | 默认 | 背景 | 影响与限制 |
|---|---:|---|---|
| `--bag` | 空 | ROS bag，topic 来自 camchain/IMU YAML | bag/H5/corner-file 三种数据源之一 |
| `--h5file` | 空 | H5 图像数据 | 必须和 `--h5timestampfile`、`--imufile` 一起用 |
| `--corner_file` | 空 | 预提取角点 pkl | 必须和 `--image_timestamp_file`、`--imu_data_file` 一起用 |
| `--h5timestampfile` | 空 | H5 图像时间戳文件 | H5 模式必填 |
| `--image_timestamp_file` | 空 | corner-file 图像时间戳文件 | corner-file 模式必填 |
| `--imufile` | 空 | H5 模式 IMU 数据文件，可传多个 | 多 IMU 时数量必须等于 `--imu` YAML 数 |
| `--imu_data_file` | 空 | corner-file 模式 IMU 数据文件，可传多个 | 多 IMU 时数量必须等于 `--imu` YAML 数 |
| `--bag-from-to` | 空 | 起止时间 `[start end]`，单位秒 | 对 bag/H5/corner 图像和 IMU 都做时间裁剪 |
| `--bag-freq` | 空 | 图像特征提取频率，单位 Hz | 对图像观测降频 |
| `--perform-synchronization` | 关闭 | Kalibr 内置 clock synchronization | 默认不做 |
| `--fixture_id` | `fixture` | corner-file 输出命名后缀 | 只影响结果文件名 |
| `--trim-imu-edge-count` | `imu_data_file` 模式为 `1000`，其他模式为 `0` | 裁掉首尾 IMU 样本 | 避免 spline 边界影响；wrapper 默认空，传入时转发 |
| `--cams` | 必填 | camchain YAML | 原生参数名是 `--cams`，不是 `--cam` |
| `--recompute-camera-chain-extrinsics` | 关闭 | 重新估计相机链外参 | 默认固定 camchain 外参；只建议调试 camchain 外参问题时打开 |
| `--reprojection-sigma` | `1.0` | 角点重投影噪声标准差，单位 px | 改变 camera residual 权重 |
| `--imu` | 必填 | 一个或多个 IMU YAML | 第一个 IMU 是 reference IMU |
| `--imu-delay-by-correlation` | 关闭 | 多 IMU 间延迟相关估计 | 多 IMU 时可打开 |
| `--imu-models` | 每个 IMU 为 `calibrated` | 每个 IMU 的估计模型 | 支持 `calibrated`、`scale-misalignment`、`scale-misalignment-size-effect` |
| `--target` | 必填 | target YAML | 与相机标定一致 |
| `--no-time-calibration` | 关闭 | 禁用 cam-IMU time offset 标定 | 原生命令默认会估时间偏移；wrapper 默认加此参数，除非使用 `--estimate-time-offset` |
| `--max-iter` | `30` | 最大优化迭代数 | 影响收敛和耗时 |
| `--recover-covariance` | 关闭 | 恢复设计变量协方差 | 会增加耗时/内存 |
| `--timeoffset-padding` | `0.03` | time offset 可变化范围，单位秒 | 影响 spline 时间边界 |
| `--pose-knots-per-second` | `100` | pose spline knot rate | 越大轨迹更灵活、变量更多 |
| `--bias-knots-per-second` | `50` | IMU bias spline knot rate | 越大 bias 更灵活、变量更多 |
| `--show-extraction` | 关闭 | 显示 target 提取过程 | 会禁用报告弹窗 |
| `--extraction-stepping` | 关闭 | 逐帧显示提取过程 | GUI 调试用 |
| `--verbose` | 关闭 | 详细日志 | 会强制 `--show-extraction` 并禁用报告弹窗 |
| `--dont-show-report` | 关闭 | 不弹出报告窗口 | Docker/无 GUI 建议打开 |
| `--export-poses` | 关闭 | 导出优化后的 pose CSV | wrapper 默认打开，可用 `--no-export-poses` 关闭 |

## 结果与判断

| 文件 | 说明 |
|---|---|
| `calibration_report.md` | 人读报告，包含输入、warning/error、结果摘要 |
| `calibration_report.json` | 机器可读报告 |
| `kalibr_cam_imu.log` | cam-IMU 原始 Kalibr 日志 |
| `cam-camchain.yaml` / `*-results-imucam.txt` | Kalibr 标定结果 |

## 常见限制

- `cam-cam` 多相机图片要求各 `camN` 图片数量一致，并按自然排序同步。
- 没有外部时间戳时，不支持多相机视频同步。
- `--fast-extraction always` 最快，但如果底层 rosbag/文件系统出现并发读取问题，建议用 `auto`。
- cam-IMU 的 time offset、knot rate、IMU 裁边会显著影响结果，评测时必须把 Kalibr 与 Ceres 的口径写清楚。
- 如果目标是让别人真正一键部署，建议发布 DockerHub 或 GHCR 预构建镜像；只让用户 clone 后本地 build 不算一键部署。
- 多 IMU 已进入 `cam-imu` wrapper；没有硬编码最大数量，实际上限来自 Kalibr 变量规模、内存、运行时间和命令行长度。
- bag 模式通过展开后的各 IMU YAML 的 `rostopic` 取数；H5/corner-file 模式通过多个 IMU CSV 取数。
