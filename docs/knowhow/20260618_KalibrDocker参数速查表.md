# Kalibr Docker 参数速查表

## 场景

这个仓库对外暴露的是 Docker wrapper，不是原始 Kalibr 工作空间。用户只需要记住镜像 `kalibr-camera-calibration:20.04` 和两个子命令：`cam-cam`、`cam-imu`。

## Docker 入口

| 命令 | 用途 | 影响 |
|---|---|---|
| `docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .` | 构建本地镜像 | 固定 Ubuntu 20.04 / ROS Noetic / Kalibr 环境 |
| `docker run --rm kalibr-camera-calibration:20.04 --help` | 查看总入口 | 验证镜像可用 |
| `docker run --rm kalibr-camera-calibration:20.04 cam-cam --help` | 查看 cam-cam 参数 | 只覆盖相机/多相机标定 |
| `docker run --rm kalibr-camera-calibration:20.04 cam-imu --help` | 查看 cam-imu 参数 | 只覆盖 IMU-camera 标定 |

挂载约定：输入目录建议只读挂载为 `:ro`，输出目录挂载为可写。Apple Silicon 或非 amd64 机器可加 `--platform linux/amd64`。

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
| `--imu-yaml` | 必填 | Kalibr IMU 噪声 YAML | 决定 gyro/accel 噪声权重 |
| `--output` | 必填 | 输出目录 | 写入日志、结果、报告 |
| `--lang` | `zh` | `zh/en` | 报告语言 |
| `--corner-file` | 空 | 预提取角点 pkl | corner-file 模式三件套之一 |
| `--image-timestamp-file` | 空 | corner-file 对应图像时间戳 | corner-file 模式三件套之一 |
| `--imu-data-file` | 空 | corner-file 模式 IMU CSV/TXT | corner-file 模式三件套之一 |
| `--h5-file` | 空 | H5 图像观测文件 | H5 模式三件套之一 |
| `--h5-timestamp-file` | 空 | H5 对应图像时间戳 | H5 模式三件套之一 |
| `--imu-csv` | 空 | H5 模式 IMU CSV | H5 模式三件套之一 |
| `--fixture-id` | `fixture` | corner-file 输出命名后缀 | 影响 Kalibr 结果文件名 |
| `--trim-imu-edge-count` | 空 | 裁掉首尾 IMU 样本数 | benchmark 对齐常用 `1000`，能避开 spline 边界 |
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

`cam-imu` 必须二选一输入模式：`--corner-file/--image-timestamp-file/--imu-data-file` 或 `--h5-file/--h5-timestamp-file/--imu-csv`，不能混用。

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
