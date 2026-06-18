# Kalibr OneShot Docker / 一键相机标定 Docker

把 Kalibr 的 `cam-cam` 和 `cam-imu` 封进一个可交付的 Docker 入口：准备图片、视频、corner-file 或 H5 数据，跑一条命令，拿到标定结果、日志和可读诊断报告。

这个仓库适合交给外部用户或自动化评测脚本调用：不要求安装 ROS，不要求理解 Kalibr 的工作空间结构，也不要求手工翻长日志。

## 它解决什么

| 能力 | 输入 | 输出 |
|---|---|---|
| `cam-cam` | 单相机图片、单视频、`cam0/cam1/...` 多相机图片目录 + AprilGrid YAML | Kalibr camchain、FOV、Markdown/JSON 诊断报告 |
| `cam-imu` wrapper | 多相机 camchain、一个或多个 IMU YAML、bag，或 corner-file/H5 + 每个 IMU 一份 CSV | Kalibr IMU-camera 标定结果、日志、报告 |
| 原始 Kalibr CLI | ROS bag、camchain、一个或多个 IMU YAML | 低层调试和复刻官方命令 |
| 批量/离线交付 | DockerHub/GHCR 镜像、Docker image tar 或本地 build | 任意装了 Docker 的机器一键运行 |

## 快速开始

最省事的公开交付方式是发布预构建镜像到 DockerHub 或 GHCR。用户只需要 `docker pull`，不需要在本机编译 ROS/Kalibr；本地 build 更适合开发者调试或私有 fork。

```bash
# 推荐给普通用户：直接拉公开镜像，镜像名发布后替换为你的真实 namespace。
docker pull wangyendt/kalibr-camera-calibration:20.04
docker run --rm wangyendt/kalibr-camera-calibration:20.04 --help

# 开发者或私有环境：从源码本地构建。
docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .
docker run --rm kalibr-camera-calibration:20.04 --help
```

单相机图片标定：

```bash
docker run --rm -v /ABS/images:/input:ro -v /ABS/aprilgrid.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input --target /target.yaml --output /output --lang zh
```

BenchmarkCalibration 兼容的 cam-IMU corner-file 模式：

```bash
docker run --rm -v /ABS/cam_imu_data:/data:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-imu --target /data/aprilgrid.yaml --cam-chain /data/camchain.yaml --imu-yaml /data/imu.yaml --corner-file /data/corners.pkl --image-timestamp-file /data/image_timestamps.txt --imu-data-file /data/imu.csv --fixture-id 1 --trim-imu-edge-count 1000 --timeoffset-padding 0.04 --output /output --lang zh
```

## 当前支持的输入

- `cam-cam` wrapper：图片目录、单视频、单张图片、`cam0/cam1/...` 多相机图片目录；多相机要求每路图片数量一致并按文件名自然排序同步。
- `cam-imu` wrapper：多相机 camchain、一个或多个 `--imu-yaml`、`--bag`，或 corner-file/H5 + 图像时间戳 + 每个 IMU 一份 CSV；这是离线标定入口，不是直接吃图片目录的入口。
- 原始 Kalibr CLI：镜像里仍可直接运行 `rosrun kalibr ...`，用于低层调试、复刻官方命令或检查 wrapper 生成的命令。
- 当前 wrapper 不做多相机视频同步；没有外部时间戳时无法判断多个视频之间的同步关系。

## 为什么这个 fork 更适合交付

- Docker 内置 Ubuntu 20.04 / ROS Noetic / Kalibr / `vio_common`，环境固定。
- `cam-cam` 会自动整理图片/视频、生成 bag、跑 Kalibr、输出报告。
- 支持 `--fast-extraction auto`：优先多进程提角点，遇到 rosbag 并发读取异常自动回退单线程。
- 报告里直接给 warning/error、补拍建议、每个 pinhole 相机的 `hfov/vfov/dfov`。
- `cam-imu` 支持 bag、corner-file、H5 三种输入；多 IMU 时每个 IMU YAML 对应 bag topic 或一份 IMU CSV。

## Ceres Cam-IMU 子工程

Kalibr cam-IMU 的完整公式推导和 C++/Ceres 复现在独立仓库：

```bash
git submodule update --init --recursive kalibr-camimu-ceres
```

如果没有走 submodule，也可以在本仓库根目录克隆：

```bash
git clone ../kalibr-camimu-ceres kalibr-camimu-ceres
```

该子工程把 Kalibr cam-IMU 推导成书，并用 Ceres 复现独立求解链路。12 组 benchmark 数据上，Ceres 原生二进制平均墙钟约 `103 s`，Kalibr Docker 基线约 `203 s`；严格倍率需要在同一 native Linux 环境复测。实验入口见 `kalibr-camimu-ceres/docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md`。

## 文档

- 常用命令：[docs/常用命令.txt](docs/%E5%B8%B8%E7%94%A8%E5%91%BD%E4%BB%A4.txt)
- Docker 参数速查：[docs/knowhow/20260618_KalibrDocker参数速查表.md](docs/knowhow/20260618_KalibrDocker%E5%8F%82%E6%95%B0%E9%80%9F%E6%9F%A5%E8%A1%A8.md)
- 外部使用与一致性：[docs/knowhow/20260603_KalibrDocker外部使用与一致性要点.md](docs/knowhow/20260603_KalibrDocker%E5%A4%96%E9%83%A8%E4%BD%BF%E7%94%A8%E4%B8%8E%E4%B8%80%E8%87%B4%E6%80%A7%E8%A6%81%E7%82%B9.md)
- Docker 与 Benchmark 一致性实验：[docs/experiment/20260602_Docker与BenchmarkCalibration一致性验证.md](docs/experiment/20260602_Docker%E4%B8%8EBenchmarkCalibration%E4%B8%80%E8%87%B4%E6%80%A7%E9%AA%8C%E8%AF%81.md)

## English

Kalibr OneShot Docker is a delivery-focused Kalibr fork. It wraps `cam-cam` and `cam-imu` calibration behind one Docker image, so users can run practical calibration without installing ROS or learning the original Kalibr workspace layout.

Core commands:

```bash
# Public delivery: publish a prebuilt image, then users only pull and run.
docker pull wangyendt/kalibr-camera-calibration:20.04
docker run --rm wangyendt/kalibr-camera-calibration:20.04 --help

# Developer fallback: build locally from source.
docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .
docker run --rm kalibr-camera-calibration:20.04 cam-cam --help
docker run --rm kalibr-camera-calibration:20.04 cam-imu --help
```

Supported inputs: the wrapper handles image folders, one video, multi-camera image folders, and offline cam-IMU bag/corner-file/H5 inputs. `cam-imu` accepts multiple IMU YAMLs; bag mode reads the topics from those YAMLs, and corner-file/H5 modes take one IMU CSV per IMU.

What this fork adds:

- Docker-first `cam-cam` and `cam-imu` wrappers.
- Image/video normalization, bag creation, Kalibr execution, and reports in one run.
- Chinese/English diagnostics with actionable warnings.
- Fast `cam-cam` extraction with automatic fallback.
- BenchmarkCalibration-compatible offline `cam-imu` inputs.

The Ceres reimplementation and derivation book live in the `kalibr-camimu-ceres` submodule. It documents Kalibr cam-IMU from equations to implementation and provides a standalone C++/Ceres solver.

## Upstream And License

This repository is based on ETH ASL Kalibr. The original BSD license text is retained in [LICENSE](LICENSE), including the upstream copyright notice and redistribution terms.
