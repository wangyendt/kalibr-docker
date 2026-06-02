# Kalibr Camera Calibration Docker / Kalibr 标定 Docker

This fork keeps Kalibr as the calibration engine, but adds a Docker-first
workflow for practical camera calibration. The goal is simple: on any machine
with Docker installed, a user should be able to provide an image folder or one
video plus a calibration target YAML, run one command, and receive both Kalibr
results and a readable diagnostic report.

本 fork 仍然以 Kalibr 作为标定算法核心，但把工程目标转为 Docker-first 的实用标定工具：用户在任何装了 Docker 的机器上，给一个图片目录或一个视频，再给标定板 YAML，就能一行命令完成 cam-cam 标定，并拿到可读的质量报告和补拍建议。

## 中文说明

### 项目目标

这个仓库不是简单把 Kalibr 放进 Docker。它要解决外部用户真正会遇到的问题：

- 输入可能是单相机图片、单个视频、双相机/三相机图片子目录。
- 运行环境可能是 macOS、Windows、Ubuntu、虚拟机共享盘或 Apple Silicon。
- Kalibr 原始日志太长，用户需要明确的 warning/error 和补拍建议。
- cam-cam 需要尽量快，但不能因为多进程读 bag 丢图。
- cam-imu 需要保留 ProductionCalibration `private/zc-shuangzi-debug` 分支的离线 corner-file 路径，并支持后续 cam-imu 标定封装。

当前对外入口是 Docker 镜像 `kalibr-camera-calibration:20.04`，内部包含：

- Ubuntu 20.04 / ROS Noetic / Kalibr
- `vio_common` bag 生成工具
- `cam-cam` wrapper
- `cam-imu` wrapper
- Markdown/JSON 诊断报告
- 每个已标定相机的 `hfov` / `vfov` / `dfov` 视场角估算

### 快速开始

构建本地镜像：

```bash
docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .
```

如果镜像已发布到 DockerHub 或 GHCR，外部用户可以先 pull；发布前也可以通过 `docker save` / `docker load` 离线传递镜像。详见 [docs/常用命令.txt](docs/%E5%B8%B8%E7%94%A8%E5%91%BD%E4%BB%A4.txt)。

单相机图片文件夹：

```bash
docker run --rm -v /ABS/images:/input:ro -v /ABS/april_6x6.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input --target /target.yaml --output /output --lang zh
```

单相机视频：

```bash
docker run --rm -v /ABS/video.mp4:/input/video.mp4:ro -v /ABS/april_6x6.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input/video.mp4 --target /target.yaml --output /output --lang zh --video-sample-fps 4
```

双相机或三相机图片目录：

```text
multi_cam_input/
  cam0/*.png
  cam1/*.png
  cam2/*.png
```

```bash
docker run --rm -v /ABS/multi_cam_input:/input:ro -v /ABS/april_6x6.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input --target /target.yaml --output /output --models pinhole-radtan,pinhole-radtan --lang zh
```

多相机要求每个 `camN` 图片数量一致，文件自然排序后按同一序号配对。多相机视频暂不支持，因为没有外部时间戳时无法可靠同步。

### 标定板 YAML

当前默认推荐 Kalibr 标准 AprilGrid：

```yaml
target_type: 'aprilgrid'
tagCols: 17
tagRows: 10
tagSize: 0.05
tagSpacing: 0.3
```

注意：

- `tagCols` / `tagRows` 是 tag 数，不是角点数。
- `tagSize` 单位是米。
- `tagSpacing` 是比例，定义为 tag 间距除以 `tagSize`。
- YAML 必须和真实标定板的行列数、尺寸、tag family 匹配。

### cam-cam 快速模式

`cam-cam` 支持：

```text
--fast-extraction auto
--fast-extraction always
--fast-extraction never
```

默认推荐 `auto`：先用 Kalibr 多进程角点提取；如果日志检测到 rosbag 多进程读取异常，自动归档 fast 尝试并用单线程重跑。

这次 fork 已修复原始 Kalibr 多进程读 bag 的关键问题：worker 启动后会重新打开自己的 rosbag handle，避免 fork 后多个进程共享同一个文件偏移并发 `seek/read`。实测 `cam2cam_clahe` 中 `--fast-extraction always` 已能提取 `20/20`，并且 `Fallback used: False`。

### cam-cam FOV 输出

`cam-cam` 标定完成后，报告会为每个已标定 pinhole 相机计算视场角：

- `hfov`: 水平视场角
- `vfov`: 垂直视场角
- `dfov`: 对角视场角

多相机输入会按 `cam0`、`cam1`、... 分别输出 N 组 FOV。FOV 优先使用最终 `cam-camchain.yaml` 中的 `intrinsics` 和 `resolution` 计算，并写入 `calibration_report.md` 与 `calibration_report.json`。

### cam-imu 输入边界

只有图片文件夹或视频加 target YAML 时，只能直接跑 `cam-cam`。

`cam-imu` 还需要：

- cam-cam 产出的 camera chain
- IMU 噪声 YAML
- corner-file 或 H5 相机观测文件
- 图像时间戳
- IMU 数据文件

ProductionCalibration 兼容 corner-file 模式示例：

```bash
docker run --rm -v /ABS/cam_imu_data:/data:ro -v /ABS/output_cam_imu:/output kalibr-camera-calibration:20.04 cam-imu --target /data/aprilgrid.yaml --cam-chain /data/camchain.yaml --imu-yaml /data/imu.yaml --corner-file /data/corners.pkl --image-timestamp-file /data/image_timestamps.txt --imu-data-file /data/imu.csv --fixture-id 1 --output /output --lang zh
```

要严格匹配 ProductionCalibration `private/zc-shuangzi-debug` 的离线参数，需要显式加：

```text
--estimate-time-offset --timeoffset-padding 0.04 --no-export-poses
```

### 一致性状态

cam-imu 在参数完全匹配时，Docker 输出与 ProductionCalibration 旧输出已经验证为 `1e-12` 量级浮点末位差异。

cam-cam 需要更谨慎：Docker forked Kalibr 已经修复 fast 模式多进程读 bag 问题，但本地 ProductionCalibration 项目版 cam-cam 还没有同步 `BagImageDatasetReader.reopen()` 修复。因此不能声称 Docker fast 与 ProductionCalibration fast 现在已经严格一致。要么把修复同步回 ProductionCalibration，要么后续统一以 Docker fork 作为 cam-cam 执行入口。

### 文档入口

- 常用命令：[docs/常用命令.txt](docs/%E5%B8%B8%E7%94%A8%E5%91%BD%E4%BB%A4.txt)
- 外部使用与一致性要点：[docs/knowhow/20260603_KalibrDocker外部使用与一致性要点.md](docs/knowhow/20260603_KalibrDocker%E5%A4%96%E9%83%A8%E4%BD%BF%E7%94%A8%E4%B8%8E%E4%B8%80%E8%87%B4%E6%80%A7%E8%A6%81%E7%82%B9.md)
- 标定提速方案：[docs/knowhow/20260603_Kalibr标定提速方案.md](docs/knowhow/20260603_Kalibr%E6%A0%87%E5%AE%9A%E6%8F%90%E9%80%9F%E6%96%B9%E6%A1%88.md)
- Docker 路线图：[docs/roadmap/20260603_KalibrDocker标定能力路线图.md](docs/roadmap/20260603_KalibrDocker%E6%A0%87%E5%AE%9A%E8%83%BD%E5%8A%9B%E8%B7%AF%E7%BA%BF%E5%9B%BE.md)
- Docker 与 Production 一致性实验：[docs/experiment/20260602_Docker与ProductionCalibration一致性验证.md](docs/experiment/20260602_Docker%E4%B8%8EProductionCalibration%E4%B8%80%E8%87%B4%E6%80%A7%E9%AA%8C%E8%AF%81.md)

## English

### Project Goal

This repository is a Docker-first fork of Kalibr for practical camera
calibration. It is designed for users who want to provide image data and a
target file, run one command, and receive both calibration results and a
diagnostic report with actionable warnings.

The wrapper currently provides:

- `cam-cam`: single-camera images, one video, or multi-camera image folders.
- `cam-imu`: ProductionCalibration-compatible corner-file mode and H5 mode.
- automatic image normalization and bag creation through `vio_common`.
- human-readable `calibration_report.md` and machine-readable `calibration_report.json`.
- per-camera `hfov` / `vfov` / `dfov` estimates for calibrated pinhole cameras.
- Chinese and English warning/error messages.

### Quick Start

Build the image:

```bash
docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .
```

Single camera from an image folder:

```bash
docker run --rm -v /ABS/images:/input:ro -v /ABS/april_6x6.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input --target /target.yaml --output /output --lang en
```

Single camera from one video:

```bash
docker run --rm -v /ABS/video.mp4:/input/video.mp4:ro -v /ABS/april_6x6.yaml:/target.yaml:ro -v /ABS/output:/output kalibr-camera-calibration:20.04 cam-cam --input /input/video.mp4 --target /target.yaml --output /output --lang en --video-sample-fps 4
```

Multi-camera input uses `cam0`, `cam1`, ... subfolders. Every camera folder must
contain the same number of images. Multi-camera video input is intentionally not
supported without external synchronization timestamps.

### Target YAML

The recommended target format is Kalibr AprilGrid:

```yaml
target_type: 'aprilgrid'
tagCols: 17
tagRows: 10
tagSize: 0.05
tagSpacing: 0.3
```

`tagSize` is in meters. `tagSpacing` is the ratio `space / tagSize`.
`tagCols` and `tagRows` are tag counts, not corner counts.

### Fast cam-cam Extraction

`cam-cam` supports:

```text
--fast-extraction auto
--fast-extraction always
--fast-extraction never
```

`auto` is the recommended default. It runs Kalibr multiprocessing extraction
first and falls back to single-thread extraction only when a rosbag
multiprocessing read failure is detected.

This fork fixes the original multiprocessing bag-read issue by reopening the
rosbag inside each worker process. That prevents forked workers from sharing the
parent process file offset during concurrent `seek/read`.

### cam-cam FOV Output

After `cam-cam` finishes, the report includes one FOV record for each calibrated
pinhole camera: horizontal FOV, vertical FOV, and diagonal FOV. Multi-camera
inputs produce one record per `camN`. The values are computed from the final
`cam-camchain.yaml` intrinsics and resolution, then written to both
`calibration_report.md` and `calibration_report.json`.

### cam-imu Requirements

An image folder or video plus a target YAML is enough for `cam-cam`, but not for
`cam-imu`.

`cam-imu` also requires:

- a camera chain from cam-cam calibration,
- an IMU noise YAML,
- camera observations as a corner file or H5 file,
- image timestamps,
- IMU measurements.

### Compatibility

For cam-imu, Docker results match the ProductionCalibration-compatible path when
the same parameters are used:

```text
--estimate-time-offset --timeoffset-padding 0.04 --no-export-poses
```

For cam-cam, this Docker fork has the fixed fast path. The local
ProductionCalibration cam-cam copy has not yet received the `reopen()` rosbag
fix, so Docker fast mode and ProductionCalibration fast mode should not be
claimed as strictly identical until that patch is synchronized or the Docker
fork becomes the single execution path.

## Upstream Kalibr

![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

[![ROS1 Ubuntu 20.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml)
[![ROS1 Ubuntu 18.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1804_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1804_build.yaml)
[![ROS1 Ubuntu 16.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1604_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_1604_build.yaml)

## Introduction

Kalibr is a toolbox that solves the following calibration problems:

1. **Multi-Camera Calibration**: Intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view with support for a wide range of [camera models](https://github.com/ethz-asl/kalibr/wiki/supported-models).
1. **Visual-Inertial Calibration (CAM-IMU)**: Spatial and temporal calibration of an IMU w.r.t a camera-system along with IMU intrinsic parameters
1. **Multi-Inertial Calibration (IMU-IMU)**: Spatial and temporal calibration of an IMU w.r.t a base inertial sensor along with IMU intrinsic parameters (requires 1-aiding camera sensor).
1. **Rolling Shutter Camera Calibration**: Full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras.

To install follow the [install wiki page](https://github.com/ethz-asl/kalibr/wiki/installation) instructions for which you can either use Docker or install from source in a ROS workspace.
Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.
For questions or comments, please open an issue on Github.


## News / Events

* **Nov 24, 2022** - Some new visualization of trajectory and IMU rate for the generated report along with fixed support for exporting poses to file (see PR [#578](https://github.com/ethz-asl/kalibr/pull/578),[#581](https://github.com/ethz-asl/kalibr/pull/581),[#582](https://github.com/ethz-asl/kalibr/pull/582))
* **May 3, 2022** - Support for Ubuntu 20.04 along with Docker scripts have been merged into master via PR [#515](https://github.com/ethz-asl/kalibr/pull/515). A large portion was upgrading to Python 3. A special thanks to all the contributors that made this possible. Additionally, contributed fixes for the different validation and visualization scripts have been merged.
* **Febuary 3, 2020** - Initial Ubuntu 18.04 support has been merged via PR [#241](https://github.com/ethz-asl/kalibr/pull/241). Additionally, support for inputting an initial guess for focal length can be provided from the cmd-line on failure to initialize them.
* **August 15, 2018** - Double sphere camera models have been contributed to the repository via PR [#210](https://github.com/ethz-asl/kalibr/pull/210). If you are interested you can refer to the [paper](https://arxiv.org/abs/1807.08957) for a nice overview of the models in the repository.
* **August 25, 2016** - Rolling shutter camera calibration support was added as a feature via PR [#65](https://github.com/ethz-asl/kalibr/pull/65). The [paper](https://www.cv-foundation.org/openaccess/content_cvpr_2013/papers/Oth_Rolling_Shutter_Camera_2013_CVPR_paper.pdf) provides details for those interested.
* **May 18, 2016** - Support for multiple IMU-to-IMU spacial and IMU intrinsic calibration was released.
* **June 18, 2014** - Initial public release of the repository.


## Authors
* Paul Furgale
* Hannes Sommer
* Jérôme Maye
* Jörn Rehder
* Thomas Schneider ([email](thomas.schneider@voliro.com))
* Luc Oth


## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

## Acknowledgments
This work is supported in part by the European Union's Seventh Framework Programme (FP7/2007-2013) under grants #269916 (V-Charge), and #610603 (EUROPA2).

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
