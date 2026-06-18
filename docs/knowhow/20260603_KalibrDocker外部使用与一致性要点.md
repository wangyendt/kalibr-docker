# Kalibr Docker 外部使用与一致性要点

## 背景

这个 know-how 记录的是对外使用 Kalibr Docker 标定工具时最容易混淆的几件事：另一台电脑应该执行哪些命令、标定板 yaml 要写成什么格式、只给图片能跑到哪一步、cam-cam 的 Docker fast 模式和 BenchmarkCalibration 项目版是否一致。

这些信息需要从常用命令、路线图和一致性实验中抽出来单独保存。未来无论是写 README、发布 DockerHub/GHCR 镜像，还是给外部用户排查问题，都应该优先引用这份规则。

## 一句话结论

如果另一台电脑只有图片文件夹或一个视频，再加一个标定板配置文件，它可以直接运行 `cam-cam`。它不能直接运行 `cam-imu`。

`cam-imu` 至少还需要已知相机内参 camchain、IMU 噪声 yaml、相机观测文件 corner-file 或 H5、图像时间戳和 IMU 数据文件。

Docker 当前 forked Kalibr 的 cam-cam fast 模式已经修复了多进程读 bag 的共享句柄问题；但本地 BenchmarkCalibration 项目版 cam-cam 还没有同步这个 `reopen()` 修复，所以不能说 Docker fast 和 BenchmarkCalibration fast 现在已经严格一致。

## 新机器怎么拿到镜像

真正的“一行标定”前提是目标机器已经有镜像。

如果没有发布 DockerHub/GHCR 镜像，可以走离线镜像：

- 在已有镜像的机器上执行 `常用命令.txt` 第 28 行，把 `kalibr-camera-calibration:20.04` 保存成 tar。
- 把 tar 传到目标机器。
- 在目标机器执行第 31 行 `docker load`。
- 用第 16 行确认镜像存在。

如果已经发布公开镜像：

- DockerHub 使用第 34 行。
- GHCR 使用第 37 行。

外部发布后，用户才可以只执行一次 `docker pull`，之后每次标定都是一行 `docker run`。

## cam-cam 应该用哪些命令

常用命令中的行号如下：

| 场景 | 行号 | 说明 |
| --- | ---: | --- |
| 单相机图片文件夹 | 57 | `/ABS/images` 目录下直接放 png/jpg/jpeg/bmp/tif 图片 |
| 单相机视频 | 60 | `/ABS/video.mp4` 按 `--video-sample-fps` 抽帧 |
| 双相机图片文件夹 | 63 | `/ABS/multi_cam_input/cam0` 和 `cam1`，数量必须一致 |
| 三相机图片文件夹 | 66 | `cam0/cam1/cam2`，数量必须一致 |
| CLAHE 预处理 | 69 | 光照或局部对比度影响 AprilTag 时使用 |
| 强制 resize | 72 | 输入分辨率或宽高比不一致时建议显式指定 |
| 指定焦距初值 | 75 | 默认焦距初始化明显不合适时使用 |
| 推荐 fast auto | 78 | 默认推荐：先快跑，异常自动单线程兜底 |
| 强制 fast | 81 | 只适合确认环境稳定且愿意自己承担风险的实验 |
| 强制 stable | 84 | 等价于 `--no-multithreading`，更稳但慢 |
| verbose 调试 | 87 | 打印 Kalibr 原始日志并保存更多中间文件 |
| 只做预检 | 90 | 整理数据和诊断，不运行 Kalibr |

对外默认推荐第 57 行或第 78 行。第 78 行显式写出 `--fast-extraction auto`，更能让用户理解默认行为。

## cam-imu 应该用哪些命令

cam-imu 的常用命令行号如下：

| 场景 | 行号 | 说明 |
| --- | ---: | --- |
| BenchmarkCalibration 兼容 corner-file 模式 | 100 | 需要 corner pkl、图像时间戳、IMU 数据 |
| H5 模式 | 103 | 需要 images.h5、图像时间戳、IMU CSV |
| 开启时间偏移估计 | 106 | wrapper 默认关闭 time offset 标定 |
| 严格匹配 Benchmark 参数 | 109 | 用于 Docker 与 Benchmark 数值一致性验证 |
| verbose 调试 | 112 | 排查 IMU 激励、残差、时间偏移、优化失败 |
| 快速预检 | 115 | 降低 max iter，只看趋势，不能替代正式结果 |
| knot rate 提速实验 | 118 | 改变优化变量数量，必须和默认参数对比 |

如果用户只给图片文件夹和标定板 yaml，应该先跑 cam-cam，得到 camchain。之后还要准备 IMU 数据和相机观测文件，才能进入 cam-imu。

## 标定板 yaml 格式

当前对外默认推荐 Kalibr 标准 AprilGrid：

```yaml
target_type: 'aprilgrid'
tagCols: 17
tagRows: 10
tagSize: 0.05
tagSpacing: 0.3
```

字段解释：

- `target_type`: 标定板类型，AprilGrid 写 `'aprilgrid'`。
- `tagCols`: AprilTag 列数，不是角点数。
- `tagRows`: AprilTag 行数，不是角点数。
- `tagSize`: 单个 tag 黑色外边框边长，单位是米。
- `tagSpacing`: tag 间距与 `tagSize` 的比例，Kalibr 定义为 `space / tagSize`。

常见错误：

- 把角点数写成 tag 数。
- `tagCols` 和 `tagRows` 写反。
- `tagSize` 用毫米值，例如把 `0.05` 写成 `50`。
- `tagSpacing` 写成实际米数，而不是比例。
- 图片里使用的标定板和 yaml 中的行列数、tag 尺寸不一致。

## cam-cam 一致性边界

当前要区分三条路径：

1. Docker forked Kalibr fast mode。
2. Docker forked Kalibr stable mode。
3. BenchmarkCalibration 仓库里的旧 cam-cam Kalibr。

Docker forked Kalibr 已经加入：

- `BagImageDatasetReader.reopen()`
- `TargetExtractor.multicoreExtractionWrapper()` worker 启动时重新打开 bag
- wrapper 层 `--fast-extraction auto/always/never`

因此 Docker fast 在 `cam2cam_clahe` 上已经实测：

- 无 `ROSBagException`
- 无 `ROSBagFormatException`
- `Extracted corners for 20 images (of 20 images)`
- `Fallback used: False`

但是本地 BenchmarkCalibration 的 cam-cam Kalibr 代码还没有这个 `reopen()` 修复。如果直接运行旧 Benchmark fast 路径，它仍可能出现多进程读 bag 丢图，导致提取图像数不同，结果自然不会严格一致。

要建立严格一致性基准，需要先做其中一件事：

- 把 `reopen()` 修复同步回 BenchmarkCalibration。
- 或者明确 BenchmarkCalibration 不再作为 cam-cam 执行入口，以 Docker fork 为唯一基准。

在这之前，对外描述应该是：Docker fast 模式已修复并通过当前数据验证；BenchmarkCalibration 项目版尚未同步该修复，所以两者不保证一致。

## 为什么 fast auto 是默认推荐

`--fast-extraction never` 稳定，但会牺牲角点提取速度。

`--fast-extraction always` 快，但如果未来某个环境再次触发多进程读包异常，用户可能得到少用图的结果。

`--fast-extraction auto` 的直觉是让系统承担这部分复杂性：先走快路径；如果日志检测到 rosbag 读取异常，自动归档 fast 结果并单线程重跑。报告里会写清楚 fallback 是否发生、每次 attempt 的 return code 和 log 文件。

修复 `reopen()` 后，常见路径会停留在 fast，不会回退；`auto` 仍保留安全网。

## English Quick Reference

If a user only has an image folder or one video plus a target YAML, they can run `cam-cam` directly. They cannot run `cam-imu` directly.

`cam-imu` additionally requires a camera chain, an IMU noise YAML, camera observations as a corner file or H5 file, image timestamps, and IMU measurements.

The recommended target YAML is Kalibr's AprilGrid format:

```yaml
target_type: 'aprilgrid'
tagCols: 17
tagRows: 10
tagSize: 0.05
tagSpacing: 0.3
```

`tagSize` is in meters. `tagSpacing` is a ratio: spacing divided by tag size.

Docker fast cam-cam has been fixed in this fork by reopening the rosbag in each multiprocessing worker. The local BenchmarkCalibration cam-cam copy has not been patched yet, so Docker fast and BenchmarkCalibration fast should not be claimed as strictly identical until that patch is synchronized or BenchmarkCalibration is retired as the execution path.
