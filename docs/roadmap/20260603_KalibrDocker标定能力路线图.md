# Kalibr Docker 标定能力路线图

## 背景

这个仓库的目标不是简单把 Kalibr 放进 Docker。真正的目标是让外部用户在任何装了 Docker 的机器上，用一行命令完成相机标定，并且在失败或质量不佳时能看懂原因，不需要自己读 Kalibr 的长日志。

这意味着系统要同时解决四类问题：

- 输入形态不统一：用户可能给一个图片目录、一个视频、或多个相机子目录。
- 环境不统一：macOS、Windows、Ubuntu、虚拟机共享盘、Apple Silicon/amd64 都应该能用同一套入口。
- Kalibr 原始错误不够面向用户：优化发散、角点少、覆盖差、IMU 激励不足等问题需要转换成可执行建议。
- 速度和稳定性冲突：`--no-multithreading` 稳，但慢；多进程快，但原始 Kalibr 在某些环境下会因为 rosbag 读取而丢图或报错。

当前路线选择是：以 Docker wrapper 作为对外稳定入口，以 forked Kalibr 作为算法核心，把 ProductionCalibration `private/zc-shuangzi-debug` 中需要保留的 cam-cam / cam-imu 行为迁移进这个公开可构建的工程。

## 产品直觉

用户真正想要的是“给数据，拿结果，知道该怎么补拍”。所以 wrapper 不能只是执行 Kalibr，它要承担一个标定助理的角色。

对 cam-cam，用户通常不知道自己缺的是四角、边缘、中心、近远距离，还是标定板旋转姿态。报告应该把角点分布、检测数量、重投影误差、焦距异常、主点偏移、畸变系数都变成明确的 warning/error。

对 cam-imu，用户更难判断问题。外参、时间偏移、陀螺仪 residual、加速度 residual、视觉 residual、IMU 时间范围重叠都可能导致失败。报告至少要告诉用户是数据时间、激励、噪声参数、camchain 质量，还是优化设置的问题。

对速度，默认策略应该是“先快，出问题自动兜底”。单纯要求用户在快和稳之间选择，会把内部实现复杂性暴露给调用者。更好的接口是默认 `auto`：大多数情况下用快路径，检测到已知风险时自动回退，并在报告里解释发生了什么。

## 已完成能力

### Docker 统一入口

镜像名为 `kalibr-camera-calibration:20.04`。镜像基于 Ubuntu 20.04 / ROS Noetic / Kalibr，并内置 `vio_common` 的 bag 生成工具。

对外入口是：

- `cam-cam`：图片目录、单视频、多相机图片子目录。
- `cam-imu`：ProductionCalibration 兼容的 corner-file 模式，以及 H5 输入模式。

`cam-cam` 的输入判断规则：

- 输入目录下直接是图片：默认单相机，整理成 `mav0/cam0`。
- 输入是单个视频：抽帧后默认单相机。
- 输入目录下是 `cam0/cam1/...` 子目录：按多相机处理，要求每个相机图片数量一致。
- 多相机模式暂不支持视频，因为不同视频之间的帧同步没有外部时间戳时不可判定。

输出统一写到 `/output`，核心文件包括：

- `dataset/mav0/camN/*.png`
- `cam.bag`
- `kalibr_cam_cam.log` 或 `kalibr_cam_imu.log`
- `calibration_report.md`
- `calibration_report.json`
- Kalibr 生成的 camchain/results/report 文件

### cam-cam fast extraction

新增参数：

```text
--fast-extraction auto
--fast-extraction always
--fast-extraction never
```

默认是 `auto`。

`always` 是强制快路径，使用 Kalibr 多进程角点提取。

`never` 是强制稳路径，等价于给 Kalibr 加 `--no-multithreading`。

`auto` 是生产推荐路径：先多进程快跑；如果日志出现 rosbag 多进程读取异常，就归档 fast 尝试并自动用单线程重跑。归档文件包括：

- `kalibr_cam_cam_fast.log`
- `cam-camchain-fast.yaml`
- `cam-results-cam-fast.txt`
- `cam-report-cam-fast.pdf`

最终结果仍使用标准文件名：

- `kalibr_cam_cam.log`
- `cam-camchain.yaml`
- `cam-results-cam.txt`
- `cam-report-cam.pdf`

报告会记录：

- `Fast extraction mode`
- `Fallback used`
- `Fallback reason`
- 每次 attempt 的 return code 和 log 文件
- 每个相机的 `Extracted corners for X images (of Y images)`

### rosbag 多进程读取根因修复

最初为了稳定，我们用 `--no-multithreading` 避免多进程读 bag。但这会牺牲速度。

复测发现，把 bag 放到容器内部 `/tmp` 仍然会触发 `ROSBagFormatException`，所以根因不是 Parallels 共享盘或 macOS 文件系统，而是 fork 后子进程继承了父进程已经打开的 rosbag 文件句柄。多个进程共享同一个底层 open file description，并发 `seek/read` 时会互相移动文件偏移，导致某个 worker 读到错误位置，表现为 header 读取错误或实际提取图片数减少。

修复方式：

- `BagImageDatasetReader` 增加 `reopen()`。
- `TargetExtractor.multicoreExtractionWrapper()` 在 worker 启动时调用 `dataset.reopen()`。
- 每个 worker 独立打开自己的 rosbag handle，不再共享父进程的文件偏移。

这个修复保留了多进程速度，同时消除了已知的共享句柄读包风险。

实测 `cam2cam_clahe`：

- `--fast-extraction always`
- 无 `ROSBagException`
- 无 `ROSBagFormatException`
- `Extracted corners for 20 images (of 20 images)`
- `Fallback used: False`
- Kalibr 进程 return code 为 `0`

wrapper 最终退出码仍可能是 `2`，这是因为质量门限触发，例如这组数据重投影 RMS 标准差约 `3.19px`，被报告判为 `high_reprojection`。这不是 fast extraction 失败。

### cam-cam 诊断报告

报告解析 Kalibr 结果并输出质量指标：

- 重投影误差
- fx/fy 差异
- 主点偏移
- 畸变系数幅值
- 每个已标定 pinhole 相机的水平/垂直/对角视场角：`hfov` / `vfov` / `dfov`
- Kalibr 实际提取图像数
- 输入图片亮度、对比度、模糊度
- wrapper 轻量 AprilTag 检测的覆盖指标

FOV 计算已接入 `calibration_report.md` 和 `calibration_report.json`。多相机标定会按 `cam0`、`cam1`、... 输出 N 组 FOV。计算优先使用最终 `cam-camchain.yaml` 中每个相机自己的 `intrinsics` 和 `resolution`，并用主点到图像边缘的射线夹角计算水平/垂直 FOV，用图像两条对角线端点射线夹角的最大值计算 DFOV。

有一个重要细节：wrapper 的轻量 `dt_apriltags` 检测器并不总能完全复现 Kalibr 内部目标检测。当前已增加兜底：

- 如果 wrapper 轻量检测为 `0/20`，但 Kalibr 实际为 `20/20`，报告不会把覆盖指标当作失败依据。
- 报告会明确写出 `Kalibr extracted: 20/20`。
- 覆盖指标会标记为“包装器轻量检测器不可用”。

这避免了报告自相矛盾，也避免用户误以为 Kalibr 没提到角点。

### cam-imu Production 兼容路径

`cam-imu` 支持两种输入模式：

- corner-file 模式：`--corner-file`、`--image-timestamp-file`、`--imu-data-file`
- H5 模式：`--h5-file`、`--h5-timestamp-file`、`--imu-csv`

ProductionCalibration `private/zc-shuangzi-debug` 的 cam-imu 离线路径使用 corner-file 模式，并且要匹配以下参数：

- 开启时间偏移估计：`--estimate-time-offset`
- 时间偏移 padding：`--timeoffset-padding 0.04`
- 不导出 poses：`--no-export-poses`

一致性验证中，fixture 1 在参数完全匹配时，Docker 和 Production 输出只存在 `1e-12` 量级的浮点末位差异。结论是 cam-imu 算法路径已经可以复现旧 Production 行为；默认参数不同导致的差异不应被误判为算法不一致。

### cam-imu 提速参数

新增 wrapper 参数，并透传给 forked Kalibr：

```text
--max-iter
--pose-knots-per-second
--bias-knots-per-second
```

默认值保持旧行为：

```text
--max-iter 30
--pose-knots-per-second 100
--bias-knots-per-second 50
```

这些参数的设计原则是“可用于筛查，不默认改变正式结果”。

`--max-iter 10` 适合快速预检，用来确认时间戳是否对齐、残差趋势是否正常、外参是否明显跑飞。它不能替代正式标定，因为收敛程度变了。

降低 `pose/bias knots per second` 可以明显减少样条变量数量，加快 build problem 和 optimization。但它改变了运动模型自由度，可能影响外参、时间偏移和 residual，因此只能作为显式实验参数，不能默认开启。

## 当前一致性状态

### cam-imu

cam-imu 在参数匹配时已经验证为数值一致。当前推荐把 Docker 作为后续对外使用入口。

### cam-cam

需要分清三个对象：

1. Docker 当前 forked Kalibr fast mode。
2. Docker 当前 forked Kalibr stable mode。
3. 本地 ProductionCalibration 仓库里的旧 cam-cam Kalibr。

这次修复已经进入 Docker 所用的 forked Kalibr。Docker fast mode 不再复现之前的 rosbag 多进程读包异常，并且在 `cam2cam_clahe` 上提取 `20/20`。

但本地 ProductionCalibration 仓库里的 cam-cam Kalibr 代码目前没有 `BagImageDatasetReader.reopen()` 修复。因此，如果直接运行 ProductionCalibration 旧 cam-cam fast 路径，它仍可能遇到原来的多进程读 bag 风险，结果可能因为少用图片而和 Docker 不一致。

如果要让 cam-cam 与 ProductionCalibration 项目版严格一致，有两条路线：

- 迁移路线：把 `reopen()` 修复同步回 ProductionCalibration 的 cam-cam Kalibr，之后用相同 bag、相同 target、相同 camera model、相同 `KALIBR_MANUAL_FOCAL_LENGTH_INIT`、相同是否展示 report 的参数对比。
- 收敛路线：把 Docker fork 作为唯一生产入口，ProductionCalibration 不再直接跑旧 cam-cam Kalibr，只保留上层数据管理或历史脚本。

当前更推荐收敛到 Docker fork。原因是公开 Docker 工程能被外部用户构建、发布和复现；ProductionCalibration 是 private 仓库，不适合作为外部依赖。

## 对外使用契约

### cam-cam 输入契约

单相机图片目录：

```text
images/
  000.png
  001.png
  ...
```

单相机视频：

```text
video.mp4
```

多相机图片目录：

```text
multi_cam_input/
  cam0/
    000.png
    001.png
  cam1/
    000.png
    001.png
  cam2/
    000.png
    001.png
```

要求：

- 多相机每个 `camN` 图片数量一致。
- 文件按自然排序后按序号配对。
- 多相机不支持视频。
- 图片分辨率不一致时 wrapper 会 warning，并默认 crop + resize 到统一尺寸。
- 更推荐用户显式传 `--resize WIDTHxHEIGHT`，避免自动归一化尺寸不符合预期。

### target yaml 契约

当前主要支持 Kalibr 标准 `aprilgrid`：

```yaml
target_type: 'aprilgrid'
tagCols: 17
tagRows: 10
tagSize: 0.05
tagSpacing: 0.3
```

字段含义：

- `target_type`: 标定板类型，AprilGrid 使用 `'aprilgrid'`。
- `tagCols`: AprilTag 列数。
- `tagRows`: AprilTag 行数。
- `tagSize`: 单个 tag 黑色外边框边长，单位米。
- `tagSpacing`: tag 之间空隙与 `tagSize` 的比例，Kalibr 定义为 `space / tagSize`。

常见错误：

- 把角点数当成 tag 数。
- `tagRows/tagCols` 写反。
- `tagSize` 单位写成毫米。
- `tagSpacing` 写成实际米数，而不是比例。
- 图片里用的是 tag36h11 AprilGrid，但 yaml 对应了另一块板。

后续可以扩展 checkerboard，但对外默认应该优先引导 AprilGrid，因为 Kalibr 的 AprilGrid 在遮挡、视角变化和角点 ID 对应上更稳。

### cam-imu 输入契约

cam-imu 不是“只给图片和标定板”就能跑。它至少需要：

- 已知相机内参 camchain：通常来自 cam-cam 标定。
- IMU 噪声参数 yaml。
- 相机观测数据：corner-file 或 H5。
- 图像时间戳。
- IMU 数据文件。

因此，如果另一台电脑只有图片文件夹或一个视频，再加一个标定板 target yaml，只能直接运行 cam-cam。要运行 cam-imu，必须先准备好 corner-file/H5 这类相机观测输入，以及和它时间对齐的 IMU 数据。

corner-file 模式更接近 ProductionCalibration 当前离线流程。H5 模式适合未来把图像流统一打包。

## 下一阶段路线

### Phase 1: 发布公开镜像

目标是让外部用户不需要本地构建。

待办：

- 发布 `wangyendt/kalibr-camera-calibration:20.04` 到 DockerHub，或发布 `ghcr.io/wangyendt/kalibr-camera-calibration:20.04` 到 GHCR。
- 在 README 中明确 amd64 / Apple Silicon 的平台说明。
- 为每个 release 记录 Kalibr commit、vio_common commit、Docker image digest。

这样用户在新机器上只需要：

```text
docker pull ...
docker run ...
```

如果不发布镜像，用户必须先 `docker build` 或 `docker load`，严格来说就不是“一行标定”。

### Phase 2: cam-cam 结果一致性基准

目标是建立可重复的 cam-cam 数值基准。

待办：

- 选一组质量更好的 cam-cam 数据，不要用当前 `cam2cam_clahe` 这种重投影约 `3.19px` 的数据作为唯一基准。
- 对比 Docker fast、Docker never、Production patched fast。
- 固定 bag、target、models、focal init。
- 记录内参、畸变、重投影、使用图片数、Kalibr report 中 processed/used 数量。
- 明确允许的浮点误差范围。

判断标准：

- 如果三者提取图像数一致，且优化输入一致，结果应在浮点噪声范围内一致。
- 如果 Production 未同步 `reopen()` 修复，则不应把它作为 fast 一致性基准。

### Phase 3: 更可靠的覆盖诊断

当前 wrapper 的轻量 AprilTag 检测可能和 Kalibr 内部检测不一致。它适合预检，但不适合作为最终角点覆盖真相。

下一步更可靠的方式：

- 直接从 Kalibr target extraction 结果中读取角点位置。
- 或让 Kalibr 导出每张图的 target observation。
- 用同一份 observation 计算 3x3 覆盖、边缘缺失、中心偏移、尺度变化、roll/pitch/yaw proxy。

这样报告中的角点覆盖建议会和 Kalibr 真正使用的数据完全一致。

### Phase 4: cam-imu 速度网格实验

目标是找出“快但不明显损失质量”的参数组合。

候选实验维度：

- `--max-iter`: 10 / 15 / 20 / 30
- `--pose-knots-per-second`: 40 / 60 / 80 / 100
- `--bias-knots-per-second`: 20 / 30 / 40 / 50
- 是否估计 time offset
- 是否裁剪 IMU 边缘数据
- 是否降采样 IMU

每组记录：

- 总耗时
- build problem 耗时
- optimizer 耗时
- reprojection residual
- gyro residual
- accel residual
- timeshift
- T_cam_imu 旋转和平移差异
- 是否收敛

推荐策略：

- 默认正式标定仍用 Production 兼容参数。
- 快速预检可以降低 `--max-iter`。
- knot rate 降低必须先通过网格实验，不能凭单次结果变成默认值。

### Phase 5: 错误消息系统增强

目标是让用户看到的每个 warning/error 都能直接转化为补拍动作。

需要增强的消息类型：

- 图像上半区/下半区缺角点。
- 左右边缘缺样。
- 四角缺样。
- 标定板尺度变化不足，对应 z 方向激励不足。
- roll 变化不足。
- pitch/yaw 倾斜不足。
- 多相机共同可见 tag 太少。
- cam-imu 旋转轴激励不足。
- cam-imu 平移和重力方向姿态变化不足。
- 时间戳不重叠或单位疑似错误。
- IMU 噪声参数数量级异常。

报告不应该只说“质量差”，而应该说“下一轮采集应该补什么”。

## 当前结论

Docker fork 已经具备对外一行运行的基本形态。cam-cam fast 的核心稳定性问题已经修复，cam-imu 已经能复现 Production 兼容路径。

短期最重要的不是继续堆参数，而是完成公开镜像发布、建立一致性基准、把覆盖诊断改成基于 Kalibr 实际 observation。这样这个项目才能从“能跑”变成“外部用户可以放心使用并理解结果”。
