# Docker 与 ProductionCalibration 一致性验证

## 背景

目标是确认 `kalibr-camera-calibration:20.04` Docker 镜像能复现 ProductionCalibration `private/zc-shuangzi-debug` 分支的 cam-cam 和 cam-imu 标定结果。验证使用本地镜像 `5236e909dd6b`，ProductionCalibration 临时 worktree 为 `/private/tmp/production_calibration_zc_shuangzi_debug`，分支 HEAD 为 `e6ff47d delete old data when calibration starts`。

## 数据与命令口径

cam-imu 使用 `/Users/wayne/Documents/work/data/cam_imu_2`。Production 分支脚本实际参数是 `dataN.csv`、`--timeoffset-padding 0.04`、开启 temporal calibration、不导出 poses。因此 Docker 一致性命令必须带 `--estimate-time-offset --timeoffset-padding 0.04 --no-export-poses`。

cam-cam 使用 `/Users/wayne/Documents/work/data/ost_calibration/vpcam_to_vpcam/cam2cam_clahe`。该目录没有现成 Production 输出，所以验证方式是复用 Docker 生成的同一个 `cam.bag`，在容器内直接执行 Production 脚本等价的原始 `rosrun kalibr kalibr_calibrate_cameras` 命令。

## cam-imu 结果

fixture 1 在匹配 Production 参数后，Docker 输出与目录中已有 Production 输出只存在浮点末位差异。

| 指标 | Production | Docker | 最大差异 |
| --- | ---: | ---: | ---: |
| reprojection mean px | 0.2036812221853951 | 0.2036812221864683 | 1.07e-12 |
| gyro mean rad/s | 0.17005413959270102 | 0.17005413958939428 | 3.31e-12 |
| accel mean m/s^2 | 0.8166658975791938 | 0.8166658975778435 | 1.35e-12 |
| timeshift s | -0.5465044612341835 | -0.5465044612331194 | 1.06e-12 |
| camchain intrinsics | identical | identical | 0 |
| camchain distortion | identical | identical | 0 |
| T_cam_imu | near-identical | near-identical | 7.97e-13 |

第一次用 Docker 默认 cam-imu 参数跑 fixture 1 时，结果不一致：Docker 默认关闭 temporal calibration，timeshift 为 `0s`，而 Production 输出为 `-0.5465s`。这不是算法差异，而是参数口径不同。

fixture 2 按同样 Production 参数启动后，Kalibr 日志停在 `Progress 22886 / 22891`，长时间没有生成 report，已手动停止容器。该 fixture 没有形成完整对照结果。

## cam-cam 结果

Production 分支 cam-cam 脚本固定 `KALIBR_MANUAL_FOCAL_LENGTH_INIT=500`，原始 Kalibr 命令没有 `--no-multithreading`。Docker wrapper 使用同样焦距初值，但增加 `--no-multithreading`，用于避免多进程读 rosbag 的不稳定。

原始 Production 等价命令在容器内运行时出现 rosbag 多进程 header 读取异常，最终 `Extracted corners for 18 images (of 20 images)`，并 `Processed 18 images with 16 images used`。Docker wrapper 路径则 `Extracted corners for 20 images (of 20 images)`，并 `Processed 20 images with 18 images used`。

| 指标 | Production 等价命令 | Docker wrapper | 最大差异 |
| --- | ---: | ---: | ---: |
| fx/fy/cx/cy | [1456.4724, 1456.6528, 1510.3797, 1507.2719] | [1457.0162, 1457.2959, 1509.1411, 1506.2027] | 1.2386 px |
| distortion | [0.060580, -0.051796, 0.000254, -0.000116] | [0.060565, -0.051967, 0.000457, -0.000184] | 2.03e-4 |
| reprojection std | [2.373479, 2.144453] | [2.362993, 2.164200] | 0.019747 px |

两条路径结果接近，但不能逐字一致。差异的直接原因是 Production 原始命令多进程读 bag 时丢了部分图，wrapper 的 `--no-multithreading` 避免了这个问题。

另外，`cam2cam_clahe` 的重投影 RMS 标准差约 `3.204px`，wrapper 判定为质量 error。链路能跑通，但这组图不适合作为合格标定样例。

## 结论

cam-imu 在参数完全匹配时已经验证为数值一致，差异在 `1e-12` 量级。

cam-cam 的 Docker wrapper 与 Production 原始脚本不是逐字一致，因为 wrapper 为了稳定性增加了 `--no-multithreading`。如果目标是生产可靠运行，保留 wrapper 当前行为更合理；如果目标是复刻旧脚本的逐字输出，需要增加一个兼容开关或把 Production 脚本也改为 `--no-multithreading`。

对外一行命令运行的前提是目标机器已有镜像。外部用户没有本地镜像时，需要 `docker build`、`docker load`，或从 DockerHub/GHCR `docker pull`。如果希望 Windows/Ubuntu 用户真正只执行一行标定命令，建议发布公开镜像。
