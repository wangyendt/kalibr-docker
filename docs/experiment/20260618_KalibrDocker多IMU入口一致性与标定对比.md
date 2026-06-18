# Kalibr Docker 多 IMU 入口一致性与标定对比

## 背景

`cam-imu` wrapper 已扩展为支持多个 `--imu-yaml`，并让 bag、H5、corner-file 三种入口都能传入多份 IMU 数据。需要验证两件事：

- 参数链路是否一致：官方 bag 入口和新增 H5/corner-file 入口是否都把同一组多 IMU 参数传给 `kalibr_calibrate_imu_camera`。
- 结果是否可信：同一 benchmark session 上，4 次单 IMU 标定和 1 次 4-IMU 联合标定的 residual、外参、timeshift 是否在合理范围内一致。

验证使用 session `2025_03_14_00_10_18`，公开文档中统一记为 benchmark dataset。该 session 的 `cam_imu` 目录包含 `cam0_640x400_corners.pkl`、`0_save_timestamp.txt`、`data1.csv` 到 `data4.csv`、`aprilgrid.yaml` 和 `cam0-camchain-640x400.yaml`。本 session 没有 `.bag` 或 `.h5` 文件，因此 bag/H5/corner 的真实同数据运行不能在这份数据上同时完成；本次对 bag/H5/corner 做命令构造一致性检查，对 corner-file 做真实标定运行。

## 固定配置

| 项目 | 值 |
| --- | --- |
| Docker image | `kalibr-camera-calibration:20.04` |
| 平台 | macOS Docker Desktop，镜像为 `linux/amd64`，主机为 Apple Silicon |
| 相机 | `cam0-camchain-640x400.yaml` |
| target | `aprilgrid.yaml` |
| 角点 | `cam0_640x400_corners.pkl` |
| 图像时间戳 | `0_save_timestamp.txt` |
| IMU 数据 | `data1.csv`、`data2.csv`、`data3.csv`、`data4.csv` |
| Kalibr 迭代 | `--max-iter 30` |
| 时间偏移 | `--estimate-time-offset` |
| 多 IMU 延迟 | 4-IMU 联合标定打开 `--imu-delay-by-correlation` |

注意：`--imu-yaml` 要传 Kalibr 输入型 flat IMU noise YAML。该 session 中历史 `cam0_640x400_corners-*-imu.yaml` 是 Kalibr 结果型 nested YAML，外层有 `imu0:`，直接传给官方 Kalibr 会报 `Field 'update_rate' missing`。本次验证从这 4 个结果 YAML 中提取 noise 参数，生成临时 flat YAML，并把 rostopic 设置为 `/imu0` 到 `/imu3`。

## 入口一致性

用 monkey patch 截获 wrapper 生成的原始命令，不启动 Kalibr。三种入口生成的共同参数链路一致：

| 入口 | `--imu` 数量 | `--imu-models` | 数据入口参数 | 数据文件数量 |
| --- | ---: | --- | --- | ---: |
| bag | 4 | `calibrated` x4 | `--bag` | 1 |
| H5 | 4 | `calibrated` x4 | `--imufile` | 4 |
| corner-file | 4 | `calibrated` x4 | `--imu_data_file` | 4 |

结论：wrapper 对 bag/H5/corner 三种模式保留同一组 `--target`、`--cams`、`--imu`、`--imu-models`、knot rate、time offset 参数，只替换数据源入口。bag 模式仍依赖 IMU YAML 内的 `rostopic` 匹配 bag topic；H5/corner-file 模式按 `--imu-yaml` 与 `--imu-csv` / `--imu-data-file` 的顺序一一对应。

## 运行命令模板

用下面变量替换本地 benchmark 根目录：

```bash
BENCH_ROOT=/ABS/benchmark_calibration/data/2025_03_14_00_10_18/cam_imu
OUT=/tmp/kalibr_multiimu_validation_20260618
```

单 IMU 示例：

```bash
docker run --rm --platform linux/amd64 -v "$BENCH_ROOT":/data:ro -v "$OUT/flat_imus":/imus:ro -v "$OUT/runs/imu1":/output kalibr-camera-calibration:20.04 cam-imu --target /data/aprilgrid.yaml --cam-chain /data/cam0-camchain-640x400.yaml --imu-yaml /imus/imu1.yaml --corner-file /data/cam0_640x400_corners.pkl --image-timestamp-file /data/0_save_timestamp.txt --imu-data-file /data/data1.csv --fixture-id imu1 --output /output --max-iter 30 --estimate-time-offset
```

4-IMU 联合示例：

```bash
docker run --rm --platform linux/amd64 -v "$BENCH_ROOT":/data:ro -v "$OUT/flat_imus":/imus:ro -v "$OUT/runs/imu4pack":/output kalibr-camera-calibration:20.04 cam-imu --target /data/aprilgrid.yaml --cam-chain /data/cam0-camchain-640x400.yaml --imu-yaml /imus/imu1.yaml /imus/imu2.yaml /imus/imu3.yaml /imus/imu4.yaml --imu-models calibrated calibrated calibrated calibrated --imu-delay-by-correlation --corner-file /data/cam0_640x400_corners.pkl --image-timestamp-file /data/0_save_timestamp.txt --imu-data-file /data/data1.csv /data/data2.csv /data/data3.csv /data/data4.csv --fixture-id imu4pack --output /output --max-iter 30 --estimate-time-offset
```

## 标定结果摘要

所有 5 次运行的 Kalibr return code 都是 `0`，结果文件均生成。wrapper 对 IMU1 和 4-IMU 联合运行返回过质量门禁 exit code `2`，原因是 `abs_time_shift` 超过 `0.10s` 阈值；这不代表 Kalibr 失败。

| Run | Kalibr return | Reproj mean px | Gyro mean rad/s | Accel mean m/s^2 | Timeshift s | `T_ci` translation m |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| cam-imu IMU1 | 0 | 0.179774 | 0.017779 | 0.108088 | -0.117530242 | `[0.072664, 0.096859, 0.076831]` |
| cam-imu IMU2 | 0 | 0.180073 | 0.019452 | 0.107901 | -0.095939135 | `[-0.084817, 0.097202, 0.075435]` |
| cam-imu IMU3 | 0 | 0.179053 | 0.017078 | 0.111222 | -0.013641893 | `[0.083318, 0.254837, 0.079823]` |
| cam-imu IMU4 | 0 | 0.179159 | 0.017749 | 0.103688 | 0.010181130 | `[-0.075021, 0.255333, 0.078150]` |
| cam-4imu joint | 0 | 0.210966 | 0.016290 avg | 0.117208 avg | -0.116868415 ref | `[0.070417, 0.101105, 0.082681]` ref |

联合标定的 reprojection mean 从单 IMU 的约 `0.179px` 上升到 `0.211px`，但仍处在可用范围。联合标定的 gyro residual 平均值略低，accel residual 平均值略高。更关键的是外参和时间偏移组合回各 IMU 后，与 4 次单跑结果接近。

## 联合结果与单跑结果对齐

联合结果直接输出 reference IMU 的 `T_ci`，并为其他 IMU 输出相对 reference IMU 的 `T_ib (imu0 to imui)` 和相对时间 offset。对比时使用：

- `T_cam_imui = T_cam_imu0 * inverse(T_ib_i)`
- `timeshift(cam to imui) = timeshift(cam to imu0) - time_offset_i`

| IMU | Joint gyro mean | Joint accel mean | Joint relative time offset vs IMU1 | Joint-composed shift | Single shift | Shift delta ms | Translation delta mm | Rotation delta deg |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| IMU1 | 0.015726 | 0.118592 | 0.000000000 | -0.116868415 | -0.117530242 | 0.662 | 7.570 | 0.2271 |
| IMU2 | 0.017213 | 0.121535 | -0.021344542 | -0.095523872 | -0.095939135 | 0.415 | 7.170 | 0.1329 |
| IMU3 | 0.016100 | 0.115667 | -0.104795150 | -0.012073265 | -0.013641893 | 1.569 | 6.421 | 0.1799 |
| IMU4 | 0.016122 | 0.113037 | -0.127949449 | 0.011081034 | 0.010181130 | 0.900 | 4.790 | 0.1438 |

## 结论

多 IMU wrapper 的参数链路通过了入口一致性检查。bag、H5、corner-file 都会把同一组多 IMU YAML 和 model 列表传入原始 Kalibr；H5/corner-file 还会按顺序传入同等数量的 IMU 数据文件。

corner-file 多 IMU 真实运行通过。4 次单 IMU 和 1 次 4-IMU 联合标定都生成了结果。联合结果组合回单个 IMU 后，translation 差异约 `4.8-7.6mm`，rotation 差异约 `0.13-0.23deg`，timeshift 差异约 `0.4-1.6ms`。这说明刚补的多 IMU corner-file 数据入口和官方 Kalibr 多 IMU 优化链路是连通的，且结果与单跑口径一致。

本次没有证明同一 session 上 bag/H5/corner 三种数据源的数值完全一致，因为该 session 没有 bag/H5 artifacts。若要做更强验证，需要准备同一采集序列的 bag、H5、corner-file 三套等价输入，并比较同一组 IMU YAML 下的 residual、外参和 timeshift。

## 后续建议

- 为公开示例补一份小型多 IMU fixture，包含 flat IMU YAML、corner pkl、timestamp 和 2-4 路 IMU CSV。
- 在参数文档中强调：`--imu-yaml` 是输入型 flat YAML，不是 Kalibr 输出型 nested `*-imu.yaml`。
- 如果希望 wrapper 更宽容，可以新增自动 flatten：检测到 YAML 顶层只有 `imu0:` 时，临时展开为 flat YAML 再传给 Kalibr。
