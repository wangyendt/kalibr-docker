# Camera Calibration Docker Wrapper

This wrapper turns image folders or one video into Kalibr-ready data, creates a
ROS bag with `vio_common`, runs Kalibr, and writes a human-readable diagnostic
report.

Build:

```bash
docker build -f docker/camera-calibration/Dockerfile -t kalibr-camera-calibration:20.04 .
```

Pin `vio_common` when needed:

```bash
docker build \
  --build-arg VIO_COMMON_REF=<branch-or-commit> \
  -f docker/camera-calibration/Dockerfile \
  -t kalibr-camera-calibration:20.04 .
```

Single camera from an image folder:

```bash
docker run --rm \
  -v /path/to/images:/input:ro \
  -v /path/to/target:/target:ro \
  -v /path/to/output:/output \
  kalibr-camera-calibration:20.04 \
  cam-cam --input /input --target /target --output /output
```

`cam-cam` can be omitted when the first argument is an option:

```bash
docker run --rm \
  -v /path/to/images:/input:ro \
  -v /path/to/target:/target:ro \
  -v /path/to/output:/output \
  kalibr-camera-calibration:20.04 \
  --input /input --target /target --output /output
```

Single camera from a video:

```bash
docker run --rm \
  -v /path/to/video.mp4:/input/video.mp4:ro \
  -v /path/to/target.yaml:/target.yaml:ro \
  -v /path/to/output:/output \
  kalibr-camera-calibration:20.04 \
  cam-cam --input /input/video.mp4 --target /target.yaml --output /output
```

Multi-camera from subfolders:

```text
input/
  cam0/*.png
  cam1/*.png
  cam2/*.png
```

Each camera folder must have the same number of images. Videos are intentionally
not supported in multi-camera mode because frame synchronization would be
ambiguous without external timestamps.

Useful options:

```bash
--lang zh                         # default warning/error language
--lang en                         # English output
--verbose                         # stream raw Kalibr logs; cam-cam also saves debug overlays
--resize 1280x720                 # force normalized image size
--preprocess clahe                # none, hist-eq, or clahe
--focal-length-init 2400          # exported to KALIBR_MANUAL_FOCAL_LENGTH_INIT
--fast-extraction auto            # fast multiprocessing first; fallback to single-thread on rosbag read failures
--fast-extraction always          # force fastest multiprocessing extraction; report error if a rosbag read failure is detected
--fast-extraction never           # force --no-multithreading for maximum stability
--skip-kalibr                     # only prepare data and diagnostics
```

Output includes:

- `dataset/mav0/camN/*.png`: normalized timestamped images.
- `cam.bag`: generated ROS bag.
- `kalibr_cam_cam.log`: Kalibr console log.
- `calibration_report.md` and `calibration_report.json`: diagnostics and result summary.
- Per-camera `hfov` / `vfov` / `dfov` for calibrated pinhole cameras.
- `debug/`: optional verbose diagnostic overlays.

Raw Kalibr stdout is saved to log files by default. Use `--verbose` only when
you want the full Kalibr progress printed to the terminal.

`cam-cam` defaults to `--fast-extraction auto`. It first runs Kalibr's
multiprocessing target extraction. Each worker reopens its own rosbag handle so
parallel `seek/read` does not share the parent process file descriptor. If the
log still contains rosbag multiprocessing read failures, the wrapper archives
the fast attempt as `kalibr_cam_cam_fast.log` / `cam-*-fast.*` and reruns
automatically with `--no-multithreading`. This keeps the common path fast while
preventing silent partial-image calibration results.


## Camera-IMU one-line examples

H5 image stream plus IMU CSV:

```bash
docker run --rm \
  -v /path/to/cam_imu_data:/data:ro \
  -v /path/to/output:/output \
  kalibr-camera-calibration:20.04 \
  cam-imu \
    --target /data/aprilgrid.yaml \
    --cam-chain /data/camchain.yaml \
    --imu-yaml /data/imu.yaml \
    --h5-file /data/images.h5 \
    --h5-timestamp-file /data/image_timestamps.txt \
    --imu-csv /data/imu.csv \
    --output /output
```

Pre-extracted corner file plus IMU data, compatible with the BenchmarkCalibration
corner-file path:

```bash
docker run --rm \
  -v /path/to/cam_imu_data:/data:ro \
  -v /path/to/output:/output \
  kalibr-camera-calibration:20.04 \
  cam-imu \
    --target /data/aprilgrid.yaml \
    --cam-chain /data/camchain.yaml \
    --imu-yaml /data/imu.yaml \
    --corner-file /data/corners.pkl \
    --image-timestamp-file /data/image_timestamps.txt \
    --imu-data-file /data/imu.csv \
    --fixture-id 1 \
    --output /output
```

Both `cam-cam` and `cam-imu` write `calibration_report.md` and
`calibration_report.json`. The report contains input diagnostics, parsed Kalibr
quality metrics, warnings/errors, and concrete data-collection suggestions.
`cam-imu` stages read-only inputs into `work_cam_imu/` so Docker input mounts can
stay read-only while Kalibr writes its result files.

`cam-imu` exposes speed/accuracy knobs while keeping the fork defaults:
`--max-iter 30`, `--pose-knots-per-second 100`, and
`--bias-knots-per-second 50`. Lower values can speed up screening runs, but they
change the optimizer problem and should be compared against the default result
before being used for deployment or benchmark reporting.
