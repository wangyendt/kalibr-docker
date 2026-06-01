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
--verbose                         # save debug overlays and more logs
--resize 1280x720                 # force normalized image size
--preprocess clahe                # none, hist-eq, or clahe
--focal-length-init 2400          # exported to KALIBR_MANUAL_FOCAL_LENGTH_INIT
--skip-kalibr                     # only prepare data and diagnostics
```

Output includes:

- `dataset/mav0/camN/*.png`: normalized timestamped images.
- `cam.bag`: generated ROS bag.
- `kalibr_cam_cam.log`: Kalibr console log.
- `calibration_report.md` and `calibration_report.json`: diagnostics and result summary.
- `debug/`: optional verbose diagnostic overlays.
