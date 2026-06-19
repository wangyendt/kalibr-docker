---
name: camera-calibration
description: "Skill for the Camera_calibration area of kalibr-docker. 62 symbols across 9 files."
---

# Camera_calibration

62 symbols | 9 files | Cohesion: 68%

## When to Use

- Working with code in `tools/`
- Understanding how error, tr, evaluate_cam_cam work
- Modifying camera_calibration-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `tools/camera_calibration/camera_calibration/quality.py` | _read_first, _floats, _status, _add_metric, evaluate_cam_cam (+9) |
| `tools/camera_calibration/camera_calibration/common.py` | error, ensure_dir, dedupe_messages, format_messages, sorted_natural (+5) |
| `tools/camera_calibration/camera_calibration/kalibr_runner.py` | _archive_fast_cam_cam_outputs, parse_kalibr_log, collect_result_files, run_cam_cam_pipeline, run_logged (+3) |
| `tools/camera_calibration/camera_calibration/image_normalizer.py` | read_grayscale, image_size, choose_auto_target_size, normalize_image, prepare_dataset (+3) |
| `tools/camera_calibration/camera_calibration/input_resolver.py` | resolve_target, _is_image, _is_video, _direct_images, _direct_videos (+2) |
| `tools/camera_calibration/camera_calibration/cli.py` | _namespace_to_dict, _run_cam_cam, _normalize_imu_yamls, _run_cam_imu, main (+1) |
| `tools/camera_calibration/camera_calibration/report.py` | _message_lines, _result_lines, _quality_lines, write_reports, write_cam_imu_report |
| `tools/camera_calibration/camera_calibration/diagnostics.py` | _image_files, _summarize_camera, analyze_dataset |
| `tools/camera_calibration/camera_calibration/i18n.py` | tr |

## Entry Points

Start here when exploring this area:

- **`error`** (Function) — `tools/camera_calibration/camera_calibration/common.py:92`
- **`tr`** (Function) — `tools/camera_calibration/camera_calibration/i18n.py:177`
- **`evaluate_cam_cam`** (Function) — `tools/camera_calibration/camera_calibration/quality.py:253`
- **`evaluate_cam_imu`** (Function) — `tools/camera_calibration/camera_calibration/quality.py:330`
- **`evaluate_camchain_intrinsics`** (Function) — `tools/camera_calibration/camera_calibration/quality.py:392`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `error` | Function | `tools/camera_calibration/camera_calibration/common.py` | 92 |
| `tr` | Function | `tools/camera_calibration/camera_calibration/i18n.py` | 177 |
| `evaluate_cam_cam` | Function | `tools/camera_calibration/camera_calibration/quality.py` | 253 |
| `evaluate_cam_imu` | Function | `tools/camera_calibration/camera_calibration/quality.py` | 330 |
| `evaluate_camchain_intrinsics` | Function | `tools/camera_calibration/camera_calibration/quality.py` | 392 |
| `main` | Function | `tools/camera_calibration/camera_calibration/cli.py` | 358 |
| `ensure_dir` | Function | `tools/camera_calibration/camera_calibration/common.py` | 49 |
| `dedupe_messages` | Function | `tools/camera_calibration/camera_calibration/common.py` | 96 |
| `format_messages` | Function | `tools/camera_calibration/camera_calibration/common.py` | 108 |
| `resolve_target` | Function | `tools/camera_calibration/camera_calibration/input_resolver.py` | 111 |
| `sorted_natural` | Function | `tools/camera_calibration/camera_calibration/common.py` | 32 |
| `info` | Function | `tools/camera_calibration/camera_calibration/common.py` | 84 |
| `resolve_input` | Function | `tools/camera_calibration/camera_calibration/input_resolver.py` | 50 |
| `to_jsonable` | Function | `tools/camera_calibration/camera_calibration/common.py` | 54 |
| `write_json` | Function | `tools/camera_calibration/camera_calibration/common.py` | 66 |
| `write_reports` | Function | `tools/camera_calibration/camera_calibration/report.py` | 70 |
| `write_cam_imu_report` | Function | `tools/camera_calibration/camera_calibration/report.py` | 190 |
| `read_yaml` | Function | `tools/camera_calibration/camera_calibration/common.py` | 70 |
| `analyze_dataset` | Function | `tools/camera_calibration/camera_calibration/diagnostics.py` | 244 |
| `warn` | Function | `tools/camera_calibration/camera_calibration/common.py` | 88 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Evaluate_camchain_intrinsics → _angle_between_rays_deg` | cross_community | 4 |
| `Evaluate_camchain_intrinsics → Corner` | cross_community | 4 |
| `Prepare_dataset → Ensure_dir` | cross_community | 4 |
| `Prepare_dataset → Write_png` | cross_community | 4 |
| `Main → Sorted_natural` | cross_community | 4 |
| `Main → _is_video` | cross_community | 4 |
| `Main → Info` | cross_community | 4 |
| `Main → Tr` | cross_community | 4 |
| `Main → _is_image` | cross_community | 4 |
| `Main → Ensure_dir` | cross_community | 4 |

## How to Explore

1. `context({name: "error"})` — see callers and callees
2. `query({query: "camera_calibration"})` — find related execution flows
3. Read key files listed above for implementation details
