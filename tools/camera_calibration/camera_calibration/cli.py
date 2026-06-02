import argparse
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List

from .common import CalibrationError, UserMessage, dedupe_messages, ensure_dir, format_messages, info, parse_size
from .diagnostics import analyze_dataset
from .image_normalizer import prepare_dataset
from .input_resolver import resolve_input, resolve_target
from .i18n import tr
from .kalibr_runner import collect_result_files, parse_kalibr_log, run_cam_cam_pipeline, run_cam_imu
from .quality import evaluate_cam_cam, evaluate_cam_imu
from .report import write_cam_imu_report, write_reports


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="camera_calibration",
        description="Prepare camera calibration data, create a ROS bag with vio_common, and run Kalibr.",
    )
    subparsers = parser.add_subparsers(dest="command")

    cam_cam = subparsers.add_parser("cam-cam", help="Run camera-camera or single-camera intrinsic calibration.")
    cam_cam.add_argument("--input", required=True, help="Image folder, one video file, or a folder containing camN subfolders.")
    cam_cam.add_argument("--target", required=True, help="Target YAML file or folder containing one target YAML.")
    cam_cam.add_argument("--output", required=True, help="Output directory.")
    cam_cam.add_argument("--models", default="pinhole-radtan", help="One model for all cameras or comma-separated per-camera models.")
    cam_cam.add_argument("--resize", default="", help="Force normalized size, e.g. 1280x720.")
    cam_cam.add_argument("--preprocess", default="none", choices=["none", "hist-eq", "clahe"], help="Optional grayscale preprocessing.")
    cam_cam.add_argument("--timestamp-fps", type=float, default=10.0, help="Synthetic image timestamp frequency.")
    cam_cam.add_argument("--video-sample-fps", type=float, default=4.0, help="Frame sampling frequency for single-video input.")
    cam_cam.add_argument("--max-video-frames", type=int, default=0, help="Limit sampled video frames; 0 means no explicit limit.")
    cam_cam.add_argument("--diagnostic-max-images", type=int, default=200, help="Maximum images per camera for diagnostics.")
    cam_cam.add_argument("--focal-length-init", type=float, default=None, help="Set KALIBR_MANUAL_FOCAL_LENGTH_INIT.")
    cam_cam.add_argument(
        "--fast-extraction",
        default="auto",
        choices=["auto", "always", "never"],
        help="cam-cam extraction mode: auto runs fast multiprocessing first and falls back to --no-multithreading on rosbag read failures; always forces fast mode; never forces single-thread extraction.",
    )
    cam_cam.add_argument("--lang", default="zh", choices=["zh", "en"], help="Warning/error language.")
    cam_cam.add_argument("--verbose", action="store_true", help="Save debug overlays and more intermediate files.")
    cam_cam.add_argument("--show-report", action="store_true", help="Allow Kalibr to open/show its report window.")
    cam_cam.add_argument("--skip-kalibr", action="store_true", help="Only prepare data and diagnostics.")

    cam_imu = subparsers.add_parser("cam-imu", help="Run Kalibr camera-IMU calibration on the fork's H5/CSV path.")
    cam_imu.add_argument("--target", required=True, help="Target YAML file or folder containing one target YAML.")
    cam_imu.add_argument("--lang", default="zh", choices=["zh", "en"], help="Warning/error language.")
    cam_imu.add_argument("--cam-chain", required=True, help="Camera chain YAML.")
    cam_imu.add_argument("--imu-yaml", required=True, help="IMU noise YAML.")
    cam_imu.add_argument("--h5-file", help="H5 image data file.")
    cam_imu.add_argument("--h5-timestamp-file", help="Image timestamp text file for --h5-file.")
    cam_imu.add_argument("--imu-csv", help="IMU CSV file in Kalibr-compatible order for --h5-file.")
    cam_imu.add_argument("--corner-file", help="Pre-extracted camera corner pickle file.")
    cam_imu.add_argument("--image-timestamp-file", help="Image timestamp text file for --corner-file.")
    cam_imu.add_argument("--imu-data-file", help="IMU CSV/TXT file for --corner-file.")
    cam_imu.add_argument("--fixture-id", default="fixture", help="Fixture id appended to corner-file Kalibr outputs.")
    cam_imu.add_argument("--trim-imu-edge-count", type=int, default=None, help="Discard this many IMU samples at both ends.")
    cam_imu.add_argument("--output", required=True, help="Output directory.")
    cam_imu.add_argument("--timeoffset-padding", type=float, default=0.03)
    cam_imu.add_argument("--max-iter", type=int, default=30, help="Maximum optimizer iterations passed to Kalibr.")
    cam_imu.add_argument("--pose-knots-per-second", type=int, default=100, help="Pose spline knot rate passed to the forked Kalibr cam-imu path.")
    cam_imu.add_argument("--bias-knots-per-second", type=int, default=50, help="IMU bias spline knot rate passed to the forked Kalibr cam-imu path.")
    cam_imu.add_argument("--no-time-calibration", dest="no_time_calibration", action="store_true", default=True, help="Disable time-offset calibration. This is the default.")
    cam_imu.add_argument("--estimate-time-offset", dest="no_time_calibration", action="store_false", help="Enable time-offset calibration.")
    cam_imu.add_argument("--export-poses", dest="export_poses", action="store_true", default=True, help="Export optimized poses. This is the default.")
    cam_imu.add_argument("--no-export-poses", dest="export_poses", action="store_false", help="Do not export optimized poses.")
    cam_imu.add_argument("--focal-length-init", type=float, default=None)
    cam_imu.add_argument("--verbose", action="store_true", help="Stream Kalibr output to the terminal in addition to saving logs.")

    return parser


def _namespace_to_dict(args: argparse.Namespace) -> Dict[str, Any]:
    data = vars(args).copy()
    return {key: str(value) if isinstance(value, Path) else value for key, value in data.items()}


def _filter_preliminary_diagnostic_messages(
    messages: List[UserMessage],
    extracted_counts: Dict[str, Dict[str, int]],
) -> List[UserMessage]:
    unreliable_codes = {"low_detection", "poor_coverage", "concentrated"}
    reliable_cameras = set()
    for cam_name, counts in extracted_counts.items():
        total = counts.get("total", 0)
        detected = counts.get("detected", 0)
        if total > 0 and detected / float(total) >= 0.8:
            reliable_cameras.add(cam_name)
    if not reliable_cameras:
        return messages
    filtered: List[UserMessage] = []
    for message in messages:
        if message.code in unreliable_codes and any(message.text.startswith(f"{cam}:") for cam in reliable_cameras):
            continue
        filtered.append(message)
    return filtered


def _run_cam_cam(args: argparse.Namespace) -> int:
    output_dir = ensure_dir(Path(args.output).expanduser().resolve())
    target_yaml = resolve_target(Path(args.target))
    copied_target = output_dir / target_yaml.name
    if target_yaml.resolve() != copied_target.resolve():
        shutil.copy2(str(target_yaml), str(copied_target))

    layout = resolve_input(Path(args.input), lang=args.lang)
    prepared = prepare_dataset(
        layout=layout,
        output_root=output_dir,
        resize=parse_size(args.resize),
        preprocess=args.preprocess,
        timestamp_fps=args.timestamp_fps,
        video_sample_fps=args.video_sample_fps,
        lang=args.lang,
        max_video_frames=args.max_video_frames,
    )

    debug_dir = output_dir / "debug" if args.verbose else None
    diagnostics = analyze_dataset(
        dataset_root=prepared.root,
        camera_names=prepared.camera_names,
        target_yaml=copied_target,
        debug_dir=debug_dir,
        lang=args.lang,
        max_images=args.diagnostic_max_images,
    )

    kalibr_summary = run_cam_cam_pipeline(
        dataset_root=prepared.root,
        target_yaml=copied_target,
        camera_names=prepared.camera_names,
        output_dir=output_dir,
        model_arg=args.models,
        focal_length_init=args.focal_length_init,
        show_report=args.show_report,
        skip_kalibr=args.skip_kalibr,
        lang=args.lang,
        fast_extraction=args.fast_extraction,
        stream=args.verbose,
    )

    messages: List[UserMessage] = []
    messages.extend(prepared.messages)
    messages.extend(_filter_preliminary_diagnostic_messages(diagnostics.messages, kalibr_summary.extracted_counts))
    for cam_diag in diagnostics.cameras:
        counts = kalibr_summary.extracted_counts.get(cam_diag.camera, {})
        if counts.get("detected", 0) > 0 and cam_diag.detected_count == 0:
            info(
                messages,
                "diagnostics_detector_mismatch",
                f"{cam_diag.camera}: {tr(args.lang, 'diagnostics_detector_mismatch')}",
                tr(args.lang, "diagnostics_detector_mismatch_fix"),
            )
    messages.extend(kalibr_summary.messages)
    kalibr_returncode = 0
    if kalibr_summary.calibration_result is not None:
        kalibr_returncode = kalibr_summary.calibration_result.returncode
        if kalibr_returncode != 0:
            messages.append(UserMessage("error", "kalibr_nonzero", tr(args.lang, "kalibr_nonzero"), tr(args.lang, "kalibr_nonzero_fix")))
    calibration_quality = None
    if not args.skip_kalibr:
        calibration_quality = evaluate_cam_cam(output_dir=output_dir, target_size=prepared.target_size, lang=args.lang)
        messages.extend(calibration_quality.messages)
    messages = dedupe_messages(messages)
    write_reports(
        output_dir=output_dir,
        prepared=prepared,
        diagnostics=diagnostics,
        kalibr_summary=kalibr_summary,
        target_yaml=copied_target,
        command_args=_namespace_to_dict(args),
        messages=messages,
        calibration_quality=calibration_quality,
    )
    if messages:
        print(format_messages(messages))
    print(f"Report written to {output_dir / 'calibration_report.md'}")
    if any(message.level == "error" for message in messages):
        return kalibr_returncode or 2
    if kalibr_summary.calibration_result is not None:
        return kalibr_returncode
    return 0


def _has_all(*values: Any) -> bool:
    return all(value is not None and str(value) != "" for value in values)


def _run_cam_imu(args: argparse.Namespace) -> int:
    output_dir = ensure_dir(Path(args.output).expanduser().resolve())
    target_yaml = resolve_target(Path(args.target))
    cam_chain = Path(args.cam_chain).expanduser().resolve()
    imu_yaml = Path(args.imu_yaml).expanduser().resolve()

    h5_mode = _has_all(args.h5_file, args.h5_timestamp_file, args.imu_csv)
    corner_mode = _has_all(args.corner_file, args.image_timestamp_file, args.imu_data_file)
    if h5_mode == corner_mode:
        raise CalibrationError(
            "cam-imu requires exactly one input mode: "
            "--h5-file/--h5-timestamp-file/--imu-csv or "
            "--corner-file/--image-timestamp-file/--imu-data-file."
        )

    result = run_cam_imu(
        target_yaml=target_yaml,
        cam_chain=cam_chain,
        imu_yaml=imu_yaml,
        h5_file=Path(args.h5_file).expanduser().resolve() if h5_mode else None,
        h5_timestamp_file=Path(args.h5_timestamp_file).expanduser().resolve() if h5_mode else None,
        imu_csv=Path(args.imu_csv).expanduser().resolve() if h5_mode else None,
        corner_file=Path(args.corner_file).expanduser().resolve() if corner_mode else None,
        image_timestamp_file=Path(args.image_timestamp_file).expanduser().resolve() if corner_mode else None,
        imu_data_file=Path(args.imu_data_file).expanduser().resolve() if corner_mode else None,
        fixture_id=args.fixture_id,
        trim_imu_edge_count=args.trim_imu_edge_count,
        output_dir=output_dir,
        timeoffset_padding=args.timeoffset_padding,
        max_iter=args.max_iter,
        pose_knots_per_second=args.pose_knots_per_second,
        bias_knots_per_second=args.bias_knots_per_second,
        no_time_calibration=args.no_time_calibration,
        export_poses=args.export_poses,
        focal_length_init=args.focal_length_init,
        log_path=output_dir / "kalibr_cam_imu.log",
        stream=args.verbose,
    )

    _, log_messages = parse_kalibr_log(result.log_path, args.lang)
    messages: List[UserMessage] = list(log_messages)
    if result.returncode != 0:
        messages.append(UserMessage("error", "kalibr_nonzero", tr(args.lang, "kalibr_nonzero"), tr(args.lang, "kalibr_nonzero_fix")))

    quality = evaluate_cam_imu(output_dir=output_dir, lang=args.lang)
    messages.extend(quality.messages)
    messages = dedupe_messages(messages)
    result_files = collect_result_files(output_dir)
    write_cam_imu_report(
        output_dir=output_dir,
        target_yaml=target_yaml,
        cam_chain=cam_chain,
        imu_yaml=imu_yaml,
        command_args=_namespace_to_dict(args),
        kalibr_result=result,
        calibration_quality=quality,
        messages=messages,
        result_files=result_files,
    )
    if messages:
        print(format_messages(messages))
    print(f"cam-imu log written to {result.log_path}")
    print(f"Report written to {output_dir / 'calibration_report.md'}")
    if any(message.level == "error" for message in messages):
        return result.returncode or 2
    return result.returncode


def main(argv: List[str] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 2
    try:
        if args.command == "cam-cam":
            return _run_cam_cam(args)
        if args.command == "cam-imu":
            return _run_cam_imu(args)
        parser.print_help()
        return 2
    except CalibrationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
