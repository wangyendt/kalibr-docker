import argparse
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List

from .common import CalibrationError, UserMessage, ensure_dir, format_messages, parse_size
from .diagnostics import analyze_dataset
from .image_normalizer import prepare_dataset
from .input_resolver import resolve_input, resolve_target
from .kalibr_runner import run_cam_cam_pipeline, run_cam_imu
from .report import write_reports


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
    cam_cam.add_argument("--lang", default="zh", choices=["zh", "en"], help="Warning/error language.")
    cam_cam.add_argument("--verbose", action="store_true", help="Save debug overlays and more intermediate files.")
    cam_cam.add_argument("--show-report", action="store_true", help="Allow Kalibr to open/show its report window.")
    cam_cam.add_argument("--skip-kalibr", action="store_true", help="Only prepare data and diagnostics.")

    cam_imu = subparsers.add_parser("cam-imu", help="Run Kalibr camera-IMU calibration on the fork's H5/CSV path.")
    cam_imu.add_argument("--target", required=True, help="Target YAML file or folder containing one target YAML.")
    cam_imu.add_argument("--cam-chain", required=True, help="Camera chain YAML.")
    cam_imu.add_argument("--imu-yaml", required=True, help="IMU noise YAML.")
    cam_imu.add_argument("--h5-file", required=True, help="H5 image data file.")
    cam_imu.add_argument("--h5-timestamp-file", required=True, help="Image timestamp text file.")
    cam_imu.add_argument("--imu-csv", required=True, help="IMU CSV file in Kalibr-compatible order.")
    cam_imu.add_argument("--output", required=True, help="Output directory.")
    cam_imu.add_argument("--timeoffset-padding", type=float, default=0.03)
    cam_imu.add_argument("--no-time-calibration", dest="no_time_calibration", action="store_true", default=True, help="Disable time-offset calibration. This is the default.")
    cam_imu.add_argument("--estimate-time-offset", dest="no_time_calibration", action="store_false", help="Enable time-offset calibration.")
    cam_imu.add_argument("--export-poses", dest="export_poses", action="store_true", default=True, help="Export optimized poses. This is the default.")
    cam_imu.add_argument("--no-export-poses", dest="export_poses", action="store_false", help="Do not export optimized poses.")
    cam_imu.add_argument("--focal-length-init", type=float, default=None)

    return parser


def _namespace_to_dict(args: argparse.Namespace) -> Dict[str, Any]:
    data = vars(args).copy()
    return {key: str(value) if isinstance(value, Path) else value for key, value in data.items()}


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
    )

    messages: List[UserMessage] = []
    messages.extend(prepared.messages)
    messages.extend(diagnostics.messages)
    messages.extend(kalibr_summary.messages)
    write_reports(
        output_dir=output_dir,
        prepared=prepared,
        diagnostics=diagnostics,
        kalibr_summary=kalibr_summary,
        target_yaml=copied_target,
        command_args=_namespace_to_dict(args),
        messages=messages,
    )
    if messages:
        print(format_messages(messages))
    print(f"Report written to {output_dir / 'calibration_report.md'}")
    if kalibr_summary.calibration_result is not None:
        return kalibr_summary.calibration_result.returncode
    return 0


def _run_cam_imu(args: argparse.Namespace) -> int:
    output_dir = ensure_dir(Path(args.output).expanduser().resolve())
    target_yaml = resolve_target(Path(args.target))
    result = run_cam_imu(
        target_yaml=target_yaml,
        cam_chain=Path(args.cam_chain).expanduser().resolve(),
        imu_yaml=Path(args.imu_yaml).expanduser().resolve(),
        h5_file=Path(args.h5_file).expanduser().resolve(),
        h5_timestamp_file=Path(args.h5_timestamp_file).expanduser().resolve(),
        imu_csv=Path(args.imu_csv).expanduser().resolve(),
        output_dir=output_dir,
        timeoffset_padding=args.timeoffset_padding,
        no_time_calibration=args.no_time_calibration,
        export_poses=args.export_poses,
        focal_length_init=args.focal_length_init,
        log_path=output_dir / "kalibr_cam_imu.log",
    )
    print(f"cam-imu log written to {result.log_path}")
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
