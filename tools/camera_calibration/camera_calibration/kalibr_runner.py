import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .common import CalibrationError, UserMessage, error, warn
from .i18n import tr


@dataclass
class CommandResult:
    command: List[str]
    cwd: Path
    log_path: Path
    returncode: int


@dataclass
class KalibrRunSummary:
    bag_result: Optional[CommandResult]
    calibration_result: Optional[CommandResult]
    messages: List[UserMessage]
    extracted_counts: Dict[str, Dict[str, int]]
    result_files: List[Path]
    calibration_attempts: List[CommandResult] = field(default_factory=list)
    fast_extraction: str = "auto"
    fallback_used: bool = False
    fallback_reason: str = ""


def find_vio_common_bagcreator() -> Path:
    candidates = []
    env_path = os.environ.get("VIO_COMMON_PYTHON")
    if env_path:
        candidates.append(Path(env_path))
    candidates.extend([
        Path("/opt/vio_common/python"),
        Path("/vio_common/python"),
        Path("/workspace/vio_common/python"),
    ])
    for path in candidates:
        for script_name in ("kalibr_bagcreater.py", "kalibr_bagcreator.py"):
            script = path / script_name
            if script.exists():
                return script
    raise CalibrationError(
        "Cannot find vio_common/python/kalibr_bagcreater.py or kalibr_bagcreator.py. "
        "Set VIO_COMMON_PYTHON or rebuild the Docker image."
    )


def run_logged(
    command: Sequence[str],
    cwd: Path,
    log_path: Path,
    env: Optional[Dict[str, str]] = None,
    stream: bool = False,
) -> CommandResult:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    full_env = os.environ.copy()
    if env:
        full_env.update(env)
    if not stream:
        print(f"Running {' '.join(command[:3])}; full log: {log_path}")
    with log_path.open("w", encoding="utf-8", errors="replace") as log:
        log.write("$ " + " ".join(command) + "\n")
        log.flush()
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=full_env,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            if stream:
                sys.stdout.write(line)
            log.write(line)
        returncode = process.wait()
    return CommandResult(command=list(command), cwd=cwd, log_path=log_path, returncode=returncode)


def _stage_input_file(src: Path, work_dir: Path) -> Path:
    if src is None:
        raise CalibrationError("Internal error: cannot stage an empty cam-imu input path.")
    if not src.exists():
        raise CalibrationError(f"Input file does not exist: {src}")
    work_dir.mkdir(parents=True, exist_ok=True)
    dst = work_dir / src.name
    if dst.exists() or dst.is_symlink():
        if dst.is_dir():
            raise CalibrationError(f"Cannot stage input over directory: {dst}")
        dst.unlink()
    try:
        dst.symlink_to(src)
    except OSError:
        shutil.copy2(str(src), str(dst))
    return dst


def create_bag(dataset_root: Path, output_bag: Path, log_path: Path, stream: bool = False) -> CommandResult:
    bagcreator = find_vio_common_bagcreator()
    command = [
        "python3",
        str(bagcreator),
        "--folder",
        str(dataset_root),
        "--output_bag",
        str(output_bag),
    ]
    return run_logged(command, cwd=bagcreator.parent, log_path=log_path, stream=stream)


def _expand_models(model_arg: str, count: int) -> List[str]:
    parts = [part.strip() for part in re.split(r"[, ]+", model_arg) if part.strip()]
    if not parts:
        raise CalibrationError("At least one camera model is required.")
    if len(parts) == 1:
        return parts * count
    if len(parts) != count:
        raise CalibrationError(f"Expected one model or {count} models, got {len(parts)}: {parts}")
    return parts


def run_cam_cam(
    dataset_root: Path,
    target_yaml: Path,
    camera_names: Sequence[str],
    output_dir: Path,
    model_arg: str,
    focal_length_init: Optional[float],
    show_report: bool,
    no_multithreading: bool,
    log_path: Path,
    stream: bool = False,
) -> CommandResult:
    bag_path = output_dir / "cam.bag"
    models = _expand_models(model_arg, len(camera_names))
    topics = [f"/{cam}/image_raw" for cam in camera_names]
    command = [
        "rosrun",
        "kalibr",
        "kalibr_calibrate_cameras",
        "--target",
        str(target_yaml),
        "--bag",
        str(bag_path),
        "--models",
    ] + models + ["--topics"] + topics
    if not show_report:
        command.append("--dont-show-report")
    if no_multithreading:
        command.append("--no-multithreading")

    env = {"MPLBACKEND": "Agg"}
    if focal_length_init is not None:
        env["KALIBR_MANUAL_FOCAL_LENGTH_INIT"] = str(focal_length_init)
    return run_logged(command, cwd=output_dir, log_path=log_path, env=env, stream=stream)


def _cam_cam_result_stems() -> Sequence[Tuple[str, str]]:
    return (
        ("cam-camchain.yaml", "cam-camchain-fast.yaml"),
        ("cam-results-cam.txt", "cam-results-cam-fast.txt"),
        ("cam-report-cam.pdf", "cam-report-cam-fast.pdf"),
    )


def _archive_fast_cam_cam_outputs(output_dir: Path) -> None:
    log_path = output_dir / "kalibr_cam_cam.log"
    archived_log = output_dir / "kalibr_cam_cam_fast.log"
    if log_path.exists():
        if archived_log.exists():
            archived_log.unlink()
        log_path.rename(archived_log)
    for src_name, dst_name in _cam_cam_result_stems():
        src = output_dir / src_name
        dst = output_dir / dst_name
        if src.exists():
            if dst.exists():
                dst.unlink()
            src.rename(dst)


def detect_cam_cam_fast_extraction_failure(log_path: Path) -> Tuple[bool, str]:
    if not log_path.exists():
        return False, ""
    text = log_path.read_text(encoding="utf-8", errors="replace")
    rosbag_error = (
        "ROSBagException" in text
        or "ROSBagFormatException" in text
        or "Error reading header:" in text
        or re.search(r"expecting\s+\d+\s+bytes,\s+read\s+\d+", text) is not None
    )
    multiprocessing_trace = "Process Process-" in text and "TargetExtractor.py" in text
    if rosbag_error or multiprocessing_trace:
        return True, "rosbag multiprocessing read failure"
    return False, ""


def run_cam_imu(
    target_yaml: Path,
    cam_chain: Path,
    imu_yaml: Path,
    h5_file: Optional[Path],
    h5_timestamp_file: Optional[Path],
    imu_csv: Optional[Path],
    corner_file: Optional[Path],
    image_timestamp_file: Optional[Path],
    imu_data_file: Optional[Path],
    fixture_id: str,
    trim_imu_edge_count: Optional[int],
    output_dir: Path,
    timeoffset_padding: float,
    max_iter: int,
    pose_knots_per_second: int,
    bias_knots_per_second: int,
    no_time_calibration: bool,
    export_poses: bool,
    focal_length_init: Optional[float],
    log_path: Path,
    stream: bool = False,
) -> CommandResult:
    work_dir = output_dir / "work_cam_imu"
    target_yaml = _stage_input_file(target_yaml, work_dir)
    cam_chain = _stage_input_file(cam_chain, work_dir)
    imu_yaml = _stage_input_file(imu_yaml, work_dir)
    if corner_file is not None:
        corner_file = _stage_input_file(corner_file, work_dir)
        image_timestamp_file = _stage_input_file(image_timestamp_file, work_dir)
        imu_data_file = _stage_input_file(imu_data_file, work_dir)
    else:
        h5_file = _stage_input_file(h5_file, work_dir)
        h5_timestamp_file = _stage_input_file(h5_timestamp_file, work_dir)
        imu_csv = _stage_input_file(imu_csv, work_dir)

    command = [
        "rosrun",
        "kalibr",
        "kalibr_calibrate_imu_camera",
        "--target",
        str(target_yaml),
        "--cams",
        str(cam_chain),
        "--imu",
        str(imu_yaml),
        "--timeoffset-padding",
        str(timeoffset_padding),
        "--max-iter",
        str(max_iter),
        "--pose-knots-per-second",
        str(pose_knots_per_second),
        "--bias-knots-per-second",
        str(bias_knots_per_second),
        "--dont-show-report",
    ]
    if corner_file is not None:
        command.extend([
            "--corner_file",
            str(corner_file),
            "--image_timestamp_file",
            str(image_timestamp_file),
            "--imu_data_file",
            str(imu_data_file),
            "--fixture_id",
            fixture_id,
        ])
        if trim_imu_edge_count is not None:
            command.extend(["--trim-imu-edge-count", str(trim_imu_edge_count)])
    else:
        command.extend([
            "--h5file",
            str(h5_file),
            "--h5timestampfile",
            str(h5_timestamp_file),
            "--imufile",
            str(imu_csv),
        ])
        if trim_imu_edge_count is not None:
            command.extend(["--trim-imu-edge-count", str(trim_imu_edge_count)])
    if no_time_calibration:
        command.append("--no-time-calibration")
    if export_poses:
        command.append("--export-poses")
    env = {"MPLBACKEND": "Agg"}
    if focal_length_init is not None:
        env["KALIBR_MANUAL_FOCAL_LENGTH_INIT"] = str(focal_length_init)
    return run_logged(command, cwd=work_dir, log_path=log_path, env=env, stream=stream)

def parse_kalibr_log(log_path: Path, lang: str) -> Tuple[Dict[str, Dict[str, int]], List[UserMessage]]:
    messages: List[UserMessage] = []
    extracted_counts: Dict[str, Dict[str, int]] = {}
    if not log_path.exists():
        return extracted_counts, messages
    text = log_path.read_text(encoding="utf-8", errors="replace")
    current_cam: Optional[str] = None
    for line in text.splitlines():
        match_cam = re.search(r"Initializing\s+(cam\d+):", line)
        if match_cam:
            current_cam = match_cam.group(1)
        match_extract = re.search(r"Extracted corners for\s+(\d+)\s+images\s+\(of\s+(\d+)\s+images\)", line)
        if match_extract and current_cam:
            extracted_counts[current_cam] = {
                "detected": int(match_extract.group(1)),
                "total": int(match_extract.group(2)),
            }
    if "cannot allocate memory in static TLS block" in text:
        warn(messages, "tls_error", tr(lang, "tls_error"), tr(lang, "tls_error_fix"))
    if "Optimization diverged" in text or "Did not converge" in text or "Max. attemps reached" in text:
        warn(messages, "kalibr_diverged", tr(lang, "kalibr_diverged"), tr(lang, "kalibr_diverged_fix"))
    if "initialization of focal length" in text.lower() and "failed" in text.lower():
        warn(messages, "focal_init_failed", tr(lang, "focal_init_failed"), tr(lang, "focal_init_failed_fix"))
    if "Tried to add second view to a given cameraId & timestamp" in text:
        warn(messages, "duplicate_timestamp", tr(lang, "duplicate_timestamp"), tr(lang, "duplicate_timestamp_fix"))
    if "System solution failed" in text or "matrix not positive definite" in text:
        warn(messages, "linear_solver_warning", tr(lang, "linear_solver_warning"), tr(lang, "linear_solver_warning_fix"))
    if re.search(r"(^|[^A-Za-z])[-+]?nan([^A-Za-z]|$)|(^|[^A-Za-z])[-+]?inf([^A-Za-z]|$)", text, flags=re.IGNORECASE):
        warn(messages, "nonfinite_result", tr(lang, "nonfinite_result"), tr(lang, "nonfinite_result_fix"))
    if "No corners could be extracted" in text or "Extracted corners for 0 images" in text:
        warn(messages, "no_corners", tr(lang, "no_corners"), tr(lang, "no_corners_fix"))
    if "time ranges" in text and "do not overlap" in text:
        warn(messages, "imu_no_overlap", tr(lang, "imu_no_overlap"), tr(lang, "imu_no_overlap_fix"))
    if "Could not find any IMU messages" in text:
        warn(messages, "imu_no_messages", tr(lang, "imu_no_messages"), tr(lang, "imu_no_messages_fix"))
    if "ImportError" in text or "ModuleNotFoundError" in text or "No module named" in text:
        warn(messages, "python_import_error", tr(lang, "python_import_error"), tr(lang, "python_import_error_fix"))
    return extracted_counts, messages


def collect_result_files(output_dir: Path) -> List[Path]:
    patterns = [
        "*.yaml",
        "*.txt",
        "*.pdf",
        "*.csv",
        "*.log",
        "*.bag",
    ]
    files: List[Path] = []
    for pattern in patterns:
        files.extend(path for path in output_dir.rglob(pattern) if not path.is_symlink())
    dataset_bag = output_dir / "cam.bag"
    if dataset_bag.exists() and dataset_bag not in files:
        files.append(dataset_bag)
    return sorted(files)


def run_cam_cam_pipeline(
    dataset_root: Path,
    target_yaml: Path,
    camera_names: Sequence[str],
    output_dir: Path,
    model_arg: str,
    focal_length_init: Optional[float],
    show_report: bool,
    skip_kalibr: bool,
    lang: str,
    fast_extraction: str = "auto",
    stream: bool = False,
) -> KalibrRunSummary:
    messages: List[UserMessage] = []
    bag_result: Optional[CommandResult] = None
    calibration_result: Optional[CommandResult] = None
    calibration_attempts: List[CommandResult] = []
    extracted_counts: Dict[str, Dict[str, int]] = {}
    fallback_used = False
    fallback_reason = ""

    bag_path = output_dir / "cam.bag"
    if not skip_kalibr:
        bag_result = create_bag(dataset_root, bag_path, output_dir / "bag_creator.log", stream=stream)
        if bag_result.returncode != 0:
            raise CalibrationError(f"Bag creation failed, see {bag_result.log_path}")
        no_multithreading = fast_extraction == "never"
        calibration_result = run_cam_cam(
            dataset_root=dataset_root,
            target_yaml=target_yaml,
            camera_names=camera_names,
            output_dir=output_dir,
            model_arg=model_arg,
            focal_length_init=focal_length_init,
            show_report=show_report,
            no_multithreading=no_multithreading,
            log_path=output_dir / "kalibr_cam_cam.log",
            stream=stream,
        )
        calibration_attempts.append(calibration_result)
        extracted_counts, log_messages = parse_kalibr_log(calibration_result.log_path, lang)
        messages.extend(log_messages)
        fast_failed, fast_reason = detect_cam_cam_fast_extraction_failure(calibration_result.log_path)
        if fast_failed and fast_extraction == "auto":
            fallback_used = True
            fallback_reason = fast_reason
            messages = [message for message in messages if message not in log_messages]
            warn(messages, "fast_extraction_fallback", tr(lang, "fast_extraction_fallback"), tr(lang, "fast_extraction_fallback_fix"))
            _archive_fast_cam_cam_outputs(output_dir)
            calibration_attempts[-1].log_path = output_dir / "kalibr_cam_cam_fast.log"
            calibration_result = run_cam_cam(
                dataset_root=dataset_root,
                target_yaml=target_yaml,
                camera_names=camera_names,
                output_dir=output_dir,
                model_arg=model_arg,
                focal_length_init=focal_length_init,
                show_report=show_report,
                no_multithreading=True,
                log_path=output_dir / "kalibr_cam_cam.log",
                stream=stream,
            )
            calibration_attempts.append(calibration_result)
            extracted_counts, log_messages = parse_kalibr_log(calibration_result.log_path, lang)
            messages.extend(log_messages)
        elif fast_failed and fast_extraction == "always":
            fallback_reason = fast_reason
            error(messages, "fast_extraction_failed", tr(lang, "fast_extraction_failed"), tr(lang, "fast_extraction_failed_fix"))
    return KalibrRunSummary(
        bag_result=bag_result,
        calibration_result=calibration_result,
        messages=messages,
        extracted_counts=extracted_counts,
        result_files=collect_result_files(output_dir),
        calibration_attempts=calibration_attempts,
        fast_extraction=fast_extraction,
        fallback_used=fallback_used,
        fallback_reason=fallback_reason,
    )
