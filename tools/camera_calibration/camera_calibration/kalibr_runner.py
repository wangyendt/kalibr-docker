import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .common import CalibrationError, UserMessage, warn
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


def find_vio_common_python() -> Path:
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
        if (path / "kalibr_bagcreater.py").exists():
            return path
    raise CalibrationError(
        "Cannot find vio_common/python/kalibr_bagcreater.py. "
        "Set VIO_COMMON_PYTHON or rebuild the Docker image."
    )


def run_logged(command: Sequence[str], cwd: Path, log_path: Path, env: Optional[Dict[str, str]] = None) -> CommandResult:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    full_env = os.environ.copy()
    if env:
        full_env.update(env)
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
            sys.stdout.write(line)
            log.write(line)
        returncode = process.wait()
    return CommandResult(command=list(command), cwd=cwd, log_path=log_path, returncode=returncode)


def create_bag(dataset_root: Path, output_bag: Path, log_path: Path) -> CommandResult:
    vio_common = find_vio_common_python()
    command = [
        "python3",
        str(vio_common / "kalibr_bagcreater.py"),
        "--folder",
        str(dataset_root),
        "--output_bag",
        str(output_bag),
    ]
    return run_logged(command, cwd=vio_common, log_path=log_path)


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
    log_path: Path,
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

    env = {"MPLBACKEND": "Agg"}
    if focal_length_init is not None:
        env["KALIBR_MANUAL_FOCAL_LENGTH_INIT"] = str(focal_length_init)
    return run_logged(command, cwd=output_dir, log_path=log_path, env=env)


def run_cam_imu(
    target_yaml: Path,
    cam_chain: Path,
    imu_yaml: Path,
    h5_file: Path,
    h5_timestamp_file: Path,
    imu_csv: Path,
    output_dir: Path,
    timeoffset_padding: float,
    no_time_calibration: bool,
    export_poses: bool,
    focal_length_init: Optional[float],
    log_path: Path,
) -> CommandResult:
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
        "--h5file",
        str(h5_file),
        "--h5timestampfile",
        str(h5_timestamp_file),
        "--imufile",
        str(imu_csv),
        "--timeoffset-padding",
        str(timeoffset_padding),
    ]
    if no_time_calibration:
        command.append("--no-time-calibration")
    if export_poses:
        command.append("--export-poses")
    env = {"MPLBACKEND": "Agg"}
    if focal_length_init is not None:
        env["KALIBR_MANUAL_FOCAL_LENGTH_INIT"] = str(focal_length_init)
    return run_logged(command, cwd=output_dir, log_path=log_path, env=env)


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
        files.extend(output_dir.glob(pattern))
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
) -> KalibrRunSummary:
    messages: List[UserMessage] = []
    bag_result: Optional[CommandResult] = None
    calibration_result: Optional[CommandResult] = None
    extracted_counts: Dict[str, Dict[str, int]] = {}

    bag_path = output_dir / "cam.bag"
    if not skip_kalibr:
        bag_result = create_bag(dataset_root, bag_path, output_dir / "bag_creator.log")
        if bag_result.returncode != 0:
            raise CalibrationError(f"Bag creation failed, see {bag_result.log_path}")
        calibration_result = run_cam_cam(
            dataset_root=dataset_root,
            target_yaml=target_yaml,
            camera_names=camera_names,
            output_dir=output_dir,
            model_arg=model_arg,
            focal_length_init=focal_length_init,
            show_report=show_report,
            log_path=output_dir / "kalibr_cam_cam.log",
        )
        extracted_counts, log_messages = parse_kalibr_log(calibration_result.log_path, lang)
        messages.extend(log_messages)
    return KalibrRunSummary(
        bag_result=bag_result,
        calibration_result=calibration_result,
        messages=messages,
        extracted_counts=extracted_counts,
        result_files=collect_result_files(output_dir),
    )
