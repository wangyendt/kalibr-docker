import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .common import UserMessage, error, read_yaml, warn
from .i18n import tr


@dataclass
class QualityMetric:
    name: str
    value: float
    unit: str
    status: str
    detail: str = ""


@dataclass
class CameraFov:
    camera: str
    width: int
    height: int
    hfov_deg: float
    vfov_deg: float
    dfov_deg: float
    source: str
    detail: str = ""


@dataclass
class CalibrationQuality:
    mode: str
    summary: str
    metrics: List[QualityMetric]
    messages: List[UserMessage]
    parsed_files: List[Path]
    fovs: List[CameraFov] = field(default_factory=list)


def _read_first(output_dir: Path, patterns: Sequence[str]) -> Tuple[Optional[Path], str]:
    for pattern in patterns:
        matches = sorted(path for path in output_dir.rglob(pattern) if not path.is_symlink())
        if matches:
            path = matches[0]
            return path, path.read_text(encoding="utf-8", errors="replace")
    return None, ""


def _floats(value: str) -> List[float]:
    return [float(item) for item in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value)]


def _status(value: float, warn_threshold: float, error_threshold: float, higher_is_worse: bool = True) -> str:
    if higher_is_worse:
        if value >= error_threshold:
            return "error"
        if value >= warn_threshold:
            return "warning"
        return "ok"
    if value <= error_threshold:
        return "error"
    if value <= warn_threshold:
        return "warning"
    return "ok"


def _add_metric(metrics: List[QualityMetric], messages: List[UserMessage], name: str, value: float, unit: str,
                status: str, warn_code: str, warn_text: str, warn_fix: str, detail: str = "") -> None:
    metrics.append(QualityMetric(name=name, value=value, unit=unit, status=status, detail=detail))
    if status == "warning":
        warn(messages, warn_code, warn_text, warn_fix)
    elif status == "error":
        error(messages, warn_code, warn_text, warn_fix)


def _angle_between_rays_deg(left: Tuple[float, float, float], right: Tuple[float, float, float]) -> float:
    dot = sum(a * b for a, b in zip(left, right))
    left_norm = math.sqrt(sum(a * a for a in left))
    right_norm = math.sqrt(sum(a * a for a in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        return float("nan")
    cos_angle = dot / (left_norm * right_norm)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def _pinhole_fov(
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
) -> Optional[Tuple[float, float, float]]:
    values = [fx, fy, cx, cy, float(width), float(height)]
    if any(not math.isfinite(value) for value in values):
        return None
    if fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0:
        return None

    left = ((0.0 - cx) / fx, 0.0, 1.0)
    right = ((float(width) - cx) / fx, 0.0, 1.0)
    top = (0.0, (0.0 - cy) / fy, 1.0)
    bottom = (0.0, (float(height) - cy) / fy, 1.0)

    def corner(u: float, v: float) -> Tuple[float, float, float]:
        return ((u - cx) / fx, (v - cy) / fy, 1.0)

    hfov = _angle_between_rays_deg(left, right)
    vfov = _angle_between_rays_deg(top, bottom)
    dfov = max(
        _angle_between_rays_deg(corner(0.0, 0.0), corner(float(width), float(height))),
        _angle_between_rays_deg(corner(float(width), 0.0), corner(0.0, float(height))),
    )
    if any(not (math.isfinite(value) and 0.0 < value < 180.0) for value in (hfov, vfov, dfov)):
        return None
    return hfov, vfov, dfov


def _camera_resolution(cam: Dict[str, Any], fallback_size: Optional[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
    resolution = cam.get("resolution")
    if isinstance(resolution, (list, tuple)) and len(resolution) >= 2:
        try:
            width = int(round(float(resolution[0])))
            height = int(round(float(resolution[1])))
        except (TypeError, ValueError):
            width, height = 0, 0
        if width > 0 and height > 0:
            return width, height
    if fallback_size:
        width, height = fallback_size
        if width > 0 and height > 0:
            return width, height
    return None


def _fovs_from_camchain_data(
    data: Dict[str, Any],
    fallback_size: Optional[Tuple[int, int]],
    source: str,
) -> List[CameraFov]:
    fovs: List[CameraFov] = []
    for cam_name, cam in sorted(data.items()):
        if not isinstance(cam, dict):
            continue
        intrinsics = cam.get("intrinsics")
        if not isinstance(intrinsics, (list, tuple)) or len(intrinsics) < 4:
            continue
        camera_model = str(cam.get("camera_model", "")).strip().lower()
        if camera_model and camera_model != "pinhole":
            continue
        resolution = _camera_resolution(cam, fallback_size)
        if resolution is None:
            continue
        try:
            fx, fy, cx, cy = [float(value) for value in intrinsics[:4]]
        except (TypeError, ValueError):
            continue
        fov = _pinhole_fov(fx, fy, cx, cy, resolution[0], resolution[1])
        if fov is None:
            continue
        fovs.append(CameraFov(
            camera=str(cam_name),
            width=resolution[0],
            height=resolution[1],
            hfov_deg=fov[0],
            vfov_deg=fov[1],
            dfov_deg=fov[2],
            source=source,
            detail="pinhole intrinsics [fx, fy, cx, cy]",
        ))
    return fovs


def _fovs_from_result_blocks(
    blocks: Sequence[Tuple[str, str]],
    image_size: Optional[Tuple[int, int]],
    source: str,
) -> List[CameraFov]:
    if not image_size:
        return []
    width, height = image_size
    if width <= 0 or height <= 0:
        return []
    fovs: List[CameraFov] = []
    for cam_name, block in blocks:
        model_match = re.match(rf"{re.escape(cam_name)}\s+\(([^)]+)\):", block)
        if model_match and not model_match.group(1).strip().lower().startswith("pinhole"):
            continue
        proj = re.search(r"projection:\s*\[([^\]]+)\]", block)
        if not proj:
            continue
        vals = _floats(proj.group(1))[:4]
        if len(vals) != 4:
            continue
        fov = _pinhole_fov(vals[0], vals[1], vals[2], vals[3], width, height)
        if fov is None:
            continue
        fovs.append(CameraFov(
            camera=cam_name,
            width=width,
            height=height,
            hfov_deg=fov[0],
            vfov_deg=fov[1],
            dfov_deg=fov[2],
            source=source,
            detail="Kalibr results projection [fx, fy, cx, cy]",
        ))
    return fovs


def _cam_cam_fovs(
    output_dir: Path,
    blocks: Sequence[Tuple[str, str]],
    target_size: Optional[Tuple[int, int]],
    result_source: str,
    parsed_files: List[Path],
) -> List[CameraFov]:
    fovs: List[CameraFov] = []
    camchain_path, _ = _read_first(output_dir, ["*camchain.yaml"])
    if camchain_path is not None:
        try:
            camchain_data = read_yaml(camchain_path)
        except Exception:
            camchain_data = None
        if isinstance(camchain_data, dict):
            fovs.extend(_fovs_from_camchain_data(camchain_data, target_size, camchain_path.name))
            if camchain_path not in parsed_files:
                parsed_files.append(camchain_path)

    existing_cameras = {fov.camera for fov in fovs}
    for fov in _fovs_from_result_blocks(blocks, target_size, result_source):
        if fov.camera not in existing_cameras:
            fovs.append(fov)
            existing_cameras.add(fov.camera)
    return fovs


def _camera_blocks(text: str) -> List[Tuple[str, str]]:
    blocks: List[Tuple[str, str]] = []
    matches = list(re.finditer(r"^(cam\d+)\s+\([^\n]+\):\n", text, flags=re.MULTILINE))
    for idx, match in enumerate(matches):
        start = match.start()
        end = matches[idx + 1].start() if idx + 1 < len(matches) else len(text)
        target_idx = text.find("\n\n\nTarget configuration", start, end)
        if target_idx != -1:
            end = target_idx
        blocks.append((match.group(1), text[start:end]))
    return blocks


def evaluate_cam_cam(output_dir: Path, target_size: Optional[Tuple[int, int]], lang: str) -> CalibrationQuality:
    messages: List[UserMessage] = []
    metrics: List[QualityMetric] = []
    parsed_files: List[Path] = []
    result_path, text = _read_first(output_dir, ["*results-cam.txt", "*results*.txt"])
    if result_path is None:
        error(messages, "missing_result", tr(lang, "missing_result"), tr(lang, "missing_result_fix"))
        return CalibrationQuality("cam-cam", tr(lang, "quality_unknown"), metrics, messages, parsed_files)
    parsed_files.append(result_path)
    if re.search(r"(^|[^A-Za-z])[-+]?nan([^A-Za-z]|$)|(^|[^A-Za-z])[-+]?inf([^A-Za-z]|$)", text, flags=re.IGNORECASE):
        error(messages, "nonfinite_result", tr(lang, "nonfinite_result"), tr(lang, "nonfinite_result_fix"))
        return CalibrationQuality("cam-cam", tr(lang, "quality_error"), metrics, messages, parsed_files)

    blocks = _camera_blocks(text)
    if not blocks:
        error(messages, "parse_result_failed", tr(lang, "parse_result_failed"), tr(lang, "parse_result_failed_fix"))
        return CalibrationQuality("cam-cam", tr(lang, "quality_unknown"), metrics, messages, parsed_files)

    width, height = target_size or (0, 0)
    worst_status = "ok"
    for cam_name, block in blocks:
        reproj = re.search(r"reprojection error:\s*\[([^\]]+)\]\s*\+-\s*\[([^\]]+)\]", block)
        if reproj:
            std_vals = _floats(reproj.group(2))[:2]
            if len(std_vals) == 2:
                rms_std = math.sqrt(std_vals[0] ** 2 + std_vals[1] ** 2)
                status = _status(rms_std, 0.6, 1.2)
                worst_status = _merge_status(worst_status, status)
                _add_metric(
                    metrics, messages, f"{cam_name}.reprojection_rms_std", rms_std, "px", status,
                    "high_reprojection", f"{cam_name}: {tr(lang, 'high_reprojection')} {rms_std:.3f}px",
                    tr(lang, "high_reprojection_fix"),
                )

        proj = re.search(r"projection:\s*\[([^\]]+)\]", block)
        if proj:
            vals = _floats(proj.group(1))[:4]
            if len(vals) == 4:
                fx, fy, cx, cy = vals
                aspect_diff = abs(fx - fy) / max(abs(fx), abs(fy), 1.0)
                status = _status(aspect_diff, 0.05, 0.15)
                worst_status = _merge_status(worst_status, status)
                _add_metric(
                    metrics, messages, f"{cam_name}.focal_asymmetry", aspect_diff, "ratio", status,
                    "focal_asymmetry", f"{cam_name}: {tr(lang, 'focal_asymmetry')} {aspect_diff:.2%}",
                    tr(lang, "focal_asymmetry_fix"),
                )
                if width and height:
                    cx_norm = cx / float(width)
                    cy_norm = cy / float(height)
                    offset = max(abs(cx_norm - 0.5), abs(cy_norm - 0.5))
                    status = _status(offset, 0.25, 0.40)
                    worst_status = _merge_status(worst_status, status)
                    _add_metric(
                        metrics, messages, f"{cam_name}.principal_point_offset", offset, "ratio", status,
                        "principal_point_far", f"{cam_name}: {tr(lang, 'principal_point_far')} ({cx_norm:.2f}, {cy_norm:.2f})",
                        tr(lang, "principal_point_far_fix"),
                    )

        dist = re.search(r"distortion:\s*\[([^\]]+)\]", block)
        if dist:
            vals = _floats(dist.group(1))
            if vals:
                max_abs = max(abs(v) for v in vals)
                status = _status(max_abs, 1.0, 3.0)
                worst_status = _merge_status(worst_status, status)
                _add_metric(
                    metrics, messages, f"{cam_name}.max_abs_distortion", max_abs, "coeff", status,
                    "large_distortion", f"{cam_name}: {tr(lang, 'large_distortion')} max={max_abs:.3f}",
                    tr(lang, "large_distortion_fix"),
                )

    fovs = _cam_cam_fovs(output_dir, blocks, target_size, result_path.name, parsed_files)
    summary = _summary(lang, worst_status)
    return CalibrationQuality("cam-cam", summary, metrics, messages, parsed_files, fovs=fovs)


def evaluate_cam_imu(output_dir: Path, lang: str) -> CalibrationQuality:
    messages: List[UserMessage] = []
    metrics: List[QualityMetric] = []
    parsed_files: List[Path] = []
    result_path, text = _read_first(output_dir, ["*results-imucam.txt", "*results*.txt"])
    if result_path is None:
        error(messages, "missing_result", tr(lang, "missing_result"), tr(lang, "missing_result_fix"))
        return CalibrationQuality("cam-imu", tr(lang, "quality_unknown"), metrics, messages, parsed_files)
    parsed_files.append(result_path)
    if re.search(r"(^|[^A-Za-z])[-+]?nan([^A-Za-z]|$)|(^|[^A-Za-z])[-+]?inf([^A-Za-z]|$)", text, flags=re.IGNORECASE):
        error(messages, "nonfinite_result", tr(lang, "nonfinite_result"), tr(lang, "nonfinite_result_fix"))
        return CalibrationQuality("cam-imu", tr(lang, "quality_error"), metrics, messages, parsed_files)

    worst_status = "ok"
    patterns = [
        ("reprojection_mean_px", r"Reprojection error \(cam\d+\) \[px\]:\s+mean\s+([^,\n]+)", "px", 1.0, 2.0,
         "high_reprojection", "high_reprojection_fix"),
        ("gyro_mean", r"Gyroscope error \(imu\d+\) \[rad/s\]:\s+mean\s+([^,\n]+)", "rad/s", 0.25, 0.6,
         "high_gyro_residual", "high_gyro_residual_fix"),
        ("accel_mean", r"Accelerometer error \(imu\d+\) \[m/s\^2\]:\s+mean\s+([^,\n]+)", "m/s^2", 0.8, 2.0,
         "high_accel_residual", "high_accel_residual_fix"),
        ("normalized_reprojection_mean", r"Reprojection error \(cam\d+\):\s+mean\s+([^,\n]+)", "sigma", 1.0, 2.0,
         "high_normalized_reprojection", "high_normalized_reprojection_fix"),
    ]
    for name, pattern, unit, warn_threshold, error_threshold, code, fix_code in patterns:
        match = re.search(pattern, text)
        if not match:
            continue
        value = float(match.group(1).strip())
        status = _status(value, warn_threshold, error_threshold)
        worst_status = _merge_status(worst_status, status)
        _add_metric(
            metrics, messages, name, value, unit, status,
            code, f"{tr(lang, code)} {value:.3f}{unit}", tr(lang, fix_code),
        )

    ts_match = re.search(r"timeshift\s+cam\d+\s+to\s+imu\d+:[^\n]*\n\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", text)
    if ts_match:
        value = abs(float(ts_match.group(1)))
        status = _status(value, 0.03, 0.10)
        worst_status = _merge_status(worst_status, status)
        _add_metric(
            metrics, messages, "abs_time_shift", value, "s", status,
            "large_time_shift", f"{tr(lang, 'large_time_shift')} {value:.3f}s", tr(lang, "large_time_shift_fix"),
        )

    transform = _parse_transform(text, "T_ci")
    if transform:
        tx, ty, tz = transform[0][3], transform[1][3], transform[2][3]
        norm = math.sqrt(tx * tx + ty * ty + tz * tz)
        status = _status(norm, 0.5, 1.0)
        worst_status = _merge_status(worst_status, status)
        _add_metric(
            metrics, messages, "imu_to_cam_translation_norm", norm, "m", status,
            "large_extrinsic_translation", f"{tr(lang, 'large_extrinsic_translation')} {norm:.3f}m",
            tr(lang, "large_extrinsic_translation_fix"),
        )

    summary = _summary(lang, worst_status)
    return CalibrationQuality("cam-imu", summary, metrics, messages, parsed_files)


def evaluate_camchain_intrinsics(camchain_yaml: Path, image_size: Optional[Tuple[int, int]], lang: str) -> CalibrationQuality:
    messages: List[UserMessage] = []
    metrics: List[QualityMetric] = []
    parsed_files: List[Path] = []
    if not camchain_yaml.exists():
        error(messages, "missing_camchain", tr(lang, "missing_camchain"), tr(lang, "missing_camchain_fix"))
        return CalibrationQuality("cam-chain", tr(lang, "quality_unknown"), metrics, messages, parsed_files)
    parsed_files.append(camchain_yaml)
    try:
        data = read_yaml(camchain_yaml)
    except Exception:
        error(messages, "parse_result_failed", tr(lang, "parse_result_failed"), tr(lang, "parse_result_failed_fix"))
        return CalibrationQuality("cam-chain", tr(lang, "quality_unknown"), metrics, messages, parsed_files)

    width, height = image_size or (0, 0)
    worst_status = "ok"
    fovs = _fovs_from_camchain_data(data, image_size, camchain_yaml.name)
    for cam_name, cam in sorted(data.items()):
        intrinsics = cam.get("intrinsics") if isinstance(cam, dict) else None
        if not isinstance(intrinsics, list) or len(intrinsics) < 4:
            continue
        fx, fy, cx, cy = [float(v) for v in intrinsics[:4]]
        aspect_diff = abs(fx - fy) / max(abs(fx), abs(fy), 1.0)
        status = _status(aspect_diff, 0.05, 0.15)
        worst_status = _merge_status(worst_status, status)
        _add_metric(metrics, messages, f"{cam_name}.focal_asymmetry", aspect_diff, "ratio", status,
                    "focal_asymmetry", f"{cam_name}: {tr(lang, 'focal_asymmetry')} {aspect_diff:.2%}", tr(lang, "focal_asymmetry_fix"))
        if width and height:
            offset = max(abs(cx / float(width) - 0.5), abs(cy / float(height) - 0.5))
            status = _status(offset, 0.25, 0.40)
            worst_status = _merge_status(worst_status, status)
            _add_metric(metrics, messages, f"{cam_name}.principal_point_offset", offset, "ratio", status,
                        "principal_point_far", f"{cam_name}: {tr(lang, 'principal_point_far')} ({cx/float(width):.2f}, {cy/float(height):.2f})", tr(lang, "principal_point_far_fix"))
    return CalibrationQuality("cam-chain", _summary(lang, worst_status), metrics, messages, parsed_files, fovs=fovs)


def _parse_transform(text: str, label: str) -> Optional[List[List[float]]]:
    match = re.search(label + r":.*?\n(\[\[.*?\]\])", text, flags=re.DOTALL)
    if not match:
        return None
    values = _floats(match.group(1))
    if len(values) < 16:
        return None
    return [values[i:i + 4] for i in range(0, 16, 4)]


def _merge_status(left: str, right: str) -> str:
    order = {"ok": 0, "warning": 1, "error": 2}
    return left if order[left] >= order[right] else right


def _summary(lang: str, status: str) -> str:
    if status == "error":
        return tr(lang, "quality_error")
    if status == "warning":
        return tr(lang, "quality_warning")
    return tr(lang, "quality_ok")
