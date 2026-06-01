import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .common import UserMessage, read_yaml, sorted_natural, warn
from .i18n import tr


ZONE_NAMES_ZH = [
    "左上", "上中", "右上",
    "左中", "中心", "右中",
    "左下", "下中", "右下",
]
ZONE_NAMES_EN = [
    "top-left", "top-center", "top-right",
    "middle-left", "center", "middle-right",
    "bottom-left", "bottom-center", "bottom-right",
]


@dataclass
class CameraDiagnostics:
    camera: str
    image_count: int
    analyzed_count: int
    detected_count: int
    detection_ratio: float
    image_size: Tuple[int, int]
    mean_brightness: float
    mean_contrast: float
    mean_blur_laplacian: float
    corner_count: int
    coverage_bbox: Optional[Tuple[float, float, float, float]]
    coverage_area_ratio: float
    zone_counts: List[int]
    center_std: Tuple[float, float]
    scale_mean: float
    scale_std: float
    roll_std_deg: float
    messages: List[UserMessage]


@dataclass
class DatasetDiagnostics:
    target_type: str
    detector: str
    cameras: List[CameraDiagnostics]
    messages: List[UserMessage]


def _load_gray(path: Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Failed to read image: {path}")
    return image


def _image_files(cam_dir: Path) -> List[Path]:
    return sorted_natural([path for path in cam_dir.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".pnm"}])


def _sample(paths: Sequence[Path], max_images: int) -> List[Path]:
    if max_images <= 0 or len(paths) <= max_images:
        return list(paths)
    indices = np.linspace(0, len(paths) - 1, max_images).round().astype(int)
    return [paths[int(index)] for index in indices]


def _zone_index(point: np.ndarray, width: int, height: int) -> int:
    x = min(2, max(0, int(point[0] / max(width, 1) * 3.0)))
    y = min(2, max(0, int(point[1] / max(height, 1) * 3.0)))
    return y * 3 + x


def _pca_angle(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    centered = points - points.mean(axis=0, keepdims=True)
    cov = np.cov(centered.T)
    values, vectors = np.linalg.eig(cov)
    axis = vectors[:, int(np.argmax(values))]
    return math.atan2(float(axis[1]), float(axis[0]))


def _draw_overlay(image: np.ndarray, detections: Sequence[object], out_path: Path) -> None:
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    h, w = image.shape[:2]
    for x in (w // 3, 2 * w // 3):
        cv2.line(canvas, (x, 0), (x, h), (100, 100, 100), 1)
    for y in (h // 3, 2 * h // 3):
        cv2.line(canvas, (0, y), (w, y), (100, 100, 100), 1)
    for det in detections:
        corners = np.asarray(det.corners, dtype=np.int32)
        cv2.polylines(canvas, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        center = tuple(np.asarray(det.center, dtype=np.int32).tolist())
        cv2.circle(canvas, center, 3, (0, 0, 255), -1)
        cv2.putText(canvas, str(det.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), canvas)


def _make_detector():
    try:
        from dt_apriltags import Detector
    except Exception:
        return None
    return Detector(
        families="tag36h11",
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )


def _detect(detector: object, image: np.ndarray) -> Sequence[object]:
    return detector.detect(image, estimate_tag_pose=False)


def _summarize_camera(
    cam_name: str,
    image_paths: Sequence[Path],
    detector: Optional[object],
    debug_dir: Optional[Path],
    lang: str,
) -> CameraDiagnostics:
    messages: List[UserMessage] = []
    analyzed_paths = list(image_paths)
    if not analyzed_paths:
        return CameraDiagnostics(cam_name, 0, 0, 0, 0.0, (0, 0), 0.0, 0.0, 0.0, 0, None, 0.0, [0] * 9, (0.0, 0.0), 0.0, 0.0, 0.0, messages)

    first = _load_gray(analyzed_paths[0])
    height, width = first.shape[:2]
    brightness: List[float] = []
    contrast: List[float] = []
    blur: List[float] = []
    all_points: List[np.ndarray] = []
    centers: List[Tuple[float, float]] = []
    scales: List[float] = []
    angles: List[float] = []
    detected_count = 0
    zone_counts = [0] * 9

    for idx, path in enumerate(analyzed_paths):
        image = _load_gray(path)
        brightness.append(float(np.mean(image)))
        contrast.append(float(np.std(image)))
        blur.append(float(cv2.Laplacian(image, cv2.CV_64F).var()))
        detections = _detect(detector, image) if detector is not None else []
        if detections:
            detected_count += 1
            points = np.concatenate([np.asarray(det.corners, dtype=np.float32) for det in detections], axis=0)
            all_points.append(points)
            bbox_min = points.min(axis=0)
            bbox_max = points.max(axis=0)
            bbox_w = max(1.0, float(bbox_max[0] - bbox_min[0]))
            bbox_h = max(1.0, float(bbox_max[1] - bbox_min[1]))
            centers.append(((bbox_min[0] + bbox_max[0]) / (2.0 * width), (bbox_min[1] + bbox_max[1]) / (2.0 * height)))
            scales.append((bbox_w * bbox_h) / float(width * height))
            angles.append(_pca_angle(points))
            for point in points:
                zone_counts[_zone_index(point, width, height)] += 1
            if debug_dir is not None and idx < 80:
                _draw_overlay(image, detections, debug_dir / cam_name / path.name)

    point_array = np.concatenate(all_points, axis=0) if all_points else np.empty((0, 2), dtype=np.float32)
    corner_count = int(point_array.shape[0])
    detection_ratio = detected_count / float(len(analyzed_paths))
    coverage_bbox: Optional[Tuple[float, float, float, float]] = None
    coverage_area_ratio = 0.0
    if len(point_array):
        min_xy = point_array.min(axis=0)
        max_xy = point_array.max(axis=0)
        coverage_bbox = (
            float(min_xy[0] / width),
            float(min_xy[1] / height),
            float(max_xy[0] / width),
            float(max_xy[1] / height),
        )
        coverage_area_ratio = max(0.0, coverage_bbox[2] - coverage_bbox[0]) * max(0.0, coverage_bbox[3] - coverage_bbox[1])

    if detector is None:
        warn(messages, "apriltag_missing", tr(lang, "apriltag_missing"), tr(lang, "apriltag_missing_fix"))
    elif detection_ratio < 0.6:
        warn(messages, "low_detection", f"{cam_name}: {tr(lang, 'low_detection')} ({detected_count}/{len(analyzed_paths)})", tr(lang, "low_detection_fix"))

    if coverage_bbox is not None:
        span_x = coverage_bbox[2] - coverage_bbox[0]
        span_y = coverage_bbox[3] - coverage_bbox[1]
        if span_x < 0.65 or span_y < 0.65 or coverage_area_ratio < 0.35:
            warn(messages, "poor_coverage", f"{cam_name}: {tr(lang, 'poor_coverage')} bbox=({span_x:.2f}, {span_y:.2f})", tr(lang, "poor_coverage_fix"))

        total_points = max(1, sum(zone_counts))
        max_zone = max(zone_counts)
        empty_zones = [index for index, count in enumerate(zone_counts) if count == 0]
        if max_zone / float(total_points) > 0.45 or empty_zones:
            names = ZONE_NAMES_ZH if lang == "zh" else ZONE_NAMES_EN
            missing = ", ".join(names[index] for index in empty_zones[:5])
            detail = f"max_zone={max_zone / float(total_points):.2f}"
            if missing:
                detail += f", missing={missing}"
            warn(messages, "concentrated", f"{cam_name}: {tr(lang, 'concentrated')} ({detail})", tr(lang, "concentrated_fix"))

    centers_array = np.asarray(centers, dtype=np.float32) if centers else np.empty((0, 2), dtype=np.float32)
    center_std = tuple(np.std(centers_array, axis=0).tolist()) if len(centers_array) else (0.0, 0.0)
    scale_mean = float(np.mean(scales)) if scales else 0.0
    scale_std = float(np.std(scales)) if scales else 0.0
    roll_std_deg = float(np.std(np.unwrap(np.asarray(angles))) * 180.0 / math.pi) if angles else 0.0

    if len(centers_array) >= 5 and (center_std[0] < 0.12 or center_std[1] < 0.12):
        warn(messages, "weak_xy", f"{cam_name}: {tr(lang, 'weak_xy')} std=({center_std[0]:.2f}, {center_std[1]:.2f})", tr(lang, "weak_xy_fix"))
    if len(scales) >= 5 and scale_std < 0.04:
        warn(messages, "weak_z", f"{cam_name}: {tr(lang, 'weak_z')} scale_std={scale_std:.3f}", tr(lang, "weak_z_fix"))
    if len(angles) >= 5 and roll_std_deg < 12.0:
        warn(messages, "weak_roll", f"{cam_name}: {tr(lang, 'weak_roll')} roll_std={roll_std_deg:.1f}deg", tr(lang, "weak_roll_fix"))

    return CameraDiagnostics(
        camera=cam_name,
        image_count=len(image_paths),
        analyzed_count=len(analyzed_paths),
        detected_count=detected_count,
        detection_ratio=detection_ratio,
        image_size=(width, height),
        mean_brightness=float(np.mean(brightness)) if brightness else 0.0,
        mean_contrast=float(np.mean(contrast)) if contrast else 0.0,
        mean_blur_laplacian=float(np.mean(blur)) if blur else 0.0,
        corner_count=corner_count,
        coverage_bbox=coverage_bbox,
        coverage_area_ratio=coverage_area_ratio,
        zone_counts=zone_counts,
        center_std=(float(center_std[0]), float(center_std[1])),
        scale_mean=scale_mean,
        scale_std=scale_std,
        roll_std_deg=roll_std_deg,
        messages=messages,
    )


def analyze_dataset(
    dataset_root: Path,
    camera_names: Sequence[str],
    target_yaml: Path,
    debug_dir: Optional[Path],
    lang: str,
    max_images: int,
) -> DatasetDiagnostics:
    target = read_yaml(target_yaml)
    target_type = str(target.get("target_type") or target.get("targetType") or "unknown")
    detector = _make_detector() if target_type == "aprilgrid" else None
    detector_name = "dt_apriltags/tag36h11" if detector is not None else "none"
    messages: List[UserMessage] = []
    cameras: List[CameraDiagnostics] = []

    for cam_name in camera_names:
        cam_dir = dataset_root / "mav0" / cam_name
        image_paths = _sample(_image_files(cam_dir), max_images)
        cam_diag = _summarize_camera(cam_name, image_paths, detector, debug_dir, lang)
        cameras.append(cam_diag)
        messages.extend(cam_diag.messages)

    return DatasetDiagnostics(target_type=target_type, detector=detector_name, cameras=cameras, messages=messages)
