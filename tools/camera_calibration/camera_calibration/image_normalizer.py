import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from statistics import median_low
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .common import CalibrationError, UserMessage, ensure_dir, sorted_natural, warn
from .i18n import tr
from .input_resolver import CameraInput, InputLayout


@dataclass
class PreparedDataset:
    root: Path
    mav0_root: Path
    camera_names: List[str]
    image_counts: Dict[str, int]
    target_size: Tuple[int, int]
    normalized: bool
    preprocess: str
    messages: List[UserMessage]
    source_summary: Dict[str, object]


def read_grayscale(path: Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise CalibrationError(f"Failed to read image: {path}")
    return image


def write_png(path: Path, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise CalibrationError(f"Failed to encode image: {path}")
    encoded.tofile(str(path))


def parse_video(video_path: Path, dst_dir: Path, sample_fps: float, max_frames: int = 0) -> List[Path]:
    ensure_dir(dst_dir)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise CalibrationError(f"Failed to open video: {video_path}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if source_fps <= 0:
        source_fps = sample_fps if sample_fps > 0 else 30.0
    step = 1
    if sample_fps > 0:
        step = max(1, int(round(source_fps / sample_fps)))

    output_paths: List[Path] = []
    index = 0
    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if index % step == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            out_path = dst_dir / f"frame_{saved:06d}.png"
            write_png(out_path, gray)
            output_paths.append(out_path)
            saved += 1
            if max_frames and saved >= max_frames:
                break
        index += 1
    cap.release()
    if not output_paths:
        raise CalibrationError(f"No frames extracted from video: {video_path} ({frame_count} frames reported)")
    return output_paths


def image_size(path: Path) -> Tuple[int, int]:
    image = read_grayscale(path)
    height, width = image.shape[:2]
    return width, height


def _aspect_key(size: Tuple[int, int]) -> float:
    width, height = size
    return round(float(width) / float(height), 3)


def choose_auto_target_size(sizes: Sequence[Tuple[int, int]]) -> Tuple[int, int]:
    if not sizes:
        raise CalibrationError("No images available for target-size selection.")
    unique_sizes = sorted(set(sizes), key=lambda item: (item[0] * item[1], item[0], item[1]))
    aspect_values = sorted(_aspect_key(size) for size in sizes)
    target_aspect = median_low(aspect_values)
    same_aspect = [size for size in unique_sizes if _aspect_key(size) == target_aspect]
    if same_aspect:
        width, height = same_aspect[0]
    else:
        width, height = min(unique_sizes, key=lambda size: (abs(_aspect_key(size) - target_aspect), size[0] * size[1]))
    width = max(2, int(width) // 2 * 2)
    height = max(2, int(height) // 2 * 2)
    return width, height


def center_crop_to_aspect(image: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
    target_width, target_height = target_size
    target_aspect = target_width / float(target_height)
    height, width = image.shape[:2]
    source_aspect = width / float(height)
    if math.isclose(source_aspect, target_aspect, rel_tol=1e-3, abs_tol=1e-3):
        return image
    if source_aspect > target_aspect:
        crop_width = int(round(height * target_aspect))
        x0 = max(0, (width - crop_width) // 2)
        return image[:, x0:x0 + crop_width]
    crop_height = int(round(width / target_aspect))
    y0 = max(0, (height - crop_height) // 2)
    return image[y0:y0 + crop_height, :]


def apply_preprocess(image: np.ndarray, mode: str) -> np.ndarray:
    if mode == "none":
        return image
    if mode == "hist-eq":
        return cv2.equalizeHist(image)
    if mode == "clahe":
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(image)
    raise CalibrationError(f"Unsupported preprocess mode: {mode}")


def normalize_image(image: np.ndarray, target_size: Tuple[int, int], preprocess: str) -> np.ndarray:
    cropped = center_crop_to_aspect(image, target_size)
    if (cropped.shape[1], cropped.shape[0]) != target_size:
        cropped = cv2.resize(cropped, target_size, interpolation=cv2.INTER_AREA)
    normalized = apply_preprocess(cropped, preprocess)
    return normalized


def timestamp_ns(index: int, fps: float) -> int:
    if fps <= 0:
        raise CalibrationError("timestamp fps must be positive")
    step = int(round(1_000_000_000 / fps))
    elapsed_nsec = index * step
    # vio_common's bagcreator parses names as:
    # secs = text[0:-10], nsecs = text[-9:].
    # The extra digit before the final 9 nsec digits must stay 0, otherwise
    # every whole-second boundary aliases to an earlier timestamp.
    secs = 1 + elapsed_nsec // 1_000_000_000
    nsecs = elapsed_nsec % 1_000_000_000
    return int(f"{secs}{nsecs:010d}")


def _materialize_camera_sources(
    layout: InputLayout,
    work_root: Path,
    video_sample_fps: float,
    max_video_frames: int,
) -> List[CameraInput]:
    materialized: List[CameraInput] = []
    for idx, camera in enumerate(layout.cameras):
        if camera.kind == "video":
            assert camera.video_path is not None
            frames_dir = work_root / "_video_frames" / f"cam{idx}"
            frames = parse_video(camera.video_path, frames_dir, video_sample_fps, max_video_frames)
            materialized.append(CameraInput(name=f"cam{idx}", kind="images", image_paths=frames))
        else:
            materialized.append(CameraInput(name=f"cam{idx}", kind="images", image_paths=sorted_natural(camera.image_paths)))
    return materialized


def prepare_dataset(
    layout: InputLayout,
    output_root: Path,
    resize: Optional[Tuple[int, int]],
    preprocess: str,
    timestamp_fps: float,
    video_sample_fps: float,
    lang: str,
    max_video_frames: int = 0,
) -> PreparedDataset:
    messages: List[UserMessage] = list(layout.messages)
    dataset_root = output_root / "dataset"
    if dataset_root.exists():
        shutil.rmtree(dataset_root)
    video_root = output_root / "_video_frames"
    if video_root.exists():
        shutil.rmtree(video_root)
    dataset_root = ensure_dir(dataset_root)
    mav0_root = ensure_dir(dataset_root / "mav0")
    cameras = _materialize_camera_sources(layout, output_root, video_sample_fps, max_video_frames)

    all_sizes: List[Tuple[int, int]] = []
    per_camera_sizes: Dict[str, List[Tuple[int, int]]] = {}
    for camera in cameras:
        sizes = [image_size(path) for path in camera.image_paths]
        if not sizes:
            raise CalibrationError(f"No images for {camera.name}")
        per_camera_sizes[camera.name] = sizes
        all_sizes.extend(sizes)

    unique_sizes = sorted(set(all_sizes))
    if resize is not None:
        target_size = resize
        normalized = True
    else:
        target_size = choose_auto_target_size(all_sizes)
        normalized = len(unique_sizes) > 1
        if normalized:
            warn(messages, "resolution_mismatch", tr(lang, "resolution_mismatch"), tr(lang, "resolution_mismatch_fix"))

    normalized = normalized or preprocess != "none"
    image_counts: Dict[str, int] = {}

    for cam_index, camera in enumerate(cameras):
        cam_name = f"cam{cam_index}"
        cam_dir = ensure_dir(mav0_root / cam_name)
        image_counts[cam_name] = len(camera.image_paths)
        for frame_index, src_path in enumerate(camera.image_paths):
            image = read_grayscale(src_path)
            out_image = normalize_image(image, target_size, preprocess)
            out_path = cam_dir / f"{timestamp_ns(frame_index, timestamp_fps)}.png"
            write_png(out_path, out_image)

    source_summary = {
        "source_camera_count": len(cameras),
        "source_sizes": {name: sorted(set(sizes)) for name, sizes in per_camera_sizes.items()},
        "unique_source_sizes": unique_sizes,
        "timestamp_fps": timestamp_fps,
        "video_sample_fps": video_sample_fps,
    }

    if not normalized:
        # Preserve a quick source trace without recoding the original files.
        trace_dir = ensure_dir(output_root / "source_lists")
        for camera in cameras:
            (trace_dir / f"{camera.name}.txt").write_text(
                "\n".join(str(path) for path in camera.image_paths) + "\n",
                encoding="utf-8",
            )

    return PreparedDataset(
        root=dataset_root,
        mav0_root=mav0_root,
        camera_names=[f"cam{i}" for i in range(len(cameras))],
        image_counts=image_counts,
        target_size=target_size,
        normalized=normalized,
        preprocess=preprocess,
        messages=messages,
        source_summary=source_summary,
    )
