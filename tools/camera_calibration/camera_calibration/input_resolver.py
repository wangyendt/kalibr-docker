from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from .common import CalibrationError, IMAGE_EXTS, VIDEO_EXTS, UserMessage, sorted_natural, warn
from .i18n import tr


@dataclass
class CameraInput:
    name: str
    kind: str
    image_paths: List[Path]
    video_path: Optional[Path] = None


@dataclass
class InputLayout:
    cameras: List[CameraInput]
    messages: List[UserMessage]

    @property
    def is_multicam(self) -> bool:
        return len(self.cameras) > 1


def _is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def _is_video(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in VIDEO_EXTS


def _direct_images(path: Path) -> List[Path]:
    return sorted_natural([child for child in path.iterdir() if _is_image(child)])


def _direct_videos(path: Path) -> List[Path]:
    return sorted_natural([child for child in path.iterdir() if _is_video(child)])


def _image_subdirs(path: Path) -> List[Path]:
    subdirs: List[Path] = []
    for child in sorted_natural([entry for entry in path.iterdir() if entry.is_dir()]):
        if _direct_images(child) or _direct_videos(child):
            subdirs.append(child)
    return subdirs


def resolve_input(input_path: Path, lang: str = "zh") -> InputLayout:
    messages: List[UserMessage] = []
    input_path = input_path.expanduser().resolve()
    if not input_path.exists():
        raise CalibrationError(f"Input path does not exist: {input_path}")

    if _is_video(input_path):
        warn(messages, "input_video", tr(lang, "input_video"))
        return InputLayout([CameraInput(name="cam0", kind="video", image_paths=[], video_path=input_path)], messages)

    if _is_image(input_path):
        warn(messages, "input_direct_images", tr(lang, "input_direct_images"))
        return InputLayout([CameraInput(name="cam0", kind="images", image_paths=[input_path])], messages)

    if not input_path.is_dir():
        raise CalibrationError(f"Unsupported input path: {input_path}")

    images = _direct_images(input_path)
    videos = _direct_videos(input_path)
    subdirs = _image_subdirs(input_path)

    if (images or videos) and subdirs:
        raise CalibrationError(f"{tr(lang, 'mixed_input')} {tr(lang, 'mixed_input_fix')}")

    if images and videos:
        raise CalibrationError(f"{tr(lang, 'mixed_input')} {tr(lang, 'mixed_input_fix')}")

    if images and not videos:
        warn(messages, "input_direct_images", tr(lang, "input_direct_images"))
        return InputLayout([CameraInput(name="cam0", kind="images", image_paths=images)], messages)

    if videos and not images:
        if len(videos) != 1:
            raise CalibrationError("A single-camera input directory must contain exactly one video file.")
        warn(messages, "input_video", tr(lang, "input_video"))
        return InputLayout([CameraInput(name="cam0", kind="video", image_paths=[], video_path=videos[0])], messages)

    if subdirs:
        cameras: List[CameraInput] = []
        counts = {}
        for idx, subdir in enumerate(subdirs):
            sub_videos = _direct_videos(subdir)
            if sub_videos:
                raise CalibrationError(f"{tr(lang, 'video_in_multicam')} {tr(lang, 'video_in_multicam_fix')}")
            sub_images = _direct_images(subdir)
            if not sub_images:
                continue
            name = subdir.name if subdir.name.startswith("cam") else f"cam{idx}"
            cameras.append(CameraInput(name=name, kind="images", image_paths=sub_images))
            counts[name] = len(sub_images)
        if not cameras:
            raise CalibrationError(tr(lang, "no_input"))
        if len(set(counts.values())) != 1:
            detail = ", ".join(f"{name}={count}" for name, count in counts.items())
            raise CalibrationError(f"{tr(lang, 'count_mismatch')} {detail}. {tr(lang, 'count_mismatch_fix')}")
        warn(messages, "input_multicam", tr(lang, "input_multicam"))
        return InputLayout(cameras, messages)

    raise CalibrationError(tr(lang, "no_input"))


def resolve_target(target_path: Path) -> Path:
    target_path = target_path.expanduser().resolve()
    if target_path.is_file():
        return target_path
    if not target_path.is_dir():
        raise CalibrationError(f"Target config path does not exist: {target_path}")
    yamls = sorted_natural(list(target_path.glob("*.yaml")) + list(target_path.glob("*.yml")))
    if not yamls:
        raise CalibrationError(f"No .yaml/.yml target config found in: {target_path}")
    april = [path for path in yamls if "april" in path.name.lower()]
    return april[0] if april else yamls[0]
