import json
import re
from dataclasses import asdict, dataclass, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


IMAGE_EXTS = {".bmp", ".jpeg", ".jpg", ".png", ".pnm", ".tif", ".tiff"}
VIDEO_EXTS = {".avi", ".m4v", ".mkv", ".mov", ".mp4"}


class CalibrationError(RuntimeError):
    pass


class CalibrationWarning(RuntimeWarning):
    pass


@dataclass
class UserMessage:
    level: str
    code: str
    text: str
    suggestion: str = ""


def natural_key(value: Any) -> List[Any]:
    parts = re.split(r"(\d+)", str(value))
    return [int(part) if part.isdigit() else part.lower() for part in parts]


def sorted_natural(values: Iterable[Any]) -> List[Any]:
    return sorted(values, key=natural_key)


def parse_size(value: Optional[str]) -> Optional[Tuple[int, int]]:
    if not value:
        return None
    match = re.match(r"^(\d+)[xX](\d+)$", value.strip())
    if not match:
        raise CalibrationError("--resize must use WIDTHxHEIGHT, for example 1280x720")
    width = int(match.group(1))
    height = int(match.group(2))
    if width <= 0 or height <= 0:
        raise CalibrationError("--resize width and height must be positive")
    return width, height


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def to_jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return to_jsonable(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(json.dumps(to_jsonable(data), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_yaml(path: Path) -> Dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise CalibrationError(f"YAML file is empty or invalid: {path}")
    return data


def copy_text_file(src: Path, dst: Path) -> None:
    dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")


def info(messages: List[UserMessage], code: str, text: str, suggestion: str = "") -> None:
    messages.append(UserMessage(level="info", code=code, text=text, suggestion=suggestion))


def warn(messages: List[UserMessage], code: str, text: str, suggestion: str = "") -> None:
    messages.append(UserMessage(level="warning", code=code, text=text, suggestion=suggestion))


def error(messages: List[UserMessage], code: str, text: str, suggestion: str = "") -> None:
    messages.append(UserMessage(level="error", code=code, text=text, suggestion=suggestion))


def dedupe_messages(messages: Sequence[UserMessage]) -> List[UserMessage]:
    seen = set()
    result: List[UserMessage] = []
    for msg in messages:
        key = (msg.level, msg.code, msg.text, msg.suggestion)
        if key in seen:
            continue
        seen.add(key)
        result.append(msg)
    return result


def format_messages(messages: Sequence[UserMessage]) -> str:
    lines: List[str] = []
    for msg in messages:
        if msg.level == "error":
            prefix = "ERROR"
        elif msg.level == "warning":
            prefix = "WARNING"
        else:
            prefix = "INFO"
        if msg.suggestion:
            lines.append(f"[{prefix}] {msg.text} Suggestion: {msg.suggestion}")
        else:
            lines.append(f"[{prefix}] {msg.text}")
    return "\n".join(lines)
