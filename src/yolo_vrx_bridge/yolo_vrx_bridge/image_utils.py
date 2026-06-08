from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image


DEFAULT_CLASS_NAMES = [
    "buoy",
    "debris_container",
    "fishing_boat",
    "floating_obstacle",
    "platform",
    "vessel",
]


def load_yolo_names(dataset_root: str | Path) -> List[str]:
    data_path = Path(dataset_root) / "data.yaml"
    if not data_path.exists():
        return []
    with data_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return list(data.get("names", []))


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "mono8": 1,
        "8UC3": 3,
        "8UC1": 1,
    }
    channels = channels_by_encoding.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    image = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        image = image.reshape((msg.height, msg.width))
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    image = image.reshape((msg.height, msg.width, channels))
    if msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def bgr_to_image_msg(image: np.ndarray, stamp, frame_id: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(image.shape[1] * 3)
    msg.data = image.tobytes()
    return msg


def yolo_label_path_for_image(image_path: Path) -> Path:
    split_dir = image_path.parent.parent
    return split_dir / "labels" / f"{image_path.stem}.txt"


def load_yolo_label(label_path: Path, width: int, height: int, names: List[str]) -> List[Dict]:
    detections: List[Dict] = []
    if not label_path.exists():
        return detections

    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        class_id = int(float(parts[0]))
        cx, cy, bw, bh = map(float, parts[1:5])
        x1 = max(0, int(round((cx - bw / 2.0) * width)))
        y1 = max(0, int(round((cy - bh / 2.0) * height)))
        x2 = min(width - 1, int(round((cx + bw / 2.0) * width)))
        y2 = min(height - 1, int(round((cy + bh / 2.0) * height)))
        detections.append(
            {
                "class_id": class_id,
                "class_name": names[class_id] if 0 <= class_id < len(names) else str(class_id),
                "confidence": 1.0,
                "xyxy": [x1, y1, x2, y2],
                "normalized_xywh": [cx, cy, bw, bh],
            }
        )
    return detections


def draw_detections(image: np.ndarray, detections: List[Dict], title: str = "") -> np.ndarray:
    palette: Tuple[Tuple[int, int, int], ...] = (
        (40, 190, 255),
        (255, 120, 60),
        (120, 220, 80),
        (230, 90, 180),
        (80, 180, 230),
        (180, 120, 255),
        (250, 210, 80),
    )
    canvas = image.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
        color = palette[int(det.get("class_id", 0)) % len(palette)]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        label = f"{det.get('class_name', det.get('class_id', '?'))} {det.get('confidence', 0.0):.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        y_top = max(0, y1 - th - baseline - 6)
        cv2.rectangle(canvas, (x1, y_top), (min(canvas.shape[1] - 1, x1 + tw + 8), y_top + th + baseline + 6), color, -1)
        cv2.putText(canvas, label, (x1 + 4, y_top + th + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (15, 25, 35), 2, cv2.LINE_AA)

    if title:
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 42), (15, 35, 45), -1)
        cv2.putText(canvas, title, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (235, 245, 245), 2, cv2.LINE_AA)
    return canvas
