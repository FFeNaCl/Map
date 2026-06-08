from __future__ import annotations

import json
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .image_utils import bgr_to_image_msg, load_yolo_label, load_yolo_names, yolo_label_path_for_image


class DatasetPlayer(Node):
    def __init__(self) -> None:
        super().__init__("yolo_dataset_player")

        dataset_root_value = str(self.declare_parameter("dataset_root", "").value)
        if not dataset_root_value:
            raise RuntimeError("dataset_root is empty; provide a YOLO-format dataset path only when using dataset replay")
        self.dataset_root = Path(dataset_root_value)
        self.split = str(self.declare_parameter("split", "valid").value)
        self.image_topic = str(self.declare_parameter("image_topic", "/image_raw").value)
        self.label_topic = str(self.declare_parameter("label_topic", "/dataset/labels").value)
        self.frame_id = str(self.declare_parameter("frame_id", "sim_camera").value)
        self.fps = float(self.declare_parameter("fps", 2.0).value)
        self.loop = bool(self.declare_parameter("loop", True).value)

        self.names = load_yolo_names(self.dataset_root)
        self.images = sorted((self.dataset_root / self.split / "images").glob("*.jpg"))
        if not self.images:
            raise RuntimeError(f"no images found in {self.dataset_root / self.split / 'images'}")

        self.index = 0
        self.image_pub = self.create_publisher(type(bgr_to_image_msg(cv2.imread(str(self.images[0])), self.get_clock().now().to_msg(), self.frame_id)), self.image_topic, 10)
        self.label_pub = self.create_publisher(String, self.label_topic, 10)
        period = max(0.02, 1.0 / max(self.fps, 0.1))
        self.timer = self.create_timer(period, self.publish_next)

        self.get_logger().info(
            f"replaying {len(self.images)} images from {self.dataset_root}/{self.split} on {self.image_topic}"
        )

    def publish_next(self) -> None:
        if self.index >= len(self.images):
            if not self.loop:
                return
            self.index = 0

        image_path = self.images[self.index]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning(f"failed to read image: {image_path}")
            self.index += 1
            return

        stamp = self.get_clock().now().to_msg()
        image_msg = bgr_to_image_msg(image, stamp, self.frame_id)
        self.image_pub.publish(image_msg)

        detections = load_yolo_label(yolo_label_path_for_image(image_path), image.shape[1], image.shape[0], self.names)
        label_msg = String()
        label_msg.data = json.dumps(
            {
                "source": self.dataset_root.name or "yolo_dataset",
                "split": self.split,
                "filename": image_path.name,
                "width": image.shape[1],
                "height": image.shape[0],
                "detections": detections,
            },
            ensure_ascii=True,
        )
        self.label_pub.publish(label_msg)
        self.index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
