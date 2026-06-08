from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dataset_root = LaunchConfiguration("dataset_root")
    split = LaunchConfiguration("split")
    fps = LaunchConfiguration("fps")
    model_path = LaunchConfiguration("model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root", default_value=""),
            DeclareLaunchArgument("split", default_value="valid"),
            DeclareLaunchArgument("fps", default_value="2.0"),
            DeclareLaunchArgument("model_path", default_value=""),
            Node(
                package="yolo_vrx_bridge",
                executable="dataset_player",
                name="yolo_dataset_player",
                output="screen",
                parameters=[
                    {
                        "dataset_root": dataset_root,
                        "split": split,
                        "fps": fps,
                        "image_topic": "/image_raw",
                        "label_topic": "/dataset/labels",
                        "frame_id": "vrx_sim_camera",
                    }
                ],
            ),
            Node(
                package="image_enhancer_cpp",
                executable="enhancer",
                name="image_enhancer_cpp",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/image_raw",
                        "output_topic": "/image_enhanced",
                        "publish_debug_view": True,
                        "debug_topic": "/image_enhanced_debug",
                        "use_red_compensation": True,
                        "use_gray_world": True,
                        "use_clahe": True,
                        "clahe_clip_limit": 3.0,
                        "clahe_tile_size": 8,
                    }
                ],
            ),
            Node(
                package="yolo_vrx_bridge",
                executable="yolo_detector",
                name="yolo_detector",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/image_enhanced",
                        "label_topic": "/dataset/labels",
                        "annotated_topic": "/yolo/annotated",
                        "detections_topic": "/yolo/detections",
                        "dataset_root": dataset_root,
                        "model_path": model_path,
                        "allow_label_replay": True,
                        "frame_title": "VRX / vessel YOLO detection",
                    }
                ],
            ),
        ]
    )
