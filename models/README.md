# YOLO 模型权重目录

训练完成后建议把权重放在这里，例如：

```text
models/best.onnx
models/underwater_yolov8n_best.pt
models/underwater_yolov8n_best.onnx
```

当前已经放入 `models/best.onnx`，可以直接用 ONNX Runtime 进行 YOLO 检测。启动时传入：

```bash
ros2 run yolo_vrx_bridge yolo_detector --ros-args \
  -p input_topic:=/image_enhanced \
  -p model_path:=/home/hu/underwater_slam_ws/models/best.onnx
```
