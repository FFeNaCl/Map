# 基于单目相机与 SLAM 的水下建图项目

本项目面向“大一年度项目：基于单目相机与 SLAM 的水下建图”。当前仓库保留源码、配置、报告和已经导出的 YOLOv8 ONNX 权重。项目当前主线是：

```text
真实单目相机 / VRX-Gazebo 相机
        -> /image_raw
        -> image_enhancer_cpp 水下图像增强
        -> /image_enhanced
        -> yolo_vrx_bridge 加载 models/best.onnx 做 YOLO 检测
        -> /yolo/annotated + /yolo/detections
        -> ORB-SLAM3 / RViz / 后续语义地图显示
```

中期目标：完成相机采集、图像增强、ONNX 目标检测、标定流程和演示材料整理。  
结题目标：在真实水下环境中实现单目相机采集、抗干扰目标识别、单目 SLAM 建图，并在上位机/RViz 中返回轨迹、点云和语义目标。

## 文件和目录说明

```text
/home/hu/underwater_slam_ws
├── Readme.md
├── 下一步工作Readme.md
├── config
│   ├── enhancer_underwater.yaml
│   ├── water_1280x480.orbslam3.example.yaml
│   └── yolo_vrx_demo.yaml
├── models
│   ├── README.md
│   └── best.onnx
├── src
│   ├── image_enhancer_cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/enhancer.cpp
│   └── yolo_vrx_bridge
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/yolo_vrx_pipeline.launch.py
│       ├── scripts/dataset_player
│       ├── scripts/yolo_detector
│       └── yolo_vrx_bridge
│           ├── dataset_player_node.py
│           ├── image_utils.py
│           └── yolo_detector_node.py
```
说明：

- `src/image_enhancer_cpp/src/enhancer.cpp`：C++ ROS2 水下图像增强节点，订阅 `/image_raw`，发布 `/image_enhanced`，可选发布 `/image_enhanced_debug`。
- `src/yolo_vrx_bridge/yolo_vrx_bridge/yolo_detector_node.py`：YOLO 检测节点，当前可直接加载 `models/best.onnx`，不再强制依赖数据集 `data.yaml`。
- `src/yolo_vrx_bridge/yolo_vrx_bridge/dataset_player_node.py`：可选数据集回放节点。只有恢复 YOLO 格式数据集后才使用。
- `config/enhancer_underwater.yaml`：图像增强参数模板，包含红通道补偿、Gray World 白平衡、CLAHE、去畸变等参数。
- `config/water_1280x480.orbslam3.example.yaml`：ORB-SLAM3 单目配置模板，水下标定完成后把 `ost.yaml` 的内参和畸变参数写入这里。
- `models/best.onnx`：当前保留的 YOLOv8 ONNX 权重，是交付演示中检测功能的核心文件。
- `tools/generate_ppt_materials.py`：PPT 图片生成脚本。历史图片已生成在 `/home/hu/ppt_materials`；如果没有原训练数据，数据集样例图不建议重新生成。

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- OpenCV
- Python3
- Ultralytics + ONNX Runtime
- 可选：VRX/Gazebo、ORB-SLAM3、RViz2

安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-tools \
  ros-humble-v4l2-camera \
  v4l-utils \
  python3-opencv \
  python3-pil \
  python3-yaml
```

安装 YOLO 推理依赖：

```bash
pip install ultralytics onnxruntime
```

## 编译工作空间

```bash
cd /home/hu/underwater_slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 真实相机启动步骤

### 1. 确认相机设备

```bash
ls /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### 2. 终端 1：发布相机图像

```bash
source /opt/ros/humble/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[1280, 720]" \
  -p pixel_format:=MJPG \
  -r image_raw:=/image_raw
```

如果相机不支持 MJPG，可改用：

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[640, 480]" \
  -p pixel_format:=YUYV \
  -r image_raw:=/image_raw
```

### 3. 终端 2：启动水下图像增强

```bash
cd /home/hu/underwater_slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run image_enhancer_cpp enhancer --ros-args \
  --params-file /home/hu/underwater_slam_ws/config/enhancer_underwater.yaml
```

也可以直接传参：

```bash
ros2 run image_enhancer_cpp enhancer --ros-args \
  -p input_topic:=/image_raw \
  -p output_topic:=/image_enhanced \
  -p publish_debug_view:=true \
  -p use_red_compensation:=true \
  -p use_gray_world:=true \
  -p use_clahe:=true \
  -p clahe_clip_limit:=3.0
```

### 4. 终端 3：启动 ONNX 检测

```bash
cd /home/hu/underwater_slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run yolo_vrx_bridge yolo_detector --ros-args \
  -p input_topic:=/image_enhanced \
  -p model_path:=/home/hu/underwater_slam_ws/models/best.onnx \
  -p annotated_topic:=/yolo/annotated \
  -p detections_topic:=/yolo/detections \
  -p confidence_threshold:=0.25
```

### 5. 查看图像与检测结果

```bash
ros2 topic list
ros2 topic echo /yolo/detections --once
ros2 run rqt_image_view rqt_image_view
```

推荐查看：

- `/image_raw`：相机原图。
- `/image_enhanced`：水下增强图像。
- `/image_enhanced_debug`：原图和增强图拼接对比。
- `/yolo/annotated`：ONNX 检测结果图。
- `/yolo/detections`：JSON 格式检测框、类别和置信度。

## 水下标定步骤

水下标定命令：

```bash
ros2 run camera_calibration cameracalibrator \
  --square 0.025 \
  --size 8x6 \
  --ros-args -p image:=/image_enhanced camera:=/camera
```

标定完成后，把 `ost.yaml` 中的参数写入 ORB-SLAM3 配置：

```yaml
Camera.fx: xxx
Camera.fy: xxx
Camera.cx: xxx
Camera.cy: xxx
Camera.k1: xxx
Camera.k2: xxx
Camera.p1: xxx
Camera.p2: xxx
Camera.k3: xxx
```

注意：平面防水壳会引入空气-玻璃-水多介质折射，普通针孔模型只是近似。当前阶段先用水下标定结果提高可用性，后续可考虑折射模型或圆顶端口。

## VRX/Gazebo 接入

本机已有 VRX 资源：

```text
/home/hu/vrx_humble.zip
/home/hu/vrx-humble
```

如需运行 VRX：

```bash
cd /home/hu/vrx-humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=perception_task sim_mode:=full headless:=False
```

VRX 相机话题常见形式：

```text
/wamv/sensors/cameras/front_left_camera_sensor/image_raw
```

接入增强节点：

```bash
ros2 run image_enhancer_cpp enhancer --ros-args \
  -p input_topic:=/wamv/sensors/cameras/front_left_camera_sensor/image_raw \
  -p output_topic:=/image_enhanced
```

再启动 YOLO 检测节点即可复用 `models/best.onnx`。

## 关于数据集和复训

当前交付版本只保留导出的 `models/best.onnx`。因此：

导出后可替换或新增到 `models/`，运行检测时把 `model_path` 指向新文件。

## 如何提高水下检测能力

1. 图像输入优化：使用红通道补偿减轻偏蓝偏绿，使用 Gray World 白平衡稳定颜色，使用 CLAHE 提升局部对比度，并根据水体浑浊程度调整 `clahe_clip_limit`。
2. 标定优化：在水下重新标定相机，把内参和畸变参数同步到增强节点与 ORB-SLAM3，减少检测框中心、特征点和几何位姿之间的不一致。
3. 数据优化：补充真实水池/水箱图像、不同光照与浊度图像、负样本、小目标和遮挡样本，提高模型对水下环境变化的适应能力。
4. 模型优化：从 YOLOv8n 或 YOLOv8s 开始，先保证实时性，再比较不同输入尺寸、置信度阈值和增强策略。
5. SLAM 联动：把动态目标检测框转换为 mask，在 ORB 特征提取时剔除不稳定区域；对静态目标进行多帧三角测量，作为语义地标显示到 RViz。

## 后续主线

1. 用防水棋盘格完成真实水下标定，更新 `ost.yaml` 与 ORB-SLAM3 配置。
2. 使用真实相机跑通 `/image_raw -> /image_enhanced -> /yolo/annotated`。
3. 使用 `models/best.onnx` 做水下目标检测初测，并记录置信度、误检、漏检和 FPS。
4. 接入 ORB-SLAM3 单目，先实现轨迹和稀疏点云。
5. 将 YOLO 检测框转换为动态 mask，测试是否能减少 SLAM 跟踪丢失。
6. 在 RViz 中显示增强图像、检测框、轨迹、点云和语义目标。
7. 补采或恢复训练数据，重新训练并导出更适合真实水下场景的 ONNX 模型。
