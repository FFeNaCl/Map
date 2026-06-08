# 下一步实际工作清单

这个文件只写接下来要做的实际工作，按优先级推进。目标是把中期演示链路变成真实水下建图链路。

## 第 1 步：水下相机标定

目的：把“水上标定”升级为“实际水下使用环境标定”，减少防水壳和水介质造成的误差。

要做：

1. 用防水棋盘格在水箱或游泳池中采集 20-30 个角度。
2. 相机和棋盘格必须都在水下，水质尽量接近实际实验。
3. 启动 `/image_raw -> /image_enhanced`，用增强后的图像标定。
4. 保存 `ost.yaml`。
5. 把 `fx fy cx cy k1 k2 p1 p2 k3` 写入 ORB-SLAM3 配置文件。

命令：

```bash
ros2 run camera_calibration cameracalibrator \
  --square 0.025 \
  --size 8x6 \
  --ros-args -p image:=/image_enhanced camera:=/camera
```

验收标准：

- 棋盘格角点能稳定识别。
- 重投影误差尽量小。
- 去畸变后图像边缘不再明显弯曲。

## 第 2 步：训练 YOLOv8

目的：把当前“标注回放演示”变成真实模型推理。

要做：

1. 重新补充或恢复 YOLO 格式训练数据。
2. 补充真实水下图片、负样本、小目标和遮挡样本。
3. 用 `yolov8n.pt` 先训练一个轻量模型，并与当前 `models/best.onnx` 对比。
4. 看 mAP、混淆矩阵和验证集可视化。
5. 与当前 `models/best.onnx` 对比，如果效果更好，再替换或新增到 `models/`。

命令：

```bash
mkdir -p /home/hu/underwater_slam_ws/models
yolo detect train \
  data=/path/to/new_dataset/data.yaml \
  model=yolov8n.pt \
  imgsz=640 \
  epochs=100 \
  batch=16 \
  project=/home/hu/underwater_slam_ws/runs/yolo \
  name=yolov8n_underwater
cp /home/hu/underwater_slam_ws/runs/yolo/yolov8n_underwater/weights/best.pt \
   /home/hu/underwater_slam_ws/models/underwater_yolov8n_best.pt
yolo export \
  model=/home/hu/underwater_slam_ws/models/underwater_yolov8n_best.pt \
  format=onnx \
  imgsz=640
```

验收标准：

- 验证集上各类别都能检出。
- 小目标和遮挡目标不要只看单张成功图，要看整体召回率。
- 推理帧率能满足实时演示。

## 第 3 步：跑通 YOLO 实时链路

目的：验证相机图像经过增强后能被 YOLO 实时检测。

真实相机主线命令：

```bash
ros2 run yolo_vrx_bridge yolo_detector --ros-args \
  -p input_topic:=/image_enhanced \
  -p model_path:=/home/hu/underwater_slam_ws/models/best.onnx \
  -p annotated_topic:=/yolo/annotated \
  -p detections_topic:=/yolo/detections
```

如果后续恢复 YOLO 格式数据集，才使用数据集回放模式：

```bash
ros2 launch yolo_vrx_bridge yolo_vrx_pipeline.launch.py \
  dataset_root:=/path/to/new_dataset \
  split:=valid \
  fps:=5.0 \
  model_path:=/home/hu/underwater_slam_ws/models/best.onnx
```

验收标准：

- `/yolo/annotated` 有检测框。
- `/yolo/detections` 能输出类别、置信度和 bbox。
- 检测延迟可接受。

## 第 4 步：接入 ORB-SLAM3 单目

目的：先完成稀疏建图和相机轨迹。

要做：

1. 准备 ORB-SLAM3 的相机配置 yaml。
2. 输入图像用 `/image_enhanced`。
3. 低纹理水下场景中增加特征数量、降低 FAST 阈值。
4. 先在数据集或水池短序列中测试初始化。
5. 保存轨迹、关键帧和稀疏点云。

验收标准：

- 能初始化。
- 相机缓慢移动时轨迹连续。
- 回到起点时漂移不要过大。

## 第 5 步：动态 mask 接入 SLAM 前端

目的：减少鱼、漂浮物、强反光区域对 SLAM 的影响。

要做：

1. 把 `/yolo/detections` 转换成二值 mask。
2. 对动态类别不提 ORB 特征。
3. 对静态类别保留并尝试做语义地标。
4. 对比开启/关闭 mask 的跟踪丢失次数和特征匹配数。

验收标准：

- 动态目标区域特征点明显减少。
- SLAM 跟踪更稳定。
- 地图中不再出现大量跟随动态物体移动的点。

## 第 6 步：VRX/Gazebo 联调

目的：用仿真补充数据和演示材料。

要做：

1. 编译 `/home/hu/vrx-humble`。
2. 启动 `perception_task`。
3. 找到 VRX 相机图像话题。
4. 把相机话题作为增强节点输入。
5. 截图保存 VRX、增强图、YOLO 结果、RViz 显示。

参考命令：

```bash
cd /home/hu/vrx-humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=perception_task sim_mode:=full headless:=False
```

常见相机话题：

```text
/wamv/sensors/cameras/front_left_camera_sensor/image_raw
```

如果 GUI 或桥接暂时不稳定，优先用真实相机短序列测试；只有恢复 YOLO 格式数据集后，才使用数据集回放演示：

```bash
cd /home/hu/underwater_slam_ws
source install/setup.bash
ros2 launch yolo_vrx_bridge yolo_vrx_pipeline.launch.py dataset_root:=/path/to/new_dataset split:=valid fps:=2.0
```

## 第 7 步：RViz 汇总显示

目的：把最终演示做成“上位机自动建图并返回显示”。

要显示：

- `/image_enhanced`
- `/yolo/annotated`
- 相机轨迹 `nav_msgs/Path`
- 稀疏点云 `sensor_msgs/PointCloud2`
- 语义目标 `visualization_msgs/MarkerArray`

验收标准：

- 一屏能看到图像、检测、轨迹、地图和目标。
- 可以录屏或截图用于结题 PPT。

## 第 8 步：结题前要补的数据和指标

建议记录：

- 相机帧率。
- 图像增强耗时。
- YOLO 推理 FPS。
- mAP50 和每类召回率。
- SLAM 跟踪丢失次数。
- 回环前后轨迹漂移。
- 开启/关闭图像增强的特征点数量对比。
- 开启/关闭动态 mask 的轨迹稳定性对比。

这些指标比单纯展示截图更有说服力。
