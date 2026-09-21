# ROS2 多模式视觉跟踪与相机标定

本项目是一个基于 **ROS2 + OpenCV** 的视觉伺服目标跟踪示例，主要包含：

- **主程序**：`multi_mode_vision_tracker.cpp`
  - 支持颜色跟踪、ArUco 标记检测、混合模式；
  - 使用 PID + 一维卡尔曼滤波生成 `/cmd_vel` 控制指令；
  - 记录 `tracking_log.csv` 控制日志；
  - 发布检测结果图像和掩膜图像。

- **辅助工具**：`camera_calibration.cpp`
  - 使用棋盘格进行相机标定；
  - 生成 `calibration.yaml`，供主程序读取相机内参和畸变系数。

- **离线分析脚本**：`scripts/plot_log.py`、`scripts/kalman_filter.py`
  - 仅用于分析 `tracking_log.csv`；
  - 绘制控制响应曲线、卡尔曼滤波平滑效果图；
  - 不参与 ROS2 实时控制。

> 注意：`scripts/` 下的 Python 文件只是数据分析工具，不是本项目的主程序。


## 目录结构
.
├── README.md
├── CMakeLists.txt
├── package.xml
├── config/
│   └── calibration.yaml          # 相机标定文件，由标定程序生成
├── src/
│   ├── multi_mode_vision_tracker.cpp   # 主程序：多模式视觉跟踪节点
│   └── camera_calibration.cpp          # 辅助工具：棋盘格相机标定
└── scripts/
    ├── plot_log.py               # 绘制控制响应曲线与统计
    └── kalman_filter.py          # 卡尔曼滤波平滑示例


---

## 功能特性

### 1. 多模式视觉跟踪

主节点 `MultiModeVisionTracker` 支持三种模式：

| 按键 | 模式 | 说明 |
|---|---|---|
| `1` | 颜色模式 | 仅使用 HSV 颜色阈值跟踪目标 |
| `2` | ArUco 模式 | 仅检测 ArUco 标记并估计位姿 |
| `3` | 混合模式 | 同时运行颜色跟踪和 ArUco 检测，默认模式 |

### 2. 颜色跟踪

- 默认 HSV 阈值示例为绿色范围；
- 支持鼠标拖拽框选目标区域；
- 按 `C` 键根据框选区域自动标定 HSV 阈值；
- 绘制目标中心、外接矩形和运动轨迹；
- 按 `R` 键清空轨迹。

### 3. ArUco 检测

- 使用 `DICT_6X6_250` 字典；
- 检测 ArUco 标记并绘制坐标轴；
- 使用 `solvePnP` 估计标记位姿；
- 输出标记到相机的 Z 方向距离。

### 4. PID + 卡尔曼滤波控制

- 根据颜色目标中心与图像中心的横向误差计算角速度；
- PID 参数可通过 ROS2 参数配置；
- 积分项带限幅，防止积分饱和；
- 微分项带低通滤波；
- 输出角速度限幅在 `[-0.8, 0.8]`；
- 使用一维卡尔曼滤波平滑角速度；
- 目标连续丢失超过 5 帧后，角速度置零并重置滤波器状态。

### 5. 日志记录

主程序运行后会生成：

```text
tracking_log.csv
```

日志列格式：

| 列名 | 含义 |
|---|---|
| `frame` | 帧号 |
| `error_x` | 横向误差，像素 |
| `cmd_angular` | 输出的角速度，rad/s |
| `cmd_linear` | 输出的线速度，m/s |
| `color_detected` | 是否检测到颜色目标，1/0 |
| `aruco_dist` | ArUco 标记 Z 方向距离，米 |

---

## 环境依赖

### ROS2

- ROS2 Jazzy 
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- OpenCV，需包含 ArUco 模块

### Python 分析脚本

- Python 3.8+
- pandas
- numpy
- matplotlib

---

## 编译与运行

以下以 ROS2 包名 `vision_tracker` 为例，请按你的实际包名修改。

### 1. 编译

```bash
colcon build --packages-select vision_tracker
source install/setup.bash
```

### 2. 运行相机标定程序

先运行标定程序，生成 `calibration.yaml`：

```bash
ros2 run vision_tracker camera_calibration
```

操作说明：

- 准备棋盘格，默认内角点为 `9 x 6`，需与程序一致；
- 按 `空格键` 拍摄当前画面；
- 至少拍摄 10 张不同角度、不同距离的棋盘格图像；
- 按 `Q` 键开始计算标定参数；
- 程序会输出重投影误差 RMS，并保存：

```text
calibration.yaml
```

标定文件包含：

- `image_width`
- `image_height`
- `camera_matrix`
- `distortion_coefficients`

### 3. 运行主跟踪节点

```bash
ros2 run vision_tracker multi_mode_vision_tracker --ros-args \
  -p camera_topic:=/image_raw \
  -p calibration_file:=config/calibration.yaml \
  -p marker_size:=0.03 \
  -p kp:=0.5 \
  -p ki:=0.1 \
  -p kd:=0.1 \
  -p max_integral:=1.0
```

如果 `calibration.yaml` 就在当前工作目录，也可以：

```bash
ros2 run vision_tracker multi_mode_vision_tracker --ros-args \
  -p calibration_file:=calibration.yaml
```

### 4. 运行 Python 分析脚本

确保 `tracking_log.csv` 在当前工作目录，或修改脚本中的读取路径。

```bash
python scripts/plot_log.py
python scripts/kalman_filter.py
```

生成图片：

- `control_response.png`
- `kalman_smooth.png`

---

## ROS2 接口

### 订阅话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/image_raw` | `sensor_msgs/msg/Image` | 相机原始图像，可通过参数 `camera_topic` 修改 |

### 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/detection_result` | `sensor_msgs/msg/Image` | 带检测框、轨迹、坐标轴的图像 |
| `/mask` | `sensor_msgs/msg/Image` | 颜色阈值掩膜图像 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制指令，主要使用 `angular.z` |

---

## ROS2 参数

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `camera_topic` | `/image_raw` | 相机图像话题 |
| `calibration_file` | `calibration.yaml` | 相机标定文件路径 |
| `marker_size` | `0.03` | ArUco 标记边长，单位米 |
| `kp` | `0.5` | PID 比例系数 |
| `ki` | `0.1` | PID 积分系数 |
| `kd` | `0.1` | PID 微分系数 |
| `max_integral` | `1.0` | 积分项限幅 |

---

## 键盘操作

| 按键 | 功能 |
|---|---|
| `1` | 切换到颜色模式 |
| `2` | 切换到 ArUco 模式 |
| `3` | 切换到混合模式 |
| `R` | 清空轨迹 |
| `C` | 根据鼠标框选区域标定 HSV 阈值 |
| `ESC` | 取消框选 |
| `Q` | 退出程序 |

---

## Python 分析脚本说明

### `scripts/plot_log.py`

读取 `tracking_log.csv`，绘制：

- 左轴：横向误差 `error_x`，单位像素；
- 右轴：控制角速度 `cmd_angular`，单位 rad/s；
- 保存 `control_response.png`；
- 输出控制性能统计：
  - 最大绝对误差；
  - 平均绝对误差；
  - 稳态误差，最后 100 帧均值；
  - 最大角速度。

### `scripts/kalman_filter.py`

读取 `tracking_log.csv` 中的 `cmd_angular`：

- 模拟带高斯噪声的 IMU 角速度；
- 使用一维卡尔曼滤波进行平滑；
- 对比原始控制指令、含噪测量值、滤波输出；
- 保存 `kalman_smooth.png`。

---

## 注意事项

- 主程序输出的角速度经过一维卡尔曼滤波平滑；scripts/kalman_filter.py 中用于对比的“IMU 含噪测量值”为脚本模拟数据，并非真实传感器数据
- `cv_bridge` 头文件路径可能因 ROS2 版本不同而不同，常见为：
  - `#include <cv_bridge/cv_bridge.h>`
  - 部分新版本可能使用 `.hpp`，请以你的环境为准；
- OpenCV ArUco API 在不同 OpenCV 版本中位置可能变化，请确认已安装带 ArUco 的 OpenCV；
- Python 脚本使用相对路径 `tracking_log.csv`，运行时请确保当前目录下存在该文件；
- Matplotlib 中文字体在 Linux 下可能需要安装中文字体，否则中文会显示为方框；
- 线速度 cmd_linear 目前为预留字段，尚未实现闭环控制，CSV 中恒为 0；后续计划根据 ArUco 距离或目标框面积实现前后运动。
- 目前 HSV 标定适合非红色目标（红色 H 分量跨越 0/179 边界，简单均值±标准差无法覆盖）；红色目标的 HSV 标定待后续实现
- 本项目为个人学习项目，未在真实机器人平台上做闭环实测，PID 参数、卡尔曼 Q/R、角速度限幅等均为经验值，实际部署需重新调参。

---
