# 学习记录

按时间线记录每天的学习任务、核心产出和关键思考。
- 按主题阅读：见 [docs/README.md](README.md)
- 环境/编译/权限问题：统一见 [debug_log.md](debug_log.md)

| 阶段 | 主题 | 核心产出 | 详细笔记 |
|------|------|---------|---------|
| 1 | 实时颜色识别 | HSV 阈值调参、形态学处理、轮廓查找 | [hsv_basic.md](hsv_basic.md) |
| 2 | 轨迹追踪 | deque 滑动窗口、轨迹绘制 | [trajectory.md](trajectory.md) |
| 3 | 光照自适应 | 中央 ROI 自动取色、动态容差 | [hsv_basic.md](hsv_basic.md) |
| 4 | 鼠标标定 | setMouseCallback、鼠标框选 ROI | [mouse_roi_calibration.md](mouse_roi_calibration.md) |
| 5 | 相机内参标定 | 棋盘格检测、calibrateCamera、RMS=0.459 | [camera_calibration.md](camera_calibration.md) |
| 6 | ArUco 位姿估计 | solvePnP、坐标轴绘制、Z 轴测距 | [aruco_pose.md](aruco_pose.md) |
| 7 | ROS2 节点 | 单文件 OpenCV → ROS2 节点 | [ros2_node.md](ros2_node.md) |
| 8 | 视觉伺服（纯 P） | 归一化误差、Twist 发布 | [visual_servo_p.md](visual_servo_p.md) |
| 9 | CSV 日志 | 控制曲线记录、Python 绘图、指标统计 | [csv_logging.md](csv_logging.md) |
| 10 | 卡尔曼滤波 | 一维 KF 平滑角速度，Q/R 调参 | [kalman_filter.md](kalman_filter.md) |
| 11 | 抗积分饱和 PID | 积分限幅、微分低通、dt 鲁棒获取 | [pid_tuning.md](pid_tuning.md) |
| 12 | Gazebo 仿真 | 仿真时间同步、纯视觉控制缺陷分析 | [visual_servo_p.md](visual_servo_p.md#gazebo) |

## 关键里程碑

- **HSV 动态标定跑通**：鼠标框选 ROI + meanStdDev 自动容差。
- **相机标定完成**：RMS = 0.459，畸变参数写入 calibration.yaml。
- **ArUco 测距可用**：Z 轴距离实测与真实值一致。
- **ROS2 节点跑通**：话题订阅 /image_raw，发布 /cmd_vel 与 /detection_result。
- **视觉伺服闭环**：颜色追踪 + ArUco 测距 → PID → /cmd_vel。
- **控制信号可量化**：CSV 日志 + Python 曲线 + 稳态误差/超调量统计。

## 学习路径

1. 基础视觉：HSV 颜色追踪 → 轨迹绘制 → 动态标定
2. 空间感知：相机标定 → ArUco 位姿估计
3. 系统集成：ROS2 节点 → 话题通信 → 视觉伺服
4. 控制优化：纯 P → PID → 抗积分饱和 → 卡尔曼滤波
5. 工程化：CSV 日志 → 曲线分析 → 仿真验证