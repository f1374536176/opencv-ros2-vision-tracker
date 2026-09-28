# 学习笔记

本项目从零开始学习 OpenCV + ROS2 视觉追踪，笔记按主题整理如下。
按时间/阶段顺序阅读见 [roadmap.md](roadmap.md)。

## 环境与流程

- [ROS2 功能包工作流程](ros2_workflow.md) — 创建、编译、加载、运行
- [编译与环境排错日志](debug_log.md) — 头文件、摄像头、USB、cv_bridge

## 基础视觉
- [HSV 颜色追踪](hsv_basic.md) — 阈值调参、形态学、轮廓查找
- [轨迹追踪](trajectory.md) — deque 滑动窗口与轨迹绘制
- [鼠标 ROI 标定](mouse_roi_calibration.md) — 框选区域 + 动态容差

## 空间感知
- [相机内参标定](camera_calibration.md) — 棋盘格 + calibrateCamera + RMS
- [ArUco 位姿估计](aruco_pose.md) — solvePnP + 坐标轴 + 测距

## 系统集成
- [ROS2 节点](ros2_node.md) — cv_bridge、参数、话题
- [视觉伺服（纯 P）](visual_servo_p.md) — Twist 发布与误差归一化

## 控制优化
- [PID 调参](pid_tuning.md) — 积分限幅、微分低通
- [卡尔曼滤波](kalman_filter.md) — 一维 KF 平滑角速度
- [CSV 日志与曲线](csv_logging.md) — 控制性能量化

## 排错
- [编译与环境排错](debug_log.md) — 头文件、摄像头、USB、cv_bridge