# 编译与环境排错日志

按问题类型整理跨主题的编译、权限、虚拟机配置问题。  
主题相关的调试记录见各主题笔记的「踩坑记录」小节。

## 编译

### 头文件找不到 `opencv2/opencv.hpp`

- **现象**：`fatal error: opencv2/opencv.hpp: No such file or directory`
- **排查**：
  1. 改 `c_cpp_properties.json` → 只解决 VSCode 语法识别，编译仍失败
  2. 改 `tasks.json` → VSCode 运行时仍用旧命令
- **根因**：编译命令没带 OpenCV 的 include 路径和链接库
- **解决**：在已配置好的 ROS2 功能包下编译，靠 CMake 处理依赖
- **影响主题**：hsv_basic、trajectory、camera_calibration、aruco_pose...

### 新建 `.cpp` 新建程序需要修改 CMakeLists.txt

> 注：本篇的 CMakeLists 是 ROS2 ament 包的写法。纯 CMake 项目要用 `find_package(OpenCV REQUIRED)` + `target_link_libraries`。

- **现象**：新建 `camera_calibration.cpp` 后编译报错，找不到 `main` 或找不到 OpenCV 符号。
- **根因**：CMake 默认不会自动编译新文件，必须显式声明。
- **解决**：在 `CMakeLists.txt` 里仿照原有结构添加新程序部分

```cmake
# ===== 模板：新建程序时复制这一段，按需删减依赖 =====
add_executable(<程序名> src/<源文件名>.cpp)
target_include_directories(<程序名> PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(<程序名> PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  <程序名>
  # 按需保留以下依赖，用不到的删掉
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "geometry_msgs"
)

install(TARGETS <程序名>
  DESTINATION lib/${PROJECT_NAME})

# ===== 实际例子：camera_calibration 只用 OpenCV =====
add_executable(camera_calibration src/camera_calibration.cpp)
target_include_directories(camera_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(camera_calibration PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  camera_calibration
  "OpenCV"
)
install(TARGETS camera_calibration
  DESTINATION lib/${PROJECT_NAME})
```

## 摄像头

### 摄像头无权限

- **现象**：`cap.isOpened()` 返回 false
- **根因**：虚拟机里 USB 设备一次只能被宿主机或虚拟机其中一方占用
- **解决**：在虚拟机「可连接设备」里手动把摄像头连到虚拟机
- **影响主题**：所有需要摄像头的篇

### 摄像头画面花屏

- **现象**：画面撕裂、彩色条纹
- **根因**：虚拟机 USB 兼容版本设置错误
- **解决**：修改 USB 控制器兼容版本后恢复正常
- **影响主题**：所有需要摄像头的篇

## OpenCV 窗口

### 掩膜窗口显示异常

- **现象**：`Mask` 窗口过小，显示不全
- **解决**：`namedWindow("Mask (黑白阈值)", WINDOW_AUTOSIZE)`
- **影响主题**：hsv_basic、trajectory

## ROS2 相关

### cv_bridge 头文件 `.h` vs `.hpp`

- **现象**：`fatal error: cv_bridge/cv_bridge.h: No such file or directory`
- **根因**：ROS2 版本差异，新版本改用 `.hpp`
- **解决**：改成 `#include <cv_bridge/cv_bridge.hpp>`
- **影响主题**：ros2_node、visual_servo_p、pid_tuning