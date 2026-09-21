cmake_minimum_required(VERSION 3.8)
project(vision_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 可选：打印 OpenCV 版本，方便排查 ArUco 问题
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ==================== 主程序：多模式视觉跟踪节点 ====================
add_executable(multi_mode_vision_tracker src/multi_mode_vision_tracker.cpp)

target_compile_features(multi_mode_vision_tracker PUBLIC cxx_std_17)

target_include_directories(multi_mode_vision_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  multi_mode_vision_tracker
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# ==================== 辅助工具：相机标定 ====================
add_executable(camera_calibration src/camera_calibration.cpp)

target_compile_features(camera_calibration PUBLIC cxx_std_17)

# 标定程序不用 ROS，只链接 OpenCV
ament_target_dependencies(
  camera_calibration
  OpenCV
)

# ==================== 安装可执行文件 ====================
install(TARGETS
  multi_mode_vision_tracker
  camera_calibration
  DESTINATION lib/${PROJECT_NAME}
)

# 如果存在 config 目录，就一起安装到 share/vision_tracker/config
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ==================== 测试 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()