# ROS2 功能包工作流程

> 本文件记录 ROS2 工作空间的创建、编译、加载、运行流程。  
> 适用于所有 ROS2 节点笔记，改代码后按本文三步重新编译即可。

---

## 1. 创建工作空间与功能包

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

ros2 pkg create vision_tracker \
    --build-type ament_cmake \
    --dependencies rclcpp OpenCV sensor_msgs cv_bridge image_transport \
    --node-name multi_mode_vision_tracker
```

- `--build-type ament_cmake`：C++ 功能包用 ament_cmake 构建系统。
- `--dependencies`：声明依赖，`package.xml` 和 `CMakeLists.txt` 会自动补上。
- `--node-name`：自动生成一个同名 `.cpp` 骨架，可以删掉重写。

> 注意：`vision_tracker` 是本项目的实际包名，`opencv_test` 是教程里的示例名，不要混用。

---

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --packages-select vision_tracker
```

- `--packages-select`：只编译指定包，速度快。改动只在单个包时用。
- 想全编译：去掉 `--packages-select vision_tracker`，直接 `colcon build`。
- 改完 `.cpp` 后**必须重新编译**，否则 `ros2 run` 跑的还是旧二进制。

---

## 3. 加载环境

```bash
source install/setup.bash
```

- 必须在**工作空间根目录**执行。
- 不在根目录时用绝对路径：
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- 每开一个新终端都要 source 一次，或者写进 `~/.bashrc`。

---

## 4. 运行节点

```bash
ros2 run vision_tracker multi_mode_vision_tracker
```

带参数时：

```bash
ros2 run vision_tracker multi_mode_vision_tracker --ros-args -p camera_topic:=/image_raw
```

- 第一个参数是功能包名，第二个是可执行节点名。
- 节点名在 `CMakeLists.txt` 的 `add_executable` 里定义，不一定和 `.cpp` 文件名相同。

---

## 5. 完整流程（改代码后重跑）

```bash
cd ~/ros2_ws
colcon build --packages-select vision_tracker
source install/setup.bash
ros2 run vision_tracker multi_mode_vision_tracker
```

---

## 6. 常见问题

- **`ros2 run` 找不到节点**：忘了 `source install/setup.bash`，或者忘了 `colcon build`。
- **改了代码没生效**：忘了重新 `colcon build`。
- **新终端找不到包**：每开一个新终端都要重新 `source install/setup.bash`。
- **CMake 报错找不到 OpenCV / cv_bridge**：检查 `package.xml` 和 `CMakeLists.txt` 是否声明了依赖。

> 编译、头文件、cv_bridge 等具体报错见 [debug_log.md](debug_log.md)。