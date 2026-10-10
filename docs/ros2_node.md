# ROS2 节点

> 目标：把前面所有单文件 OpenCV 程序（颜色追踪 + 轨迹 + 鼠标标定 + ArUco）改造成一个 ROS2 节点，订阅图像话题，发布处理结果。  
> 环境：Ubuntu 24.04 + ROS2 jazzy + OpenCV 4.8 + C++  
> 相关：前置 [hsv_basic.md](hsv_basic.md)、[trajectory.md](trajectory.md)、[mouse_roi_calibration.md](mouse_roi_calibration.md)、[camera_calibration.md](camera_calibration.md)、[aruco_pose.md](aruco_pose.md)。

---

## 1. 为什么要改造成 ROS2 节点

单文件 OpenCV 程序的问题：

| 问题 | 表现 |
|------|------|
| 摄像头独占 | `VideoCapture cap(0)` 打开后，别的程序拿不到图像 |
| 数据与功能耦合 | 采集、处理、显示在一个 `while(true)` 里，无法拆分 |
| 没有外部参数 | 改 HSV 阈值、改标定文件路径都要重新编译 |
| 没有对外输出 | 结果只显示在窗口里，控制层拿不到 |

ROS2 节点化的收益：

| 收益 | 说明 |
|------|------|
| 图像共享 | 一个摄像头节点发布，多个处理节点订阅 |
| 参数化 | 用 `declare_parameter` 声明，启动时通过 launch 或命令行覆盖 |
| 数据流 | 处理结果发布到话题，控制节点直接订阅 |
| 可拆分 | 颜色追踪、ArUco 检测、控制逻辑可以分成独立节点 |

本篇的目标不是把功能拆成多个节点，而是**先把已有的单文件程序打包成一个节点**，为后续拆分做准备。

---

## 2. 节点类的骨架

### 2.1 继承 `rclcpp::Node`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class MultiModeVisionTracker : public rclcpp::Node {
public:
    MultiModeVisionTracker() : Node("multi_mode_vision_tracker") {
        // ...
    }
private:
    // ...
};
```

| 部分 | 说明 |
|------|------|
| `class MultiModeVisionTracker` | 自定义的节点类 |
| `: public rclcpp::Node` | 公有继承 ROS2 的节点基类 |
| `public:` | 构造函数对外，`main` 里要创建对象 |
| `private:` | 成员变量和内部逻辑 |

**继承之后**，`MultiModeVisionTracker` 的每一个实例，本质上就是一个完整的 ROS2 节点。可以调用 `this->create_subscription(...)`、`this->get_logger()` 等父类方法。

### 2.2 构造函数与初始化列表

```cpp
MultiModeVisionTracker() : Node("multi_mode_vision_tracker") {
```

| 部分 | 含义 |
|------|------|
| `MultiModeVisionTracker()` | 构造函数名，必须和类名一致 |
| `:` | 初始化列表的开始 |
| `Node("multi_mode_vision_tracker")` | 给父类 `rclcpp::Node` 传节点名 |

**初始化列表的意义**：在进入构造函数体之前，先把父类 `rclcpp::Node` 构造好，把节点名注册进 ROS2。执行 `ros2 node list` 时看到的名字就是 `/multi_mode_vision_tracker`。

### 2.3 `public` / `private` / `protected`

| 标签 | 权限范围 | 使用场景 |
|------|---------|---------|
| `public:` | 所有人（类外部、`main`、其他类） | 构造函数、对外接口 |
| `private:` | 仅本类内部 | 成员变量、内部处理函数 |
| `protected:` | 本类 + 子类 | 想让子类继承的工具函数 |

标签下代码**没有优先级关系**，只有作用域不同。习惯上先写 `public` 后写 `private`，但 `private` 里声明的成员变量可以在 `public` 的构造函数里直接初始化。

### 2.4 析构函数

```cpp
~MultiModeVisionTracker() {
    destroyAllWindows();
}
```

- `~` 是析构函数标志。
- 不需要手动调用，`main` 结束或 `rclcpp::shutdown()` 触发节点销毁时，C++ 运行时会自动进入大括号。
- 适合放清理代码：关闭窗口、释放文件、flush 日志。

---

## 3. 通信

### 3.1 订阅图像话题

```cpp
img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    topic, 10,
    std::bind(&MultiModeVisionTracker::imageCallback, this, std::placeholders::_1));
```

| 参数 | 说明 |
|------|------|
| `<sensor_msgs::msg::Image>` | 模板参数，指定消息类型 |
| `topic` | 话题名，从参数 `camera_topic` 读取 |
| `10` | 队列深度，缓存最近 10 条消息 |
| `std::bind(...)` | 回调绑定器 |

**`std::bind` 三件事**：

1. `&MultiModeVisionTracker::imageCallback`：函数地址。
2. `this`：当前对象指针。非静态成员函数有隐藏参数 `this`，`bind` 把它固定下来。
3. `std::placeholders::_1`：占位符，代表“回调函数第一个参数”，由 ROS2 在消息到达时填入。

### 3.2 发布结果和掩膜

```cpp
result_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detection_result", 10);
mask_pub_   = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);
```

- 第一个参数是话题名。
- 第二个参数是队列深度，防止发布阻塞。发布者队列过大时会增加内存占用。

> **补充**：主程序里图像相关话题实际用的是 `rclcpp::SensorDataQoS().keep_last(1)`，只保留最新一帧，避免图像积压。参数填 `10` 是初版写法，跑图像流时建议换成 SensorDataQoS。

发布时：

```cpp
sensor_msgs::msg::Image::SharedPtr out_msg =
    cv_bridge::CvImage(msg->header, "bgr8", frame_undistorted).toImageMsg();
result_pub_->publish(*out_msg);
```

| 部分 | 说明 |
|------|------|
| `cv_bridge::CvImage(...)` | 临时构造一个桥接对象 |
| `msg->header` | 借用输入图像的时间戳和坐标系 |
| `"bgr8"` | 编码格式 |
| `frame_undistorted` | 要发布的像素数据 |
| `.toImageMsg()` | 装箱成 `sensor_msgs::msg::Image` |
| `*out_msg` | 解引用智能指针，拿到消息对象 |
| `.publish(...)` | 塞进发布队列 |

发布掩膜前要先判空：

```cpp
if (!mask_.empty()) {
    sensor_msgs::msg::Image::SharedPtr mask_msg =
        cv_bridge::CvImage(msg->header, "mono8", mask_).toImageMsg();
    mask_pub_->publish(*mask_msg);
}
```

**为什么必须判空**：

- ArUco 模式下不会调用 `processColorTracking`，`mask_` 不会更新；
- 如果 `imageCallback` 开头没做 `mask_.release()`，`/mask` 里发的就是上一帧颜色模式留下的旧掩膜；
- 如果从来没进过颜色模式，`mask_` 为空，`cv_bridge::CvImage(..., "mono8", mask_)` 包一个空 `Mat`，可能抛异常或发布无效消息。

所以主程序里做了两层保护：

```cpp
// 每帧开头先清空，避免旧图残留
mask_.release();

// 发布前再判空，避免空 Mat 装箱
if (!mask_.empty()) {
    ...
}
```

### 3.3 头文件的作用

| 头文件 | 作用 |
|--------|------|
| `rclcpp/rclcpp.hpp` | ROS2 C++ 客户端库，让程序拥有节点身份 |
| `sensor_msgs/msg/image.hpp` | ROS2 标准图像消息定义 |
| `cv_bridge/cv_bridge.hpp` | ROS2 图像消息 ↔ `cv::Mat` 的桥接库 |

> **坑**：ROS2 jazzy 的 `cv_bridge` 头文件是 `.hpp`，早期版本是 `.h`。AI 生成的代码可能混用。

---

## 4. 参数系统

### 4.1 声明参数

```cpp
this->declare_parameter<string>("camera_topic", "/image_raw");
this->declare_parameter<string>("calibration_file", "calibration.yaml");
this->declare_parameter<double>("marker_size", 0.03);
```

| 部分 | 说明 |
|------|------|
| `this->` | 指向当前节点对象 |
| `declare_parameter<类型>("名字", 默认值)` | 在 ROS2 参数服务器注册 |

**性质**：参数是**外部接口**，启动时可以通过 launch 文件或 `ros2 param set` 覆盖，不用重新编译。

**适合参数化的变量**：

- 环境相关：摄像头话题、标定文件路径
- 物理尺寸：ArUco 边长
- 控制参数：PID 增益

### 4.2 读取参数

```cpp
string calib_file = this->get_parameter("calibration_file").as_string();
double marker_size = this->get_parameter("marker_size").as_double();
```

| 部分 | 说明 |
|------|------|
| `get_parameter("名字")` | 返回参数对象 |
| `.as_string()` | 按字符串解包 |
| `.as_double()` | 按 double 解包 |

类型不匹配会报错。

### 4.3 错误处理与 `shutdown`

```cpp
if (!fs.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开标定文件: %s", calib_file.c_str());
    rclcpp::shutdown();
    return;
}
```

| 部分 | 说明 |
|------|------|
| `RCLCPP_ERROR(logger, fmt, ...)` | 打印错误日志 |
| `this->get_logger()` | 当前节点的日志记录器 |
| `%s` | 格式化占位符 |
| `calib_file.c_str()` | `std::string` → `const char*` |
| `rclcpp::shutdown()` | 全局关闭信号，停止所有节点 |

日志级别：`RCLCPP_INFO` / `RCLCPP_WARN` / `RCLCPP_ERROR` / `RCLCPP_DEBUG`。

---

## 5. 图像回调里的 OpenCV 流程

### 5.1 `cv_bridge` 转换

```cpp
cv_bridge::CvImagePtr cv_ptr;
try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
} catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
}
Mat frame = cv_ptr->image;
if (frame.empty()) return;
```

| 部分 | 说明 |
|------|------|
| `cv_bridge::CvImagePtr` | 智能指针，指向 ROS2 图像 + OpenCV 矩阵混合体 |
| `try { ... }` | 高危区，出错时抛异常而不是崩溃 |
| `catch (cv_bridge::Exception &e)` | 捕获桥接异常 |
| `e.what()` | 异常信息 |
| `toCvCopy(msg, encoding)` | 复制转换，深拷贝，安全但开销大 |
| `BGR8` | 目标编码，OpenCV 默认彩色格式 |

> 另一个函数是 `toCvShare`，浅拷贝，零开销但不安全。需要修改图像时用 `toCvCopy`。

### 5.2 去畸变

```cpp
Mat frame_undistorted;
undistort(frame, frame_undistorted, cameraMatrix_, distCoeffs_);
```

`cameraMatrix_` 和 `distCoeffs_` 在构造函数里从 `calibration.yaml` 加载。

### 5.3 模式分发

```cpp
switch (current_mode_) {
    case MODE_COLOR_ONLY:
        processColorTracking(frame_undistorted);
        break;
    case MODE_ARUCO_ONLY:
        processArucoDetection(frame_undistorted);
        break;
    case MODE_FUSION:
        processColorTracking(frame_undistorted);
        processArucoDetection(frame_undistorted);
        break;
}
```

三种模式对应三种功能组合，避免把颜色和 ArUco 逻辑写在一个大函数里。

### 5.4 显示与按键

```cpp
imshow("Detection Result", frame_undistorted);
if (!mask_.empty())
    imshow("Mask (黑白阈值)", mask_);
else
    imshow("Mask (黑白阈值)", Mat::zeros(frame_undistorted.size(), CV_8UC1));

int key = waitKey(1);
handleKeyInput(key, frame_undistorted);
```

`Mat::zeros(size, CV_8UC1)` 生成全黑灰度图，作为“没有掩膜时”的占位。

| 参数 | 说明 |
|------|------|
| `CV_8U` | 8 位无符号整数，0–255 |
| `C1` | 1 通道，灰度图 |

### 5.5 按键处理

```cpp
void handleKeyInput(int key, Mat &frame) {
    if (key == 'q' || key == 'Q') {
        RCLCPP_INFO(this->get_logger(), "退出程序");
        rclcpp::shutdown();
    }
    else if (key == '1') { current_mode_ = MODE_COLOR_ONLY; }
    else if (key == '2') { current_mode_ = MODE_ARUCO_ONLY; }
    else if (key == '3') { current_mode_ = MODE_FUSION; }
    else if (key == 'r' || key == 'R') { trajectory_.clear(); }
    else if (key == 'c' || key == 'C') { /* 标定 */ }
    else if (key == 27) { /* 取消框选 */ }
}
```

---

## 6. 鼠标回调的静态化

### 6.1 为什么需要 `static`

OpenCV 的 `setMouseCallback` 只接受**普通函数指针**或**静态函数**，不接受非静态成员函数。

非静态成员函数有隐藏参数 `this`，签名不匹配；静态成员函数属于类本身，没有 `this`，签名和普通函数一致。

### 6.2 `userdata` 传 `this`

```cpp
setMouseCallback("Detection Result", onMouse, this);
```

第三个参数是 `void*`，把当前对象的地址传进去。

### 6.3 `reinterpret_cast` 还原

```cpp
static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MultiModeVisionTracker* self =
        reinterpret_cast<MultiModeVisionTracker*>(userdata);
    self->mouseHandler(event, x, y, flags);
}

void mouseHandler(int event, int x, int y, int flags) {
    // 能访问所有成员变量，业务逻辑放这里
}
```

| 层次 | 职责 |
|------|------|
| 静态 `onMouse` | 拆箱，把 `void*` 还原成对象指针 |
| 成员 `mouseHandler` | 真正的状态机，读写成员变量 |

---

## 7. `main` 函数

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiModeVisionTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

| 部分 | 说明 |
|------|------|
| `rclcpp::init(argc, argv)` | 开机自检，扫描命令行里的 ROS2 专用参数 |
| `std::make_shared<...>()` | 在堆上创建节点，自动管理内存 |
| `rclcpp::spin(node)` | 阻塞主线程，进入事件循环 |
| `rclcpp::shutdown()` | 显式关闭 |

**`rclcpp::init` 会拦截**：

- `__ns:=/my_space` 命名空间
- `-p camera_topic:=/front_cam` 参数覆盖
- `--ros-args` 及其后续内容

剩下的参数才留给业务代码。

**`rclcpp::spin` 的行为**：调用后主线程卡在这一行，不再往下执行，直到 `rclcpp::shutdown()` 被触发。

---

## 8. 踩坑记录

### 8.1 头文件 `.h` vs `.hpp`

- **现象**：编译报找不到 `cv_bridge/cv_bridge.h`。
- **根因**：ROS2 jazzy 的 `cv_bridge` 头文件是 `cv_bridge.hpp`，早期版本是 `.h`。AI 生成的代码可能混用。
- **排查**：`dpkg -L ros-jazzy-cv-bridge | grep cv_bridge` 看实际文件名。
- **解决**：按实际安装版本改后缀。

### 8.2 `spin()` 与 `waitKey` 的关系

- **现象**：代码里 `waitKey(1)` 在 `imageCallback` 里，看起来像会阻塞 `spin()`。
- **分析**：`rclcpp::spin` 是单线程事件循环，`imageCallback` 执行期间不会处理其他回调。`waitKey(1)` 只阻塞 1ms，远小于每帧画面处理时间，实际影响可忽略。
- **但是**：后续高频控制应使用多线程执行器，把图像回调、控制回调、显示逻辑放到不同回调组或独立线程；或者把 UI 交互改成 Service/Action，`imageCallback` 只做数据处理。当前没有其他回调（定时器、service），所以直接 `waitKey` 没问题。
- **方向**：
  - 当前：`waitKey(1)` 只阻塞 1ms，远小于每帧处理时间，无其他回调时影响可忽略。
  - 长期：如果要做高频控制（>100Hz），应把显示逻辑从 `imageCallback` 剥离到独立的 `rclcpp::TimerBase` 或单独线程。
  - 更彻底：新建一个 ROS2 Service 或 Action 处理 UI 交互，`imageCallback` 只做数据处理。

### 8.3 静态回调里访问成员变量

- **现象**：直接把 `mouseHandler` 当回调注册给 `setMouseCallback`，编译报签名不匹配。
- **根因**：非静态成员函数有隐藏参数 `this`，函数指针类型和 OpenCV 要求的不一致。
- **解决**：静态 `onMouse` + `userdata` 传 `this` + `reinterpret_cast` 还原。见 6。

### 8.4 `declare_parameter` 后必须 `get_parameter`

- **现象**：改了 launch 文件里的参数值，程序里用的还是默认值。
- **根因**：只声明不读取，参数值不会自动同步到变量。
- **解决**：在构造函数里对每个参数调用 `get_parameter("name").as_xxx()` 显式取值。

---

## 9. 验证结果

### 9.1 节点启动

```bash
ros2 run <package> multi_mode_vision_tracker
ros2 node list
# 输出：/multi_mode_vision_tracker
```

### 9.2 话题查看

```bash
ros2 topic list
# /image_raw（订阅）
# /detection_result（发布）
# /mask（发布）

ros2 topic hz /detection_result
# 显示发布频率
```

### 9.3 参数查看

```bash
ros2 param list /multi_mode_vision_tracker
ros2 param get /multi_mode_vision_tracker camera_topic
ros2 param set /multi_mode_vision_tracker marker_size 0.05
```

### 9.4 功能验证

- 三种模式（1/2/3）切换正常。
- 颜色追踪：中心点坐标打印、轨迹绘制正常。
- ArUco：Z 轴距离打印正常。
- 鼠标框选标定：按 C 生效，ESC 取消。
- 按 Q 退出，`rclcpp::shutdown()` 被调用，程序正常结束。

### 9.5 坐标系约定与转换

可通过给位姿数据乘正交旋转矩阵，既然涉及计算那么旋转向量就需要用到罗德里格斯转化为旋转矩阵。

ROS2 标准是右手坐标系（REP-103，X 前 / Y 左 / Z 上）。图像坐标系是左上原点、Y 向下。

当前节点只发布处理后的图像，没有发布位姿或 `cmd_vel`，所以还没有涉及坐标变换,目前的视觉伺服也只做了角速度旋转，**未涉及坐标系变换**。图像坐标 → 底盘坐标的完整转换留待后续实现。

---

## 10. 待办 / 已知缺陷

- [ ] `waitKey` 仍在 `imageCallback` 里，严格说是同步阻塞
- [ ] 未使用 `rclcpp::TimerBase`，显示与业务逻辑耦合
- [ ] 未发布 `cmd_vel`，没有控制闭环
- [ ] 未做相机坐标系 → 底盘坐标系的转换
- [ ] 多模式节点还没拆分成独立节点

---

## 附：本篇涉及的关键 API

### ROS2 核心

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|---------|------|
| `rclcpp::init` | `void rclcpp::init(int argc, char const* const* argv);` | `rclcpp::init(argc, argv);` | 初始化 ROS2 |
| `rclcpp::Node` | `class Node : public std::enable_shared_from_this<Node>;` | `class X : public rclcpp::Node {}` | 节点基类 |
| `rclcpp::spin` | `void rclcpp::spin(typename NodeT::SharedPtr node);` | `rclcpp::spin(node);` | 阻塞事件循环 |
| `rclcpp::shutdown` | `void rclcpp::shutdown();` | `rclcpp::shutdown();` | 全局关闭 |
| `Node::get_logger` | `rclcpp::Logger get_logger() const;` | `this->get_logger()` | 获取日志记录器 |
| `RCLCPP_INFO` | 宏 | `RCLCPP_INFO(logger, "fmt", ...);` | 信息日志 |
| `RCLCPP_WARN` | 宏 | `RCLCPP_WARN(logger, "fmt", ...);` | 警告日志 |
| `RCLCPP_ERROR` | 宏 | `RCLCPP_ERROR(logger, "fmt", ...);` | 错误日志 |

### 通信

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|---------|------|
| `Node::create_subscription` | `template<typename MessageT, typename CallbackT> typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(const std::string& topic_name, const rclcpp::QoS& qos, CallbackT&& callback);` | `create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(...));` | 创建订阅 |
| `Node::create_publisher` | `template<typename MessageT> typename rclcpp::Publisher<MessageT>::SharedPtr create_publisher(const std::string& topic_name, const rclcpp::QoS& qos);` | `create_publisher<sensor_msgs::msg::Image>("/mask", 10);` | 创建发布者 |
| `Publisher::publish` | `void publish(const MessageT& msg);` | `result_pub_->publish(*out_msg);` | 发布消息 |

### 参数

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|---------|------|
| `Node::declare_parameter` | `template<typename ParameterT> auto declare_parameter(const std::string& name, const ParameterT& default_value, ...);` | `declare_parameter<string>("camera_topic", "/image_raw");` | 声明参数 |
| `Node::get_parameter` | `rclcpp::Parameter get_parameter(const std::string& name) const;` | `get_parameter("calibration_file");` | 获取参数对象 |
| `Parameter::as_string` | `std::string as_string() const;` | `.as_string()` | 按字符串解包 |
| `Parameter::as_double` | `double as_double() const;` | `.as_double()` | 按 double 解包 |

### cv_bridge

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|---------|------|
| `cv_bridge::toCvCopy` | `CvImagePtr toCvCopy(const sensor_msgs::msg::Image& source, const std::string& encoding);` | `toCvCopy(msg, BGR8);` | 深拷贝转换 |
| `cv_bridge::CvImage` | `CvImage(const std_msgs::msg::Header& header, const std::string& encoding, const cv::Mat& image);` | `CvImage(msg->header, "bgr8", frame);` | 构造桥接对象 |
| `CvImage::toImageMsg` | `sensor_msgs::msg::Image::SharedPtr toImageMsg() const;` | `.toImageMsg();` | 装箱为 ROS2 消息 |

### 智能指针与函数绑定

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|---------|------|
| `std::make_shared` | `template<class T, class... Args> shared_ptr<T> make_shared(Args&&... args);` | `std::make_shared<MultiModeVisionTracker>();` | 创建共享指针 |
| `std::bind` | `template<class F, class... Args> bind(F&& f, Args&&... args);` | `std::bind(&X::cb, this, _1);` | 绑定函数和参数 |
| `std::placeholders::_1` | 占位符 | `std::placeholders::_1` | 代表第一个参数位置 |
| `reinterpret_cast` | `reinterpret_cast<T*>(expr)` | `reinterpret_cast<X*>(userdata);` | 从 `void*` 还原对象指针 |

### 与前面几篇共用的部分

`Mat`、`Scalar`、`Point`、`Rect`、`cvtColor`、`inRange`、`erode`、`dilate`、`findContours`、`boundingRect`、`rectangle`、`drawMarker`、`line`、`putText`、`meanStdDev`、`setMouseCallback`、`FileStorage`、`solvePnP`、`drawFrameAxes`、`undistort` 等已在前面各篇的 API 表中列出，这里不重复。

---

## 与前面几篇的差异对照

| 项目 | hsv_basic | trajectory | mouse_roi | camera_cal | aruco_pose | ros2_node |
|------|-----------|------------|-----------|------------|------------|-----------|
| 程序形态 | 单文件 | 单文件 | 单文件 | 单文件 | 单文件 | ROS2 节点 |
| 数据来源 | `VideoCapture` | 同左 | 同左 | 同左 | 同左 | 订阅话题 |
| 数据出口 | 窗口 | 窗口 | 窗口 | YAML | 窗口 | 话题 + 窗口 |
| 参数方式 | 硬编码 | 硬编码 | 全局变量 | 常量 | 常量 | `declare_parameter` |
| 鼠标回调 | 无 | 无 | 普通函数 | 无 | 无 | 静态 + `this` |
| 新增关键 API | `inRange` | `deque` | `setMouseCallback` | `FileStorage` | `solvePnP` | `create_subscription` / `create_publisher` |

---

## 衔接说明

### 上游依赖

本篇整合了前面所有单文件程序：

- 颜色追踪逻辑来自 [hsv_basic.md](hsv_basic.md)
- 轨迹绘制来自 [trajectory.md](trajectory.md)
- 鼠标框选标定来自 [mouse_roi_calibration.md](mouse_roi_calibration.md)
- `calibration.yaml` 来自 [camera_calibration.md](camera_calibration.md)
- ArUco 位姿估计来自 [aruco_pose.md](aruco_pose.md)

### 下游应用

当前节点只发布处理后的图像（`/detection_result`、`/mask`），还没有：

- 发布 `cmd_vel` 实现控制闭环
- 发布位姿话题（`geometry_msgs/Pose`）
- 拆分多节点架构

后续如果要做视觉伺服，需要：

1. 从颜色中心或 ArUco 位姿算出误差
2. PID 或 P 控制生成 `cmd_vel`
3. 通过 `create_publisher<geometry_msgs::msg::Twist>` 发布

这些属于控制层，不在本篇范围。
