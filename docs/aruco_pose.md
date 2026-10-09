# ArUco 检测与位姿估计

> 目标：加载相机标定文件，检测 ArUco 标记，用 `solvePnP` 估计标记在相机坐标系下的三维位姿，绘制坐标轴并输出 Z 轴距离。  
> 环境：Ubuntu 24.04 + OpenCV 4.8 + C++  
> 相关：依赖 [camera_calibration.md](camera_calibration.md) 输出的 `calibration.yaml`；ROS2 版本见 [ros2_node.md](ros2_node.md)。

---

## 1. 核心原理

### 1.1 ArUco 是什么

ArUco 是一种**方形二值编码标记**：黑色边框 + 内部黑白格子构成的唯一 ID 图案。相比纯色物体或棋盘格，它有两大优势：

| 优势 | 说明 |
|------|------|
| 可识别身份 | 每个标记有唯一 ID，同屏多个标记互不干扰 |
| 可估计位姿 | 方形四角点 + 已知边长 → PnP 求解 3D 位姿 |

相比棋盘格，ArUco 不需要完整可见（遮挡一部分仍能识别），适合实际场景。

### 1.2 位姿估计要解决什么问题

给定：

- **3D 点**：标记的四个角点在**标记坐标系**下的坐标，已知（边长确定）
- **2D 点**：这四个角点在**图像**中的像素坐标，检测得到
- **相机内参 + 畸变系数**：由 [camera_calibration.md](camera_calibration.md) 提供

求：

- **`rvec`**：标记坐标系 → 相机坐标系的旋转向量（3×1）
- **`tvec`**：标记坐标系 → 相机坐标系的平移向量（3×1）

\[
P_{cam} = R \cdot P_{obj} + t
\]

其中：

- `rvec` 经 `cv::Rodrigues` 得到 `R`
- `tvec` 是 `t`
- `P_obj` 是物体坐标系下的点，比如 ArUco 码角点
- `P_cam` 是相机坐标系下的点

这本质是 **Perspective-n-Point（PnP）** 问题：已知 n 个 3D-2D 对应点和相机内参，求物体/ArUco 码在相机坐标系下的位姿,物体坐标系到相机坐标系的旋转和平移。

### 1.3 旋转向量的物理意义

`rvec` 是 3×1 向量，不是直观的三个欧拉角。它的**方向**是旋转轴，**模长**是旋转角度（弧度）。这种表示叫**轴角表示**。

罗德里格斯公式（Rodrigues）负责两种表示之间的无损转换：

| 转换方向 | 函数调用 |
|---------|---------|
| 旋转向量 → 旋转矩阵 | `Rodrigues(rvec, rotMat);` |
| 旋转矩阵 → 旋转向量 | `Rodrigues(rotMat, rvec);` |

**是否需要手动调用？**

| 场景 | 是否需要 |
|------|---------|
| 只画坐标轴（`drawFrameAxes`） | 不需要，函数内部自动转换 |
| 需要基于旋转矩阵做变换 | 需要，如把相机坐标系下的距离换算到机器人底盘坐标系 |

> 例：要把标记位姿从相机坐标系转到机器人底盘坐标系，先 `Rodrigues(rvec, R)` 构造 \(T_{cam\leftarrow marker}\)，再左乘相机到底盘的外参 \(T_{base\leftarrow cam}\)，得到 \(T_{base\leftarrow marker}\)。

### 1.4 处理流程

```
加载 calibration.yaml（相机内参 + 畸变系数）
    ↓
读取帧
    ↓
detectMarkers 检测 ArUco 标记，得到 corners（4 个角点）和 ids
    ↓
对每个标记：solvePnP(objectPoints, corners[i], ...) 求 rvec / tvec
    ↓
drawFrameAxes 绘制三维坐标轴
    ↓
打印 tvec[2]（Z 轴距离，单位：米）
    ↓
显示窗口 + ESC 退出
```

---

## 2. 关键代码

### 2.1 加载相机标定文件

```cpp
Mat cameraMatrix, distCoeffs;
FileStorage fs(CALIB_FILE, FileStorage::READ);
if (!fs.isOpened()) {
    cerr << "错误：无法打开标定文件 " << CALIB_FILE << endl;
    return -1;
}
fs["camera_matrix"] >> cameraMatrix;
fs["distortion_coefficients"] >> distCoeffs;

if (cameraMatrix.empty() || distCoeffs.empty()) {
    cerr << "错误：标定文件缺少 camera_matrix 或 distortion_coefficients" << endl;
    return -1;
}
```

**两个必须检查**：

1. `fs.isOpened()` — 文件能不能打开
2. `cameraMatrix.empty()` — 打开后有没有内容

只检查第一个不够，标定文件格式不对时，`>>` 会静默失败，`cameraMatrix` 仍为空。

### 2.2 创建 ArUco 字典

```cpp
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
```

- `DICT_6X6_250`：每个标记内部是 6×6 的格子，共 250 个不同 ID。
- 其它常用字典：`DICT_4X4_50`（小尺寸、少数量）、`DICT_5X5_100`、`DICT_7X7_1000`。
- **字典必须和打印出来的标记匹配**，否则一个都检测不到。

### 2.3 检测标记

```cpp
vector<int> ids;
vector<vector<Point2f>> corners;
aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams);
```

| 参数 | 说明 |
|------|------|
| `frame` | 输入图像（BGR） |
| `dictionary` | 使用的字典 |
| `corners` | 输出：每个标记的 4 个角点，顺序固定为左上、右上、右下、左下 |
| `ids` | 输出：每个标记的 ID |
| `detectorParams` | 检测参数（默认即可） |

**可选参数**：

- 后面可以追加 `rejectedCandidates`（检测失败的候选轮廓）、`cameraMatrix`、`distCoeffs`（内部先正畸再检测）。
- 检测失败的候选轮廓可导出用其他颜色标记方便调试

### 2.4 定义标记的 3D 角点

```cpp
vector<Point3f> objectPoints;
float halfSize = MARKER_SIZE / 2.0f;
objectPoints.push_back(Point3f(-halfSize,  halfSize, 0)); // 左上
objectPoints.push_back(Point3f( halfSize,  halfSize, 0)); // 右上
objectPoints.push_back(Point3f( halfSize, -halfSize, 0)); // 右下
objectPoints.push_back(Point3f(-halfSize, -halfSize, 0)); // 左下
```

**关键点**：

- 坐标系以标记中心为原点
- X 轴向右，Y 轴向上，Z 轴垂直标记平面向外（右手坐标系）
- 顺序必须和 `detectMarkers` 输出的 `corners[i]` 一致：**左上开始顺时针**

> **踩坑**：如果按像素坐标系的“Y 向下”来定义（即左上 `(-h, -h, 0)`），输出数据会严重失真，坐标轴乱飞、位姿剧烈跳动，甚至出现满屏飞线。

### 2.5 求解位姿

```cpp
Mat rvec, tvec;
bool success = solvePnP(objectPoints, corners[i],
                        cameraMatrix, distCoeffs,
                        rvec, tvec,
                        false,
                        SOLVEPNP_IPPE_SQUARE);
```

| 参数 | 说明 |
|------|------|
| `objectPoints` | 标记的 3D 角点（固定，不随检测变） |
| `corners[i]` | 当前标记的 2D 角点 |
| `cameraMatrix` / `distCoeffs` | 相机内参和畸变 |
| `rvec` / `tvec` | 输出：旋转向量、平移向量 |
| `false` | 不使用初始估计 |
| `SOLVEPNP_IPPE_SQUARE` | 专门针对方形平面标记的算法，比通用 PnP 更稳更快 |

**为什么不用 `estimatePoseSingleMarkers`**：该函数在新版 OpenCV 中已弃用，虽然还能跑，但 `solvePnP` 更通用、更明确。

### 2.6 绘制坐标轴

```cpp
drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, MARKER_SIZE * 1.5f);
```

最后一个参数是坐标轴长度，这里取标记边长的 1.5 倍。轴长可以按数学公式换算，让轴在画面中呈现固定像素长度，但没必要——对可视化来说，和标记尺寸成比例更直观。

### 2.7 读取 Z 轴距离

```cpp
double z_distance = tvec[2];     // 也可写 tvec.at<double>(2) 或 tvec.at<double>(2, 0)
double z_cm = z_distance * 100.0;
cout << "标记 ID " << ids[i] << " 的 Z 轴距离: " << z_cm << " cm" << endl;
```

`tvec` 在 OpenCV 中是 3×1 的 `Mat`，用 `tvec[2]` 取第三个元素（Z 分量）。

> 若不确定维度，先 `tvec = tvec.reshape(1, 3);` 统一成 3×1，再用 `tvec[2]` 或 `tvec.at<double>(2,0)`。

**Mat 访问的两种写法**：

| 写法 | 说明 |
|------|------|
| `tvec[2]` | 一维索引，简写 |
| `tvec.at<double>(2, 0)` | 双参数，行、列索引，更明确 |

### 2.8 Vec3d vs Mat 存 rvec / tvec

两种写法：

```cpp
vector<Mat>  rvecs, tvecs;    // 写法一：Mat
vector<Vec3d> rvecs, tvecs;   // 写法二：Vec3d
```

| 维度 | `Mat` | `Vec3d` |
|------|-------|---------|
| 内存分配 | 每帧动态分配 | 栈上，固定大小 |
| 访问速度 | 慢 | 快 |
| 语义清晰度 | 弱，需看维度 | 强，一眼知道是三维 |
| 数学运算 | 支持矩阵运算 | 支持向量运算 |
| 适用场景 | 通用，需要灵活维度 | 固定 3D 向量，性能敏感 |

处理视频流每帧调用时，**推荐 `Vec3d`**，性能更高。而且 `rvec` / `tvec` 永远是 3 维，不需要 `Mat` 的灵活性。符合以下条件的 `Mat` 有隐式转换运算符可通过下面方法转存 `Vec3d`。（这里不详细展开）

> `Mat` 对尺寸为 3×1 或 1×3、类型为 `double` 的矩阵有到 `Vec3d` 的隐式转换，
> 所以 `rvecs.push_back(rvec)` 能直接把 `solvePnP` 的输出存进 `vector<Vec3d>`。

```cpp
vector<Vec3d> rvecs, tvecs;
rvecs.push_back(rvec);   // Mat -> Vec3d
tvecs.push_back(tvec);
```



---

## 3. 参数与调参

### 3.1 `MARKER_SIZE` 必须是实际物理尺寸

```cpp
const float MARKER_SIZE = 0.03f;   // 3 cm
```

- 单位是**米**，不是像素。
- **必须精确测量打印出的标记边长**。写 3 cm 但实际是 5 cm，测出来的距离会按线性比例有偏差。
- 建议打印后立刻用尺子量，写进代码。

### 3.2 字典怎么选

| 字典 | 内部格子 | ID 数量 | 适用场景 |
|------|---------|--------|---------|
| `DICT_4X4_50` | 4×4 | 50 | 小尺寸、少量标记 |
| `DICT_5X5_100` | 5×5 | 100 | 中等 |
| `DICT_6X6_250` | 6×6 | 250 | 本项目使用，通用 |
| `DICT_7X7_1000` | 7×7 | 1000 | 大尺寸、多标记 |

**原则**：格子越少、ID 越少，越容易检测（鲁棒），但越容易混淆；反之则反。一般项目用 `DICT_6X6_250` 足够。

---

## 4. 踩坑记录

### 4.1 坐标系定义反了导致数据完全错误

- **现象**：3D 角点写成像素坐标系下“左上 `(-h, -h, 0)`”（按像素坐标系 Y 向下），输出 `tvec` 完全错乱，距离毫无意义。
- **根因**：像素坐标和 3D 坐标是**两套坐标系**。`objectPoints` 用的是**右手系**（X 右、Y 上、Z 出平面），不是像素坐标的 Y 向下。
- **解决**：严格按照“左上 → 右上 → 右下 → 左下”顺时针，Y 轴向上为正的坐标系。



### 4.2 标定文件读不到

- **现象**：`fs.isOpened()` 返回 false。
- **根因**：`FileStorage` 用相对路径，从不同工作目录运行找不到 `calibration.yaml`。
- **解决**：
  - 把 `calibration.yaml` 放在运行目录下；
  - 或改成绝对路径；
  - 或加命令行参数指定路径。

### 4.3 标定文件格式不全

- **现象**：`fs.isOpened()` 通过，但 `cameraMatrix.empty()` 为 true。
- **根因**：标定文件缺 `camera_matrix` 或 `distortion_coefficients` 字段，`>>` 静默失败。
- **解决**：**必须双重检查**：`isOpened` + `empty`。

### 4.4 检测不到标记

- **现象**：画面里明明有 ArUco，但一个都检测不到。
- **排查方向**：
  1. **字典不匹配**：打印的标记和代码里的 `DICT_*` 不一致 → 换字典。
  2. **光照太暗或反光**：标记的黑白对比度不够 → 换位置或补光。
  3. **标记太小**：在画面中占的像素太少 → 靠近摄像头，或打印更大。
  4. **标记太斜**：角度过大导致角点识别失败 → 调整姿态。


---

## 5. 验证结果

### 5.1 距离精度

- 标记放在**距摄像头 30 cm** 处，程序输出的 Z 轴距离与真实值一致，误差在可接受范围。
- 精度得益于标定质量：标定文件中的 RMS 见 [camera_calibration.md](camera_calibration.md)
- 标定差 → 测距差；测距准 → 说明标定准，**两者相互验证**。

### 5.2 坐标轴可视化

- `drawFrameAxes` 画出三色坐标轴：红（X）、绿（Y）、蓝（Z）。
- Z 轴指向标记平面外，指向相机为正方向。
- 标记旋转时，坐标轴跟随旋转，直观反映标记姿态。

### 5.3 多标记支持

- 同一画面有多个 ArUco 时，`ids` 和 `corners` 会包含所有标记，逐个 `solvePnP`。
- 每个标记独立估计位姿，互不干扰。

---

## 6. 待办 / 已知缺陷

- [ ] 未实现标记坐标系 → 机器人底盘坐标系的转换，只做到了相机坐标系下的距离输出。
- [ ] 角度信息（法向量方向）未利用，可在 `drawFrameAxes` 后加判断，实现“只在特定角度抓取”。
- [ ] 检测参数未调优，极端光照或大角度下可能丢标记。
- [ ] 未做标记遮挡时的鲁棒性测试（部分遮挡时能否继续估计位姿）。

---

## 附：本篇涉及的关键 API

### ArUco 模块

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|-------------|------|
| `getPredefinedDictionary` | `Ptr<Dictionary> getPredefinedDictionary(int dict);` | `getPredefinedDictionary(aruco::DICT_6X6_250)` | 创建预定义字典 |
| `DetectorParameters::create` | `static Ptr<DetectorParameters> create();` | `aruco::DetectorParameters::create()` | 创建默认检测参数 |
| `detectMarkers` | `void detectMarkers(InputArray image, const Ptr<Dictionary>& dictionary, OutputArrayOfArrays corners, OutputArray ids, const Ptr<DetectorParameters>& params = DetectorParameters::create(), OutputArrayOfArrays rejectedImgPoints = noArray(), InputArray cameraMatrix = noArray(), InputArray distCoeff = noArray());` | `detectMarkers(frame, dictionary, corners, ids, detectorParams);` | 检测 ArUco 标记，可选传入相机参数在内部先正畸 |
| `drawDetectedMarkers` | `void drawDetectedMarkers(InputOutputArray image, InputArrayOfArrays corners, InputArray ids = noArray(), Scalar borderColor = Scalar(0,255,0), Scalar textColor = Scalar(0,0,255));` | `drawDetectedMarkers(frame, corners, ids);` | 绘制检测结果 |

### 位姿估计

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|-------------|------|
| `solvePnP` | `bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE);` | `solvePnP(objectPoints, corners[i], cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);` | 求解 PnP 位姿 |
| `drawFrameAxes` | `void drawFrameAxes(InputOutputArray image, InputArray cameraMatrix, InputArray distCoeffs, InputArray rvec, InputArray tvec, float length, int thickness = 3);` | `drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, MARKER_SIZE * 1.5f);` | 绘制三维坐标轴 |
| `Rodrigues` | `void Rodrigues(InputArray src, OutputArray dst, OutputArray jacobian = noArray());` | 本篇未显式调用 | 旋转向量 ↔ 旋转矩阵互转 |

### 智能指针

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|-------------|------|
| `Ptr<T>` | `template<class T> class Ptr;` | `Ptr<aruco::Dictionary> dict;` | OpenCV 智能指针，自动 delete |
| `Ptr<T>::create` | `static Ptr<T> create(...);` | `aruco::DetectorParameters::create()` | 工厂方法，创建对象 |
| `Ptr<T>::operator->` | `T* operator->();` | `detectorParams->adaptiveThreshWinSizeMin = 5;` | 通过智能指针访问对象成员（示例的用法为 `DetectorParameters` 默认参数不够，检测不稳时调整参数 |


### 数据结构

| API | 准确签名 / 调用 | 用法举例 | 说明 |
|-----|----------------|-------------|------|
| `Vec3d` | `Vec3d(double v0, double v1, double v2);` | `vector<Vec3d> rvecs, tvecs;` | 三维 double 向量 |
| `Mat::at<T>` | `template<typename T> T& at(int i0, int i1);` | `tvec.at<double>(2, 0)` | 矩阵元素访问，双参数（行、列） |
| `Mat::operator[]` | `Mat operator[](int row) const;` | `tvec[2]` | 返回第 row 行（0-indexed）。`solvePnP` 输出通常是 3×1，`tvec[2]` 合法；若是 1×3，先 `tvec = tvec.reshape(1, 3)` 再索引  |

### 与前面几篇共用的部分

`FileStorage`、`VideoCapture`、`namedWindow`、`imshow`、`waitKey`、`destroyAllWindows`、`Mat`、`Point3f`、`Point2f` 等已在 [camera_calibration.md](camera_calibration.md) 和 [hsv_basic.md](hsv_basic.md) 的 API 表中列出，这里不重复。

---

## 与前面几篇的差异对照

| 项目 | hsv_basic.md | mouse_roi_calibration.md | camera_calibration.md | aruco_pose.md |
|------|-------------|--------------------------|-----------------------|---------------|
| 核心目标 | 颜色追踪 | 鼠标框选标定 | 相机内参标定 | ArUco 位姿估计 |
| 输入 | 视频流 | 视频流 | 视频流 | 视频流 + `calibration.yaml` |
| 输出 | 目标中心 | HSV 阈值 | `calibration.yaml` | `rvec` / `tvec` + Z 轴距离 |
| 关键新 API | `inRange` / `findContours` | `setMouseCallback` / `meanStdDev` | `calibrateCamera` / `FileStorage` | `detectMarkers` / `solvePnP` / `drawFrameAxes` |
| 依赖 | 无 | hsv_basic + trajectory | 无 | camera_calibration |

---

## 衔接说明

### 上游依赖

本篇的输入 `calibration.yaml` 来自 [camera_calibration.md](camera_calibration.md)。标定质量直接决定测距精度：

- RMS 越小 → 内参和畸变越准 → `solvePnP` 越准 → Z 轴距离越准。
- 本篇的“30 cm 处测距准确”，就是标定合格的间接验证。

### 下游应用

本篇输出 `tvec`（相机坐标系下的平移）和 `rvec`（姿态）可用于：

1. **抓取**：法向量方向 → 判断标记面朝向 → 决定机械臂接近方向。
2. **导航**：Z 轴距离 → 决定小车前进后退（见 [visual_servo_p.md](visual_servo_p.md)）。
3. **多坐标系融合**：配合 [kalman_filter.md](kalman_filter.md) 做位姿平滑。

**当前局限**：只输出相机坐标系下的距离，未做相机 → 底盘的坐标变换。这属于“感知”层面，真正闭环还需要 [visual_servo_p.md](visual_servo_p.md) 的控制层。