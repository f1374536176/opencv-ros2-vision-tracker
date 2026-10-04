# 相机内参与畸变系数标定

> 目标：用棋盘格拍摄多角度图像，计算相机内参矩阵和畸变系数，输出 `calibration.yaml` 供后续去畸变和位姿估计使用。  
> 环境：Ubuntu 24.04 + OpenCV 4.8 + C++  
> 相关：ArUco 位姿估计依赖本篇结果，见 [aruco_pose.md](aruco_pose.md)。

---

## 1. 核心原理

### 1.1 为什么要做相机标定

相机的成像过程不是理想的针孔模型。真实镜头会带来两类偏差：

| 偏差 | 表现 | 参数 |
|------|------|------|
| 内参误差 | 焦距、主点位置和理论值不一致 | `camera_matrix`（3×3） |
| 畸变 | 直线变弯，画面边缘拉伸 | `distortion_coefficients`（1×5） |

去畸变和位姿估计都需要这两个参数。不标定直接测距，误差可能有几厘米到几十厘米。

### 1.2 内参矩阵

```
[ fx   0   cx ]
[  0  fy   cy ]
[  0   0    1 ]
```

- `fx`、`fy`：焦距（像素单位），通常接近但不相等。
- `cx`、`cy`：主点（光轴和成像平面的交点），接近图像中心但不完全在中心。

### 1.3 畸变系数

OpenCV 默认 5 个系数，顺序是：

```
[k1, k2, p1, p2, k3]
```

| 参数 | 名称 | 影响 |
|------|------|------|
| k1, k2, k3 | 径向畸变 | 直线变成桶形或枕形 |
| p1, p2 | 切向畸变 | 镜头和成像平面不平行导致 |

### 1.4 标定流程

```
拍摄多张棋盘格图像（≥10 张，不同角度、距离）
    ↓
对每张图检测棋盘格内角点
    ↓
亚像素精确化，得到精准的 2D 像素坐标
    ↓
配对世界坐标（3D）和图像坐标（2D）
    ↓
calibrateCamera 求解内参和畸变
    ↓
输出 RMS（重投影误差）作为质量指标
    ↓
保存到 calibration.yaml
```

---

## 2. 关键代码

### 2.1 定义棋盘格尺寸和世界坐标

```cpp
Size boardSize(9, 6);   // 内角点列数 × 行数，必须和实际棋盘格匹配
vector<vector<Point3f>> objectPoints;   // 每张图对应的世界坐标（3D）
vector<vector<Point2f>> imagePoints;    // 每张图对应的图像坐标（2D）
vector<Point3f> objp;

// 世界坐标：每个格子边长 1 单位，Z=0（棋盘格是平面）
for (int r = 0; r < boardSize.height; r++) {
    for (int c = 0; c < boardSize.width; c++) {
        objp.push_back(Point3f(c, r, 0));
    }
}
```

**关键**：`boardSize` 是**内角点数**，不是格子数。一张 10×7 的棋盘格有 9×6 个内角点。

### 2.2 棋盘格检测

```cpp
cvtColor(frame, gray, COLOR_BGR2GRAY);
vector<Point2f> corners;
bool found = findChessboardCorners(
    gray, boardSize, corners,
    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
```

- 输入必须是**灰度图**。
- `CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE` 是固定搭配：自适应阈值 + 图像归一化，提升不同光照下的检测稳定性。
- 返回值 `bool`，检测到返回 `true`。

### 2.3 亚像素精确化

```cpp
cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
             TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 搜索窗口 | `Size(11, 11)` | 在角点周围 11×11 像素内精确搜索，死经验值 |
| 死区 | `Size(-1, -1)` | 不用死区，固定套路 |
| 终止条件 | `TermCriteria(EPS + MAX_ITER, 30, 0.001)` | 迭代 30 次或精度 0.001，满足其一即停 |

`findChessboardCorners` 给出的是粗略像素位置，`cornerSubPix` 把它精确到亚像素级。这一步直接影响 RMS。

### 2.4 可视化与采集

```cpp
drawChessboardCorners(frame, boardSize, corners, found);
putText(frame, "Detected! Press SPACE to capture.",
        Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
```

`drawChessboardCorners` 的 `found` 参数决定显示方式：

- `true`：彩色相连的棋盘格角点
- `false`：凌乱散点（未检测到）

按空格键保存当前帧：

```cpp
if (key == ' ') {
    if (found) {
        objectPoints.push_back(objp);
        imagePoints.push_back(corners);
        captured++;
    }
}
```

### 2.5 核心标定

```cpp
Mat cameraMatrix, distCoeffs, R, T;
double rms = calibrateCamera(
    objectPoints, imagePoints,
    Size(frame.cols, frame.rows),
    cameraMatrix, distCoeffs, R, T);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `objectPoints` | `vector<vector<Point3f>>` | 每张图的世界坐标 |
| `imagePoints` | `vector<vector<Point2f>>` | 每张图的图像坐标 |
| `imageSize` | `Size` | 图像尺寸 |
| `cameraMatrix` | `Mat&` | 输出：内参矩阵 3×3 |
| `distCoeffs` | `Mat&` | 输出：畸变系数 1×5 |
| `R` / `T` | `Mat&` | 输出：每张图的旋转和平移（此处不关注） |
| 返回值 | `double` | **重投影误差 RMS**，越小越好 |

### 2.6 数据持久化：`FileStorage`

`FileStorage` 是 OpenCV 的 YAML/XML/JSON 读写类。

**写入：**

```cpp
FileStorage fs("calibration.yaml", FileStorage::WRITE);
if (fs.isOpened()) {
    fs << "image_width" << frame.cols;
    fs << "image_height" << frame.rows;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "reprojection_error" << rms;
    fs.release();   // 必须调用，否则数据可能滞留在缓冲区
}
```

**读取（在 ArUco 篇用到）：**

```cpp
FileStorage fs("calibration.yaml", FileStorage::READ);
if (!fs.isOpened()) { /* 错误处理 */ }
fs["camera_matrix"] >> cameraMatrix;
fs["distortion_coefficients"] >> distCoeffs;
fs.release();
```

**关键点：**

- 构造参数 2：`READ` / `WRITE` / `APPEND`
- **必须检查 `isOpened()`**，否则读写会静默失败
- **写完后必须 `release()`**，把缓冲刷到硬盘

---

## 3. 参数与调参

### 3.1 拍摄多少张

- **最低 10 张**，程序里设了硬门槛。
- 推荐 15–20 张，覆盖不同角度、距离、位置。
- 每张图的棋盘格应**占据画面 1/3 以上**，太小角点检测精度低。

### 3.2 角度和距离怎么变

| 变化 | 覆盖的畸变模型 |
|------|---------------|
| 倾斜（旋转棋盘格） | 切向畸变 p1、p2 |
| 远近变化 | 焦距 fx、fy |
| 上下左右移动 | 主点 cx、cy |
| 边缘区域拍摄 | 径向畸变 k1、k2、k3 |

只在画面中央拍，无法标定边缘畸变。


---

## 4. 踩坑记录

### 4.1 `boardSize` 和实际棋盘格不匹配

- **现象**：一直检测不到棋盘格。
- **根因**：`boardSize(9, 6)` 是**内角点数**，不是格子数。一张 10×7 的棋盘格内角点是 9×6。
- **解决**：数清楚棋盘格的内角点数，改 `boardSize`。

### 4.2 照片数量不足 10 张就按 Q

- **现象**：程序报 `错误：有效照片不足 10 张，无法标定！`
- **解决**：拍到 10 张以上再按 Q。程序里已加拦截逻辑，不足 10 张会提示继续拍。

### 4.3 RMS 未写入 YAML

- **现象**：标定算出的 RMS 只在终端打印，没有保存。
- **解决**：加一行 `fs << "reprojection_error" << rms;`。后续如果标定质量有问题，可以回头查 RMS。

### 4.4 潜在风险：背景条纹被误判为棋盘格

- **现象**：背景有规则条纹时 `findChessboardCorners` 可能误判。
- **规避**：
  - 换到干净背景拍摄。
  - 用非条纹图案的棋盘格（比如传统黑白方格）。
  - 轻微遮挡破坏背景纹理的完整性。
- 本次实测背景无规则条纹，未遇到误判。

### 4.5 标定文件路径问题

- **现象**：主程序读不到 `calibration.yaml`。
- **根因**：`FileStorage` 用的是相对路径，当前工作目录不对就找不到。
- **解决**：用绝对路径，或者确保程序在 `calibration.yaml` 所在目录下运行。



---

## 5. 验证结果

### 5.1 标定质量


| RMS | 评价 |
|------|------|
| < 0.5 | 优秀 |
| 0.5 – 1.0 | 合格 |
| > 1.0 | 重拍，检查是否模糊、角度变化不足 |

本次 0.459，属于优秀范围。

### 5.2 输出文件

```yaml
%YAML:1.0
image_width: 640
image_height: 480
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 6.1217971024432006e+02, 0., 3.0803473043006500e+02,
           0., 6.1293056258999820e+02, 2.2167676340783603e+02,
           0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -1.7824879614286063e-01, 6.5307199311175823e-01,
           3.5833382132817986e-03, -6.3120531215987350e-04,
           -9.4975959189127712e-01 ]
reprojection_error: 4.5913556519325854e-01
```

**内参解读**：

- `fx ≈ 612.18`，`fy ≈ 612.93`，两者接近，符合预期。
- `cx ≈ 308.03`，`cy ≈ 221.68`，接近图像中心 (320, 240)，合理。

**畸变解读**：

- `k1 = -0.178`（负值 → 桶形畸变）
- `k2 = 0.653`（正值，补偿 k1）
- `p1 = 0.0036`，`p2 = -0.00063`（切向畸变，很小）
- `k3 = -0.950`（高阶补偿）

### 5.3 去畸变效果

在后续程序里用 `undistort(frame, frame_undistorted, cameraMatrix, distCoeffs)` 处理，画面边缘的直线明显变直。
```cpp
Mat frame_undistorted;
undistort(frame, frame_undistorted, cameraMatrix, distCoeffs);
imshow("Undistorted", frame_undistorted);
```

---

## 6. 待办 / 已知缺陷

- [ ] `R` 和 `T` 目前被丢弃，实际它们记录了每张图的位姿，可以用于验证标定质量。
- [ ] 标定结果未做交叉验证（用不同批次图片重标，看参数一致性）。
- [ ] 未测试标定后去畸变对测距精度的影响。
- [ ] 棋盘格图案可以换成 ChArUco（棋盘格 + ArUco），鲁棒性更强。

---

## 附：本篇涉及的关键 API

### 相机标定

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `findChessboardCorners` | `bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags = 0);` | `findChessboardCorners(gray, boardSize, corners, ...)` | 检测棋盘格内角点，返回是否找到 |
| `cornerSubPix` | `void cornerSubPix(InputArray image, InputOutputArray corners, Size winSize, Size zeroZone, TermCriteria criteria);` | `cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), ...)` | 亚像素精确化 |
| `drawChessboardCorners` | `void drawChessboardCorners(InputOutputArray image, Size patternSize, InputArray corners, bool patternWasFound);` | `drawChessboardCorners(frame, boardSize, corners, found);` | 可视化角点 |
| `calibrateCamera` | `double calibrateCamera(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, Size imageSize, InputOutputArray cameraMatrix, InputOutputArray distCoeffs, OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs, int flags = 0, TermCriteria criteria = ...);` | `calibrateCamera(objectPoints, imagePoints, Size(frame.cols, frame.rows), cameraMatrix, distCoeffs, R, T);` | 核心标定函数，返回 RMS |
| `TermCriteria` | `TermCriteria(int type, int maxCount, double epsilon);` | `TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001)` | 迭代终止条件 |

### 数据持久化

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `FileStorage` | `FileStorage(const String& filename, int flags, const String& encoding = String());` | `FileStorage fs("calibration.yaml", FileStorage::WRITE);` | 文件读写类 |
| `FileStorage::isOpened` | `bool isOpened() const;` | `fs.isOpened()` | 检查文件是否成功打开 |
| `FileStorage::release` | `void release();` | `fs.release();` | 关闭文件，写入后必须调用 |
| `FileStorage::operator<<` | `FileStorage& operator<<(const String& name, const Mat& val);` | `fs << "camera_matrix" << cameraMatrix;` | 写入数据 |
| `FileStorage::operator[]` | `FileNode operator[](const String& nodename) const;` | `fs["camera_matrix"] >> cameraMatrix;` | 读取节点，配合 `>>` 使用 |

### 图像处理与绘制

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `cvtColor` | 见 [hsv_basic.md](hsv_basic.md) | `cvtColor(frame, gray, COLOR_BGR2GRAY);` | 本篇用彩色转灰度 |
| `putText` | 见 [mouse_roi_calibration.md](mouse_roi_calibration.md) | `putText(frame, "...", ...);` | 画面写字 |

### 数据结构

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `Point3f` | `Point3f(float x, float y, float z);` | `Point3f(c, r, 0)` | 3D 浮点点 |
| `Point2f` | `Point2f(float x, float y);` | `vector<Point2f> corners;` | 2D 浮点点 |
| `Size` | `Size(int width, int height);` | `Size(9, 6)`、`Size(11, 11)` | 二维尺寸 |

---

## 与前面几篇的差异对照

| 项目 | hsv_basic.md | trajectory.md | mouse_roi_calibration.md | camera_calibration.md |
|------|-------------|---------------|--------------------------|-----------------------|
| 核心目标 | 颜色追踪 | 轨迹绘制 | 鼠标框选标定 | 相机内参标定 |
| 输入 | 视频流 | 视频流 | 视频流 | 视频流 |
| 输出 | 目标中心 | 轨迹线 | HSV 阈值 | camera_matrix + distCoeffs |
| 关键新 API | `inRange` / `findContours` | `deque` / `line` | `setMouseCallback` / `meanStdDev` | `calibrateCamera` / `FileStorage` |
| 数据保存 | 无 | 无 | 无 | `calibration.yaml` |
| 依赖 | 无 | hsv_basic | hsv_basic + trajectory | 无（但被 aruco_pose 依赖） |

---

## 衔接说明

本篇的输出 `calibration.yaml` 是 [aruco_pose.md](aruco_pose.md) 的输入。ArUco 位姿估计需要：

1. 相机内参 `camera_matrix` → `solvePnP` 用来把 3D 点投影到 2D 图像
2. 畸变系数 `distortion_coefficients` → 去畸变，提升位姿精度

所以这一篇是“感知前置工作”，本身不直接参与实时追踪，但决定了后续测距的精度。