# HSV 颜色追踪与动态标定

> 目标：用 HSV 阈值分割实现实时颜色追踪，并通过 ROI 取样让阈值能适应光照变化。  
> 环境：Ubuntu 24.04 + OpenCV 4.8 + C++  
> 相关：鼠标框选标定见 [mouse_roi_calibration.md](mouse_roi_calibration.md)，轨迹绘制见 [trajectory.md](trajectory.md)。

---

## 1. 核心原理

### 1.1 为什么颜色识别用 HSV 而不是 BGR

在 OpenCV 默认的 BGR 空间中，一个颜色的“本相”由蓝、绿、红三个通道共同决定，改变光照会同时影响三个通道，阈值很难稳定。而 HSV 把颜色信息拆成三个相对独立的维度：

| 通道 | 含义 | 取值范围（OpenCV） | 影响 |
|------|------|-------------------|--------|
| H | 色相（Hue） | 0–179 | 决定颜色种类（蓝、绿、红等） |
| S | 饱和度（Saturation） | 0–255 | 影响颜色的纯度，从灰色到纯色 |
| V | 明度（Value） | 0–255 | 影响颜色明亮度，从黑色到最亮 |

H 单独分离出来之后，**光照强度主要影响 V，颜色种类主要由 H 决定**，所以阈值分割更稳定。

> 注意：OpenCV 的 H 范围是 0–179，不是常见的 0–360。红色分布在 H=0–10 和 H=170–179 两段，需要两个 mask 按位或合并。

### 1.2 处理流程

```
读取帧 (cap >> frame)
    ↓
BGR 高斯去噪 (GaussianBlur)
    ↓
BGR → HSV (cvtColor)
    ↓
阈值分割 (inRange) → 得到二值掩膜 mask
    ↓
形态学去噪：腐蚀 → 膨胀 (erode → dilate)
    ↓
找轮廓 (findContours)
    ↓
遍历轮廓，取面积最大者，面积 > 500 才认
    ↓
画外接矩形 + 中心十字
    ↓
显示窗口 + 按键检测
```

---

### 1.3 Mat 的内存模型

`Mat` 不是普通数组，而是 OpenCV 的**智能句柄**。理解它的内存模型，才能理解 ROI 为什么是浅拷贝、什么时候必须 `.clone()`。

**两段式结构**

| 部分 | 内容 | 生命周期 |
|------|------|---------|
| 数据头（header） | 尺寸、类型、通道数、指向数据体的指针、引用计数 | 随 `Mat` 对象本身 |
| 数据体（data） | 真正的像素矩阵 | 由引用计数管理 |

**句柄语义**

- `Mat` 本身不是原生指针，声明和使用时**不需要加 `*`**。
- 赋值时只复制数据头，不复制数据体，两个头指向同一块数据：

```cpp
Mat img1 = imread("a.jpg");
Mat img2 = img1;          // 只复制头，img1 和 img2 指向同一数据体
```
- 这称为浅拷贝。修改 img2 的像素，img1 也会变。
- 数据头内部维护引用计数：每多一个 Mat 指向它，计数 +1；每销毁一个，计数 -1。
- 引用计数归零时，数据体自动释放。所以 Mat 被称为智能句柄，不需要手动 delete。

需要独立副本时用 .clone()进行深拷贝：

```cpp
Mat roi_img = frame(center_roi).clone();   // 独立副本，改 roi_img 不影响 frame
```

---

### 1.4 Scalar ：一个“含义待定”的四元组

`Scalar` 不是“颜色类”，它只是一个装着 **4 个 `double`** 的容器。每个数字代表什么，**完全取决于把它传给了哪个函数**。

| 使用场景 | 第 1 个 | 第 2 个 | 第 3 个 | 第 4 个 |
|---------|--------|--------|--------|--------|
| HSV 阈值 | H | S | V | 未使用（0） |
| BGR 颜色 | B | G | R | 未使用（0） |

所以看到 `Scalar(0, 100, 100)`，不能直接说“这是绿色”，要看它被传进的是 `inRange`（HSV）还是 `rectangle`（BGR）。

**两种写法**

```cpp
// 1. 命名对象：需要反复使用，或作为阈值长期持有
Scalar lower_green(45, 100, 100);
Scalar upper_green(75, 255, 255);
inRange(hsv, lower_green, upper_green, mask);

// 2. 临时对象：即用即销，通常用于绘制
rectangle(frame, rect, Scalar(255, 0, 0), 2);       // 蓝色框
drawMarker(frame, center, Scalar(0, 0, 255), MARKER_CROSS, 20, 2);  // 红色十字
```

阈值分割时推荐命名，便于复用和调参，因为 `inRange` 要两个 `Scalar` 上下界；绘制时用第二种，省去命名。

**访问分量**

`Scalar` 重载了 `[]`，返回 `double`：

```cpp
Scalar mean_hsv = mean(hsv_roi);
int h = (int)mean_hsv[0];   // H
int s = (int)mean_hsv[1];   // S
int v = (int)mean_hsv[2];   // V
```

这也是为什么均值能直接拆成 H、S、V 三个整数去生成动态阈值。

**同一个 Scalar，在不同函数里含义不同**

写颜色代码可能会这样：

```cpp
rectangle(frame, rect, Scalar(0, 255, 0), 2);   // 以为是绿色
```

在 BGR 里确实是绿色。但如果把同一个 `Scalar(0, 255, 0)` 传给 `inRange`，它会被当成 `H=0, S=255, V=0`，是**纯黑的红**，不是绿色。

> **同一个 `Scalar`，在 `inRange` 里是 HSV，在 `rectangle` 里是 BGR。**  
> 看到颜色不对，先确认它进了哪个函数。

---

## 2. 关键代码

### 2.1 打开摄像头

```cpp
VideoCapture cap(0);
if (!cap.isOpened()) {
    cerr << "错误：摄像头打开失败！检查驱动或权限" << endl;
    return -1;
}
```

- `0` 表示默认摄像头，传文件名则读取视频文件。
- 必须检查 `isOpened()`，否则后面 `cap >> frame` 会静默失败。

### 2.2 去噪 + 转 HSV + 阈值分割

```cpp
GaussianBlur(frame, blur, Size(5, 5), 0);
cvtColor(blur, hsv, COLOR_BGR2HSV);
inRange(hsv, g_lower, g_upper, mask);
```

- 高斯核必须是奇数，过大导致目标边缘模糊、中心定位偏移。
- `inRange` 输出的是黑白二值掩膜：白色是目标，黑色是背景，等于把三通道降维成一通道。
- `g_lower` / `g_upper` 是全局动态阈值，初始为荧光绿的默认值，按 G 标定后会被覆盖，定义见 2.5。

### 2.3 形态学去噪

```cpp
Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
erode(mask, mask, kernel, Point(-1, -1), 1);
dilate(mask, mask, kernel, Point(-1, -1), 2);
```

先腐蚀后膨胀（开运算），去掉细小噪点，保留主体。若先膨胀后腐蚀（闭运算），则填充空洞、连接邻近物体。
这里腐蚀 1 次、膨胀 2 次，是为了让目标区域稍微“胖”一点，便于后续轮廓稳定。

### 2.4 找最大轮廓并绘制

```cpp
vector<vector<Point>> contours;
findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

int max_idx = 0;
double max_area = 0;
for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area > max_area) { max_area = area; max_idx = i; }
}

if (max_area > 500) {   // 面积过滤，防止噪点误报
    Rect rect = boundingRect(contours[max_idx]);
    Point center(rect.x + rect.width/2, rect.y + rect.height/2);
    rectangle(frame, rect, Scalar(255, 0, 0), 2);
    drawMarker(frame, center, Scalar(0, 0, 255), MARKER_CROSS, 20, 2);
}
```

- `RETR_EXTERNAL` 只取最外层轮廓，`CHAIN_APPROX_SIMPLE` 只存拐点，减少数据量。
- `max_area > 500` 是经验阈值，低于它的轮廓当噪声丢弃。

### 2.5 中央 ROI 自动取色

```cpp
// 全局动态阈值
Scalar g_lower(45, 100, 100);
Scalar g_upper(75, 255, 255);

// 按 G 确认时，从画面中央 ROI 提取 HSV 均值，生成当前光照下的阈值
void calibrateHSV(Mat &frame) {
    // 1. 取画面正中央 1/4 面积作为取样区
    int roi_w = frame.cols / 2;
    int roi_h = frame.rows / 2;
    int start_x = frame.cols / 4;
    int start_y = frame.rows / 4;

    Rect center_roi(start_x, start_y, roi_w, roi_h);

    // 2. ROI 边界保护，防止小分辨率或异常尺寸下越界
    if (center_roi.x < 0) center_roi.x = 0;
    if (center_roi.y < 0) center_roi.y = 0;
    if (center_roi.x > frame.cols) center_roi.x = frame.cols;
    if (center_roi.y > frame.rows) center_roi.y = frame.rows;

    if (center_roi.x + center_roi.width > frame.cols)
        center_roi.width = frame.cols - center_roi.x;
    if (center_roi.y + center_roi.height > frame.rows)
        center_roi.height = frame.rows - center_roi.y;

    Mat roi_img = frame(center_roi);        // ROI 是浅拷贝，只复制信息头
    Mat hsv_roi;
    cvtColor(roi_img, hsv_roi, COLOR_BGR2HSV);

    // 3. 求 H/S/V 均值
    Scalar mean_hsv = mean(hsv_roi);
    int h = (int)mean_hsv[0];
    int s = (int)mean_hsv[1];
    int v = (int)mean_hsv[2];

    // 4. 以均值为中心生成 ±容差，并裁剪到合法范围
    int lower_h = std::max(0,   h - 10);
    int lower_s = std::max(0,   s - 50);
    int lower_v = std::max(0,   v - 50);

    int upper_h = std::min(179, h + 10);   // OpenCV 中 H 最大是 179
    int upper_s = std::min(255, s + 50);
    int upper_v = std::min(255, v + 50);

    // 5. 写回全局阈值
    g_lower = Scalar(lower_h, lower_s, lower_v);
    g_upper = Scalar(upper_h, upper_s, upper_v);
    g_isCalibrated = true;

    printf(">>> 标定成功！HSV 范围: H[%d-%d], S[%d-%d], V[%d-%d]\n",
           lower_h, upper_h, lower_s, upper_s, lower_v, upper_v);
}
```
> 完整的按键触发、预览框绘制、确认/取消流程见 2.6 主循环完整骨架。

### 2.6 主循环完整骨架

```cpp
bool calibrating = false;   // 标定预览模式状态，2.5 的 calibrateHSV 在确认时被调用

while (true) {
    cap >> frame;
    if (frame.empty()) break;

    // 关键：克隆一份干净帧，专门给标定用
    // 否则后面在 frame 上画的蓝框、红十字会污染 HSV 均值
    Mat calib_frame = frame.clone();

    // 1. 高斯去噪 + 转 HSV
    GaussianBlur(frame, blur, Size(5, 5), 0);
    cvtColor(blur, hsv, COLOR_BGR2HSV);

    // 2. 阈值分割：使用动态阈值
    inRange(hsv, g_lower, g_upper, mask);

    // 3. 形态学处理
    erode(mask, mask, kernel, Point(-1, -1), 1);
    dilate(mask, mask, kernel, Point(-1, -1), 2);

    // 4. 找轮廓，取最大面积，画框和中心十字
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        int max_idx = 0;
        double max_area = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }

        if (max_area > 500) {
            Rect rect = boundingRect(contours[max_idx]);
            Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);

            rectangle(frame, rect, Scalar(255, 0, 0), 2);      // 蓝框
            drawMarker(frame, center, Scalar(0, 0, 255),
                       MARKER_CROSS, 20, 2);                    // 红十字

            printf("目标中心坐标: X=%3d, Y=%3d\n", center.x, center.y);
        }
    }

    // 5. 如果处于标定预览模式，画取样框和提示文字
    if (calibrating) {
        int roi_w = frame.cols / 2;
        int roi_h = frame.rows / 2;
        int start_x = frame.cols / 4;
        int start_y = frame.rows / 4;
        Rect preview_roi(start_x, start_y, roi_w, roi_h);

        // 黄色矩形框，表示取样区域
        rectangle(frame, preview_roi, Scalar(0, 255, 255), 2, LINE_8);

        // 画面顶部提示
        putText(frame, "Sampling ROI (press G to confirm, ESC to cancel)",
                Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7,
                Scalar(0, 255, 255), 2);
    }

    // 6. 显示
    imshow("Detection Result", frame);
    imshow("Mask (黑白阈值)", mask);

    // 7. 按键处理
    int key = waitKey(1);

    if (key == 'q') break;

    // G 键：第一次进入预览，第二次确认标定
    if (key == 'g' || key == 'G') {
        if (!calibrating) {
            calibrating = true;
            cout << ">>> 标定预览模式：请把目标移到黄色取样框内，"
                    "再按 G 确认；按 ESC 取消" << endl;
        } else {
            calibrateHSV(calib_frame);   // 用干净帧标定，避免标注污染
            calibrating = false;
            cout << ">>> 阈值已更新！" << endl;
        }
    }

    // ESC 键：取消标定预览
    if (key == 27) {   // ESC
        if (calibrating) {
            calibrating = false;
            cout << ">>> 已取消标定" << endl;
        }
    }
}
cap.release();
destroyAllWindows();
```

---

## 3. 参数与调参

### 3.1 最终使用的阈值

使用荧光绿物体作为目标，附近无干扰色：

```cpp
Scalar lower_green(45, 100, 100);
Scalar upper_green(75, 255, 255);
```

对应的 H 区间是 **45–75**，落在 OpenCV 色相环的绿色主区间（35–85）内。实测识别很稳，一出现就能锁定，mask 掩膜也干净。

### 3.2 调参时各通道的作用

| 通道 | 调大效果 | 调小效果 | 什么时候动它 |
|------|---------|---------|-------------|
| H | 包含更多相邻色相 | 只保留核心色相 | 抓不到目标 → 放宽；误抓背景 → 收紧 |
| S | 包含更浅/更灰的颜色 | 只保留高饱和颜色 | 反光面发白导致漏抓 → 放宽 S 下限 |
| V | 包含更暗的颜色 | 只保留明亮区域 | 阴影处漏抓 → 放宽 V 下限 |

### 3.3 红色双区间问题

红色在 HSV 中分布在 **H=0–10** 和 **H=170–179** 两段。只抓一段会导致“红杯子有时候漏掉”。正确做法是生成两个 mask 再按位或：

```cpp
inRange(hsv, Scalar(0,100,100),   Scalar(10,255,255),  mask1);
inRange(hsv, Scalar(170,100,100), Scalar(179,255,255), mask2);
mask = mask1 | mask2;
```

这是**光照变化下的鲁棒性问题**，而不是参数没调好。


### 3.4 容差为什么取 ±10 / ±50 / ±50

中央 ROI 用 `mean()` 求均值，再固定 ±10 / ±50 / ±50 生成阈值。

| 通道 | 容差 | 理由 |
|------|------|------|
| H | ±10 | 色相最敏感，放宽容易误抓邻近色 |
| S | ±50 | 光照变化对饱和度影响大，需要更宽容 |
| V | ±50 | 同上，明度受阴影和反光影响显著 |

**局限性**：容差写死，光照剧烈变化时跟不上。后续在鼠标标定版本中改成用 `meanStdDev()` 根据标准差自适应，见 [mouse_roi_calibration.md](mouse_roi_calibration.md)。


### 3.5 边界保护


颜色空间和 ROI 都有物理边界，越界会导致程序异常或阈值失效。

**颜色阈值边界**

- H 上限是 179，S/V 上限是 255。
- 生成动态阈值时要钳制：

```cpp
int lower_h = std::max(0,   h - 10);
int upper_h = std::min(179, h + 10);
```

动态容差版本同样要钳制，只是把 10 换成 delta_h，见 [mouse_roi_calibration.md](mouse_roi_calibration.md)

**ROI 边界**

- 固定 ROI 和鼠标框选的 ROI 都可能越界，越界访问会崩溃。
- 提取前先裁剪，具体代码见 2.5 的 `calibrateHSV`。
- 鼠标标定版本的裁剪逻辑相同，见 [mouse_roi_calibration.md](mouse_roi_calibration.md)。

---
## 4. 踩坑记录
### 4.1 肤色误识别

- **现象**：抓红色时，脸上的肤色阴影也被识别成红色。
- **根因**：肤色和红色的 H 值接近，S/V 区间也有重叠。
- **思考**：可以写一个“色彩空间数据查看函数”，输入截图，输出各像素的 HSV 分布，再根据数据反推合理阈值范围，而不是盲调。

### 4.2 强反光物体识别不准

- **现象**：反光面接近白色，S 和 V 偏离目标真实颜色。
- **影响**：mask 出现空洞，轮廓断裂，中心偏移。
- **方向**：放宽 S 下限，或对反光面单独做形态学闭运算填补。

### 4.3 中央 ROI 的鲁棒性缺陷

- **现象**：物体在画面角落时按 `G`，程序取到的是中央背景色（比如白墙），把背景当目标。
- **根因**：取样区域固定，物体没有占据 ROI。
- **思考**：
  - 用 `meanStdDev()` 检查标准差，太大就拒绝标定并提示“请将物体靠近取样框”。
  - 更彻底的方案是鼠标手动框选 ROI（见 [mouse_roi_calibration.md](mouse_roi_calibration.md)）。

### 4.4 光照变化后无法锁定

- **现象**：把荧光绿物体从客厅拿到阳台，只按一次 `G` 无法稳定锁定。
- **根因**：光照变化对 H 影响小，但对 S 和 V 影响显著，固定容差跟不上。
- **方向**：动态容差 + 更频繁地重新标定。

> 编译环境、摄像头权限、虚拟机 USB 等跨主题问题，见 [debug_log.md](debug_log.md)。

---

## 5. 验证结果

- 荧光绿目标进入画面后能立即识别，不会跟丢。
- Mask 窗口能直观看到阈值分割效果，白区就是目标。
- 预览、取样、取消等行为的交互流程正常。
- 中心坐标打印格式：`目标中心坐标: X=xxx, Y=xxx`。

---

## 6. 待办 / 已知缺陷

- [ ] 红色双区间（0–10 和 170–179）合并，解决漏抓，案已给出，未集成。
- [ ] 写一个 HSV 数据查看工具，输入截图输出色彩分布，辅助调参。
- [ ] 中央 ROI 改为鼠标手动框选（已在 mouse_roi_calibration.md 完成）。
- [ ] 标定时用 `meanStdDev()` 做拒绝校验，标准差过大时提示重新取样。
- [ ] 反光面导致 mask 断裂时，尝试闭运算填补。
- [ ] 动态容差上限限制，防止容差被标准差拉得过宽导致误抓背景。

---

## 附：本篇涉及的关键 API


### 视频输入

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `VideoCapture` | `VideoCapture();`<br>`VideoCapture(int index);`<br>`VideoCapture(const String& filename, int apiPreference = CAP_ANY);` | 打开摄像头或视频文件 |
| `VideoCapture::isOpened` | `bool isOpened() const;` | `cap.isOpened()` | 检查是否打开成功 |
| `VideoCapture::operator>>` | `VideoCapture& operator>>(Mat& image);` | `cap >> frame;` | 读取一帧 |
| `VideoCapture::release` | `void release();` | `cap.release();` | 释放摄像头 |

---

### 色彩空间与阈值

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `Scalar` | `Scalar(double v0, double v1 = 0, double v2 = 0, double v3 = 0);` | `Scalar lower_green(45, 100, 100);` | HSV 中依次对应 H、S、V |
| `Scalar::operator[]` | `double& operator[](int i);` | `mean_hsv[0]` | 取第 i 个分量 |
| `cvtColor` | `void cvtColor(InputArray src, OutputArray dst, int code, int dstCn = 0);` | `cvtColor(blur, hsv, COLOR_BGR2HSV);` | BGR 转 HSV |
| `inRange` | `void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst);` | `inRange(hsv, g_lower, g_upper, mask);` | 输出二值掩膜 |

---

### 形态学与轮廓

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `getStructuringElement` | `Mat getStructuringElement(int shape, Size ksize, Point anchor = Point(-1,-1));` | `getStructuringElement(MORPH_RECT, Size(5,5));` | 生成形态学核 |
| `erode` | `void erode(InputArray src, OutputArray dst, InputArray kernel, Point anchor = Point(-1,-1), int iterations = 1, int borderType = BORDER_CONSTANT, const Scalar& borderValue = morphologyDefaultBorderValue());` | `erode(mask, mask, kernel, Point(-1,-1), 1);` | 腐蚀，anchor 默认中心，iterations 为迭代次数 |
| `dilate` | `void dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor = Point(-1,-1), int iterations = 1, int borderType = BORDER_CONSTANT, const Scalar& borderValue = morphologyDefaultBorderValue());` | `dilate(mask, mask, kernel, Point(-1,-1), 2);` | 膨胀 |
| `findContours` | `void findContours(InputArray image, OutputArrayOfArrays contours, int mode, int method, Point offset = Point());` | `findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);` | 只取外层轮廓，只存拐点 |
| `contourArea` | `double contourArea(InputArray contour, bool oriented = false);` | `contourArea(contours[i]);` | 返回轮廓面积 |
| `boundingRect` | `Rect boundingRect(InputArray array);` | `boundingRect(contours[max_idx]);` | 返回外接矩形 |

---

### 绘制与交互

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `namedWindow` | `void namedWindow(const String& winname, int flags = WINDOW_AUTOSIZE);` | `namedWindow("Detection Result", WINDOW_AUTOSIZE);` | 创建窗口 |
| `imshow` | `void imshow(const String& winname, InputArray mat);` | `imshow("Detection Result", frame);` | 显示图像 |
| `waitKey` | `int waitKey(int delay = 0);` | `waitKey(1);` | 等待按键，delay 毫秒 |
| `destroyAllWindows` | `void destroyAllWindows();` | `destroyAllWindows();` | 销毁所有窗口 |
| `rectangle` | `void rectangle(InputOutputArray img, Rect rec, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);` | `rectangle(frame, rect, Scalar(255,0,0), 2);` | 画矩形框 |
| `drawMarker` | `void drawMarker(InputOutputArray img, Point position, const Scalar& color, int markerType = MARKER_CROSS, int markerSize = 20, int thickness = 1, int line_type = 8);` | `drawMarker(frame, center, Scalar(0,0,255), MARKER_CROSS, 20, 2);` | 画中心标记 |
| `putText` | `void putText(InputOutputArray img, const String& text, Point org, int fontFace, double fontScale, Scalar color, int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false);` | `putText(frame, "...", Point(20,30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,255), 2);` | 画面写字 |

---

### Mat / Rect / Point

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `Mat` | `Mat();` | `Mat frame, blur, hsv, mask;` | 图像矩阵 |
| `Mat::cols` / `Mat::rows` | `int cols; int rows;` | `frame.cols`, `frame.rows` | 列数 / 行数，成员变量，非函数，访问不加括号 |
| `Mat::empty` | `bool empty() const;` | `frame.empty()` | 判断是否为空 |
| `Mat::operator()` | `Mat operator()(const Rect& roi) const;` | `Mat roi_img = frame(center_roi);` 等价于 `frame.operator()(center_roi)` | 提取 ROI，浅拷贝 |
| `Mat::clone` | `Mat clone() const;` | `frame(center_roi).clone()` | 深拷贝 |
| `Rect` | `Rect(int x, int y, int width, int height);` | `Rect center_roi(start_x, start_y, roi_w, roi_h);` | 矩形区域 |
| `Point` | `Point(int x, int y);` | `Point center(rect.x + rect.width/2, rect.y + rect.height/2);` | 图像坐标点 |

---

### 统计

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `mean` | `Scalar mean(InputArray src, InputArray mask = noArray());` | `Scalar mean_hsv = mean(hsv_roi)` | 求均值，返回 `Scalar`数据，可传 mask |

---

### 非 OpenCV 辅助

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `std::max` / `std::min` | `std::max(a, b)` / `std::min(a, b)` | `max(0, h - 10)` | 边界裁剪 |
| `printf` | C 标准库 | `printf("...", ...);` | 终端打印 |

---
