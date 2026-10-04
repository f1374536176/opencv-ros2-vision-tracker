# 鼠标 ROI 标定

> 目标：用鼠标拖拽框选目标区域，按 `C` 键从框选区域提取 HSV 均值和标准差，生成动态阈值。  
> 环境：Ubuntu 24.04 + OpenCV 4.8 + C++  
> 相关：HSV 基础与中央 ROI 标定见 [hsv_basic.md](hsv_basic.md)，轨迹绘制见 [trajectory.md](trajectory.md)。

---

## 1. 核心原理

### 1.1 从“中央 ROI”到“鼠标框选”

中央 ROI 标定的缺陷在 [hsv_basic.md](hsv_basic.md) 的 4.7 已经记录：取样区域固定，物体不在中央就会取到背景色。解决方向很直接——**把取样区域的决定权交给用户，让用户用鼠标框选**。

鼠标框选要解决三个问题：

1. **监听鼠标事件**：按下、拖动、松开，OpenCV 通过 `setMouseCallback` 注册回调函数。
2. **记录框选状态**：正在拖拽、已框选完成、已取消，这些状态要用全局变量维护。
3. **提取并计算阈值**：从 `Rect` 区域取 ROI，转 HSV，求均值和标准差，生成上下界。

### 1.2 状态机思路

鼠标交互天然是**状态驱动**的。三个阶段对应三个状态：

```
空闲 ──左键按下──▶ 拖拽中 ──左键松开──▶ 已框选
  ▲                                        │
  └────────── ESC 或 按 C 标定完成 ─────────┘
```

对应到代码里的三个全局变量：

| 变量 | 含义 | 状态为 true 时 |
|------|------|---------------|
| `g_drawing` | 正在按住左键拖拽 | 画跟随鼠标的绿色矩形 |
| `g_roi_selected` | 框选已完成 | 画黄色矩形 + 提示文字 |
| `g_roi_rect` | 最终选定的矩形区域 | 按 C 时从这里提取 ROI |

用独立的状态变量代替嵌套 `if-else`，主循环里只需要根据状态决定画什么，逻辑清晰。

### 1.3 动态容差 vs 固定容差

| 方案 | 容差来源 | 优点 | 缺点 |
|------|---------|------|------|
| 固定容差（hsv_basic） | 写死 ±10 / ±50 / ±50 | 简单，可预测 | 光照剧变跟不上 |
| 动态容差（本篇） | `meanStdDev` 的标准差 × 1.5 | 自动适应颜色分布 | 框选混入背景时容差会被拉宽 |

动态容差的核心公式：

```cpp
int delta_h = max(10, (int)(stddev_hsv[0] * 1.5));
```

- 标准差大 → 颜色分布散 → 放宽容差
- 标准差小 → 颜色集中 → 收紧容差
- `max(10, ...)` 保证最小容差不小于 10，防止阈值太窄

---

## 2. 关键代码

### 2.1 全局状态变量

```cpp
// 动态阈值
Scalar g_lower(45, 100, 100);
Scalar g_upper(75, 255, 255);

// 鼠标框选状态
bool g_drawing = false;          // 是否正在按住鼠标拖拽
bool g_roi_selected = false;     // 是否已经框选完成
Point g_start_pt, g_end_pt;      // 鼠标按下和释放的坐标
Rect g_roi_rect;                 // 最终选定的矩形区域
```

放在 `main` 外面，因为回调函数和主循环都要访问它们。

### 2.2 注册鼠标回调

```cpp
namedWindow("Detection Result", WINDOW_AUTOSIZE);
setMouseCallback("Detection Result", onMouse, nullptr);
```

- `setMouseCallback` **只需注册一次**，必须在 `while` 循环之前执行。
- `waitKey(1)` 不仅刷新画面，还负责处理系统消息。**没有 `waitKey`，回调函数永远不会被调用。**
- 第三个参数 `userdata` 传 `nullptr`，因为本篇用全局变量传状态；如果要传对象指针，见 [ros2_node.md](ros2_node.md) 里用 `this` 的版本。

### 2.3 鼠标回调函数

```cpp
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        g_drawing = true;
        g_start_pt = Point(x, y);
        g_end_pt = Point(x, y);
        g_roi_selected = false;
    }
    else if (event == EVENT_MOUSEMOVE && g_drawing) {
        g_end_pt = Point(x, y);          // 拖拽中不断更新终点
    }
    else if (event == EVENT_LBUTTONUP) {
        g_drawing = false;
        g_end_pt = Point(x, y);

        // 归一化：保证 x_min < x_max，无论鼠标从哪个方向拖
        int x_min = min(g_start_pt.x, g_end_pt.x);
        int x_max = max(g_start_pt.x, g_end_pt.x);
        int y_min = min(g_start_pt.y, g_end_pt.y);
        int y_max = max(g_start_pt.y, g_end_pt.y);

        // 面积太小的框选视为误触
        if ((x_max - x_min) > 10 && (y_max - y_min) > 10) {
            g_roi_rect = Rect(x_min, y_min, x_max - x_min, y_max - y_min);
            g_roi_selected = true;
        } else {
            g_roi_selected = false;
        }
    }
}
```

关键点：

- **`EVENT_MOUSEMOVE` 要判 `g_drawing`**：不拖拽时鼠标移动不需要更新终点。
- **归一化起止点**：用户可能从左下拖到右上，`x_min/x_max/y_min/y_max` 保证 `Rect` 参数合法。
- **最小尺寸过滤**：小于 10 像素的框选视为误触，避免误标定。

### 2.4 鼠标交互绘制（放在主循环 `imshow` 前）

```cpp
if (g_drawing) {
    // 正在拖拽：绿色矩形，跟随鼠标
    rectangle(frame, g_start_pt, g_end_pt, Scalar(0, 255, 0), 2);
}
if (g_roi_selected) {
    // 框选完成：黄色矩形 + 提示文字
    rectangle(frame, g_roi_rect, Scalar(0, 255, 255), 2, LINE_8);
    putText(frame, "Press C to calibrate, ESC to cancel",
            Point(g_roi_rect.x, g_roi_rect.y - 10),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
}
```

### 2.5 用 `meanStdDev` 生成动态阈值

```cpp
void calibrateHSV(Mat &frame, Rect roi) {
    // 1. 边界保护：防止框选超出图像边缘
    if (roi.x < 0) roi.x = 0;
    if (roi.y < 0) roi.y = 0;
    if (roi.x > frame.cols) roi.x = frame.cols;
    if (roi.y > frame.rows) roi.y = frame.rows;
    if (roi.x + roi.width  > frame.cols) roi.width  = frame.cols - roi.x;
    if (roi.y + roi.height > frame.rows) roi.height = frame.rows - roi.y;

    if (roi.width <= 0 || roi.height <= 0) {
        cout << ">>> 错误：ROI无效，请重新框选！" << endl;
        return;
    }

    // 2. 提取 ROI 并转 HSV
    Mat roi_img = frame(roi);
    Mat hsv_roi;
    cvtColor(roi_img, hsv_roi, COLOR_BGR2HSV);

    // 3. 求均值和标准差
    Scalar mean_hsv, stddev_hsv;
    meanStdDev(hsv_roi, mean_hsv, stddev_hsv);

    int h = (int)mean_hsv[0];
    int s = (int)mean_hsv[1];
    int v = (int)mean_hsv[2];

    // 4. 动态容差：标准差越大，容差越宽
    int delta_h = max(10, (int)(stddev_hsv[0] * 1.5));
    int delta_s = max(50, (int)(stddev_hsv[1] * 1.5));
    int delta_v = max(50, (int)(stddev_hsv[2] * 1.5));

    // 5. 生成阈值并裁剪到合法范围
    int lower_h = max(0,   h - delta_h);
    int lower_s = max(0,   s - delta_s);
    int lower_v = max(0,   v - delta_v);
    int upper_h = min(179, h + delta_h);
    int upper_s = min(255, s + delta_s);
    int upper_v = min(255, v + delta_v);

    g_lower = Scalar(lower_h, lower_s, lower_v);
    g_upper = Scalar(upper_h, upper_s, upper_v);

    printf(">>> 标定成功！H[%d-%d], S[%d-%d], V[%d-%d]\n",
           lower_h, upper_h, lower_s, upper_s, lower_v, upper_v);
}
```

### 2.6 按键处理

```cpp
int key = waitKey(1);   // 必须在 imshow 之后调用，否则鼠标回调也不会被处理
if (key == 'c' || key == 'C') {
    if (g_roi_selected) {
        calibrateHSV(frame, g_roi_rect);
        g_roi_selected = false;          // 标定完清除框选
        cout << ">>> 标定完成！" << endl;
    } else {
        cout << ">>> 请先用鼠标拖拽框选目标区域！" << endl;
    }
}
if (key == 27) {   // ESC：取消框选
    if (g_roi_selected || g_drawing) {
        g_roi_selected = false;
        g_drawing = false;
        cout << ">>> 已取消框选" << endl;
    }
}
```

`Q` 退出、`R` 清空轨迹的逻辑和 [trajectory.md](trajectory.md) 里一致，不重复。

---

## 3. 参数与调参

### 3.1 动态容差系数为什么取 1.5

```cpp
int delta_h = max(10, (int)(stddev_hsv[0] * 1.5));
```

1.5 是经验值，兼顾覆盖率和抗干扰。如果框选纯色目标，标准差接近 0，`max(10, ...)` 保证最小容差。

### 3.2 最小容差为什么是 10 / 50 / 50

`max(10, ...)`、`max(50, ...)`、`max(50, ...)` 里的三个常数，是**下限保护**：

- 纯色目标的标准差接近 0，如果直接 `delta_h = 0`，阈值会变成 `H=h, H=h`，稍有色偏就抓不到。
- 下限让阈值始终有一定宽容度，避免“太精确反而脆弱”。

这三个值沿用了 [hsv_basic.md](hsv_basic.md) 固定容差版本的经验值。

### 3.3 容差上限缺失

当前代码只有下限 `max`，**没有上限**。如果框选时混入一半背景，标准差会很大，`delta_h` 可能被拉到 50 甚至 100，导致误抓背景。

**建议加一个上限**：

```cpp
int delta_h = min(30, max(10, (int)(stddev_hsv[0] * 1.5)));
int delta_s = min(100, max(50, (int)(stddev_hsv[1] * 1.5)));
int delta_v = min(100, max(50, (int)(stddev_hsv[2] * 1.5)));
```

超过上限时提示“颜色分布过散，请重新框选”。

---

## 4. 问题记录

### 4.1 `setMouseCallback` 没生效

- **现象**：注册了回调函数，但鼠标拖动没有任何反应。
- **根因**：主循环里没有 `waitKey`。OpenCV 的鼠标事件依赖 `waitKey` 处理系统消息队列。
- **解决**：确保 `waitKey(1)` 在主循环里每帧都调用。

### 4.2 框选方向反了导致 `Rect` 参数为负

- **现象**：从右下往左上拖动时，`Rect` 宽高为负数，提取 ROI 崩溃。
- **根因**：`g_start_pt` 和 `g_end_pt` 谁大谁小不确定。
- **解决**：在 `EVENT_LBUTTONUP` 里用 `min` / `max` 判断，保证 `x_min < x_max`。

### 4.3 框选混入背景导致误抓

- **现象**：框选时一半是目标、一半是背景，标定后阈值太宽，把背景也抓进来。
- **根因**：`meanStdDev` 的均值被背景拉偏，标准差变大，动态容差跟着放宽。
- **方向**：
  - 加容差上限（见 3.3）。
  - 加校验：如果标准差超过某个值，拒绝标定并提示“请只框选目标”。
  - 更彻底：用鼠标点击取样（单点或小区域）而非大范围框选。


### 4.4 反光面与暗面导致容差被拉宽

- **现象**：光滑反光物体，色面和反光面（接近白色）的 HSV 差异很大，标准差随之变大。
- **结果**：动态容差可能接近 100，阈值范围过宽。
- **方向**：手动框选只框纯色区域，避开反光面；或者对标定区域做中值滤波后再求均值。

---

## 5. 验证结果

- 鼠标拖拽时画面显示绿色矩形，松开后显示黄色矩形和提示文字。
- 按 `C` 后终端打印标定结果，如 `H[40-70], S[80-180], V[90-200]`。
- 对纯色目标标定后，识别明显比固定阈值稳定，尤其是色相相近的背景不再误抓。
- 按 `ESC` 能取消框选，按 `R` 能清空轨迹，互不干扰。
- 框选区域过小时视为误触，不触发标定。

---

## 6. 待办 / 已知缺陷

- [ ] 加容差上限（`min(30, ...)` / `min(100, ...)` / `min(100, ...)`）。
- [ ] 加标准差校验：标准差过大时拒绝标定并提示重新框选。
- [ ] 鼠标点击取样（单点或小区域）作为大范围框选的补充。
- [ ] 标定结果持久化：可选写入 YAML，下次启动直接加载。
- [ ] 标定预览可视化：在框选区域内叠加当前 HSV 掩膜，方便确认。

---

## 附：本篇涉及的关键 API

### 鼠标交互

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `setMouseCallback` | `void setMouseCallback(const String& winname, MouseCallback onMouse, void* userdata = 0);` | `setMouseCallback("Detection Result", onMouse, nullptr);` | 为窗口注册鼠标回调，只需注册一次 |
| `MouseCallback` | `typedef void (*MouseCallback)(int event, int x, int y, int flags, void* userdata);` | `void onMouse(int event, int x, int y, int flags, void* userdata)` | 回调函数签名 |
| `EVENT_LBUTTONDOWN` | 常量 | `event == EVENT_LBUTTONDOWN` | 左键按下 |
| `EVENT_LBUTTONUP` | 常量 | `event == EVENT_LBUTTONUP` | 左键松开 |
| `EVENT_MOUSEMOVE` | 常量 | `event == EVENT_MOUSEMOVE` | 鼠标移动 |
| `EVENT_RBUTTONDOWN` | 常量 | 未使用 | 右键按下，本篇未用 |

### 统计

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `meanStdDev` | `void meanStdDev(InputArray src, OutputArray mean, OutputArray stddev, InputArray mask = noArray());` | `meanStdDev(hsv_roi, mean_hsv, stddev_hsv);` | 同时求均值和标准差，输出为 `Scalar` 或 `Mat` |

### 绘制

| API | 准确签名 / 调用 | 笔记中的用法 | 说明 |
|-----|----------------|-------------|------|
| `putText` | `void putText(InputOutputArray img, const String& text, Point org, int fontFace, double fontScale, Scalar color, int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false);` | `putText(frame, "...", Point(x, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,255), 2);` | 在画面写文字 |

### 与 hsv_basic.md 共用的部分

`Scalar`、`cvtColor`、`inRange`、`rectangle`、`Rect`、`Point`、`Mat` 等已在 [hsv_basic.md](hsv_basic.md) 的 API 表中列出，这里不重复。

`deque`、`line` 等已在 [trajectory.md](trajectory.md) 的 API 表中列出，这里不重复。

---

## 与 hsv_basic.md / trajectory.md 的差异对照

| 项目 | hsv_basic.md | trajectory.md | mouse_roi_calibration.md |
|------|-------------|---------------|--------------------------|
| 核心目标 | 找到目标、定位中心 | 记住历史中心、画出轨迹 | 鼠标框选取样、动态容差 |
| 新增头文件 | 无 | `<deque>` | 无 |
| 新增全局变量 | `g_lower` / `g_upper` | 无 | `g_drawing` / `g_roi_selected` / `g_roi_rect` |
| 新增函数 | `calibrateHSV(frame)` | 无 | `onMouse(...)` + `calibrateHSV(frame, roi)` |
| 容差方式 | 固定 ±10/±50/±50 | 无 | 动态 `meanStdDev × 1.5` |
| 新增按键 | `G` 标定 / `ESC` 取消 | `R` 清空轨迹 | `C` 确认标定 / `ESC` 取消框选 |
