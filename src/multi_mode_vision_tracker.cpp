#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <deque>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <algorithm>

using namespace std;
using namespace cv;

class MultiModeVisionTracker : public rclcpp::Node {
public:
    MultiModeVisionTracker() : Node("multi_mode_vision_tracker") {
        // 声明参数
        this->declare_parameter<string>("camera_topic", "/image_raw");
        this->declare_parameter<string>("calibration_file", "calibration.yaml");
        this->declare_parameter<double>("marker_size", 0.03);
        this->declare_parameter<double>("kp", 0.5);
        this->declare_parameter<double>("ki", 0.1);
        this->declare_parameter<double>("kd", 0.1);
        this->declare_parameter<double>("max_integral", 1.0);

        // 读取
        Kp_ = this->get_parameter("kp").as_double();
        Ki_ = this->get_parameter("ki").as_double();
        Kd_ = this->get_parameter("kd").as_double();
        max_integral_ = this->get_parameter("max_integral").as_double();

        // 默认HSV阈值（绿色范围示例）
        lower_ = Scalar(45, 100, 100);
        upper_ = Scalar(75, 255, 255);

        // 形态学核
        kernel_ = getStructuringElement(MORPH_RECT, Size(5, 5));

        // 读取相机标定文件
        string calib_file = this->get_parameter("calibration_file").as_string();
        FileStorage fs(calib_file, FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开标定文件: %s", calib_file.c_str());
            rclcpp::shutdown();
            return;
        }

        int img_w, img_h;
        fs["image_width"] >> img_w;
        fs["image_height"] >> img_h;
        fs["camera_matrix"] >> cameraMatrix_;
        fs["distortion_coefficients"] >> distCoeffs_;
        fs.release();
        if (cameraMatrix_.empty() || distCoeffs_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "标定数据无效！");
            rclcpp::shutdown();
            return;
        }

        // ArUco 字典和检测器参数
        dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        detectorParams_ = aruco::DetectorParameters::create();

        // ArUco 三维坐标（以米为单位）
        float half = this->get_parameter("marker_size").as_double() / 2.0f;
        objectPoints_ = {
            Point3f(-half,  half, 0),
            Point3f( half,  half, 0),
            Point3f( half, -half, 0),
            Point3f(-half, -half, 0)
        };

        // 创建窗口并绑定鼠标回调
        namedWindow("Detection Result", WINDOW_AUTOSIZE);
        namedWindow("Mask (黑白阈值)", WINDOW_AUTOSIZE);
        setMouseCallback("Detection Result", onMouse, this); // 传递this指针

        // 订阅图像话题
        string topic = this->get_parameter("camera_topic").as_string();
        auto img_qos = rclcpp::SensorDataQoS().keep_last(1); // 保留最新一帧
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic, img_qos,
            std::bind(&MultiModeVisionTracker::imageCallback, this, std::placeholders::_1));

        // 发布结果和掩膜（可选，如果想在ROS2中查看也可以不发布，直接用窗口看）
        result_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detection_result", img_qos);
        mask_pub_   = this->create_publisher<sensor_msgs::msg::Image>("/mask", img_qos);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        RCLCPP_INFO(this->get_logger(), "启动完成，当前模式: 混合 (3)");
        RCLCPP_INFO(this->get_logger(), "操作提示: 1-颜色 2-ArUco 3-混合  r-清轨迹  c-标定  ESC-取消框选  q-退出");


        log_file_.open("tracking_log.csv");
        if(log_file_.is_open()) {
            log_file_ << "frame,error_x,cmd_angular,cmd_linear,color_detected,aruco_dist\n";
            RCLCPP_INFO(this->get_logger(), "CSV日志文件创建成功:tracking_log.csv");
        } else {
            RCLCPP_WARN(this->get_logger(), "无法创建日志文件 tracking_log.csv");
        }
    }
    ~MultiModeVisionTracker() {
        destroyAllWindows();
        if(log_file_.is_open()) {
            log_file_.close();
        }
    }
private:
    // ===== 图像回调 =====
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS图像 -> OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        // 去畸变
        Mat frame_undistorted;
        undistort(frame, frame_undistorted, cameraMatrix_, distCoeffs_);

        // 重置检测标志
        color_detected_ = false;
        aruco_detected_ = false;
        mask_.release();// 清空掩膜

        // 根据当前模式处理
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

        // 显示窗口
        imshow("Detection Result", frame_undistorted);
        if (!mask_.empty())
            imshow("Mask (黑白阈值)", mask_);
        else
            imshow("Mask (黑白阈值)", Mat::zeros(frame_undistorted.size(), CV_8UC1));

        // 处理键盘输入（必须在imshow之后调用waitKey）
        int key = waitKey(1);
        handleKeyInput(key, frame_undistorted);

        double dt = 1.0 / 30.0; // 最终默认值
        // --- 1. 优先用相机时间戳 ---
        rclcpp::Time stamp(msg->header.stamp);
        bool stamp_valid = (stamp.nanoseconds() > 0) && has_last_stamp_;

        if (stamp_valid) {
            double dt_stamp = (stamp - last_stamp_).seconds();
            if (dt_stamp > 0.001 && dt_stamp < 0.1) {
                dt = dt_stamp;
            } else {
                stamp_valid = false; // 时间戳异常，回退
            }
        }

        // --- 2. 回退到系统单调时钟 ---
        if (!stamp_valid) {
            auto now = std::chrono::steady_clock::now();
            if (has_last_wall_time_) {
                double dt_wall = std::chrono::duration<double>(now - last_wall_time_).count();
                if (dt_wall > 0.001 && dt_wall < 0.1) {
                    dt = dt_wall;
                }
            }
            last_wall_time_ = now;
            has_last_wall_time_ = true;
        }

        // --- 3. 更新相机时间戳状态 ---
        if (stamp.nanoseconds() > 0) {
            last_stamp_ = stamp;
            has_last_stamp_ = true;
        } else {
            has_last_stamp_ = false; // 时间戳无效，下次重新开始
        }
        if (dt > 0.1) dt = 0.1; // 最大 100ms，约 10fps
        if (dt < 0.001) dt = 0.001;

        // 计算控制信号并发布
        computeControlSignal(frame_undistorted.cols, frame_undistorted.rows,dt);
        // 发布结果图像
        sensor_msgs::msg::Image::SharedPtr out_msg =
            cv_bridge::CvImage(msg->header, "bgr8", frame_undistorted).toImageMsg();
        result_pub_->publish(*out_msg);

        // 发布掩膜图像（如果有）
        if (!mask_.empty()) {
            sensor_msgs::msg::Image::SharedPtr mask_msg =
                cv_bridge::CvImage(msg->header, "mono8", mask_).toImageMsg();
            mask_pub_->publish(*mask_msg);
        }
    }

    // ===== 计算控制信号 =====
    void computeControlSignal(int frame_width, int frame_height, double dt) {
        geometry_msgs::msg::Twist cmd;

        // ----- 角速度（根据颜色偏差）-----
        // ----- 角速度（PID控制）-----
        if (color_detected_) {
            lost_frames_ = 0; // 重置丢失计数器
            double error = (last_color_center_.x - frame_width / 2.0)/(frame_width / 2.0); // 归一化误差 [-1,1]
    
            // 1. 比例项 (P)
            double P_term = Kp_ * error;
    
            // 2. 积分项 (I) —— 带限幅防饱和
            integral_error_ += error * dt;
            // 钳制积分项，防止累计过大（这是抗积分饱和的核心）
            if (integral_error_ > max_integral_) integral_error_ = max_integral_;
            if (integral_error_ < -max_integral_) integral_error_ = -max_integral_;
            double I_term = Ki_ * integral_error_;
    
            // 3. 微分项 (D)
            double D_term = 0.0;
            if (has_last_error_) {
                double derivative_raw = (error - last_error_) / dt;
                double alpha = 0.7;
                derivative_filt_ = alpha * derivative_filt_ + (1.0 - alpha) * derivative_raw;
                D_term = Kd_ * derivative_filt_;
            } else {
                has_last_error_ = true; // 第一次不计算D项
                derivative_filt_ = 0.0;
            }
            last_error_ = error;
    
            // 4. 合成最终角速度（负号是因为图像坐标系Y向下，但我们控制的是转向）
            double raw_angular = -(P_term + I_term + D_term);
    
            // 5. 对输出进行限幅（防止发送超出底盘最大转速的指令）
            // 6. 经过卡尔曼滤波平滑后发布（你已经有了applyKalmanFilter）
            raw_angular = std::clamp(raw_angular, -0.8, 0.8);
            cmd.angular.z = applyKalmanFilter(raw_angular);
    
        } else {
            lost_frames_++;
            integral_error_ = 0.0;
            has_last_error_ = false; // 下次重新计算D项
            derivative_filt_ = 0.0;
            if(lost_frames_ > 5) { // 连续丢失5帧以上才认为目标丢失
                kf_x_est_ = 0.0; // 重置卡尔曼滤波器状态
                kf_P_ = 1.0; // 重置协方差
            } 
            cmd.angular.z = 0.0; // 停止旋转
        }

        // 写入当前帧数据到CSV（即使没检测到，也记录0值，这样曲线是连续的）
        if (log_file_.is_open()) {
            frame_counter_++;
            double err = color_detected_ ? (last_color_center_.x - frame_width / 2.0) : 0.0;
            double ang = cmd.angular.z;
            double lin = cmd.linear.x;
            int det = color_detected_ ? 1 : 0;
            double dist = aruco_detected_ ? last_aruco_z_ : 0.0;
    
            log_file_ << frame_counter_ << ","
                      << err << ","
                      << ang << ","
                      << lin << ","
                      << det << ","
                      << dist << "\n";
            // 注意：不 flush，让系统自己缓冲，提高性能
        }

        // 发布控制指令
        cmd_pub_->publish(cmd);
    }

    // ===== 键盘处理（保留所有原有快捷键） =====
    void handleKeyInput(int key, Mat &frame) {
        if (key == 'q' || key == 'Q') {
            RCLCPP_INFO(this->get_logger(), "退出程序");
            rclcpp::shutdown();
        }
        else if (key == '1') { current_mode_ = MODE_COLOR_ONLY; RCLCPP_INFO(this->get_logger(), "切换至颜色模式"); }
        else if (key == '2') { current_mode_ = MODE_ARUCO_ONLY; RCLCPP_INFO(this->get_logger(), "切换至ArUco模式"); }
        else if (key == '3') { current_mode_ = MODE_FUSION;      RCLCPP_INFO(this->get_logger(), "切换至混合模式"); }
        else if (key == 'r' || key == 'R') {
            trajectory_.clear();
            RCLCPP_INFO(this->get_logger(), "轨迹已清空");
        }
        else if (key == 'c' || key == 'C') {
            if (roi_selected_) {
                calibrateHSV(frame, roi_rect_);
                roi_selected_ = false;
                RCLCPP_INFO(this->get_logger(), "HSV标定完成");
            } else {
                RCLCPP_WARN(this->get_logger(), "请先用鼠标拖拽框选目标区域");
            }
        }
        else if (key == 27) { // ESC
            if (roi_selected_ || drawing_) {
                roi_selected_ = false;
                drawing_ = false;
                RCLCPP_INFO(this->get_logger(), "已取消框选");
            }
        }
    }

    // ===== 鼠标回调（静态函数，通过userdata访问对象） =====
    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        MultiModeVisionTracker* self = reinterpret_cast<MultiModeVisionTracker*>(userdata);
        self->mouseHandler(event, x, y, flags);
    }

    void mouseHandler(int event, int x, int y, int flags) {
        if (event == EVENT_LBUTTONDOWN) {
            drawing_ = true;
            start_pt_ = Point(x, y);
            end_pt_ = Point(x, y);
            roi_selected_ = false;
        }
        else if (event == EVENT_MOUSEMOVE && drawing_) {
            end_pt_ = Point(x, y);
        }
        else if (event == EVENT_LBUTTONUP) {
            drawing_ = false;
            end_pt_ = Point(x, y);

            int x_min = min(start_pt_.x, end_pt_.x);
            int x_max = max(start_pt_.x, end_pt_.x);
            int y_min = min(start_pt_.y, end_pt_.y);
            int y_max = max(start_pt_.y, end_pt_.y);

            if ((x_max - x_min) > 10 && (y_max - y_min) > 10) {
                roi_rect_ = Rect(x_min, y_min, x_max - x_min, y_max - y_min);
                roi_selected_ = true;
            } else {
                roi_selected_ = false;
            }
        }
    }

    // ===== 颜色跟踪（含轨迹、鼠标交互绘制） =====
    void processColorTracking(Mat &frame) {
        Mat blurred, hsv;
        GaussianBlur(frame, blurred, Size(5, 5), 0);
        cvtColor(blurred, hsv, COLOR_BGR2HSV);
        inRange(hsv, lower_, upper_, mask_);
        erode(mask_, mask_, kernel_, Point(-1, -1), 1);
        dilate(mask_, mask_, kernel_, Point(-1, -1), 2);

        vector<vector<Point>> contours;
        findContours(mask_, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        color_detected_ = false; // 默认未检测到
        if (!contours.empty()) {
            size_t max_idx = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > max_area) { max_area = area; max_idx = i; }
            }
            if (max_area > 500) {
                color_detected_ = true;
                Rect rect = boundingRect(contours[max_idx]);
                Point center(rect.x + rect.width/2, rect.y + rect.height/2);
                rectangle(frame, rect, Scalar(255, 0, 0), 2);
                drawMarker(frame, center, Scalar(0, 0, 255), MARKER_CROSS, 20, 2);
                trajectory_.push_back(center);
                if (trajectory_.size() > 30) trajectory_.pop_front();
                RCLCPP_INFO(this->get_logger(), "目标中心: X=%d, Y=%d", center.x, center.y);
                last_color_center_ = center;
            } else {
                color_detected_ = false;
            }
        }
        // 绘制轨迹
        for (size_t i = 1; i < trajectory_.size(); i++)
            line(frame, trajectory_[i-1], trajectory_[i], Scalar(0, 255, 0), 3);

        // 鼠标交互绘制
        if (drawing_)
            rectangle(frame, start_pt_, end_pt_, Scalar(0, 255, 0), 2);
        if (roi_selected_) {
            rectangle(frame, roi_rect_, Scalar(0, 255, 255), 2, LINE_8);
            putText(frame, "Press C to calibrate, ESC to cancel",
                    Point(roi_rect_.x, roi_rect_.y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        }
    }

    // ===== ArUco 检测 =====
    void processArucoDetection(Mat &frame) {
        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(frame, dictionary_, corners, ids, detectorParams_);

        aruco_detected_ = false; // 默认未检测到
        if (!ids.empty()) {
            aruco::drawDetectedMarkers(frame, corners, ids);
            vector<Vec3d> rvecs, tvecs;
            for (size_t i = 0; i < ids.size(); i++) {
                Vec3d rvec, tvec;
                bool ok = solvePnP(objectPoints_, corners[i], cameraMatrix_, distCoeffs_,
                                   rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
                if (ok) {
                    rvecs.push_back(rvec);
                    tvecs.push_back(tvec);
                    drawFrameAxes(frame, cameraMatrix_, distCoeffs_,
                                  rvec, tvec, this->get_parameter("marker_size").as_double() * 1.5f);
                    double dist_cm = tvec[2] * 100.0;
                    double dist_m = tvec[2]; // 单位：米
                    last_aruco_z_ = dist_m;
                    aruco_detected_ = true;
                    RCLCPP_INFO(this->get_logger(), "ArUco ID %d: Z距离 = %.1f cm", ids[i], dist_cm);
                } else {
                    RCLCPP_WARN(this->get_logger(), "ID %d 位姿估计失败", ids[i]);
                }
            }
        }
    }

    // ===== HSV标定 =====
    void calibrateHSV(Mat &frame, Rect roi) {
        if (roi.x < 0) roi.x = 0;
        if (roi.y < 0) roi.y = 0;
        if (roi.x + roi.width > frame.cols) roi.width = frame.cols - roi.x;
        if (roi.y + roi.height > frame.rows) roi.height = frame.rows - roi.y;
        if (roi.width <= 0 || roi.height <= 0) return;

        Mat roi_img = frame(roi);
        Mat hsv_roi;

        cvtColor(roi_img, hsv_roi, COLOR_BGR2HSV);
        Scalar mean_hsv, stddev_hsv;
        meanStdDev(hsv_roi, mean_hsv, stddev_hsv);

        int h = (int)mean_hsv[0];
        int s = (int)mean_hsv[1];
        int v = (int)mean_hsv[2];

        int delta_h = max(10, (int)(stddev_hsv[0] * 1.5));
        int delta_s = max(50, (int)(stddev_hsv[1] * 1.5));
        int delta_v = max(50, (int)(stddev_hsv[2] * 1.5));

        lower_ = Scalar(max(0, h - delta_h), max(0, s - delta_s), max(0, v - delta_v));
        upper_ = Scalar(min(179, h + delta_h), min(255, s + delta_s), min(255, v + delta_v));

        RCLCPP_INFO(this->get_logger(), "标定 HSV 范围: H[%d-%d] S[%d-%d] V[%d-%d]",
                    (int)lower_[0], (int)upper_[0], (int)lower_[1], (int)upper_[1],
                    (int)lower_[2], (int)upper_[2]);
    }

    double applyKalmanFilter(double measurement) {
        // 预测步骤
        double x_pred = kf_x_est_;
        double P_pred = kf_P_ + kf_Q_;

        // 更新步骤
        double K = P_pred / (P_pred + kf_R_); // 卡尔曼增益
        kf_x_est_ = x_pred + K * (measurement - x_pred);
        kf_P_ = (1 - K) * P_pred;

        return kf_x_est_;
    }

    // ===== 成员变量 =====

    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // 日志文件
    std::ofstream log_file_;
    int frame_counter_ = 0;

    // 标定参数
    Mat cameraMatrix_, distCoeffs_;

    // ArUco
    Ptr<aruco::Dictionary> dictionary_;
    Ptr<aruco::DetectorParameters> detectorParams_;
    vector<Point3f> objectPoints_;

    // 颜色追踪参数
    Scalar lower_, upper_;
    Mat mask_;
    Mat kernel_;
    deque<Point> trajectory_;

    // 模式与交互
    enum VisionMode { MODE_COLOR_ONLY, MODE_ARUCO_ONLY, MODE_FUSION };
    VisionMode current_mode_ = MODE_FUSION;

    // 鼠标框选标定
    bool drawing_ = false;
    bool roi_selected_ = false;
    Point start_pt_, end_pt_;
    Rect roi_rect_;

    // 颜色追踪结果
    bool color_detected_ = false;
    cv::Point last_color_center_{0, 0};

    // ArUco 结果
    bool aruco_detected_ = false;
    double last_aruco_z_ = 0.0;   // 单位：米

    // 卡尔曼滤波参数
    double kf_x_est_ = 0.0;   // 当前估计值
    double kf_P_ = 1.0;       // 当前误差协方差
    double kf_Q_ = 0.01;      // 过程噪声（可调参数）
    double kf_R_ = 0.1;       // 测量噪声（可调参数）

    // PID 参数（可调，建议后期设为ROS参数）
    double Kp_ = 0.5;
    double Ki_ = 0.1;   // 积分系数
    double Kd_ = 0.1;    // 微分系数
    double max_integral_ = 1.0;  // 积分限幅（防止积分饱和）

    // PID 状态量
    double integral_error_ = 0.0;
    double last_error_ = 0.0;
    double derivative_filt_ = 0.0;
    bool has_last_error_ = false;// 标记是否有上一次误差值，用于第一次不计算D项

    // 丢失帧计数器
    int lost_frames_ = 0;

    // 时间戳相关
    rclcpp::Time last_stamp_;// 上一帧相机时间戳
    bool has_last_stamp_ = false;// 是否有上一帧时间戳
    std::chrono::steady_clock::time_point last_wall_time_; // 系统时间回退
    bool has_last_wall_time_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    cv::startWindowThread();// 启动OpenCV窗口线程，避免阻塞ROS2回调
    auto node = std::make_shared<MultiModeVisionTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}