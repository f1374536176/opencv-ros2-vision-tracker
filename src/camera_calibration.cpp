#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头！" << endl;
        return -1;
    }

    // 棋盘格内角点尺寸（列数，行数）—— 必须与你的棋盘格图片匹配
    Size boardSize(9, 6);
    vector<vector<Point3f>> objectPoints; // 世界坐标系中的3D点
    vector<vector<Point2f>> imagePoints;  // 图像坐标系中的2D点
    vector<Point3f> objp;

    // 初始化棋盘格的世界坐标（假设每个格子边长 1 单位，Z=0）
    for (int r = 0; r < boardSize.height; r++) {
        for (int c = 0; c < boardSize.width; c++) {
            objp.push_back(Point3f(c, r, 0));
        }
    }

    Mat frame, gray;
    int captured = 0;
    cout << "================== 标定工具 ==================" << endl;
    cout << "操作指南：" << endl;
    cout << "  [空格键] 拍摄当前画面（检测到棋盘格会自动保存）" << endl;
    cout << "  [Q 键]   完成拍摄，开始计算标定参数" << endl;
    cout << "当前已拍摄: " << captured << " 张" << endl;
    cout << "==============================================" << endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;

        // 检测棋盘格
        bool found = findChessboardCorners(gray, boardSize, corners, 
                                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // 亚像素精确化（让角点更准）
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), 
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
            drawChessboardCorners(frame, boardSize, corners, found);
            putText(frame, "Detected! Press SPACE to capture.", Point(20, 30), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        } else {
            putText(frame, "Please adjust angle/distance.", Point(20, 30), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        }

        imshow("Camera Calibration", frame);

        int key = waitKey(1);
        if (key == ' ') { // 空格键保存
            if (found) {
                objectPoints.push_back(objp);
                imagePoints.push_back(corners);
                captured++;
                cout << ">>> 已拍摄第 " << captured << " 张，继续移动棋盘格..." << endl;
            } else {
                cout << ">>> 未检测到棋盘格，请调整角度或光照！" << endl;
            }
        }
        if (key == 'q' && captured >= 10) { // 至少需要10张不同角度
            break;
        }
        if (key == 'q' && captured < 10) {
            cout << ">>> 照片不足（需 10 张以上），当前仅 " << captured << " 张，继续拍摄！" << endl;
        }
    }

    cap.release();
    destroyAllWindows();

    if (captured < 10) {
        cerr << "错误：有效照片不足 10 张，无法标定！" << endl;
        return -1;
    }

    cout << "开始计算标定参数..." << endl;

    // 核心标定函数！
    Mat cameraMatrix, distCoeffs, R, T;
    double rms = calibrateCamera(objectPoints, imagePoints, Size(frame.cols, frame.rows), 
                                 cameraMatrix, distCoeffs, R, T);

    cout << "================== 标定结果 ==================" << endl;
    cout << "重投影误差 RMS: " << rms << " (越小越好，< 0.5 为优秀)" << endl;
    cout << "\n相机内参矩阵 (Camera Matrix):\n" << cameraMatrix << endl;
    cout << "\n畸变系数 (Distortion Coefficients):\n" << distCoeffs << endl;
    cout << "==============================================" << endl;

    // 可选：保存为YAML文件，方便以后直接读取
    FileStorage fs("calibration.yaml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "image_width" << frame.cols;
        fs << "image_height" << frame.rows;
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();
        cout << "\n参数已保存至 calibration.yaml" << endl;
    }

    return 0;
}