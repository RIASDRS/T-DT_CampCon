#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace armor_detection {

// 3D坐标点
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

// 2D坐标点
struct Point2D {
    double x, y;
    Point2D(double x = 0, double y = 0) : x(x), y(y) {}
};

// 装甲板3D模型尺寸（单位：mm）
struct ArmorModel {
    static constexpr double WIDTH = 140.0;           // 装甲板宽度  
    static constexpr double HEIGHT = 125.0;          // 装甲板高度 
    static constexpr double lWIDTH = 235.0;           // 装甲板宽度  
    static constexpr double lHEIGHT = 127.0;          // 装甲板高度 
    static constexpr double LIGHT_BAR_HEIGHT = 55.0;  // 灯条高度
};

// 相机参数结构
struct CameraParams {
    cv::Mat camera_matrix;      // 相机内参矩阵
    cv::Mat dist_coeffs;        // 畸变系数
    double fx, fy, cx, cy;      // 内参分解值
    
    bool valid = false;
};

// 位姿结果
struct PoseResult {
    cv::Mat rvec;               // 旋转向量
    cv::Mat tvec;               // 平移向量
    double distance;            // 距离（mm）
    double pitch;               // 俯仰角（度）
    double yaw;                 // 偏航角（度）
    bool valid = false;
};

// 灯条配对结果
struct LightBarPair {
    cv::RotatedRect left_light;
    cv::RotatedRect right_light;
    float distance;
    float angle_diff;
    bool valid = false;
};

// 检测结果
struct DetectionResult {
    cv::RotatedRect armor_rect;         // 装甲板矩形
    std::vector<cv::Point2f> points2d;  // 2D特征点
    std::vector<Point3D> points3d;      // 3D模型点
    PoseResult pose;                    // 位姿信息
    int digit = -1;                     // 识别数字
    double confidence = 0.0;            // 置信度
    bool valid = false;
};

} // namespace armor_detection

#endif