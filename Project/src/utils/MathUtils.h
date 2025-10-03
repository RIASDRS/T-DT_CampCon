#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <opencv2/opencv.hpp>
#include <cmath>

namespace armor_detection {

class MathUtils {
public:
    // 计算两点之间的距离
    static double distance(const cv::Point2f& p1, const cv::Point2f& p2);
    
    // 计算角度差（处理角度环绕）
    static float angleDifference(float angle1, float angle2);
    
    // 角度转弧度
    static double deg2rad(double degrees);
    
    // 弧度转角度
    static double rad2deg(double radians);
    
    // 计算向量的角度
    static double vectorAngle(const cv::Point2f& vec);
    
    // 坐标转换：图像坐标到世界坐标（简化版本）
    static cv::Point2f imageToWorld(const cv::Point2f& image_point, 
                                   const cv::Mat& camera_matrix,
                                   double object_height = 0);
    
    // 计算两点连线的中点
    static cv::Point2f midpoint(const cv::Point2f& p1, const cv::Point2f& p2);
    
    // 计算矩形的面积
    static double rotatedRectArea(const cv::RotatedRect& rect);
};

} // namespace armor_detection

#endif