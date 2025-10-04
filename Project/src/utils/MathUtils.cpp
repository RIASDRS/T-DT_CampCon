#include "MathUtils.h"

namespace armor_detection {

double MathUtils::distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return cv::norm(p1 - p2);
}

float MathUtils::angleDifference(float angle1, float angle2) {
    float diff = std::abs(angle1 - angle2);
    return std::min(diff, 180.0f - diff);
}

double MathUtils::deg2rad(double degrees) {
    return degrees * CV_PI / 180.0;
}

double MathUtils::rad2deg(double radians) {
    return radians * 180.0 / CV_PI;
}

double MathUtils::vectorAngle(const cv::Point2f& vec) {
    return std::atan2(vec.y, vec.x) * 180.0 / CV_PI;
}

cv::Point2f MathUtils::imageToWorld(const cv::Point2f& image_point, 
                                   const cv::Mat& camera_matrix,
                                   double object_height) {
    if (camera_matrix.empty()) {
        return image_point;
    }
    
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    
    // 简化版本：假设物体在水平面上，相机高度已知
    // 实际应用中需要更复杂的计算
    cv::Point2f world_point;
    world_point.x = (image_point.x - cx) / fx;
    world_point.y = (image_point.y - cy) / fy;
    
    return world_point;
}

cv::Point2f MathUtils::midpoint(const cv::Point2f& p1, const cv::Point2f& p2) {
    return cv::Point2f((p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f);
}

double MathUtils::rotatedRectArea(const cv::RotatedRect& rect) {
    return rect.size.width * rect.size.height;
}

} // namespace armor_detection