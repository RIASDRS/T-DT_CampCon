#include "PnPSolver.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

namespace armor_detection {

PnPSolver::PnPSolver() {}

void PnPSolver::setCameraParams(const CameraParams& params) {
    camera_params_ = params;
    params_loaded_ = params.valid;
}

PoseResult PnPSolver::solveArmorPose(const std::vector<cv::Point2f>& points2d, 
                                    const ArmorModel& model) {
    PoseResult result;
    
    if (!params_loaded_) {
        std::cerr << "[ERROR] 相机参数未加载" << std::endl;
        return result;
    }
    
    if (points2d.size() != 4) {
        std::cerr << "[ERROR] 需要4个2D点进行PnP解算，当前点数: " << points2d.size() << std::endl;
        return result;
    }
    
    // 生成3D模型点
    auto points3d = generateArmorModelPoints(model);
    
    // 转换为OpenCV格式
    std::vector<cv::Point3f> object_points;
    for (const auto& p : points3d) {
        object_points.emplace_back(p.x, p.y, p.z);
    }
    
    try {
        // 使用PnP解算位姿
        cv::solvePnP(object_points, points2d, 
                     camera_params_.camera_matrix, camera_params_.dist_coeffs,
                     result.rvec, result.tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        // 计算距离
        result.distance = cv::norm(result.tvec);
        
        // 计算欧拉角
        cv::Mat rotation_matrix;
        cv::Rodrigues(result.rvec, rotation_matrix);
        
        // 提取角度
        result.pitch = std::asin(-rotation_matrix.at<double>(2, 1)) * 180 / CV_PI;
        result.yaw = std::atan2(rotation_matrix.at<double>(2, 0), 
                               rotation_matrix.at<double>(2, 2)) * 180 / CV_PI;
        
        result.valid = true;
        
        std::cout << "[Done] PnP解算成功 - 距离: " << result.distance << "mm" << std::endl;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] PnP解算失败: " << e.what() << std::endl;
    }
    
    return result;
}

std::vector<Point3D> PnPSolver::generateArmorModelPoints(const ArmorModel& model) {
    std::vector<Point3D> points;
    double half_width = model.WIDTH / 2;
    double half_height = model.HEIGHT / 2;
    
    // 装甲板四个角点（假设在XY平面，Z=0）
    // 顺序：左上 → 右上 → 右下 → 左下
    points.emplace_back(-half_width, -half_height, 0);  // 左上
    points.emplace_back(half_width, -half_height, 0);   // 右上
    points.emplace_back(half_width, half_height, 0);    // 右下
    points.emplace_back(-half_width, half_height, 0);   // 左下
    
    return points;
}

PoseResult PnPSolver::solveLightBarPose(const cv::RotatedRect& light_bar) {
    // 简化版本，实际应用中可能需要更复杂的处理
    PoseResult result;
    // 实现留空，根据需求补充
    return result;
}

double PnPSolver::calculateReprojectionError(const std::vector<cv::Point2f>& points2d,
                                           const std::vector<Point3D>& points3d,
                                           const PoseResult& pose) {
    if (!pose.valid || points2d.size() != points3d.size()) {
        return -1.0;
    }
    
    std::vector<cv::Point2f> projected_points;
    std::vector<cv::Point3f> object_points;
    
    for (const auto& p : points3d) {
        object_points.emplace_back(p.x, p.y, p.z);
    }
    
    cv::projectPoints(object_points, pose.rvec, pose.tvec,
                     camera_params_.camera_matrix, camera_params_.dist_coeffs,
                     projected_points);
    
    double total_error = 0.0;
    for (size_t i = 0; i < points2d.size(); i++) {
        double error = cv::norm(points2d[i] - projected_points[i]);
        total_error += error;
    }
    
    return total_error / points2d.size();
}

} // namespace armor_detection