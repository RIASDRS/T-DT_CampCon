#include "CameraCalibrator.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

namespace armor_detection {

CameraCalibrator::CameraCalibrator() {}

bool CameraCalibrator::loadParams(const std::string& file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] 无法打开相机参数文件: " << file_path << std::endl;
        return false;
    }
    
    fs["camera_matrix"] >> params_.camera_matrix;
    fs["distortion_coefficients"] >> params_.dist_coeffs;
    
    if (params_.camera_matrix.empty() || params_.dist_coeffs.empty()) {
        std::cerr << "[ERROR] 相机参数读取失败" << std::endl;
        return false;
    }
    
    // 分解内参矩阵
    params_.fx = params_.camera_matrix.at<double>(0, 0);
    params_.fy = params_.camera_matrix.at<double>(1, 1);
    params_.cx = params_.camera_matrix.at<double>(0, 2);
    params_.cy = params_.camera_matrix.at<double>(1, 2);
    
    params_.valid = true;
    std::cout << "[Done] 相机参数加载成功" << std::endl;
    std::cout << "       内参矩阵: [" << params_.fx << ", 0, " << params_.cx << "]" << std::endl;
    std::cout << "                [0, " << params_.fy << ", " << params_.cy << "]" << std::endl;
    
    return true;
}

bool CameraCalibrator::saveParams(const std::string& file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] 无法创建相机参数文件: " << file_path << std::endl;
        return false;
    }
    
    fs << "camera_matrix" << params_.camera_matrix;
    fs << "distortion_coefficients" << params_.dist_coeffs;
    
    std::cout << "[Done] 相机参数保存成功: " << file_path << std::endl;
    return true;
}

bool CameraCalibrator::calibrate(const std::vector<std::string>& image_paths, 
                                cv::Size board_size, float square_size) {
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    
    // 生成物体坐标点
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            obj.emplace_back(j * square_size, i * square_size, 0);
        }
    }
    
    cv::Size image_size;
    
    // 处理所有标定图像
    for (const auto& image_path : image_paths) {
        if (!std::filesystem::exists(image_path)) {
            std::cerr << "[ERROR] 标定图像不存在: " << image_path << std::endl;
            continue;
        }
        
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "[ERROR] 无法读取图像: " << image_path << std::endl;
            continue;
        }
        
        // 记录图像尺寸（使用第一张有效图像的尺寸）
        if (image_size.width == 0) {
            image_size = image.size();
        }
        
        std::vector<cv::Point2f> corners;
        bool found = findChessboardCorners(image, board_size, corners);
        
        if (found) {
            image_points.push_back(corners);
            object_points.push_back(obj);
            std::cout << "[Done] 找到角点: " << image_path << std::endl;
        } else {
            std::cerr << "[ERROR] 未找到角点: " << image_path << std::endl;
        }
    }
    
    if (image_points.size() < 5) {
        std::cerr << "[ERROR] 有效标定图像不足，需要至少5张" << std::endl;
        return false;
    }
    
    // 执行标定 - 修正参数顺序
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    
    double rms = cv::calibrateCamera(object_points, image_points, 
                                    image_size, camera_matrix,
                                    dist_coeffs, rvecs, tvecs,
                                    cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST);
    
    params_.camera_matrix = camera_matrix;
    params_.dist_coeffs = dist_coeffs;
    params_.valid = true;
    
    // 分解内参
    params_.fx = camera_matrix.at<double>(0, 0);
    params_.fy = camera_matrix.at<double>(1, 1);
    params_.cx = camera_matrix.at<double>(0, 2);
    params_.cy = camera_matrix.at<double>(1, 2);
    
    std::cout << "[Done] 相机标定完成" << std::endl;
    std::cout << "       RMS误差: " << rms << std::endl;
    std::cout << "       内参矩阵: [" << params_.fx << ", 0, " << params_.cx << "]" << std::endl;
    std::cout << "                 [0, " << params_.fy << ", " << params_.cy << "]" << std::endl;
    
    return true;
}

bool CameraCalibrator::findChessboardCorners(const cv::Mat& image, cv::Size board_size,
                                           std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    bool found = cv::findChessboardCorners(gray, board_size, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH + 
                                          cv::CALIB_CB_NORMALIZE_IMAGE + 
                                          cv::CALIB_CB_FAST_CHECK);
    
    if (found) {
        // 亚像素精确化
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }
    
    return found;
}

cv::Mat CameraCalibrator::undistortImage(const cv::Mat& distorted) {
    if (!params_.valid) {
        std::cerr << "[ERROR] 相机参数未加载，无法去畸变" << std::endl;
        return distorted.clone();
    }
    
    cv::Mat undistorted;
    cv::undistort(distorted, undistorted, params_.camera_matrix, params_.dist_coeffs);
    return undistorted;
}

} // namespace armor_detection