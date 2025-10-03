#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include "common.h"
#include <string>

namespace armor_detection {

class CameraCalibrator {
public:
    CameraCalibrator();
    
    // 加载相机参数
    bool loadParams(const std::string& file_path);
    
    // 保存相机参数
    bool saveParams(const std::string& file_path);
    
    // 执行相机标定（使用棋盘格）
    bool calibrate(const std::vector<std::string>& image_paths, 
                   cv::Size board_size, float square_size);
    
    // 获取相机参数
    CameraParams getCameraParams() const { return params_; }
    
    // 去畸变
    cv::Mat undistortImage(const cv::Mat& distorted);
    
    // 检查参数是否有效
    bool isParamsValid() const { return params_.valid; }

private:
    CameraParams params_;
    
    bool findChessboardCorners(const cv::Mat& image, cv::Size board_size,
                              std::vector<cv::Point2f>& corners);
};

} // namespace armor_detection

#endif