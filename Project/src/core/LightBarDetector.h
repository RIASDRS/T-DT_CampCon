#ifndef LIGHTBARDETECTOR_H
#define LIGHTBARDETECTOR_H

#include "common.h"
#include "UnifiedRotatedRect.h"
#include <vector>

namespace armor_detection {

class LightBarDetector {
public:
    struct Config {
        cv::Scalar lower_red = cv::Scalar(1, 232, 167);
        cv::Scalar upper_red = cv::Scalar(27, 255, 255);
        double min_contour_area = 120;
        double max_contour_area = 5000;
        float min_aspect_ratio = 2.0f;
        float max_aspect_ratio = 8.0f;
    };

    LightBarDetector();
    
    // 设置配置
    void setConfig(const Config& config) { config_ = config; }
    
    // 检测灯条
    std::vector<cv::RotatedRect> detect(const cv::Mat& frame);
    
    // 灯条配对
    std::vector<LightBarPair> pairLightBars(const std::vector<cv::RotatedRect>& light_bars);

private:
    Config config_;
    
    // 图像预处理
    cv::Mat preprocessImage(const cv::Mat& frame);
    
    // 灯条过滤条件
    bool isValidLightBar(const cv::RotatedRect& light_bar);
    
    // 配对条件检查
    bool isValidPair(const cv::RotatedRect& left, const cv::RotatedRect& right);
};

} // namespace armor_detection

#endif