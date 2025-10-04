#include "LightBarDetector.h"
#include <iostream>
#include <algorithm>

namespace armor_detection {

LightBarDetector::LightBarDetector() {
    // 默认配置
    config_ = Config();
}

cv::Mat LightBarDetector::preprocessImage(const cv::Mat& frame) {
    cv::Mat hsv, mask1, mask2, mask;
    
    // 转换为HSV颜色空间
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // 红色在HSV中有两个范围
    cv::inRange(hsv, config_.lower_red, config_.upper_red, mask1);
    
    // 第二个红色范围（红色在HSV中环绕）
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(180, 255, 255);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    
    // 合并两个红色范围
    mask = mask1 | mask2;
    
    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    
    return mask;
}

std::vector<cv::RotatedRect> LightBarDetector::detect(const cv::Mat& frame) {
    std::vector<cv::RotatedRect> light_bars;
    
    // 预处理
    cv::Mat mask = preprocessImage(frame);
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 过滤轮廓
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < config_.min_contour_area || area > config_.max_contour_area) {
            continue;
        }
        
        cv::RotatedRect rect = cv::minAreaRect(contour);
        rect = unifyRotatedRect(rect);
        
        if (isValidLightBar(rect)) {
            light_bars.push_back(rect);
        }
    }
    
    std::cout << "🔍 检测到 " << light_bars.size() << " 个灯条" << std::endl;
    return light_bars;
}

bool LightBarDetector::isValidLightBar(const cv::RotatedRect& light_bar) {
    float width = light_bar.size.width;
    float height = light_bar.size.height;
    
    // 确保宽度是较长的边
    if (width < height) {
        std::swap(width, height);
    }
    
    float aspect_ratio = width / height;
    
    // 检查长宽比
    if (aspect_ratio < config_.min_aspect_ratio || aspect_ratio > config_.max_aspect_ratio) {
        return false;
    }
    
    // 检查尺寸
    if (width < 10 || height < 3) {
        return false;
    }
    
    return true;
}

std::vector<LightBarPair> LightBarDetector::pairLightBars(const std::vector<cv::RotatedRect>& light_bars) {
    std::vector<LightBarPair> pairs;
    
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const cv::RotatedRect& left = light_bars[i];
            const cv::RotatedRect& right = light_bars[j];
            
            if (isValidPair(left, right)) {
                LightBarPair pair;
                pair.left_light = left;
                pair.right_light = right;
                pair.distance = cv::norm(left.center - right.center);
                pair.angle_diff = std::abs(left.angle - right.angle);
                pair.valid = true;
                pairs.push_back(pair);
            }
        }
    }
    
    std::cout << "[Linked] 配对成功 " << pairs.size() << " 对灯条" << std::endl;
    return pairs;
}

bool LightBarDetector::isValidPair(const cv::RotatedRect& left, const cv::RotatedRect& right) {
    // 计算灯条长度
    float left_length = std::max(left.size.width, left.size.height);
    float right_length = std::max(right.size.width, right.size.height);
    
    // 计算几何关系
    float angle_gap = std::min(std::abs(left.angle - right.angle), 
                              std::abs(90 - std::abs(left.angle - right.angle)));
    float distance = cv::norm(left.center - right.center);
    float mean_length = (left_length + right_length) / 2;
    
    // 各种比值计算
    float len_gap_ratio = std::abs(left_length - right_length) / std::max(left_length, right_length);
    float lengap_ratio = std::abs(left_length - right_length) / mean_length;
    float y_gap = std::abs(left.center.y - right.center.y);
    float y_gap_ratio = y_gap / mean_length;
    float x_gap = std::abs(left.center.x - right.center.x);
    float x_gap_ratio = x_gap / mean_length;
    float ratio = distance / mean_length;
    
    // 配对条件（使用你原有的参数）
    if (angle_gap > 10 ||
        len_gap_ratio > 1.0 ||
        lengap_ratio > 0.8 ||
        y_gap_ratio > 1.5 ||
        x_gap_ratio > 3.25 ||
        x_gap_ratio < 0.8 ||
        ratio > 3.25 ||
        ratio < 0.8) {
        return false;
    }
    
    return true;
}

} // namespace armor_detection