#include "ArmorDetector.h"
#include <iostream>
#include <chrono>

namespace armor_detection {

ArmorDetector::ArmorDetector() {}

bool ArmorDetector::init(const std::string& model_path, const std::string& camera_params_path) {
    std::cout << "🚀 初始化装甲板检测系统..." << std::endl;
    
    // 加载数字识别模型
    if (!digit_recognizer_.loadModel(model_path)) {
        std::cerr << "❌ 数字识别模型加载失败" << std::endl;
        return false;
    }
    
    // 加载相机参数
    if (!camera_calibrator_.loadParams(camera_params_path)) {
        std::cerr << "⚠️  相机参数加载失败，将使用默认参数" << std::endl;
        // 可以设置默认相机参数或继续运行
    } else {
        // 设置PnP解算器的相机参数
        pnp_solver_.setCameraParams(camera_calibrator_.getCameraParams());
    }
    
    std::cout << "✅ 装甲板检测系统初始化完成" << std::endl;
    return true;
}

std::vector<DetectionResult> ArmorDetector::processFrame(const cv::Mat& frame) {
    auto start_time = std::chrono::high_resolution_clock::now();
    latest_results_.clear();
    
    // 步骤1: 检测灯条
    auto light_bars = light_bar_detector_.detect(frame);
    
    // 步骤2: 配对灯条
    auto light_pairs = light_bar_detector_.pairLightBars(light_bars);
    
    // 步骤3: 为每个灯条对创建装甲板结果
    for (const auto& pair : light_pairs) {
        if (pair.valid) {
            auto armor_result = createArmorResult(pair, frame);
            if (armor_result.valid) {
                latest_results_.push_back(armor_result);
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "⏱️  处理耗时: " << duration.count() << "ms" << std::endl;
    
    return latest_results_;
}

DetectionResult ArmorDetector::createArmorResult(const LightBarPair& pair, const cv::Mat& frame) {
    DetectionResult result;
    
    // 提取特征点
    result.points2d = extractArmorPoints(pair);
    
    // 创建装甲板旋转矩形
    cv::Point2f armor_center = (pair.left_light.center + pair.right_light.center) * 0.5f;
    float armor_width = pair.distance;
    float armor_height = (std::max(pair.left_light.size.width, pair.left_light.size.height) + 
                         std::max(pair.right_light.size.width, pair.right_light.size.height)) * 0.6f;
    
    result.armor_rect = cv::RotatedRect(armor_center, 
                                       cv::Size2f(armor_width, armor_height),
                                       (pair.left_light.angle + pair.right_light.angle) / 2);
    result.armor_rect = unifyRotatedRect(result.armor_rect);
    
    // 数字识别
    result.digit = recognizeDigit(frame, result);
    
    // 3D位姿解算
    if (config_.enable_pnp && pnp_solver_.isParamsLoaded()) {
        calculatePose(result);
    }
    
    result.valid = true;
    
    std::cout << "🎯 创建装甲板结果 - 数字: " << result.digit << std::endl;
    
    return result;
}

std::vector<cv::Point2f> ArmorDetector::extractArmorPoints(const LightBarPair& pair) {
    std::vector<cv::Point2f> points;
    
    // 使用装甲板矩形的四个顶点作为特征点
    cv::Point2f vertices[4];
    cv::Point2f armor_center = (pair.left_light.center + pair.right_light.center) * 0.5f;
    float armor_width = pair.distance;
    float armor_height = (std::max(pair.left_light.size.width, pair.left_light.size.height) + 
                         std::max(pair.right_light.size.width, pair.right_light.size.height)) * 0.6f;
    
    cv::RotatedRect armor_rect(armor_center, 
                              cv::Size2f(armor_width, armor_height),
                              (pair.left_light.angle + pair.right_light.angle) / 2);
    armor_rect = unifyRotatedRect(armor_rect);
    armor_rect.points(vertices);
    
    // 顺序: 左上 → 右上 → 右下 → 左下
    for (int i = 0; i < 4; i++) {
        points.push_back(vertices[i]);
    }
    
    return points;
}

int ArmorDetector::recognizeDigit(const cv::Mat& frame, const DetectionResult& armor_result) {
    // 获取数字区域
    cv::Rect roi = armor_result.armor_rect.boundingRect();
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    
    if (roi.width > 10 && roi.height > 10) {
        cv::Mat digit_roi = frame(roi);
        return digit_recognizer_.recognize(digit_roi);
    }
    
    return -1;
}

void ArmorDetector::calculatePose(DetectionResult& result) {
    if (result.points2d.size() == 4) {
        result.pose = pnp_solver_.solveArmorPose(result.points2d);
        
        if (result.pose.valid) {
            // 计算重投影误差
            result.points3d = std::vector<Point3D>{
                Point3D(-ArmorModel::WIDTH/2, -ArmorModel::HEIGHT/2, 0),
                Point3D(ArmorModel::WIDTH/2, -ArmorModel::HEIGHT/2, 0),
                Point3D(ArmorModel::WIDTH/2, ArmorModel::HEIGHT/2, 0),
                Point3D(-ArmorModel::WIDTH/2, ArmorModel::HEIGHT/2, 0)
            };
            
            double error = pnp_solver_.calculateReprojectionError(
                result.points2d, result.points3d, result.pose);
            
            std::cout << "📏 重投影误差: " << error << " 像素" << std::endl;
        }
    }
}

void ArmorDetector::drawResults(cv::Mat& image, const std::vector<DetectionResult>& results) {
    for (const auto& result : results) {
        if (!result.valid) continue;
        
        // 绘制装甲板矩形
        cv::Point2f vertices[4];
        result.armor_rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 3);
        }
        
        // 绘制特征点
        for (const auto& point : result.points2d) {
            cv::circle(image, point, 3, cv::Scalar(0, 255, 255), -1);
        }
        
        // 显示数字和距离信息
        std::string info;
        if (result.digit > 0) {
            info = "Num:" + std::to_string(result.digit);
        } else {
            info = "Unknown";
        }
        
        if (result.pose.valid) {
            info += " Dist:" + std::to_string((int)result.pose.distance) + "mm";
        }
        
        cv::putText(image, info, result.armor_rect.center,
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        
        // 显示位姿信息
        if (result.pose.valid) {
            std::string pose_info = "Pitch:" + std::to_string((int)result.pose.pitch) + 
                                   " Yaw:" + std::to_string((int)result.pose.yaw);
            cv::putText(image, pose_info, 
                       cv::Point(result.armor_rect.center.x, result.armor_rect.center.y + 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }
    
    // 显示统计信息
    std::string stats = "Armors: " + std::to_string(results.size());
    cv::putText(image, stats, cv::Point(10, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
}

} // namespace armor_detection