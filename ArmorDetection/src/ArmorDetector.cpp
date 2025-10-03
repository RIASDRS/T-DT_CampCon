#include "ArmorDetector.h"
#include "UnifiedRotatedRect.h"
#include "DigitRecognizer.h"
#include <iostream>
#include <sstream>    // 添加这个头文件用于std::stringstream
#include <iomanip>    // 添加这个头文件用于std::setprecision
#include <algorithm>  // 添加这个头文件用于std::max, std::min, std::swap
// 静态成员初始化
const cv::Scalar ArmorDetector::LR(1, 232, 167);
const cv::Scalar ArmorDetector::UR(27, 255, 255);

ArmorDetector::ArmorDetector() {}

bool ArmorDetector::init(const std::string& model_path)
{
    return digit_recognizer_.loadModel(model_path);
}

void ArmorDetector::setConfig(const DetectorConfig& config)
{
    config_ = config;
}

void ArmorDetector::preprocessImage(const cv::Mat& src, cv::Mat& mask)
{
    cv::Mat hsvimage;
    cv::cvtColor(src, hsvimage, cv::COLOR_BGR2HSV);
    cv::inRange(hsvimage, LR, UR, mask);
    cv::GaussianBlur(mask, mask, cv::Size(3, 3), 10, 20);
    
    cv::Mat struct1 = cv::getStructuringElement(0, cv::Size(3, 3));
    cv::dilate(mask, mask, struct1);
}

std::vector<cv::RotatedRect> ArmorDetector::detectLightBars(const cv::Mat& mask)
{
    std::vector<cv::RotatedRect> light_bars;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t n = 0; n < contours.size(); n++)
    {
        if (cv::contourArea(contours[n]) > config_.min_contour_area)
        {
            cv::RotatedRect rrect = cv::minAreaRect(contours[n]);
            rrect = unifyRotatedRect(rrect);
            light_bars.push_back(rrect);
        }
    }
    
    return light_bars;
}

std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> ArmorDetector::pairLightBars(
    const std::vector<cv::RotatedRect>& light_bars)
{
    std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armor_pairs;
    
    for (size_t i = 0; i < light_bars.size(); i++)
    {
        for (size_t j = i + 1; j < light_bars.size(); j++)
        {
            const cv::RotatedRect &leftLight = light_bars[i];
            const cv::RotatedRect &rightLight = light_bars[j];

            // 计算灯条长度
            float leftLength = std::max(leftLight.size.width, leftLight.size.height);
            float rightLength = std::max(rightLight.size.width, rightLight.size.height);

            // 计算几何关系
            float angleGap = std::min(std::abs(leftLight.angle - rightLight.angle), 
                                    std::abs(90 - std::abs(leftLight.angle - rightLight.angle)));
            float dis = cv::norm(leftLight.center - rightLight.center);
            float midLen = (leftLength + rightLength) / 2;

            // 各种比值计算
            float LenGap_ratio = std::abs(leftLength - rightLength) / std::max(leftLength, rightLength);
            float lengap_ratio = std::abs(leftLength - rightLength) / midLen;
            float yGap = std::abs(leftLight.center.y - rightLight.center.y);
            float yGap_ratio = yGap / midLen;
            float xGap = std::abs(leftLight.center.x - rightLight.center.x);
            float xGap_ratio = xGap / midLen;
            float ratio = dis / midLen;

            // 装甲板配对条件
            if (angleGap > config_.max_angle_gap ||
                LenGap_ratio > config_.max_len_gap_ratio ||
                lengap_ratio > config_.max_lengap_ratio ||
                yGap_ratio > config_.max_y_gap_ratio ||
                xGap_ratio > config_.max_x_gap_ratio ||
                xGap_ratio < config_.min_x_gap_ratio ||
                ratio > config_.max_ratio ||
                ratio < config_.min_ratio)
            {
                std::cout << "|❌| [wrong pair:" << "  leftLength" << leftLength << "  rightLength" << rightLength 
                          << "  dis" << dis << "  midLen" << midLen << "  angleGap" << angleGap 
                          << "  LenGap_ratio" << LenGap_ratio << "  yGap_ratio" << yGap_ratio 
                          << "  xGap_ratio" << xGap_ratio << "  ratio" << ratio 
                          << "  leftLight.angle" << leftLight.angle << "  rightLight.angle" << rightLight.angle << "]" << std::endl;
                continue;
            }

            // 配对成功
            armor_pairs.push_back(std::make_pair(leftLight, rightLight));
            std::cout << "|✅| RIGHT PAIR";
        }
    }
    
    return armor_pairs;
}

void ArmorDetector::drawResults(cv::Mat& image, 
                               const std::vector<cv::RotatedRect>& light_bars,
                               const std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>>& armor_pairs,
                               double fps)
{
    // 绘制灯条
    for (const auto& light : light_bars)
    {
        cv::Point2f points[4];
        light.points(points);
        for (int i = 0; i < 4; i++)
        {
            if (i == 3)
            {
                cv::line(image, points[i], points[0], cv::Scalar(0, 255, 0), 2, 8);
                break;
            }
            cv::line(image, points[i], points[i + 1], cv::Scalar(0, 255, 0), 2, 8);
        }
        cv::circle(image, light.center, 2, cv::Scalar(255, 0, 0), 2, 8);
    }

    // 绘制装甲板
    for (const auto& armor_pair : armor_pairs)
    {
        const cv::RotatedRect& leftLight = armor_pair.first;
        const cv::RotatedRect& rightLight = armor_pair.second;
        
        cv::Point2f armor_center = (leftLight.center + rightLight.center) * 0.5f;
        float leftLength = std::max(leftLight.size.width, leftLight.size.height);
        float rightLength = std::max(rightLight.size.width, rightLight.size.height);
        float midLen = (leftLength + rightLength) / 2;
        float dis = cv::norm(leftLight.center - rightLight.center);
        
        // 绘制装甲板矩形
        cv::RotatedRect armor_rect = cv::RotatedRect(
            armor_center,
            cv::Size2f(midLen * 2, dis),
            (leftLight.angle + rightLight.angle) / 2);
        armor_rect = unifyRotatedRect(armor_rect);

        cv::Point2f vertices[4];
        armor_rect.points(vertices);
        for (int k = 0; k < 4; k++)
        {
            cv::line(image, vertices[k], vertices[(k + 1) % 4], cv::Scalar(0, 0, 255), 3);
        }

        // 绘制中心点和连线
        cv::circle(image, armor_center, 6, cv::Scalar(255, 0, 0), -1);
        cv::line(image, leftLight.center, rightLight.center, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // 数字识别和显示
        cv::Rect digit_roi = armor_rect.boundingRect();
        digit_roi &= cv::Rect(0, 0, image.cols, image.rows);

        if (digit_roi.width > 10 && digit_roi.height > 10)
        {
            cv::Mat armor_roi = image(digit_roi);
            int digit = digit_recognizer_.recognize(armor_roi);

            if (digit > 0 && digit <= 5)
            {
                cv::putText(image, std::to_string(digit), armor_center,
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 0), 2);
            }
            cv::rectangle(image, digit_roi, cv::Scalar(0, 255, 255), 2);
        }

        // 显示信息
        std::string info = "D:" + std::to_string((int)dis) + " L:" + std::to_string((int)midLen);
        cv::putText(image, info, cv::Point(armor_center.x - 40, armor_center.y - 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    // 显示统计信息
    std::string stats = "Lights: " + std::to_string(light_bars.size()) +
                        "  Armors: " + std::to_string(armor_pairs.size());
    cv::putText(image, stats, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

    std::stringstream ss;
    ss << "FPS: " << std::fixed << std::setprecision(2) << fps;
    cv::putText(image, ss.str(), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}

void ArmorDetector::processFrame(const cv::Mat& src, cv::Mat& output)
{
    output = src.clone();
    
    // 预处理
    cv::Mat mask;
    preprocessImage(src, mask);
    
    // 检测灯条
    auto light_bars = detectLightBars(mask);
    
    // 配对灯条
    auto armor_pairs = pairLightBars(light_bars);
    
    // 绘制结果
    drawResults(output, light_bars, armor_pairs, 0); // FPS需要在外部计算
}