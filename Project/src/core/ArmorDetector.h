#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "DigitRecognizer.h" 

class ArmorDetector
{
public:
    // 颜色阈值
    static const cv::Scalar LR;
    static const cv::Scalar UR;

    struct DetectorConfig {
        double min_contour_area = 120;
        float max_angle_gap = 10;
        float max_len_gap_ratio = 1.0;
        float max_lengap_ratio = 0.8;
        float max_y_gap_ratio = 1.5;
        float min_x_gap_ratio = 0.8;
        float max_x_gap_ratio = 3.25;
        float min_ratio = 0.8;
        float max_ratio = 3.25;
    };

    ArmorDetector();
    bool init(const std::string& model_path = "armor_digit_1to5_svm.yml");
    void processFrame(const cv::Mat& src, cv::Mat& output);
    void setConfig(const DetectorConfig& config);

private:
    DigitRecognizer digit_recognizer_;
    DetectorConfig config_;
    
    void preprocessImage(const cv::Mat& src, cv::Mat& mask);
    std::vector<cv::RotatedRect> detectLightBars(const cv::Mat& mask);
    std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> pairLightBars(
        const std::vector<cv::RotatedRect>& light_bars);
    void drawResults(cv::Mat& image, 
                    const std::vector<cv::RotatedRect>& light_bars,
                    const std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>>& armor_pairs,
                    double fps);
};

#endif