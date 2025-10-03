#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "common.h"
#include "LightBarDetector.h"
#include "DigitRecognizer.h"
#include "PnPSolver.h"
#include "CameraCalibrator.h"
#include <vector>

namespace armor_detection {

class ArmorDetector {
public:
    struct Config {
        bool enable_pnp = true;
        bool show_debug = true;
        bool save_results = false;
    };

    ArmorDetector();
    
    // 初始化
    bool init(const std::string& model_path = "../../data/svm_model.yml",
              const std::string& camera_params_path = "../../data/camera_params.yml");
    
    // 处理单帧图像
    std::vector<DetectionResult> processFrame(const cv::Mat& frame);
    
    // 设置配置
    void setConfig(const Config& config) { config_ = config; }
    
    // 获取最新结果
    const std::vector<DetectionResult>& getLatestResults() const { return latest_results_; }
    
    // 绘制检测结果
    void drawResults(cv::Mat& image, const std::vector<DetectionResult>& results);

private:
    LightBarDetector light_bar_detector_;
    DigitRecognizer digit_recognizer_;
    PnPSolver pnp_solver_;
    CameraCalibrator camera_calibrator_;
    Config config_;
    
    std::vector<DetectionResult> latest_results_;
    
    // 从灯条对创建装甲板结果
    DetectionResult createArmorResult(const LightBarPair& pair, const cv::Mat& frame);
    
    // 提取装甲板特征点
    std::vector<cv::Point2f> extractArmorPoints(const LightBarPair& pair);
    
    // 数字识别
    int recognizeDigit(const cv::Mat& frame, const DetectionResult& armor_result);
    
    // 计算装甲板3D位姿
    void calculatePose(DetectionResult& result);
};

} // namespace armor_detection

#endif