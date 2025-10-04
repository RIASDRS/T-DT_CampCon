#include "DigitRecognizer.h"
#include <iostream>
#include <filesystem>

namespace armor_detection {

DigitRecognizer::DigitRecognizer()
{
    hog = cv::HOGDescriptor(
        cv::Size(32, 32),
        cv::Size(16, 16),
        cv::Size(8, 8),
        cv::Size(8, 8),
        9);
}

bool DigitRecognizer::loadModel(const std::string &model_path)
{
    // 首先检查文件是否存在
    if (!std::filesystem::exists(model_path)) {
        std::cerr << "[ERROR] 模型文件不存在: " << model_path << std::endl;
        return false;
    }
    
    try
    {
        svm_model = cv::ml::SVM::load(model_path);
        std::cout << "[Done] 数字识别模型加载成功: " << model_path << std::endl;
        return true;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "[ERROR] 模型加载失败: " << e.what() << std::endl;
        return false;
    }
}

int DigitRecognizer::recognize(const cv::Mat &digit_roi)
{
    if (svm_model.empty()) {
        return -1;
    }

    if (digit_roi.empty()) {
        return -1;
    }

    try
    {
        // 预处理
        cv::Mat processed;
        if (digit_roi.channels() == 3)
        {
            cv::cvtColor(digit_roi, processed, cv::COLOR_BGR2GRAY);
        }
        else
        {
            processed = digit_roi.clone();
        }

        cv::resize(processed, processed, cv::Size(32, 32));
        cv::threshold(processed, processed, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // 提取HOG特征
        std::vector<float> descriptors;
        hog.compute(processed, descriptors);

        // 预测
        cv::Mat feature = cv::Mat(descriptors).t();
        feature.convertTo(feature, CV_32F);
        float prediction = svm_model->predict(feature);

        return static_cast<int>(prediction);
    }
    catch (const cv::Exception &e)
    {
        return -1;
    }
}

} // namespace armor_detection