#include "DigitRecognizer.h"
#include <iostream>

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
    try
    {
        svm_model = cv::ml::SVM::load(model_path);
        std::cout << "✅ 数字识别模型加载成功: " << model_path << std::endl;
        return true;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "❌ 模型加载失败: " << e.what() << std::endl;
        return false;
    }
}

int DigitRecognizer::recognize(const cv::Mat &digit_roi)
{
    if (svm_model.empty())
    {
        std::cout << "❌ 模型未加载" << std::endl;
        return -1;
    }

    if (digit_roi.empty())
    {
        std::cout << "❌ 数字区域为空" << std::endl;
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

        std::cout << "🔢 识别结果: " << prediction << std::endl;
        return static_cast<int>(prediction);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "❌ 识别错误: " << e.what() << std::endl;
        return -1;
    }
}