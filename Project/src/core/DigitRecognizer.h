#ifndef DIGITRECOGNIZER_H
#define DIGITRECOGNIZER_H

#include <opencv2/opencv.hpp>
#include <string>

class DigitRecognizer
{
public:
    DigitRecognizer();
    bool loadModel(const std::string &model_path);
    int recognize(const cv::Mat &digit_roi);

private:
    cv::Ptr<cv::ml::SVM> svm_model;
    cv::HOGDescriptor hog;
};

#endif