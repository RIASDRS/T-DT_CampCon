#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

int main(){
    // cv::Mat UR,LR,UB,LB;
    // UR = cv::imread("../UpperRed.png",cv::IMREAD_COLOR);
    // LR = cv::imread("../LowerRed.png",cv::IMREAD_COLOR);
    // cv::cvtColor(UR,UR,cv::COLOR_BGR2HSV);
    // cv::cvtColor(LR,LR,cv::COLOR_BGR2HSV);
    // std::cout << UR << "+++++++++++++++++++++++" << LR;

     cv::Mat src = cv::imread("../LR.png", cv::IMREAD_COLOR);
    if (src.empty())
    {
        std::cout << "无法加载图像: " << "LR.png" << std::endl;
        return -1;
    }
    std::cout << "成功加载";
    cv::imshow("Original_Image", src);
    cv::waitKey(0);
    
    cv::Mat hsvimage;
    cv::cvtColor(src, hsvimage, cv::COLOR_BGR2HSV);
    cv::imshow("HSV Image", hsvimage);
    std::cout << hsvimage << std::endl;
    cv::waitKey(0);
}