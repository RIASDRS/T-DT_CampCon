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
    std::string Path = "/home/hz/T-DT_CampCon/src/test2.avi";
    cv::VideoCapture cap(Path, cv::CAP_ANY);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video." << std::endl;
        return -1;
    }
    cv::Mat frame;
    cv::Scalar lowerb(100);
    cv::Scalar upperb(255);
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break; // End of video
        }
        cv::Mat channels[3];
        cv::split(frame, channels);
        cv::Mat GrayRed;
        cv::Mat GrayGreen;
        cv::Mat GrayBlue;
        cv::Mat Cal;
        // X色灰度减去绿色灰度
        //cv::subtract(channels[0], channels[1], Cal);
        cv::subtract(channels[0], channels[1], Cal);
        
        // 归一化到0-255
        //cv::normalize(RedCal, RedCal, 0, 255, cv::NORM_MINMAX);
        cv::inRange(Cal, 100,255,Cal);
        // 转换为3通道图像以便显示
        cv::cvtColor(channels[2], GrayRed, cv::COLOR_GRAY2BGR);
        cv::cvtColor(channels[1], GrayGreen, cv::COLOR_GRAY2BGR);
        cv::cvtColor(channels[0], GrayBlue, cv::COLOR_GRAY2BGR);
        
        std::vector<std::vector<cv::Point>> contours;                                            // 轮廓点集
        std::vector<cv::Vec4i> hierarchy;                                                        // 轮廓层级
        cv::findContours(Cal, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::cvtColor(Cal, Cal, cv::COLOR_GRAY2BGR);

        cv::Mat contourImage = Cal.clone();       
        cv::drawContours(contourImage, contours, -1, cv::Scalar(0, 0, 255), 2);

        cv::imshow("Video Frame RED", GrayRed);
        cv::imshow("Video Frame GREEN", GrayGreen);
        cv::imshow("Video Frsme BLUE", GrayBlue);
        cv::imshow("Video Frame CAL", Cal);
        cv::imshow("Video Frame CNTRS",contourImage);

        if (cv::waitKey(30) >= 0) {
            break; // Exit on any key
        }
    }
    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}