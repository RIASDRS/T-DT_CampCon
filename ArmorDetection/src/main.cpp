#include "ArmorDetector.h"
#include "UnifiedRotatedRect.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>    // 添加这个头文件用于std::stringstream

int main()
{
    ArmorDetector detector;
    if (!detector.init("armor_digit_1to5_svm.yml"))
    {
        std::cerr << "❌ 无法加载数字识别模型!" << std::endl;
        return -1;
    }

    int frame_count = 0;
    double fps = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    cv::VideoCapture cap("/home/hz/T-DT_CampCon/ArmorDetection/src/test1.avi", cv::CAP_FFMPEG);
    
    if (!cap.isOpened())
    {
        std::cerr << "❌ 无法打开视频文件!" << std::endl;
        return -1;
    }

    int i = 0;
    while (true)
    {
        cv::Mat src;
        cap >> src;
        if (src.empty())
        {
            break;
        }

        frame_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;

        if (elapsed >= 1.0)
        {
            fps = frame_count / elapsed;
            frame_count = 0;
            start_time = current_time;
            std::cout << "当前FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
        }

        cv::Mat result;
        detector.processFrame(src, result);

        cv::imshow("Armor Detection", result);
        
        i++;
        std::cout << "FRAME== " << i << " ===================================" << std::endl;
        
        if (cv::waitKey(25) == 27)
        {
            break;
        }
    }

    return 0;
}