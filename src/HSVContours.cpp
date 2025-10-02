#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <chrono>

cv::Scalar LR(1, 232, 167);
cv::Scalar UR(27, 255, 255);
cv::Mat src;
cv::Mat hsvimage;
cv::Mat mask;

int main()
{
    int frame_count = 0;
    double fps = 0;
    auto start_time = std::chrono::_V2::high_resolution_clock::now(); 
    cv::VideoCapture cap("/home/hz/T-DT_CampCon/src/test1.avi", cv::CAP_ANY);
    while (true)
    {
        cap >> src;
        if (src.empty())
        {
            break;
        }

        frame_count++;    // 计算帧数
        
                          // 计算实时帧率（每秒更新）
        auto current_time = std::chrono::_V2::high_resolution_clock::now();          // 获取当前时间
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;
        
        if (elapsed >= 1.0) { // 每秒更新一次帧率显示
            fps = frame_count / elapsed;            // 计算FPS
            frame_count = 0;                        // 重置帧计数
            start_time = current_time;              // 重置开始时间
            
            // 在控制台输出帧率（可选）
            std::cout << "当前FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
        }
        
            // 在帧上绘制FPS信息
        std::stringstream ss;  // 用于存储FPS信息
        ss << "FPS: " << std::fixed << std::setprecision(2) << fps; // 保留两位小数
        
        if (cv::waitKey(1) == 27) {
            break;
        }
        cv::cvtColor(src, hsvimage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvimage, LR, UR, mask);
        cv::GaussianBlur(mask, mask, cv::Size(3, 3), 10, 20);
        cv::Mat struct1, struct2;
        struct1 = cv::getStructuringElement(0, cv::Size(3, 3));
        cv::dilate(mask, mask, struct1);
        std::vector<std::vector<cv::Point>> contours;                                            // 轮廓点集
        std::vector<cv::Vec4i> hierarchy;                                                        // 轮廓层级
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 查找轮廓
        cv::Mat contourImage = src.clone();
        // cv::drawContours(contourImage, contours, -1, cv::Scalar(0, 255, 0), 2); // 画出所有轮廓
        //  cv::imshow("Contours", contourImage); // 显示轮廓图像

        for (int n = 0; n < contours.size(); n++)
        {
            if (cv::contourArea(contours[n]) > 30) // 面积过滤
            {
                // 最小外接矩形
                cv::RotatedRect rrect = minAreaRect(contours[n]);
                cv::Point2f points[4];
                rrect.points(points);           // 读取最小外接矩形的 4 个顶点
                cv::Point2f cpt = rrect.center; // 最小外接矩形的中心

                // 绘制旋转矩形与中心位置
                for (int i = 0; i < 4; i++)
                {
                    if (i == 3)
                    {
                        line(contourImage, points[i], points[0], cv::Scalar(0, 255, 0), 2, 8, 0);
                        break;
                    }
                    line(contourImage, points[i], points[i + 1], cv::Scalar(0, 255, 0), 2, 8, 0);
                }
                // 绘制矩形的中心
                circle(contourImage, cpt, 2, cv::Scalar(255, 0, 0), 2, 8, 0);
            }
        }

        
        cv::putText(contourImage, ss.str(), cv::Point(40, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);//BGR:x坐标450，y坐标40，字体大小1.0，颜色白色，粗细2

        imshow("min", contourImage);
        cv::waitKey(25);
    }
}