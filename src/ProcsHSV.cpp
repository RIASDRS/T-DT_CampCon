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
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break; // End of video
        }
        cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        cv::imshow("Video Frame", frame);
        if (cv::waitKey(30) >= 0) {
            break; // Exit on any key
        }
    }
    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}