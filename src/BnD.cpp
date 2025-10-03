#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>

// 对已有的RotatedRect进行统一化处理
cv::RotatedRect unifyRotatedRect(const cv::RotatedRect &rect)
{
    cv::Point2f center = rect.center;
    cv::Size2f size = rect.size;
    float angle = rect.angle;

    // 确保高度大于宽度
    if (size.width > size.height)
    {
        std::swap(size.width, size.height);
        angle = 90 - angle;
    }

    return cv::RotatedRect(center, size, angle);
}
class DigitRecognizer
{
public:
    DigitRecognizer()
    {
        hog = cv::HOGDescriptor(
            cv::Size(32, 32),
            cv::Size(16, 16),
            cv::Size(8, 8),
            cv::Size(8, 8),
            9);
    }

    bool loadModel(const std::string &model_path)
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

    int recognize(const cv::Mat &digit_roi)
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

private:
    cv::Ptr<cv::ml::SVM> svm_model;
    cv::HOGDescriptor hog;
};

cv::Scalar LR(1, 232, 167);
cv::Scalar UR(27, 255, 255);
cv::Mat src;
cv::Mat hsvimage;
cv::Mat mask;

int main()
{
    // 初始化数字识别器
    DigitRecognizer digit_recognizer;
    if (!digit_recognizer.loadModel("armor_digit_1to5_svm.yml"))
    {
        std::cerr << "❌ 无法加载数字识别模型!" << std::endl;
        return -1;
    }

    int frame_count = 0;
    double fps = 0;
    auto start_time = std::chrono::_V2::high_resolution_clock::now();
    cv::VideoCapture cap("/home/hz/T-DT_CampCon/src/test1.avi", cv::CAP_FFMPEG);
    int i = 0;
    while (true)
    {
        cap >> src;
        if (src.empty())
        {
            break;
        }

        frame_count++; // 计算帧数

        // 计算实时帧率（每秒更新）
        auto current_time = std::chrono::_V2::high_resolution_clock::now(); // 获取当前时间
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;

        if (elapsed >= 1.0)
        {                                // 每秒更新一次帧率显示
            fps = frame_count / elapsed; // 计算FPS
            frame_count = 0;             // 重置帧计数
            start_time = current_time;   // 重置开始时间

            // 在控制台输出帧率（可选）
            std::cout << "当前FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
        }

        // 在帧上绘制FPS信息
        std::stringstream ss;                                       // 用于存储FPS信息
        ss << "FPS: " << std::fixed << std::setprecision(2) << fps; // 保留两位小数

        if (cv::waitKey(1) == 27)
        {
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

        // 存储检测到的灯条
        std::vector<cv::RotatedRect> light_bars;

        for (int n = 0; n < contours.size(); n++)
        {
            if (cv::contourArea(contours[n]) > 120) // 面积过滤
            {
                // 最小外接矩形
                cv::RotatedRect rrect = minAreaRect(contours[n]);
                rrect = unifyRotatedRect(rrect);
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

                // 存储灯条
                light_bars.push_back(rrect);
            }
        }

        // 灯条配对成装甲板
        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armor_pairs;
        for (size_t i = 0; i < light_bars.size(); i++)
        {
            for (size_t j = i + 1; j < light_bars.size(); j++)
            {
                cv::RotatedRect &leftLight = light_bars[i];
                cv::RotatedRect &rightLight = light_bars[j];

                // 计算灯条长度（取较长的边作为长度）
                float leftLength = std::max(leftLight.size.width, leftLight.size.height);
                float rightLength = std::max(rightLight.size.width, rightLight.size.height);

                // 计算几何关系
                float angleGap = std::min(std::abs(leftLight.angle - rightLight.angle), std::abs(90 - std::abs(leftLight.angle - rightLight.angle)));
                float dis = cv::norm(leftLight.center - rightLight.center);
                float midLen = (leftLength + rightLength) / 2;

                // 各种比值计算
                float LenGap_ratio = std::abs(leftLength - rightLength) / std::max(leftLength, rightLength);
                float lengap_ratio = std::abs(leftLength - rightLength) / midLen;
                float yGap = std::abs(leftLight.center.y - rightLight.center.y);
                float yGap_ratio = yGap / midLen;
                float xGap = std::abs(leftLight.center.x - rightLight.center.x);
                float xGap_ratio = xGap / midLen;
                float ratio = dis / midLen;

                // 专业的装甲板配对条件
                if (angleGap > 10 ||
                    LenGap_ratio > 1.0 ||
                    lengap_ratio > 0.8 ||
                    yGap_ratio > 1.5 ||
                    xGap_ratio > 3.25 ||
                    xGap_ratio < 0.8 ||
                    ratio > 3.25 ||
                    ratio < 0.8)
                {
                    std::cout << "|❌| [wrong pair:" << "  leftLength" << leftLength << "  rightLength" << rightLength << "  dis" << dis << "  midLen" << midLen << "  angleGap" << angleGap << "  LenGap_ratio" << LenGap_ratio << "  yGap_ratio" << yGap_ratio << "  xGap_ratio" << xGap_ratio << "  ratio" << ratio << "  leftLight.angle" << leftLight.angle << "  rightLight.angle" << rightLight.angle << "]" << std::endl;
                    continue;
                }

                // 配对成功
                armor_pairs.push_back(std::make_pair(leftLight, rightLight));
                std::cout << "|✅| RIGHT PAIR";

                // ========== 绘制装甲板 ==========

                // 1. 绘制装甲板旋转矩形（红色）
                cv::Point2f armor_center = (leftLight.center + rightLight.center) * 0.5f;
                cv::RotatedRect armor_rect = cv::RotatedRect(
                    armor_center,
                    cv::Size2f(midLen * 2, dis),
                    (leftLight.angle + rightLight.angle) / 2);
                armor_rect = unifyRotatedRect(armor_rect);
                std::cout << "  Armor Angle" << (leftLight.angle + rightLight.angle) / 2;

                cv::Point2f vertices[4];
                armor_rect.points(vertices);
                for (int k = 0; k < 4; k++)
                {
                    cv::line(contourImage, vertices[k], vertices[(k + 1) % 4], cv::Scalar(0, 0, 255), 3);
                }

                // 2. 绘制装甲板中心点（蓝色）
                cv::circle(contourImage, armor_center, 6, cv::Scalar(255, 0, 0), -1);

                // 3. 绘制灯条连线（绿色虚线）
                cv::line(contourImage, leftLight.center, rightLight.center, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                cv::Rect digit_roi = armor_rect.boundingRect();
                digit_roi &= cv::Rect(0, 0, src.cols, src.rows);

                if (digit_roi.width > 10 && digit_roi.height > 10)
                {
                    cv::Mat armor_roi = src(digit_roi);
                    int digit = digit_recognizer.recognize(armor_roi);

                    if (digit > 0 && digit <= 5)
                    {
                        cv::putText(contourImage, std::to_string(digit),
                                    armor_center,
                                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 0), 2);
                    }

                    // 绘制ROI边界框（黄色）
                    cv::rectangle(contourImage, digit_roi, cv::Scalar(0, 255, 255), 2);
                }

                // 5. 在装甲板上方显示配对信息
                std::string info = "D:" + std::to_string((int)dis) + " L:" + std::to_string((int)midLen);
                cv::putText(contourImage, info,
                            cv::Point(armor_center.x - 40, armor_center.y - 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                // 数字识别
                if (digit_roi.x >= 0 && digit_roi.y >= 0 &&
                    digit_roi.x + digit_roi.width <= contourImage.cols &&
                    digit_roi.y + digit_roi.height <= contourImage.rows &&
                    digit_roi.width > 10 && digit_roi.height > 10)
                {

                    cv::Mat digit_image = src(digit_roi);
                    int digit = digit_recognizer.recognize(digit_image);

                    // 如果识别成功，显示数字
                    if (digit > 0 && digit <= 5)
                    {
                        cv::putText(contourImage, "Num:" + std::to_string(digit),
                                    cv::Point(armor_center.x - 20, armor_center.y + 50),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
                    }
                }
            }
        }

        // 在图像上显示检测统计信息
        std::string stats = "Lights: " + std::to_string(light_bars.size()) +
                            "  Armors: " + std::to_string(armor_pairs.size());
        cv::putText(contourImage, stats, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

        // 显示FPS信息
        cv::putText(contourImage, ss.str(), cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        imshow("BnD", contourImage);
        i++;
        std::cout << "FRAME== " << i << " ===================================" << std::endl;
        std::string image_path = std::to_string(i) + ".png";
        cv::imwrite(image_path, contourImage);
        cv::waitKey(25);
    }
}