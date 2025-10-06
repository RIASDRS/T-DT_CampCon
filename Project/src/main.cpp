#include "common.h"
#include "core/ArmorDetector.h"
#include "core/TrajectoryPredictor.h"
#include "utils/ConfigLoader.h"
#include "utils/ImageUtils.h"
#include <iostream>
#include <chrono>
#include <filesystem>
#include <iomanip>

using namespace armor_detection;

TrajectoryPredictor predictor(3);
int num_predictions = 5;

//3维点投射
void project3d(cv::Mat frame, std::vector<cv::Point3f> objectPoints){
    

    // 2. 定义相机内参矩阵 (示例参数，需要根据实际相机调整)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        1.7774091341308808e+03, 0., 7.1075979428865026e+02, 0., 
           1.7754170626354828e+03, 5.3472407285624729e+02, 0., 0., 1.);

    // 3. 定义畸变系数 (示例参数)
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -5.6313426428564950e-01, 1.8301501710641366e-01,
           1.9661478907901904e-03, 9.6259122849674621e-04,
           5.6883803390679100e-01);

    // 4. 定义相机姿态 (旋转向量和平移向量)
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);  // 旋转向量
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);  // 平移向量，Z方向移动使相机远离物体

    // 5. 投影三维点到二维图像平面
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // 7. 在图像上绘制投影点
    for (size_t i = 0; i < imagePoints.size(); i++) {
        cv::Point2f pt = imagePoints[i];
        
        // 绘制圆点
        cv::circle(frame, pt, 5, cv::Scalar(0, 0, 255), -1);  // 红色实心圆
        
        // 添加点编号
        cv::putText(frame, std::to_string(i + 1), 
                   cv::Point(pt.x + 10, pt.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 255, 255), 1);
    }
}

// 检查文件是否存在
bool checkFileExists(const std::string& path) {
    bool exists = std::filesystem::exists(path);
    if (!exists) {
        std::cerr << "[ERROR] 文件不存在: " << path << std::endl;
    }
    return exists;
}

// 创建输出目录
bool createOutputDirectory(const std::string& dir_path) {
    try {
        std::filesystem::create_directories(dir_path);
        std::cout << "[Done] 创建输出目录: " << dir_path << std::endl;
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "[ERROR] 无法创建输出目录: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "[Launched] 3D装甲板检测系统启动..." << std::endl;
    
    // 加载配置
    ConfigLoader config_loader;
    if (!config_loader.loadFromCommandLine(argc, argv)) { //在config—loader中的loadFromCommmandLine抓取命令行传递的配置参数
        return 0; // 用户请求帮助信息
    }
    
    // 尝试加载配置文件（如果存在）
    std::string config_file = "/home/hz/T-DT_CampCon/Project/config/default_config.txt";
    if (checkFileExists(config_file)) {
        config_loader.loadFromFile(config_file); //调用config-loader读取配置文件中的信息
    }
    
    auto config = config_loader.getConfig();  // 
    config_loader.printConfig();
    
    // 检查必要文件
    if (!checkFileExists(config.model_path)) {
        return -1;
    }
    
    // 如果没有输出目录，创建输出目录
    if (config.save_results && !createOutputDirectory(config.output_dir)) {
        return -1;
    }
    
    // 初始化装甲板检测器
    ArmorDetector detector;
    ArmorDetector::Config detector_config;
    detector_config.enable_pnp = config.enable_pnp;
    detector.setConfig(detector_config);
    
    if (!detector.init(config.model_path, config.camera_params_path)) {
        std::cerr << "[ERROR] 检测器初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "[Done] 系统初始化完成，开始处理..." << std::endl;
    
    // 打开视频源
    cv::VideoCapture cap;
    if (config.video_path.empty() || config.video_path == "0") {
        // 使用相机
        cap.open(config.camera_id);
        std::cout << "[Config] 使用相机: ID " << config.camera_id << std::endl;
    } else {
        // 使用视频文件
        cap.open(config.video_path);
        std::cout << "[Config] 使用视频文件: " << config.video_path << std::endl;
    }
    
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 无法打开视频源" << std::endl;
        return -1;
    }
    
    // 获取视频信息
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    
    std::cout << "[Info] 视频信息: " << frame_width << "x" << frame_height 
              << ", FPS: " << fps << ", 总帧数: " << total_frames << std::endl;
    
    cv::Mat frame;
    int frame_count = 0;
    auto total_start_time = std::chrono::high_resolution_clock::now();
    
    // 主处理循环
    while (true) {
        
        if (!cap.read(frame)) {
            if (frame_count == 0) {
                std::cerr << "[ERROR] 无法读取第一帧" << std::endl;
                break;
            } else {
                std::cout << "[Done] 视频处理完成" << std::endl;
                break;
            }
        }
        cv::Mat display_frame = frame.clone();//提前至此处用于3d
        frame_count++;
        
        // 处理当前帧
        auto results = detector.processFrame(frame);//获取vector<DetectionResult>
        for(const auto &detection_result : results)//遍历vector<DetectionResult>
        {
            std::cout << "[Info] center_point: " <<detection_result.digit << ":";
            PoseResult pose = detection_result.pose;
            for(const auto &center_point : pose.center_point)
            {
                predictor.addPoint(center_point.x,center_point.y,center_point.z, 5);
                std::cout <<center_point.x << ","<<center_point.y <<","<< center_point.z << std::endl;
            }
        }

        if (predictor.getPointCount()>=5){
            predictor.fit();
            auto predictions = predictor.predict(num_predictions);
            std::cout << "\n多项式拟合预测结果:" << std::endl;
            for (size_t i = 0; i < predictions.size(); i++) {
                std::cout << "预测点 " << i+1 << ": (" 
                    << predictions[i].x << ", " 
                    << predictions[i].y << ", " 
                    << predictions[i].z << ")" << std::endl;
                }       
            project3d(display_frame, predictions);
        }
    
        
        // 绘制结果
        //cv::Mat display_frame = frame.clone();
        detector.drawResults(display_frame, results);
        
        // 显示帧率信息
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - total_start_time).count() / 1000.0;
        
        double current_fps = frame_count / elapsed;
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps));
        cv::putText(display_frame, fps_text, cv::Point(10, frame_height - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        
        // 保存结果图像
        if (config.save_results && !results.empty()) {
            std::string filename = config.output_dir + "/frame_" + 
                                  std::to_string(frame_count) + ".jpg";
            ImageUtils::saveImage(display_frame, filename);
        }
        
        // 显示结果
        if (config.show_debug) {
            cv::imshow("3D Armor Detection", display_frame);
            
            int key = cv::waitKey(50);
            if (key == 27) { // ESC键退出
                std::cout << "[PAUSED]  用户中断处理" << std::endl;
                break;
            } else if (key == ' ') { // 空格键暂停
                cv::waitKey(0);
            }
        }
        
        // 每10帧输出一次进度
        if (frame_count % 10 == 0) {
            std::cout << "[Done] 已处理 " << frame_count << " 帧" << std::endl;
        }
    }
    
    // 统计信息
    auto total_end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end_time - total_start_time).count() / 1000.0;
    
    double average_fps = frame_count / total_duration;
    
    std::cout << "\n[Finished] 处理完成!" << std::endl;
    std::cout << "[Info] 统计信息:" << std::endl;
    std::cout << "   总帧数: " << frame_count << std::endl;
    std::cout << "   总耗时: " << std::fixed << std::setprecision(2) << total_duration << " 秒" << std::endl;
    std::cout << "   平均FPS: " << std::fixed << std::setprecision(2) << average_fps << std::endl;
    
    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}