#include "common.h"
#include "core/ArmorDetector.h"
#include "utils/ConfigLoader.h"
#include "utils/ImageUtils.h"
#include <iostream>
#include <chrono>
#include <filesystem>
#include <iomanip>

using namespace armor_detection;

// 检查文件是否存在
bool checkFileExists(const std::string& path) {
    bool exists = std::filesystem::exists(path);
    if (!exists) {
        std::cerr << "❌ 文件不存在: " << path << std::endl;
    }
    return exists;
}

// 创建输出目录
bool createOutputDirectory(const std::string& dir_path) {
    try {
        std::filesystem::create_directories(dir_path);
        std::cout << "✅ 创建输出目录: " << dir_path << std::endl;
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "❌ 无法创建输出目录: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "🚀 3D装甲板检测系统启动..." << std::endl;
    
    // 加载配置
    ConfigLoader config_loader;
    if (!config_loader.loadFromCommandLine(argc, argv)) {
        return 0; // 用户请求帮助信息
    }
    
    // 尝试加载配置文件（如果存在）
    std::string config_file = "/home/hz/T-DT_CampCon/Project/config/default_config.txt";
    if (checkFileExists(config_file)) {
        config_loader.loadFromFile(config_file);
    }
    
    auto config = config_loader.getConfig();
    config_loader.printConfig();
    
    // 检查必要文件
    if (!checkFileExists(config.model_path)) {
        return -1;
    }
    
    // 创建输出目录
    if (config.save_results && !createOutputDirectory(config.output_dir)) {
        return -1;
    }
    
    // 初始化装甲板检测器
    ArmorDetector detector;
    ArmorDetector::Config detector_config;
    detector_config.enable_pnp = config.enable_pnp;
    detector.setConfig(detector_config);
    
    if (!detector.init(config.model_path, config.camera_params_path)) {
        std::cerr << "❌ 检测器初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "✅ 系统初始化完成，开始处理..." << std::endl;
    
    // 打开视频源
    cv::VideoCapture cap;
    if (config.video_path.empty() || config.video_path == "0") {
        // 使用相机
        cap.open(config.camera_id);
        std::cout << "📷 使用相机: ID " << config.camera_id << std::endl;
    } else {
        // 使用视频文件
        cap.open(config.video_path);
        std::cout << "🎥 使用视频文件: " << config.video_path << std::endl;
    }
    
    if (!cap.isOpened()) {
        std::cerr << "❌ 无法打开视频源" << std::endl;
        return -1;
    }
    
    // 获取视频信息
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    
    std::cout << "📊 视频信息: " << frame_width << "x" << frame_height 
              << ", FPS: " << fps << ", 总帧数: " << total_frames << std::endl;
    
    cv::Mat frame;
    int frame_count = 0;
    auto total_start_time = std::chrono::high_resolution_clock::now();
    
    // 主处理循环
    while (true) {
        if (!cap.read(frame)) {
            if (frame_count == 0) {
                std::cerr << "❌ 无法读取第一帧" << std::endl;
                break;
            } else {
                std::cout << "✅ 视频处理完成" << std::endl;
                break;
            }
        }
        
        frame_count++;
        
        // 处理当前帧
        auto results = detector.processFrame(frame);
        
        // 绘制结果
        cv::Mat display_frame = frame.clone();
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
            
            int key = cv::waitKey(25);
            if (key == 27) { // ESC键退出
                std::cout << "⏹️  用户中断处理" << std::endl;
                break;
            } else if (key == ' ') { // 空格键暂停
                cv::waitKey(0);
            }
        }
        
        // 每10帧输出一次进度
        if (frame_count % 10 == 0) {
            std::cout << "📈 已处理 " << frame_count << " 帧" << std::endl;
        }
    }
    
    // 统计信息
    auto total_end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end_time - total_start_time).count() / 1000.0;
    
    double average_fps = frame_count / total_duration;
    
    std::cout << "\n🎉 处理完成!" << std::endl;
    std::cout << "📊 统计信息:" << std::endl;
    std::cout << "   总帧数: " << frame_count << std::endl;
    std::cout << "   总耗时: " << std::fixed << std::setprecision(2) << total_duration << " 秒" << std::endl;
    std::cout << "   平均FPS: " << std::fixed << std::setprecision(2) << average_fps << std::endl;
    
    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}