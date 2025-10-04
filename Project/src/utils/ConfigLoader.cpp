#include "ConfigLoader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace armor_detection {

ConfigLoader::ConfigLoader() {
    setDefaultValues();
}

void ConfigLoader::setDefaultValues() {
    config_ = AppConfig(); //默认的值，在h文件中配置
}

bool ConfigLoader::loadFromFile(const std::string& file_path) { //从配置文件读取
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "❌ 无法打开配置文件: " << file_path << std::endl;
        return false;
    }
    
    std::map<std::string, std::string> config_map; //存在map里
    std::string line;
    
    while (std::getline(file, line)) {
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') continue;
        
        size_t pos = line.find('='); //用等号定位
        if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            
            // 去除空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            
            // 去除引号
            if (!value.empty() && value[0] == '"' && value[value.size()-1] == '"') {
                value = value.substr(1, value.size()-2);
            }
            
            config_map[key] = value;
        }
    }
    
    file.close(); //读取完毕
    
    // 更新配置
    config_.video_path = getValueFromMap(config_map, "video_path", config_.video_path);
    config_.model_path = getValueFromMap(config_map, "model_path", config_.model_path);
    config_.camera_params_path = getValueFromMap(config_map, "camera_params_path", config_.camera_params_path);
    config_.output_dir = getValueFromMap(config_map, "output_dir", config_.output_dir);
    
    std::string enable_pnp_str = getValueFromMap(config_map, "enable_pnp", "true");
    config_.enable_pnp = (enable_pnp_str == "true" || enable_pnp_str == "1");
    
    std::string show_debug_str = getValueFromMap(config_map, "show_debug", "true");
    config_.show_debug = (show_debug_str == "true" || show_debug_str == "1");
    
    std::string camera_id_str = getValueFromMap(config_map, "camera_id", "0");
    config_.camera_id = std::stoi(camera_id_str);
    
    std::cout << "✅ 配置文件加载成功: " << file_path << std::endl;
    return true;
}

bool ConfigLoader::loadFromCommandLine(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--video" && i + 1 < argc) {
            config_.video_path = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            config_.model_path = argv[++i];
        } else if (arg == "--camera-params" && i + 1 < argc) {
            config_.camera_params_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            config_.output_dir = argv[++i];
        } else if (arg == "--camera" && i + 1 < argc) {
            config_.camera_id = std::stoi(argv[++i]);
        } else if (arg == "--no-pnp") {
            config_.enable_pnp = false;
        } else if (arg == "--no-debug") {
            config_.show_debug = false;
        } else if (arg == "--save-results") {
            config_.save_results = true;
        } else if (arg == "--help") {
            std::cout << "用法: " << argv[0] << " [选项]" << std::endl;
            std::cout << "选项:" << std::endl;
            std::cout << "  --video <路径>         视频文件路径" << std::endl;
            std::cout << "  --model <路径>         模型文件路径" << std::endl;
            std::cout << "  --camera-params <路径> 相机参数文件路径" << std::endl;
            std::cout << "  --output <目录>        输出目录" << std::endl;
            std::cout << "  --camera <ID>          相机ID (默认: 0)" << std::endl;
            std::cout << "  --no-pnp               禁用PnP位姿解算" << std::endl;
            std::cout << "  --no-debug             禁用调试显示" << std::endl;
            std::cout << "  --save-results         保存结果图像" << std::endl;
            std::cout << "  --help                 显示此帮助信息" << std::endl;
            return false;
        }
    }
    
    return true;
}

std::string ConfigLoader::getValueFromMap(const std::map<std::string, std::string>& config_map, 
                                         const std::string& key, const std::string& default_val) {
    auto it = config_map.find(key);
    if (it != config_map.end()) {
        return it->second;
    }
    return default_val;
}

void ConfigLoader::printConfig() const {
    std::cout << "📋 当前配置:" << std::endl;
    std::cout << "  视频路径: " << config_.video_path << std::endl;
    std::cout << "  模型路径: " << config_.model_path << std::endl;
    std::cout << "  相机参数: " << config_.camera_params_path << std::endl;
    std::cout << "  输出目录: " << config_.output_dir << std::endl;
    std::cout << "  相机ID: " << config_.camera_id << std::endl;
    std::cout << "  PnP解算: " << (config_.enable_pnp ? "启用" : "禁用") << std::endl;
    std::cout << "  调试显示: " << (config_.show_debug ? "启用" : "禁用") << std::endl;
    std::cout << "  保存结果: " << (config_.save_results ? "是" : "否") << std::endl;
}

} // namespace armor_detection