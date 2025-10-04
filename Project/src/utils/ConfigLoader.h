#ifndef CONFIGLOADER_H
#define CONFIGLOADER_H

#include <string>
#include <map>

namespace armor_detection {

struct AppConfig {
    std::string video_path = "/home/hz/T-DT_CampCon/Project/data/test_video.avi";
    std::string model_path = "/home/hz/T-DT_CampCon/Project/data/svm_model.yml";
    std::string camera_params_path = "/home/hz/T-DT_CampCon/Project/data/camera_params.yml";
    std::string output_dir = "/home/hz/T-DT_CampCon/Project/output";
    bool enable_pnp = true;
    bool save_results = false;
    bool show_debug = true;
    int camera_id = 0;
};

class ConfigLoader {
public:
    ConfigLoader();
    
    // 从文件加载配置
    bool loadFromFile(const std::string& file_path);
    
    // 从命令行参数加载配置
    bool loadFromCommandLine(int argc, char* argv[]);
    
    // 获取配置
    const AppConfig& getConfig() const { return config_; }
    
    // 显示配置信息
    void printConfig() const;

private:
    AppConfig config_;
    
    void setDefaultValues();
    std::string getValueFromMap(const std::map<std::string, std::string>& config_map, 
                               const std::string& key, const std::string& default_val);
};

} // namespace armor_detection

#endif