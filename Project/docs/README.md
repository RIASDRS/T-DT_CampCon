project/                                 # 项目根目录
├── 📄 CMakeLists.txt                    # 根构建配置
├── 📁 build/                            # 构建目录（空，cmake生成）
├── 📁 src/                              # 源代码目录
│   ├── 📄 CMakeLists.txt                   # 源代码构建配置
│   ├── 📄 main.cpp                         # 主程序入口
│   ├── 📁 include/                         # 公共头文件
│   │   └── 📄 common.h                         # 通用定义和结构体
│   ├── 📁 core/                            # 核心算法模块
│   │   ├── 📄 CameraCalibrator.h/cpp           # 相机标定
│   │   ├── 📄 PnPSolver.h/cpp                  # PnP位姿解算
│   │   ├── 📄 ArmorDetector.h/cpp              # 装甲板检测主类
│   │   ├── 📄 LightBarDetector.h/cpp           # 灯条检测
│   │   ├── 📄 DigitRecognizer.h/cpp            # 数字识别
│   │   └── 📄 UnifiedRotatedRect.h/cpp         # 统一旋转矩形
│   └── 📁 utils/                           # 工具类
│       ├── 📄 ConfigLoader.h/cpp               # 配置加载
│       ├── 📄 ImageUtils.h/cpp                 # 图像工具
│       └── 📄 MathUtils.h/cpp                  # 数学工具
├── 📁 data/                             # 数据文件目录
│   ├── 📁 models/                          # 模型文件
│   │   └── 📄 armor_digit_1to5_svm.yml         # *必须：数字识别模型
│   ├── 📁 calibration/                     # 相机标定文件
│   │   └── 📄 camera_params.yml                # *必须：相机参数
│   ├── 📁 videos/                          # 测试视频
│   │   └── 📄 test_video.avi                   # 可选：测试视频
│   └── 📁 images/                          # 测试图像
│       └── ...                                 # 可选：测试图片
├── 📁 config/                           # 程序配置文件
│   ├── 📄 default_config.txt               # *必须：主配置文件
│   ├── 📄 development_config.txt           # 可选：开发环境配置
│   ├── 📄 production_config.txt            # 可选：生产环境配置
│   └── 📄 camera_config.txt                # 可选：相机专用配置
├── 📁 scripts/                          # 脚本文件
│   ├── 📄 build.sh                         # 构建脚本
│   ├── 📄 run.sh                           # 运行脚本
│   ├── 📄 run_camera.sh                    # 相机运行脚本
│   ├── 📄 run_video.sh                     # 视频运行脚本
│   ├── 📄 setup_directories.sh             # 目录设置脚本
│   ├── 📄 create_configs.sh                # 配置文件创建脚本
│   ├── 📄 test_system.sh                   # 系统测试脚本
│   └── 📄 clean.sh                         # 清理脚本
├── 📁 output/                           # 输出目录（程序生成）
│   └── ...                                 # 检测结果、日志等
└── 📁 logs/                             # 日志目录
    └── ...                                 # 运行日志文件

📋 文件用途总结
       文件	                  类型	   必须	   用途
default_config.txt	        文本配置	✅	程序运行参数
armor_digit_1to5_svm.yml	模型数据	✅	数字识别
camera_params.yml	        相机数据	✅	3D定位
development_config.txt	    文本配置	❌	开发调试
production_config.txt	    文本配置	❌	生产环境
camera_config.txt	        文本配置	❌	相机专用