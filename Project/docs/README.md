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


关系图
├── 📁 src/
│   ├── 📄 main.cpp
│   │   ├── 🔧 函数
│   │   │   ├── checkFileExists()
│   │   │   ├── createOutputDirectory()
│   │   │   └── main()
│   │   └── 🔗 调用关系
│   │       ├── → ConfigLoader (配置管理)
│   │       ├── → ArmorDetector (主检测器)
│   │       └── → ImageUtils (图像工具)
│   │
│   ├── 📁 include/
│   │   └── 📄 common.h
│   │       ├── 🏷️  命名空间: armor_detection
│   │       ├── 📊 结构体
│   │       │   ├── Point3D (3D坐标)
│   │       │   ├── Point2D (2D坐标)
│   │       │   ├── ArmorModel (装甲板尺寸)
│   │       │   ├── CameraParams (相机参数)
│   │       │   ├── PoseResult (位姿结果)
│   │       │   ├── LightBarPair (灯条配对)
│   │       │   └── DetectionResult (检测结果)
│   │       └── 📏 常量
│   │           └── WIDTH/HEIGHT (装甲板物理尺寸)
│   │
│   ├── 📁 core/ (核心算法模块)
│   │   ├── 📄 ArmorDetector.h/cpp
│   │   │   ├── 🏗️  类: ArmorDetector
│   │   │   │   ├── 🔧 公有方法
│   │   │   │   │   ├── init() - 初始化
│   │   │   │   │   ├── processFrame() - 处理帧
│   │   │   │   │   ├── setConfig() - 设置配置
│   │   │   │   │   └── drawResults() - 绘制结果
│   │   │   │   ├── 🔒 私有方法
│   │   │   │   │   ├── createArmorResult() - 创建装甲板结果
│   │   │   │   │   ├── extractArmorPoints() - 提取特征点
│   │   │   │   │   ├── recognizeDigit() - 数字识别
│   │   │   │   │   └── calculatePose() - 计算位姿
│   │   │   │   └── 📦 成员变量
│   │   │   │       ├── light_bar_detector_ (灯条检测器)
│   │   │   │       ├── digit_recognizer_ (数字识别器)
│   │   │   │       ├── pnp_solver_ (PnP解算器)
│   │   │   │       ├── camera_calibrator_ (相机标定器)
│   │   │   │       └── latest_results_ (最新结果)
│   │   │   └── 🔗 依赖关系
│   │   │       ├── ← common.h (结构体定义)
│   │   │       ├── → LightBarDetector (灯条检测)
│   │   │       ├── → DigitRecognizer (数字识别)
│   │   │       ├── → PnPSolver (位姿解算)
│   │   │       └── → CameraCalibrator (相机标定)
│   │   │
│   │   ├── 📄 LightBarDetector.h/cpp
│   │   │   ├── 🏗️  类: LightBarDetector
│   │   │   │   ├── 🔧 方法
│   │   │   │   │   ├── detect() - 检测灯条
│   │   │   │   │   ├── pairLightBars() - 配对灯条
│   │   │   │   │   └── setConfig() - 设置配置
│   │   │   │   ├── 🔒 私有方法
│   │   │   │   │   ├── preprocessImage() - 图像预处理
│   │   │   │   │   ├── isValidLightBar() - 灯条验证
│   │   │   │   │   └── isValidPair() - 配对验证
│   │   │   │   └── 📦 成员变量
│   │   │   │       └── config_ (配置参数)
│   │   │   └── 🔗 依赖关系
│   │   │       ├── ← common.h (结构体定义)
│   │   │       └── → UnifiedRotatedRect (旋转矩形处理)
│   │   │
│   │   ├── 📄 DigitRecognizer.h/cpp
│   │   │   ├── 🏗️  类: DigitRecognizer
│   │   │   │   ├── 🔧 方法
│   │   │   │   │   ├── loadModel() - 加载模型
│   │   │   │   │   └── recognize() - 数字识别
│   │   │   │   └── 📦 成员变量
│   │   │   │       ├── svm_model_ (SVM模型)
│   │   │   │       └── hog_ (HOG特征提取器)
│   │   │   └── 🔗 依赖关系
│   │   │       └── ← OpenCV (机器学习)
│   │   │
│   │   ├── 📄 PnPSolver.h/cpp
│   │   │   ├── 🏗️  类: PnPSolver
│   │   │   │   ├── 🔧 方法
│   │   │   │   │   ├── setCameraParams() - 设置相机参数
│   │   │   │   │   ├── solveArmorPose() - 解算装甲板位姿
│   │   │   │   │   ├── solveLightBarPose() - 解算灯条位姿
│   │   │   │   │   └── calculateReprojectionError() - 计算重投影误差
│   │   │   │   ├── 🔒 私有方法
│   │   │   │   │   ├── generateArmorModelPoints() - 生成3D模型点
│   │   │   │   │   └── generateLightBarModelPoints() - 生成灯条模型点
│   │   │   │   └── 📦 成员变量
│   │   │   │       ├── camera_params_ (相机参数)
│   │   │   │       └── params_loaded_ (参数加载标志)
│   │   │   └── 🔗 依赖关系
│   │   │       ├── ← common.h (结构体定义)
│   │   │       └── → OpenCV (PnP解算)
│   │   │
│   │   ├── 📄 CameraCalibrator.h/cpp
│   │   │   ├── 🏗️  类: CameraCalibrator
│   │   │   │   ├── 🔧 方法
│   │   │   │   │   ├── loadParams() - 加载参数
│   │   │   │   │   ├── saveParams() - 保存参数
│   │   │   │   │   ├── calibrate() - 标定相机
│   │   │   │   │   └── undistortImage() - 去畸变
│   │   │   │   ├── 🔒 私有方法
│   │   │   │   │   └── findChessboardCorners() - 查找棋盘格角点
│   │   │   │   └── 📦 成员变量
│   │   │   │       └── params_ (相机参数)
│   │   │   └── 🔗 依赖关系
│   │   │       └── ← common.h (结构体定义)
│   │   │
│   │   └── 📄 UnifiedRotatedRect.h/cpp
│   │       ├── 🔧 函数
│   │       │   └── unifyRotatedRect() - 统一旋转矩形
│   │       └── 🔗 依赖关系
│   │           └── ← OpenCV (几何处理)
│   │
│   └── 📁 utils/ (工具模块)
│       ├── 📄 ConfigLoader.h/cpp
│       │   ├── 🏗️  类: ConfigLoader
│       │   │   ├── 🔧 方法
│       │   │   │   ├── loadFromFile() - 从文件加载
│       │   │   │   ├── loadFromCommandLine() - 从命令行加载
│       │   │   │   ├── getConfig() - 获取配置
│       │   │   │   └── printConfig() - 打印配置
│       │   │   ├── 🔒 私有方法
│       │   │   │   ├── setDefaultValues() - 设置默认值
│       │   │   │   └── getValueFromMap() - 从map获取值
│       │   │   └── 📦 成员变量
│       │   │       └── config_ (应用配置)
│       │   └── 📊 结构体
│       │       └── AppConfig (应用配置)
│       │
│       ├── 📄 ImageUtils.h/cpp
│       │   ├── 🔧 静态方法
│       │   │   ├── resizeImage() - 调整大小
│       │   │   ├── enhanceContrast() - 增强对比度
│       │   │   ├── extractROI() - 提取ROI
│       │   │   ├── drawRotatedRect() - 绘制旋转矩形
│       │   │   ├── drawPoints() - 绘制点
│       │   │   ├── saveImage() - 保存图像
│       │   │   └── createDebugImage() - 创建调试图像
│       │   └── 🔗 依赖关系
│       │       └── ← OpenCV (图像处理)
│       │
│       └── 📄 MathUtils.h/cpp
│           ├── 🔧 静态方法
│           │   ├── distance() - 计算距离
│           │   ├── angleDifference() - 计算角度差
│           │   ├── deg2rad()/rad2deg() - 角度弧度转换
│           │   ├── vectorAngle() - 计算向量角度
│           │   ├── imageToWorld() - 图像到世界坐标
│           │   ├── midpoint() - 计算中点
│           │   └── rotatedRectArea() - 计算旋转矩形面积
│           └── 🔗 依赖关系
│               └── ← OpenCV (数学计算)
│
├── 📁 config/ (配置文件)
│   ├── 📄 default_config.txt
│   ├── 📄 development_config.txt
│   ├── 📄 production_config.txt
│   └── 📄 camera_config.txt
│
├── 📁 data/ (数据文件)
│   ├── 📁 models/
│   │   └── 📄 armor_digit_1to5_svm.yml
│   ├── 📁 calibration/
│   │   └── 📄 camera_params.yml
│   ├── 📁 videos/
│   │   └── 📄 test_video.avi
│   └── 📁 images/
│
├── 📁 scripts/ (脚本文件)
│   ├── 📄 build.sh
│   ├── 📄 run.sh
│   ├── 📄 run_camera.sh
│   ├── 📄 run_video.sh
│   ├── 📄 setup_directories.sh
│   ├── 📄 create_configs.sh
│   ├── 📄 test_system.sh
│   └── 📄 clean.sh
│
└── 📁 output/ (输出目录)
    ├── 📁 debug/
    ├── 📁 production/
    └── 📁 camera_live/