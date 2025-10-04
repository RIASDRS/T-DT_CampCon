#!/bin/bash

# 装甲板检测系统运行脚本
# 用法: ./run.sh [参数]

set -e  # 遇到错误退出

echo "🚀 启动装甲板检测系统..."

# 进入构建目录
cd ../build/bin

# 检查可执行文件是否存在
if [ ! -f "armor_detection" ]; then
    echo "❌ 错误: 可执行文件不存在，请先运行 build.sh 构建项目"
    exit 1
fi

# 设置默认参数
DEFAULT_VIDEO="../data/test_video.avi"
DEFAULT_MODEL="../data/svm_model.yml"
DEFAULT_CAMERA_PARAMS="../data/camera_params.yml"

# 如果没有提供参数，使用默认值
if [ $# -eq 0 ]; then
    echo "📝 使用默认参数运行..."
    echo "   视频文件: $DEFAULT_VIDEO"
    echo "   模型文件: $DEFAULT_MODEL"
    echo "   相机参数: $DEFAULT_CAMERA_PARAMS"
    echo ""
    
    # 检查默认文件是否存在
    if [ ! -f "$DEFAULT_VIDEO" ]; then
        echo "⚠️  警告: 默认视频文件不存在: $DEFAULT_VIDEO"
        echo "💡 提示: 请提供视频文件路径参数"
        echo "用法: $0 --video <视频路径> [其他参数]"
        echo ""
        echo "或者使用相机:"
        echo "./run.sh --camera 0"
        exit 1
    fi
    
    ./armor_detection --video "$DEFAULT_VIDEO" --model "$DEFAULT_MODEL" --camera-params "$DEFAULT_CAMERA_PARAMS"
else
    # 使用提供的参数
    ./armor_detection "$@"
fi

echo ""
echo "👋 程序退出"