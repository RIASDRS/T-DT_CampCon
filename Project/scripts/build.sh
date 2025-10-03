#!/bin/bash

# 装甲板检测系统构建脚本
# 用法: ./build.sh [Debug|Release]

set -e  # 遇到错误退出

echo "🔨 开始构建装甲板检测系统..."

# 获取构建类型
BUILD_TYPE="Release"
if [ $# -gt 0 ]; then
    BUILD_TYPE="$1"
fi

# 检查构建类型是否有效
if [ "$BUILD_TYPE" != "Debug" ] && [ "$BUILD_TYPE" != "Release" ]; then
    echo "❌ 错误: 构建类型必须是 Debug 或 Release"
    echo "用法: $0 [Debug|Release]"
    exit 1
fi

# 创建构建目录
echo "📁 创建构建目录..."
mkdir -p ../build
cd ../build

# 清理之前的构建（可选）
# echo "🧹 清理构建目录..."
# rm -rf ./*

# 配置CMake
echo "⚙️  配置CMake (构建类型: $BUILD_TYPE)..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..

# 编译
echo "📦 编译项目..."
make -j$(nproc)

# 检查构建是否成功
if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 构建成功！"
    echo "📊 构建信息:"
    echo "   构建类型: $BUILD_TYPE"
    echo "   输出目录: $(pwd)/bin/"
    echo "   可执行文件: $(pwd)/bin/armor_detection"
    echo ""
    echo "🚀 运行命令:"
    echo "   ./bin/armor_detection --video ../data/test_video.avi"
    echo "   ./bin/armor_detection --camera 0"
else
    echo "❌ 构建失败！"
    exit 1
fi