cmake_minimum_required(VERSION 3.10)
project(ArmorDetection3D)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 添加子目录
add_subdirectory(src)

# 创建数据目录
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/data)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/config)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/output)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Output directory: ${CMAKE_BINARY_DIR}/bin")