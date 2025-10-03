# 源代码目录 CMakeLists.txt

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# 源文件列表
set(CORE_SOURCES
    core/CameraCalibrator.cpp
    core/PnPSolver.cpp
    core/ArmorDetector.cpp
    core/LightBarDetector.cpp
    core/DigitRecognizer.cpp
    core/UnifiedRotatedRect.cpp
)

set(UTILS_SOURCES
    utils/ConfigLoader.cpp
    utils/ImageUtils.cpp
    utils/MathUtils.cpp
)

# 主可执行文件
add_executable(armor_detection_3d
    main.cpp
    ${CORE_SOURCES}
    ${UTILS_SOURCES}
)

# 链接库
target_link_libraries(armor_detection_3d ${OpenCV_LIBS})

# 设置可执行文件属性
set_target_properties(armor_detection_3d PROPERTIES
    OUTPUT_NAME "armor_detection"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "OpenCV found: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")