cmake_minimum_required(VERSION 3.10)
project(ArmorDetection)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(armor_detection
    main.cpp
    UnifiedRotatedRect.cpp
    DigitRecognizer.cpp
    ArmorDetector.cpp
)

target_link_libraries(armor_detection ${OpenCV_LIBS})