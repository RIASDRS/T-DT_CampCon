 
cmake_minimum_required(VERSION 3.0.0)
project(ts)

find_package(OpenCV 4.7 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ts BnD.cpp)

target_link_libraries(ts ${OpenCV_LIBS})
