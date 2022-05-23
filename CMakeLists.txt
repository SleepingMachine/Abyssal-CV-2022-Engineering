cmake_minimum_required(VERSION 3.21)

set(PROJECT_NAME Abyssal_CV_2022_Engineering)
project(${PROJECT_NAME})

#set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20  -pthread" )
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/cmake-build-debug)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
find_package(Threads REQUIRED)

include_directories( include )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})

add_executable(Abyssal_CV_2022_Engineering src/main.cpp include/example.hpp include/example-utils.hpp include/example-imgui.hpp src/camera/camera-stream.cpp src/camera/camera-stream.hpp src/ore/ore-identify.cpp src/ore/ore-identify.hpp src/ore/ore-tool.hpp asset/robomaster-config.hpp src/depth/depth-analysis.cpp src/depth/depth-analysis.hpp src/depth/depth-tool.hpp src/record/record-video.cpp src/record/record-video.hpp src/serial/serial-port.cpp src/serial/serial-port.hpp src/switch/switch-mode.cpp src/switch/switch-mode.hpp src/box/box-identify.cpp src/box/box-identify.hpp src/switch/switch-tool.hpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${realsense2_LIBRARY})
