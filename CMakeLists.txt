cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME Abyssal_CV_2022_Engineering)
project(${PROJECT_NAME})

#set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20  -pthread" )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/cmake-build-debug)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})

add_executable(Abyssal_CV_2022_Engineering
        main.cpp
        src/camera/camera-stream.cpp
        include/camera/camera-stream.hpp
        include/camera/camera-cv-helpers.hpp
        src/control/control-module.cpp include/control/control-module.hpp
        include/control/control-read-config.hpp
        include/depth/depth-analysis.hpp
        src/depth/depth-analysis.cpp
        include/depth/depth-tools.hpp
        tools/tools-config.hpp
        asset/robomaster-define.hpp
        src/identify/identify-box.cpp
        src/identify/identify-ore.cpp
        include/identify/identify-box.hpp
        include/identify/identify-ore.hpp
        include/identify/identify-tools.hpp
        src/serial/serial-port.cpp
        include/serial/serial-port.hpp
        src/identify/identify-stream.cpp
        include/identify/identify-stream.hpp
        include/record/record-data.hpp
        src/record/record-data.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/3rdparty/libMVSDK.so")
