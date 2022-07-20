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
        include/camera/camera-stream.hpp include/camera/camera-cv-helpers.hpp src/control/control-module.cpp include/control/control-module.hpp include/control/control-read-config.hpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
