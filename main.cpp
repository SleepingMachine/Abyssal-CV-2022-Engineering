#include "include/camera/camera-stream.hpp"
#include "include/control/control-module.hpp"
#include "include/depth/depth-analysis.hpp"

#include <thread>

std::mutex mutex_camera;
std::mutex mutex_depth_analysis;
std::mutex mutex_serial_port_data;

std::atomic_bool camera_start;
std::atomic_bool serial_port_start;

cv::Mat frame_depth                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_depth_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);

int main(int argc, char** argv) {
    std::thread control_thread (SwitchControl::SwitchMode);
    std::thread camera_thread (CameraStream::StreamRetrieve, &frame_color, &frame_depth);
    std::thread depth_thread  (DepthSolution::DepthSolutionStream, &frame_color, &frame_depth, &frame_color_depth_analysis, &frame_depth_depth_analysis);

    control_thread.join();
    camera_thread.join();
}
