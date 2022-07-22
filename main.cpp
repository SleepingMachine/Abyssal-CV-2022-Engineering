#include "include/camera/camera-stream.hpp"
#include "include/control/control-module.hpp"
#include "include/depth/depth-analysis.hpp"
#include "include/serial/serial-port.hpp"
#include "include/identify/identify-stream.hpp"

std::mutex mutex_camera;
std::mutex mutex_depth_analysis;
std::mutex mutex_serial_port_data;

std::atomic_bool camera_start                     = false;
std::atomic_bool serial_port_start                = false;
std::atomic_bool configuration_file_read_complete = false;

cv::Mat frame_depth                     (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color                     (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color_depth_analysis_near (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color_depth_analysis_far  (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_depth_depth_analysis      (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);

static int64 sent_serial_port_data;

int main(int argc, char** argv) {
    std::thread control_thread  (SwitchControl::SwitchMode);
    std::thread serial_thread   (SerialPort::SendData, &sent_serial_port_data);
    std::thread camera_thread   (CameraStream::StreamRetrieve, &frame_color, &frame_depth);
    std::thread depth_thread    (DepthSolution::DepthSolutionStream, &frame_color, &frame_depth, &frame_color_depth_analysis_near, &frame_color_depth_analysis_far, &frame_depth_depth_analysis);
    std::thread identify_thread (IdentifyStream::IdentifyDivert, &frame_color_depth_analysis_near, &frame_color_depth_analysis_far, &frame_depth_depth_analysis, &sent_serial_port_data);

    control_thread .join();
    serial_thread  .join();
    camera_thread  .join();
    depth_thread   .join();
    identify_thread.join();

    return 0;
}
