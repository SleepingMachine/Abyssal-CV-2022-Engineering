#include "camera/camera-stream.hpp"
#include "serial/serial-port.hpp"
#include "serial/serial-TUP/serial-port-TUP.hpp"
#include "depth/depth-analysis.hpp"
#include "record/record-video.hpp"
#include "switch/switch-mode.hpp"
#include <thread>

std::mutex mutex_camera;
std::mutex mutex_depth_analysis;
std::mutex mutex_serial_port_data;

static int64 sent_serial_port_data;

extern std::atomic_bool camera_is_open;
extern std::atomic_bool serial_port_start;

cv::Mat frame_depth                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_depth_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
//rs2::pipeline_profile profile;

int main(int argc, char** argv){
    camera_is_open = true;
    serial_port_start = true;

    std::thread serial_thread (SerialPort::SendData, &sent_serial_port_data);
    std::thread camera_thread (CameraStream::StreamRetrieve, &frame_color, &frame_depth);
    std::thread depth_thread  (DepthSolution::DepthSolutionStream, &frame_color, &frame_depth, &frame_color_depth_analysis, &frame_depth_depth_analysis);
    std::thread switch_thread (SwitchControl::SwitchMode, &frame_color_depth_analysis, &frame_depth_depth_analysis, &sent_serial_port_data);
    //std::thread serial_thread(SerialPortTUP::SerialSynchronizeTUP, &sent_serial_port_data);
    std::thread record_thread (RecordVideo::SaveRunningVideo, &frame_color);

    camera_thread.join();
    depth_thread .join();
    switch_thread.join();
    record_thread.join();
    serial_thread.join();
    return 0;
}

