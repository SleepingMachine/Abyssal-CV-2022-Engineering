#include "include/camera/camera-stream.hpp"
#include "include/control/control-module.hpp"

#include <thread>


std::mutex mutex_camera;


int main() {
    std::thread camera_thread (CameraStream::StreamRetrieve);
    std::thread control_thread (SwitchControl::SwitchMode);
    camera_thread.join();
    control_thread.join();
}
