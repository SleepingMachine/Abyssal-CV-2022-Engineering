//
// Created by sleepingmachine on 22-7-22.
//

#include "../include/identify/identify-stream.hpp"

extern std::atomic_bool camera_start;
extern std::atomic_bool serial_port_start;
extern std::atomic_bool configuration_file_read_complete;

extern std::mutex mutex_depth;

std::atomic_bool _near_thread_state_flag;
std::atomic_bool _far_thread_state_flag;

IdentifyStream::IdentifyStream() {}

int IdentifyStream::IdentifyDivert(cv::Mat *import_src_color_near, cv::Mat *import_src_color_far, cv::Mat *import_src_depth,
                                   int64 *sent_data) {
    IdentifyStream::IdentifyNearStream(import_src_color_near);
    return 0;
}

void IdentifyStream::IdentifyNearStream(cv::Mat *import_src_color_near) {
    cv::Mat a;
    while(true){
        if (mutex_depth.try_lock()) {
            a = *import_src_color_near;
            mutex_depth.unlock();
        }
        cv::imshow("1", a);
        cv::waitKey(1);
    }
    //这样可以
}


