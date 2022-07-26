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
    while(!camera_start || !serial_port_start){
        cv::waitKey(10);
    }
    std::thread identify_near_thread( IdentifyStream::IdentifyNearStream,import_src_color_near, import_src_depth);
    std::thread identify_far_thread ( IdentifyStream::IdentifyFarStream ,import_src_color_far,  import_src_depth);

    identify_near_thread.join();
    identify_far_thread .join();
    return 0;
}

void IdentifyStream::IdentifyNearStream(cv::Mat *import_src_color_near, cv::Mat *import_src_depth) {
    IdentifyOre::OreIdentifyStream(import_src_color_near, import_src_depth);
}

void IdentifyStream::IdentifyFarStream(cv::Mat *import_src_color_far, cv::Mat *import_src_depth) {
    IdentifyBox::BoxIdentifyStream(import_src_color_far, import_src_depth);
}

