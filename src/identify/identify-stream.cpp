//
// Created by sleepingmachine on 22-7-22.
//

#include "../include/identify/identify-stream.hpp"

extern std::atomic_bool camera_start;
extern std::atomic_bool serial_port_start;
extern std::atomic_bool configuration_file_read_complete;

std::atomic_bool _near_thread_state_flag;
std::atomic_bool _far_thread_state_flag;

IdentifyStream::IdentifyStream() {}

int IdentifyStream::IdentifyDivert(cv::Mat *import_src_color_near, cv::Mat *import_src_color_far, cv::Mat *import_src_depth,
                                   int64 *sent_data) {
    while(!configuration_file_read_complete){
        cv::waitKey(1);
    }
    std::cout << SwitchControl::functionConfig_._operating_mode;

    if(SwitchControl::functionConfig_._operating_mode == OperatingMode::SEARCH_MODE){
        std::thread identify_near_thread(IdentifyOre::OreIdentifyStream, import_src_color_near, import_src_depth);
        //identify_near_thread.join();
    }
    return 0;
}

