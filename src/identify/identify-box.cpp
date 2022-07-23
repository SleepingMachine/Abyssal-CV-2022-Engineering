//
// Created by sleepingmachine on 22-7-22.
//
#include "identify/identify-box.hpp"

extern std::mutex mutex_depth;

extern std::atomic_bool _far_thread_state_flag;

IdentifyBox::IdentifyBox() {}

cv::Mat IdentifyBox::src_color_      (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_      (480, 640, CV_8UC3);

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    while (true){
        if(_far_thread_state_flag) {
            if (mutex_depth.try_lock()) {
                temp_src_color = *import_src_color;
                temp_src_depth = *import_src_depth;
                mutex_depth.unlock();
            }
            if (!temp_src_color.empty()) {
                temp_src_color.copyTo(src_color_);
                temp_src_depth.copyTo(src_depth_);
            }
            cv::imshow("1", src_color_);
        }
    }
}


