//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_STREAM_HPP
#define ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_STREAM_HPP

#include "identify-ore.hpp"
#include "identify-box.hpp"

#include <opencv2/core/mat.hpp>
#include <thread>
#include <tiff.h>

class IdentifyStream{
private:
    static void IdentifyNearStream(cv::Mat *import_src_color_near, cv::Mat *import_src_depth, int64 *sent_data);
    static void IdentifyFarStream (cv::Mat *import_src_color_far,  cv::Mat *import_src_depth, int64 *sent_data);
public:
    static int IdentifyDivert(cv::Mat *import_src_color_near, cv::Mat *import_src_color_far, cv::Mat* import_src_depth, int64* sent_data_ore, int64* sent_data_box);
    IdentifyStream();
    ~IdentifyStream() {};
};
#endif //ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_STREAM_HPP
