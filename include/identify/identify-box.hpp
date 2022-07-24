//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP
#define ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP

#include "control/control-module.hpp"
#include "../include/identify/identify-tools.hpp"
#include "../asset/robomaster-define.hpp"
#include "../tools/tools-config.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

class IdentifyBox{
private:
    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat src_gray_;
    static cv::Mat purple_src_;
    static cv::Mat separation_src_data_;
    static cv::Mat separation_src_;
    static cv::Mat separation_src_green_;
    static cv::Mat dst_color_;

    static BoxPara boxPara_;

    static std::vector<cv::Mat> split_src_;

    static void ImagePreprocess();

public:
    static void BoxIdentifyStream(cv::Mat* import_src_color, cv::Mat* import_src_depth);

    struct BoxStruct{
        cv::RotatedRect box_components_UL_rect;
        cv::RotatedRect box_components_UR_rect;
        cv::RotatedRect box_components_LR_rect;
        cv::RotatedRect box_components_LL_rect;
        cv::RotatedRect box_rect;
        float box_pitch_angle;
        float box_yaw_angle;
        float box_depth;
    };

    IdentifyBox();
    ~IdentifyBox() {};

};

#endif //ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP
