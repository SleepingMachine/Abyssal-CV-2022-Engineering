//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP
#define ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP

#include "control/control-module.hpp"
#include "../include/identify/identify-tools.hpp"
#include "../asset/robomaster-define.hpp"
#include "../tools/tools-config.hpp"

#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

class IdentifyBox{
private:
    static int open_;
    static int close_;
    static int erode_;
    static int dilate_;

    static bool _find_box_flag;

    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat src_gray_;
    static cv::Mat purple_src_;
    static cv::Mat separation_src_data_;
    static cv::Mat separation_src_;
    static cv::Mat separation_src_green_;
    static cv::Mat dst_color_;

    static cv::Mat target_graphics_box_components_;

    static BoxPara boxPara_;

    static std::vector<cv::Mat> split_src_;

    static std::vector<std::vector<cv::Point2i>> all_contours_;
    static std::vector<cv::Vec4i> hierarchy_;
    static std::vector<cv::Point2i> box_components_inside_corners_;

    static std::vector<std::vector<cv::Point2i>> suspected_box_components_contours_;
    static std::vector<cv::RotatedRect> suspected_box_components_rects_;

    static void ImagePreprocess();
    static void SearchBoxComponents();
    static void BoxPairing();
    static void AuxiliaryGraphicsDrawing();
    static void ResourceRelease();

public:
    static void BoxIdentifyStream(cv::Mat* import_src_color, cv::Mat* import_src_depth);

    struct BoxStruct{
        BoxStruct(){};
        cv::Point2i box_components_UL;
        cv::Point2i box_components_UR;
        cv::Point2i box_components_LR;
        cv::Point2i box_components_LL;

        cv::Point2i box_center;
        //cv::RotatedRect box_rect;
        float box_pitch_angle;
        float box_yaw_angle;
        float box_depth;
    };

    IdentifyBox();
    ~IdentifyBox() {};
private:
    static BoxStruct target_box;
};

#endif //ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_BOX_HPP
