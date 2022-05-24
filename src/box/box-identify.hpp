//
// Created by sleepingmachine on 22-5-23.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_BOX_IDENTIFY_HPP
#define ABYSSAL_CV_2022_ENGINEERING_BOX_IDENTIFY_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include "../asset/robomaster-config.hpp"
#include <math.h>
#include "box-tool.hpp"

#include "iostream"
#include <atomic>

class IdentifyBox{
public:
    friend class SwitchControl;

    struct BoxComponentsStruct{
        cv::RotatedRect box_components_rect;
        int box_components_type;
    };
    struct BoxStruct{
        cv::RotatedRect box_components_TL;
        cv::RotatedRect box_components_TR;
        cv::RotatedRect box_components_LR;
        cv::RotatedRect box_components_LL;

        cv::Point2f center_point;
    };
private:

    static int hmin_0_;
    static int hmax_0_;
    static int smin_0_;
    static int smax_0_;
    static int vmin_0_;
    static int vmax_0_;

    static int hmin_1_;
    static int hmax_1_;
    static int smin_1_;
    static int smax_1_;
    static int vmin_1_;
    static int vmax_1_;

    static int open_;
    static int close_;
    static int erode_;
    static int dilate_;

    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat src_color_HSV_;
    static cv::Mat color_mask_;
    static cv::Mat dst_color_;

    static std::vector<std::vector<cv::Point2i>> all_contours_;
    static std::vector<cv::Vec4i> hierarchy_;

    static std::vector<std::vector<cv::Point2i>> suspected_box_components_contours_;
    static std::vector<cv::RotatedRect> suspected_box_components_rects_;

    static std::vector<BoxComponentsStruct> box_components_TL_;
    static std::vector<BoxComponentsStruct> box_components_TR_;
    static std::vector<BoxComponentsStruct> box_components_LL_;
    static std::vector<BoxComponentsStruct> box_components_LR_;

    static std::vector<BoxStruct> boxs;

    //static std::vector<std::vector<cv::Point2i>> suspected_ore_contours_;

    static void ImagePreprocess(const cv::Mat &src);
    static void SearchSuspectedBoxComponents(cv::Mat &preprocessed);
    static void BoxComponentsFilter();
    static void ResourceRelease();
    static void DrawReferenceGraphics();
    static void BoxPairing();
public:
    IdentifyBox();
    ~IdentifyBox() {};

    static void BoxIdentifyStream(cv::Mat* import_src_color, cv::Mat* import_src_depth);

};

#endif //ABYSSAL_CV_2022_ENGINEERING_BOX_IDENTIFY_HPP
