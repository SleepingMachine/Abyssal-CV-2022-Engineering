//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_ORE_HPP
#define ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_ORE_HPP

#include "control/control-module.hpp"
#include "../include/identify/identify-tools.hpp"
#include "../asset/robomaster-define.hpp"
#include "../tools/tools-config.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

class IdentifyOre{
public:
    friend class SwitchControl;
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

    static int target_ore_index_;

    static bool _target_ore_fall_;

    static std::vector<std::vector<cv::Point2i>> all_contours_;
    static std::vector<cv::Vec4i> hierarchy_;

    static std::vector<std::vector<cv::Point2i>> suspected_ore_contours_;
    static std::vector<cv::RotatedRect> suspected_ore_rects_;
    //static std::vector<int> target_ore_location_;

    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat src_color_HSV_;
    static cv::Mat color_mask_0_;
    static cv::Mat color_mask_1_;
    static cv::Mat dst_color_;

    static void ImagePreprocess(const cv::Mat &src);
    static void SearchOre(cv::Mat &preprocessed);
    static void DepthCalculation();
    static void TargetSelection();
    static void DropDetection();
    static void AuxiliaryGraphicsDrawing();
    static void ResourceRelease();

public:
    struct OreStruct{
        cv::RotatedRect ore_rect;
        float ore_depth;
    };
    IdentifyOre();
    ~IdentifyOre() {};

    static void OreIdentifyStream(cv::Mat* import_src_color, cv::Mat* import_src_depth);

    struct TargetOreLocationStruct {
        cv::Point2f target_ore_center;
        float       target_ore_depth;
        float       target_ore_radius;
        TargetOreLocationStruct(){
            target_ore_center = cv::Point2f(0, 0);
            target_ore_depth  = 0;
            target_ore_radius = 0;
        }
    };
private:
    static std::vector<OreStruct> ore_structs_;
    static std::vector<cv::Point2f> target_ore_track_;
    static TargetOreLocationStruct current_target_ore_location_;
};

#endif //ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_ORE_HPP
