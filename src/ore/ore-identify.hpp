//
// Created by sleepingmachine on 22-4-14.
//
#include "ore-tool.hpp"
#include "../asset/robomaster-config.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

#include <atomic>

#ifndef ABYSSAL_CV_2022_ENGINEERING_ORE_IDENTIFY_HPP
#define ABYSSAL_CV_2022_ENGINEERING_ORE_IDENTIFY_HPP
class IdentifyOre{
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

    static std::vector<std::vector<cv::Point2i>> all_contours_;
    static std::vector<std::vector<cv::Point2i>> suspected_ore_contours_;

    static std::vector<cv::Vec4i> hierarchy_;

    static std::vector<cv::RotatedRect> suspected_ore_rects_;

    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat src_color_HSV_;
    static cv::Mat color_mask_;
    static cv::Mat dst_color_;

    static void ImagePreprocess(const cv::Mat &src);
    static void SearchOre(cv::Mat &preprocessed);
    static void DepthCalculation();
    static void resourceRelease();
    static void DrawReferenceGraphics();

public:
    IdentifyOre();
    ~IdentifyOre() {};

    static void OreIdentifyStream(cv::Mat* import_src_color, cv::Mat* import_src_depth, int* sentData);
};
#endif //ABYSSAL_CV_2022_ENGINEERING_ORE_IDENTIFY_HPP
