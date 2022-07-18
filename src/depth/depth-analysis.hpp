//
// Created by sleepingmachine on 22-4-15.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP
#define ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP

#include "../asset/robomaster-config.hpp"
#include "depth-tool.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

#include <librealsense2/rs.hpp>

#include <atomic>

class DepthSolution{
private:
    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat dst_depth_analysis_;
    //static cv::Mat mask_depth_filter_;
    static rs2::pipeline_profile profile_;
    static void DeepConversion();
    static void DrawReferenceGraphics();
public:
    static FunctionConfig functionConfig_;
    static void DepthSolutionStream(cv::Mat* import_src_color, cv::Mat* import_src_depth, cv::Mat* export_dst_color, cv::Mat* export_dst_depth);
    DepthSolution();
    ~DepthSolution() {};

};
#endif //ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP
