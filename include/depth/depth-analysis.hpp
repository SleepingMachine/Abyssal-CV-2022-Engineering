//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP
#define ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP

#include "depth-tools.hpp"
#include "../include/camera/camera-stream.hpp"
#include "../include/control/control-module.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"

#include <librealsense2/rs.hpp>

#include <atomic>
#include <thread>

class DepthSolution{
private:
    static cv::Mat src_color_;
    static cv::Mat src_depth_;
    static cv::Mat dst_depth_analysis_near_;
    static cv::Mat dst_depth_analysis_far_ ;

    static int DeepSegmentation();
    static int AuxiliaryGraphicsDrawing();

public:
    static int DepthSolutionStream(cv::Mat* import_src_color, cv::Mat* import_src_depth, cv::Mat* export_dst_color_near, cv::Mat* export_dst_color_far, cv::Mat* export_dst_depth);

    DepthSolution();
    ~DepthSolution() {};

};

#endif //ABYSSAL_CV_2022_ENGINEERING_DEPTH_ANALYSIS_HPP
