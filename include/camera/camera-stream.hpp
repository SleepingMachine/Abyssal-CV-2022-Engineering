//
// Created by sleepingmachine on 2022/4/11.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_CAMERA_STREAM_HPP
#define ABYSSAL_CV_2022_ENGINEERING_CAMERA_STREAM_HPP

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>

using namespace std;
using namespace cv;

const int kCameraFrameWidth  = 640;
const int kCameraFrameHeight = 480;
const int kCameraFps         = 60;

class CameraStream{
private:
    static rs2::frameset frames_;
    static rs2::pipeline pipe_;
    static rs2::config cfg_;
    static rs2::colorizer colorizer_;

    static rs2::pipeline_profile profile_;
public:
    static void InitCamera();
    static int StreamRetrieve(cv::Mat* pFrame_color, cv::Mat* pFrame_depth);
};

#endif //ABYSSAL_CV_2022_ENGINEERING_CAMERA_STREAM_HPP