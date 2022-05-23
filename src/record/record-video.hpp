//
// Created by sleepingmachine on 22-5-6.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_RECORD_VIDEO_HPP
#define ABYSSAL_CV_2022_ENGINEERING_RECORD_VIDEO_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <stdio.h>
#include <time.h>

#include "../asset/robomaster-config.hpp"
#include "../src/depth/depth-analysis.hpp"

class RecordVideo{
public:
    static void SaveRunningVideo(cv::Mat* pFrame);

private:
    static cv::Mat temp_;
    static cv::Mat src_;
};
#endif //ABYSSAL_CV_2022_ENGINEERING_RECORD_VIDEO_HPP
