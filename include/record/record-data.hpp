//
// Created by sleepingmachine on 22-7-23.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_RECORD_DATA_HPP
#define ABYSSAL_CV_2022_ENGINEERING_RECORD_DATA_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../include/control/control-module.hpp"

class RecordData{
public:
    static int SaveDataStream(cv::Mat* pFrame);
    RecordData();
    ~RecordData();
private:
    static void SaveVideo(cv::Mat* pFrame);
    static void SaveLog();

    static std::string output_video_path_;
    static cv::VideoWriter output_video_;

    static cv::Mat temp_;
    static cv::Mat src_;
};

#endif //ABYSSAL_CV_2022_ENGINEERING_RECORD_DATA_HPP
