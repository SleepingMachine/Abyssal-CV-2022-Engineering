//
// Created by sleepingmachine on 22-5-23.
//

#include "box-identify.hpp"

extern std::mutex mutex_depth_analysis;

int IdentifyBox::hmin_0_ = 0;
int IdentifyBox::hmax_0_ = 65;
int IdentifyBox::smin_0_ = 235;
int IdentifyBox::smax_0_ = 255;
int IdentifyBox::vmin_0_ = 115;
int IdentifyBox::vmax_0_ = 255;

int IdentifyBox::hmin_1_ = 0;
int IdentifyBox::hmax_1_ = 0;
int IdentifyBox::smin_1_ = 0;
int IdentifyBox::smax_1_ = 0;
int IdentifyBox::vmin_1_ = 0;
int IdentifyBox::vmax_1_ = 0;

int IdentifyBox::open_   = 1;
int IdentifyBox::close_  = 13;
int IdentifyBox::erode_  = 5;
int IdentifyBox::dilate_ = 3;

IdentifyBox::IdentifyBox() {}

cv::Mat IdentifyBox::src_color_     (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_     (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_color_HSV_ (480, 640, CV_8UC3);
cv::Mat IdentifyBox::color_mask_    (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_     (480, 640, CV_8UC3);

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    if (mutex_depth_analysis.try_lock()) {
        temp_src_color = *import_src_color;
        temp_src_depth = *import_src_depth;
        mutex_depth_analysis.unlock();
    }
    if (!temp_src_color.empty()){
        temp_src_color.copyTo(src_color_);
        temp_src_depth.copyTo(src_depth_);
    }

    ImagePreprocess(src_color_);

    cv::imshow("输入图像", color_mask_);
    //std::cout << "这个模式还没写完 别急" << std::endl;
}

void IdentifyBox::ImagePreprocess(const cv::Mat &src) {
    cv::cvtColor(src, src_color_HSV_, CV_BGR2HSV, 0);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_0_, smin_0_, vmin_0_),
                cv::Scalar(hmax_0_, smax_0_, vmax_0_),
                color_mask_);

    //morphologyEx(color_mask_, color_mask_, 2, getStructuringElement(cv::MORPH_RECT,cv::Size(open_,    open_)));
    //morphologyEx(color_mask_, color_mask_, 3, getStructuringElement(cv::MORPH_RECT,cv::Size(close_,  close_)));
    //morphologyEx(color_mask_, color_mask_, 0, getStructuringElement(cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    //morphologyEx(color_mask_, dst_color_, 1, getStructuringElement(cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
}
