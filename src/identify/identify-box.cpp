//
// Created by sleepingmachine on 22-7-22.
//
#include "identify/identify-box.hpp"

extern std::mutex mutex_depth;

extern std::atomic_bool _far_thread_state_flag;

IdentifyBox::IdentifyBox() {}

BoxPara IdentifyBox::boxPara_ = BoxParaFactory::getBoxPara();

cv::Mat IdentifyBox::src_color_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_gray_             (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_data_  (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_       (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_green_ (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_            (480, 640, CV_8UC3);

std::vector<cv::Mat> IdentifyBox::split_src_;

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    while (true){
        if(_far_thread_state_flag) {
            if (mutex_depth.try_lock()) {
                temp_src_color = *import_src_color;
                temp_src_depth = *import_src_depth;
                mutex_depth.unlock();
            }
            if (!temp_src_color.empty()) {
                temp_src_color.copyTo(src_color_);
                temp_src_depth.copyTo(src_depth_);
            }
            ImagePreprocess();
            cv::imshow("0", src_color_);
            cv::imshow("1", dst_color_);
        }
    }
}

void IdentifyBox::ImagePreprocess() {
    cv::split(src_color_, split_src_);                                                               //分离色彩通道
    cv::cvtColor(src_color_, src_gray_, cv::COLOR_BGR2GRAY);                                        //获取灰度图
    cv::threshold(src_gray_, separation_src_data_, 240, 255, cv::THRESH_BINARY);
    cv::bitwise_not(separation_src_data_, separation_src_data_);

    cv::threshold(src_gray_, src_gray_, boxPara_.gray_threshold_RED, 255, cv::THRESH_BINARY);     //灰度二值化
    cv::subtract(split_src_[2], split_src_[0], separation_src_);                                 //红蓝通道相减
    cv::subtract(split_src_[2], split_src_[1], separation_src_green_);                             //红绿通道相减
    cv::threshold(separation_src_, separation_src_, boxPara_.separation_threshold_RED, 255, cv::THRESH_BINARY);             //红蓝二值化
    cv::threshold(separation_src_green_, separation_src_green_, boxPara_.separation_threshold_GREEN, 255, cv::THRESH_BINARY);//红绿二值化
    cv::dilate(separation_src_, separation_src_, IdentifyTool::structuringElement3());
    cv::dilate(separation_src_green_, separation_src_green_, IdentifyTool::structuringElement3());                                        //膨胀

    dst_color_ = separation_src_ & src_gray_ & separation_src_green_ & separation_src_data_;                                                                //逻辑与获得最终二值化图像
    cv::dilate(dst_color_, dst_color_, IdentifyTool::structuringElement3());
}


