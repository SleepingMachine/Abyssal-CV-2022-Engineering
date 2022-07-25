//
// Created by sleepingmachine on 22-7-22.
//
#include "identify/identify-box.hpp"

extern std::mutex mutex_depth;

extern std::atomic_bool _far_thread_state_flag;

IdentifyBox::IdentifyBox() {}

BoxPara IdentifyBox::boxPara_ = BoxParaFactory::getBoxPara();

int IdentifyBox::open_   = 1;
int IdentifyBox::close_  = 1;
int IdentifyBox::erode_  = 2;
int IdentifyBox::dilate_ = 8;

cv::Mat IdentifyBox::src_color_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_gray_             (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_data_  (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_       (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_green_ (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_            (480, 640, CV_8UC3);

std::vector<cv::Mat> IdentifyBox::split_src_;

std::vector<std::vector<cv::Point2i>> IdentifyBox::all_contours_;
std::vector<cv::Vec4i> IdentifyBox::hierarchy_;

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);
    //IdentifyTool::CreatTrackbars(&open_,&close_,&erode_,&dilate_);

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
            SearchBoxComponents();
            AuxiliaryGraphicsDrawing();
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
    morphologyEx(dst_color_, dst_color_, 2, getStructuringElement (cv::MORPH_RECT,cv::Size(open_,   open_)));
    morphologyEx(dst_color_,  dst_color_,   3, getStructuringElement (cv::MORPH_RECT,cv::Size(close_,  close_)));
    morphologyEx(dst_color_,  dst_color_,   0, getStructuringElement (cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    morphologyEx(dst_color_,  dst_color_,   1, getStructuringElement (cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
}

void IdentifyBox::SearchBoxComponents() {
    cv::findContours(dst_color_, all_contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //cv::Mat test = imread("../asset/box_component_900.jpge");

    for(int i = 0; i < all_contours_.size(); ++i){
        if (hierarchy_[i][3] == -1){
            cv::RotatedRect scanRect = cv::minAreaRect(all_contours_[i]);
            IdentifyTool::drawRotatedRect(src_color_, scanRect, cv::Scalar(15, 198, 150), 2, 16);
        }
    }
}

void IdentifyBox::AuxiliaryGraphicsDrawing() {
    cv::imshow("0", src_color_);
    cv::imshow("1", dst_color_);
}


