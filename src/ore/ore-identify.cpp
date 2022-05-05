//
// Created by sleepingmachine on 22-4-14.
//

#include "ore-identify.hpp"

//extern std::mutex mutex_color;
extern std::mutex mutex_depth_analysis;
extern std::atomic_bool CameraisOpen;

IdentifyOre::IdentifyOre() {}

int IdentifyOre::hmin_0_ = 0;
int IdentifyOre::hmax_0_ = 65;
int IdentifyOre::smin_0_ = 245;
int IdentifyOre::smax_0_ = 255;
int IdentifyOre::vmin_0_ = 115;
int IdentifyOre::vmax_0_ = 255;

int IdentifyOre::hmin_1_ = 0;
int IdentifyOre::hmax_1_ = 0;
int IdentifyOre::smin_1_ = 0;
int IdentifyOre::smax_1_ = 0;
int IdentifyOre::vmin_1_ = 0;
int IdentifyOre::vmax_1_ = 0;

int IdentifyOre::open_   = 1;
int IdentifyOre::close_  = 10;
int IdentifyOre::erode_  = 3;
int IdentifyOre::dilate_ = 3;

std::vector<std::vector<cv::Point2i>> IdentifyOre::allContours_;
std::vector<cv::Vec4i> IdentifyOre::hierarchy_;

std::vector<cv::RotatedRect> IdentifyOre::suspected_ore_;

cv::Mat IdentifyOre::src_color_     (480, 640, CV_8UC3);
cv::Mat IdentifyOre::src_depth_     (480, 640, CV_8UC3);
cv::Mat IdentifyOre::src_color_HSV_ (480, 640, CV_8UC3);
cv::Mat IdentifyOre::color_mask_    (480, 640, CV_8UC3);
cv::Mat IdentifyOre::dst_color_     (480, 640, CV_8UC3);

static OrePara orePara = OreParaFactory::getOrePara();

void IdentifyOre::OreIdentifyStream(cv::Mat *import_src_color, cv::Mat* import_src_depth, int *sentData) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    OreTool::CreatTrackbars(&hmin_0_, &hmax_0_, &smin_0_, &smax_0_, &vmin_0_, &vmax_0_,
                            &hmin_1_, &hmax_1_, &smin_1_, &smax_1_, &vmin_1_, &vmax_1_,
                              &open_,    &close_,    &erode_,   &dilate_);


    while (CameraisOpen){
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
        SearchOre(dst_color_);
        DrawReferenceGraphics();
        resourceRelease();
    }
}

void IdentifyOre::ImagePreprocess(const cv::Mat &src) {
    cv::cvtColor(src, src_color_HSV_, CV_BGR2HSV, 0);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_0_, smin_0_, vmin_0_),
                                    cv::Scalar(hmax_0_, smax_0_, vmax_0_),
                                    color_mask_);

    morphologyEx(color_mask_, color_mask_, 2, getStructuringElement(cv::MORPH_RECT,cv::Size(open_,    open_)));
    morphologyEx(color_mask_, color_mask_, 3, getStructuringElement(cv::MORPH_RECT,cv::Size(close_,  close_)));
    morphologyEx(color_mask_, color_mask_, 0, getStructuringElement(cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    morphologyEx(color_mask_, dst_color_, 1, getStructuringElement(cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
    //cv::imshow("Src", src);
    //cv::imshow("ImagePreprocess", dst_color_);
}

void IdentifyOre::SearchOre(cv::Mat &preprocessed) {
    cv::findContours(preprocessed, allContours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < allContours_.size(); ++i) {
        if (hierarchy_[i][3] == -1) {
            cv::RotatedRect scanRect = cv::minAreaRect(allContours_[i]);                    //检测最小面积的矩形
            cv::Point2f vertices[4];
            scanRect.points(vertices);
            if (scanRect.size.area() < orePara.min_ore_area_){
                continue;
            }
            if (OreTool::getRectLengthWidthRatio(scanRect) < orePara.min_ore_length_width_ratio || OreTool::getRectLengthWidthRatio(scanRect) > orePara.max_ore_length_width_ratio){
                continue;
            }
            suspected_ore_.push_back(scanRect);
        }
    }
}

void IdentifyOre::resourceRelease() {
    allContours_  .clear();                                //轮廓
    hierarchy_    .clear();
    suspected_ore_.clear();
}

void IdentifyOre::DrawReferenceGraphics() {
    for (int i = 0; i < suspected_ore_.size(); ++i) {
        OreTool::drawRotatedRect(src_color_, suspected_ore_[i], cv::Scalar(15, 198, 150), 4, 16);
    }
    cv::imshow("Color", src_color_);
    //cv::imshow("Depth", src_depth_);
    cv::imshow("Mask",  dst_color_);
}




