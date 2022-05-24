//
// Created by sleepingmachine on 22-5-23.
//

#include "box-identify.hpp"

extern std::mutex mutex_depth_analysis;

int IdentifyBox::hmin_0_ = 0;
int IdentifyBox::hmax_0_ = 0;
int IdentifyBox::smin_0_ = 0;
int IdentifyBox::smax_0_ = 0;
int IdentifyBox::vmin_0_ = 0;
int IdentifyBox::vmax_0_ = 0;

int IdentifyBox::hmin_1_ = 0;
int IdentifyBox::hmax_1_ = 0;
int IdentifyBox::smin_1_ = 0;
int IdentifyBox::smax_1_ = 0;
int IdentifyBox::vmin_1_ = 0;
int IdentifyBox::vmax_1_ = 0;

int IdentifyBox::open_   = 0;
int IdentifyBox::close_  = 0;
int IdentifyBox::erode_  = 0;
int IdentifyBox::dilate_ = 0;

IdentifyBox::IdentifyBox() {}

cv::Mat IdentifyBox::src_color_     (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_     (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_color_HSV_ (480, 640, CV_8UC3);
cv::Mat IdentifyBox::color_mask_    (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_     (480, 640, CV_8UC3);

std::vector<std::vector<cv::Point2i>> IdentifyBox::all_contours_;
std::vector<cv::Vec4i> IdentifyBox::hierarchy_;

std::vector<std::vector<cv::Point2i>> IdentifyBox::suspected_box_components_contours_;
std::vector<cv::RotatedRect> IdentifyBox::suspected_box_components_rects_;
//std::vector<std::vector<cv::Point2i>> IdentifyBox::suspected_ore_contours_;

static BoxPara boxPara = BoxParaFactory::getBoxPara();

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
    SearchSuspectedBoxComponents(dst_color_);
    BoxComponentsFilter();
    DrawReferenceGraphics();

    ResourceRelease();
}

void IdentifyBox::ImagePreprocess(const cv::Mat &src) {
    cv::cvtColor(src, src_color_HSV_, CV_BGR2HSV, 0);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_0_, smin_0_, vmin_0_),
                cv::Scalar(hmax_0_, smax_0_, vmax_0_),
                color_mask_);

    morphologyEx(color_mask_, dst_color_, 2, getStructuringElement(cv::MORPH_RECT,cv::Size(open_,   open_)));
    morphologyEx(dst_color_,  dst_color_, 3, getStructuringElement(cv::MORPH_RECT,cv::Size(close_,  close_)));
    morphologyEx(dst_color_,  dst_color_, 0, getStructuringElement(cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    morphologyEx(dst_color_,  dst_color_, 1, getStructuringElement(cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
}

void IdentifyBox::SearchSuspectedBoxComponents(cv::Mat &preprocessed) {
    cv::findContours(preprocessed, all_contours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < all_contours_.size(); ++i) {
        if (hierarchy_[i][3] == -1) {
            cv::RotatedRect scanRect = cv::minAreaRect(all_contours_[i]);                    //检测最小面积的矩形
            cv::Point2f vertices[4];
            scanRect.points(vertices);
            if (scanRect.size.area() < boxPara.min_suspected_box_components_area){
                continue;
            }
            if (BoxTool::getRectLengthWidthRatio(scanRect) < boxPara.min_suspected_box_length_width_ratio || BoxTool::getRectLengthWidthRatio(scanRect) > boxPara.max_suspected_box_length_width_ratio){
                continue;
            }
            suspected_box_components_rects_.push_back(scanRect);
            suspected_box_components_contours_.push_back(all_contours_[i]);
        }
    }
}

void IdentifyBox::ResourceRelease() {
    suspected_box_components_contours_.clear();
    suspected_box_components_rects_.clear();
}

void IdentifyBox::DrawReferenceGraphics() {
    for (int i = 0; i < suspected_box_components_rects_.size(); ++i) {
        BoxTool::drawRotatedRect(src_color_, suspected_box_components_rects_[i], cv::Scalar(15, 198, 150), 2, 16);
    }
    cv::imshow("Dst",  src_color_);
    cv::imshow("Mask", dst_color_);
    //std::cout << "这个模式还没写完 别急" << std::endl;
}

void IdentifyBox::BoxComponentsFilter() {
    cv::Mat M, rotated;
    cv::Mat cropped;
    //cv::namedWindow("crop", 0);
    for (int i = 0; i < suspected_box_components_rects_.size(); ++i) {
        double rect_angle = static_cast<double>(suspected_box_components_rects_[i].angle);
        cv::Size rect_size = suspected_box_components_rects_[i].size;
        if (rect_angle < -45.0) {
            rect_angle += 90.0;
            cv::swap(rect_size.width, rect_size.height);
        }
        M = getRotationMatrix2D(suspected_box_components_rects_[i].center, rect_angle, 1.0);
// perform the affine transformation
        warpAffine(dst_color_, rotated, M, dst_color_.size(), cv::INTER_CUBIC);
// crop the resulting image
        getRectSubPix(rotated, rect_size, suspected_box_components_rects_[i].center, cropped);
        cv::resize(cropped, cropped, cv::Size(20, 20), 0, 0, cv::INTER_LINEAR);
        cv::threshold(cropped,cropped,50,255,0);

        int counter_U = 0;
        int counter_D = 0;
        int counter_L = 0;
        int counter_R = 0;

        for (int u = 0; u < 20; ++u) {
            if ((int)cropped.at<uchar>(u, 1) == 255) {
                counter_U++;
            }
        }
        for (int d = 0; d < 20; ++d) {
            if ((int)cropped.at<uchar>(d, 18) == 255) {
                counter_D++;
            }
        }
        for (int l = 0; l < 20; ++l) {
            if ((int)cropped.at<uchar>(18, l) == 255) {
                counter_L++;
            }
        }
        for (int r = 0; r < 20; ++r) {
            if ((int)cropped.at<uchar>(1, r) == 255) {
                counter_R++;
            }
            //cv::waitKey(200);
        }

        BoxComponentsStruct temp_boxComponents;

        if (counter_U > counter_D && counter_R > counter_L){
            cv::putText(src_color_, "TR", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_boxComponents.box_components_rect = suspected_box_components_rects_[i];
            temp_boxComponents.box_components_type = BoxComponentsType::BOX_COMPONENTS_TR;
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
        }
        else if (counter_D > counter_U && counter_R > counter_L){
            cv::putText(src_color_, "LR", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_boxComponents.box_components_rect = suspected_box_components_rects_[i];
            temp_boxComponents.box_components_type = BoxComponentsType::BOX_COMPONENTS_LR;
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
        }
        else if (counter_U > counter_D && counter_L > counter_R){
            cv::putText(src_color_, "TL", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_boxComponents.box_components_rect = suspected_box_components_rects_[i];
            temp_boxComponents.box_components_type = BoxComponentsType::BOX_COMPONENTS_TL;
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
        }
        else if (counter_D > counter_U && counter_L > counter_R){
            cv::putText(src_color_, "LL", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_boxComponents.box_components_rect = suspected_box_components_rects_[i];
            temp_boxComponents.box_components_type = BoxComponentsType::BOX_COMPONENTS_LL;
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
        }
        //cv::imshow("crop", cropped);
        //cv::waitKey(500);
    }
}