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
cv::Mat IdentifyBox::color_mask_0_  (480, 640, CV_8UC3);
cv::Mat IdentifyBox::color_mask_1_  (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_     (480, 640, CV_8UC3);

std::vector<std::vector<cv::Point2i>> IdentifyBox::all_contours_;
std::vector<cv::Vec4i> IdentifyBox::hierarchy_;

std::vector<std::vector<cv::Point2i>> IdentifyBox::suspected_box_components_contours_;
std::vector<cv::RotatedRect> IdentifyBox::suspected_box_components_rects_;

std::vector<IdentifyBox::BoxComponentsStruct> IdentifyBox::box_components_TL_;
std::vector<IdentifyBox::BoxComponentsStruct> IdentifyBox::box_components_TR_;
std::vector<IdentifyBox::BoxComponentsStruct> IdentifyBox::box_components_LL_;
std::vector<IdentifyBox::BoxComponentsStruct> IdentifyBox::box_components_LR_;

std::vector<IdentifyBox::BoxStruct> IdentifyBox::boxs;

//std::vector<std::vector<cv::Point2i>> IdentifyBox::suspected_ore_contours_;

static BoxPara boxPara = BoxParaFactory::getBoxPara();

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth, int64* sent_data) {
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
    BoxPairing();
    DrawReferenceGraphics();
    if (boxs.size() == 1){
        int temp_target_x = boxs[0].center_point.x;
        int temp_target_y = boxs[0].center_point.y;
        *sent_data = temp_target_x * 10000 + temp_target_y * 10;
        //long long test = temp_target_x * 10000000 + temp_target_y * 10000;
        //std::cout << test << std::endl;
    }
    else{
        *sent_data = 0;
    }
    ResourceRelease();
}

void IdentifyBox::ImagePreprocess(const cv::Mat &src) {
    cv::cvtColor(src, src_color_HSV_, CV_BGR2HSV, 0);

    cv::inRange(src_color_HSV_, cv::Scalar(hmin_0_, smin_0_, vmin_0_),
                cv::Scalar(hmax_0_, smax_0_, vmax_0_),
                color_mask_0_);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_1_, smin_1_, vmin_1_),
                cv::Scalar(hmax_1_, smax_1_, vmax_1_),
                color_mask_1_);

    color_mask_0_ = color_mask_0_ | color_mask_1_;

    morphologyEx(color_mask_0_, dst_color_, 2, getStructuringElement (cv::MORPH_RECT,cv::Size(open_,   open_)));
    morphologyEx(dst_color_,  dst_color_,   3, getStructuringElement (cv::MORPH_RECT,cv::Size(close_,  close_)));
    morphologyEx(dst_color_,  dst_color_,   0, getStructuringElement (cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    morphologyEx(dst_color_,  dst_color_,   1, getStructuringElement (cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
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
            if (BoxTool::getRectLengthWidthRatio(scanRect) < boxPara.min_suspected_box_length_width_ratio ||
                BoxTool::getRectLengthWidthRatio(scanRect) > boxPara.max_suspected_box_length_width_ratio){
                continue;
            }
            if (cv::contourArea( all_contours_[i], false ) / scanRect.size.area() <= boxPara.min_suspected_box_components_duty_cycle ||
                cv::contourArea( all_contours_[i], false ) / scanRect.size.area() >= boxPara.max_suspected_box_components_duty_cycle){
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
    box_components_LR_.clear();
    box_components_TR_.clear();
    box_components_LL_.clear();
    box_components_TL_.clear();
    boxs.clear();
}

void IdentifyBox::DrawReferenceGraphics() {
    for (int i = 0; i < suspected_box_components_rects_.size(); ++i) {
        BoxTool::drawRotatedRect(src_color_, suspected_box_components_rects_[i], cv::Scalar(15, 198, 150), 2, 16);
    }
    if (boxs.size() == 1){
        cv::circle(src_color_, boxs[0].center_point, 15, cv::Scalar(255,0,255), 10);
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
        //rect_angle -= 90;

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
            if ((int)cropped.at<uchar>(u, 2) == 255) {
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
            if ((int)cropped.at<uchar>(2, r) == 255) {
                counter_R++;
            }
            //cv::waitKey(200);
        }

        BoxComponentsStruct temp_box_components;

        if (counter_U > counter_D && counter_R > counter_L){
            cv::putText(src_color_, "TR", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_box_components.box_components_rect = suspected_box_components_rects_[i];
            temp_box_components.box_components_type = BoxComponentsType::BOX_COMPONENTS_TR;
            box_components_TR_.push_back(temp_box_components);
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
            cv::imshow("debug_TR", cropped);
        }
        else if (counter_D > counter_U && counter_R > counter_L){
            cv::putText(src_color_, "LR", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_box_components.box_components_rect = suspected_box_components_rects_[i];
            temp_box_components.box_components_type = BoxComponentsType::BOX_COMPONENTS_LR;
            box_components_LR_.push_back(temp_box_components);
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
            cv::imshow("debug_LR", cropped);
        }
        else if (counter_U > counter_D && counter_L > counter_R){
            cv::putText(src_color_, "TL", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_box_components.box_components_rect = suspected_box_components_rects_[i];
            temp_box_components.box_components_type = BoxComponentsType::BOX_COMPONENTS_TL;
            box_components_TL_.push_back(temp_box_components);
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
            cv::imshow("debug_TL", cropped);
        }
        else if (counter_D > counter_U && counter_L > counter_R){
            cv::putText(src_color_, "LL", suspected_box_components_rects_[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),1,8,false);
            temp_box_components.box_components_rect = suspected_box_components_rects_[i];
            temp_box_components.box_components_type = BoxComponentsType::BOX_COMPONENTS_LL;
            box_components_LL_.push_back(temp_box_components);
            //std::cout << counter_U << " " << counter_D <<" "<< counter_L << " " << counter_R << std::endl;
            cv::imshow("debug_LL", cropped);
        }
        //cv::imshow("crop", cropped);
        //cv::waitKey(500);
    }
}

void IdentifyBox::BoxPairing() {
    IdentifyBox::BoxStruct temp_box;
    if (box_components_LL_.size() == 1 && box_components_LR_.size() == 1 && box_components_TL_.size() == 1 && box_components_TR_.size() == 1){
        //cv::line(src_color_, box_components_TL.back().box_components_rect.center, box_components_LR.back().box_components_rect.center, cv::Scalar(0,255,255), 2);
        if (box_components_TL_.back().box_components_rect.center.x < box_components_TR_.back().box_components_rect.center.x &&
            box_components_TL_.back().box_components_rect.center.y < box_components_LL_.back().box_components_rect.center.y &&
            box_components_LL_.back().box_components_rect.center.x < box_components_LR_.back().box_components_rect.center.x &&
            box_components_TR_.back().box_components_rect.center.y < box_components_LR_.back().box_components_rect.center.y){
            //cv::line(src_color_, box_components_TL_.back().box_components_rect.center, box_components_LR_.back().box_components_rect.center, cv::Scalar(0,255,255), 2);
            //cv::line(src_color_, box_components_TR_.back().box_components_rect.center, box_components_LL_.back().box_components_rect.center, cv::Scalar(0,255,255), 2);
            temp_box.box_components_TL = box_components_TL_.back().box_components_rect;
            temp_box.box_components_TR = box_components_TR_.back().box_components_rect;
            temp_box.box_components_LR = box_components_LR_.back().box_components_rect;
            temp_box.box_components_LL = box_components_LL_.back().box_components_rect;
            temp_box.center_point = BoxTool::getCrossPoint(box_components_TL_.back().box_components_rect.center, box_components_LR_.back().box_components_rect.center,box_components_TR_.back().box_components_rect.center, box_components_LL_.back().box_components_rect.center);
            boxs.push_back(temp_box);
        }
    }
    else if(box_components_LL_.size() == 1 && box_components_LR_.size() == 1 && box_components_TL_.size() == 1 && box_components_TR_.empty()){
        if (box_components_TL_.back().box_components_rect.center.y < box_components_LL_.back().box_components_rect.center.y &&
            box_components_LL_.back().box_components_rect.center.x < box_components_LR_.back().box_components_rect.center.x){
            //cv::line(src_color_, box_components_TL_.back().box_components_rect.center, box_components_LR_.back().box_components_rect.center, cv::Scalar(0,255,255), 2);
            temp_box.box_components_TL = box_components_TL_.back().box_components_rect;
            //temp_box.box_components_TR = box_components_TR_.back().box_components_rect;
            temp_box.box_components_LR = box_components_LR_.back().box_components_rect;
            temp_box.box_components_LL = box_components_LL_.back().box_components_rect;
            temp_box.center_point = BoxTool::getTwoPointCenterPoint(box_components_TL_.back().box_components_rect.center, box_components_LR_.back().box_components_rect.center);
            boxs.push_back(temp_box);
        }
    }
}
