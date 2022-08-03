//
// Created by sleepingmachine on 22-7-22.
//

#include "../include/identify/identify-ore.hpp"

//extern std::mutex mutex_color;
extern std::mutex mutex_depth;
extern std::mutex mutex_serial_port_data_ore;
//extern std::atomic_bool camera_is_open;

extern std::atomic_bool _near_thread_state_flag;


IdentifyOre::IdentifyOre() {}

/*
int IdentifyOre::hmin_0_ = 0;
int IdentifyOre::hmax_0_ = 65;
int IdentifyOre::smin_0_ = 137;
int IdentifyOre::smax_0_ = 255;
int IdentifyOre::vmin_0_ = 126;
int IdentifyOre::vmax_0_ = 255;

int IdentifyOre::hmin_1_ = 0;
int IdentifyOre::hmax_1_ = 65;
int IdentifyOre::smin_1_ = 125;
int IdentifyOre::smax_1_ = 255;
int IdentifyOre::vmin_1_ = 133;
int IdentifyOre::vmax_1_ = 255;
*/

int IdentifyOre::hmin_0_ = 28;
int IdentifyOre::hmax_0_ = 58;
int IdentifyOre::smin_0_ = 139;
int IdentifyOre::smax_0_ = 255;
int IdentifyOre::vmin_0_ = 13;
int IdentifyOre::vmax_0_ = 255;

int IdentifyOre::hmin_1_ = 0;
int IdentifyOre::hmax_1_ = 255;
int IdentifyOre::smin_1_ = 125;
int IdentifyOre::smax_1_ = 255;
int IdentifyOre::vmin_1_ = 133;
int IdentifyOre::vmax_1_ = 255;

int IdentifyOre::open_   = 1;
int IdentifyOre::close_  = 13;
int IdentifyOre::erode_  = 5;
int IdentifyOre::dilate_ = 3;

int IdentifyOre::target_ore_index_ = -1;

bool IdentifyOre::_target_ore_fall_ = false;

IdentifyOre::TargetOreLocationStruct IdentifyOre::current_target_ore_location_;

std::vector<std::vector<cv::Point2i>> IdentifyOre::all_contours_;
std::vector<std::vector<cv::Point2i>> IdentifyOre::suspected_ore_contours_;

std::vector<cv::Vec4i> IdentifyOre::hierarchy_;

std::vector<cv::RotatedRect> IdentifyOre::suspected_ore_rects_;
std::vector<IdentifyOre::OreStruct> IdentifyOre::ore_structs_;
std::vector<cv::Point2f> IdentifyOre::target_ore_track_;

cv::Mat IdentifyOre::src_color_      (480, 640, CV_8UC3);
cv::Mat IdentifyOre::src_depth_      (480, 640, CV_8UC3);
cv::Mat IdentifyOre::src_color_HSV_  (480, 640, CV_8UC3);
cv::Mat IdentifyOre::color_mask_0_   (480, 640, CV_8UC3);
cv::Mat IdentifyOre::color_mask_1_   (480, 640, CV_8UC3);
cv::Mat IdentifyOre::dst_color_      (480, 640, CV_8UC3);

static OrePara orePara = OreParaFactory::getOrePara();

void IdentifyOre::OreIdentifyStream(cv::Mat *import_src_color, cv::Mat* import_src_depth, int64 *sent_data) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    IdentifyTool::CreatTrackbars(&hmin_0_, &hmax_0_, &smin_0_, &smax_0_, &vmin_0_, &vmax_0_,
                           &hmin_1_, &hmax_1_, &smin_1_, &smax_1_, &vmin_1_, &vmax_1_,
                            &open_,    &close_,    &erode_,   &dilate_);
    while (true){
        if(_near_thread_state_flag){
            if (mutex_depth.try_lock()) {
                temp_src_color = *import_src_color;
                temp_src_depth = *import_src_depth;
                mutex_depth.unlock();
            }
            if (!temp_src_color.empty()){
                temp_src_color.copyTo(src_color_);
                temp_src_depth.copyTo(src_depth_);
            }

            ImagePreprocess(src_color_);
            SearchOre(dst_color_);
            DepthCalculation();
            TargetSelection();

            if (SwitchControl::functionConfig_._enable_ore_drop_detection){
                DropDetection();
            }

            if (target_ore_index_ >= 0){
                if (mutex_serial_port_data_ore.try_lock()){
                    int temp_target_x = ore_structs_[target_ore_index_].ore_rect.center.x;
                    int temp_target_y = ore_structs_[target_ore_index_].ore_rect.center.y;

                    //*sent_data = temp_target_x * 10000 + temp_target_y * 10 + temp_ore_fall;
                    *sent_data = temp_target_x * 1000 + temp_target_y;
                    mutex_serial_port_data_ore.unlock();
                }
            }
            else if (mutex_serial_port_data_ore.try_lock()){
                    *sent_data = 0;
                    mutex_serial_port_data_ore.unlock();
            }

            if (SwitchControl::functionConfig_._enable_debug_mode){
                AuxiliaryGraphicsDrawing();
            }
            ResourceRelease();
        }
    }
}

void IdentifyOre::ImagePreprocess(const cv::Mat &src) {
    cv::cvtColor(src, src_color_HSV_, CV_BGR2HSV, 0);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_0_, smin_0_, vmin_0_),
                cv::Scalar(hmax_0_, smax_0_, vmax_0_),
                color_mask_0_);
    cv::inRange(src_color_HSV_, cv::Scalar(hmin_1_, smin_1_, vmin_1_),
                cv::Scalar(hmax_1_, smax_1_, vmax_1_),
                color_mask_1_);

    //cv::imshow("ore", src_color_);
    //cv::imshow("mask1", color_mask_1_);

    color_mask_0_ = color_mask_1_ | color_mask_0_;

    morphologyEx(color_mask_0_, dst_color_,   2, getStructuringElement(cv::MORPH_RECT,cv::Size(open_,   open_)));
    morphologyEx(dst_color_,    dst_color_,   3, getStructuringElement(cv::MORPH_RECT,cv::Size(close_,  close_)));
    morphologyEx(dst_color_,    dst_color_,   0, getStructuringElement(cv::MORPH_RECT,cv::Size(erode_,  erode_)));
    morphologyEx(dst_color_,    dst_color_,   1, getStructuringElement(cv::MORPH_RECT,cv::Size(dilate_, dilate_)));
    //cv::imshow("Src", src);
    //cv::imshow("ImagePreprocess", dst_color_);
}

void IdentifyOre::SearchOre(cv::Mat &preprocessed) {
    cv::findContours(preprocessed, all_contours_, hierarchy_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < all_contours_.size(); ++i) {
        if (hierarchy_[i][3] == -1) {
            cv::RotatedRect scanRect = cv::minAreaRect(all_contours_[i]);                    //检测最小面积的矩形
            cv::Point2f vertices[4];
            scanRect.points(vertices);
            if (scanRect.size.area() < orePara.min_ore_area){
                continue;
            }
            if (IdentifyTool::getRectLengthWidthRatio(scanRect) < orePara.min_ore_length_width_ratio || IdentifyTool::getRectLengthWidthRatio(scanRect) > orePara.max_ore_length_width_ratio){
                continue;
            }
            suspected_ore_rects_.push_back(scanRect);
            suspected_ore_contours_.push_back(all_contours_[i]);
        }
    }
}

void IdentifyOre::ResourceRelease() {
    all_contours_.clear();                                //轮廓
    hierarchy_.clear();
    suspected_ore_rects_.clear();
    suspected_ore_contours_.clear();
    ore_structs_.clear();
    target_ore_index_ = -1;
    current_target_ore_location_ = IdentifyOre::TargetOreLocationStruct();
    _target_ore_fall_ = false;
}

void IdentifyOre::AuxiliaryGraphicsDrawing() {
    if (SwitchControl::functionConfig_._enable_debug_mode){
        if (target_ore_index_ >= 0){
            IdentifyTool::drawRotatedRect(src_color_, ore_structs_[target_ore_index_].ore_rect, cv::Scalar(15, 198, 150), 4, 16);
            /*
            if (_target_ore_fall_ == true){
                cv::circle(src_color_, current_target_ore_location_.target_ore_center, 15, cv::Scalar(0,0,255), 10);
            }*/
            cv::circle(src_color_, ore_structs_[target_ore_index_].ore_rect.center, 15, cv::Scalar(15, 198, 150), 10);

            /*
            std::cout << "找到[" << ore_structs_.size() << "]个矿石, 目标矿石中心点的平面坐标为["
                      << ore_structs_[target_ore_index_].ore_rect.center.x << "," << ore_structs_[target_ore_index_].ore_rect.center.y
                      << "],深度为[" << ore_structs_[target_ore_index_].ore_depth << "]，参考单位为[cm]" << "\n"
                      << "当前保存的矿石轨迹座标点数为[" << target_ore_track_.size() << "]"
                      << std::endl;
            */
        }



        cv::imshow("Ore",   src_color_);
        cv::imshow("Ore Mask",  dst_color_);
        //cv::imshow("Depth", src_depth_);
    }

}

void IdentifyOre::DepthCalculation() {
    float depth_scale2m = 0.001;//DepthTool::get_depth_scale(profile_.get_device());
    float depth_scale2cm = 0.1;

    if (!suspected_ore_rects_.empty() && !suspected_ore_contours_.empty()){
        for (int i = 0; i < suspected_ore_contours_.size(); ++i) {
            float estimated_depth = 0;
            for (int j = 0; j < suspected_ore_contours_[i].size(); ++j) {
                estimated_depth += depth_scale2cm * src_depth_.at<uint16_t>(suspected_ore_contours_[i][j].y,suspected_ore_contours_[i][j].x);
            }
            estimated_depth /= suspected_ore_contours_[i].size();

            IdentifyOre::OreStruct temp_orestruct;
            temp_orestruct.ore_depth = estimated_depth;
            temp_orestruct.ore_rect  = cv::minAreaRect(suspected_ore_contours_[i]);
            ore_structs_.push_back(temp_orestruct);
            //std::cout << estimated_depth << std::endl;
        }
    }
}

void IdentifyOre::TargetSelection() {
    if (ore_structs_.size() < 1){
        target_ore_index_ = -1;
        return;
    }
    else if (ore_structs_.size() == 1){
        target_ore_index_ = 0;
        return;
    }
    else{
        int max_ore_rect_area = 0;
        for (int i = 0; i < ore_structs_.size(); ++i) {
            if (ore_structs_[i].ore_rect.size.area() > max_ore_rect_area){
                target_ore_index_ = i;
                max_ore_rect_area = ore_structs_[i].ore_rect.size.area();
            }
        }
    }

}

void IdentifyOre::DropDetection() {
    if (target_ore_index_ >= 0){
        current_target_ore_location_.target_ore_center = cv::Point2f(ore_structs_[target_ore_index_].ore_rect.center.x, ore_structs_[target_ore_index_].ore_rect.center.y);
        current_target_ore_location_.target_ore_depth = ore_structs_[target_ore_index_].ore_depth;
        current_target_ore_location_.target_ore_radius = (ore_structs_[target_ore_index_].ore_rect.size.height + ore_structs_[target_ore_index_].ore_rect.size.height) / 2;

        if (target_ore_track_.empty()){
            target_ore_track_.push_back(current_target_ore_location_.target_ore_center);
        }

        else if(IdentifyTool::getTwoPointDistance(current_target_ore_location_.target_ore_center, target_ore_track_.back()) < current_target_ore_location_.target_ore_radius){
            if (target_ore_track_.size() >= orePara.ore_track_point_records_num){
                std::vector<cv::Point2f>::iterator k = target_ore_track_.begin();
                target_ore_track_.erase(k);
            }
            target_ore_track_.push_back(current_target_ore_location_.target_ore_center);
        }

    }
    else{
        if (!target_ore_track_.empty()){
            std::vector<cv::Point2f>::iterator k = target_ore_track_.begin();
            target_ore_track_.erase(k);
        }
    }

    if(target_ore_track_.size() >= orePara.ore_track_point_records_num * 0.5){

        float sum_x = 0;
        for (int i = 0; i < target_ore_track_.size(); ++i) {
            sum_x += target_ore_track_[i].x;
        }
        //float sum_x = std::accumulate(std::begin(target_ore_track_), std::end(target_ore_track_), 0.0);
        float mean_x = sum_x / target_ore_track_.size();
        float variance_x  = 0.0;
        for (uint16_t i = 0 ; i < target_ore_track_.size() ; i++)
        {
            variance_x = variance_x + pow(target_ore_track_[i].x-mean_x,2);
        }
        variance_x /= target_ore_track_.size();

        float sum_y = 0;
        for (int i = 0; i < target_ore_track_.size(); ++i) {
            sum_y += target_ore_track_[i].y;
        }
        //float sum_x = std::accumulate(std::begin(target_ore_track_), std::end(target_ore_track_), 0.0);
        float mean_y = sum_y / target_ore_track_.size();
        float variance_y  = 0.0;
        for (uint16_t i = 0 ; i < target_ore_track_.size() ; i++)
        {
            variance_y = variance_y + pow(target_ore_track_[i].y-mean_y,2);
        }
        variance_y /= target_ore_track_.size();

        //float fit_line_slope = IdentifyTool::trackLineFit(target_ore_track_);
        //std::cout << "轨迹点拟合斜率为[" << fit_line_slope  << "]" << std::endl;
        if (/*abs(fit_line_slope) >= 20 &&*/ variance_y > 100 * variance_x){
            //cv::circle(src_color_, current_target_ore_location_.target_ore_center, 15, cv::Scalar(0,0,255), 10);
            if (current_target_ore_location_.target_ore_depth < orePara.catch_mode_max_trigger_distance && current_target_ore_location_.target_ore_depth > orePara.catch_mode_min_trigger_distance){
                cv::circle(src_color_, current_target_ore_location_.target_ore_center, 20, cv::Scalar(150,100,255), 5);
                _target_ore_fall_ = true;
            }
        }
        //cv::line(src_color_, cv::Point(320,240), cv::Point(320 + 50, 240 + 50 * fit_line_slope), cv::Scalar (0,0,255), 2);
    }
}