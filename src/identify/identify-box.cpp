//
// Created by sleepingmachine on 22-7-22.
//
#include "identify/identify-box.hpp"

extern std::mutex mutex_depth;

extern std::atomic_bool _far_thread_state_flag;

IdentifyBox::IdentifyBox() {}

BoxPara IdentifyBox::boxPara_ = BoxParaFactory::getBoxPara();

int IdentifyBox::open_   = 1;
int IdentifyBox::close_  = 8;
int IdentifyBox::erode_  = 2;
int IdentifyBox::dilate_ = 4;

bool IdentifyBox::_find_box_flag = false;

cv::Mat IdentifyBox::src_color_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_depth_            (480, 640, CV_8UC3);
cv::Mat IdentifyBox::src_gray_             (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_data_  (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_       (480, 640, CV_8UC3);
cv::Mat IdentifyBox::separation_src_green_ (480, 640, CV_8UC3);
cv::Mat IdentifyBox::dst_color_            (480, 640, CV_8UC3);

cv::Mat IdentifyBox::target_graphics_box_components_ = imread("../asset/box_component_250.jpge");

std::vector<cv::Mat> IdentifyBox::split_src_;

std::vector<std::vector<cv::Point2i>> IdentifyBox::all_contours_;
std::vector<cv::Vec4i> IdentifyBox::hierarchy_;
std::vector<cv::Point2i> IdentifyBox::box_components_inside_corners_;

IdentifyBox::BoxStruct IdentifyBox::target_box;

std::vector<std::vector<cv::Point2i>> IdentifyBox::suspected_box_components_contours_;
std::vector<cv::RotatedRect> IdentifyBox::suspected_box_components_rects_;

void IdentifyBox::BoxIdentifyStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);
    IdentifyTool::CreatTrackbars(&open_,&close_,&erode_,&dilate_);

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
            //BoxComponentsFilter();
            BoxPairing();
            AuxiliaryGraphicsDrawing();
            ResourceRelease();
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

    cv::Mat dst_gray;
    cv::cvtColor(target_graphics_box_components_,dst_gray, COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2i>> target_contours;
    cv::findContours(dst_gray, target_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    Moments m_target = moments(target_contours[0]);
    Mat hu_target;
    HuMoments(m_target, hu_target);

    //vector<double> filter_value;
    //vector<int>    filter_index;
    for(int i = 0; i < all_contours_.size(); ++i){
        if (hierarchy_[i][3] == -1){
            cv::RotatedRect scanRect = cv::minAreaRect(all_contours_[i]);

            if (scanRect.size.area() < boxPara_.min_suspected_box_components_area){
                continue;
            }

            if (IdentifyTool::getRectLengthWidthRatio(scanRect) < boxPara_.min_suspected_box_length_width_ratio ||
                IdentifyTool::getRectLengthWidthRatio(scanRect) > boxPara_.max_suspected_box_length_width_ratio){
                continue;
            }
            if (cv::contourArea( all_contours_[i], false ) / scanRect.size.area() <= boxPara_.min_suspected_box_components_duty_cycle ||
                cv::contourArea( all_contours_[i], false ) / scanRect.size.area() >= boxPara_.max_suspected_box_components_duty_cycle){
                continue;
            }

            Moments m_src = moments(all_contours_[i]);
            Mat hu_src;
            HuMoments(m_src, hu_src);
            matchShapes(hu_target, hu_src, CONTOURS_MATCH_I1, 0);

            //filter_index.push_back(i);
            //filter_value.push_back(matchShapes(hu_target, hu_src, CONTOURS_MATCH_I1, 0));

            double dist = matchShapes(hu_target, hu_src, CONTOURS_MATCH_I1, 0);
            if (dist > boxPara_.max_suspected_box_components_hu_value)
            {
                continue;
            }

            suspected_box_components_rects_.push_back(scanRect);
            suspected_box_components_contours_.push_back(all_contours_[i]);
        }
    }

    /*
    for (int i = 0; i < filter_index.size(); i++) {
        for (int j = 0; j < filter_index.size() - i; j++) {
            if (filter_value[j] > filter_value[j+1]) {        // 相邻元素两两对比
                double temp_value = filter_value[j+1];        // 元素交换
                int    temp_index = filter_index[j+1];

                filter_value[j+1] = filter_value[j];
                filter_value[j] = temp_value;

                filter_index[j+1] = filter_index[j];
                filter_index[j] = temp_index;
            }
        }
    }
    for (int i = 0; filter_index.size(); ++i) {
        //suspected_box_components_rects_.push_back(scanRect);
        if (i <= 3){
            suspected_box_components_contours_.push_back(all_contours_[filter_index[i]]);
        }
        }*/
        //std::cout << filter_value[i] << " " << filter_index[i] << std::endl;
}

void IdentifyBox::AuxiliaryGraphicsDrawing() {
    for (int i = 0; i < suspected_box_components_rects_.size(); ++i) {
        IdentifyTool::drawRotatedRect(src_color_, suspected_box_components_rects_[i], cv::Scalar(15, 198, 150), 2, 16);
    }
    if(_find_box_flag){
        cv::line(src_color_, target_box.box_components_UL, target_box.box_components_LL, cv::Scalar(124,211,32),2);
        cv::line(src_color_, target_box.box_components_UR, target_box.box_components_LR, cv::Scalar(124,211,32),2);
        cv::line(src_color_, target_box.box_components_UL, target_box.box_components_UR, cv::Scalar(124,211,32),2);
        cv::line(src_color_, target_box.box_components_LL, target_box.box_components_LR, cv::Scalar(124,211,32),2);
        cv::circle(src_color_, target_box.box_center, 15, cv::Scalar(124,211,32), 10);
    }

    cv::imshow("0", src_color_);
    cv::imshow("1", dst_color_);
}

void IdentifyBox::ResourceRelease() {
    suspected_box_components_contours_.clear();
    suspected_box_components_rects_   .clear();
    box_components_inside_corners_    .clear();
    target_box = IdentifyBox::BoxStruct();
    _find_box_flag = false;
}

void IdentifyBox::BoxPairing() {
    std::vector<cv::Point2i> hull;
    for (int i = 0; i < suspected_box_components_contours_.size(); ++i) {
        cv::convexHull(suspected_box_components_contours_[i],hull);
        cv::Point2i right_angle_point;
        int counter_max = 0;
        for (int j = 0; j < suspected_box_components_contours_[i].size(); ++j) {
            int counter_min = 640;
            for (int k = 0; k < hull.size(); ++k) {
                int counter = IdentifyTool::getTwoPointDistance(suspected_box_components_contours_[i][j],hull[k]);
                if (counter < counter_min){
                    counter_min = counter;
                }
            }
            if (counter_min > counter_max){
                counter_max = counter_min;
                right_angle_point = suspected_box_components_contours_[i][j];
            }
        }
        box_components_inside_corners_.push_back(right_angle_point);
        //circle(src_color_, right_angle_point, 4, Scalar(255, 255, 0), 2, 8, 0);//点
    }

    /*
    for (int i = 0; i < box_components_inside_corners_.size(); i++) {
        for (int j = 0; j < box_components_inside_corners_.size() - i; j++) {
            if (box_components_inside_corners_[j].x < box_components_inside_corners_[j+1].x) {        // 相邻元素两两对比
                cv::Point2i temp = box_components_inside_corners_[j+1];        // 元素交换
                box_components_inside_corners_[j+1] = box_components_inside_corners_[j];
                box_components_inside_corners_[j] = temp;
            }
        }
    }
    if (box_components_inside_corners_.size() == 4){
        target_box.box_components_UL = box_components_inside_corners_[0];
        target_box.box_components_UR = box_components_inside_corners_[1].y < box_components_inside_corners_[2].y ? box_components_inside_corners_[1] : box_components_inside_corners_[2];
        target_box.box_components_UL = box_components_inside_corners_[3];
        target_box.box_components_LL = box_components_inside_corners_[1].y > box_components_inside_corners_[2].y ? box_components_inside_corners_[1] : box_components_inside_corners_[2];
        target_box.box_center =  IdentifyTool::getCrossPoint(target_box.box_components_UL, target_box.box_components_LR, target_box.box_components_UR, target_box.box_components_LL);
        _find_box_flag = true;
    }
    else{
        _find_box_flag = false;
    }*/


    float box_reference_center_x;
    float box_reference_center_y;
    for (int i = 0; i < box_components_inside_corners_.size(); ++i) {
        box_reference_center_x += box_components_inside_corners_[i].x;
        box_reference_center_y += box_components_inside_corners_[i].y;
    }
    box_reference_center_x /= box_components_inside_corners_.size();
    box_reference_center_y /= box_components_inside_corners_.size();
    for (int i = 0; i < box_components_inside_corners_.size(); ++i) {
        if(box_components_inside_corners_[i].x < box_reference_center_x && box_components_inside_corners_[i].y < box_reference_center_y){
            target_box.box_components_UL = box_components_inside_corners_[i];
        }
        else if(box_components_inside_corners_[i].x > box_reference_center_x && box_components_inside_corners_[i].y < box_reference_center_y){
            target_box.box_components_UR = box_components_inside_corners_[i];
        }
        else if(box_components_inside_corners_[i].x > box_reference_center_x && box_components_inside_corners_[i].y > box_reference_center_y){
            target_box.box_components_LR = box_components_inside_corners_[i];
        }
        else if(box_components_inside_corners_[i].x < box_reference_center_x && box_components_inside_corners_[i].y > box_reference_center_y){
            target_box.box_components_LL = box_components_inside_corners_[i];
        }
    }
    if(target_box.box_components_LL.x && target_box.box_components_LL.y &&
       target_box.box_components_LR.x && target_box.box_components_LR.y &&
       target_box.box_components_UL.x && target_box.box_components_UL.y &&
       target_box.box_components_UR.x && target_box.box_components_UR.y){
        _find_box_flag = true;
        target_box.box_center =  IdentifyTool::getCrossPoint(target_box.box_components_UL, target_box.box_components_LR, target_box.box_components_UR, target_box.box_components_LL);
    }
    else{
        _find_box_flag = false;
    }
}



