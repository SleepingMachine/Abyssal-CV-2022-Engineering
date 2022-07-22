//
// Created by sleepingmachine on 22-7-20.
//

#include "../include/depth/depth-analysis.hpp"

extern std::atomic_bool camera_start;

extern std::mutex mutex_camera;
extern std::mutex mutex_depth;

cv::Mat DepthSolution::src_color_               (480, 640, CV_8UC3);
cv::Mat DepthSolution::src_depth_               (480, 640, CV_8UC3);
cv::Mat DepthSolution::dst_depth_analysis_near_ (480, 640, CV_8UC3);
cv::Mat DepthSolution::dst_depth_analysis_far_  (480, 640, CV_8UC3);

int DepthSolution::DepthSolutionStream(cv::Mat* import_src_color, cv::Mat* import_src_depth, cv::Mat* export_dst_color_near, cv::Mat* export_dst_color_far, cv::Mat* export_dst_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);
    //int count = 0;
    //int loop_start_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    camera_start = true;
    while (true) {
        if (mutex_camera.try_lock()) {
            temp_src_color = *import_src_color;
            temp_src_depth = *import_src_depth;
            mutex_camera.unlock();
        }
        if (!temp_src_color.empty() && !temp_src_depth.empty()){
            temp_src_color.copyTo(src_color_);
            temp_src_depth.copyTo(src_depth_);
        }

        DeepSegmentation();

        //cv::imshow("color", src_color_);
        //cv::imshow("depth", src_depth_);
        //cv::waitKey(1);

        if (mutex_depth.try_lock()) {
            dst_depth_analysis_near_.copyTo(*export_dst_color_near);
            dst_depth_analysis_near_.copyTo(*export_dst_color_far);
            src_depth_.copyTo(*export_dst_depth);
            mutex_depth.unlock();
        }

        /*
        count++;
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - loop_start_time >= 1){
            loop_start_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout<<"帧率[" << count << "]" << std::endl;
            count = 0;
        }*/
    }

    return 0;
}

int DepthSolution::DeepSegmentation() {
    cv::Mat mask_depth_filter_near(480, 640, CV_8UC1);
    cv::Mat mask_depth_filter_far (480, 640, CV_8UC1);

    mask_depth_filter_near.setTo(255);
    mask_depth_filter_far .setTo(255);
    dst_depth_analysis_near_.setTo(0);
    dst_depth_analysis_far_ .setTo(0);

    if (SwitchControl::functionConfig_._operating_mode == OperatingMode::EXCHANGE_MODE){
        for (int y = 0; y < src_depth_.rows; ++y) {
            for (int x = 0; x < src_depth_.cols; ++x) {
                if ((CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) < CameraStream::cameraPara_.min_recognition_distance_near) ||
                    (CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) > CameraStream::cameraPara_.max_recognition_distance_near)){
                    mask_depth_filter_near.at<uchar>(y, x) = 0;
                }

                if(CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) == 0){
                    mask_depth_filter_near.at<uchar>(y, x) = 255;
                }
            }
        }
        //cv::blur(mask_depth_filter_near, mask_depth_filter_near, Size(5, 5));
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask_depth_filter_near, mask_depth_filter_near, MORPH_OPEN, kernel, Point(-1, -1));
        morphologyEx(mask_depth_filter_near, mask_depth_filter_near, MORPH_CLOSE, kernel, Point(-1, -1));
        cv::GaussianBlur(mask_depth_filter_near, mask_depth_filter_near, Size(3, 3), 0);
        src_color_.copyTo(dst_depth_analysis_near_, mask_depth_filter_near);

        for (int y = 0; y < src_depth_.rows; ++y) {
            for (int x = 0; x < src_depth_.cols; ++x) {
                if ((CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) < CameraStream::cameraPara_.min_recognition_distance_far) ||
                    (CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) > CameraStream::cameraPara_.max_recognition_distance_far)){
                    mask_depth_filter_far.at<uchar>(y, x) = 0;
                }

                if(CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) == 0){
                    mask_depth_filter_far.at<uchar>(y, x) = 255;
                }
            }
        }
        //cv::blur(mask_depth_filter_near, mask_depth_filter_near, Size(5, 5));
        morphologyEx(mask_depth_filter_far, mask_depth_filter_far, MORPH_OPEN, kernel, Point(-1, -1));
        morphologyEx(mask_depth_filter_far, mask_depth_filter_far, MORPH_CLOSE, kernel, Point(-1, -1));
        cv::GaussianBlur(mask_depth_filter_far, mask_depth_filter_far, Size(3, 3), 0);
        src_color_.copyTo(dst_depth_analysis_far_, mask_depth_filter_far);
    }
    else if (SwitchControl::functionConfig_._operating_mode == OperatingMode::SEARCH_MODE){
        for (int y = 0; y < src_depth_.rows; ++y) {
            for (int x = 0; x < src_depth_.cols; ++x) {
                if ((CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) < CameraStream::cameraPara_.min_recognition_distance_search) ||
                    (CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) > CameraStream::cameraPara_.max_recognition_distance_search)){
                    mask_depth_filter_near.at<uchar>(y, x) = 0;
                }

                if(CameraStream::cameraPara_.depth_scale2cm * src_depth_.at<uint16_t>(y,x) == 0){
                    mask_depth_filter_near.at<uchar>(y, x) = 255;
                }
            }
        }
        //cv::blur(mask_depth_filter_near, mask_depth_filter_near, Size(5, 5));
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask_depth_filter_near, mask_depth_filter_near, MORPH_OPEN, kernel, Point(-1, -1));
        morphologyEx(mask_depth_filter_near, mask_depth_filter_near, MORPH_CLOSE, kernel, Point(-1, -1));
        cv::GaussianBlur(mask_depth_filter_near, mask_depth_filter_near, Size(3, 3), 0);
        src_color_.copyTo(dst_depth_analysis_near_, mask_depth_filter_near);
    }

    //cv::imshow("mask0", dst_depth_analysis_far_ );
    //cv::imshow("mask1", dst_depth_analysis_near_);
    return 0;
}

int DepthSolution::AuxiliaryGraphicsDrawing() {
    return 0;
}
