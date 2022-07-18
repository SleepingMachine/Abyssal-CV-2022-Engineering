//
// Created by sleepingmachine on 22-4-15.
//

#include "depth-analysis.hpp"

cv::Mat DepthSolution::src_color_          (480, 640, CV_8UC3);
cv::Mat DepthSolution::src_depth_          (480, 640, CV_8UC3);
cv::Mat DepthSolution::dst_depth_analysis_ (480, 640, CV_8UC3);
//cv::Mat DepthSolution::mask_depth_filter_ (480, 640, CV_8UC3);

extern std::mutex mutex_camera;
extern std::mutex mutex_depth_analysis;
extern std::atomic_bool camera_is_open;

FunctionConfig DepthSolution::functionConfig_ = FunctionConfigFactory::getFunctionConfig();
static OrePara orePara = OreParaFactory::getOrePara();
static BoxPara boxPara = BoxParaFactory::getBoxPara();

void DepthSolution::DepthSolutionStream(cv::Mat *import_src_color, cv::Mat *import_src_depth, cv::Mat* export_dst_color, cv::Mat* export_dst_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    while (camera_is_open){
        if (mutex_camera.try_lock()) {
            temp_src_color = *import_src_color;
            temp_src_depth = *import_src_depth;
            mutex_camera.unlock();
        }
        if (!temp_src_color.empty() && !temp_src_depth.empty()){
            temp_src_color.copyTo(src_color_);
            temp_src_depth.copyTo(src_depth_);
        }

        DeepConversion();
        //DrawReferenceGraphics();

        if (mutex_camera.try_lock()) {
            dst_depth_analysis_.copyTo(*export_dst_color);
            src_depth_.copyTo(*export_dst_depth);
            mutex_camera.unlock();
        }
    }
}

void DepthSolution::DeepConversion() {
    //获取深度像素与现实单位比例
    float depth_scale2m = 0.001;//DepthTool::get_depth_scale(profile_.get_device());

    float depth_scale2cm = 0.1;

    cv::Mat mask_depth_filter(480, 640, CV_8UC1);

    //mask_depth_filter = src_depth_.clone();
    mask_depth_filter.setTo(255);
    dst_depth_analysis_.setTo(0);

    for (int y = 0; y < src_depth_.rows; ++y) {
        for (int x = 0; x < src_depth_.cols; ++x) {
            //std::cout << depth_scale * src_depth_.at<uint16_t>(y,x) << std::endl;
            switch (functionConfig_._mining_mode) {
                case 0:
                    if ((depth_scale2cm * src_depth_.at<uint16_t>(y,x) > orePara.grip_mode_max_recognition_distance) || (depth_scale2cm * src_depth_.at<uint16_t>(y,x) < orePara.grip_mode_min_recognition_distance)){
                        mask_depth_filter.at<uchar>(y, x) = 0;
                    }
                    break;
                case 1:
                    if ((depth_scale2cm * src_depth_.at<uint16_t>(y,x) > orePara.catch_mode_max_recognition_distance) || (depth_scale2cm * src_depth_.at<uint16_t>(y,x) < orePara.catch_mode_min_recognition_distance)){
                        mask_depth_filter.at<uchar>(y, x) = 0;
                    }
                    break;
                case 2:
                    if ((depth_scale2cm * src_depth_.at<uint16_t>(y,x) > boxPara.exchange_mode_max_recognition_distance) || (depth_scale2cm * src_depth_.at<uint16_t>(y,x) < boxPara.exchange_mode_min_recognition_distance)){
                        mask_depth_filter.at<uchar>(y, x) = 0;
                    }
                    break;
            }
        }
    }
    src_color_.copyTo(dst_depth_analysis_, mask_depth_filter);
}

void DepthSolution::DrawReferenceGraphics() {
    if (functionConfig_._enable_debug_mode){
        cv::rectangle(dst_depth_analysis_, cv::Point(295,215), cv::Point(345,265), cv::Scalar(255,0,0), 2);
    }
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}