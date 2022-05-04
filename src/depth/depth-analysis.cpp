//
// Created by sleepingmachine on 22-4-15.
//

#include "depth-analysis.hpp"

cv::Mat DepthSolution::src_color_         (480, 640, CV_8UC3);
cv::Mat DepthSolution::src_depth_         (480, 640, CV_8UC3);
//cv::Mat DepthSolution::mask_depth_filter_ (480, 640, CV_8UC3);

extern std::mutex mutex_depth_solution;;
extern std::atomic_bool CameraisOpen;

FunctionConfig DepthSolution::functionConfig_ = FunctionConfigFactory::getFunctionConfig();

void DepthSolution::DepthSolutionStream(cv::Mat *import_src_color, cv::Mat *import_src_depth) {
    cv::Mat temp_src_color(480, 640, CV_8UC3);
    cv::Mat temp_src_depth(480, 640, CV_8UC3);

    while (CameraisOpen){
        if (mutex_depth_solution.try_lock()) {
            temp_src_color = *import_src_color;
            temp_src_depth = *import_src_depth;
            mutex_depth_solution.unlock();
        }
        if (!temp_src_color.empty() && !temp_src_depth.empty()){
            temp_src_color.copyTo(src_color_);
            temp_src_depth.copyTo(src_depth_);
        }
        DeepConversion();
    }
}

void DepthSolution::DeepConversion() {
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = 0.001;//DepthTool::get_depth_scale(profile_.get_device());

    cv::Mat mask_depth_filter(480, 640, CV_8UC3);

    //mask_depth_filter = src_depth_.clone();
    mask_depth_filter.setTo(0);

    for(int y = 0; y < 480; y++){
        for(int x = 0; x < 640; x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth_scale*src_depth_.at<uint16_t>(y,x) * 1000 < functionConfig_.grip_mode_max_recognition_distance
                && depth_scale*src_depth_.at<uint16_t>(y,x) * 1000 > functionConfig_.grip_mode_min_recognition_distance){
                mask_depth_filter.at<uint16_t>(y,x) = (uint16_t)255;
            }
            else if (src_depth_.at<uint16_t>(y,x) == 0){
                mask_depth_filter.at<uint16_t>(y,x) = (uint16_t)255;
            }
            else
            {
                mask_depth_filter.at<uint16_t>(y,x) = (uint16_t)0;
            }
        }
    }
    cv::imshow("1111", mask_depth_filter);
    //cv::threshold(mask_depth_filter_, mask_depth_filter_, 127, 255, cv::THRESH_BINARY);
    //cv::threshold(mask_depth_filter_, mask_depth_filter_, 125,255, cv::THRESH_BINARY_INV);
    //src_color_ = src_color_ & mask_depth_filter;

    /*
    std::cout<<"遍历完成，有效像素点:"<<effective_pixel<<std::endl;
    float effective_distance=distance_sum/effective_pixel;
    std::cout<<"目标距离："<<effective_distance<<" m"<<std::endl;
    char distance_str[30];
    sprintf(distance_str,"the distance is:%f m",effective_distance);
    cv::putText(src_depth_,(std::string)distance_str,cv::Point(src_depth_.cols*0.02,src_depth_.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(255,255,255),2,8);
    */
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