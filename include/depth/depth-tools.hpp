//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_DEPTH_TOOLS_HPP
#define ABYSSAL_CV_2022_ENGINEERING_DEPTH_TOOLS_HPP

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class DepthTool{
public:
    static float get_depth_scale(rs2::device dev){
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
};

#endif //ABYSSAL_CV_2022_ENGINEERING_DEPTH_TOOLS_HPP
