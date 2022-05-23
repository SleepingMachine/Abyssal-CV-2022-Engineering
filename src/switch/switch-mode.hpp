//
// Created by sleepingmachine on 22-5-21.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_SWITCH_MODE_HPP
#define ABYSSAL_CV_2022_ENGINEERING_SWITCH_MODE_HPP

#include "iostream"
#include <opencv2/opencv.hpp>
#include <atomic>
//#include <opencv2/imgproc.hpp>
#include "switch-tool.hpp"
#include "../src/depth/depth-analysis.hpp"
#include "../ore/ore-identify.hpp"
#include "../src/box/box-identify.hpp"

class SwitchControl {
private:
    static void InitColorThresholdParameters();
public:
    SwitchControl();
    ~SwitchControl();

    static void SwitchMode(cv::Mat* import_src_color, cv::Mat* import_src_depth, int* sent_serial_port_data);
};
#endif //ABYSSAL_CV_2022_ENGINEERING_SWITCH_MODE_HPP
