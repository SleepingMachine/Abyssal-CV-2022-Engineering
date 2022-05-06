//
// Created by sleepingmachine on 22-5-6.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_CONTROL_SWITCH_HPP
#define ABYSSAL_CV_2022_ENGINEERING_CONTROL_SWITCH_HPP
class ControlSwitch{
public:
    ControlSwitch();
    ~ControlSwitch() {};

    //static void SwitchMode(cv::Mat *pFrame, int *sentPortData);
private:
    static int sendData;
    static int lastMode;
    //static cv::Mat src;
    static int getFrameErrorCounter;
    static void initColorThresholdParameters();
};
#endif //ABYSSAL_CV_2022_ENGINEERING_CONTROL_SWITCH_HPP
