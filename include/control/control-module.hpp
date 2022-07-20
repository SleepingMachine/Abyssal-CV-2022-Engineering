//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP
#define ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP

#include <iostream>

class SwitchControl {
private:

public:
    SwitchControl();
    ~SwitchControl();


    static int SwitchMode();
    static int ReadConfig();
};
#endif //ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP
