//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP
#define ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP

#include "../tools/tools-config.hpp"
#include <iostream>

class SwitchControl {
private:
    static int ReadConfig();
public:
    static int SwitchMode();
    static FunctionConfig functionConfig_;

    SwitchControl();
    ~SwitchControl();

};
#endif //ABYSSAL_CV_2022_ENGINEERING_CONTROL_MODULE_HPP
