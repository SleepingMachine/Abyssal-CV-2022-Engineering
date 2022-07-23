//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_SERIAL_PORT_HPP
#define ABYSSAL_CV_2022_ENGINEERING_SERIAL_PORT_HPP

#include "../tools/tools-config.hpp"
//#include "../src/depth/depth-analysis.hpp"

#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string>
#include <atomic>
#include <time.h>

class SerialPort{
public:
    SerialPort();
    ~SerialPort();

    static void GetHitPointData(int tempData);
    static void SendData(int64* sentData);

private:
    static std::string read_device_;
    static std::string write_device_;
    static int baud_write_;
    static int baud_read_;

    static char sent_data_[8];
    static char read_data_[3];

    static char cache_read_data_[6];
    //static bool _cache_read_data_wait_flag_;

    static void CheckPortAvailability();
    static void GetSerialInfo();

};

#endif //ABYSSAL_CV_2022_ENGINEERING_SERIAL_PORT_HPP
