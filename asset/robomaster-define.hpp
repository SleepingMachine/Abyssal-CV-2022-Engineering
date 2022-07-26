//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_DEFINE_HPP
#define ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_DEFINE_HPP

typedef enum {
    SEARCH_MODE  = 0,
    EXCHANGE_MODE
} OperatingMode;

typedef enum {
    REALSENSE_CAMERA  = 0,
    USB_CAMERA,
    LOCAL_VIDEO
} CameraType;

typedef enum {
    LOST  = 0,
    TWO_POINTS,
    THREE_POINTS,
    FULL_POINTS
} BoxIdentifyStatus;

#endif //ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_DEFINE_HPP
