//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_TOOLS_CONFIG_HPP
#define ABYSSAL_CV_2022_ENGINEERING_TOOLS_CONFIG_HPP

#include <string>
#include "../asset/robomaster-define.hpp"

struct FunctionConfig
{
    int _operating_mode                            = OperatingMode::EXCHANGE_MODE;
    //可以载入本地视频或是使用usb免驱摄像头用于测试，需要注意无法读取到深度信息
    int _camera_type                               = REALSENSE_CAMERA;

    bool _enable_save_video                          = false;

    //bool _enable_local_video_stream                = true;
    std::string local_video_path                   = "/home/sleepingmachine/视频/box2.mp4";

    bool _enable_ore_drop_detection                = false;
    bool _enable_debug_mode                        = true;
};

class FunctionConfigFactory{
private:
    static FunctionConfigFactory &instance() {
        static FunctionConfigFactory serialConfigFactory;
        return serialConfigFactory;
    }

public:
    FunctionConfig functionConfig;
    static FunctionConfig getFunctionConfig() {
        return instance().functionConfig;
    }

    static void resetAllConfig() {
        instance().functionConfig = FunctionConfig();
    }
};


struct CameraPara {
    float depth_scale2m                   = 0.001;
    float depth_scale2cm                  = depth_scale2m * 100;
    int   min_recognition_distance_near   = 40;
    int   max_recognition_distance_near   = 100;
    int   min_recognition_distance_far    = 80;
    int   max_recognition_distance_far    = 200;
    int   min_recognition_distance_search = 80;
    int   max_recognition_distance_search = 180;
};

class CameraParaFactory {
private:
    static CameraParaFactory &instance() {
        static CameraParaFactory cameraParaFactory;
        return cameraParaFactory;
    }

public:
    static CameraPara getCameraPara() {
        return instance().cameraPara;
    }

    static void resetAllConfig() {
        instance().cameraPara = CameraPara();
    }

public:
    CameraPara cameraPara;
};

//矿石参数
struct OrePara {

    int   min_ore_area                            = 1200;
    float min_ore_length_width_ratio              = 0.6;
    float max_ore_length_width_ratio              = 1.8;

    float grip_mode_min_recognition_distance      = 60.0;
    float grip_mode_max_recognition_distance      = 120.0;

    float catch_mode_min_recognition_distance     = 75.0;
    float catch_mode_max_recognition_distance     = 140.0;
    float catch_mode_max_trigger_distance         = 115;
    float catch_mode_min_trigger_distance         = 80;

    int   ore_track_point_records_num             = 100;
};

class OreParaFactory {
private:
    static OreParaFactory &instance() {
        static OreParaFactory oreParaFactory;
        return oreParaFactory;
    }

public:
    static OrePara getOrePara() {
        return instance().orePara;
    }

    static void resetAllConfig() {
        instance().orePara = OrePara();
    }

public:
    OrePara orePara;
};

struct BoxPara {
    int   min_suspected_box_components_area      = 100;

    float exchange_mode_min_recognition_distance  = 70.0;
    float exchange_mode_max_recognition_distance  = 300.0;

    float min_suspected_box_length_width_ratio    = 0.6;
    float max_suspected_box_length_width_ratio    = 1.6;

    float min_suspected_box_components_duty_cycle = 0.2;
    float max_suspected_box_components_duty_cycle = 0.7;


};

class BoxParaFactory {
private:
    static BoxParaFactory &instance() {
        static BoxParaFactory boxParaFactory;
        return boxParaFactory;
    }

public:
    static BoxPara getBoxPara() {
        return instance().boxPara;
    }

    static void resetAllConfig() {
        instance().boxPara = BoxPara();
    }

public:
    BoxPara boxPara;
};

struct SerialConfig
{
    std::string readPortPath  = "/dev/ttyUSB0";
    std::string writePortPath  = "/dev/ttyUSB0";
    int baud_write_Port = 460800;
    int baud_read_Port = 460800;
};

class SerialConfigFactory{
private:
    static SerialConfigFactory &instance() {
        static SerialConfigFactory serialConfigFactory;
        return serialConfigFactory;
    }

public:
    SerialConfig serialConfig;
    static SerialConfig getSerialConfig() {
        return instance().serialConfig;
    }

    static void resetAllConfig() {
        instance().serialConfig = SerialConfig();
    }
};
#endif //ABYSSAL_CV_2022_ENGINEERING_TOOLS_CONFIG_HPP
