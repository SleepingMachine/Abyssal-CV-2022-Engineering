//
// Created by sleepingmachine on 22-4-15.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
#define ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
#include <string>

typedef enum {
    GRIP_MODE  = 0,
    CATCH_MODE,
    EXCHANGE_MODE
} MiningMode;

struct FunctionConfig
{

    int _mining_mode                             = EXCHANGE_MODE;

    int ore_track_point_records_num              = 100;

    bool _enableSaveVideo                        = false;

    //载入本地视频用于测试，需要注意本地视频无法读取到深度信息
    bool _enable_local_video_stream              = true;
    std::string local_video_path                 = "/home/sleepingmachine/视频/box2.mp4";

    bool _enable_debug_mode                      = false;
/*
    // RED  => false
    // BLUE => true
    bool _enemyColor                = true;

    bool _enableEnergyBuffMode      = false;

    bool _enableSaveVideo           = false;
    bool _enableRoiScaling          = true;

    bool _enableLocalVideoStreaming = true;
    //std::string localVideoPath = "/home/sleepingmachine/视频/lbb_fan.mp4";
    //std::string localVideoPath = "/home/sleepingmachine/视频/lbr_fan.mp4";
    std::string localVideoPath = "/home/sleepingmachine/RoboMaster-Code/Abyssal-CV-2022/asset/AutoSaveVideo/2022.03.26 14-57-45.avi";
*/
    //bool _enableDebugMode           = true;

    float grip_mode_min_recognition_distance     = 60.0;
    float grip_mode_max_recognition_distance     = 120.0;

    float catch_mode_min_recognition_distance    = 75.0;
    float catch_mode_max_recognition_distance    = 140.0;

    float exchange_mode_min_recognition_distance = 70.0;
    float exchange_mode_max_recognition_distance = 120.0;
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

//敌方颜色
typedef enum {
    ENEMY_RED = 0,
    ENEMY_BLUE
} EnemyColor;

//我方颜色
typedef enum {
    OWN_RED = 0,
    OWN_BLUE
} OwnColor;

//长短边
enum EXTREMUM {
    SHORT_SIDE = 0,
    LONG_SIDE
};

typedef enum {
    InitMode = 0,
    ArmorMode,
    EnergyBuffMode
} NowMode;

//装甲板参数
struct OrePara {
    EnemyColor enemyColor;
    int min_ore_area_ = 800;
    float min_ore_length_width_ratio = 0.5;
    float max_ore_length_width_ratio = 2;

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

struct SerialConfig
{
    std::string readPortPath  = "/dev/ttyUSB0";
    std::string writePortPath  = "/dev/ttyUSB1";
    int baud_writePort = 460800;
    int baud_readPort = 460800;
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

#endif //ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
