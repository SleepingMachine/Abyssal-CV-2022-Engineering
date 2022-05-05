//
// Created by sleepingmachine on 22-4-15.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
#define ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
#include <string>

struct FunctionConfig
{
    // Catch Mode => false
    // Grip Mode  => true
    bool _mining_mode                        = true;
    float grip_mode_min_recognition_distance = 60.0;
    float grip_mode_max_recognition_distance = 120.0;

    bool _enable_debug_mode                  = false;
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
    int min_ore_area_ = 600;
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
#endif //ABYSSAL_CV_2022_ENGINEERING_ROBOMASTER_CONFIG_HPP
