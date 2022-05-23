//
// Created by sleepingmachine on 22-5-21.
//

#include "switch-mode.hpp"

extern std::mutex mutex_depth_analysis;
extern std::atomic_bool camera_is_open;

void SwitchControl::SwitchMode(cv::Mat *import_src_color, cv::Mat *import_src_depth, int *sent_serial_port_data) {
    InitColorThresholdParameters();

    if (DepthSolution::functionConfig_._mining_mode == MiningMode::GRIP_MODE || DepthSolution::functionConfig_._mining_mode == MiningMode::CATCH_MODE){
        OreTool::CreatTrackbars(&IdentifyOre::hmin_0_, &IdentifyOre::hmax_0_, &IdentifyOre::smin_0_, &IdentifyOre::smax_0_, &IdentifyOre::vmin_0_, &IdentifyOre::vmax_0_,
                                &IdentifyOre::hmin_1_, &IdentifyOre::hmax_1_, &IdentifyOre::smin_1_, &IdentifyOre::smax_1_, &IdentifyOre::vmin_1_, &IdentifyOre::vmax_1_,
                                &IdentifyOre::open_, &IdentifyOre::close_, &IdentifyOre::erode_, &IdentifyOre::dilate_);
        while (camera_is_open){
            IdentifyOre::OreIdentifyStream(import_src_color, import_src_depth);
        }
    }
    else if (DepthSolution::functionConfig_._mining_mode == MiningMode::EXCHANGE_MODE){
        std::cout << "这个模式还没写 别急" << std::endl;
    }

}

void SwitchControl::InitColorThresholdParameters() {
    //矿石识别
    IdentifyOre::hmin_0_ = 0;
    IdentifyOre::hmax_0_ = 65;
    IdentifyOre::smin_0_ = 235;
    IdentifyOre::smax_0_ = 255;
    IdentifyOre::vmin_0_ = 115;
    IdentifyOre::vmax_0_ = 255;

    IdentifyOre::hmin_1_ = 0;
    IdentifyOre::hmax_1_ = 0;
    IdentifyOre::smin_1_ = 0;
    IdentifyOre::smax_1_ = 0;
    IdentifyOre::vmin_1_ = 0;
    IdentifyOre::vmax_1_ = 0;

    IdentifyOre::open_   = 1;
    IdentifyOre::close_  = 13;
    IdentifyOre::erode_  = 5;
    IdentifyOre::dilate_ = 3;
}
