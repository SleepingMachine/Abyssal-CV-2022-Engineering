//
// Created by sleepingmachine on 22-5-21.
//

#include "switch-mode.hpp"

extern std::mutex mutex_depth_analysis;
extern std::mutex mutex_serial_port_data;
extern std::atomic_bool camera_is_open;

long SwitchControl::sent_data_;

void SwitchControl::SwitchMode(cv::Mat *import_src_color, cv::Mat *import_src_depth, int64* sent_serial_port_data) {
    InitColorThresholdParameters();
    if(DepthSolution::functionConfig_._enable_debug_mode){
        if (DepthSolution::functionConfig_._mining_mode == MiningMode::GRIP_MODE || DepthSolution::functionConfig_._mining_mode == MiningMode::CATCH_MODE){
            OreTool::CreatTrackbars(&IdentifyOre::hmin_0_, &IdentifyOre::hmax_0_, &IdentifyOre::smin_0_, &IdentifyOre::smax_0_, &IdentifyOre::vmin_0_, &IdentifyOre::vmax_0_,
                                    &IdentifyOre::hmin_1_, &IdentifyOre::hmax_1_, &IdentifyOre::smin_1_, &IdentifyOre::smax_1_, &IdentifyOre::vmin_1_, &IdentifyOre::vmax_1_,
                                    &IdentifyOre::open_, &IdentifyOre::close_, &IdentifyOre::erode_, &IdentifyOre::dilate_);
        }
        else if (DepthSolution::functionConfig_._mining_mode == MiningMode::EXCHANGE_MODE){
            BoxTool::CreatTrackbars(&IdentifyBox::hmin_0_, &IdentifyBox::hmax_0_, &IdentifyBox::smin_0_, &IdentifyBox::smax_0_, &IdentifyBox::vmin_0_, &IdentifyBox::vmax_0_,
                                    &IdentifyBox::hmin_1_, &IdentifyBox::hmax_1_, &IdentifyBox::smin_1_, &IdentifyBox::smax_1_, &IdentifyBox::vmin_1_, &IdentifyBox::vmax_1_,
                                    &IdentifyBox::open_, &IdentifyBox::close_, &IdentifyBox::erode_, &IdentifyBox::dilate_);
        }
    }

    while (camera_is_open) {
        if (DepthSolution::functionConfig_._mining_mode == MiningMode::GRIP_MODE || DepthSolution::functionConfig_._mining_mode == MiningMode::CATCH_MODE){
            IdentifyOre::OreIdentifyStream(import_src_color, import_src_depth, &sent_data_);
        }
        else if (DepthSolution::functionConfig_._mining_mode == MiningMode::EXCHANGE_MODE){
                IdentifyBox::BoxIdentifyStream(import_src_color, import_src_depth, &sent_data_);
        }

        if (mutex_serial_port_data.try_lock()) {
            *sent_serial_port_data = sent_data_;
            mutex_serial_port_data.unlock();
        }
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

    //兑换框识别
    IdentifyBox::hmin_0_ = 0;
    IdentifyBox::hmax_0_ = 109;
    IdentifyBox::smin_0_ = 0;
    IdentifyBox::smax_0_ = 255;
    IdentifyBox::vmin_0_ = 150;
    IdentifyBox::vmax_0_ = 255;

    IdentifyBox::hmin_1_ = 0;
    IdentifyBox::hmax_1_ = 181;
    IdentifyBox::smin_1_ = 77;
    IdentifyBox::smax_1_ = 255;
    IdentifyBox::vmin_1_ = 205;
    IdentifyBox::vmax_1_ = 255;

    IdentifyBox::open_   = 1;
    IdentifyBox::close_  = 1;
    IdentifyBox::erode_  = 1;
    IdentifyBox::dilate_ = 2;
}
