//
// Created by sleepingmachine on 22-5-21.
//

#include "switch-mode.hpp"

extern std::mutex mutex_depth_analysis;
extern std::atomic_bool camera_is_open;

void SwitchControl::SwitchMode(cv::Mat *import_src_color, cv::Mat *import_src_depth, int *sent_serial_port_data) {
    while(camera_is_open){
        if (DepthSolution::functionConfig_._mining_mode == MiningMode::GRIP_MODE || DepthSolution::functionConfig_._mining_mode == MiningMode::CATCH_MODE){
            IdentifyOre::OreIdentifyStream(import_src_color, import_src_depth);
        }
        else if (DepthSolution::functionConfig_._mining_mode == MiningMode::EXCHANGE_MODE){
            std::cout << "这个模式还没写 别急" << std::endl;
        }
    }
}
