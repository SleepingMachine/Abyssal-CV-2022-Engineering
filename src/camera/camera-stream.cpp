//
// Created by sleepingmachine on 2022/4/11.
//

#include "../include/camera/camera-stream.hpp"

extern std::mutex mutex_camera;
extern std::atomic_bool camera_start;

rs2::frameset CameraStream::frames_;
rs2::pipeline CameraStream::pipe_;
rs2::config CameraStream::cfg_;
//rs2::colorizer CameraStream::colorizer_;
rs2::pipeline_profile CameraStream::profile_;

CameraPara CameraStream::cameraPara_ = CameraParaFactory::getCameraPara();

int CameraStream::StreamRetrieve(cv::Mat *pFrame_color, cv::Mat *pFrame_depth) try {
    InitCamera();
    while (true){
        frames_ = pipe_.wait_for_frames();//等待所有配置的流生成框架

        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames_ = align_to_color.process(frames_);

        rs2::frame color_frame = frames_.get_color_frame();
        rs2::frame depth_frame = frames_.get_depth_frame();

        cv::Mat frame_color(Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::Mat frame_depth(Size(kCameraFrameWidth, kCameraFrameHeight), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        if (mutex_camera.try_lock()) {
            frame_color.copyTo(*pFrame_color);
            frame_depth.copyTo(*pFrame_depth);
            mutex_camera.unlock();
        }

        waitKey(1);

        //double loop_end_time = (double)cv::getTickCount();
        //double loop_total_time = (loop_end_time-loop_start_time)/(cv::getTickFrequency());
        //double fps = 1/loop_total_time;
        //std::cout<<"本次相机取图耗时["<<loop_total_time << "] 相机帧率[" << fps << "]" << std::endl;
    }
    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    exit(0);
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    exit(0);
    return EXIT_FAILURE;
}

void CameraStream::InitCamera(){
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() <= 0){
        exit(0);
        throw std::runtime_error("No device detected. Is it plugged in?");
    }

    rs2::device dev = list.front();

    cfg_.enable_stream(RS2_STREAM_COLOR, kCameraFrameWidth, kCameraFrameHeight, RS2_FORMAT_BGR8, kCameraFps);//向配置添加所需的流
    cfg_.enable_stream(RS2_STREAM_DEPTH, kCameraFrameWidth, kCameraFrameHeight, RS2_FORMAT_Z16, kCameraFps);

    profile_ = pipe_.start(cfg_);

    auto depth_stream=profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    CameraStream::cameraPara_.depth_scale2m = DepthTool::get_depth_scale(profile_.get_device());
    //cv::Mat color_hsv(480, 640, CV_8UC3);
}
