//
// Created by sleepingmachine on 2022/4/11.
//

#include "camera-stream.hpp"

rs2::frameset CameraStream::frames_;
rs2::pipeline CameraStream::pipe_;
rs2::config CameraStream::cfg_;
rs2::colorizer CameraStream::colorizer_;
rs2::pipeline_profile CameraStream::profile_;

std::atomic_bool camera_is_open;

extern std::mutex mutex_camera;

void CameraStream::InitCamera(){
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() <= 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    //
    //rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    //rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
    //rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg_.enable_stream(RS2_STREAM_COLOR, kCameraFrameWidth, kCameraFrameHeight, RS2_FORMAT_BGR8, kCameraFps);//向配置添加所需的流
    cfg_.enable_stream(RS2_STREAM_DEPTH, kCameraFrameWidth, kCameraFrameHeight, RS2_FORMAT_Z16, kCameraFps);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // get depth scale
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream
    profile_ = pipe_.start(cfg_);

    auto depth_stream=profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    //cv::Mat color_hsv(480, 640, CV_8UC3);
}

int CameraStream::StreamRetrieve(cv::Mat *pFrame_color, cv::Mat *pFrame_depth) try {
    if (!DepthSolution::functionConfig_._enable_local_video_stream){
        InitCamera();
        while (true){
            frames_ = pipe_.wait_for_frames();//等待所有配置的流生成框架

            // Align to depth
            //rs2::align align_to_depth(RS2_STREAM_DEPTH);
            //frames_ = align_to_depth.process(frames_);

            // Align to color
            rs2::align align_to_color(RS2_STREAM_COLOR);
            frames_ = align_to_color.process(frames_);

            // Get imu data
            /*
            if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
            {
                rs2_vector accel_sample = accel_frame.get_motion_data();
                std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
            }
            if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
            {
                rs2_vector gyro_sample = gyro_frame.get_motion_data();
                std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            }
            */
            //Get each frame
            rs2::frame color_frame = frames_.get_color_frame();
            rs2::frame depth_frame = frames_.get_depth_frame();
            //rs2::frame depth_frame_4_show = frames_.get_depth_frame().apply_filter(colorizer_);
            //rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
            //rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);


            // Creating OpenCV Matrix from a color image
            Mat frame_color(Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            //Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
            //Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
            Mat frame_depth(Size(kCameraFrameWidth, kCameraFrameHeight), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
            //Mat frame_depth(Size(kCameraFrameWidth, kCameraFrameHeight),CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
            //float depth_scale = DepthTool::get_depth_scale(profile_.get_device());
            //std:: cout << depth_scale << std::endl;

            if (mutex_camera.try_lock()) {
                frame_color.copyTo(*pFrame_color);
                frame_depth.copyTo(*pFrame_depth);
                mutex_camera.unlock();
            }
            // Display in a GUI
            //namedWindow("Display Image", WINDOW_AUTOSIZE );
            //cvtColor(color, color_hsv, COLOR_BGR2HSV);
            //imshow("Display Color", frame_color);
            //imshow("Display Depth", frame_depth*10);
            //imshow("Display Depth", pic_depth*20);
            waitKey(1);
            /*
            imshow("Display pic_left", pic_left);
            waitKey(1);
            imshow("Display pic_right",pic_right);
            waitKey(1);
            */
        }
    }
    else{
        cv::VideoCapture capture;
        cv::Mat frame_color_local;
        cv::Mat frame_depth_local;

        capture.open(DepthSolution::functionConfig_.local_video_path);
        if (!capture.isOpened()) {
            printf("could not read this video file...\n");
            exit(0);
        }

        //capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
        //capture.set(cv::CAP_PROP_EXPOSURE, 500);
        //capture.set(CAP_PROP_EXPOSURE, -12);

        while (true) {
            capture >> frame_color_local;  //读取当前帧
            //若视频播放完成，退出循环
            if (frame_color_local.empty()) {
                //std::cout << 1 << std::endl;
                exit(0);
            }

            if (frame_color_local.cols != 640 || frame_color_local.rows != 480) {
                cv::resize(frame_color_local, frame_color_local, cv::Size(640, 480));
            }
            /*
            if (ControlSwitch::functionConfig._imageOrientationCorrection){
            cv::flip(frame, frame, 0);
            }
            */
            frame_color_local.copyTo(frame_depth_local);
            frame_depth_local.setTo(4);
            if (mutex_camera.try_lock()) {
                frame_color_local.copyTo(*pFrame_color);
                frame_depth_local.copyTo(*pFrame_depth);
                mutex_camera.unlock();
            }
            //imshow("读取视频", frame_color_local);  //显示当前帧
            waitKey(5);  //延时30ms
        }
    }

    return 0;
}

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
