#include "camera/camera-stream.hpp"
#include "serial/serial-port.hpp"
#include "depth/depth-analysis.hpp"
#include "record/record-video.hpp"
#include "switch/switch-mode.hpp"
#include <thread>

//std::mutex mutex_color;
std::mutex mutex_camera;
std::mutex mutex_depth_analysis;
std::mutex mutex_serial_port_data;

static int64 sent_serial_port_data;
extern std::atomic_bool camera_is_open;
extern std::atomic_bool serial_port_start;

cv::Mat frame_depth                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color                (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_color_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
cv::Mat frame_depth_depth_analysis (Size(kCameraFrameWidth, kCameraFrameHeight), CV_8UC3);
//rs2::pipeline_profile profile;

int main(int argc, char** argv){
    camera_is_open = true;
    serial_port_start = true;

    std::thread camera_thread (CameraStream::StreamRetrieve, &frame_color, &frame_depth);
    std::thread depth_thread  (DepthSolution::DepthSolutionStream, &frame_color, &frame_depth, &frame_color_depth_analysis, &frame_depth_depth_analysis);
    std::thread switch_thread (SwitchControl::SwitchMode, &frame_color_depth_analysis, &frame_depth_depth_analysis, &sent_serial_port_data);
    std::thread serial_thread (SerialPort::SendData, &sent_serial_port_data);
    std::thread record_thread (RecordVideo::SaveRunningVideo, &frame_color);

    camera_thread.join();
    depth_thread .join();
    switch_thread.join();
    record_thread.join();
    serial_thread.join();
    return 0;
}
/*
int main(int argc, char** argv) try
{
    // judge whether devices is exist or not 
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() <= 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    //
    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流
    //cv::Mat color_hsv(480, 640, CV_8UC3);

    while(true)
    {
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架

        // Align to depth 
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);
        //frames = align_to_depth.process(frames);

        // Align to color
        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames = align_to_color.process(frames);

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
        /*
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        //rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        //rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);


        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        //Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
        //Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
        Mat pic_depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        // Display in a GUI
        //namedWindow("Display Image", WINDOW_AUTOSIZE );
        //cvtColor(color, color_hsv, COLOR_BGR2HSV);
        imshow("Display Color", color);
        waitKey(1);
        imshow("Display Depth", pic_depth*20);
        waitKey(1);
        /*
        imshow("Display pic_left", pic_left);
        waitKey(1);
        imshow("Display pic_right",pic_right);
        waitKey(1);
        */
 /*   }
    return 0;
}

// error
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
*/

