// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;//创建一个以非默认配置的配置用来配置管道

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);//向配置添加所需的流
    //彩色图片：分辨率640*480 8位bgr格式 每秒30帧
    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);//指示管道使用所请求的配置启动流

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;//相机预热-删除几个第一帧让自动曝光稳定。
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架
    }

    //Get each frame
    rs2::frame color_frame = frames.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    //创建一个mat类，大小，格式，数据地址（其实。。。。不知道到底是啥，后期再补充），步长
    // Display in a GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", color);

    waitKey(0);

    return 0;
}