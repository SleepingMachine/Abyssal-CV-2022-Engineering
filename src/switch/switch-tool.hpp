//
// Created by sleepingmachine on 22-5-23.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_SWITCH_TOOL_HPP
#define ABYSSAL_CV_2022_ENGINEERING_SWITCH_TOOL_HPP



class SwitchTool{
public:
    SwitchTool();
    ~SwitchTool() {};

    static void CreatTrackbars(int *hmin_0, int *hmax_0, int *smin_0, int *smax_0, int *vmin_0, int *vmax_0,
                               int *hmin_1, int *hmax_1, int *smin_1, int *smax_1, int *vmin_1, int *vmax_1,
                               int *open,   int *close,  int *erode,  int *dilate){
        cv::namedWindow("矿石识别中的阈值调整",cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("hmin0", "矿石识别中的阈值调整",hmin_0, 255,NULL);
        cv::createTrackbar("hmax0", "矿石识别中的阈值调整",hmax_0, 255,NULL);
        cv::createTrackbar("smin0", "矿石识别中的阈值调整",smin_0, 255,NULL);
        cv::createTrackbar("smax0", "矿石识别中的阈值调整",smax_0, 255,NULL);
        cv::createTrackbar("vmin0", "矿石识别中的阈值调整",vmin_0, 255,NULL);
        cv::createTrackbar("vmax0", "矿石识别中的阈值调整",vmax_0, 255,NULL);

        cv::createTrackbar("hmin1", "矿石识别中的阈值调整",hmin_1, 255,NULL);
        cv::createTrackbar("hmax1", "矿石识别中的阈值调整",hmax_1, 255,NULL);
        cv::createTrackbar("smin1", "矿石识别中的阈值调整",smin_1, 255,NULL);
        cv::createTrackbar("smax1", "矿石识别中的阈值调整",smax_1, 255,NULL);
        cv::createTrackbar("vmin1", "矿石识别中的阈值调整",vmin_1, 255,NULL);
        cv::createTrackbar("vmax1", "矿石识别中的阈值调整",vmax_1, 255,NULL);

        cv::createTrackbar("open", "矿石识别中的阈值调整",    open, 10,NULL);
        cv::createTrackbar("close", "矿石识别中的阈值调整",  close, 30,NULL);
        cv::createTrackbar("erode", "矿石识别中的阈值调整",  erode, 10,NULL);
        cv::createTrackbar("dilate", "矿石识别中的阈值调整",dilate, 20,NULL);
    }
private:

};

#endif //ABYSSAL_CV_2022_ENGINEERING_SWITCH_TOOL_HPP
