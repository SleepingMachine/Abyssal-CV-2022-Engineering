//
// Created by sleepingmachine on 22-7-23.
//

#include "../include/record/record-data.hpp"

extern std::atomic_bool camera_start;

extern std::mutex mutex_camera;

cv::Mat RecordData::src_  (480, 640, CV_8UC3);
cv::Mat RecordData::temp_ (480, 640, CV_8UC3);

std::string RecordData::output_video_path_;
cv::VideoWriter RecordData::output_video_;

RecordData::~RecordData() {}

void RecordData::SaveVideo(cv::Mat *pFrame) {
    if (mutex_camera.try_lock()) {
        temp_ = *pFrame;
        mutex_camera.unlock();
    }
    temp_.copyTo(src_);
    if (src_.empty()) {
        std::cout << "Get Save Video Frame Fail" << std::endl;
        return;
    }

    cv::Mat output;
    //cv::Mat roi;
    //roi = src(cv::Rect(cv::Point(160,0),cv::Point(800,480)));
    //output = roi.clone();
    cv::resize(src_,output,cv::Size(640,480));
    output_video_ << output;
    //cv::imshow("Output", output);
    cv::waitKey(1);
}

int RecordData::SaveDataStream(cv::Mat *pFrame) {
    while(!camera_start){
        cv::waitKey(10);
    }
    time_t timep;
    char name[256] = {0};

    time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep

    strftime( name, sizeof(name), "../asset/auto-save-video/%Y.%m.%d %H-%M-%S.avi",localtime(&timep) ); //最后一个参数是用localtime将秒数转化为struct tm结构体

    output_video_path_ = name;
    if (SwitchControl::functionConfig_._enable_save_video){
        output_video_.open(output_video_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, cv::Size(640, 480), true);
        while(true){
            std::cout << 1;
            SaveVideo(pFrame);
            SaveLog();
        }
    }
    else{
        SaveLog();
    }
    return 0;
}

void RecordData::SaveLog() {

}


