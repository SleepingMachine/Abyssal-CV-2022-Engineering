//
// Created by sleepingmachine on 22-4-15.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_ORE_TOOL_HPP
#define ABYSSAL_CV_2022_ENGINEERING_ORE_TOOL_HPP
#include <opencv2/opencv.hpp>

class OreTool{
public:
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

    static inline void drawRotatedRect(cv::Mat mask,const cv::RotatedRect &rotatedrect,const cv::Scalar &color,int thickness, int lineType)
    {
        // 提取旋转矩形的四个角点
        cv::Point2f ps[4];
        rotatedrect.points(ps);

        // 构建轮廓线
        std::vector<std::vector<cv::Point>> tmpContours;    // 创建一个InputArrayOfArrays 类型的点集
        std::vector<cv::Point> contours;
        for (int i = 0; i != 4; ++i) {
            contours.emplace_back(cv::Point2i(ps[i]));
        }
        tmpContours.insert(tmpContours.end(), contours);

        // 绘制轮廓，即旋转矩形
        drawContours(mask, tmpContours, 0, color,thickness, lineType);  // 填充mask
    }

    static inline float getRectLengthWidthRatio(cv::RotatedRect &rect) {
        float longSide = rect.size.height > rect.size.width ? rect.size.height : rect.size.width;   //获取识别出矩形长
        float shortSide = rect.size.height < rect.size.width ? rect.size.height : rect.size.width;  //获取识别出矩形宽
        return longSide / shortSide;
    }

    static inline float trackLineFit(std::vector<cv::Point2f> points){
        cv::Vec4f lines;
        //参数设置
        double param=0;//距离模型中的数值参数C
        double reps=0.01;//坐标原点与直线之间的距离
        double aeps=0.01;//角度精度
        cv::fitLine(points,lines,cv::DIST_L1,0,0.01,0.01);
        double k=lines[1]/lines[0];//直线斜率
        //cout<<"直线斜率: "<<k<<endl;
        //cout<<"直线上一点坐标x: "<<lines[2]<<",y: "<<lines[3]<<endl;
        //cout<<"直线解析式: y="<<k<<"(x-"<<lines[2]<<")+"<<lines[3]<<endl;
        return k;
    }

    static inline float getTwoPointDistance(cv::Point2f point_1, cv::Point2f point_2) {
        return sqrtf(pow((point_1.x - point_2.x), 2) + pow((point_1.y - point_2.y), 2));
    }

private:


};
#endif //ABYSSAL_CV_2022_ENGINEERING_ORE_TOOL_HPP
