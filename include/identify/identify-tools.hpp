//
// Created by sleepingmachine on 22-7-22.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_TOOLS_HPP
#define ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_TOOLS_HPP
#include <opencv2/opencv.hpp>

class IdentifyTool{
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

        cv::createTrackbar("open",  "矿石识别中的阈值调整",    open, 10,NULL);
        cv::createTrackbar("close", "矿石识别中的阈值调整",  close, 30,NULL);
        cv::createTrackbar("erode", "矿石识别中的阈值调整",  erode, 10,NULL);
        cv::createTrackbar("dilate","矿石识别中的阈值调整",dilate, 20,NULL);
    }
    static void CreatTrackbars(int *open,   int *close,  int *erode,  int *dilate){

        cv::namedWindow("阈值调整",cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("open",  "阈值调整",open,  10,NULL);
        cv::createTrackbar("close", "阈值调整",close, 30,NULL);
        cv::createTrackbar("erode", "阈值调整",erode, 10,NULL);
        cv::createTrackbar("dilate","阈值调整",dilate,20,NULL);
    }

    static inline cv::Point2f getTwoPointCenterPoint(cv::Point2f point1, cv::Point2f point2){
        return cv::Point2f((point1.x + point2.x)/2, (point1.y + point2.y)/2);
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

    static inline cv::Point2f getCrossPoint(cv::Point2f line1, cv::Point2f line2, cv::Point2f line3, cv::Point2f line4) //交点
    {
        double x_member, x_denominator, y_member, y_denominator;
        cv::Point2f cross_point;
        x_denominator = line4.x*line2.y - line4.x*line1.y - line3.x*line2.y + line3.x*line1.y
                        - line2.x*line4.y + line2.x*line3.y + line1.x*line4.y - line1.x*line3.y;

        x_member = line3.y*line4.x*line2.x - line4.y*line3.x*line2.x - line3.y*line4.x*line1.x + line4.y*line3.x*line1.x
                   - line1.y*line2.x*line4.x + line2.y*line1.x*line4.x + line1.y*line2.x*line3.x - line2.y*line1.x*line3.x;

        if (x_denominator == 0)
            cross_point.x = 0;
        else
            cross_point.x = x_member / x_denominator;

        y_denominator = line4.y*line2.x - line4.y*line1.x - line3.y*line2.x + line1.x*line3.y
                        - line2.y*line4.x + line2.y*line3.x + line1.y*line4.x - line1.y*line3.x;

        y_member = -line3.y*line4.x*line2.y + line4.y*line3.x*line2.y + line3.y*line4.x*line1.y - line4.y*line3.x*line1.y
                   + line1.y*line2.x*line4.y - line1.y*line2.x*line3.y - line2.y*line1.x*line4.y + line2.y*line1.x*line3.y;

        if (y_denominator == 0)
            cross_point.y = 0;
        else
            cross_point.y = y_member / y_denominator;

        return cross_point;  //平行返回(0,0)
    }

    static inline float getTwoPointDistance(cv::Point2f point_1, cv::Point2f point_2) {
        return sqrtf(pow((point_1.x - point_2.x), 2) + pow((point_1.y - point_2.y), 2));
    }

    static inline cv::Mat structuringElement3() {
        return instance().StructuringElement3;
    }

private:
    static IdentifyTool &instance() {
        static IdentifyTool identifyTool;
        return identifyTool;
    }
    const cv::Mat StructuringElement3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    const cv::Mat StructuringElement5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    const cv::Mat StructuringElement7 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
};
#endif //ABYSSAL_CV_2022_ENGINEERING_IDENTIFY_TOOLS_HPP
