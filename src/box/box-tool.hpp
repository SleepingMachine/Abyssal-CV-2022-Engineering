//
// Created by sleepingmachine on 22-5-24.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_BOX_TOOL_HPP
#define ABYSSAL_CV_2022_ENGINEERING_BOX_TOOL_HPP
#include <opencv2/opencv.hpp>

class BoxTool{
public:
    static void CreatTrackbars(int *hmin_0, int *hmax_0, int *smin_0, int *smax_0, int *vmin_0, int *vmax_0,
                               int *hmin_1, int *hmax_1, int *smin_1, int *smax_1, int *vmin_1, int *vmax_1,
                               int *open,   int *close,  int *erode,  int *dilate){
        cv::namedWindow("兑换框识别中的阈值调整",cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("hmin0", "兑换框识别中的阈值调整",hmin_0, 255,NULL);
        cv::createTrackbar("hmax0", "兑换框识别中的阈值调整",hmax_0, 255,NULL);
        cv::createTrackbar("smin0", "兑换框识别中的阈值调整",smin_0, 255,NULL);
        cv::createTrackbar("smax0", "兑换框识别中的阈值调整",smax_0, 255,NULL);
        cv::createTrackbar("vmin0", "兑换框识别中的阈值调整",vmin_0, 255,NULL);
        cv::createTrackbar("vmax0", "兑换框识别中的阈值调整",vmax_0, 255,NULL);

        cv::createTrackbar("hmin1", "兑换框识别中的阈值调整",hmin_1, 255,NULL);
        cv::createTrackbar("hmax1", "兑换框识别中的阈值调整",hmax_1, 255,NULL);
        cv::createTrackbar("smin1", "兑换框识别中的阈值调整",smin_1, 255,NULL);
        cv::createTrackbar("smax1", "兑换框识别中的阈值调整",smax_1, 255,NULL);
        cv::createTrackbar("vmin1", "兑换框识别中的阈值调整",vmin_1, 255,NULL);
        cv::createTrackbar("vmax1", "兑换框识别中的阈值调整",vmax_1, 255,NULL);

        cv::createTrackbar("open",  "兑换框识别中的阈值调整",    open, 10,NULL);
        cv::createTrackbar("close", "兑换框识别中的阈值调整",  close, 30,NULL);
        cv::createTrackbar("erode", "兑换框识别中的阈值调整",  erode, 10,NULL);
        cv::createTrackbar("dilate","兑换框识别中的阈值调整",dilate, 10,NULL);
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

    static inline float getTwoPointDistance(cv::Point2f point_1, cv::Point2f point_2) {
        return sqrtf(pow((point_1.x - point_2.x), 2) + pow((point_1.y - point_2.y), 2));
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

    static inline cv::Point2f getTwoPointCenterPoint(cv::Point2f point1, cv::Point2f point2){
        return cv::Point2f((point1.x + point2.x)/2, (point1.y + point2.y)/2);
    }
private:


};
#endif //ABYSSAL_CV_2022_ENGINEERING_BOX_TOOL_HPP
