#ifndef THREADCONTROL_H
#define THREADCONTROL_H

#include "opencv2/opencv.hpp"
using namespace cv;

struct OtherParam
{
    int8_t color = 1;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
    int8_t mode = 0;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int8_t cap_mode = 1;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
};


class threadControl
{
public:
    threadControl();
    void ImageProduce();      // 获取图像线程
    void ImageProcess();      // 图像处理线程
    void ImageWrite();

private:
    Mat image_;
    OtherParam other_param;
    bool end_thread_flag = false;
};

void limit_angle(float &a, float max);
#endif // THREADCONTROL_H
