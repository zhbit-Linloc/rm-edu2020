#ifndef CAMARA_H
#define CAMARA_H
#include <opencv2/opencv.hpp>

using namespace cv;
class camara
{
public:
    camara();
    VideoWriter videowriter;
    Mat img;
    struct tm *timeinfo;
    time_t start;
    time_t end;
    void updateImg(Mat img);
    void videoWriteOpen(void);
    void videoSave();
    void updateTime();
    ~camara();
};


#endif // CAMARA_H
