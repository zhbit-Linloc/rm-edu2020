#include "camara.h"
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

camara::camara(){}

void camara::updateImg(Mat img){
    img.copyTo(this->img);
}

void camara::videoWriteOpen(){
    char tmpbuf[128];
    strftime(tmpbuf, 128, "%c", timeinfo);
    sprintf(tmpbuf, "%d.%d.%d  %02d:%02d.avi", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
    videowriter.open(tmpbuf, CV_FOURCC('X', 'V', 'I', 'D'), 25, cv::Size(img.cols, img.rows));
    if (!videowriter.isOpened()) {
        cout << "videowriter opened failure!" << endl;
    }
}

void camara::videoSave(){
    videowriter << img;
}

void camara::updateTime(){
    time_t test;
    time(&test);
    timeinfo=localtime(&test);
}

camara::~camara(){
    videowriter.release();
}
