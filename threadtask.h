#ifndef THREADTASK_H
#define THREADTASK_H
#include "opencv2/opencv.hpp"

class threadTask{
public:
    void ImageProduce();
    void ImageProcess();
    void ImageWrite();
private:
    cv::Mat image_;
};


#endif // THREADTASK_H
