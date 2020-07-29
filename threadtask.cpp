#include "threadtask.h"
#include "threadpool.h"
#include <thread>
#include "base.h"
#include "rmvideocapture.h"
#include "camara.h"
#include "serial.h"
#include "armorfind.h"
#include "opencv2/opencv.hpp"
using namespace std;
void limit_angle(float &a, float max);
void GetImageThread(RMVideoCapture &cap,Mat &out){
    cap >> out;
}


void threadTask::ImageProduce(){

    #if(SHORT_CAMERA_ENABLE)
        RMVideoCapture short_camera(CAMERA1_PATH,3);
            short_camera.setVideoFormat(VIDEO_WIDTH,VIDEO_HEIGHT,1);
            short_camera.info();
            short_camera.setExposureTime(0, 100);
            short_camera.startStream();

    #endif

    while (1){
    #if(SHORT_CAMERA_ENABLE)
        GetImageThread(short_camera, image_);
    #endif
    }
}

void threadTask::ImageProcess(){

    float yaw = 0.0, pitch = 0.0;
    _serial serial("/dev/ttyUSB0");
    Mat image;

#ifdef VIDEOSAVE
    camara Cam;
    Cam.updateImg(image_);
    Cam.videoWriteOpen();
    Cam.updateTime();
    Cam.start = Cam.end = mktime(Cam.timeinfo);
#endif
    armorFind ArmorFind;
#ifdef ARMOR_TRACK_BAR
    namedWindow("ArmorParam");
    createTrackbar("armor_gray_th", "ArmorParam", &ArmorFind.gray_th_, 255);
    createTrackbar("armor_color_th", "ArmorParam", &ArmorFind.color_th_, 255);
    createTrackbar("short_offset_x","ArmorParam",&ArmorFind.short_offset_x_,200);
    createTrackbar("short_offset_y","ArmorParam",&ArmorFind.short_offset_y_,200);

#endif

    int commond = 0;
        while (1){
#ifdef GETIME
                double t1 = getTickCount();
#endif

#ifdef VIDEOSAVE
                if ((Cam.end - Cam.start) < 20){
                    Cam.updateTime();
                    Cam.end = mktime(Cam.timeinfo);
                    Cam.updateImg(image_);
                    Cam.videoSave();
                    waitKey(35);
                }
#endif

                image_.copyTo(image);
                ArmorFind.Init(image_, image);

                commond = ArmorFind.ArmorTask();
                ArmorFind.getAngle(yaw,pitch);
//                cout << "pitch: " << pitch << endl;
//                ArmorFind.GetArmors();
//                ArmorFind.SolveAngle();
//                cout << "commond: " << commond << endl;

                limit_angle(yaw, 90);
//                serial.data.get_xy_data((int16_t)(yaw*100), (int16_t)(pitch*100), commond);
//                serial.send_data();
#ifdef SERIALREAD
    serial.read_data();
#endif

#ifdef WAITKEY
#ifdef IMSHOW
    imshow("okey", image);
#endif
    int k = waitKey(1);
    if (k == 'q');
#endif
//  ArmorFind.armorFindInit();

#ifdef GETIME
    double t2 = getTickCount();
    double t = (t2-t1)*1000/getTickFrequency();
    cout << t << endl;
#endif
        }
        close(serial.fd);

}

void threadTask::ImageWrite(){
    camara Cam;

    Cam.updateImg(image_);
    Cam.updateTime();
    Cam.videoWriteOpen();

    Cam.start = Cam.end = mktime(Cam.timeinfo);
    while ((Cam.end - Cam.start) < VIDEO_TIME){
        Cam.updateTime();
        Cam.end = mktime(Cam.timeinfo);
        Cam.updateImg(image_);
        Cam.videoSave();
        waitKey(35);
    }
}



void limit_angle(float &a, float max)
{
    if(a > max)
        a = max;
    else if(a < -max)
        a = -max;
}

