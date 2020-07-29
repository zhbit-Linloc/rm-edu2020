//#include "threadcontrol.h"
//#include <thread>
//#include "base.h"
//#include "rmvideocapture.h"
//#include "camara.h"
//#include "serial.h"
//#include "armorfind.h"
//using namespace std;
//int ContoursV = 66;
//int colorV = 1;

//static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑
//static volatile unsigned int gimbal_data_index;     // gimbal data 生成序号，用于线程之间逻辑
//static volatile unsigned int consumption_index; // 图像消耗序号

////int pV = KP_DEFAULT * 1000;
////int iV = KI_DEFAULT * 1000;
////int dV = KD_DEFAULT * 1000;
//int pV = 0;
//int iV = 0;
//int dV = 0;
//int pidMax = 1000;

//threadControl::threadControl()
//{
//    cout << "THREAD TASK ON !!!" << endl;
//}


//void GetImageThread(RMVideoCapture &cap,Mat &out){
//    cap >> out;
//}

//void threadControl::ImageProduce(){

//    #if(SHORT_CAMERA_ENABLE)
//        RMVideoCapture short_camera(CAMERA1_PATH,3);
//            short_camera.setVideoFormat(VIDEO_WIDTH,VIDEO_HEIGHT,1);
//            short_camera.info();
//            short_camera.setExposureTime(0, 100);
//            short_camera.startStream();

//    #endif

//    while (1){
//        while(produce_index - consumption_index >= BUFFER_SIZE){
//            END_THREAD;
//        }

//    #if(SHORT_CAMERA_ENABLE)
//        GetImageThread(short_camera, image_);
//    #endif

//        ++produce_index;
//        END_THREAD;
//    }
//}

//void threadControl::ImageProcess(){

//    float yaw = 0.0, pitch = 0.0;
//    _serial serial("/dev/ttyUSB0");
//    Mat image;
//    namedWindow("Trackbar", WINDOW_AUTOSIZE);

//    createTrackbar("ContoursV", "Trackbar", &ContoursV, 255);
//    createTrackbar("ModeV", "Trackbar", &colorV, 1);
//    createTrackbar("pValue", "Trackbar", &pV, pidMax);
//    createTrackbar("iValue", "Trackbar", &iV, pidMax);
//    createTrackbar("dValue", "Trackbar", &dV, pidMax);


////    while (produce_index - consumption_index <= 0){
////        END_THREAD;
////    }

//#ifdef VIDEOSAVE
//    camara Cam;
//    Cam.updateImg(image_);
//    Cam.videoWriteOpen();
//    Cam.updateTime();
//    Cam.start = Cam.end = mktime(Cam.timeinfo);
//#endif
//    armorFind ArmorFind;
//#ifdef ARMOR_TRACK_BAR
//    namedWindow("ArmorParam");
//    createTrackbar("armor_gray_th", "ArmorParam", &ArmorFind.gray_th_, 255);
//    createTrackbar("armor_color_th", "ArmorParam", &ArmorFind.color_th_, 255);
//    createTrackbar("short_offset_x","ArmorParam",&ArmorFind.short_offset_x_,200);
//    createTrackbar("short_offset_y","ArmorParam",&ArmorFind.short_offset_y_,200);

//#endif

//    int commond = 0;
//        while (1){
//            while (produce_index - consumption_index <= 0){
//                END_THREAD;
//            }
//#ifdef GETIME
//                double t1 = getTickCount();
//#endif

//#ifdef VIDEOSAVE
//                if ((Cam.end - Cam.start) < 20){
//                    Cam.updateTime();
//                    Cam.end = mktime(Cam.timeinfo);
//                    Cam.updateImg(image_);
//                    Cam.videoSave();
//                    waitKey(35);
//                }
//#endif

//                image_.copyTo(image);
//                ArmorFind.Init(image_, image);

//                commond = ArmorFind.ArmorTask();
//                ArmorFind.getAngle(yaw,pitch);
////                cout << "pitch: " << pitch << endl;
////                ArmorFind.GetArmors();
////                ArmorFind.SolveAngle();
////                cout << "commond: " << commond << endl;

//                limit_angle(yaw, 90);
////                serial.data.get_xy_data((int16_t)(yaw*100), (int16_t)(pitch*100), commond);
////                serial.send_data();
//                ++consumption_index;
//#ifdef SERIALREAD
//    serial.read_data();
//#endif

//#ifdef WAITKEY
//#ifdef IMSHOW
//    imshow("okey", image);
//#endif
//    int k = waitKey(1);
//    if (k == 'q')
//      end_thread_flag = true;
//#endif
////  ArmorFind.armorFindInit();

//#ifdef GETIME
//    double t2 = getTickCount();
//    double t = (t2-t1)*1000/getTickFrequency();
//    cout << t << endl;
//#endif
//        END_THREAD;
//        }
//        close(serial.fd);

//}

//void threadControl::ImageWrite(){
//    camara Cam;
//    while (produce_index - consumption_index <= 0){
//        END_THREAD;
//    }
//    Cam.updateImg(image_);
//    Cam.updateTime();
//    Cam.videoWriteOpen();

//    Cam.start = Cam.end = mktime(Cam.timeinfo);
//    while ((Cam.end - Cam.start) < VIDEO_TIME){
//        Cam.updateTime();
//        Cam.end = mktime(Cam.timeinfo);
//        Cam.updateImg(image_);
//        Cam.videoSave();
//        waitKey(35);
//    }
//}



//void limit_angle(float &a, float max)
//{
//    if(a > max)
//        a = max;
//    else if(a < -max)
//        a = -max;
//}
