#ifndef BASE_H
#define BASE_H

// ------ system ------ //
#define SHORT_CAMERA_ENABLE 1
#define LONG_CAMERA_ENABLE  0
#define WAITKEY
#define IMSHOW
#define SHOW_BINARY_IMAGE
#define SHOW_DRAW_RECT
#define SHOW_DRAW_SPOT
//#define SERIALREAD
//#define GETIME
#define PREDICT
//#define VIDEOSAVE
#define ARMOR_TRACK_BAR



#define FAST_DISTANCE

#define BULLET
#define USE_FIT
//#define show_rect_bound
//#define show_rect_angle
// ------ settings ------ //
// for image
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1
#define VIDEO_TIME 30


#define SERIAL_PATH "/dev/ttyUSB0"
#define SERIAL_BAUD B115200     // B115200 B921600
#define GIMBAL_PATH "/dev/ttyUSB1"
#define GIMBAL_BAUD B921600
#define CAMERA0_PATH "/dev/video1"
#define CAMERA1_PATH "/dev/video1"
#define CAMERA0_FILEPATH "../rm-autoShoot/camera_param/camera4mm_5.xml"

// 摄像头坐标系到云台坐标系
#define SHOR_X 57.0f
#define SHOR_Y 47.5f
#define SHOR_Z -111.37f
#define LONG_X 0.0f
#define LONG_Y 40.7f
#define LONG_Z -123.0f
#define PTZ_TO_BARREL 0.0f   // 补兵激光在２３ｍｍ下方
// ------ common ------ //
#define END_THREAD if(end_thread_flag) return;

#endif // BASE_H
