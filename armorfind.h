#ifndef ARMORFIND_H
#define ARMORFIND_H

#include <opencv2/opencv.hpp>
#include "armor.h"
#include "serial.h"
#include "solveangle.h"
#include "predict.h"
using namespace cv;
using namespace std;

class armorFind
{
public:
    armorFind(){
        solve_angle_ = SolveAngle(CAMERA0_FILEPATH, SHOR_X, SHOR_Y, SHOR_Z, PTZ_TO_BARREL);
        predict_ = Predictor(30);
        zeyu_predict_ = ZeYuPredict(0.01f, 0.01f, 0.01f, 0.01f, 1.0f, 3.0f);
    }
    ~armorFind(){}
    void Init(Mat inputImg,Mat outputImg);
    armor Armor;
    Mat inputImg;
    Mat outputImg;
    int color = 0;
    LED_Stick LED_stick[2];
    vector<LED_Stick> LED_Stick_v;
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
public:
    void armorFindInit(void);
    void GetArmors(void);
    void ContourThread(int thres);
    void SetColor(int color);
    void SetImg(Mat inputImg, Mat outputImg);
    void PidControl(int pValue, int iValue, int dValue);
    int ArmorTask();
private:
    double Pointdis(const Point &p1, const Point &p2);
    Point PointBetween(const Point &p1, const Point &p2);
    double AngerDiff(const double angle1, const double angle2);
    double PointLineX(const Point &p1, const Point &p2);
    double AngerAve(const double angle1, const double angle2);
    double HeightAve(const double height1, const double height2);
public:
    vector<armor> final_armor_list;
private:
    void GetLED(void);
    void FindArmor(void);
    void SortArmor(void);
    void GetFinalArmor(void);
    bool getTypeResult(bool is_small);

public:
    float fx;
    float fy;
    float height_world = 60.0;
    float overlap_dist = 100000.0;
    float barrel_ptz_offset_y = -0; // mm   + ptz is up barrel
    float ptz_camera_x = 0;       // +left
    float ptz_camera_y = 52.5;       // + camera is  ptz
    float ptz_camera_z = -135;//-225;     // - camera is front ptz
    float scale = 0.99f;

public:
    Rect GetRoi(const Mat &img);
    bool makeRectSafe(cv::Rect & rect, cv::Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }
    bool DetectArmor(Mat &img, Rect roi_rect);
    void getAngle(float &yaw, float &pitch)
    {
        yaw = angle_x_;
        pitch = angle_y_;
    }

private:
    Rect last_target_;
    int lost_cnt_ = 0;
    int detect_cnt_ = 0;

public:
    Point2f offset_point;
    int short_offset_x_ = 100;
    int short_offset_y_ = 100;
    int color_th_ = 16;
    int gray_th_ = 60;

private:
    float dist_ = 3000;
    float distance_ = 0;
    float angle_x_ = 0;
    float angle_y_ = 0;
    bool is_small_;
    vector<Point2f> points_2d_;
    Rect roi_rect;

private:
    class SolveAngle solve_angle_;
    ZeYuPredict zeyu_predict_;
    Predictor predict_;
    Kalman1 kalman;

private:
    std::list<bool> history_;
    int filter_size_ = 5;


};

#endif // ARMORFIND_H
