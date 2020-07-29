#ifndef ARMOR_H
#define ARMOR_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LED_Stick{
public:
    LED_Stick():matched(false){}

    LED_Stick(const cv::RotatedRect& R) {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    RotatedRect rect;
    bool matched;
    int match_index;
    float match_factor;

};

class armor
{
public:
    armor();
    armor(const LED_Stick& L1, const LED_Stick& L2);
    void armorInit(const LED_Stick& L1, const LED_Stick& L2);
    void armorPredict(float x, float y);
    void draw_rect( cv::Mat& img) const;
    void draw_spot(cv::Mat &img) const;

    void draw_rect2( Mat& img, Point2f roi_offset_poin) const;    // 画出装甲板
    void draw_spot2(Mat &img, Point2f roi_offset_point) const;

    int get_average_intensity(const cv::Mat& img) ;
    void max_match(vector<LED_Stick>& LED, size_t i, size_t j);
    bool is_suitable_size(void) const;

    float error_angle;
    LED_Stick LED_stick[2];
    RotatedRect rect;
    Rect2i rect2;
    int average_intensity;
    bool found = false;
    Point2i center;
};

#endif // ARMOR_H
