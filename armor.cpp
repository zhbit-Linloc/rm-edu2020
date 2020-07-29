#include "armor.h"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

armor::armor(){
    ;
}

armor::armor(const LED_Stick& L1, const LED_Stick& L2) {
    LED_stick[0] = L1;
    LED_stick[1] = L2;
    error_angle = fabs(L1.rect.angle - L2.rect.angle);

    rect.size.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect.size.height = static_cast<int>(L1.rect.size.height + L2.rect.size.height) / 2;
    rect.center.x = static_cast<int>(L1.rect.center.x + L2.rect.center.x)/2;
    rect.center.y = static_cast<int>(L1.rect.center.y + L2.rect.center.y)/2;
    rect.angle = (L1.rect.angle + L2.rect.angle) / 2;

    rect2.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect2.height = static_cast<int>((L1.rect.size.height + L1.rect.size.height)/2);
    center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x)/2);
    center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y)/2);
    rect2.x = center.x - rect2.width/3;
    rect2.y = center.y - rect2.height/3;
    rect2.width*= 2.0/3;
    rect2.height*= 2.0/3;
//    rect.size.width *= 2.0 / 3;
//    rect.size.height *= 2.0 / 3;

    found = false;
}

void armor::armorInit(const LED_Stick& L1, const LED_Stick& L2) {
    LED_stick[0] = L1;
    LED_stick[1] = L2;
    error_angle = fabs(L1.rect.angle - L2.rect.angle);

    rect.size.width = abs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
    rect.size.height = static_cast<int>(L1.rect.size.height + L2.rect.size.height) / 2;
    rect.center.x = static_cast<int>(L1.rect.center.x + L2.rect.center.x)/2;
    rect.center.y = static_cast<int>(L1.rect.center.y + L2.rect.center.y)/2;
    rect.angle = (L1.rect.angle + L2.rect.angle) / 2;

//    rect.size.width *= 2.0 / 3;
//    rect.size.height *= 2.0 / 3;

    found = false;
}

void armor::draw_rect( cv::Mat& img) const
{
//        cout << " rect " << rect.width << "  " << rect.height << "　"<< rect.x << endl;
   cv::Point2f* vertices = new cv::Point2f[4];
   rect.points(vertices);
   for (size_t i = 0; i < 4; i++) {
       line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 255, 255), 4);
   }
}

void armor::draw_spot(cv::Mat &img) const
{
    circle(img, rect.center, int(rect.size.height/3), cv::Scalar(0,0,255), 4);

}


void armor::draw_rect2( Mat& img, Point2f roi_offset_point) const
{
    rectangle(img, rect2 + Point_<int>(roi_offset_point), Scalar(255,255,255), 1);
}

void armor::draw_spot2(Mat &img, Point2f roi_offset_point) const
{
    circle(img, center + Point_<int>(roi_offset_point), int(rect2.height/4), Scalar(0,0,255), -1);
}



int armor::get_average_intensity(const Mat& img) {
    int rectX = rect.center.x - rect.size.width / 3;
    int rectY = rect.center.y - rect.size.height / 3;
    if(rect.size.width < 1 || rect.size.height < 1 || rectX < 1 || rectY < 1
            || rect.size.width + rectX > img.cols || rect.size.height + rectY > img.rows)
        return 255;
    Mat roi = img(Range(rectY, rectY + rect.size.height), Range(rectX, rectX + rect.size.width) );
    //        imshow("roi ", roi);
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}


void armor::max_match(vector<LED_Stick>& LED,size_t i,size_t j){
    RotatedRect R, L;
    if(LED_stick[0].rect.center.x > LED_stick[1].rect.center.x)
    {
        R = LED_stick[0].rect;
        L = LED_stick[1].rect;
    }else
    {
        R = LED_stick[1].rect;
        L = LED_stick[0].rect;
    }

    float angle_8 = L.angle - R.angle;
    //    cout << L.angle << " "<< R.angle << endl;
    if(angle_8 < 1e-3f)
        angle_8 = 0.0f;
    float f = error_angle + angle_8;
    if(!LED.at(i).matched && !LED.at(j).matched )
    {

        LED.at(i).matched = true;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_index = i;
        LED.at(i).match_factor = f;
        LED.at(j).match_factor = f;
    }
    if(LED.at(i).matched && !LED.at(j).matched)
    {
        if(f < LED.at(i).match_factor)
        {
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }
    }
    if(LED.at(j).matched && !LED.at(i).matched)
    {
        if(f < LED.at(j).match_factor )
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
        }
    }
    if(LED.at(j).matched && LED.at(i).matched
            && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
    {
        LED.at(LED.at(j).match_index).matched = false;
        LED.at(LED.at(i).match_index).matched = false;
        LED.at(i).matched = true;
        LED.at(i).match_factor = f;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_factor = f;
        LED.at(j).match_index = i;
    }
}


bool armor::is_suitable_size(void) const
{
    // 两个灯条体型相似
    if(LED_stick[0].rect.size.height*0.7f < LED_stick[1].rect.size.height
            && LED_stick[0].rect.size.height*1.3f > LED_stick[1].rect.size.height)
    {
        float armor_width = fabs(LED_stick[0].rect.center.x - LED_stick[1].rect.center.x);
        if(armor_width > LED_stick[0].rect.size.width
                && armor_width > LED_stick[1].rect.size.width)
        {
            float h_max = (LED_stick[0].rect.size.height + LED_stick[1].rect.size.height)/2.0f;
            // 两个灯条高度差不大
            if(fabs(LED_stick[0].rect.center.y - LED_stick[1].rect.center.y) < 0.8f* h_max )
            {
                // 长宽比判断
                if(h_max*4.0f > rect.size.width && h_max < 1.2f* rect.size.width)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


void armor::armorPredict(float x, float y){
    rect.center.x += x;
    rect.center.y += y;
}




