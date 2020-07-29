#include "armorfind.h"
#include "math.h"
#include "base.h"
#include "solveangle.h"

using namespace cv;
using namespace std;

vector<vector<Point>> contours;
vector<RotatedRect> RA;
vector<RotatedRect> R;
vector<double> heights;

void armorFind::Init(Mat inputImg,Mat outputImg){
    this->inputImg = inputImg;
    this->outputImg = outputImg;
    armorFindInit();
}

double armorFind::Pointdis(const Point &p1,const Point &p2){
    return sqrt( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

Point armorFind::PointBetween(const Point &p1,const Point &p2){
    return Point((p1.x + p2.x) / 2,(p1.y + p2.y) / 2);
}

double armorFind::AngerDiff(const double angle1, const double angle2){
    return abs(angle1 - angle2);
}

double armorFind::PointLineX(const Point &p1,const Point &p2){
    return abs(p1.x - p2.x);
}

double armorFind::AngerAve(const double angle1, const double angle2){
    return abs(angle1 + angle2) / 2;
}

double armorFind::HeightAve(const double height1, const double height2){
    return abs(height1 + height2) / 2;
}

void armorFind::SetColor(int color){
    this->color = color;
}

void armorFind::SetImg(Mat inputImg, Mat outputImg){
    this->inputImg = inputImg;
    this->outputImg = outputImg;
}

Mat contourThreadkernel = getStructuringElement(MORPH_ELLIPSE,Size(9,9));
Mat gray;
void armorFind::ContourThread(int thres){
//    Mat thres_whole;  //inputafter;
//    Mat binary;
//    vector<Mat> splited;
//    split(inputImg,splited);
//    cvtColor(inputImg,thres_whole,COLOR_BGR2GRAY);
//    cvtColor(inputImg, gray, COLOR_BGR2GRAY);
//    threshold(thres_whole,thres_whole,100,255,THRESH_BINARY);
//    if(color == 0) {
//        subtract(splited[0],splited[2],binary);// blue
//    }
//    else {
//        subtract(splited[2],splited[0],binary);// red
//    }
//    threshold(binary,binary,thres,255,THRESH_BINARY);
//    dilate(binary,binary,contourThreadkernel);
//    binary = binary & thres_whole;
//    findContours(binary,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
//#ifndef IMSHOW
//    imshow("binary", binary);
//#endif


    // **预处理** -图像进行相应颜色的二值化
//    vector<LED_Stick> LED_Stick_v;  // 声明所有可能的灯条容器
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(inputImg,gray,COLOR_BGR2GRAY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(inputImg, bgr);
    Mat result_img;
    if(color == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    threshold(gray, binary_brightness_img, gray_th_, 255, CV_THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);

    // **提取可能的灯条** -利用灯条（灰度）周围有相应颜色的光圈包围
    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef SHOW_BINARY_IMAGE
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif

    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 15 && length <4000)
                {                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
#ifdef SHOW_LIGHT_CONTOURS
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),1);
                    }
#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef SHOW_LIGHT_PUT_TEXT
                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);
                    }
                }
                break;
            }
        }
    }

}

void armorFind::FindArmor(){
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
//                putText(outputImg, to_string(arm_tmp.rect.size.width/(arm_tmp.rect.size.height+0.0001)), arm_tmp.rect.center , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    if(arm_tmp.get_average_intensity(gray)< 100 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
                    }
                }
            }
        }
    }
}

void armorFind::GetLED(){
//    for(size_t i = 0; i < contours.size(); i++)
//    {

//        double area = contourArea(contours[i]);
//        if (area < 20.0 || 1e5 < area) continue;
//                double length = arcLength(contours[i], true);
//                if (length < 15 || length > 400) continue;

//#ifdef USE_FIT
//                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
//                    if (contours[i].size() < 5) continue;
//                    RotatedRect RRect = fitEllipse(contours[i]);
//#ifdef show_rect_bound
//                    // 旋转矩形提取四个点
//                    Point2f rect_point[4];
//                    RRect.points(rect_point);
//                    for (int i = 0; i < 3 ; i++)
//                    {
//                        line(outputImg, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(0,255,0),2);
//                    }
//#endif
//                    // 角度换算，将拟合椭圆0~360 -> -180~180
//                    if(RRect.angle > 90.0f)
//                        RRect.angle =  RRect.angle - 180.0f;
//#ifdef show_rect_angle
//                    putText(outputImg, to_string(RRect.angle), RRect.center + Point2f(2,2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
//#endif
//#else
//                    RotatedRect RRect = minAreaRect( Mat(contours[i]));
//#ifdef show_rect_angle
//                    putText(img, to_string(int(RRect.angle)), RRect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2);
//#endif
//#ifdef show_rect_bound

//                    Point2f rect_point[4];
//                    RRect.points(rect_point);
//                    for (int i = 0; i < 3 ; i++)
//                    {
//                        line(img, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(255,0,255),2);
//                    }
//#endif
//                    if(RRect.size.height < RRect.size.width)    // convert angle to
//                    {
//                        RRect.angle+= 90;
//                        double tmp = RRect.size.height;
//                        RRect.size.height = RRect.size.width;
//                        RRect.size.width = tmp;
//                    }
//#endif
//                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
//                    {
//                        LED_Stick r(RRect);
//                        LED_Stick_v.push_back(r);
//                    }

//    }

    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
#ifdef SHOW_ARMOR_PUT_TEXT
                putText(img, to_string(arm_tmp.rect.width/(arm_tmp.rect.height+0.0001)), arm_tmp.center + Point_<int>(offset_roi_point) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    // TODO(cz): 推荐使用255值的面积进行判断
                    if(arm_tmp.get_average_intensity(gray)< 50 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
                    }
                }
            }
        }
    }
}

void armorFind::SortArmor(){
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }
}

void armorFind::GetFinalArmor(){
//    float dist=1e8;
//    Armor.rect.center.x = 320; // trackbar get only positive vulue;
//    Armor.rect.center.y = 180;
//    Point2f aim_center = Armor.rect.center;
//    float dx,dy;
//    for (size_t i = 0; i < final_armor_list.size() ; i++ )
//    {
//        dx = pow((final_armor_list.at(i).rect.center.x - aim_center.x), 2.0f);
//        dy = pow((final_armor_list.at(i).rect.center.y - aim_center.y), 2.0f);
//        if( dx + dy < dist) Armor = final_armor_list.at(i);
//#ifndef IMSHOW
//        final_armor_list.at(i).draw_rect(outputImg);
//        final_armor_list.at(i).draw_spot(outputImg);
//#endif
//        Armor.found = true;
//    }

    Point2f offset_roi_point(roi_rect.x, roi_rect.y);
    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_DRAW_RECT
        final_armor_list.at(i).draw_rect2(outputImg, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI_RECTANGLE
    rectangle(img, roi_rect,Scalar(255, 0, 255),1);
#endif


}

void armorFind::GetArmors(void){
    GetLED();
    FindArmor();
    SortArmor();
    GetFinalArmor();
}

int armorFind::ArmorTask(void){
    Rect roi = GetRoi(outputImg);
    if (DetectArmor(outputImg, roi)){
        bool final_armor_type = getTypeResult(is_small_);
        solve_angle_.Generate3DPoints((uint)final_armor_type, Point2f());
        solve_angle_.getAngle(points_2d_, 15,angle_x_,angle_y_,distance_);   // pnp姿态结算
        angle_x_ = kalman.run(angle_x_);
        return 1;
    }else
    {
        angle_x_ = 0;
        angle_y_ = 0;
        distance_ = 0;
        dist_ = 0;
        return 0;
    }
}


bool armorFind::getTypeResult(bool is_small)
{
    if (history_.size() < filter_size_){
        history_.push_back(is_small);
    }
    else {
        history_.push_back(is_small);
        history_.pop_front();
    }

    int vote_cnt[2] = {0};
    for (std::list<bool>::const_iterator it = history_.begin(); it != history_.end(); ++it){
        *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
    }

    if (vote_cnt[0] == vote_cnt[1])
        return is_small;
    return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
}


//void armorFind::SolveAngle(void){
//    vector<Point2f>	imagePoints;
//    float theta_z;
//    float theta_y;
//    float theta_x;
//    float sy;
//    bool singular;

//    Mat objPM;//三维点矩阵
//    vector<Point3f> objectPoints = Generate3DPoints(SMALL_ARMOR);//三维坐标点
//    vector<Point2f> projectedPoints;//三维点投影到二维点的向量用于重画
//    Mat(objectPoints).convertTo(objPM, CV_32F);//把三维点向量变成三维点矩阵
//    double cameraD[3][3] = { { 515.5552 ,0 , 319.1154, },
//                            { 0, 515.9649 ,  182.0495, },
//                            { 0 , 0 , 1.0000 } };//通过matlab标定的单相机内参
//    double distC[5] = { 0.0000, -0.3762, 0.1724 ,  0.0000 , 0.0000 };//通过matlab获取的相机畸变参数
//    Mat cameraMatrix(3, 3, cv::DataType<double>::type, cameraD);
//    Mat distCoeffs(5, 1, cv::DataType<double>::type, distC);

//    Mat rvec(3, 1, cv::DataType<double>::type);
//    Mat tvec(3, 1, cv::DataType<double>::type);

//    if (Armor.found == true) {
//        getTarget2dPoinstion(Armor.rect, imagePoints, Point2f(0, 0));
//        if (imagePoints.size() == 4 && objPM.size > 0)
//        {
//            solvePnP(objPM, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

//#ifdef BULLET
//            tvec.at<double>(2,0)*=scale;
//            double rm[3][3];
//            Mat rotMat(3, 3, CV_64FC1, rm);
//            Rodrigues(rvec, rotMat);
//        //    theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;
//            theta_y = atan2(static_cast<float>(rm[1][0]), static_cast<float>(rm[0][0])) * 57.2958f;//x
//        //    theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;//y
//        //    theta_y = atan2(rm[2][1], rm[2][2]) * 57.2958;//z

//            // 坐标系转换 -摄像头坐标到云台坐标
//            double theta = -atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);
//            double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
//            double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
//            Mat t_camera_ptz(3,1,CV_64FC1,t_data);
//            Mat r_camera_ptz(3,3,CV_64FC1,r_data);
//            Mat position_in_ptz;
//            position_in_ptz = r_camera_ptz * tvec - t_camera_ptz;

//            //计算子弹下坠补偿
//            double bullet_speed = static_cast<double>( ballet_speed);
//            const double *_xyz = (const double *)position_in_ptz.data;
//            double down_t = 0.0;
//            if(bullet_speed > 10e-3)
//                down_t = _xyz[2] /1000.0 / bullet_speed;
//            double offset_gravity = 0.5 * 9.8 * down_t*down_t * 1000;
//                    offset_gravity = 0;
//            // 计算角度
//            double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
//            double alpha = 0.0, thta = 0.0;
//            alpha = asin(static_cast<double>(barrel_ptz_offset_y)/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));

//            if(xyz[1] < 0)
//            {
//                thta = atan(-xyz[1]/xyz[2]);
//                ZiTaiAngle.pitch = static_cast<float>(-(alpha+thta)); //camera coordinate
//            }else if(xyz[1] < static_cast<double>(barrel_ptz_offset_y))
//            {
//                theta = atan(xyz[1]/xyz[2]);
//                ZiTaiAngle.pitch = static_cast<float>(-(alpha - thta));
//            }else
//            {
//                theta = atan(xyz[1]/xyz[2]);
//                ZiTaiAngle.pitch = static_cast<float>((theta-alpha));   // camera coordinate
//            }
//            ZiTaiAngle.yaw = static_cast<float>(atan2(xyz[0],xyz[2]));
//            ZiTaiAngle.yaw = static_cast<float>(ZiTaiAngle.yaw) * 57.2957805f;
//            ZiTaiAngle.pitch = static_cast<float>(ZiTaiAngle.pitch) * 57.2957805f;
//            ZiTaiAngle.dist = static_cast<float>(xyz[2]);
//#else
////            double rm[3][3];
////            cv::Mat rotMat(3, 3, CV_64FC1, rm);//使得rm数组与rotmat矩阵共享数据
////            Rodrigues(rvec, rotMat);
////            sy = (float)sqrt(rm[0][0] * rm[0][0] + rm[1][0] * rm[1][0]);
////            singular = sy < 1e-6;
////            if (!singular){
////                theta_z = (float)atan2(rm[1][0], rm[0][0]) * 57.2958;
////                theta_y = (float)atan2(-rm[2][0], sy) * 57.2958;
////                theta_x = (float)atan2(rm[2][1], rm[2][2]) * 57.2958;
////            }
////            else {
////                theta_z = 0;
////                theta_y = (float)atan2(-rm[2][0], sy) * 57.2958;
////                theta_x = (float)atan2(-rm[1][2], rm[1][1]) * 57.2958;
////            }


//           //平移矩阵
//            double tx = tvec.ptr<double>(0)[0];
//            double ty = tvec.ptr<double>(0)[1];
//            double tz = tvec.ptr<double>(0)[2];
//            ZiTaiAngle.pitch = -3 + getAngle(-ty, tz) ;
//            ZiTaiAngle.yaw = getAngle(-tx, tz) ;
//#endif
////            cout << "rvec: " << endl << rvec << endl;
////            cout << "tvec：" << endl << tvec << endl;
////            cout << "tx:" << tx << endl;
////            cout << "ty:" << ty << endl;
////            cout << "tz:" << tz << endl;
//            cout << "pitch:" << ZiTaiAngle.pitch << endl;
//            cout << "yaw: " << ZiTaiAngle.yaw << endl;
////            cout << "roll: " << ZiTaiAngle.roll << endl;



////            Point centerP;
////            centerP.x = (projectedPoints[0].x + projectedPoints[1].x + projectedPoints[2].x + projectedPoints[3].x) / 4;
////            centerP.y = (projectedPoints[0].y + projectedPoints[1].y + projectedPoints[2].y + projectedPoints[3].y) / 4;
////            cout << "centerP:" << centerP << endl;

//    /*		char theta_z_name[20];
//            char theta_y_name[20];
//            char theta_x_name[20];
//            char x_name[20];
//            char y_name[20];
//            char z_name[20];
//            sprintf_s(z_name, "z%d", int(tz * -1));
//            sprintf_s(y_name, "y%d", int(ty * -1));
//            sprintf_s(x_name, "x%d", int(tx * -1));
//            sprintf_s(theta_z_name, "theta_z%d", int(theta_z));
//            sprintf_s(theta_y_name, "theta_y%d", int(theta_y));
//            sprintf_s(theta_x_name, "theta_x%d", int(theta_x));
//            putText(binary, theta_z_name, Point(200, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
//            putText(binary, theta_y_name, Point(250, 150), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
//            putText(binary, theta_x_name, Point(300, 200), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
//            putText(binary, z_name, Point(50, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
//            putText(binary, y_name, Point(100, 150), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
//            putText(binary, x_name, Point(150, 200), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));*/
//        }

////                cv::circle(binary, Point((R[mark].center.x + RA[mark].center.x) / 2, (R[mark].center.y + RA[mark].center.y) / 2), 15, cv::Scalar(0, 0, 255), 4);

//    }
//}

void armorFind::armorFindInit(void) {
    Armor.found = false;
}



Rect armorFind::GetRoi(const Mat &img)
{
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    if(rect_tmp.x == 0 || rect_tmp.y == 0
            || rect_tmp.width == 0 || rect_tmp.height == 0
            || lost_cnt_ >= 15 || detect_cnt_%100 == 0
        #ifdef FORCE_CHANGE_CAMERA
            || update_cap_cnt < 20
        #endif
            )
    {
        last_target_ = Rect(0,0,img_size.width, img_size.height);
        rect_roi = Rect(0,0,img_size.width, img_size.height);
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt_ < 30)
            scale = 3;
        else if(lost_cnt_ <= 60)
            scale = 4;
        else if(lost_cnt_ <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width)*0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height)*0.5f);

        rect_roi = Rect(x, y, w, h);

        if(makeRectSafe(rect_roi, img_size)== false)
        {
            rect_roi = Rect(0,0,img_size.width, img_size.height);
        }
    }
    return rect_roi;
}


bool armorFind::DetectArmor(Mat &img, Rect roi_rect)
{
    // **预处理** -图像进行相应颜色的二值化
    Mat roi_image = img(roi_rect);
    Point2f offset_roi_point(roi_rect.x, roi_rect.y);
//    cout << "offset_roi_point" << offset_roi_point << endl;
    vector<LED_Stick> LED_Stick_v;  // 声明所有可能的灯条容器
    Mat binary_brightness_img, binary_color_img, gray;
    cvtColor(roi_image,gray,COLOR_BGR2GRAY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    //    dilate(img, img, element);
    vector<cv::Mat> bgr;
    split(roi_image, bgr);
    Mat result_img;
    if(color == 0)
    {
        subtract(bgr[2], bgr[1], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    threshold(gray, binary_brightness_img, gray_th_, 255, CV_THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);

    // **提取可能的灯条** -利用灯条（灰度）周围有相应颜色的光圈包围
    //    printf("bin_th = %d, color_th = %d\r\n", show_bin_th, show_color_th);
#ifdef SHOW_BINARY_IMAGE
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //#pragma omp for
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 15 && length <4000)
                {                    // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
#ifdef SHOW_LIGHT_CONTOURS
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, rect_point[i]+offset_roi_point, rect_point[(i+1)%4]+offset_roi_point, Scalar(255,0,255),1);
                    }
#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef SHOW_LIGHT_PUT_TEXT
                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        LED_Stick r(RRect);
                        LED_Stick_v.push_back(r);
                    }
                }
                break;
            }
        }
    }

    // **寻找可能的装甲板** -遍历每个可能的灯条, 两两灯条拟合成装甲板进行逻辑判断
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        for(size_t j = i + 1; j < LED_Stick_v.size() ; j++)
        {
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(j) );
            if (arm_tmp.error_angle < 8.0f)
            {
#ifdef SHOW_ARMOR_PUT_TEXT
                putText(img, to_string(arm_tmp.rect.width/(arm_tmp.rect.height+0.0001)), arm_tmp.center + Point_<int>(offset_roi_point) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                // TODO(cz): 推荐加入灯条宽度要小于装甲板宽度的条件
                if(arm_tmp.is_suitable_size())
                {
                    // TODO(cz): 推荐使用255值的面积进行判断
                    if(arm_tmp.get_average_intensity(gray)< 50 )
                    {
                        arm_tmp.max_match(LED_Stick_v, i, j);
                    }
                }
            }
        }
    }

    // **分类装甲板** -根据灯条匹配状态得到最终装甲板
    vector<armor> final_armor_list;
    for(size_t i = 0; i < LED_Stick_v.size() ; i++)
    {
        if(LED_Stick_v.at(i).matched)
        {
            LED_Stick_v.at(LED_Stick_v.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp( LED_Stick_v.at(i), LED_Stick_v.at(LED_Stick_v.at(i).match_index));
            final_armor_list.push_back(arm_tmp);
        }
    }

    // **选择装甲板** -根据距离图像中心最短选择
    float dist=1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi_rect.width/2, roi_rect.height/2);
    float dx,dy;
    for (size_t i = 0; i < final_armor_list.size() ; i++ )
    {
#ifdef FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((final_armor_list.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((final_armor_list.at(i).center.y - roi_center.y), 2.0f);
#endif
        if( dx + dy < dist){
            target = final_armor_list.at(i);
            dist = dx + dy;
        }
#ifdef SHOW_DRAW_RECT
        final_armor_list.at(i).draw_rect2(img, offset_roi_point);
#endif
        found_flag = true;
    }
#ifdef SHOW_ROI_RECTANGLE
    rectangle(img, roi_rect,Scalar(255, 0, 255),1);
#endif
    // **计算装甲板四个点顶点** -用于pnp姿态结算
    // TODO(cz): 四个点的不同的bug修复
    RotatedRect target_rect;
    if(found_flag)
    {
#ifdef SHOW_DRAW_SPOT
        target.draw_spot2(img, offset_roi_point + offset_point);
#endif
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if(target.LED_stick[0].rect.center.x > target.LED_stick[1].rect.center.x)
        {
            R = target.LED_stick[0].rect;
            L = target.LED_stick[1].rect;
        }else
        {
            R = target.LED_stick[1].rect;
            L = target.LED_stick[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];

        //show target data
//        putText(img, to_string(target.Led_stick[0].rect.angle), target.center - Point2i(0,20),FONT_HERSHEY_SIMPLEX,2,Scalar(255,255,255));

        // 计算补偿，用于调试调整准心        
        offset_point = Point2f(100, 100) - Point2f(short_offset_x_,short_offset_y_);

        points_2d_.clear();
        vector<Point2f> points_roi_tmp;
        for(int i=0;i<4;i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point + offset_point);
//            putText(img, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
//            circle(img, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
//            circle(img, points_2d_.at(i), 3, Scalar(i*50, i*50, 255), -1);
        }
        // 计算当前装甲板类型，到后面task中还有滤波，可以有误差
        float armor_h = target.rect2.height;
        float armor_w = target.rect2.width;
        if(armor_w / armor_h < 3.0f)
            is_small_ = 1;
        else
            is_small_ = 0;

        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
        rectangle(img, last_target_,Scalar(255,255,255), 1);
        lost_cnt_ = 0;
    }else {
        //计算ROI的相关参数
        lost_cnt_ ++;
    }
    detect_cnt_++;
    return found_flag;
}


