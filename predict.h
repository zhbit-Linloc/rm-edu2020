#ifndef PREDICT_H
#define PREDICT_H

#include <opencv2/opencv.hpp>
#include <list>
using namespace cv;

class ZeYuPredict
{
public:
    ZeYuPredict();
    ZeYuPredict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta);
    void ClearFilters(void);
    void setQRT(int Qp, int Qv, int Rp , int dta, float pre_dta);
    void setdelta(float t){
        delta = t;
    }
    float run_position(float gim_angle);
    float run_position(float gimbal_anlge, float v);
    float run_position(float gimbal_anlge, float v, float u);
    float getSpeed()
    {
        return v;
    }
    bool exit_flag;
private:
    Mat Qvar;
    Mat R;      // 单观测变量方差
    Mat Rvar;   // 两个观测变量协方差
    Mat A;
    Mat B;
    Mat H;      // 单变量观测矩阵
    Mat H2;     // 两变量观测矩阵
    float delta;
    float pre_delta;

    //state
    Mat po;         //error covarience
    Mat po_pre;     //error co
    Mat x_pre;    //the last angle
    Mat kg;       //kalman gain
    Mat xkf;      //kalman angle
    float v;
    float last_v;
    float predict;  //predict angle
};



class Kalman1
{
public:
    Kalman1(){
        Q_ = 0.01f;
        R_ = 0.02f;
        t_ = 1.0f;
        x_ = 0.0f;
        p_ = 0.01f;
    }
    Kalman1(float Q, float R, float t, float x0, float p0){
        Q_ = Q;
        R_ = R;
        t_ = t;
        x_ = x0;
        p_ = p0;
    }
    void setParam(int R, int Q, int t){
        if(R<1)
            R=1;
        if(Q<1)
            Q=1;
        if(t<1)
            t=1;
        R_ = static_cast<float>(R*0.01f);
        Q_ = static_cast<float>(Q*0.01f);
        t_ = static_cast<float>(t);
    }
    float run(float data){
        x_pre_ = x_;                                      //x(k|k-1) = AX(k-1|k-1)+BU(k)
        p_pre_ = p_ + Q_;                              //p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);               //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data - x_pre_);          //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                   //p(k|k) = (I-kg(k)H)P(k|k-1)
        return x_;
    }
    float merge_run(float data1, float data2)
    {
        x_pre_ = data1;
        p_pre_ = p_ + Q_;                              //p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);               //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data2 - x_pre_);          //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                   //p(k|k) = (I-kg(k)H)P(k|k-1)
        return x_;
    }
public:
    float R_;
    float Q_;
    float p_pre_;
    float x_pre_;
    float x_;
    float p_;
    float kg_;
    float t_;

};




class Predictor {
public:
    Predictor(int _history_size = 5)
        : history_size(_history_size){}

    bool setRecord(double value, double time);

    // Using history data to predict the query value
    // (fit the data using quadratic curve at min MSE criterion)
    double predict(double time);
    void clear(){
        history_value.clear();
        history_time.clear();
    }

private:
    std::list<double> history_value;
    std::list<double> history_time;
    int history_size;
};

#endif // PREDICT_H
