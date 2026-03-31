//
// Created by admin on 2025/05/21.
//

#include "Car_ratio.h"
#include <cstdio>
#include <ctime>
#include <cmath> // 用于 fmod 函数

#define SECONDS_IN_A_WEEK 604800       // 一周的秒数
#define GPS_EPOCH 315964800            // GPS起始时间 1980-01-06 00:00:00 UTC 的时间戳
#define UTC_OFFSET (8 * 3600)            // 北京时间相对于UTC的偏移秒数

void LQR_car::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                    const Matrix &M, Matrix *ptr_K) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
        Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
        M.rows() != Q.rows() || M.cols() != R.cols()) {
        // LOGE( "LQR solver: one or more matrices have incompatible dimensions.");
        return;
    }
    Matrix AT = A.transpose();
    Matrix BT = B.transpose();
    Matrix MT = M.transpose();
    P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance) {
        Matrix P_next =
                AT * P * A -
                (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        // check the difference between P and P_next
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }
/*     LOGI("Riccati Matrix P=%s", toString(P).data());
     if (num_iteration >= max_num_iteration) {
         LOGI( "LQR solver cannot converge to a solution,"
               "last consecutive result diff is: %lf",diff);
     } else {
         LOGI("LQR solver converged at iteration:%d "
              "max consecutive result diff%lf.: ",num_iteration,diff);
     }*/
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void LQR_car::Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
                    Matrix *ptr_K) {
    Matrix M = Matrix::Zero(Q.rows(), R.cols());
    this->Solve(A, B, Q, R, M, ptr_K);
}

void LQR_car::Update_Q_R_matrix(double q11, double q22, double r00, double r11,int heading){
    double q1,q2,r;
    q1 = q11, q2 = q22, r = r00;
    if(heading == 0){
        q1 = q11;
        // 参数自适应车速 车速过快调节力度需要小
        if(abs(cur_state.v)<0.75 && abs(cur_state.v)>0.2){      // 0.036~2.5km/h
            q2 = 165;
            r = 65;
        }
        if(abs(cur_state.v)<1 && abs(cur_state.v)>0.75){       // 2.5~3.6km/h
            q2 = 135;
            r = 70;
        }
        if(abs(cur_state.v)<1.75 && abs(cur_state.v)>1.2){       // 3.6~6.3km/h
            q2 = 125;
            r = 80;
        }
        if(abs(cur_state.v)< 2 && abs(cur_state.v)>1.75){       // 6.3~7.2km/h
            q2 = 115;
            r = 90;
        }
        if(abs(cur_state.v)< 2.5 && abs(cur_state.v)>2){        // 7.2~9km/h
            q2 = 90;
            r = 115;
        }
        if(abs(cur_state.v)< 3 && abs(cur_state.v)>2.5){       // 9~10.8km/h
            q2 = 85;
            r = 130;
        }
        if(abs(cur_state.v)< 3.5 && abs(cur_state.v)>3){       // 10.8~11.6km/h
            q2 = 85;
            r = 145;
        }
        if(abs(cur_state.v)>3.5){                               // >11.6km/h
            q2 = 80;
            r = 160;
        }
        q22 = q2 + 100 - q22;    // q22 默认值100
        r00 = r + 100 - r00;
    }

    if(cur_state.v < 0){
        if(cur_state.v < -1){
            q11 = 20;   q2 = 110;   r = 70;
        }
        q11 = q11 + 10;    q22 = q22 - 10;    r00 = r00 + 10;
    }
    CarData.car_q1 = q11;
    CarData.car_q2 = q22;
    CarData.car_r = r00;
    LOGI("q1:%f,q2:%f,r:%f",q11,q22,r00);
    Q(0,0)=q11;
    Q(1,1)=q22;
    R(0,0)=r00;
    R(1,1)=r11;
}

void LQR_car::Solve(Matrix *ptr_K) {
    this->Solve(A, B, Q, R, ptr_K);
}

void LQR_car::Update_A_B_matrix(double L11)  {
//void LQR::Update_A_B_matrix(double L11, double L22, double L33)  {
// 倒车时车速符号取反了。(倒车时车速为负值)
    A(0,0)=1.0;
    A(1,0)=cur_state.v*dt;
    A(1,1)=1.0;
    B(0,0)=cur_state.v*dt/L11;
}

double LQR_car::CALC(Eigen::Vector2d *dif_state, Eigen::MatrixXd *ptr_K) {
    this->Solve(A,B,Q,R,ptr_K);
    Matrix U=-(*ptr_K) * (*dif_state);
    Matrix KK=*ptr_K;
    LOGI("KK1=%lf,KK2=%lf",KK(0,0),KK(0,1));
    LOGI("U1=%lf,U2=%lf",U(0,0),U(1,0));
    CarData.car_K0 = KK(0,0);
    CarData.car_K1 = KK(0,1);
    CarData.car_U0 = U(0,0);
    return U(0,0);
}

LQR_car::LQR_car(uint a1, double a2, double a3){
    tolerance=a2;
    max_num_iteration=a1;
    dt=a3;

    A = Matrix::Zero(2, 2);
    B = Matrix::Zero(2, 2);
    Q = Matrix::Zero(2, 2);
    R = Matrix::Zero(2, 2);
    P = Matrix::Zero(2, 2);
}

void LQR_car::update_car_state(double x, double y, double psi, double v) {
    cur_state={x,y,psi,v};
}

void LQR_car::update_imp_state(double x, double y, double psi, double v) {
    imp_state={x,y,psi,v};
}

// 由于不需要闰秒处理，注释掉相关数组和函数
/*int leap_seconds[] = {
    46828800, 78364800, 109900800, 173059200, 252028800,
    315187200, 346723200, 393984000, 425520000, 457056000,
    504489600, 551750400, 599184000, 820108800, 914803200,
    1025136000, 1119744000, 1167264000, -1
};

// 计算闰秒的数量（不再使用，注释掉）
int calculate_leap_seconds(time_t gps_time) {
    int count = 0;
    for (int i = 0; leap_seconds[i] != -1; i++) {
        if (gps_time >= leap_seconds[i]) {
            count++;
        } else {
            break;
        }
    }
    return count;
}*/

// GPS时间转北京时间
void LQR_car::gps_to_beijing_time(unsigned int gps_week, long double gps_seconds, struct tm *beijing_time, long double *milliseconds) {
    if (gps_seconds < 0) {
        LOGI("Error: GPS seconds cannot be negative.\n");
        return;
    }
    // 将秒数除以100（浮点除法）
    gps_seconds = gps_seconds / 100.0 - 0.36;

    // 如果秒数超出一周的范围，仅保留周内秒数
    if (gps_seconds >= SECONDS_IN_A_WEEK) {
        gps_seconds = fmod(gps_seconds, SECONDS_IN_A_WEEK); // 保留周内秒数
        LOGI("Input seconds exceeded one week.\n");
    }

    // 计算从GPS起始时间的总秒数
    time_t gps_time = GPS_EPOCH + gps_week * SECONDS_IN_A_WEEK + (time_t)gps_seconds;

    // 提取毫秒部分
    *milliseconds = gps_seconds - (time_t)gps_seconds;

    // 不进行闰秒调整
    /* int leap_seconds = calculate_leap_seconds(gps_time);
    time_t utc_time = gps_time - leap_seconds; */

    // 直接用gps_time，不调整闰秒
    time_t utc_time = gps_time;

    // 加上北京时间的偏移
    time_t beijing_time_t = utc_time + UTC_OFFSET;

    // 转换为北京时间结构体
    *beijing_time = *gmtime(&beijing_time_t);
}


double LQR_car::deg_trans(double deg)    // 角度转换函数
{
    deg=M_PI*2-deg;             // 正常传入的航向角在0~2PI范围，此处的角度转换有点多余
    while (deg>M_PI*2)
    {
        deg-=M_PI*2;
    }
    while(deg<-M_PI*2)
    {
        deg+=M_PI*2;
    }
    if (deg>=0 && deg<M_PI/2)   // 遇见return函数体返回对应数值，并结束函数
        return M_PI/2+deg;
    if (deg>=M_PI/2 && deg<=M_PI*2)
        return deg-M_PI*3/2;
    return 0;
}