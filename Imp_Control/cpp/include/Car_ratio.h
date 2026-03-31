//
// Created by admin on 2025/05/21.
//

#ifndef IMP_CAR_RATIO_H
#define IMP_CAR_RATIO_H

#include <iostream>
#include <string>
#include "Eigen/Dense"
#include <vector>
#include "android_log.h"
#include <cmath>
#include <cstdio>
#include <ctime>

using Matrix = Eigen::MatrixXd;
typedef struct Car_State{
    double x;
    double y;
    double psi;
    double v;
    double rol;
    double dpsi;
    double err;
}Car_State;

/*typedef struct ref_Point{
    double x;
    double y;
    double psi;
    double v;
}Ref_Point;*/

typedef struct Car_data{
    double car_q1;
    double car_q2;
    double car_r;
    double car_K0;
    double car_K1;
    double car_U0;
}Car_data;

class LQR_car {
public:
    Car_data CarData{};
    LQR_car(uint a1, double a2, double a3);
    void Update_Q_R_matrix(double q11, double q22, double r00, double r11,int heading);
    double CALC(Eigen::Vector2d *dif_state, Eigen::MatrixXd *ptr_K);
    void update_imp_state(double x, double y, double psi, double v);
    void update_car_state(double x,double y,double psi,double v);
    void Update_A_B_matrix(double l1);
    static double deg_trans(double deg);
    static void gps_to_beijing_time(unsigned int gps_week, long double gps_seconds, struct tm *beijing_time, long double *milliseconds);

private:
    double tolerance;
    double dt;
    double gam{};
    int index{};
    uint max_num_iteration;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd M;
    Eigen::MatrixXd P;

    // std::vector<Ref_Point> path{};
    Car_State cur_state{};
    Car_State imp_state{};

    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,
               const Matrix &M,Matrix *ptr_K);
    void Solve(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R,Matrix *ptr_K);
    void Solve(Matrix *ptr_K);

};

#endif //IMP_CAR_RATIO_H
